# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""UnitBehaviors -- per-unit-type combat AI.

Architecture
------------
UnitBehaviors runs each tick and decides what each combatant does:

  - Turrets: stationary, rotate to face nearest hostile in range, fire.
  - Drones: fast strafe runs -- approach hostiles, fire burst, pull back.
  - Rovers: tanky advance -- move toward nearest hostile, engage at range.
  - Hostile kids: approach objective, shoot at defenders in range, dodge.
    Enhanced hostile tactics: flanking, group rush, cover-seeking,
    reconning, suppressing, retreating under fire.

The behavior layer sits ABOVE the waypoint/tick movement system.  It does
NOT override waypoints for friendlies that have been dispatched (those
follow their assigned path while engaging targets of opportunity).  For
idle friendlies without waypoints, behaviors may set temporary engagement
headings.

Hostile kids follow their existing waypoint path (edge -> objective ->
escape) and fire at any defenders in range.  Occasional random dodge
offsets are applied to their position to make them harder to hit.
"""

from __future__ import annotations

import math
import random
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .combat import CombatSystem
    from .target import SimulationTarget

from .degradation import can_fire_degraded

# Drone strafe parameters
_DRONE_RETREAT_RANGE = 15.0  # pull back to this distance after firing

# Flanking parameters
_FLANK_STEP = 1.5       # lateral offset per flank tick (meters)
_FLANK_COOLDOWN = 0.5   # minimum seconds between flank offsets
_FLANK_DETECT_RANGE = 50.0  # max distance to detect a turret for flanking

# Group rush parameters
_GROUP_RUSH_RADIUS = 30.0   # hostiles within this distance form a group
_GROUP_RUSH_MIN_COUNT = 3   # minimum hostiles for group rush
_GROUP_RUSH_SPEED_MULT = 1.2  # 20% speed boost during rush

# Cover-seeking parameters
_COVER_HEALTH_THRESHOLD = 0.5  # seek cover below this health fraction

# Weapon projectile type by asset_type (matches weapons.py loadouts)
_WEAPON_TYPES: dict[str, str | None] = {
    "turret": "nerf_turret_gun",
    "heavy_turret": "nerf_heavy_turret",
    "missile_turret": "nerf_missile_launcher",
    "drone": "nerf_dart_gun",
    "scout_drone": "nerf_scout_gun",
    "rover": "nerf_cannon",
    "tank": "nerf_tank_cannon",
    "apc": "nerf_apc_mg",
    "person": "nerf_pistol",
    "hostile_person": "nerf_pistol",
    "hostile_leader": "nerf_pistol",
    "hostile_vehicle": "nerf_cannon",
    "swarm_drone": "nerf_dart_gun",
    # Mission-type weapon mappings
    "instigator": "thrown_object",
    "rioter": "melee_strike",
    "scout_swarm": None,           # Cannot fire
    "attack_swarm": "nerf_dart_gun",
    "bomber_swarm": None,          # Detonation, not projectile
    # Graphling agents (crystal creatures)
    "graphling": "crystal_shard",
}


def _effective_range_3d(
    attacker_altitude: float,
    target_altitude: float,
    weapon_range: float,
    attacker_type: str,
) -> float:
    """Calculate effective weapon range accounting for 3D altitude.

    AA penalty: if target altitude > 5.0 and attacker is on the ground
    (altitude < 5.0) and attacker is NOT a missile_turret, reduce
    effective weapon_range by 40%.

    Args:
        attacker_altitude: Attacker's altitude in meters.
        target_altitude: Target's altitude in meters.
        weapon_range: Base weapon range.
        attacker_type: Asset type of the attacker.

    Returns:
        Effective weapon range after any altitude penalties.
    """
    if target_altitude > 5.0 and attacker_altitude < 5.0:
        if attacker_type != "missile_turret":
            return weapon_range * 0.6
    return weapon_range


class UnitBehaviors:
    """Per-unit-type combat AI. Called each tick to decide actions."""

    def __init__(self, combat_system: CombatSystem) -> None:
        self._combat = combat_system
        self._last_dodge: dict[str, float] = {}   # target_id -> last dodge time
        self._last_flank: dict[str, float] = {}   # target_id -> last flank time
        self._group_rush_ids: set[str] = set()     # currently rushing hostiles
        self._base_speeds: dict[str, float] = {}   # original speeds before rush boost
        self._obstacles = None                      # obstacle geometry for cover-seeking
        self._spatial_grid = None                   # SpatialGrid for O(1) neighbor queries
        self._terrain_map = None                    # TerrainMap for LOS checks when firing
        self._comms = None                             # UnitComms for inter-unit signals
        self._game_mode_type: str | None = None    # "civil_unrest", "drone_swarm", etc.
        self._de_escalation_timers: dict[str, float] = {}  # rioter_id -> accumulated proximity time
        self._bomber_original_speeds: dict[str, float] = {}  # bomber_id -> speed before dive

        # Sensor awareness tracking
        self._detected_base_speeds: dict[str, float] = {}  # speeds before detection boost
        self._detected_ids: set[str] = set()                # hostiles with active detection boost

        # Upgrade system reference (for EMP stun checks)
        self._upgrade_system = None

        # Pathfinding router callback
        self._router = None  # Callable[[start, end, asset_type, alliance], list]

    def set_router(self, route_fn) -> None:
        """Set the pathfinding router callback.

        route_fn signature: (start, end, unit_type, alliance) -> list[waypoints]
        """
        self._router = route_fn

    def set_obstacles(self, obstacles) -> None:
        """Set obstacle geometry for cover-seeking behavior.

        Args:
            obstacles: An object with a .polygons attribute, where each polygon
                       is a list of (x, y) tuples defining building outlines.
        """
        self._obstacles = obstacles

    def set_spatial_grid(self, grid) -> None:
        """Set the SpatialGrid for O(1) neighbor queries.

        When set, _nearest_in_range and _detect_group_rush use grid.query_radius()
        instead of O(n) / O(n^2) brute-force scans.
        """
        self._spatial_grid = grid

    def set_terrain_map(self, terrain_map) -> None:
        """Set the TerrainMap for LOS checks when firing.

        When set, combat.fire() receives terrain_map and checks
        line_of_sight() before creating a projectile.
        """
        self._terrain_map = terrain_map

    def set_comms(self, comms) -> None:
        """Set the UnitComms for inter-unit signal broadcasting."""
        self._comms = comms

    def set_game_mode_type(self, mode_type: str | None) -> None:
        """Set the current game mode type for behavior dispatch.

        Args:
            mode_type: "civil_unrest", "drone_swarm", or None for standard.
        """
        self._game_mode_type = mode_type

    def set_upgrade_system(self, upgrade_system) -> None:
        """Set the UpgradeSystem for EMP stun checks.

        When set, stunned units (is_emp_stunned) skip all behavior
        (cannot move or fire) for the stun duration.
        """
        self._upgrade_system = upgrade_system

    def _is_emp_stunned(self, target_id: str) -> bool:
        """Check if a unit is EMP stunned via the UpgradeSystem.

        Returns False if no UpgradeSystem is wired (backward compatible).
        """
        if self._upgrade_system is None:
            return False
        return self._upgrade_system.is_emp_stunned(target_id)

    def tick(self, dt: float, targets: dict[str, SimulationTarget],
             vision_state=None) -> None:
        """For each active combatant, run its type-specific behavior."""
        friendlies = {
            k: v for k, v in targets.items()
            if v.alliance == "friendly" and v.status in ("active", "idle", "stationary", "arrived")
            and v.is_combatant
        }
        hostiles = {
            k: v for k, v in targets.items()
            if v.alliance == "hostile" and v.status == "active"
            and v.is_combatant
        }

        for tid, t in friendlies.items():
            # EMP-stunned friendlies cannot act
            if self._is_emp_stunned(tid):
                continue

            if t.asset_type == "missile_turret" and self._game_mode_type == "drone_swarm":
                self._missile_turret_aa_priority(t, hostiles, vision_state=vision_state)
            elif t.asset_type == "turret":
                self._turret_behavior(t, hostiles, vision_state=vision_state)
            elif t.asset_type in ("heavy_turret", "missile_turret"):
                self._turret_behavior(t, hostiles, vision_state=vision_state)
            elif t.asset_type == "drone":
                self._drone_behavior(t, hostiles, vision_state=vision_state)
            elif t.asset_type in ("scout_drone",):
                self._drone_behavior(t, hostiles, vision_state=vision_state)
            elif t.asset_type == "rover":
                self._rover_behavior(t, hostiles, vision_state=vision_state)
            elif t.asset_type in ("tank", "apc"):
                self._rover_behavior(t, hostiles, vision_state=vision_state)
            elif t.asset_type == "graphling":
                self._graphling_behavior(t, hostiles, vision_state=vision_state)

            # Rover de-escalation in civil_unrest mode
            if t.asset_type == "rover" and self._game_mode_type == "civil_unrest":
                self._rover_de_escalation(t, targets, dt=dt)

        # Detect group rushes before individual hostile ticks
        self._detect_group_rush(hostiles)

        for tid, t in hostiles.items():
            # EMP-stunned hostiles cannot act (no movement, no firing)
            if self._is_emp_stunned(tid):
                continue

            # Dispatch based on crowd_role and drone_variant
            if t.crowd_role == "civilian":
                self._civilian_behavior(t, friendlies)
            elif t.crowd_role == "instigator":
                self._instigator_behavior(t, friendlies, dt=dt)
            elif t.crowd_role == "rioter":
                self._rioter_behavior(t, friendlies)
            elif t.drone_variant == "bomber_swarm":
                self._bomber_behavior(t, friendlies, dt=dt)
            elif t.drone_variant == "scout_swarm":
                # Scout drones: emit contact signals but don't fire
                self._scout_swarm_behavior(t, friendlies)
            else:
                self._hostile_kid_behavior(t, friendlies)

    def _turret_behavior(
        self,
        turret: SimulationTarget,
        hostiles: dict[str, SimulationTarget],
        vision_state=None,
    ) -> None:
        """Stationary. Rotate toward nearest hostile in range. Fire when aimed.

        FSM-aware: in cooldown state, waits for weapon to reset.  All other
        combat-capable states (idle, scanning, tracking, engaging) allow firing
        so the turret is always combat-ready when enemies appear.
        """
        fsm = getattr(turret, "fsm_state", None)

        # In cooldown, wait for weapon reset (FSM handles transition back)
        if fsm == "cooldown":
            return

        # Degradation: too damaged to fire
        if not can_fire_degraded(turret):
            return

        # Turrets rotate toward nearest enemy (regardless of vision cone)
        # because they're stationary platforms with sensors — heading drives
        # the vision cone NEXT tick.  Only firing requires vision confirmation.
        any_target = self._nearest_in_range(turret, hostiles)
        if any_target is not None:
            dx = any_target.position[0] - turret.position[0]
            dy = any_target.position[1] - turret.position[1]
            turret.heading = math.degrees(math.atan2(dx, dy))

        # Only fire at targets confirmed visible by vision system
        target = self._nearest_in_range(turret, hostiles, vision_state=vision_state)
        if target is None:
            return

        # Fire with weapon-specific projectile type (terrain LOS check)
        ptype = _WEAPON_TYPES.get(turret.asset_type, "nerf_dart")
        self._combat.fire(turret, target, projectile_type=ptype,
                          terrain_map=self._terrain_map)

    def _drone_behavior(
        self,
        drone: SimulationTarget,
        hostiles: dict[str, SimulationTarget],
        vision_state=None,
    ) -> None:
        """Fast, fragile. Strafe runs -- approach, fire burst, retreat.

        FSM-aware: in scouting state, observes but doesn't fire.  In orbiting,
        maintains distance without engaging.  In rtb, ignores enemies.
        Only fires in engaging state.
        """
        fsm = getattr(drone, "fsm_state", None)

        # In rtb, ignore hostiles entirely (heading home)
        if fsm == "rtb":
            return

        # Degradation: too damaged to fire
        if not can_fire_degraded(drone):
            return

        # Track nearest enemy for heading (vision cone follows heading)
        any_target = self._nearest_in_range(drone, hostiles)
        if any_target is not None:
            dx = any_target.position[0] - drone.position[0]
            dy = any_target.position[1] - drone.position[1]
            drone.heading = math.degrees(math.atan2(dx, dy))

            # Altitude matching: adjust to hostile drone altitude band
            if any_target.altitude > 5.0:
                alt_diff = any_target.altitude - drone.altitude
                # Move altitude toward target's band (within 10m)
                if abs(alt_diff) > 10.0:
                    step = 5.0 * 0.1  # 5m/s * dt approximation
                    if alt_diff > 0:
                        drone.altitude += step
                    else:
                        drone.altitude -= step

        # In scouting or orbiting, observe but don't fire
        if fsm in ("scouting", "orbiting"):
            return

        # Scout drones in civil_unrest: emit contact signal for active instigators
        if self._game_mode_type == "civil_unrest" and drone.asset_type == "scout_drone":
            for h in hostiles.values():
                if getattr(h, "crowd_role", None) == "instigator" and \
                   getattr(h, "instigator_state", None) == "active":
                    if self._comms is not None:
                        from .comms import SIGNAL_CONTACT
                        self._comms.emit_contact(
                            drone.target_id, drone.position, drone.alliance,
                            enemy_pos=h.position,
                        )

        # In civil_unrest mode, drones track but do NOT fire
        if self._game_mode_type == "civil_unrest":
            return

        # Only fire at vision-confirmed targets
        target = self._nearest_in_range(drone, hostiles, vision_state=vision_state)
        if target is None:
            return

        # Ammo check
        if drone.ammo_count == 0:
            return

        # Fire if in range (engaging state, or no FSM set, terrain LOS check)
        ptype = _WEAPON_TYPES.get(drone.asset_type, "nerf_dart")
        self._combat.fire(drone, target, projectile_type=ptype,
                          terrain_map=self._terrain_map)

    def _rover_behavior(
        self,
        rover: SimulationTarget,
        hostiles: dict[str, SimulationTarget],
        vision_state=None,
    ) -> None:
        """Tanky. Move toward nearest hostile, engage at range.

        FSM-aware: in retreating state, does not engage.  In rtb state,
        ignores enemies and heads home.  In patrolling, fires at targets
        of opportunity but doesn't pursue beyond patrol route.
        """
        fsm = getattr(rover, "fsm_state", None)

        # In rtb or retreating, do not engage (heading home or withdrawing)
        if fsm in ("rtb", "retreating"):
            return

        # Degradation: too damaged to fire
        if not can_fire_degraded(rover):
            return

        # Track nearest enemy for heading (vision cone follows heading)
        any_target = self._nearest_in_range(rover, hostiles)
        if any_target is not None:
            dx = any_target.position[0] - rover.position[0]
            dy = any_target.position[1] - rover.position[1]
            rover.heading = math.degrees(math.atan2(dx, dy))

        # Only fire at vision-confirmed targets
        target = self._nearest_in_range(rover, hostiles, vision_state=vision_state)
        if target is None:
            return

        # Fire if in range (terrain LOS check)
        ptype = _WEAPON_TYPES.get(rover.asset_type, "nerf_dart")
        self._combat.fire(rover, target, projectile_type=ptype,
                          terrain_map=self._terrain_map)

    def _graphling_behavior(
        self,
        unit: SimulationTarget,
        hostiles: dict[str, SimulationTarget],
        vision_state=None,
    ) -> None:
        """Graphling combat: pursue nearest hostile, fire when in range."""
        # Always keep graphlings in active status during combat
        if unit.status in ("arrived", "idle"):
            unit.status = "active"

        if not hostiles:
            return

        # Find nearest hostile (any distance)
        best = None
        best_dist = float("inf")
        for enemy in hostiles.values():
            dx = enemy.position[0] - unit.position[0]
            dy = enemy.position[1] - unit.position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < best_dist:
                best_dist = dist
                best = enemy

        if best is None:
            return

        # Face the target
        dx = best.position[0] - unit.position[0]
        dy = best.position[1] - unit.position[1]
        unit.heading = math.degrees(math.atan2(dx, dy))

        if best_dist <= unit.weapon_range:
            # In range: fire
            ptype = _WEAPON_TYPES.get(unit.asset_type, "nerf_dart")
            self._combat.fire(unit, best, projectile_type=ptype,
                              terrain_map=self._terrain_map)
        else:
            # Out of range: pursue
            # Only re-path when: no waypoints, arrived, or target moved >20m
            need_repath = (
                not unit.waypoints
                or unit.status == "arrived"
            )
            if not need_repath:
                # Check if target has moved significantly from our current destination
                last_wp = unit.waypoints[-1] if unit.waypoints else None
                if last_wp is not None:
                    ddx = best.position[0] - last_wp[0]
                    ddy = best.position[1] - last_wp[1]
                    if math.sqrt(ddx * ddx + ddy * ddy) > 20.0:
                        need_repath = True

            if need_repath:
                if self._router is not None:
                    path = self._router(
                        tuple(unit.position[:2]),
                        tuple(best.position[:2]),
                        unit.asset_type,
                        unit.alliance,
                    )
                    if path:
                        unit.waypoints = path
                        unit._waypoint_index = 0
                else:
                    unit.waypoints = [tuple(best.position[:2])]
                    unit._waypoint_index = 0
                unit.status = "active"

    def _hostile_kid_behavior(
        self,
        kid: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
    ) -> None:
        """Hostile AI with enhanced tactics.

        Base behavior: follow waypoints, shoot at defenders in range, dodge.
        Enhanced: flanking turrets, group rush, cover-seeking, reconning,
        suppressing fire, retreating under fire.
        """
        # React to comms signals from allies
        if self._comms is not None:
            self._react_to_signals(kid)

        # FSM state-specific behavior modifiers
        fsm_state = getattr(kid, "fsm_state", None)

        # Morale effects — broken units flee, suppressed units don't fire
        morale = getattr(kid, "morale", 1.0)
        if morale < 0.1:
            # Broken: flee, no combat
            return
        morale_suppressed = morale < 0.3

        # Reconning: reduce speed (cautious scouting) — apply once on entry
        if fsm_state == "reconning":
            if not getattr(kid, '_reconning_speed_set', False):
                kid._reconning_base_speed = kid.speed
                kid.speed = kid.speed * 0.6
                kid._reconning_speed_set = True
        elif getattr(kid, '_reconning_speed_set', False):
            kid.speed = kid._reconning_base_speed
            kid._reconning_speed_set = False

        # Suppressing: increase fire rate (reduced cooldown) — apply once on entry
        if fsm_state == "suppressing":
            if not getattr(kid, '_suppressing_cooldown_set', False):
                kid._suppressing_base_cooldown = kid.weapon_cooldown
                kid.weapon_cooldown = kid.weapon_cooldown * 0.5
                kid._suppressing_cooldown_set = True
        elif getattr(kid, '_suppressing_cooldown_set', False):
            kid.weapon_cooldown = kid._suppressing_base_cooldown
            kid._suppressing_cooldown_set = False

        # Retreating under fire: move toward cover with zigzag
        if fsm_state == "retreating_under_fire":
            self._retreating_under_fire_behavior(kid, friendlies)

        # Emit retreat signal when fleeing
        if fsm_state == "fleeing" and self._comms is not None:
            self._comms.emit_retreat(kid.target_id, kid.position, kid.alliance)

        # Fire at nearest defender in range (skip if morale-suppressed or too degraded)
        if not morale_suppressed and can_fire_degraded(kid):
            target = self._nearest_in_range(kid, friendlies)
            if target is not None:
                # Emit contact signal for allies
                if self._comms is not None:
                    self._comms.emit_contact(
                        kid.target_id, kid.position, kid.alliance,
                        enemy_pos=target.position,
                    )
                ptype = _WEAPON_TYPES.get(kid.asset_type, "nerf_pistol")
                self._combat.fire(kid, target, projectile_type=ptype,
                                  terrain_map=self._terrain_map)

        # Cover-seeking: damaged hostile moves toward nearest building edge
        # When seeking cover, skip flanking and dodge (cover takes priority)
        seeking_cover = self._should_seek_cover(kid)
        if seeking_cover:
            self._seek_cover(kid)
            return

        # Flanking: lateral offset when facing stationary turret
        self._try_flank(kid, friendlies)

        # Occasional dodge (random perpendicular offset every 2-4 seconds)
        # Reduced during group rush
        now = time.time()
        last_dodge = self._last_dodge.get(kid.target_id, 0.0)
        dodge_interval = random.uniform(2.0, 4.0)
        if kid.target_id in self._group_rush_ids:
            dodge_interval *= 3.0  # Dodge less during rush
        if now - last_dodge > dodge_interval:
            self._last_dodge[kid.target_id] = now
            # Small perpendicular offset
            offset = random.uniform(-1.5, 1.5)
            heading_rad = math.radians(kid.heading)
            # Perpendicular to heading
            kid.position = (
                kid.position[0] + math.cos(heading_rad) * offset,
                kid.position[1] - math.sin(heading_rad) * offset,
            )

    # -- Comms signal reaction ---------------------------------------------------

    def _react_to_signals(self, kid: SimulationTarget) -> None:
        """React to comms signals from nearby allies.

        - distress: insert sender position as priority waypoint (converge)
        - contact: insert enemy position as priority waypoint (hunt)
        """
        if self._comms is None:
            return

        # Only react for advancing/reconning hostiles (not fleeing/engaging)
        fsm = getattr(kid, "fsm_state", None)
        if fsm in ("fleeing", "spawning", "engaging", "suppressing"):
            return

        signals = self._comms.get_signals_for_unit(kid)
        if not signals:
            return

        # React to highest priority signal
        # Priority: distress > contact > retreat
        distress = [s for s in signals if s.signal_type == "distress"]
        contact = [s for s in signals if s.signal_type == "contact"]

        target_pos = None
        if distress:
            # Converge on the nearest distress call
            nearest = min(distress, key=lambda s: math.hypot(
                s.position[0] - kid.position[0],
                s.position[1] - kid.position[1],
            ))
            target_pos = nearest.position
        elif contact:
            # Move toward reported enemy position
            for sig in contact:
                if sig.target_position is not None:
                    target_pos = sig.target_position
                    break

        if target_pos is not None:
            # Route through pathfinder then insert as priority waypoints
            if self._router is not None:
                try:
                    routed = self._router(kid.position, target_pos,
                                          kid.asset_type, kid.alliance)
                    if routed:
                        if kid.waypoints:
                            kid.waypoints = list(routed) + kid.waypoints
                        else:
                            kid.waypoints = list(routed)
                        kid._waypoint_index = 0
                        return
                except Exception:
                    pass
            # Fallback: insert direct waypoint
            if kid.waypoints:
                kid.waypoints.insert(0, target_pos)
                kid._waypoint_index = 0
            else:
                kid.waypoints = [target_pos]
                kid._waypoint_index = 0

    # -- Flanking ---------------------------------------------------------------

    def _try_flank(
        self,
        hostile: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
    ) -> bool:
        """Apply lateral flanking offset if facing a stationary turret.

        Returns True if flank was applied.
        """
        now = time.time()
        last_flank = self._last_flank.get(hostile.target_id, 0.0)
        if now - last_flank < _FLANK_COOLDOWN:
            return False

        # Find nearest stationary turret within flanking detection range
        turret = self._nearest_stationary_in_range(hostile, friendlies, _FLANK_DETECT_RANGE)
        if turret is None:
            return False

        # Apply lateral offset perpendicular to the approach vector
        dx = turret.position[0] - hostile.position[0]
        dy = turret.position[1] - hostile.position[1]
        dist = math.hypot(dx, dy)
        if dist < 0.1:
            return False

        # Perpendicular direction (choose a consistent side based on target_id hash)
        perp_sign = 1.0 if hash(hostile.target_id) % 2 == 0 else -1.0
        perp_x = -dy / dist * perp_sign
        perp_y = dx / dist * perp_sign

        # Detected hostiles use wider flank angle (double step)
        step = _FLANK_STEP
        if getattr(hostile, "detected", False):
            step *= 2.0

        hostile.position = (
            hostile.position[0] + perp_x * step,
            hostile.position[1] + perp_y * step,
        )
        self._last_flank[hostile.target_id] = now
        return True

    def _nearest_stationary_in_range(
        self,
        unit: SimulationTarget,
        enemies: dict[str, SimulationTarget],
        max_range: float,
    ) -> SimulationTarget | None:
        """Find the nearest stationary (speed==0) enemy within max_range.

        Uses SpatialGrid when available for O(k) lookup instead of O(n).
        """
        best = None
        best_dist = float("inf")

        if self._spatial_grid is not None:
            candidates = self._spatial_grid.query_radius(unit.position, max_range)
            for candidate in candidates:
                if candidate.target_id not in enemies:
                    continue
                if candidate.speed > 0:
                    continue
                dx = candidate.position[0] - unit.position[0]
                dy = candidate.position[1] - unit.position[1]
                dist = dx * dx + dy * dy
                if dist < best_dist:
                    best_dist = dist
                    best = candidate
        else:
            mr2 = max_range * max_range
            for enemy in enemies.values():
                if enemy.speed > 0:
                    continue
                dx = enemy.position[0] - unit.position[0]
                dy = enemy.position[1] - unit.position[1]
                dist = dx * dx + dy * dy
                if dist <= mr2 and dist < best_dist:
                    best_dist = dist
                    best = enemy
        return best

    # -- Group rush -------------------------------------------------------------

    def _detect_group_rush(self, hostiles: dict[str, SimulationTarget]) -> None:
        """Detect groups of 3+ hostiles within GROUP_RUSH_RADIUS and boost them.

        Uses SpatialGrid when available for O(n*k) instead of O(n^2).
        """
        new_rush_ids: set[str] = set()

        if self._spatial_grid is not None:
            # Grid-accelerated: query radius around each hostile
            for h in hostiles.values():
                nearby = self._spatial_grid.query_radius(h.position, _GROUP_RUSH_RADIUS)
                # Count only hostiles (exclude self and non-hostiles)
                nearby_count = sum(
                    1 for n in nearby
                    if n.target_id != h.target_id and n.target_id in hostiles
                )
                if nearby_count >= _GROUP_RUSH_MIN_COUNT - 1:
                    new_rush_ids.add(h.target_id)
        else:
            # Brute-force fallback
            hostile_list = list(hostiles.values())
            r2 = _GROUP_RUSH_RADIUS * _GROUP_RUSH_RADIUS
            for i, h in enumerate(hostile_list):
                nearby_count = 0
                for j, other in enumerate(hostile_list):
                    if i == j:
                        continue
                    dx = h.position[0] - other.position[0]
                    dy = h.position[1] - other.position[1]
                    if dx * dx + dy * dy <= r2:
                        nearby_count += 1
                if nearby_count >= _GROUP_RUSH_MIN_COUNT - 1:
                    new_rush_ids.add(h.target_id)

        # Apply speed boost to newly rushing hostiles
        for tid in new_rush_ids:
            if tid not in self._group_rush_ids:
                h = hostiles.get(tid)
                if h is not None:
                    self._base_speeds[tid] = h.speed
                    h.speed = h.speed * _GROUP_RUSH_SPEED_MULT

        # Remove boost from hostiles no longer rushing
        for tid in self._group_rush_ids - new_rush_ids:
            if tid in self._base_speeds:
                h = hostiles.get(tid)
                if h is not None:
                    h.speed = self._base_speeds.pop(tid)

        self._group_rush_ids = new_rush_ids

    # -- Cover-seeking ----------------------------------------------------------

    def _should_seek_cover(self, hostile: SimulationTarget) -> bool:
        """Check if a hostile should seek cover (damaged + obstacles available)."""
        if self._obstacles is None:
            return False
        max_hp = getattr(hostile, "max_health", 100.0)
        if max_hp <= 0:
            return False
        health_pct = hostile.health / max_hp
        return health_pct < _COVER_HEALTH_THRESHOLD

    def _seek_cover(self, hostile: SimulationTarget) -> None:
        """Move hostile toward the nearest building polygon edge."""
        if self._obstacles is None:
            return
        polygons = getattr(self._obstacles, "polygons", [])
        if not polygons:
            return

        # Find the nearest point on any polygon edge
        best_point = None
        best_dist = float("inf")

        for poly in polygons:
            n = len(poly)
            for i in range(n):
                p1 = poly[i]
                p2 = poly[(i + 1) % n]
                cp = self._closest_point_on_segment(hostile.position, p1, p2)
                dx = cp[0] - hostile.position[0]
                dy = cp[1] - hostile.position[1]
                dist = math.hypot(dx, dy)
                if dist < best_dist:
                    best_dist = dist
                    best_point = cp

        if best_point is not None and best_dist > 0.1:
            # Move toward the cover point (fraction of the distance per tick)
            dx = best_point[0] - hostile.position[0]
            dy = best_point[1] - hostile.position[1]
            step = min(hostile.speed * 0.1, best_dist)  # move speed * dt toward cover
            hostile.position = (
                hostile.position[0] + (dx / best_dist) * step,
                hostile.position[1] + (dy / best_dist) * step,
            )

    @staticmethod
    def _closest_point_on_segment(
        point: tuple[float, float],
        seg_a: tuple[float, float],
        seg_b: tuple[float, float],
    ) -> tuple[float, float]:
        """Return the closest point on line segment (seg_a, seg_b) to point."""
        ax, ay = seg_a
        bx, by = seg_b
        px, py = point

        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay

        ab_sq = abx * abx + aby * aby
        if ab_sq < 1e-12:
            return seg_a  # Degenerate segment

        t = (apx * abx + apy * aby) / ab_sq
        t = max(0.0, min(1.0, t))
        return (ax + t * abx, ay + t * aby)

    # -- Retreating under fire --------------------------------------------------

    def _retreating_under_fire_behavior(
        self,
        hostile: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
    ) -> None:
        """Move toward cover with zigzag while firing back.

        Different from straight-line flee: uses lateral zigzag offsets
        and moves toward nearest cover, not away from enemies.
        """
        # Move toward cover if available
        if self._obstacles is not None:
            self._seek_cover(hostile)

        # Zigzag: apply lateral offset each tick
        heading_rad = math.radians(hostile.heading)
        zigzag_offset = random.uniform(-1.0, 1.0)
        hostile.position = (
            hostile.position[0] + math.cos(heading_rad) * zigzag_offset,
            hostile.position[1] - math.sin(heading_rad) * zigzag_offset,
        )

    # -- Sensor awareness -------------------------------------------------------

    def apply_sensor_awareness(self, hostile: SimulationTarget) -> None:
        """Apply sensor detection effects to a hostile.

        When a hostile is detected by a sensor, it speeds up by 20% (urgency)
        and prefers wider flanking angles.
        """
        if not getattr(hostile, "detected", False):
            return

        # Only apply once per detection
        if hostile.target_id in self._detected_ids:
            return

        self._detected_ids.add(hostile.target_id)
        self._detected_base_speeds[hostile.target_id] = hostile.speed
        hostile.speed = hostile.speed * 1.2

    def remove_sensor_awareness(self, hostile: SimulationTarget) -> None:
        """Remove sensor detection effects from a hostile.

        Restores original speed when detection expires.
        """
        if hostile.target_id in self._detected_base_speeds:
            hostile.speed = self._detected_base_speeds.pop(hostile.target_id)
        self._detected_ids.discard(hostile.target_id)

    # -- Helpers ----------------------------------------------------------------

    def _nearest_in_range(
        self,
        unit: SimulationTarget,
        enemies: dict[str, SimulationTarget],
        vision_state=None,
    ) -> SimulationTarget | None:
        """Find the nearest enemy within weapon_range.

        Uses SpatialGrid when available for O(k) lookup instead of O(n).
        When vision_state is provided, pre-filters to only targets visible
        to this unit. When None (backward compat), all enemies in range
        are valid.
        """
        # Pre-filter to visible targets when vision is available
        if vision_state is not None:
            visible_targets = set(vision_state.can_see.get(unit.target_id, []))
            enemies = {k: v for k, v in enemies.items() if k in visible_targets}

        best: SimulationTarget | None = None
        best_dist = float("inf")

        if self._spatial_grid is not None:
            # Grid-accelerated: only check targets in weapon_range radius
            candidates = self._spatial_grid.query_radius(unit.position, unit.weapon_range)
            for candidate in candidates:
                if candidate.target_id not in enemies:
                    continue
                dx = candidate.position[0] - unit.position[0]
                dy = candidate.position[1] - unit.position[1]
                dist = dx * dx + dy * dy
                if dist < best_dist:
                    best_dist = dist
                    best = candidate
        else:
            # Brute-force fallback
            wr2 = unit.weapon_range * unit.weapon_range
            for enemy in enemies.values():
                dx = enemy.position[0] - unit.position[0]
                dy = enemy.position[1] - unit.position[1]
                dist = dx * dx + dy * dy
                if dist <= wr2 and dist < best_dist:
                    best_dist = dist
                    best = enemy
        return best

    # -- Mission-type behaviors ---------------------------------------------------

    def _instigator_behavior(
        self,
        instigator: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
        dt: float = 0.1,
    ) -> None:
        """Instigator activation cycle: hidden(8s)->activating(2s)->active(5s)->hidden.

        During 'active': fires thrown_object projectiles at nearest friendly.
        During 'hidden' and 'activating': acts like civilian, no combat.
        """
        state = instigator.instigator_state
        instigator.instigator_timer += dt

        if state == "hidden":
            if instigator.instigator_timer >= 8.0:
                instigator.instigator_state = "activating"
                instigator.instigator_timer = 0.0
            # Act like civilian -- no combat
            return

        elif state == "activating":
            if instigator.instigator_timer >= 2.0:
                instigator.instigator_state = "active"
                instigator.instigator_timer = 0.0
            # Still passive during activation
            return

        elif state == "active":
            if instigator.instigator_timer >= 5.0:
                instigator.instigator_state = "hidden"
                instigator.instigator_timer = 0.0
                return

            # During active: fire thrown_object at nearest friendly (range 15m, cooldown 3s)
            if instigator.ammo_count == 0:
                return
            target = self._nearest_in_range(instigator, friendlies)
            if target is not None:
                ptype = _WEAPON_TYPES.get("instigator", "thrown_object")
                if ptype is not None:
                    self._combat.fire(instigator, target, projectile_type=ptype,
                                      terrain_map=self._terrain_map)

    def _bomber_behavior(
        self,
        bomber: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
        dt: float = 0.1,
    ) -> None:
        """Bomber drone: dives and detonates on target.

        Does NOT fire projectiles. When within 40m: enters diving state.
        Diving: altitude decreases 10m/s, horizontal speed halved.
        Detonation: when altitude <= 0 or distance < 3m.
        If eliminated during dive: no detonation (harmless crash).
        """
        # Dead bombers don't detonate
        if bomber.status in ("eliminated", "destroyed", "neutralized"):
            return

        state = bomber.instigator_state  # repurposed: "approaching" / "diving"

        # Find nearest friendly target
        nearest = None
        nearest_dist = float("inf")
        for f in friendlies.values():
            dx = f.position[0] - bomber.position[0]
            dy = f.position[1] - bomber.position[1]
            dist = math.hypot(dx, dy)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest = f

        if nearest is None:
            return

        if state != "diving" and nearest_dist <= 40.0:
            # Enter diving state
            bomber.instigator_state = "diving"
            if bomber.target_id not in self._bomber_original_speeds:
                self._bomber_original_speeds[bomber.target_id] = bomber.speed
            bomber.speed = bomber.speed / 2.0

        if bomber.instigator_state == "diving":
            # Altitude decreases at 10m/s
            bomber.altitude -= 10.0 * dt

            # Check detonation: altitude <= 0 or distance < 3m
            if bomber.altitude <= 0 or nearest_dist < 3.0:
                # Bomber is still alive at detonation time
                if bomber.health > 0:
                    # Build targets dict for detonation
                    all_targets = dict(friendlies)
                    all_targets[bomber.target_id] = bomber
                    self._combat.detonate_bomber(bomber, all_targets, radius=5.0)

    def _rover_de_escalation(
        self,
        rover: SimulationTarget,
        targets: dict[str, SimulationTarget],
        dt: float = 0.1,
        rover_fired: bool = False,
    ) -> None:
        """De-escalation: rover within 15m of rioter for 3s converts to civilian.

        Only in civil_unrest mode. If rover fires during de-escalation:
        timer resets, nearby civilians have 30% conversion-to-rioter chance.
        """
        if self._game_mode_type != "civil_unrest":
            return

        # If rover fired, reset all timers and convert nearby civilians
        if rover_fired:
            self._de_escalation_timers.clear()
            # 30% chance to convert nearby civilians to rioters
            for t in targets.values():
                if t.crowd_role == "civilian" and t.alliance == "hostile":
                    dx = t.position[0] - rover.position[0]
                    dy = t.position[1] - rover.position[1]
                    if math.hypot(dx, dy) <= 15.0:
                        if random.random() < 0.3:
                            t.crowd_role = "rioter"
                            t.is_combatant = True
            return

        # Check rioters within 15m and accumulate proximity time
        for tid, t in targets.items():
            if t.crowd_role != "rioter":
                continue
            dx = t.position[0] - rover.position[0]
            dy = t.position[1] - rover.position[1]
            dist = math.hypot(dx, dy)

            if dist <= 15.0:
                timer = self._de_escalation_timers.get(tid, 0.0)
                timer += dt
                self._de_escalation_timers[tid] = timer

                if timer >= 3.0:
                    # Convert rioter to civilian
                    t.crowd_role = "civilian"
                    t.is_combatant = False
                    self._de_escalation_timers.pop(tid, None)
                    # Award de-escalation points via event bus
                    self._combat._event_bus.publish("de_escalation", {
                        "rover_id": rover.target_id,
                        "rioter_id": tid,
                        "points": 200,
                    })
            else:
                # Out of range: reset timer
                self._de_escalation_timers.pop(tid, None)

    def _civilian_behavior(
        self,
        civilian: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
    ) -> None:
        """Civilian: random wandering within crowd zone, no combat.

        Does NOT fire, does NOT engage.
        """
        # Civilians just follow their waypoints (handled by tick()).
        # No combat actions.
        pass

    def _rioter_behavior(
        self,
        rioter: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
    ) -> None:
        """Rioter: melee-range only attacks using melee_strike weapon type.

        Rioters can only attack targets within 3m range.
        """
        if not can_fire_degraded(rioter):
            return

        if rioter.ammo_count == 0:
            return

        target = self._nearest_in_range(rioter, friendlies)
        if target is not None:
            ptype = _WEAPON_TYPES.get("rioter", "melee_strike")
            if ptype is not None:
                self._combat.fire(rioter, target, projectile_type=ptype,
                                  terrain_map=self._terrain_map)

    def _scout_swarm_behavior(
        self,
        scout: SimulationTarget,
        friendlies: dict[str, SimulationTarget],
    ) -> None:
        """Scout swarm drone: recon only, emits contact signals but no weapons.

        When a friendly is within detection range, emits SIGNAL_CONTACT
        for attack drones to converge on.
        """
        if self._comms is None:
            return

        # Find nearest friendly in detection range (use a generous scan radius)
        scan_range = 40.0  # Scout detection range
        for f in friendlies.values():
            dx = f.position[0] - scout.position[0]
            dy = f.position[1] - scout.position[1]
            dist = math.hypot(dx, dy)
            if dist <= scan_range:
                from .comms import SIGNAL_CONTACT
                self._comms.emit_contact(
                    scout.target_id, scout.position, scout.alliance,
                    enemy_pos=f.position,
                )
                break  # One signal per tick

    def _missile_turret_aa_priority(
        self,
        turret: SimulationTarget,
        hostiles: dict[str, SimulationTarget],
        vision_state=None,
    ) -> None:
        """Missile turret with AA priority: targets aerial units first.

        In drone_swarm mode, filter hostiles to aerial targets first.
        If no aerial targets, fall back to ground targets.
        """
        # Filter to aerial targets (altitude > 5.0)
        aerial = {k: v for k, v in hostiles.items()
                  if getattr(v, "altitude", 0.0) > 5.0}

        if aerial:
            self._turret_behavior(turret, aerial, vision_state=vision_state)
        else:
            self._turret_behavior(turret, hostiles, vision_state=vision_state)

    def remove_unit(self, target_id: str) -> None:
        """Remove all per-unit state for a single unit."""
        self._last_dodge.pop(target_id, None)
        self._last_flank.pop(target_id, None)
        self._base_speeds.pop(target_id, None)
        self._detected_base_speeds.pop(target_id, None)
        self._bomber_original_speeds.pop(target_id, None)

    def clear_dodge_state(self) -> None:
        """Reset all dodge timers and tactical state (for game reset)."""
        self._last_dodge.clear()
        self._last_flank.clear()
        self._group_rush_ids.clear()
        self._base_speeds.clear()
        self._detected_base_speeds.clear()
        self._detected_ids.clear()
        self._de_escalation_timers.clear()
        self._bomber_original_speeds.clear()
