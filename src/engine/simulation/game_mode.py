# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""GameMode — state machine, wave controller, and scoring.

Architecture
------------
GameMode manages the flow of a Nerf war session through a linear state
machine:

  setup -> countdown (5s) -> active -> wave_complete -> active -> ... -> victory | defeat

The wave controller defines 10 waves of increasing difficulty.  Each
wave specifies a hostile count, speed multiplier, and health multiplier.
Hostiles are spawned in staggered batches (not all at once) via the
SimulationEngine's ``spawn_hostile()`` method.

Victory is achieved by surviving all 10 waves (all hostiles in the final
wave eliminated).  Defeat occurs when all friendly combatants are
eliminated.

Scoring:
  - 100 points per hostile eliminated
  - 50 point time bonus per wave (decreasing by 5 per 10s elapsed)
  - Wave completion bonuses: wave_number * 200

Events published on EventBus for frontend and announcer:
  - ``game_state_change``: any state transition
  - ``wave_start``: new wave begins
  - ``wave_complete``: wave cleared
  - ``game_over``: victory or defeat
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from engine.comms.event_bus import EventBus
    from .combat import CombatSystem
    from .engine import SimulationEngine
    from .target import SimulationTarget


@dataclass
class WaveConfig:
    """Configuration for a single wave of hostiles."""

    name: str
    count: int
    speed_mult: float
    health_mult: float
    # Mixed-type composition: list of (asset_type, count) tuples.
    # When set, _spawn_wave_hostiles uses these instead of spawning all "person".
    # Individual counts should sum to self.count.
    composition: list[tuple[str, int]] | None = None
    # Spawn direction: random, north, south, east, west, pincer, surround
    spawn_direction: str = "random"
    # Extended fields for infinite mode
    has_elites: bool = False
    elite_count: int = 0
    elite_health_mult: float = 1.0
    has_boss: bool = False
    boss_health_mult: float = 1.0
    score_mult: float = 1.0


# 10 waves of increasing difficulty with mixed unit compositions.
# Early waves are all foot soldiers; later waves introduce vehicles, leaders,
# and swarm drones for tactical variety.
WAVE_CONFIGS: list[WaveConfig] = [
    # Waves 1-5: random directions (early game, building tension)
    WaveConfig("Scout Party",    count=3,  speed_mult=0.8, health_mult=0.7),
    WaveConfig("Raiding Party",  count=5,  speed_mult=1.0, health_mult=1.0),
    WaveConfig("Assault Squad",  count=7,  speed_mult=1.0, health_mult=1.2,
               composition=[("person", 5), ("hostile_vehicle", 2)]),
    WaveConfig("Heavy Assault",  count=8,  speed_mult=1.1, health_mult=1.5,
               composition=[("person", 6), ("hostile_vehicle", 2)]),
    WaveConfig("Blitz Attack",   count=10, speed_mult=1.3, health_mult=1.2,
               composition=[("person", 8), ("hostile_vehicle", 2)]),
    # Waves 6-10: tactical directions (flanking, pincer, surround)
    WaveConfig("Armored Push",   count=8,  speed_mult=0.9, health_mult=2.0,
               composition=[("person", 4), ("hostile_vehicle", 2), ("hostile_leader", 2)],
               spawn_direction="east"),
    WaveConfig("Swarm",          count=15, speed_mult=1.4, health_mult=0.8,
               composition=[("person", 12), ("swarm_drone", 3)],
               spawn_direction="pincer"),
    WaveConfig("Elite Strike",   count=6,  speed_mult=1.2, health_mult=2.5,
               composition=[("hostile_leader", 3), ("hostile_vehicle", 3)],
               spawn_direction="west"),
    WaveConfig("Full Invasion",  count=20, speed_mult=1.3, health_mult=1.5,
               composition=[("person", 12), ("hostile_vehicle", 4), ("hostile_leader", 4)],
               spawn_direction="pincer"),
    WaveConfig("FINAL STAND",    count=25, speed_mult=1.5, health_mult=2.0,
               composition=[("person", 15), ("hostile_vehicle", 5), ("hostile_leader", 3), ("swarm_drone", 2)],
               spawn_direction="surround"),
]

# Time between staggered spawns within a wave
_SPAWN_STAGGER = 0.5  # seconds

# Delay before auto-advancing to next wave after wave_complete
_WAVE_ADVANCE_DELAY = 5.0  # seconds

# Countdown duration before wave 1
_COUNTDOWN_DURATION = 5.0  # seconds


class GameMode:
    """Game state machine + wave controller + scoring."""

    STATES = ("setup", "countdown", "active", "wave_complete", "victory", "defeat")

    def __init__(
        self,
        event_bus: EventBus,
        engine: SimulationEngine,
        combat_system: CombatSystem,
        infinite: bool = False,
    ) -> None:
        self._event_bus = event_bus
        self._engine = engine
        self._combat = combat_system
        self.infinite: bool = infinite

        # Adaptive difficulty scaler (always available)
        from .difficulty import DifficultyScaler
        self.difficulty: DifficultyScaler = DifficultyScaler()

        # Infinite wave generator (created once, used when wave > 10)
        self._infinite_wave_mode: InfiniteWaveMode = InfiniteWaveMode(
            difficulty=self.difficulty,
        )

        self.state: str = "setup"
        self.wave: int = 0
        self.score: int = 0
        self.total_eliminations: int = 0
        self.wave_eliminations: int = 0
        self._countdown_remaining: float = _COUNTDOWN_DURATION
        self._wave_start_time: float = 0.0
        self._game_start_time: float = 0.0
        self._wave_complete_time: float = 0.0
        self._wave_hostile_ids: set[str] = set()
        self._spawn_thread: threading.Thread | None = None

        # Scenario support (optional — None means use default WAVE_CONFIGS)
        self._scenario: object | None = None
        self._scenario_waves: list | None = None

        # Mission-type fields (civil unrest, drone swarm)
        self.game_mode_type: str = "battle"
        self.de_escalation_score: int = 0
        self.infrastructure_health: float = 0.0
        self.infrastructure_max: float = 1000.0
        self.civilian_harm_count: int = 0
        self.civilian_harm_limit: int = 5

    # -- Public interface -------------------------------------------------------

    def begin_war(self) -> None:
        """Transition from setup to countdown. Starts the war."""
        if self.state != "setup":
            return
        # Reset all friendly units to full readiness
        for t in self._engine.get_targets():
            if t.alliance == "friendly" and t.is_combatant:
                t.battery = 1.0
                t.health = t.max_health
                if t.status in ("low_battery", "idle", "stationary"):
                    t.status = "active"
        self.state = "countdown"
        self._countdown_remaining = _COUNTDOWN_DURATION
        self._game_start_time = time.time()
        self.wave = 1
        self.score = 0
        self.total_eliminations = 0
        self.wave_eliminations = 0
        self._combat.reset_streaks()
        self._publish_state_change()

    def tick(self, dt: float) -> None:
        """Called every engine tick. Manages state transitions."""
        if self.state == "countdown":
            self._tick_countdown(dt)
        elif self.state == "active":
            self._tick_active(dt)
        elif self.state == "wave_complete":
            self._tick_wave_complete(dt)

    def reset(self) -> None:
        """Reset to setup state. Clear game-mode hostiles and scenario data."""
        self.state = "setup"
        self.wave = 0
        self.score = 0
        self.total_eliminations = 0
        self.wave_eliminations = 0
        self._countdown_remaining = _COUNTDOWN_DURATION
        self._wave_hostile_ids.clear()
        self._combat.reset_streaks()
        self._combat.clear()
        # Clear scenario data so next begin_war() uses default WAVE_CONFIGS
        # unless a new scenario is loaded
        self._scenario_waves = None
        self._scenario = None
        # Reset game mode type and mode-specific fields
        self.game_mode_type = "battle"
        self.de_escalation_score = 0
        self.infrastructure_health = 0.0
        self.civilian_harm_count = 0
        self.difficulty.reset()
        self._publish_state_change()

    def on_target_eliminated(self, target_id: str) -> None:
        """Notify the game mode that a target was eliminated.

        Called by the engine/combat integration to update elimination counts
        and scoring.  Any hostile elimination counts toward score; wave
        hostiles also count toward wave completion.
        """
        if self.state != "active":
            return
        # All hostile eliminations score points
        self.total_eliminations += 1
        self.score += 100  # points per elimination
        # Wave hostiles also count toward wave completion
        if target_id in self._wave_hostile_ids:
            self.wave_eliminations += 1
        self._publish_state_change()

    def on_civilian_harmed(self) -> None:
        """Record a civilian harm event (civil unrest mode).

        Increments civilian_harm_count, subtracts 500 from de_escalation_score,
        and triggers defeat at civilian_harm_limit (default 5).
        """
        self.civilian_harm_count += 1
        self.de_escalation_score -= 500
        self._event_bus.publish("civilian_harmed", {
            "harm_count": self.civilian_harm_count,
            "harm_limit": self.civilian_harm_limit,
            "de_escalation_score": self.de_escalation_score,
        })
        if self.civilian_harm_count >= self.civilian_harm_limit and self.state == "active":
            self.state = "defeat"
            self._event_bus.publish("game_over", self._build_game_over_data(
                "defeat", reason="excessive_force",
                waves_completed=self.wave - 1,
            ))
            self._publish_state_change()

    def on_infrastructure_damaged(self, amount: float) -> None:
        """Apply damage to infrastructure health (drone swarm mode).

        Reduces infrastructure_health by amount and triggers defeat at 0.
        """
        self.infrastructure_health = max(0.0, self.infrastructure_health - amount)
        if self.infrastructure_health <= 0.0 and self.state == "active":
            self.state = "defeat"
            self._event_bus.publish("game_over", self._build_game_over_data(
                "defeat", reason="infrastructure_destroyed",
                waves_completed=self.wave - 1,
            ))
            self._publish_state_change()

    def get_state(self) -> dict:
        """Return serializable game state for API/frontend.

        Includes mode-specific fields when game_mode_type is
        "civil_unrest" or "drone_swarm".
        """
        wave_config = self._current_wave_config()
        if self._scenario_waves is not None:
            total_waves = len(self._scenario_waves)
        elif self.infinite:
            total_waves = -1
        else:
            total_waves = len(WAVE_CONFIGS)
        state: dict = {
            "state": self.state,
            "wave": self.wave,
            "wave_name": wave_config.name if wave_config else "",
            "total_waves": total_waves,
            "countdown": math.ceil(self._countdown_remaining) if self._countdown_remaining > 0 else 0,
            "score": self.score,
            "total_eliminations": self.total_eliminations,
            "wave_eliminations": self.wave_eliminations,
            "wave_hostiles_remaining": self._count_wave_hostiles_alive(),
            "infinite": self.infinite,
            "game_mode_type": self.game_mode_type,
            "difficulty_multiplier": self.difficulty.get_multiplier(),
        }
        if self.game_mode_type == "civil_unrest":
            state["de_escalation_score"] = self.de_escalation_score
            state["civilian_harm_count"] = self.civilian_harm_count
            state["civilian_harm_limit"] = self.civilian_harm_limit
            state["weighted_total_score"] = int(
                self.score * 0.3 + self.de_escalation_score * 0.7
            )
        elif self.game_mode_type == "drone_swarm":
            state["infrastructure_health"] = self.infrastructure_health
            state["infrastructure_max"] = self.infrastructure_max
        return state

    # -- State tick handlers ----------------------------------------------------

    def _tick_countdown(self, dt: float) -> None:
        self._countdown_remaining -= dt
        if self._countdown_remaining <= 0:
            self._countdown_remaining = 0
            self.state = "active"
            self._start_wave(self.wave)
            self._publish_state_change()

    def _tick_active(self, dt: float) -> None:
        # Check defeat: all friendly combatants eliminated
        # low_battery units are still alive (reduced capability, not dead)
        _ALIVE_STATUSES = ("active", "idle", "stationary", "low_battery", "arrived")
        friendlies_alive = [
            t for t in self._engine.get_targets()
            if t.alliance == "friendly" and t.is_combatant
            and t.status in _ALIVE_STATUSES
        ]
        if not friendlies_alive:
            self.state = "defeat"
            self._event_bus.publish("game_over", self._build_game_over_data(
                "defeat", reason="all_friendlies_eliminated",
                waves_completed=self.wave - 1,
            ))
            self._publish_state_change()
            return

        # Check wave complete: all wave hostiles eliminated or escaped
        alive = self._count_wave_hostiles_alive()
        if alive == 0 and not self._is_spawning():
            self._on_wave_complete()

    def _tick_wave_complete(self, dt: float) -> None:
        elapsed = time.time() - self._wave_complete_time
        if elapsed >= _WAVE_ADVANCE_DELAY:
            self.wave += 1
            # Determine total wave count: scenario waves, or default WAVE_CONFIGS
            if self._scenario_waves is not None:
                total_waves = len(self._scenario_waves)
            else:
                total_waves = len(WAVE_CONFIGS)
            if not self.infinite and self.wave > total_waves:
                # All waves cleared — victory! (finite mode only)
                self.state = "victory"
                self._event_bus.publish("game_over", self._build_game_over_data(
                    "victory", reason="all_waves_cleared",
                    waves_completed=total_waves,
                ))
                self._publish_state_change()
            else:
                self.state = "active"
                self._start_wave(self.wave)
                self._publish_state_change()

    # -- Scenario support -------------------------------------------------------

    def load_scenario(self, scenario) -> None:
        """Load a BattleScenario, replacing default WAVE_CONFIGS.

        Places defenders from the scenario onto the engine and stores
        wave definitions for use during gameplay.  Applies mode_config
        settings (civilian_harm_limit, infrastructure_max, etc.) when
        present.
        """
        from .scenario import BattleScenario, WaveDefinition
        self._scenario = scenario
        self._scenario_waves = list(scenario.waves)

        # Apply mode-specific configuration from the scenario
        mc = getattr(scenario, "mode_config", None) or {}
        if mc:
            # Civil unrest settings
            if "civilian_harm_limit" in mc:
                self.civilian_harm_limit = int(mc["civilian_harm_limit"])
            if "de_escalation_multiplier" in mc:
                self._de_escalation_multiplier = float(mc["de_escalation_multiplier"])
            # Drone swarm settings
            if "infrastructure_max" in mc:
                self.infrastructure_max = float(mc["infrastructure_max"])
                self.infrastructure_health = self.infrastructure_max

        # Update engine map bounds when scenario specifies larger area.
        # This ensures hostile spawning, escape detection, and hazard
        # placement use the full scenario area instead of the default 200m.
        scenario_bounds = getattr(scenario, "map_bounds", None)
        if scenario_bounds and scenario_bounds > self._engine._map_bounds:
            self._engine.set_map_bounds(scenario_bounds)

        # Place pre-defined defenders
        from .target import SimulationTarget
        for d in scenario.defenders:
            tid = f"{d.asset_type}-{d.name or 'auto'}-{id(d)}"
            target = SimulationTarget(
                target_id=tid,
                name=d.name or f"{d.asset_type.title()}",
                alliance="friendly",
                asset_type=d.asset_type,
                position=d.position,
                speed=0.0 if d.asset_type in ("turret", "heavy_turret", "missile_turret") else 2.0,
            )
            target.apply_combat_profile()
            self._engine.add_target(target)

    # -- Wave management --------------------------------------------------------

    def _current_wave_config(self) -> WaveConfig | None:
        # Scenario waves take priority over hardcoded WAVE_CONFIGS
        if self._scenario_waves and 1 <= self.wave <= len(self._scenario_waves):
            wave_def = self._scenario_waves[self.wave - 1]
            return WaveConfig(
                name=wave_def.name,
                count=wave_def.total_count,
                speed_mult=wave_def.speed_mult,
                health_mult=wave_def.health_mult,
            )
        if 1 <= self.wave <= len(WAVE_CONFIGS):
            return WAVE_CONFIGS[self.wave - 1]
        if self.infinite and self.wave > len(WAVE_CONFIGS):
            return self._infinite_wave_mode.get_wave_config(self.wave)
        return None

    def _start_wave(self, wave_num: int) -> None:
        """Spawn hostiles for this wave in a background thread (staggered)."""
        self.wave_eliminations = 0
        self._wave_start_time = time.time()
        self._wave_hostile_ids.clear()

        # Wave 3+ adds environmental pressure via random hazard spawning.
        # Hazard count scales with wave number (1 per wave, capped at 5).
        if wave_num >= 3 and hasattr(self._engine, 'hazard_manager'):
            hazard_count = min(wave_num - 2, 5)
            self._engine.hazard_manager.spawn_random(
                hazard_count, self._engine._map_bounds,
            )

        # Check if a scenario is loaded with custom wave definitions
        if self._scenario_waves is not None and 1 <= wave_num <= len(self._scenario_waves):
            wave_def = self._scenario_waves[wave_num - 1]
            event_data = {
                "wave_number": wave_num,
                "wave_name": wave_def.name,
                "hostile_count": wave_def.total_count,
            }
            if wave_def.briefing:
                event_data["briefing"] = wave_def.briefing
            if wave_def.threat_level:
                event_data["threat_level"] = wave_def.threat_level
            if wave_def.intel:
                event_data["intel"] = wave_def.intel
            self._event_bus.publish("wave_start", event_data)
            # Notify stats tracker of wave start
            if hasattr(self._engine, 'stats_tracker'):
                self._engine.stats_tracker.on_wave_start(
                    wave_num, wave_def.name, wave_def.total_count,
                )
            self._spawn_thread = threading.Thread(
                target=self._spawn_scenario_wave,
                args=(wave_def,),
                name=f"wave-{wave_num}-spawner",
                daemon=True,
            )
            self._spawn_thread.start()
            return

        # Default WAVE_CONFIGS path
        config = self._current_wave_config()
        if config is None:
            return

        self._event_bus.publish("wave_start", {
            "wave_number": wave_num,
            "wave_name": config.name,
            "hostile_count": config.count,
        })
        # Notify stats tracker of wave start
        if hasattr(self._engine, 'stats_tracker'):
            self._engine.stats_tracker.on_wave_start(
                wave_num, config.name, config.count,
            )

        # Spawn in background thread to stagger over time
        self._spawn_thread = threading.Thread(
            target=self._spawn_wave_hostiles,
            args=(config,),
            name=f"wave-{wave_num}-spawner",
            daemon=True,
        )
        self._spawn_thread.start()

    def _spawn_scenario_wave(self, wave_def) -> None:
        """Spawn hostiles from a scenario WaveDefinition (mixed types)."""
        from .scenario import WaveDefinition
        spawn_index = 0
        for group in wave_def.groups:
            for i in range(group.count):
                if self.state not in ("active",):
                    return
                hostile = self._engine.spawn_hostile_typed(
                    asset_type=group.asset_type,
                    speed=group.speed * wave_def.speed_mult,
                    health=group.health * wave_def.health_mult,
                    drone_variant=group.drone_variant,
                )
                self._wave_hostile_ids.add(hostile.target_id)
                spawn_index += 1
                if spawn_index < wave_def.total_count:
                    time.sleep(_SPAWN_STAGGER)

    def _spawn_wave_hostiles(self, config: WaveConfig) -> None:
        """Spawn hostiles with staggered timing.

        When config.composition is set, spawns mixed unit types in the
        specified quantities.  Otherwise falls back to all "person" type.

        Difficulty adjustments are applied on top of wave config multipliers:
        count is scaled by the difficulty multiplier, and health/speed bonuses
        are applied additively.
        """
        if config.composition:
            self._spawn_mixed_wave(config)
            return

        # Apply adaptive difficulty adjustments
        adj = self.difficulty.get_wave_adjustments(config.count)
        spawn_count = adj["hostile_count"]
        health_factor = config.health_mult * (1.0 + adj["hostile_health_bonus"])
        speed_factor = config.speed_mult * (1.0 + adj["hostile_speed_bonus"])
        if adj["easy"] and adj["speed_reduction"] > 0:
            speed_factor *= (1.0 - adj["speed_reduction"])

        for i in range(spawn_count):
            if self.state not in ("active",):
                break
            hostile = self._engine.spawn_hostile(direction=config.spawn_direction)
            # Apply wave + difficulty multipliers
            hostile.speed *= speed_factor
            hostile.health *= health_factor
            hostile.max_health *= health_factor
            self._wave_hostile_ids.add(hostile.target_id)
            if i < spawn_count - 1:
                time.sleep(_SPAWN_STAGGER)

    def _spawn_mixed_wave(self, config: WaveConfig) -> None:
        """Spawn a wave with mixed hostile unit types from config.composition.

        Each (asset_type, count) tuple in the composition list spawns that
        many units of the given type, with wave speed/health multipliers
        applied.  Difficulty adjustments are layered on top.
        Units are spawned in the listed order with staggered timing.
        """
        # Apply adaptive difficulty adjustments
        adj = self.difficulty.get_wave_adjustments(config.count)
        health_factor = config.health_mult * (1.0 + adj["hostile_health_bonus"])
        speed_factor = config.speed_mult * (1.0 + adj["hostile_speed_bonus"])
        if adj["easy"] and adj["speed_reduction"] > 0:
            speed_factor *= (1.0 - adj["speed_reduction"])

        spawn_index = 0
        total = sum(c for _, c in config.composition)
        for asset_type, type_count in config.composition:
            for _ in range(type_count):
                if self.state not in ("active",):
                    return
                hostile = self._engine.spawn_hostile_typed(
                    asset_type=asset_type,
                    speed=None,  # use default speed for type
                    health=None,  # apply from profile
                    direction=config.spawn_direction,
                )
                # Apply wave + difficulty multipliers
                hostile.speed *= speed_factor
                hostile.health *= health_factor
                hostile.max_health *= health_factor
                self._wave_hostile_ids.add(hostile.target_id)
                spawn_index += 1
                if spawn_index < total:
                    time.sleep(_SPAWN_STAGGER)

    def _is_spawning(self) -> bool:
        """Check if the wave spawner thread is still running."""
        return self._spawn_thread is not None and self._spawn_thread.is_alive()

    def _count_wave_hostiles_alive(self) -> int:
        """Count wave hostiles that are still active threats."""
        count = 0
        for t in self._engine.get_targets():
            if t.target_id in self._wave_hostile_ids and t.status == "active":
                count += 1
        return count

    def _on_wave_complete(self) -> None:
        """Handle wave completion: scoring, events, state transition."""
        elapsed = time.time() - self._wave_start_time
        # Time bonus: starts at 50, decreases by 5 per 10s elapsed
        time_bonus = max(0, 50 - int(elapsed / 10) * 5)
        wave_bonus = self.wave * 200
        self.score += wave_bonus + time_bonus

        # Record wave performance for adaptive difficulty
        hostiles_spawned = len(self._wave_hostile_ids)
        escapes = max(0, hostiles_spawned - self.wave_eliminations)
        friendly_damage = 0.0
        friendly_max_health = 0.0
        for t in self._engine.get_targets():
            if t.alliance == "friendly" and t.is_combatant:
                friendly_max_health += t.max_health
                friendly_damage += max(0.0, t.max_health - t.health)
        self.difficulty.record_wave({
            "eliminations": self.wave_eliminations,
            "hostiles_spawned": hostiles_spawned,
            "wave_time": elapsed,
            "friendly_damage_taken": friendly_damage,
            "friendly_max_health": max(1.0, friendly_max_health),
            "escapes": escapes,
        })

        self.state = "wave_complete"
        self._wave_complete_time = time.time()

        config = self._current_wave_config()
        self._event_bus.publish("wave_complete", {
            "wave_number": self.wave,
            "wave_name": config.name if config else "",
            "time_elapsed": round(elapsed, 1),
            "eliminations": self.wave_eliminations,
            "score_bonus": wave_bonus + time_bonus,
        })
        # Notify stats tracker of wave completion with score earned this wave
        if hasattr(self._engine, 'stats_tracker'):
            self._engine.stats_tracker.on_wave_complete(wave_bonus + time_bonus)
        self._publish_state_change()

    # -- Event publishing -------------------------------------------------------

    def _build_game_over_data(self, result: str, **extra) -> dict:
        """Build a game_over event dict with mode-specific fields included."""
        data = {
            "result": result,
            "final_score": self.score,
            "score": self.score,
            "wave": self.wave,
            "total_eliminations": self.total_eliminations,
            "game_mode_type": self.game_mode_type,
            **extra,
        }
        if self.game_mode_type == "civil_unrest":
            data["de_escalation_score"] = self.de_escalation_score
            data["civilian_harm_count"] = self.civilian_harm_count
            data["civilian_harm_limit"] = self.civilian_harm_limit
            data["weighted_total_score"] = int(
                self.score * 0.3 + self.de_escalation_score * 0.7
            )
        elif self.game_mode_type == "drone_swarm":
            data["infrastructure_health"] = self.infrastructure_health
            data["infrastructure_max"] = self.infrastructure_max
        return data

    def _publish_state_change(self) -> None:
        self._event_bus.publish("game_state_change", self.get_state())


# ---------------------------------------------------------------------------
# InfiniteWaveMode — procedural wave generation beyond wave 10
# ---------------------------------------------------------------------------

# Scaling constants
_INFINITE_BASE_COUNT = 3
_INFINITE_COUNT_GROWTH = 1.08       # count = base * growth^wave
_INFINITE_SPEED_GROWTH = 0.03       # speed_mult = 1.0 + 0.03 * wave
_INFINITE_HEALTH_BRACKET = 5        # health steps every 5 waves
_INFINITE_HEALTH_STEP = 0.2         # +0.2 per bracket
_INFINITE_ELITE_THRESHOLD = 10      # elites appear after wave 10
_INFINITE_BOSS_THRESHOLD = 20       # bosses appear after wave 20
_INFINITE_BOSS_INTERVAL = 5         # boss every 5 waves after threshold
_INFINITE_ELITE_HEALTH_MULT = 2.0   # elite health multiplier
_INFINITE_BOSS_HEALTH_MULT = 5.0    # boss health multiplier


class InfiniteWaveMode:
    """Procedural wave generation for endless survival mode.

    Generates WaveConfig objects for any wave number using deterministic
    scaling formulas:
      - count:      round(base_count * 1.08^wave_num), minimum 1
      - speed_mult: 1.0 + 0.03 * wave_num
      - health_mult: 1.0 + 0.2 * floor(wave_num / 5)
      - elites:     appear after wave 10 (2x health)
      - bosses:     appear on wave 21, 26, 31, ... (5x health)
      - score_mult: wave_num / 10
    """

    def __init__(
        self,
        base_count: int = _INFINITE_BASE_COUNT,
        difficulty: object | None = None,
    ) -> None:
        self._base_count = base_count
        self._difficulty = difficulty

    def get_wave_config(self, wave_num: int) -> WaveConfig:
        """Generate a WaveConfig for the given wave number.

        Args:
            wave_num: The wave number (1-based). Any positive integer works.

        Returns:
            A WaveConfig with procedurally scaled parameters.
        """
        # Count: exponential growth, minimum 1
        raw_count = self._base_count * (_INFINITE_COUNT_GROWTH ** wave_num)
        count = max(1, round(raw_count))

        # Speed: linear growth
        speed_mult = 1.0 + _INFINITE_SPEED_GROWTH * wave_num

        # Health: step function (increases every 5 waves)
        health_bracket = wave_num // _INFINITE_HEALTH_BRACKET
        health_mult = 1.0 + _INFINITE_HEALTH_STEP * health_bracket

        # Elites: appear after wave 10
        has_elites = wave_num > _INFINITE_ELITE_THRESHOLD
        elite_count = 0
        if has_elites:
            elite_count = max(1, (wave_num - _INFINITE_ELITE_THRESHOLD) // 3)

        # Boss: appears on 21, 26, 31, ... (first at 21, then every 5)
        has_boss = (
            wave_num > _INFINITE_BOSS_THRESHOLD
            and (wave_num - (_INFINITE_BOSS_THRESHOLD + 1)) % _INFINITE_BOSS_INTERVAL == 0
        )

        # Score multiplier: linear with wave number
        score_mult = wave_num / 10.0

        # Wave name
        if has_boss:
            name = f"BOSS WAVE {wave_num}"
        elif has_elites:
            name = f"Elite Assault {wave_num}"
        else:
            name = f"Wave {wave_num}"

        return WaveConfig(
            name=name,
            count=count,
            speed_mult=speed_mult,
            health_mult=health_mult,
            has_elites=has_elites,
            elite_count=elite_count,
            elite_health_mult=_INFINITE_ELITE_HEALTH_MULT,
            has_boss=has_boss,
            boss_health_mult=_INFINITE_BOSS_HEALTH_MULT,
            score_mult=score_mult,
        )


# ---------------------------------------------------------------------------
# InstigatorDetector -- identifies instigators via sustained proximity
# ---------------------------------------------------------------------------

# Friendly unit types that can identify instigators (scout/recon roles)
_IDENTIFIER_TYPES: frozenset[str] = frozenset({"scout_drone", "drone", "rover"})

# Alive statuses -- units must be in one of these to act as identifiers
_ALIVE_STATUSES: frozenset[str] = frozenset({"active", "idle", "stationary"})

# Default detection range in meters (overridable in constructor)
_DEFAULT_DETECTION_RANGE = 50.0

# Default sustained proximity time in seconds required for identification
_DEFAULT_DETECTION_TIME = 3.0

# De-escalation score awarded per instigator identification
_IDENTIFICATION_SCORE = 50


class InstigatorDetector:
    """Detects instigators in civil_unrest mode via sustained proximity.

    Friendly scout units (rover, drone, scout_drone) that stay within
    detection_range of an instigator for detection_time seconds will
    identify that instigator, publishing an ``instigator_identified``
    event and awarding de-escalation score points.

    Proximity timers are tracked per (friendly_id, instigator_id) pair.
    If the friendly moves out of range, the timer resets to zero.
    Already-identified instigators are skipped.
    """

    def __init__(
        self,
        event_bus: EventBus,
        detection_range: float = _DEFAULT_DETECTION_RANGE,
        detection_time: float = _DEFAULT_DETECTION_TIME,
        game_mode: GameMode | None = None,
        crowd_density_tracker: object | None = None,
    ) -> None:
        self._event_bus = event_bus
        self._detection_range = detection_range
        self._detection_time = detection_time
        self._game_mode = game_mode
        self._crowd_density_tracker = crowd_density_tracker
        # Proximity timers: (friendly_id, instigator_id) -> accumulated seconds
        self._timers: dict[tuple[str, str], float] = {}

    def tick(
        self,
        dt: float,
        targets: dict[str, SimulationTarget],
        game_mode_type: str,
    ) -> None:
        """Run one detection tick. Called from the engine tick loop.

        Args:
            dt: Time delta in seconds.
            targets: All simulation targets (dict of target_id -> SimulationTarget).
            game_mode_type: Current game mode type string (e.g. "civil_unrest").
        """
        if game_mode_type != "civil_unrest":
            return

        # Partition targets into identifiers and instigators
        identifiers: list[SimulationTarget] = []
        instigators: list[SimulationTarget] = []

        for t in targets.values():
            if (
                t.alliance == "friendly"
                and t.asset_type in _IDENTIFIER_TYPES
                and t.status in _ALIVE_STATUSES
            ):
                identifiers.append(t)
            elif (
                t.crowd_role == "instigator"
                and not t.identified
                and t.status in _ALIVE_STATUSES
            ):
                instigators.append(t)

        if not identifiers or not instigators:
            return

        # Track which (friendly, instigator) pairs are in range this tick
        in_range_pairs: set[tuple[str, str]] = set()
        range_sq = self._detection_range * self._detection_range

        for friendly in identifiers:
            fx, fy = friendly.position
            for instigator in instigators:
                ix, iy = instigator.position
                dx = fx - ix
                dy = fy - iy
                dist_sq = dx * dx + dy * dy
                if dist_sq <= range_sq:
                    pair = (friendly.target_id, instigator.target_id)
                    in_range_pairs.add(pair)

                    # Accumulate timer
                    elapsed = self._timers.get(pair, 0.0) + dt
                    self._timers[pair] = elapsed

                    if elapsed >= self._detection_time:
                        # Check crowd density at instigator position
                        if (
                            self._crowd_density_tracker is not None
                            and not self._crowd_density_tracker.can_identify_instigator(
                                instigator.position
                            )
                        ):
                            continue  # Too dense — timer stays but cannot identify
                        self._identify(instigator, friendly)
                        # Remove all timers for this instigator (it is done)
                        self._clear_instigator_timers(instigator.target_id)
                        break  # This instigator is identified, move on

        # Reset timers for pairs no longer in range
        stale_keys = [
            k for k in self._timers
            if k not in in_range_pairs
        ]
        for k in stale_keys:
            del self._timers[k]

    def _identify(
        self,
        instigator: SimulationTarget,
        identifier: SimulationTarget,
    ) -> None:
        """Mark instigator as identified and publish event."""
        instigator.identified = True

        self._event_bus.publish("instigator_identified", {
            "target_id": instigator.target_id,
            "identifier_id": identifier.target_id,
            "position": {
                "x": instigator.position[0],
                "y": instigator.position[1],
            },
        })

        # Award de-escalation score if game mode is available
        if self._game_mode is not None:
            self._game_mode.de_escalation_score += _IDENTIFICATION_SCORE

    def remove_unit(self, target_id: str) -> None:
        """Clean up timers for a removed unit (either friendly or instigator)."""
        keys_to_remove = [
            k for k in self._timers
            if k[0] == target_id or k[1] == target_id
        ]
        for k in keys_to_remove:
            del self._timers[k]

    def _clear_instigator_timers(self, instigator_id: str) -> None:
        """Remove all proximity timers for a given instigator."""
        keys_to_remove = [
            k for k in self._timers if k[1] == instigator_id
        ]
        for k in keys_to_remove:
            del self._timers[k]
