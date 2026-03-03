# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""SimulationTarget — a movable entity on the battlespace map.

Architecture
------------
SimulationTarget is a *flat dataclass* by design.  Every entity type
(rover, drone, turret, person, vehicle, animal) shares the same fields.
Type-specific behaviour is encoded in lookup tables (_DRAIN_RATES) and
in the tick() state machine rather than in a class hierarchy.

Why flat, not subclassed:
  - All targets flow through the same tick loop, EventBus serialisation,
    and TargetTracker pipeline.  Polymorphism would add dispatch overhead
    with no behavioural gain — a turret is just speed=0, a person has
    battery drain=0.
  - The level-format loader and spawners create targets from data; a
    component/subclass system would require a factory registry for the
    same outcome.

Waypoints live *on* the target because navigation is per-entity state
(current index, loop vs. one-shot).  A separate pathfinding service
would be warranted if targets needed collision avoidance or A*; for
waypoint-following this is simpler.

Threat classification is NOT a property of the target — it is computed
by ThreatClassifier in ``amy/escalation.py`` from zone membership and
dwell time.  SimulationTarget carries only *intrinsic* state.

Combat fields (health, weapon_range, etc.) support the projectile-based
combat system.  Unit type stat profiles are applied via
``apply_combat_profile()`` or set explicitly by factories/spawners.
"""

from __future__ import annotations

import hashlib
import math
import random as _random
import time as _time
from dataclasses import dataclass, field
from typing import Callable, TYPE_CHECKING

if TYPE_CHECKING:
    from .inventory import UnitInventory
    from .movement import MovementController


# ---------------------------------------------------------------------------
# West Dublin, CA street names and business data for identity generation
# ---------------------------------------------------------------------------

_DUBLIN_STREETS: list[str] = [
    "Dublin Blvd", "Village Parkway", "Silvergate Dr", "Alegre Dr",
    "Hacienda Dr", "Tassajara Rd", "Gleason Dr", "Amador Valley Blvd",
    "Dougherty Rd", "Clark Ave", "Briar Rose Ln", "Emerald Glen Dr",
    "Iron Horse Pkwy", "Regional St", "Golden Gate Dr", "Grafton St",
    "Sierra Ct", "Canyon Creek Cir", "Martindale Ct", "Stagecoach Rd",
    "Donner Way", "Donlon Way", "Scarlett Dr", "San Ramon Rd",
    "Antone Way", "Central Pkwy", "Lockhart St", "Finnian Way",
    "Kolln St", "Hansen Dr",
]

_DUBLIN_BUSINESSES: list[str] = [
    "Valley Auto Repair", "Dublin Ranch Dental", "Tri-Valley Fitness",
    "Lucky Supermarket", "Whole Foods Dublin", "Peet's Coffee",
    "Safeway Distribution", "Kaiser Permanente Dublin", "Target Dublin",
    "Ross Dress for Less", "Raley's", "Starbucks Village Parkway",
    "Dublin Toyota", "Bay Club", "Round Table Pizza",
    "Wells Fargo Dublin Blvd", "Chase Bank Hacienda", "In-N-Out Burger",
    "Tractor Supply Co", "The UPS Store Dublin", "Subway Dougherty",
    "Emerald Glen Recreation", "Dublin Sports Grounds", "Habit Burger",
]

_FIRST_NAMES: list[str] = [
    "James", "Maria", "David", "Priya", "Carlos", "Linda",
    "Aiden", "Susan", "Derek", "Tommy", "Patricia", "Miguel",
    "Jennifer", "Robert", "Mei", "Omar", "Elena", "Jamal",
    "Kenji", "Sarah", "Andre", "Rosa", "Kevin", "Fatima",
    "Brian", "Yuki", "Marcus", "Lila", "Trevor", "Anita",
    "Hassan", "Christine", "Raj", "Donna", "Tyrell", "Nina",
]

_LAST_NAMES: list[str] = [
    "Nakamura", "Okafor", "Venkatesh", "Reyes", "Medina",
    "Walsh", "Park", "Chen", "Gutierrez", "Patel", "Kim",
    "Johnson", "Williams", "Santos", "Garcia", "Martinez",
    "Nguyen", "Tanaka", "O'Brien", "Murphy", "Larsen",
    "Petrova", "Ali", "Singh", "Moreira", "Campbell",
    "Friedman", "Kowalski", "Adams", "Torres",
]

_VEHICLE_MAKES: list[tuple[str, list[str]]] = [
    ("Toyota", ["Camry", "Corolla", "RAV4", "Highlander", "Tacoma", "Prius"]),
    ("Honda", ["Civic", "Accord", "CR-V", "Pilot", "Odyssey"]),
    ("Ford", ["F-150", "Escape", "Explorer", "Mustang", "Maverick"]),
    ("Chevrolet", ["Silverado", "Malibu", "Equinox", "Tahoe", "Bolt EV"]),
    ("Tesla", ["Model 3", "Model Y", "Model S", "Model X"]),
    ("Subaru", ["Outback", "Forester", "Crosstrek", "Impreza"]),
    ("BMW", ["3 Series", "X3", "X5", "5 Series"]),
    ("Hyundai", ["Elantra", "Tucson", "Santa Fe", "Ioniq 5"]),
    ("Nissan", ["Altima", "Rogue", "Sentra", "Leaf"]),
    ("Kia", ["Sorento", "Sportage", "Forte", "EV6"]),
]

_VEHICLE_COLORS: list[str] = [
    "White", "Black", "Silver", "Gray", "Blue", "Red",
    "Green", "Beige", "Dark Blue", "Maroon", "Gold", "Brown",
]


# ---------------------------------------------------------------------------
# Deterministic identity generation functions (seeded from target_id)
# ---------------------------------------------------------------------------

def _seed_rng(target_id: str, salt: str = "") -> _random.Random:
    """Create a deterministic RNG seeded from target_id + salt."""
    h = hashlib.sha256(f"{target_id}:{salt}".encode()).digest()
    seed = int.from_bytes(h[:8], "big")
    return _random.Random(seed)


def generate_short_id(target_id: str) -> str:
    """Generate a 6-char uppercase hex identifier, deterministic per target_id.

    Example: "A3F2B1"
    """
    h = hashlib.sha256(f"short_id:{target_id}".encode()).hexdigest()
    return h[:6].upper()


def generate_mac_address(target_id: str, device: str = "wifi") -> str:
    """Generate a realistic-looking MAC address, deterministic per target_id + device.

    Uses locally-administered, unicast bit pattern (second nibble is 2/6/A/E).
    Example: "4A:3B:C2:1D:E5:F6"
    """
    rng = _seed_rng(target_id, f"mac:{device}")
    # First byte: locally-administered unicast (bit 1 set, bit 0 clear)
    first = rng.randint(0, 255) | 0x02  # set local bit
    first = first & 0xFE  # clear multicast bit
    octets = [first] + [rng.randint(0, 255) for _ in range(5)]
    return ":".join(f"{b:02X}" for b in octets)


def generate_license_plate(target_id: str) -> str:
    """Generate a California-format license plate, deterministic per target_id.

    Format: 1ABC234 (digit, 3 letters, 3 digits)
    Example: "8FKR472"
    """
    rng = _seed_rng(target_id, "plate")
    d1 = rng.randint(1, 9)
    letters = "".join(rng.choice("ABCDEFGHJKLMNPRSTUVWXYZ") for _ in range(3))
    d3 = rng.randint(100, 999)
    return f"{d1}{letters}{d3}"


def generate_cell_id(target_id: str) -> str:
    """Generate a fake IMSI-like cell identifier, deterministic per target_id.

    Format: 310-260-XXXXXXXXX (US MCC-MNC prefix + 9-digit MSIN)
    Example: "310-260-482917365"
    """
    rng = _seed_rng(target_id, "cell")
    msin = "".join(str(rng.randint(0, 9)) for _ in range(9))
    return f"310-260-{msin}"


def generate_address(target_id: str, salt: str = "home") -> str:
    """Generate a street address in West Dublin, CA, deterministic per target_id.

    Example: "4827 Silvergate Dr"
    """
    rng = _seed_rng(target_id, f"addr:{salt}")
    number = rng.randint(1000, 9999)
    street = rng.choice(_DUBLIN_STREETS)
    return f"{number} {street}"


def generate_serial(target_id: str) -> str:
    """Generate a robot/drone serial number, deterministic per target_id.

    Format: TRT-YYYY-NNNNN
    Example: "TRT-2026-00042"
    """
    rng = _seed_rng(target_id, "serial")
    year = rng.choice([2024, 2025, 2026])
    seq = rng.randint(1, 99999)
    return f"TRT-{year}-{seq:05d}"


def generate_firmware_version(target_id: str) -> str:
    """Generate a firmware version string, deterministic per target_id.

    Example: "v3.2.1-rc4"
    """
    rng = _seed_rng(target_id, "fw")
    major = rng.randint(1, 5)
    minor = rng.randint(0, 9)
    patch = rng.randint(0, 15)
    suffix = rng.choice(["", "", "", "-rc1", "-rc2", "-beta", "-hotfix"])
    return f"v{major}.{minor}.{patch}{suffix}"


def generate_person_name(target_id: str) -> tuple[str, str]:
    """Generate a (first_name, last_name) tuple, deterministic per target_id.

    Returns names reflecting West Dublin's diverse demographics.
    """
    rng = _seed_rng(target_id, "name")
    first = rng.choice(_FIRST_NAMES)
    last = rng.choice(_LAST_NAMES)
    return (first, last)


def generate_employer(target_id: str) -> tuple[str, str]:
    """Generate (business_name, business_address), deterministic per target_id."""
    rng = _seed_rng(target_id, "employer")
    biz = rng.choice(_DUBLIN_BUSINESSES)
    addr = generate_address(target_id, salt="work")
    return (biz, addr)


def generate_vehicle_info(target_id: str) -> dict[str, str | int]:
    """Generate vehicle make/model/year/color, deterministic per target_id.

    Returns dict with keys: make, model, year, color.
    """
    rng = _seed_rng(target_id, "vehicle")
    make_entry = rng.choice(_VEHICLE_MAKES)
    make = make_entry[0]
    model = rng.choice(make_entry[1])
    year = rng.randint(2012, 2026)
    color = rng.choice(_VEHICLE_COLORS)
    return {"make": make, "model": model, "year": year, "color": color}


# ---------------------------------------------------------------------------
# UnitIdentity dataclass
# ---------------------------------------------------------------------------

@dataclass
class UnitIdentity:
    """Rich identity information for a simulation unit.

    Generated once at spawn time. All fields optional — only relevant
    fields are populated based on unit type (person, vehicle, robot, etc.).
    """

    short_id: str = ""          # 6-char hex like "A3F2B1" (displayed above unit)

    # Person fields (civilian/hostile persons)
    first_name: str = ""
    last_name: str = ""
    home_address: str = ""
    employer: str = ""
    work_address: str = ""

    # Device signatures (radio detection — phone/watch BLE, WiFi, cellular)
    bluetooth_mac: str = ""     # Phone/watch BLE MAC
    wifi_mac: str = ""          # Phone WiFi MAC
    cell_id: str = ""           # IMSI or cell tower ID

    # Vehicle fields
    license_plate: str = ""
    vehicle_make: str = ""
    vehicle_model: str = ""
    vehicle_year: int = 0
    vehicle_color: str = ""
    owner_name: str = ""
    owner_address: str = ""

    # Robot/drone fields
    serial_number: str = ""
    firmware_version: str = ""
    operator: str = ""

    def to_dict(self) -> dict:
        """Serialize identity to dict, omitting empty/zero fields."""
        out: dict = {"short_id": self.short_id}
        # Person
        if self.first_name:
            out["first_name"] = self.first_name
        if self.last_name:
            out["last_name"] = self.last_name
        if self.home_address:
            out["home_address"] = self.home_address
        if self.employer:
            out["employer"] = self.employer
        if self.work_address:
            out["work_address"] = self.work_address
        # Devices
        if self.bluetooth_mac:
            out["bluetooth_mac"] = self.bluetooth_mac
        if self.wifi_mac:
            out["wifi_mac"] = self.wifi_mac
        if self.cell_id:
            out["cell_id"] = self.cell_id
        # Vehicle
        if self.license_plate:
            out["license_plate"] = self.license_plate
        if self.vehicle_make:
            out["vehicle_make"] = self.vehicle_make
        if self.vehicle_model:
            out["vehicle_model"] = self.vehicle_model
        if self.vehicle_year:
            out["vehicle_year"] = self.vehicle_year
        if self.vehicle_color:
            out["vehicle_color"] = self.vehicle_color
        if self.owner_name:
            out["owner_name"] = self.owner_name
        if self.owner_address:
            out["owner_address"] = self.owner_address
        # Robot
        if self.serial_number:
            out["serial_number"] = self.serial_number
        if self.firmware_version:
            out["firmware_version"] = self.firmware_version
        if self.operator:
            out["operator"] = self.operator
        return out


def build_identity(target_id: str, asset_type: str, alliance: str) -> UnitIdentity:
    """Build a UnitIdentity with fields populated based on unit type.

    All generation is deterministic per target_id so the same unit
    always gets the same identity across sessions.

    Population rules:
      - ALL units get short_id
      - person (any alliance) -> name, address, devices, employer
      - vehicle -> license plate, owner info, make/model/year/color
      - robot/drone/turret/tank/apc -> serial, firmware, operator
    """
    ident = UnitIdentity(short_id=generate_short_id(target_id))

    # Person types (hostile, neutral, or friendly person)
    if asset_type == "person":
        first, last = generate_person_name(target_id)
        ident.first_name = first
        ident.last_name = last
        ident.home_address = generate_address(target_id, salt="home")
        biz, biz_addr = generate_employer(target_id)
        ident.employer = biz
        ident.work_address = biz_addr
        ident.bluetooth_mac = generate_mac_address(target_id, device="bluetooth")
        ident.wifi_mac = generate_mac_address(target_id, device="wifi")
        ident.cell_id = generate_cell_id(target_id)

    # Vehicle types
    elif asset_type in ("vehicle", "hostile_vehicle"):
        vinfo = generate_vehicle_info(target_id)
        ident.license_plate = generate_license_plate(target_id)
        ident.vehicle_make = vinfo["make"]
        ident.vehicle_model = vinfo["model"]
        ident.vehicle_year = vinfo["year"]
        ident.vehicle_color = vinfo["color"]
        # Owner registration
        first, last = generate_person_name(target_id)
        ident.owner_name = f"{first} {last}"
        ident.owner_address = generate_address(target_id, salt="owner")

    # Robot/drone/turret/tank/apc/sensor types
    elif asset_type in (
        "rover", "drone", "turret", "scout_drone", "swarm_drone",
        "heavy_turret", "missile_turret", "tank", "apc", "camera", "sensor",
    ):
        ident.serial_number = generate_serial(target_id)
        ident.firmware_version = generate_firmware_version(target_id)
        ident.operator = "TRITIUM Defense Systems"
        # Robots have WiFi and BLE for mesh networking
        ident.wifi_mac = generate_mac_address(target_id, device="wifi")
        ident.bluetooth_mac = generate_mac_address(target_id, device="bluetooth")

    return ident


# Battery drain rates per second by asset type
_DRAIN_RATES: dict[str, float] = {
    "rover": 0.001,
    "drone": 0.002,
    "turret": 0.0005,
    "scout_drone": 0.0025,
    "person": 0.0,
    "vehicle": 0.0,
    "animal": 0.0,
    "heavy_turret": 0.0004,
    "missile_turret": 0.0003,
    "tank": 0.0008,
    "apc": 0.0010,
    "swarm_drone": 0.003,
    # Mission-type entries (simulation-only, no real battery)
    "instigator": 0.0,
    "rioter": 0.0,
    "civilian": 0.0,
    "scout_swarm": 0.0,
    "attack_swarm": 0.0,
    "bomber_swarm": 0.0,
}

# Combat stat profiles by (asset_type, alliance).
# Format: (health, max_health, weapon_range, weapon_cooldown, weapon_damage, is_combatant)
_COMBAT_PROFILES: dict[str, tuple[float, float, float, float, float, bool]] = {
    "turret":           (200.0, 200.0, 80.0, 1.5, 15.0, True),
    "drone":            (60.0,  60.0,  50.0, 1.0,  8.0, True),
    "rover":            (150.0, 150.0, 60.0, 2.0, 12.0, True),
    "person_hostile":   (80.0,  80.0,  40.0, 2.5, 10.0, True),
    "person_neutral":   (50.0,  50.0,   0.0, 0.0,  0.0, False),
    "vehicle":          (300.0, 300.0,  0.0, 0.0,  0.0, False),
    "animal":           (30.0,  30.0,   0.0, 0.0,  0.0, False),
    # Heavy units
    "tank":             (400.0, 400.0, 100.0, 3.0, 30.0, True),
    "apc":              (300.0, 300.0,  60.0, 1.0,  8.0, True),
    "heavy_turret":     (350.0, 350.0, 120.0, 2.5, 25.0, True),
    "missile_turret":   (200.0, 200.0, 150.0, 5.0, 50.0, True),
    # Scout variant
    "scout_drone":      (40.0,  40.0,  40.0, 1.5,  5.0, True),
    # Hostile variants
    "hostile_vehicle":  (200.0, 200.0, 70.0, 2.0, 15.0, True),
    "hostile_leader":   (150.0, 150.0, 50.0, 2.0, 12.0, True),
    # Swarm drone: fast, fragile, short-range
    "swarm_drone":      (25.0,  25.0,  20.0, 1.0,  5.0, True),
    # Civil unrest crowd roles
    "instigator":       (60.0,  60.0,  15.0, 3.0,  5.0, True),   # Low range, thrown objects
    "rioter":           (50.0,  50.0,   3.0, 2.0,  3.0, True),   # Melee range only
    "civilian":         (50.0,  50.0,   0.0, 0.0,  0.0, False),  # Non-combatant
    # Non-combatant sensors
    "camera":           (50.0,  50.0,   0.0, 0.0,  0.0, False),  # Passive observation
    "sensor":           (30.0,  30.0,   0.0, 0.0,  0.0, False),  # Passive detection
    # Drone swarm variants
    "scout_swarm":      (15.0,  15.0,   0.0, 0.0,  0.0, False),  # Recon only, no weapons
    "attack_swarm":     (30.0,  30.0,  25.0, 1.0,  8.0, True),   # Strafing runs
    "bomber_swarm":     (50.0,  50.0,   0.0, 0.0, 40.0, True),   # Kamikaze, damage on detonation
    # Graphling agents (crystal creatures from the graphlings project)
    "graphling":        (80.0,  80.0,  25.0, 1.5,  8.0, True),   # Light combatant, very fast
}


def _profile_key(
    asset_type: str,
    alliance: str,
    crowd_role: str | None = None,
    drone_variant: str | None = None,
) -> str:
    """Return the combat profile lookup key for a target.

    Dispatch order:
      1. crowd_role overrides (instigator, rioter, civilian)
      2. drone_variant overrides (scout_swarm, attack_swarm, bomber_swarm)
      3. person + alliance → person_hostile / person_neutral
      4. Fallback to asset_type
    """
    if crowd_role is not None and crowd_role in _COMBAT_PROFILES:
        return crowd_role
    if drone_variant is not None and drone_variant in _COMBAT_PROFILES:
        return drone_variant
    if asset_type == "person":
        if alliance == "hostile":
            return "person_hostile"
        return "person_neutral"
    return asset_type


@dataclass
class SimulationTarget:
    """A single simulated entity (rover, drone, turret, person, etc.).

    Lifecycle:
      active -> idle/stationary/arrived/escaped/neutralized/eliminated/despawned/low_battery -> destroyed
      Hostiles: active -> escaped (reached exit edge) or neutralized/eliminated (intercepted/killed)
      Friendlies: active -> arrived (dispatch) or loops patrol (loop_waypoints=True)
      Neutrals: active -> despawned (reached destination)
    """

    target_id: str
    name: str
    alliance: str  # "friendly", "hostile", "neutral", "unknown"
    asset_type: str  # "rover", "drone", "turret", "person", "vehicle", "animal"
    position: tuple[float, float]  # (x, y) on map — units are abstract map coords
    heading: float = 0.0  # degrees, 0 = north (+y), clockwise
    altitude: float = 0.0  # meters above ground (0 = ground level)
    speed: float = 1.0  # units/second (0.0 for turrets)
    battery: float = 1.0  # 0.0-1.0 (persons/animals drain at 0.0)
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    _waypoint_index: int = 0
    status: str = "active"  # "active", "idle", "stationary", "arrived", "escaped", "neutralized", "eliminated", "despawned", "low_battery", "destroyed"
    loop_waypoints: bool = False  # True for friendly patrol routes, False for one-shot paths

    # Combat fields
    health: float = 100.0
    max_health: float = 100.0
    weapon_range: float = 15.0    # meters — how far this unit can shoot
    weapon_cooldown: float = 2.0  # seconds between shots
    weapon_damage: float = 10.0   # damage per hit
    last_fired: float = 0.0       # timestamp of last shot
    kills: int = 0
    is_combatant: bool = True     # False for civilians/animals
    vision_range: float = 15.0    # meters — how far this unit can see

    # FSM state name (set by engine from unit FSM, None when no FSM assigned)
    fsm_state: str | None = None

    # Extended simulation fields
    squad_id: str | None = None       # Squad membership (None = solo unit)
    morale: float = 1.0               # Unit morale 0.0 (broken) to 1.0 (full)
    max_morale: float = 1.0           # Maximum morale cap
    degradation: float = 0.0          # Equipment degradation 0.0 = pristine, 1.0 = broken
    detected: bool = False            # Whether this unit has been spotted by enemies
    detected_at: float = 0.0          # Timestamp when first detected
    visible: bool = True              # Whether this target is visible to friendlies
    detected_by: list[str] = field(default_factory=list)  # IDs of units that can see this target
    radio_detected: bool = False      # Whether detected via radio signals (BLE/WiFi/cell)
    radio_signal_strength: float = 0.0  # Signal strength 0.0-1.0 (inverse square of distance)
    is_leader: bool = False           # Whether this unit is the squad leader
    _fleeing: bool = False            # Internal fleeing state (unit is retreating)

    # Mission-type fields (civil unrest, drone swarm)
    crowd_role: str | None = None         # "civilian", "instigator", "rioter" for civil unrest
    drone_variant: str | None = None      # "scout_swarm", "attack_swarm", "bomber_swarm" for drone swarm
    instigator_state: str = "hidden"      # Activation cycle: hidden / activating / active
    instigator_timer: float = 0.0         # Timer for activation cycle (seconds)
    identified: bool = False              # Whether this instigator has been identified by a friendly scout
    ammo_count: int = -1                  # -1 = unlimited, 20 for missile turrets
    ammo_max: int = -1                    # Max ammo capacity (-1 = unlimited)

    # Source classification: where this target came from
    # "sim" = locally simulated, "real" = physical hardware/YOLO, "graphling" = remote agent
    source: str = "sim"

    # Rich identity information (generated once at spawn from target_id)
    identity: UnitIdentity | None = field(default=None, repr=False)

    # Per-unit inventory (armor, weapons, consumables)
    inventory: UnitInventory | None = field(default=None, repr=False)

    # Smooth movement controller (assigned in __post_init__ for mobile combatants)
    movement: MovementController | None = field(default=None, repr=False)

    # Track waypoints version to detect external changes
    _waypoints_version: int = field(default=0, repr=False)

    # Building collision check: callable(x, y) -> True if blocked.
    # Set by engine.add_target() when obstacles are loaded.
    # Flying units (drone, scout_drone, swarm_drone) are exempt.
    _collision_check: Callable[[float, float], bool] | None = field(
        default=None, repr=False
    )

    # Types that fly over buildings (exempt from collision)
    _FLYING_TYPES: tuple[str, ...] = field(
        default=("drone", "scout_drone", "swarm_drone"), init=False, repr=False
    )

    def __post_init__(self) -> None:
        """Auto-create MovementController and UnitIdentity.

        MovementController: Only combatant units (friendly or hostile) with
        speed > 0 get smooth movement.  Neutral entities (pedestrians,
        animals, vehicles) use legacy linear movement.

        UnitIdentity: Generated once at spawn, deterministic per target_id.
        """
        # Auto-generate identity if not provided
        if self.identity is None:
            self.identity = build_identity(
                self.target_id, self.asset_type, self.alliance,
            )

        # Auto-build inventory loadout for combatants if not provided
        if self.is_combatant and self.inventory is None:
            from .inventory import build_loadout
            self.inventory = build_loadout(
                self.target_id, self.asset_type, self.alliance,
            )

        # Set vision_range from unit type registry if still at default
        if self.vision_range == 15.0:
            from engine.units import get_type
            utype = get_type(self.asset_type)
            if utype is not None:
                self.vision_range = utype.vision_radius

        if self.speed > 0 and self.is_combatant and self.alliance != "neutral":
            from .movement import MovementController
            # Convert target heading (0=north, 90=east) to MC heading (0=east, 90=north)
            mc_heading = (90.0 - self.heading) % 360.0
            self.movement = MovementController(
                max_speed=self.speed,
                turn_rate=max(180.0, self.speed * 90.0),
                acceleration=max(4.0, self.speed * 2.0),
                deceleration=max(6.0, self.speed * 3.0),
                x=self.position[0],
                y=self.position[1],
                heading=mc_heading,
            )
            # Push initial waypoints to controller if any
            if self.waypoints:
                self.movement.set_path(self.waypoints, loop=self.loop_waypoints)
                self._waypoints_version = id(self.waypoints)

    def apply_combat_profile(self) -> None:
        """Apply combat stats from _COMBAT_PROFILES based on asset_type, alliance,
        crowd_role, and drone_variant."""
        key = _profile_key(
            self.asset_type, self.alliance,
            crowd_role=self.crowd_role,
            drone_variant=self.drone_variant,
        )
        profile = _COMBAT_PROFILES.get(key)
        if profile is None:
            return
        (self.health, self.max_health, self.weapon_range,
         self.weapon_cooldown, self.weapon_damage, self.is_combatant) = profile

    def set_collision_check(
        self,
        check: Callable[[float, float], bool],
        height_at: Callable[[float, float], float | None] | None = None,
    ) -> None:
        """Set the building collision checker.

        For flying types with a *height_at* callable, collision is 3D-aware:
        blocked only when ``height_at(x, y)`` is not None AND ``self.altitude
        <= height``.  Without *height_at*, flying types are exempt (legacy).

        Ground types always use the 2D ``check`` callable as-is.
        """
        if self.asset_type in self._FLYING_TYPES:
            if height_at is not None:
                # 3D-aware collision: blocked only when inside footprint AND
                # altitude is at or below the roof height.
                def _3d_check(x: float, y: float) -> bool:
                    h = height_at(x, y)
                    return h is not None and self.altitude <= h
                self._collision_check = _3d_check
            else:
                self._collision_check = None
            return
        self._collision_check = check

    def apply_damage(self, amount: float) -> bool:
        """Apply *amount* damage. Returns True if this target is eliminated (health <= 0)."""
        if self.status in ("destroyed", "eliminated", "neutralized"):
            return True
        self.health = max(0.0, self.health - amount)
        if self.health <= 0:
            self.status = "eliminated"
            return True
        return False

    def can_fire(self) -> bool:
        """Check if this target can fire right now (cooldown elapsed, has weapon, alive)."""
        if self.status not in ("active", "idle", "stationary"):
            return False
        if self.weapon_range <= 0 or self.weapon_damage <= 0:
            return False
        if not self.is_combatant:
            return False
        now = _time.time()
        return (now - self.last_fired) >= self.weapon_cooldown

    def tick(self, dt: float) -> None:
        """Advance simulation by *dt* seconds."""
        if self.status in ("destroyed", "low_battery", "neutralized", "escaped", "eliminated"):
            return

        # Battery drain
        drain = _DRAIN_RATES.get(self.asset_type, 0.001) * dt
        self.battery = max(0.0, self.battery - drain)
        if self.battery < 0.05:
            self.status = "low_battery"
            return

        if self.movement is not None:
            self._tick_with_controller(dt)
        else:
            self._tick_legacy(dt)

    def _tick_with_controller(self, dt: float) -> None:
        """Movement driven by MovementController (smooth acceleration/turns)."""
        mc = self.movement
        assert mc is not None  # caller guarantees

        # Sync waypoints to controller if they changed externally
        wv = id(self.waypoints)
        if wv != self._waypoints_version:
            self._waypoints_version = wv
            if self.waypoints:
                mc.set_path(self.waypoints, loop=self.loop_waypoints)
            else:
                mc.stop()

        # No waypoints and controller has arrived => idle/stationary
        if not self.waypoints and mc.arrived:
            if self.status == "active":
                self.status = "idle"
            return
        if self.speed <= 0:
            if self.status == "active":
                self.status = "stationary"
            return

        # Save pre-tick position for collision rollback
        pre_x, pre_y = mc.x, mc.y

        # Tick the controller
        mc.tick(dt)

        # Building collision check: if new position is inside a building,
        # revert to pre-tick position and advance past the blocking waypoint.
        if self._collision_check is not None and self._collision_check(mc.x, mc.y):
            mc.x = pre_x
            mc.y = pre_y
            # Advance waypoint index past the blocking segment
            if mc._waypoints and mc._waypoint_index < len(mc._waypoints):
                mc._waypoint_index += 1
                if mc._waypoint_index >= len(mc._waypoints):
                    if mc._patrol_loop:
                        mc._waypoint_index = 0
                    else:
                        # All remaining waypoints are blocked by buildings.
                        # Do NOT set arrived=True — that triggers terminal
                        # status ("escaped"/"arrived"/"despawned") which
                        # permanently strands the unit.  Instead, rewind
                        # to the last waypoint and stop movement so the
                        # behavior tick can detect the stall and re-path.
                        mc._waypoint_index = max(0, len(mc._waypoints) - 1)
                        mc.speed = 0.0

        # Sync position and heading from controller.
        # MovementController uses math convention: 0=east, 90=north (atan2(dy,dx)).
        # SimulationTarget uses map convention: 0=north, 90=east (atan2(dx,dy)).
        # Conversion: target_heading = (90 - mc_heading) % 360
        self.position = (mc.x, mc.y)
        self.heading = (90.0 - mc.heading) % 360.0

        # Check arrival
        if mc.arrived:
            # Controller finished its path — determine terminal status
            if self.alliance == "neutral":
                self.status = "despawned"
            elif self.alliance == "hostile":
                self.status = "escaped"
            elif self.loop_waypoints:
                # Friendly patrol — controller handles looping internally
                pass
            else:
                self.status = "arrived"

    def _tick_legacy(self, dt: float) -> None:
        """Legacy linear movement (non-combatants: neutrals, animals, etc.)."""
        # Movement toward current waypoint
        if not self.waypoints:
            if self.status == "active":
                self.status = "idle"
            return
        if self.speed <= 0:
            if self.status == "active":
                self.status = "stationary"
            return

        tx, ty = self.waypoints[self._waypoint_index]

        # Pre-check: if the current waypoint itself is inside a building,
        # skip it immediately (path segment crosses a building).
        if self._collision_check is not None and self._collision_check(tx, ty):
            if self._waypoint_index >= len(self.waypoints) - 1:
                # All remaining waypoints blocked — terminal status
                if self.alliance == "neutral":
                    self.status = "despawned"
                elif self.alliance == "hostile":
                    # Don't escape — stay active to participate in combat
                    pass
                else:
                    self.status = "idle"
                return
            self._waypoint_index += 1
            return

        dx = tx - self.position[0]
        dy = ty - self.position[1]
        dist = math.hypot(dx, dy)

        if dist < 1.0:
            # Arrived at waypoint — advance or finish
            if self._waypoint_index >= len(self.waypoints) - 1:
                # Path complete — terminal status depends on alliance
                if self.alliance == "neutral":
                    self.status = "despawned"
                elif self.alliance == "hostile":
                    self.status = "escaped"
                elif self.loop_waypoints:
                    # Friendly patrol — loop back to start
                    self._waypoint_index = 0
                else:
                    # Friendly one-shot dispatch — arrived at destination
                    self.status = "arrived"
                return
            else:
                self._waypoint_index += 1
            return

        # Update heading: atan2(dx, dy) so 0 = north (+y), convert to degrees
        self.heading = math.degrees(math.atan2(dx, dy))

        # Move toward waypoint
        step = min(self.speed * dt, dist)
        new_x = self.position[0] + (dx / dist) * step
        new_y = self.position[1] + (dy / dist) * step

        # Building collision check: reject moves into buildings
        if self._collision_check is not None and self._collision_check(new_x, new_y):
            # Movement would enter a building — skip to next waypoint
            if self._waypoint_index >= len(self.waypoints) - 1:
                if self.alliance == "neutral":
                    self.status = "despawned"
                elif self.alliance == "hostile":
                    # Don't escape — stay active to participate in combat
                    pass
                else:
                    self.status = "idle"
                return
            self._waypoint_index += 1
            return

        self.position = (new_x, new_y)

    def to_dict(self, viewer_alliance: str | None = "friendly") -> dict:
        """Serialize for EventBus / API consumption.

        Includes both local position (meters from reference) and real
        lat/lng/alt from the server-side geo-reference.  If no reference
        is set, lat/lng default to 0.

        Args:
            viewer_alliance: Alliance of the viewer for fog-of-war.
                "friendly" (default): friendly units show full inventory,
                    hostile/neutral units show fog dict.
                None: internal/debug view, shows everything.

        Note: threat_level is NOT included — it is computed externally by
        ThreatClassifier and lives in ThreatRecord, not on the target.
        """
        from engine.tactical.geo import local_to_latlng
        geo = local_to_latlng(self.position[0], self.position[1], self.altitude)
        d = {
            "target_id": self.target_id,
            "name": self.name,
            "alliance": self.alliance,
            "asset_type": self.asset_type,
            "position": {"x": self.position[0], "y": self.position[1]},
            "lat": geo["lat"],
            "lng": geo["lng"],
            "alt": geo["alt"],
            "heading": self.heading,
            "altitude": self.altitude,
            "speed": self.speed,
            "battery": round(self.battery, 4),
            "status": self.status,
            "waypoints": [{"x": w[0], "y": w[1]} for w in self.waypoints],
            "loop_waypoints": self.loop_waypoints,
            "health": round(self.health, 1),
            "max_health": round(self.max_health, 1),
            "kills": self.kills,
            "is_combatant": self.is_combatant,
            "fsm_state": self.fsm_state,
            "squad_id": self.squad_id,
            "morale": round(self.morale, 2),
            "degradation": round(self.degradation, 2),
            "detected": self.detected,
            "visible": self.visible,
            "detected_by": self.detected_by,
            "radio_detected": self.radio_detected,
            "radio_signal_strength": round(self.radio_signal_strength, 3),
            "weapon_range": round(self.weapon_range, 1),
            "vision_range": round(self.vision_range, 1),
            "crowd_role": self.crowd_role,
            "drone_variant": self.drone_variant,
            "instigator_state": self.instigator_state,
            "identified": self.identified,
            "ammo_count": self.ammo_count,
            "ammo_max": self.ammo_max,
            "identity": self.identity.to_dict() if self.identity else None,
            "source": self.source,
            "role_name": getattr(self, "role_name", None),
        }

        # Inventory serialization with fog-of-war
        if self.inventory is not None:
            # Show full inventory when:
            #   - viewer_alliance is None (internal/debug view)
            #   - this unit is friendly (always reveal own team)
            #   - this unit is eliminated (loot reveal)
            if (
                viewer_alliance is None
                or self.alliance == "friendly"
                or self.alliance == viewer_alliance
                or self.status == "eliminated"
            ):
                d["inventory"] = self.inventory.to_dict()
            else:
                d["inventory"] = self.inventory.to_fog_dict()
        else:
            d["inventory"] = None

        return d
