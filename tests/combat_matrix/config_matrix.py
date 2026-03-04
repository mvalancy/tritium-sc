# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""BattleConfig and ConfigMatrix — parametric battle configuration generator."""

from __future__ import annotations

import itertools
import random
from dataclasses import dataclass, field
from typing import Any


@dataclass
class LoadoutProfile:
    """Hostile weapon loadout profile."""

    name: str
    weapon_name: str
    damage: float
    weapon_range: float
    cooldown: float
    ammo: int
    weapon_class: str


@dataclass
class BattleConfig:
    """Complete configuration for a single battle run."""

    config_id: str
    defender_count: int
    defender_types: list[str]
    hostile_count: int
    hostile_type: str = "person"
    loadout_profile: str = "small_arms"
    weapon_overrides: dict[str, Any] = field(default_factory=dict)
    map_bounds: float = 100.0
    seed: int = 42
    total_defender_ammo: int = 0
    total_hostile_ammo: int = 0

    @property
    def total_ammo(self) -> int:
        return self.total_defender_ammo + self.total_hostile_ammo


# -- Loadout profiles -------------------------------------------------------

LOADOUT_PROFILES: dict[str, LoadoutProfile] = {
    "small_arms": LoadoutProfile(
        name="small_arms", weapon_name="nerf_pistol",
        damage=10, weapon_range=8, cooldown=2.5, ammo=15, weapon_class="ballistic",
    ),
    "rifles": LoadoutProfile(
        name="rifles", weapon_name="nerf_rifle",
        damage=12, weapon_range=40, cooldown=1.5, ammo=25, weapon_class="ballistic",
    ),
    "shotguns": LoadoutProfile(
        name="shotguns", weapon_name="nerf_shotgun",
        damage=25, weapon_range=8, cooldown=2.5, ammo=6, weapon_class="ballistic",
    ),
    "smg_rush": LoadoutProfile(
        name="smg_rush", weapon_name="nerf_smg",
        damage=6, weapon_range=20, cooldown=0.3, ammo=50, weapon_class="ballistic",
    ),
    "rockets": LoadoutProfile(
        name="rockets", weapon_name="nerf_rpg",
        damage=60, weapon_range=50, cooldown=8.0, ammo=3, weapon_class="missile",
    ),
    "heavy_aoe": LoadoutProfile(
        name="heavy_aoe", weapon_name="nerf_tank_cannon",
        damage=30, weapon_range=25, cooldown=3.0, ammo=6, weapon_class="aoe",
    ),
}

# -- Force ratios ------------------------------------------------------------

FORCE_RATIOS: list[tuple[int, int]] = [
    (4, 4), (4, 10), (4, 20), (4, 40), (2, 8), (8, 8), (1, 10),
]

# -- Defender mixes ----------------------------------------------------------

DEFENDER_MIXES: dict[str, list[str]] = {
    "all_turrets": ["turret"],
    "all_mobile": ["rover", "drone"],
    "mixed_ground": ["turret", "rover", "tank", "apc"],
    "air_support": ["turret", "drone", "scout_drone", "drone"],
    "combined_arms": ["heavy_turret", "tank", "rover", "drone"],
}

# -- Map sizes ---------------------------------------------------------------

MAP_SIZES: dict[str, float] = {
    "tight": 50.0,
    "medium": 100.0,
    "large": 200.0,
}

# Default ammo per defender type (from _DEFAULT_WEAPONS in weapons.py)
_DEFENDER_AMMO: dict[str, int] = {
    "turret": 100,
    "drone": 20,
    "rover": 40,
    "tank": 20,
    "apc": 60,
    "heavy_turret": 50,
    "missile_turret": 10,
    "scout_drone": 15,
}


def _expand_types(mix_key: str, count: int) -> list[str]:
    """Cycle through mix types to fill count."""
    cycle = DEFENDER_MIXES[mix_key]
    return [cycle[i % len(cycle)] for i in range(count)]


def _compute_defender_ammo(types: list[str]) -> int:
    """Sum default ammo for a list of defender types."""
    return sum(_DEFENDER_AMMO.get(t, 30) for t in types)


class ConfigMatrix:
    """Generates battle configurations for parametric testing."""

    @staticmethod
    def generate_fast_sweep(count: int = 50, seed: int = 42) -> list[BattleConfig]:
        """Latin Hypercube Sampling across all axes.

        Selects `count` configurations spread across the full parameter space
        (loadouts x ratios x mixes x maps) without exhaustive enumeration.
        """
        rng = random.Random(seed)

        loadout_keys = list(LOADOUT_PROFILES.keys())
        mix_keys = list(DEFENDER_MIXES.keys())
        map_keys = list(MAP_SIZES.keys())
        ratio_list = list(FORCE_RATIOS)

        # Shuffle each axis independently for LHS-like coverage
        loadout_indices = list(range(count))
        mix_indices = list(range(count))
        map_indices = list(range(count))
        ratio_indices = list(range(count))
        rng.shuffle(loadout_indices)
        rng.shuffle(mix_indices)
        rng.shuffle(map_indices)
        rng.shuffle(ratio_indices)

        configs: list[BattleConfig] = []
        for i in range(count):
            loadout_key = loadout_keys[loadout_indices[i] % len(loadout_keys)]
            mix_key = mix_keys[mix_indices[i] % len(mix_keys)]
            map_key = map_keys[map_indices[i] % len(map_keys)]
            defenders, hostiles = ratio_list[ratio_indices[i] % len(ratio_list)]

            profile = LOADOUT_PROFILES[loadout_key]
            defender_types = _expand_types(mix_key, defenders)
            map_bounds = MAP_SIZES[map_key]

            defender_ammo = _compute_defender_ammo(defender_types)
            hostile_ammo = hostiles * profile.ammo

            config_id = f"lhs_{i:03d}_{loadout_key}_{defenders}v{hostiles}_{mix_key}_{map_key}"
            configs.append(BattleConfig(
                config_id=config_id,
                defender_count=defenders,
                defender_types=defender_types,
                hostile_count=hostiles,
                loadout_profile=loadout_key,
                weapon_overrides={
                    "weapon_damage": profile.damage,
                    "weapon_range": profile.weapon_range,
                    "weapon_cooldown": profile.cooldown,
                    "ammo_count": profile.ammo,
                    "ammo_max": profile.ammo,
                },
                map_bounds=map_bounds,
                seed=seed + i,
                total_defender_ammo=defender_ammo,
                total_hostile_ammo=hostile_ammo,
            ))

        return configs

    @staticmethod
    def generate_full_sweep(seed: int = 42) -> list[BattleConfig]:
        """All loadout x ratio combos with random mix/map per combo."""
        rng = random.Random(seed)

        loadout_keys = list(LOADOUT_PROFILES.keys())
        ratio_list = list(FORCE_RATIOS)
        mix_keys = list(DEFENDER_MIXES.keys())
        map_keys = list(MAP_SIZES.keys())

        configs: list[BattleConfig] = []
        idx = 0
        for loadout_key, (defenders, hostiles) in itertools.product(loadout_keys, ratio_list):
            mix_key = rng.choice(mix_keys)
            map_key = rng.choice(map_keys)

            profile = LOADOUT_PROFILES[loadout_key]
            defender_types = _expand_types(mix_key, defenders)
            map_bounds = MAP_SIZES[map_key]

            defender_ammo = _compute_defender_ammo(defender_types)
            hostile_ammo = hostiles * profile.ammo

            config_id = f"full_{idx:03d}_{loadout_key}_{defenders}v{hostiles}_{mix_key}_{map_key}"
            configs.append(BattleConfig(
                config_id=config_id,
                defender_count=defenders,
                defender_types=defender_types,
                hostile_count=hostiles,
                loadout_profile=loadout_key,
                weapon_overrides={
                    "weapon_damage": profile.damage,
                    "weapon_range": profile.weapon_range,
                    "weapon_cooldown": profile.cooldown,
                    "ammo_count": profile.ammo,
                    "ammo_max": profile.ammo,
                },
                map_bounds=map_bounds,
                seed=seed + idx,
                total_defender_ammo=defender_ammo,
                total_hostile_ammo=hostile_ammo,
            ))
            idx += 1

        return configs
