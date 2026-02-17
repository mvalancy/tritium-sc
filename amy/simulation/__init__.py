"""Backward-compatible re-export — canonical location is simulation/."""
from simulation.ambient import AmbientSpawner
from simulation.behaviors import UnitBehaviors
from simulation.combat import CombatSystem, Projectile
from simulation.engine import SimulationEngine
from simulation.game_mode import GameMode, WaveConfig, WAVE_CONFIGS
from simulation.loader import load_layout
from simulation.target import SimulationTarget

__all__ = [
    "AmbientSpawner",
    "CombatSystem",
    "GameMode",
    "Projectile",
    "SimulationEngine",
    "SimulationTarget",
    "UnitBehaviors",
    "WaveConfig",
    "WAVE_CONFIGS",
    "load_layout",
]
