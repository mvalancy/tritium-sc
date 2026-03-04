# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Dataclasses for combat matrix telemetry and assertions."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class UnitSnapshot:
    """Point-in-time snapshot of a single unit."""

    timestamp: float
    target_id: str
    name: str
    alliance: str
    asset_type: str
    health: float
    ammo_count: int
    position: tuple[float, float]
    status: str
    fsm_state: str = ""


@dataclass
class Assertion:
    """Result of a single expectation check."""

    name: str
    severity: str  # "critical", "major", "minor"
    expected: Any
    actual: Any
    passed: bool
    message: str = ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "severity": self.severity,
            "expected": str(self.expected),
            "actual": str(self.actual),
            "passed": self.passed,
            "message": self.message,
        }


@dataclass
class BattleMetrics:
    """All telemetry collected from a single battle run."""

    config_id: str

    # Unit counts (from API polls)
    initial_friendly_count: int = 0
    initial_hostile_count: int = 0
    max_total_units: int = 0
    neutral_count_max: int = 0

    # Combat stats (from /api/game/stats)
    total_shots_fired: int = 0
    total_shots_hit: int = 0
    total_eliminations: int = 0
    total_damage_dealt: float = 0.0

    # Ammo pools (computed from config)
    total_ammo_pool: int = 0

    # Game result
    game_result: str = ""  # "victory", "defeat", "timeout"
    final_score: int = 0

    # Per-unit stats (from /api/game/stats)
    unit_stats: list[dict] = field(default_factory=list)

    # Per-unit snapshots over time
    snapshots: list[UnitSnapshot] = field(default_factory=list)

    # WebSocket event counts
    ws_projectile_fired: int = 0
    ws_target_eliminated: int = 0
    ws_game_over: int = 0

    # OpenCV results
    green_blob_detected: bool = False
    red_blob_detected: bool = False
    bright_fx_max: int = 0
    frame_motion_detected: bool = False

    # Audio
    audio_rms: float = 0.0

    # Timing
    battle_duration: float = 0.0

    # Screenshots
    screenshots: list[str] = field(default_factory=list)

    # Errors
    errors: list[str] = field(default_factory=list)

    # Assertions
    assertions: list[Assertion] = field(default_factory=list)

    @property
    def critical_pass(self) -> bool:
        return all(
            a.passed for a in self.assertions if a.severity == "critical"
        )

    @property
    def major_pass(self) -> bool:
        return all(
            a.passed for a in self.assertions if a.severity == "major"
        )

    @property
    def pass_rate(self) -> float:
        if not self.assertions:
            return 0.0
        return sum(1 for a in self.assertions if a.passed) / len(self.assertions)

    def to_dict(self) -> dict:
        return {
            "config_id": self.config_id,
            "initial_friendly_count": self.initial_friendly_count,
            "initial_hostile_count": self.initial_hostile_count,
            "max_total_units": self.max_total_units,
            "neutral_count_max": self.neutral_count_max,
            "total_shots_fired": self.total_shots_fired,
            "total_shots_hit": self.total_shots_hit,
            "total_eliminations": self.total_eliminations,
            "total_damage_dealt": round(self.total_damage_dealt, 2),
            "total_ammo_pool": self.total_ammo_pool,
            "game_result": self.game_result,
            "final_score": self.final_score,
            "ws_projectile_fired": self.ws_projectile_fired,
            "ws_target_eliminated": self.ws_target_eliminated,
            "ws_game_over": self.ws_game_over,
            "green_blob_detected": self.green_blob_detected,
            "red_blob_detected": self.red_blob_detected,
            "bright_fx_max": self.bright_fx_max,
            "frame_motion_detected": self.frame_motion_detected,
            "audio_rms": round(self.audio_rms, 6),
            "battle_duration": round(self.battle_duration, 2),
            "errors": self.errors,
            "pass_rate": round(self.pass_rate, 4),
            "critical_pass": self.critical_pass,
            "assertions": [a.to_dict() for a in self.assertions],
        }
