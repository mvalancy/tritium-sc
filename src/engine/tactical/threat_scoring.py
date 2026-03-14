# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Predictive threat scoring — behavior-based threat probability for targets.

Analyzes target behavior patterns to assign a threat probability score:
  - Loitering detection (stationary in sensitive area)
  - Zone violations (entering restricted geofences)
  - Unusual timing (activity outside normal hours)
  - Movement anomalies (erratic path, speed changes)
  - Appearance frequency (how often a device appears/disappears)

The ThreatScorer runs periodically over all tracked targets and updates
their threat_score field. This feeds into the existing ThreatClassifier
for escalation decisions.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

logger = logging.getLogger("threat-scoring")

# Score weights for different behavioral indicators
LOITER_WEIGHT = 0.25
ZONE_VIOLATION_WEIGHT = 0.30
TIMING_WEIGHT = 0.15
MOVEMENT_ANOMALY_WEIGHT = 0.15
APPEARANCE_WEIGHT = 0.15

# Loitering thresholds
LOITER_MIN_DURATION = 300.0  # 5 minutes stationary
LOITER_RADIUS = 5.0  # meters — movement within this radius = stationary

# Timing: normal hours (local time)
NORMAL_HOURS_START = 6   # 6 AM
NORMAL_HOURS_END = 22    # 10 PM

# Movement anomaly thresholds
ERRATIC_HEADING_CHANGES = 8  # heading reversals in short period
SPEED_ANOMALY_THRESHOLD = 3.0  # sudden speed change ratio

# Score decay per evaluation cycle
SCORE_DECAY = 0.95


@dataclass
class BehaviorProfile:
    """Behavioral profile for a tracked target."""
    target_id: str
    threat_score: float = 0.0

    # Loitering state
    stationary_since: float = 0.0
    stationary_position: tuple[float, float] = (0.0, 0.0)
    loiter_score: float = 0.0

    # Zone violation count
    zone_violations: int = 0
    zone_score: float = 0.0

    # Timing score
    timing_score: float = 0.0
    off_hours_sightings: int = 0
    total_sightings: int = 0

    # Movement analysis
    heading_history: list[float] = field(default_factory=list)
    speed_history: list[float] = field(default_factory=list)
    movement_score: float = 0.0

    # Appearance pattern
    appearance_count: int = 0
    disappearance_count: int = 0
    appearance_score: float = 0.0

    last_position: tuple[float, float] = (0.0, 0.0)
    last_heading: float = 0.0
    last_speed: float = 0.0
    last_updated: float = 0.0

    def compute_threat_score(self) -> float:
        """Compute weighted threat score from all behavioral indicators."""
        self.threat_score = min(1.0, max(0.0, (
            self.loiter_score * LOITER_WEIGHT
            + self.zone_score * ZONE_VIOLATION_WEIGHT
            + self.timing_score * TIMING_WEIGHT
            + self.movement_score * MOVEMENT_ANOMALY_WEIGHT
            + self.appearance_score * APPEARANCE_WEIGHT
        )))
        return self.threat_score

    def to_dict(self) -> dict[str, Any]:
        return {
            "target_id": self.target_id,
            "threat_score": round(self.threat_score, 3),
            "loiter_score": round(self.loiter_score, 3),
            "zone_score": round(self.zone_score, 3),
            "timing_score": round(self.timing_score, 3),
            "movement_score": round(self.movement_score, 3),
            "appearance_score": round(self.appearance_score, 3),
            "zone_violations": self.zone_violations,
            "off_hours_ratio": (
                round(self.off_hours_sightings / max(1, self.total_sightings), 2)
            ),
        }


class ThreatScorer:
    """Predictive threat scoring engine for tracked targets.

    Call `evaluate()` periodically with the current target list to update
    threat scores. The scorer maintains behavioral profiles for each target
    and computes a composite threat probability.

    Parameters
    ----------
    geofence_checker:
        Optional callable(target_id, position) -> bool that returns True
        if the position violates a geofence.
    on_score_update:
        Optional callback(target_id, score, profile_dict) invoked when
        a target's score changes significantly.
    """

    def __init__(
        self,
        geofence_checker: Optional[Callable[[str, tuple[float, float]], bool]] = None,
        on_score_update: Optional[Callable[[str, float, dict], None]] = None,
    ) -> None:
        self._lock = threading.Lock()
        self._profiles: dict[str, BehaviorProfile] = {}
        self._geofence_checker = geofence_checker
        self._on_score_update = on_score_update

    def evaluate(self, targets: list[Any]) -> dict[str, float]:
        """Evaluate all targets and return updated threat scores.

        Args:
            targets: List of TrackedTarget objects (or dicts with target_id,
                     position, heading, speed, source, last_seen).

        Returns:
            Dict mapping target_id -> threat_score.
        """
        now = time.monotonic()
        scores: dict[str, float] = {}

        with self._lock:
            active_ids = set()

            for target in targets:
                # Extract target data
                if hasattr(target, "target_id"):
                    tid = target.target_id
                    pos = target.position
                    heading = target.heading
                    speed = target.speed
                    source = target.source
                else:
                    tid = target.get("target_id", "")
                    pos_data = target.get("position", (0.0, 0.0))
                    if isinstance(pos_data, dict):
                        pos = (pos_data.get("x", 0.0), pos_data.get("y", 0.0))
                    else:
                        pos = pos_data
                    heading = target.get("heading", 0.0)
                    speed = target.get("speed", 0.0)
                    source = target.get("source", "")

                if not tid:
                    continue
                active_ids.add(tid)

                # Skip friendly simulation units
                alliance = getattr(target, "alliance", None) or (
                    target.get("alliance", "") if isinstance(target, dict) else ""
                )
                if alliance == "friendly":
                    scores[tid] = 0.0
                    continue

                # Get or create profile
                if tid not in self._profiles:
                    self._profiles[tid] = BehaviorProfile(
                        target_id=tid,
                        last_position=pos,
                        last_updated=now,
                    )

                profile = self._profiles[tid]
                old_score = profile.threat_score

                # Update behavioral indicators
                self._update_loitering(profile, pos, now)
                self._update_zone_violations(profile, tid, pos)
                self._update_timing(profile)
                self._update_movement(profile, heading, speed)

                # Compute composite score
                score = profile.compute_threat_score()
                scores[tid] = score

                # Update position/time
                profile.last_position = pos
                profile.last_heading = heading
                profile.last_speed = speed
                profile.last_updated = now

                # Notify on significant change
                if self._on_score_update and abs(score - old_score) > 0.1:
                    try:
                        self._on_score_update(tid, score, profile.to_dict())
                    except Exception:
                        pass

            # Decay scores for targets no longer visible
            for tid in list(self._profiles.keys()):
                if tid not in active_ids:
                    profile = self._profiles[tid]
                    profile.loiter_score *= SCORE_DECAY
                    profile.zone_score *= SCORE_DECAY
                    profile.movement_score *= SCORE_DECAY
                    profile.compute_threat_score()
                    if profile.threat_score < 0.01:
                        del self._profiles[tid]

        return scores

    def _update_loitering(
        self, profile: BehaviorProfile, pos: tuple[float, float], now: float
    ) -> None:
        """Detect if target is loitering (stationary too long)."""
        dx = pos[0] - profile.stationary_position[0]
        dy = pos[1] - profile.stationary_position[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < LOITER_RADIUS:
            # Still within loiter radius
            if profile.stationary_since == 0.0:
                profile.stationary_since = now
            duration = now - profile.stationary_since
            if duration > LOITER_MIN_DURATION:
                # Loitering detected — score increases with duration
                profile.loiter_score = min(1.0, duration / (LOITER_MIN_DURATION * 4))
        else:
            # Moved — reset loiter state
            profile.stationary_since = now
            profile.stationary_position = pos
            profile.loiter_score *= 0.8  # Gradual decay

    def _update_zone_violations(
        self, profile: BehaviorProfile, tid: str, pos: tuple[float, float]
    ) -> None:
        """Check if target is violating any geofences."""
        if self._geofence_checker is None:
            return

        try:
            violated = self._geofence_checker(tid, pos)
            if violated:
                profile.zone_violations += 1
                profile.zone_score = min(1.0, profile.zone_violations * 0.2)
        except Exception:
            pass

    def _update_timing(self, profile: BehaviorProfile) -> None:
        """Score based on time-of-day activity patterns."""
        import datetime
        hour = datetime.datetime.now().hour
        profile.total_sightings += 1

        if hour < NORMAL_HOURS_START or hour >= NORMAL_HOURS_END:
            profile.off_hours_sightings += 1

        if profile.total_sightings > 5:
            off_ratio = profile.off_hours_sightings / profile.total_sightings
            profile.timing_score = min(1.0, off_ratio * 1.5)

    def _update_movement(
        self, profile: BehaviorProfile, heading: float, speed: float
    ) -> None:
        """Detect erratic movement patterns."""
        # Track heading changes
        profile.heading_history.append(heading)
        if len(profile.heading_history) > 20:
            profile.heading_history = profile.heading_history[-20:]

        profile.speed_history.append(speed)
        if len(profile.speed_history) > 20:
            profile.speed_history = profile.speed_history[-20:]

        # Count heading reversals (sign changes in heading delta)
        reversals = 0
        if len(profile.heading_history) >= 3:
            for i in range(2, len(profile.heading_history)):
                d1 = profile.heading_history[i - 1] - profile.heading_history[i - 2]
                d2 = profile.heading_history[i] - profile.heading_history[i - 1]
                if d1 * d2 < 0 and abs(d1) > 10 and abs(d2) > 10:
                    reversals += 1

        heading_anomaly = min(1.0, reversals / ERRATIC_HEADING_CHANGES)

        # Speed anomaly — sudden changes
        speed_anomaly = 0.0
        if len(profile.speed_history) >= 2:
            prev = max(0.1, profile.speed_history[-2])
            curr = max(0.1, profile.speed_history[-1])
            ratio = max(curr / prev, prev / curr)
            if ratio > SPEED_ANOMALY_THRESHOLD:
                speed_anomaly = min(1.0, (ratio - 1.0) / 5.0)

        profile.movement_score = max(heading_anomaly, speed_anomaly)

    def get_profile(self, target_id: str) -> Optional[dict[str, Any]]:
        """Get the behavioral profile for a specific target."""
        with self._lock:
            profile = self._profiles.get(target_id)
            return profile.to_dict() if profile else None

    def get_all_profiles(self, min_score: float = 0.0) -> list[dict[str, Any]]:
        """Get all behavioral profiles, optionally filtered by minimum score."""
        with self._lock:
            return [
                p.to_dict()
                for p in sorted(
                    self._profiles.values(),
                    key=lambda p: p.threat_score,
                    reverse=True,
                )
                if p.threat_score >= min_score
            ]

    def get_score(self, target_id: str) -> float:
        """Get the current threat score for a target."""
        with self._lock:
            profile = self._profiles.get(target_id)
            return profile.threat_score if profile else 0.0

    def get_status(self) -> dict[str, Any]:
        """Return scorer status for API."""
        with self._lock:
            total = len(self._profiles)
            high_threat = sum(1 for p in self._profiles.values() if p.threat_score >= 0.7)
            medium_threat = sum(
                1 for p in self._profiles.values() if 0.3 <= p.threat_score < 0.7
            )
        return {
            "total_profiles": total,
            "high_threat_count": high_threat,
            "medium_threat_count": medium_threat,
            "has_geofence_checker": self._geofence_checker is not None,
        }
