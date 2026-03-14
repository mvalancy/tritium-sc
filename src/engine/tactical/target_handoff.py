# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Target Handoff Tracking — sensor-to-sensor coverage transitions.

When a target moves from one camera/sensor's field of view to another's,
logs the handoff event. Critical for Re-ID — "person left camera A's FOV,
appeared on camera B 30 seconds later."

Handoff events are used by:
  - TargetCorrelator for cross-sensor identity linking
  - Dossier enrichment for movement trail reconstruction
  - Anomaly detection for unexpected coverage gaps

The HandoffTracker monitors target visibility per sensor and detects
exit/entry transitions. It maintains a short-term memory of recent
departures to match with arrivals at other sensors.
"""
from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

logger = logging.getLogger("target_handoff")

# Maximum time between departure and arrival to consider a handoff (seconds)
MAX_HANDOFF_GAP = 120.0

# How long to keep departed targets in memory for potential matching
DEPARTURE_MEMORY_TTL = 180.0

# Maximum handoff events to retain
MAX_HANDOFF_HISTORY = 5000


@dataclass(slots=True)
class HandoffEvent:
    """A target handoff from one sensor to another."""

    handoff_id: str = ""
    target_id: str = ""
    from_sensor: str = ""
    to_sensor: str = ""
    departure_time: float = 0.0
    arrival_time: float = 0.0
    gap_seconds: float = 0.0
    confidence: float = 0.0
    target_type: str = ""       # person, vehicle, device, etc.
    from_position: tuple[float, float] = (0.0, 0.0)
    to_position: tuple[float, float] = (0.0, 0.0)

    def to_dict(self) -> dict[str, Any]:
        return {
            "handoff_id": self.handoff_id,
            "target_id": self.target_id,
            "from_sensor": self.from_sensor,
            "to_sensor": self.to_sensor,
            "departure_time": self.departure_time,
            "arrival_time": self.arrival_time,
            "gap_seconds": round(self.gap_seconds, 2),
            "confidence": round(self.confidence, 3),
            "target_type": self.target_type,
            "from_position": list(self.from_position),
            "to_position": list(self.to_position),
        }


@dataclass
class _DepartureRecord:
    """Internal record of a target departing a sensor's coverage."""

    target_id: str
    sensor_id: str
    departure_time: float
    last_position: tuple[float, float]
    target_type: str
    last_rssi: float = 0.0


@dataclass
class _SensorTrack:
    """Tracks which targets are currently visible to a sensor."""

    sensor_id: str
    visible_targets: dict[str, float] = field(default_factory=dict)  # target_id -> last_seen
    target_positions: dict[str, tuple[float, float]] = field(default_factory=dict)
    target_types: dict[str, str] = field(default_factory=dict)


class HandoffTracker:
    """Monitors target visibility across sensors and detects handoffs.

    Call `update_visibility()` whenever a sensor reports seeing a target.
    The tracker detects when a target disappears from one sensor and
    appears at another within the handoff window.

    Parameters
    ----------
    max_gap:
        Maximum seconds between departure and arrival to count as a handoff.
    visibility_timeout:
        Seconds without an update before a target is considered departed.
    on_handoff:
        Optional callback invoked with each HandoffEvent.
    """

    def __init__(
        self,
        max_gap: float = MAX_HANDOFF_GAP,
        visibility_timeout: float = 10.0,
        on_handoff: Optional[Callable[[HandoffEvent], None]] = None,
    ) -> None:
        self._max_gap = max_gap
        self._visibility_timeout = visibility_timeout
        self._on_handoff = on_handoff

        self._sensors: dict[str, _SensorTrack] = {}
        self._departures: deque[_DepartureRecord] = deque(maxlen=1000)
        self._handoffs: deque[HandoffEvent] = deque(maxlen=MAX_HANDOFF_HISTORY)
        self._lock = threading.Lock()
        self._handoff_counter = 0

    def update_visibility(
        self,
        sensor_id: str,
        target_id: str,
        position: tuple[float, float] = (0.0, 0.0),
        target_type: str = "",
        timestamp: Optional[float] = None,
    ) -> Optional[HandoffEvent]:
        """Report that a sensor currently sees a target.

        Call this periodically for each target visible to each sensor.
        Returns a HandoffEvent if this arrival matches a recent departure
        from another sensor.

        Args:
            sensor_id: Unique sensor/camera identifier.
            target_id: Unique target identifier.
            position: Target position (x, y).
            target_type: Target classification (person, vehicle, etc.).
            timestamp: Event timestamp (defaults to time.monotonic()).

        Returns:
            HandoffEvent if a handoff was detected, else None.
        """
        now = timestamp or time.monotonic()

        with self._lock:
            # Ensure sensor track exists
            if sensor_id not in self._sensors:
                self._sensors[sensor_id] = _SensorTrack(sensor_id=sensor_id)

            track = self._sensors[sensor_id]
            was_visible = target_id in track.visible_targets

            # Update visibility
            track.visible_targets[target_id] = now
            track.target_positions[target_id] = position
            if target_type:
                track.target_types[target_id] = target_type

            # If target was NOT previously visible at this sensor,
            # check for a matching departure from another sensor
            handoff = None
            if not was_visible:
                handoff = self._match_departure(
                    sensor_id=sensor_id,
                    target_id=target_id,
                    arrival_time=now,
                    position=position,
                    target_type=target_type or track.target_types.get(target_id, ""),
                )

        if handoff and self._on_handoff:
            try:
                self._on_handoff(handoff)
            except Exception as exc:
                logger.debug("Handoff callback failed: %s", exc)

        return handoff

    def check_departures(self, timestamp: Optional[float] = None) -> list[str]:
        """Check all sensors for targets that have timed out (departed).

        Call this periodically (e.g. every second) to detect departures.

        Returns:
            List of target_ids that departed sensors.
        """
        now = timestamp or time.monotonic()
        departed: list[str] = []

        with self._lock:
            for track in self._sensors.values():
                timed_out: list[str] = []
                for tid, last_seen in track.visible_targets.items():
                    if (now - last_seen) >= self._visibility_timeout:
                        timed_out.append(tid)

                for tid in timed_out:
                    pos = track.target_positions.pop(tid, (0.0, 0.0))
                    ttype = track.target_types.pop(tid, "")
                    del track.visible_targets[tid]

                    # Record departure for future matching
                    self._departures.append(_DepartureRecord(
                        target_id=tid,
                        sensor_id=track.sensor_id,
                        departure_time=now,
                        last_position=pos,
                        target_type=ttype,
                    ))
                    departed.append(tid)

            # Prune old departures
            cutoff = now - DEPARTURE_MEMORY_TTL
            while self._departures and self._departures[0].departure_time < cutoff:
                self._departures.popleft()

        return departed

    def _match_departure(
        self,
        sensor_id: str,
        target_id: str,
        arrival_time: float,
        position: tuple[float, float],
        target_type: str,
    ) -> Optional[HandoffEvent]:
        """Try to match a target's arrival with a recent departure.

        Must be called while holding self._lock.
        """
        best_match: Optional[_DepartureRecord] = None
        best_gap = self._max_gap

        for dep in reversed(self._departures):
            # Same target, different sensor
            if dep.target_id != target_id:
                continue
            if dep.sensor_id == sensor_id:
                continue

            gap = arrival_time - dep.departure_time
            if gap < 0:
                continue
            if gap > self._max_gap:
                continue
            if gap < best_gap:
                best_match = dep
                best_gap = gap

        if best_match is None:
            return None

        # Compute confidence based on gap duration
        # Short gap = high confidence, long gap = lower confidence
        confidence = max(0.1, 1.0 - (best_gap / self._max_gap))

        self._handoff_counter += 1
        handoff = HandoffEvent(
            handoff_id=f"handoff_{self._handoff_counter}",
            target_id=target_id,
            from_sensor=best_match.sensor_id,
            to_sensor=sensor_id,
            departure_time=best_match.departure_time,
            arrival_time=arrival_time,
            gap_seconds=best_gap,
            confidence=confidence,
            target_type=target_type or best_match.target_type,
            from_position=best_match.last_position,
            to_position=position,
        )

        self._handoffs.append(handoff)
        logger.info(
            "Handoff detected: %s from %s -> %s (gap=%.1fs, conf=%.2f)",
            target_id, best_match.sensor_id, sensor_id,
            best_gap, confidence,
        )

        return handoff

    def get_handoffs(
        self,
        target_id: Optional[str] = None,
        limit: int = 100,
    ) -> list[dict[str, Any]]:
        """Get recent handoff events, optionally filtered by target.

        Args:
            target_id: Filter to only this target's handoffs.
            limit: Maximum number of events to return.

        Returns:
            List of handoff event dicts, newest first.
        """
        with self._lock:
            events = list(self._handoffs)

        if target_id:
            events = [e for e in events if e.target_id == target_id]

        events = sorted(events, key=lambda e: e.arrival_time, reverse=True)
        return [e.to_dict() for e in events[:limit]]

    def get_sensor_coverage(self) -> dict[str, Any]:
        """Get current sensor coverage status.

        Returns:
            Dict mapping sensor_id to currently visible target count.
        """
        with self._lock:
            return {
                sid: {
                    "visible_count": len(track.visible_targets),
                    "target_ids": sorted(track.visible_targets.keys()),
                }
                for sid, track in self._sensors.items()
            }

    def get_status(self) -> dict[str, Any]:
        """Return tracker status for API response."""
        with self._lock:
            return {
                "sensor_count": len(self._sensors),
                "pending_departures": len(self._departures),
                "total_handoffs": len(self._handoffs),
                "max_gap_seconds": self._max_gap,
                "visibility_timeout": self._visibility_timeout,
                "sensors": {
                    sid: len(track.visible_targets)
                    for sid, track in self._sensors.items()
                },
            }


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------

_tracker: Optional[HandoffTracker] = None


def get_handoff_tracker(
    max_gap: float = MAX_HANDOFF_GAP,
    visibility_timeout: float = 10.0,
    on_handoff: Optional[Callable[[HandoffEvent], None]] = None,
) -> HandoffTracker:
    """Get or create the singleton HandoffTracker."""
    global _tracker
    if _tracker is None:
        _tracker = HandoffTracker(
            max_gap=max_gap,
            visibility_timeout=visibility_timeout,
            on_handoff=on_handoff,
        )
    return _tracker
