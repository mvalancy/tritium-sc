# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""TargetHistory — thread-safe position history and movement trail tracking.

Records position updates for every tracked target, maintaining a ring buffer
of up to 1000 positions per target. Derives speed and heading from recent
positions. Automatically prunes targets not updated in 10 minutes.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass


@dataclass(slots=True)
class PositionRecord:
    """A single recorded position with timestamp."""

    x: float
    y: float
    timestamp: float


class TargetHistory:
    """Thread-safe position history recorder for tracked targets.

    Maintains a ring buffer (max 1000 entries) per target, and provides
    trail retrieval, speed estimation, and heading estimation.
    """

    MAX_RECORDS_PER_TARGET = 1000
    PRUNE_TIMEOUT = 600.0  # 10 minutes

    def __init__(self) -> None:
        self._histories: dict[str, deque[PositionRecord]] = {}
        self._lock = threading.Lock()

    def record(
        self,
        target_id: str,
        position: tuple[float, float],
        timestamp: float | None = None,
    ) -> None:
        """Append a position record to the target's history ring buffer.

        Args:
            target_id: Unique target identifier.
            position: (x, y) position tuple.
            timestamp: Monotonic timestamp; defaults to time.monotonic().
        """
        if timestamp is None:
            timestamp = time.monotonic()

        rec = PositionRecord(x=position[0], y=position[1], timestamp=timestamp)

        with self._lock:
            if target_id not in self._histories:
                self._histories[target_id] = deque(maxlen=self.MAX_RECORDS_PER_TARGET)
            self._histories[target_id].append(rec)

    def get_trail(
        self, target_id: str, max_points: int = 100
    ) -> list[tuple[float, float, float]]:
        """Return recent position trail as (x, y, timestamp) tuples.

        Args:
            target_id: Target to query.
            max_points: Maximum number of trail points to return (most recent).

        Returns:
            List of (x, y, timestamp) tuples, oldest first.
        """
        with self._lock:
            history = self._histories.get(target_id)
            if not history:
                return []
            # Take last max_points entries
            entries = list(history)[-max_points:]
        return [(r.x, r.y, r.timestamp) for r in entries]

    def get_speed(self, target_id: str, sample_count: int = 5) -> float:
        """Estimate speed from the last N positions.

        Returns speed in units per second, or 0.0 if insufficient data.
        """
        with self._lock:
            history = self._histories.get(target_id)
            if not history or len(history) < 2:
                return 0.0
            samples = list(history)[-sample_count:]

        if len(samples) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(1, len(samples)):
            dx = samples[i].x - samples[i - 1].x
            dy = samples[i].y - samples[i - 1].y
            total_distance += math.hypot(dx, dy)

        dt = samples[-1].timestamp - samples[0].timestamp
        if dt <= 0:
            return 0.0

        return total_distance / dt

    def get_heading(self, target_id: str, sample_count: int = 3) -> float:
        """Estimate heading from the last N positions.

        Returns heading in degrees (0=north/+Y, 90=east/+X, clockwise),
        or 0.0 if insufficient data.
        """
        with self._lock:
            history = self._histories.get(target_id)
            if not history or len(history) < 2:
                return 0.0
            samples = list(history)[-sample_count:]

        if len(samples) < 2:
            return 0.0

        # Average the displacement vector over the samples
        dx = samples[-1].x - samples[0].x
        dy = samples[-1].y - samples[0].y

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return 0.0

        # atan2 gives angle from +X axis CCW; convert to compass heading
        # (0=north/+Y, clockwise)
        angle_rad = math.atan2(dx, dy)
        heading = math.degrees(angle_rad) % 360
        return heading

    def get_trail_dicts(
        self, target_id: str, max_points: int = 20
    ) -> list[dict]:
        """Return trail as list of dicts for API serialization.

        Returns:
            List of {"x": float, "y": float, "t": float} dicts.
        """
        trail = self.get_trail(target_id, max_points)
        return [{"x": x, "y": y, "t": t} for x, y, t in trail]

    def prune_stale(self) -> None:
        """Remove history for targets not updated in PRUNE_TIMEOUT seconds."""
        now = time.monotonic()
        with self._lock:
            stale = [
                tid
                for tid, history in self._histories.items()
                if not history or (now - history[-1].timestamp) > self.PRUNE_TIMEOUT
            ]
            for tid in stale:
                del self._histories[tid]

    def clear(self, target_id: str | None = None) -> None:
        """Clear history for a specific target, or all targets if None."""
        with self._lock:
            if target_id is None:
                self._histories.clear()
            else:
                self._histories.pop(target_id, None)

    @property
    def tracked_count(self) -> int:
        """Number of targets with history."""
        with self._lock:
            return len(self._histories)
