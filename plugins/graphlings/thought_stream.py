# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""ThoughtCollector — observable thought stream for graphling agents.

Collects thoughts from think cycles and publishes them to the EventBus
for SSE streaming. Provides per-graphling status and recent thought history.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Optional


class ThoughtCollector:
    """Thread-safe collector for graphling thoughts.

    Records thoughts with metadata and optionally publishes them to an
    EventBus for real-time streaming via SSE.
    """

    def __init__(
        self,
        max_history: int = 100,
        event_bus: Any = None,
    ) -> None:
        self._max_history = max_history
        self._event_bus = event_bus
        self._lock = threading.Lock()
        self._thoughts: deque[dict[str, Any]] = deque(maxlen=max_history)

    def record(
        self,
        soul_id: str,
        thought: str,
        action: str = "",
        emotion: str = "",
        layer: int = 0,
        model: str = "",
    ) -> None:
        """Record a thought from a graphling think cycle."""
        entry = {
            "soul_id": soul_id,
            "thought": thought,
            "action": action,
            "emotion": emotion,
            "layer": layer,
            "model": model,
            "timestamp": time.time(),
        }

        with self._lock:
            self._thoughts.append(entry)

        if self._event_bus:
            self._event_bus.publish("graphling_thought", data=entry)

    def get_recent(
        self,
        soul_id: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> list[dict[str, Any]]:
        """Return recent thoughts, optionally filtered by soul_id."""
        with self._lock:
            entries = list(self._thoughts)

        if soul_id is not None:
            entries = [e for e in entries if e["soul_id"] == soul_id]

        if limit is not None:
            entries = entries[-limit:]

        return entries

    def build_status(
        self,
        soul_id: str,
        deployed_info: Optional[dict[str, Any]],
        compute_stats: dict[str, Any],
    ) -> dict[str, Any]:
        """Build a rich status dict for a specific graphling."""
        recent = self.get_recent(soul_id=soul_id)

        if deployed_info is not None:
            return {
                "soul_id": soul_id,
                "deployed": True,
                "role_name": deployed_info.get("role_name", ""),
                "position": deployed_info.get("position"),
                "recent_thoughts": recent,
                "compute_stats": compute_stats,
            }
        else:
            return {
                "soul_id": soul_id,
                "deployed": False,
                "recent_thoughts": [],
                "compute_stats": compute_stats,
            }
