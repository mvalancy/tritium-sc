# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Geofence engine — polygon-based zone monitoring with enter/exit detection.

Defines GeoZone (polygon regions on the tactical map) and GeofenceEngine
which tracks per-target zone membership and detects enter/exit transitions.
Uses ray-casting for point-in-polygon tests.

Events published to EventBus:
    geofence:enter  — target entered a zone
    geofence:exit   — target exited a zone
"""

from __future__ import annotations

import logging
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..comms.event_bus import EventBus

logger = logging.getLogger("amy.geofence")


@dataclass
class GeoZone:
    """A polygon zone on the tactical map."""

    zone_id: str
    name: str
    polygon: list[tuple[float, float]]  # ordered vertices
    zone_type: str = "monitored"  # "restricted", "monitored", "safe"
    alert_on_enter: bool = True
    alert_on_exit: bool = True
    enabled: bool = True
    created_at: float = field(default_factory=time.time)

    def to_dict(self) -> dict:
        return {
            "zone_id": self.zone_id,
            "name": self.name,
            "polygon": [list(p) for p in self.polygon],
            "zone_type": self.zone_type,
            "alert_on_enter": self.alert_on_enter,
            "alert_on_exit": self.alert_on_exit,
            "enabled": self.enabled,
            "created_at": self.created_at,
        }


@dataclass
class GeoEvent:
    """A geofence transition event."""

    event_id: str
    event_type: str  # "enter", "exit", "inside"
    target_id: str
    zone_id: str
    zone_name: str
    zone_type: str
    position: tuple[float, float]
    timestamp: float

    def to_dict(self) -> dict:
        return {
            "event_id": self.event_id,
            "event_type": self.event_type,
            "target_id": self.target_id,
            "zone_id": self.zone_id,
            "zone_name": self.zone_name,
            "zone_type": self.zone_type,
            "position": list(self.position),
            "timestamp": self.timestamp,
        }


def point_in_polygon(
    x: float, y: float, polygon: list[tuple[float, float]]
) -> bool:
    """Ray-casting algorithm for point-in-polygon test.

    Casts a ray from (x, y) in the +x direction and counts edge crossings.
    An odd count means the point is inside the polygon.
    """
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        # Check if the ray crosses this edge
        if (yi > y) != (yj > y):
            # Compute x-coordinate of intersection
            x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_intersect:
                inside = not inside
        j = i

    return inside


class GeofenceEngine:
    """Tracks targets against polygon zones and detects enter/exit transitions.

    Thread-safe. Maintains per-target zone membership state so that
    transitions (enter/exit) are detected on each check() call.
    """

    def __init__(self, event_bus: EventBus | None = None) -> None:
        self._event_bus = event_bus
        self._lock = threading.Lock()
        self._zones: dict[str, GeoZone] = {}
        # target_id -> set of zone_ids the target is currently inside
        self._target_zones: dict[str, set[str]] = {}
        # chronological event log (bounded)
        self._events: list[GeoEvent] = []
        self._max_events = 10000

    # ------------------------------------------------------------------
    # Zone CRUD
    # ------------------------------------------------------------------

    def add_zone(self, zone: GeoZone) -> GeoZone:
        """Add a zone. Returns the zone."""
        with self._lock:
            self._zones[zone.zone_id] = zone
        logger.info(f"Geofence zone added: {zone.name} ({zone.zone_id})")
        return zone

    def remove_zone(self, zone_id: str) -> bool:
        """Remove a zone by ID. Returns True if found and removed."""
        with self._lock:
            if zone_id not in self._zones:
                return False
            del self._zones[zone_id]
            # Clean up target state references to this zone
            for target_id in list(self._target_zones):
                self._target_zones[target_id].discard(zone_id)
                if not self._target_zones[target_id]:
                    del self._target_zones[target_id]
        logger.info(f"Geofence zone removed: {zone_id}")
        return True

    def get_zone(self, zone_id: str) -> GeoZone | None:
        """Get a zone by ID."""
        with self._lock:
            return self._zones.get(zone_id)

    def list_zones(self) -> list[GeoZone]:
        """Return all zones."""
        with self._lock:
            return list(self._zones.values())

    # ------------------------------------------------------------------
    # Target checking
    # ------------------------------------------------------------------

    def check(
        self, target_id: str, position: tuple[float, float]
    ) -> list[GeoEvent]:
        """Check a target position against all zones.

        Returns a list of GeoEvent for any transitions (enter/exit) and
        for zones where the target is currently inside.
        """
        now = time.time()
        events: list[GeoEvent] = []

        with self._lock:
            prev_zones = self._target_zones.get(target_id, set()).copy()
            current_zones: set[str] = set()

            for zone_id, zone in self._zones.items():
                if not zone.enabled:
                    continue

                inside = point_in_polygon(position[0], position[1], zone.polygon)

                if inside:
                    current_zones.add(zone_id)

                    if zone_id not in prev_zones:
                        # Enter transition
                        ev = GeoEvent(
                            event_id=uuid.uuid4().hex[:12],
                            event_type="enter",
                            target_id=target_id,
                            zone_id=zone_id,
                            zone_name=zone.name,
                            zone_type=zone.zone_type,
                            position=position,
                            timestamp=now,
                        )
                        events.append(ev)
                        self._record_event(ev)

                        if zone.alert_on_enter and self._event_bus is not None:
                            self._event_bus.publish("geofence:enter", ev.to_dict())
                    else:
                        # Still inside
                        events.append(
                            GeoEvent(
                                event_id=uuid.uuid4().hex[:12],
                                event_type="inside",
                                target_id=target_id,
                                zone_id=zone_id,
                                zone_name=zone.name,
                                zone_type=zone.zone_type,
                                position=position,
                                timestamp=now,
                            )
                        )

            # Check for exits (was in a zone, no longer)
            exited_zones = prev_zones - current_zones
            for zone_id in exited_zones:
                zone = self._zones.get(zone_id)
                if zone is None:
                    continue
                ev = GeoEvent(
                    event_id=uuid.uuid4().hex[:12],
                    event_type="exit",
                    target_id=target_id,
                    zone_id=zone_id,
                    zone_name=zone.name,
                    zone_type=zone.zone_type,
                    position=position,
                    timestamp=now,
                )
                events.append(ev)
                self._record_event(ev)

                if zone.alert_on_exit and self._event_bus is not None:
                    self._event_bus.publish("geofence:exit", ev.to_dict())

            # Update target state
            if current_zones:
                self._target_zones[target_id] = current_zones
            else:
                self._target_zones.pop(target_id, None)

        return events

    def get_target_zones(self, target_id: str) -> set[str]:
        """Get the set of zone IDs a target is currently inside."""
        with self._lock:
            return self._target_zones.get(target_id, set()).copy()

    # ------------------------------------------------------------------
    # Event log
    # ------------------------------------------------------------------

    def get_events(
        self,
        limit: int = 100,
        zone_id: str | None = None,
        target_id: str | None = None,
        event_type: str | None = None,
    ) -> list[GeoEvent]:
        """Get recent geofence events, optionally filtered."""
        with self._lock:
            filtered = self._events
            if zone_id is not None:
                filtered = [e for e in filtered if e.zone_id == zone_id]
            if target_id is not None:
                filtered = [e for e in filtered if e.target_id == target_id]
            if event_type is not None:
                filtered = [e for e in filtered if e.event_type == event_type]
            # Return most recent first
            return list(reversed(filtered[-limit:]))

    def _record_event(self, event: GeoEvent) -> None:
        """Append event to internal log (caller holds lock)."""
        self._events.append(event)
        if len(self._events) > self._max_events:
            self._events = self._events[-self._max_events:]
