# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Patrol pattern system — autonomous asset movement along waypoint routes.

Defines PatrolRoute (waypoint sequence) and PatrolManager which advances
assigned assets along their routes each tick, publishing position telemetry
via EventBus.

Events published to EventBus:
    patrol:position   — asset position updated along route
    patrol:waypoint   — asset arrived at a waypoint
    patrol:complete   — asset completed a non-looping route
"""

from __future__ import annotations

import logging
import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..comms.event_bus import EventBus

logger = logging.getLogger("amy.patrol")


@dataclass
class PatrolRoute:
    """A named sequence of waypoints that assets can follow."""

    route_id: str
    name: str
    waypoints: list[tuple[float, float]]  # ordered (x, y) positions
    loop: bool = True
    speed: float = 1.0  # units per second

    def to_dict(self) -> dict:
        return {
            "route_id": self.route_id,
            "name": self.name,
            "waypoints": [list(w) for w in self.waypoints],
            "loop": self.loop,
            "speed": self.speed,
        }


@dataclass
class PatrolAssignment:
    """Tracks an asset's progress along a patrol route."""

    asset_id: str
    route_id: str
    waypoint_index: int = 0
    position: tuple[float, float] = (0.0, 0.0)
    started_at: float = field(default_factory=time.time)
    completed: bool = False

    def to_dict(self) -> dict:
        return {
            "asset_id": self.asset_id,
            "route_id": self.route_id,
            "waypoint_index": self.waypoint_index,
            "position": list(self.position),
            "started_at": self.started_at,
            "completed": self.completed,
        }


class PatrolManager:
    """Manages patrol routes and advances assigned assets along them.

    Thread-safe. Call tick(dt) from a simulation loop to advance all
    patrolling assets toward their next waypoint.
    """

    ARRIVAL_THRESHOLD = 0.5  # distance to consider "arrived" at waypoint

    def __init__(self, event_bus: EventBus | None = None) -> None:
        self._event_bus = event_bus
        self._lock = threading.Lock()
        self._routes: dict[str, PatrolRoute] = {}
        self._assignments: dict[str, PatrolAssignment] = {}  # asset_id -> assignment

    # ------------------------------------------------------------------
    # Route CRUD
    # ------------------------------------------------------------------

    def create_route(
        self,
        name: str,
        waypoints: list[tuple[float, float]],
        loop: bool = True,
        speed: float = 1.0,
    ) -> str:
        """Create a patrol route. Returns the route_id."""
        route_id = uuid.uuid4().hex[:12]
        route = PatrolRoute(
            route_id=route_id,
            name=name,
            waypoints=waypoints,
            loop=loop,
            speed=speed,
        )
        with self._lock:
            self._routes[route_id] = route
        logger.info(f"Patrol route created: {name} ({route_id}), {len(waypoints)} waypoints")
        return route_id

    def get_route(self, route_id: str) -> PatrolRoute | None:
        """Get a route by ID."""
        with self._lock:
            return self._routes.get(route_id)

    def list_routes(self) -> list[PatrolRoute]:
        """Return all patrol routes."""
        with self._lock:
            return list(self._routes.values())

    def remove_route(self, route_id: str) -> bool:
        """Remove a route. Unassigns any assets on it. Returns True if found."""
        with self._lock:
            if route_id not in self._routes:
                return False
            del self._routes[route_id]
            # Unassign any assets on this route
            to_remove = [
                aid for aid, a in self._assignments.items()
                if a.route_id == route_id
            ]
            for aid in to_remove:
                del self._assignments[aid]
        logger.info(f"Patrol route removed: {route_id}")
        return True

    # ------------------------------------------------------------------
    # Assignment
    # ------------------------------------------------------------------

    def assign_asset(self, route_id: str, asset_id: str) -> bool:
        """Assign an asset to patrol a route. Returns True on success."""
        with self._lock:
            route = self._routes.get(route_id)
            if route is None:
                return False
            if len(route.waypoints) == 0:
                return False

            # Start at first waypoint, target the second
            start_idx = 1 if len(route.waypoints) > 1 else 0
            self._assignments[asset_id] = PatrolAssignment(
                asset_id=asset_id,
                route_id=route_id,
                waypoint_index=start_idx,
                position=route.waypoints[0],
            )
        logger.info(f"Asset {asset_id} assigned to patrol route {route_id}")
        return True

    def unassign_asset(self, asset_id: str) -> bool:
        """Stop an asset's patrol. Returns True if the asset was patrolling."""
        with self._lock:
            if asset_id not in self._assignments:
                return False
            del self._assignments[asset_id]
        logger.info(f"Asset {asset_id} unassigned from patrol")
        return True

    def get_active_patrols(self) -> list[PatrolAssignment]:
        """Return all active patrol assignments."""
        with self._lock:
            return [a for a in self._assignments.values() if not a.completed]

    def get_assignment(self, asset_id: str) -> PatrolAssignment | None:
        """Get a specific asset's patrol assignment."""
        with self._lock:
            return self._assignments.get(asset_id)

    # ------------------------------------------------------------------
    # Tick — advance all assets
    # ------------------------------------------------------------------

    def tick(self, dt: float) -> None:
        """Advance all patrolling assets toward their next waypoint.

        Args:
            dt: time delta in seconds since last tick
        """
        with self._lock:
            for asset_id, assignment in list(self._assignments.items()):
                if assignment.completed:
                    continue

                route = self._routes.get(assignment.route_id)
                if route is None or len(route.waypoints) == 0:
                    continue

                self._advance_asset(assignment, route, dt)

    def _advance_asset(
        self, assignment: PatrolAssignment, route: PatrolRoute, dt: float
    ) -> None:
        """Move an asset toward its current target waypoint (caller holds lock)."""
        target_idx = assignment.waypoint_index
        if target_idx >= len(route.waypoints):
            assignment.completed = True
            return

        target = route.waypoints[target_idx]
        px, py = assignment.position
        tx, ty = target

        dx = tx - px
        dy = ty - py
        dist = math.sqrt(dx * dx + dy * dy)

        move_dist = route.speed * dt

        if dist <= max(move_dist, self.ARRIVAL_THRESHOLD):
            # Arrived at waypoint
            assignment.position = target
            self._publish("patrol:waypoint", {
                "asset_id": assignment.asset_id,
                "route_id": assignment.route_id,
                "waypoint_index": target_idx,
                "position": list(target),
            })

            # Advance to next waypoint
            next_idx = target_idx + 1
            if next_idx >= len(route.waypoints):
                if route.loop:
                    next_idx = 0
                else:
                    assignment.completed = True
                    self._publish("patrol:complete", {
                        "asset_id": assignment.asset_id,
                        "route_id": assignment.route_id,
                    })
                    return

            assignment.waypoint_index = next_idx
        else:
            # Move toward waypoint
            ratio = move_dist / dist
            new_x = px + dx * ratio
            new_y = py + dy * ratio
            assignment.position = (new_x, new_y)

        # Publish position update
        self._publish("patrol:position", {
            "asset_id": assignment.asset_id,
            "route_id": assignment.route_id,
            "position": list(assignment.position),
            "waypoint_index": assignment.waypoint_index,
        })

    def _publish(self, event_type: str, data: dict) -> None:
        """Publish event to EventBus if available (caller holds lock)."""
        if self._event_bus is not None:
            self._event_bus.publish(event_type, data)
