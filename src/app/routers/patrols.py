# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Patrol API — route CRUD and asset assignment for autonomous patrol.

Endpoints:
    POST   /api/patrols/routes    — create a patrol route
    GET    /api/patrols/routes    — list all patrol routes
    POST   /api/patrols/assign    — assign asset to a route
    POST   /api/patrols/unassign  — stop an asset's patrol
    GET    /api/patrols/active    — list active patrol assignments
"""

from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from engine.tactical.patrol import PatrolManager

router = APIRouter(prefix="/api/patrols", tags=["patrols"])

# Module-level singleton; initialized on first access or externally via set_manager()
_manager: PatrolManager | None = None


def get_manager() -> PatrolManager:
    """Get or create the singleton PatrolManager."""
    global _manager
    if _manager is None:
        _manager = PatrolManager()
    return _manager


def set_manager(manager: PatrolManager) -> None:
    """Set the PatrolManager instance (for wiring EventBus at boot)."""
    global _manager
    _manager = manager


# ------------------------------------------------------------------
# Request / Response models
# ------------------------------------------------------------------

class CreateRouteRequest(BaseModel):
    name: str
    waypoints: list[list[float]]  # [[x1,y1], [x2,y2], ...]
    loop: bool = True
    speed: float = 1.0


class RouteResponse(BaseModel):
    route_id: str
    name: str
    waypoints: list[list[float]]
    loop: bool
    speed: float


class AssignRequest(BaseModel):
    route_id: str
    asset_id: str


class UnassignRequest(BaseModel):
    asset_id: str


class PatrolAssignmentResponse(BaseModel):
    asset_id: str
    route_id: str
    waypoint_index: int
    position: list[float]
    started_at: float
    completed: bool


# ------------------------------------------------------------------
# Route CRUD
# ------------------------------------------------------------------

@router.post("/routes", response_model=RouteResponse, status_code=201)
async def create_route(request: CreateRouteRequest):
    """Create a new patrol route."""
    if len(request.waypoints) < 2:
        raise HTTPException(
            status_code=400, detail="Route must have at least 2 waypoints"
        )
    if request.speed <= 0:
        raise HTTPException(status_code=400, detail="Speed must be positive")

    for wp in request.waypoints:
        if len(wp) != 2:
            raise HTTPException(
                status_code=400, detail="Each waypoint must be [x, y]"
            )

    manager = get_manager()
    waypoints = [tuple(w) for w in request.waypoints]
    route_id = manager.create_route(
        name=request.name,
        waypoints=waypoints,
        loop=request.loop,
        speed=request.speed,
    )
    route = manager.get_route(route_id)
    return _route_response(route)


@router.get("/routes", response_model=list[RouteResponse])
async def list_routes():
    """List all patrol routes."""
    manager = get_manager()
    return [_route_response(r) for r in manager.list_routes()]


# ------------------------------------------------------------------
# Assignment
# ------------------------------------------------------------------

@router.post("/assign", response_model=PatrolAssignmentResponse)
async def assign_asset(request: AssignRequest):
    """Assign an asset to patrol a route."""
    manager = get_manager()
    if not manager.assign_asset(request.route_id, request.asset_id):
        raise HTTPException(
            status_code=404, detail="Route not found or has no waypoints"
        )
    assignment = manager.get_assignment(request.asset_id)
    return _assignment_response(assignment)


@router.post("/unassign")
async def unassign_asset(request: UnassignRequest):
    """Stop an asset's patrol."""
    manager = get_manager()
    if not manager.unassign_asset(request.asset_id):
        raise HTTPException(status_code=404, detail="Asset is not patrolling")
    return {"status": "unassigned", "asset_id": request.asset_id}


@router.get("/active", response_model=list[PatrolAssignmentResponse])
async def list_active():
    """List all active patrol assignments."""
    manager = get_manager()
    return [_assignment_response(a) for a in manager.get_active_patrols()]


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _route_response(route) -> RouteResponse:
    return RouteResponse(
        route_id=route.route_id,
        name=route.name,
        waypoints=[list(w) for w in route.waypoints],
        loop=route.loop,
        speed=route.speed,
    )


def _assignment_response(assignment) -> PatrolAssignmentResponse:
    return PatrolAssignmentResponse(
        asset_id=assignment.asset_id,
        route_id=assignment.route_id,
        waypoint_index=assignment.waypoint_index,
        position=list(assignment.position),
        started_at=assignment.started_at,
        completed=assignment.completed,
    )
