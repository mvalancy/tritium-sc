# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Geofence API — polygon zone CRUD and event retrieval.

Endpoints:
    GET    /api/geofence/zones       — list all geofence zones
    POST   /api/geofence/zones       — create a zone
    DELETE /api/geofence/zones/{id}  — delete a zone
    GET    /api/geofence/events      — list geofence events (enter/exit)
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

from engine.tactical.geofence import GeoZone, GeofenceEngine

router = APIRouter(prefix="/api/geofence", tags=["geofence"])

# Module-level singleton; initialized on first access or externally via set_engine()
_engine: GeofenceEngine | None = None


def get_engine() -> GeofenceEngine:
    """Get or create the singleton GeofenceEngine."""
    global _engine
    if _engine is None:
        _engine = GeofenceEngine()
    return _engine


def set_engine(engine: GeofenceEngine) -> None:
    """Set the GeofenceEngine instance (for wiring EventBus at boot)."""
    global _engine
    _engine = engine


# ------------------------------------------------------------------
# Request / Response models
# ------------------------------------------------------------------

class CreateGeoZoneRequest(BaseModel):
    name: str
    polygon: list[list[float]]  # [[x1,y1], [x2,y2], ...]
    zone_type: str = "monitored"  # "restricted", "monitored", "safe"
    alert_on_enter: bool = True
    alert_on_exit: bool = True


class GeoZoneResponse(BaseModel):
    zone_id: str
    name: str
    polygon: list[list[float]]
    zone_type: str
    alert_on_enter: bool
    alert_on_exit: bool
    enabled: bool
    created_at: float


class GeoEventResponse(BaseModel):
    event_id: str
    event_type: str
    target_id: str
    zone_id: str
    zone_name: str
    zone_type: str
    position: list[float]
    timestamp: float


# ------------------------------------------------------------------
# Zone CRUD
# ------------------------------------------------------------------

@router.get("/zones", response_model=list[GeoZoneResponse])
async def list_zones():
    """List all geofence zones."""
    engine = get_engine()
    return [_zone_response(z) for z in engine.list_zones()]


@router.post("/zones", response_model=GeoZoneResponse, status_code=201)
async def create_zone(request: CreateGeoZoneRequest):
    """Create a new geofence zone."""
    if len(request.polygon) < 3:
        raise HTTPException(status_code=400, detail="Polygon must have at least 3 vertices")

    valid_types = ("restricted", "monitored", "safe")
    if request.zone_type not in valid_types:
        raise HTTPException(
            status_code=400,
            detail=f"zone_type must be one of: {valid_types}",
        )

    import uuid
    zone = GeoZone(
        zone_id=uuid.uuid4().hex[:12],
        name=request.name,
        polygon=[tuple(p) for p in request.polygon],
        zone_type=request.zone_type,
        alert_on_enter=request.alert_on_enter,
        alert_on_exit=request.alert_on_exit,
    )

    engine = get_engine()
    engine.add_zone(zone)
    return _zone_response(zone)


@router.delete("/zones/{zone_id}")
async def delete_zone(zone_id: str):
    """Delete a geofence zone."""
    engine = get_engine()
    if not engine.remove_zone(zone_id):
        raise HTTPException(status_code=404, detail="Zone not found")
    return {"status": "deleted", "zone_id": zone_id}


# ------------------------------------------------------------------
# Events
# ------------------------------------------------------------------

@router.get("/events", response_model=list[GeoEventResponse])
async def list_events(
    limit: int = Query(100, le=1000),
    zone_id: Optional[str] = Query(None),
    target_id: Optional[str] = Query(None),
    event_type: Optional[str] = Query(None, description="enter, exit, or inside"),
):
    """List recent geofence events (enter/exit transitions)."""
    engine = get_engine()
    events = engine.get_events(
        limit=limit,
        zone_id=zone_id,
        target_id=target_id,
        event_type=event_type,
    )
    return [_event_response(e) for e in events]


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _zone_response(zone: GeoZone) -> GeoZoneResponse:
    return GeoZoneResponse(
        zone_id=zone.zone_id,
        name=zone.name,
        polygon=[list(p) for p in zone.polygon],
        zone_type=zone.zone_type,
        alert_on_enter=zone.alert_on_enter,
        alert_on_exit=zone.alert_on_exit,
        enabled=zone.enabled,
        created_at=zone.created_at,
    )


def _event_response(e) -> GeoEventResponse:
    return GeoEventResponse(
        event_id=e.event_id,
        event_type=e.event_type,
        target_id=e.target_id,
        zone_id=e.zone_id,
        zone_name=e.zone_name,
        zone_type=e.zone_type,
        position=list(e.position),
        timestamp=e.timestamp,
    )
