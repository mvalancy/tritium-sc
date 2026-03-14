# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Target timeline API — chronological event history for a specific target.

Aggregates BLE sightings, camera detections, geofence events, enrichment
results, classification changes, and correlation events into a single
ordered timeline.

Endpoints:
    GET /api/targets/{target_id}/timeline — chronological event list
"""

from __future__ import annotations

import logging
from typing import Optional

from fastapi import APIRouter, Query, Request

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/targets", tags=["timeline"])


def _get_tracker(request: Request):
    """Get target tracker from Amy or app state."""
    amy = getattr(request.app.state, "amy", None)
    if amy is not None:
        tracker = getattr(amy, "target_tracker", None)
        if tracker is not None:
            return tracker
    return None


def _get_geofence_engine(request: Request):
    """Get geofence engine from app state or module singleton."""
    engine = getattr(request.app.state, "geofence_engine", None)
    if engine is not None:
        return engine
    # Fall back to module-level singleton
    try:
        from app.routers.geofence import get_engine
        return get_engine()
    except Exception:
        return None


def _get_enrichment_pipeline(request: Request):
    """Get enrichment pipeline from app state."""
    return getattr(request.app.state, "enrichment_pipeline", None)


def _get_event_log(request: Request):
    """Get event log store from app state (if available)."""
    return getattr(request.app.state, "event_log", None)


def _build_sighting_events(tracker, target_id: str) -> list[dict]:
    """Build BLE/WiFi sighting events from target history."""
    events: list[dict] = []
    if tracker is None:
        return events

    history = getattr(tracker, "history", None)
    if history is None:
        return events

    trail = history.get_trail_dicts(target_id, max_points=500)
    for pt in trail:
        events.append({
            "timestamp": pt.get("timestamp", 0.0),
            "event_type": "sighting",
            "source": "tracker",
            "data": {
                "x": pt.get("x", 0.0),
                "y": pt.get("y", 0.0),
                "speed": pt.get("speed", 0.0),
            },
            "position": {"x": pt.get("x", 0.0), "y": pt.get("y", 0.0)},
        })
    return events


def _build_geofence_events(geofence_engine, target_id: str) -> list[dict]:
    """Build geofence enter/exit events for a target."""
    events: list[dict] = []
    if geofence_engine is None:
        return events

    try:
        geo_events = geofence_engine.get_events(
            limit=200,
            target_id=target_id,
        )
    except Exception:
        return events

    for ge in geo_events:
        pos = ge.position if hasattr(ge, "position") else (0.0, 0.0)
        events.append({
            "timestamp": ge.timestamp,
            "event_type": f"geofence_{ge.event_type}",
            "source": "geofence",
            "data": {
                "zone_id": ge.zone_id,
                "zone_name": ge.zone_name,
                "zone_type": ge.zone_type,
                "event_type": ge.event_type,
            },
            "position": {"x": pos[0], "y": pos[1]},
        })
    return events


def _build_enrichment_events(pipeline, target_id: str) -> list[dict]:
    """Build enrichment result events for a target."""
    events: list[dict] = []
    if pipeline is None:
        return events

    cached = pipeline.get_cached(target_id)
    if cached is None:
        return events

    for result in cached:
        rd = result.to_dict() if hasattr(result, "to_dict") else {}
        events.append({
            "timestamp": rd.get("timestamp", 0.0),
            "event_type": "enrichment",
            "source": rd.get("provider", "enrichment"),
            "data": rd,
            "position": None,
        })
    return events


def _build_detection_events(tracker, target_id: str) -> list[dict]:
    """Build camera detection events from tracker data."""
    events: list[dict] = []
    if tracker is None:
        return events

    target = tracker.get_target(target_id)
    if target is None:
        return events

    # If this is a YOLO-detected target, include detection info
    if target.source == "yolo":
        events.append({
            "timestamp": target.last_seen,
            "event_type": "detection",
            "source": "camera",
            "data": {
                "asset_type": target.asset_type,
                "confidence": target.position_confidence,
                "alliance": target.alliance,
            },
            "position": {"x": target.position[0], "y": target.position[1]},
        })
    return events


def _build_classification_events(event_log, target_id: str) -> list[dict]:
    """Build classification change events from event log."""
    events: list[dict] = []
    if event_log is None:
        return events

    try:
        entries = event_log.get_events(
            target_id=target_id,
            event_type="classification",
            limit=100,
        )
        for entry in entries:
            events.append({
                "timestamp": entry.get("timestamp", 0.0),
                "event_type": "classification",
                "source": "classifier",
                "data": entry.get("data", {}),
                "position": entry.get("position"),
            })
    except Exception:
        pass
    return events


def _build_correlation_events(event_log, target_id: str) -> list[dict]:
    """Build correlation/fusion events from event log."""
    events: list[dict] = []
    if event_log is None:
        return events

    try:
        entries = event_log.get_events(
            target_id=target_id,
            event_type="correlation",
            limit=100,
        )
        for entry in entries:
            events.append({
                "timestamp": entry.get("timestamp", 0.0),
                "event_type": "correlation",
                "source": "correlator",
                "data": entry.get("data", {}),
                "position": entry.get("position"),
            })
    except Exception:
        pass
    return events


@router.get("/{target_id}/timeline")
async def get_target_timeline(
    request: Request,
    target_id: str,
    start: Optional[float] = Query(None, description="Start timestamp (epoch seconds)"),
    end: Optional[float] = Query(None, description="End timestamp (epoch seconds)"),
    event_types: Optional[str] = Query(
        None,
        description="Comma-separated event types to include (e.g. sighting,detection,geofence_enter)",
    ),
    limit: int = Query(500, ge=1, le=5000, description="Max events to return"),
):
    """Return chronological timeline of all events for a target.

    Aggregates events from multiple sources:
    - sighting: BLE/WiFi position sightings with RSSI
    - detection: Camera detections with confidence
    - geofence_enter/geofence_exit: Geofence transition events
    - enrichment: Intelligence enrichment results
    - classification: Alliance/threat classification changes
    - correlation: Target fusion/correlation events

    Events are returned sorted by timestamp (ascending).
    """
    tracker = _get_tracker(request)
    geofence_engine = _get_geofence_engine(request)
    enrichment_pipeline = _get_enrichment_pipeline(request)
    event_log = _get_event_log(request)

    # Verify target exists (in tracker or simulation)
    target_info = None
    if tracker is not None:
        target = tracker.get_target(target_id)
        if target is not None:
            target_info = target.to_dict()

    if target_info is None:
        # Check simulation engine
        amy = getattr(request.app.state, "amy", None)
        engine = None
        if amy is not None:
            engine = getattr(amy, "simulation_engine", None)
        if engine is None:
            engine = getattr(request.app.state, "simulation_engine", None)
        if engine is not None:
            for t in engine.get_targets():
                if t.target_id == target_id:
                    target_info = t.to_dict()
                    break

    if target_info is None:
        return {
            "target_id": target_id,
            "events": [],
            "total": 0,
            "error": "Target not found",
        }

    # Collect events from all sources
    all_events: list[dict] = []
    all_events.extend(_build_sighting_events(tracker, target_id))
    all_events.extend(_build_detection_events(tracker, target_id))
    all_events.extend(_build_geofence_events(geofence_engine, target_id))
    all_events.extend(_build_enrichment_events(enrichment_pipeline, target_id))
    all_events.extend(_build_classification_events(event_log, target_id))
    all_events.extend(_build_correlation_events(event_log, target_id))

    # Filter by time range
    if start is not None:
        all_events = [e for e in all_events if e["timestamp"] >= start]
    if end is not None:
        all_events = [e for e in all_events if e["timestamp"] <= end]

    # Filter by event type
    if event_types:
        allowed = set(event_types.split(","))
        all_events = [e for e in all_events if e["event_type"] in allowed]

    # Sort chronologically
    all_events.sort(key=lambda e: e["timestamp"])

    # Apply limit
    total = len(all_events)
    all_events = all_events[-limit:]

    return {
        "target_id": target_id,
        "target": target_info,
        "events": all_events,
        "total": total,
    }
