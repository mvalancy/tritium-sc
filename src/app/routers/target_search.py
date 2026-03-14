# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Target search and filter API — search by name/MAC/ID, filter by source/alliance/time."""

from __future__ import annotations

import time
from typing import Optional

from fastapi import APIRouter, Query, Request

router = APIRouter(prefix="/api/targets", tags=["target-search"])


def _get_tracker(request: Request):
    """Get target tracker from Amy, or fall back to app state."""
    amy = getattr(request.app.state, "amy", None)
    if amy is not None:
        tracker = getattr(amy, "target_tracker", None)
        if tracker is not None:
            return tracker
    return None


def _get_sim_engine(request: Request):
    """Get simulation engine (headless fallback when Amy is disabled)."""
    return getattr(request.app.state, "simulation_engine", None)


def _all_target_dicts(request: Request) -> list[dict]:
    """Collect all targets as dicts from tracker or simulation engine."""
    tracker = _get_tracker(request)
    if tracker is not None:
        targets = tracker.get_all()
        return [t.to_dict(history=tracker.history) for t in targets]

    engine = _get_sim_engine(request)
    if engine is not None:
        return [t.to_dict() for t in engine.get_targets()]

    return []


def _get_tracker_raw(request: Request):
    """Get raw tracker for single-target lookup with trail."""
    return _get_tracker(request)


@router.get("/search")
async def search_targets(
    request: Request,
    q: str = Query(..., min_length=1, description="Search query (name, MAC, target_id)"),
):
    """Search targets by name, MAC address, or target_id.

    Performs case-insensitive substring matching across target_id, name,
    and asset_type fields.
    """
    targets = _all_target_dicts(request)
    query = q.lower()
    results = []
    for t in targets:
        tid = (t.get("target_id") or "").lower()
        name = (t.get("name") or "").lower()
        asset_type = (t.get("asset_type") or "").lower()
        alliance = (t.get("alliance") or "").lower()
        if query in tid or query in name or query in asset_type or query in alliance:
            results.append(t)
    return {"results": results, "total": len(results), "query": q}


@router.get("/filter")
async def filter_targets(
    request: Request,
    source: Optional[str] = Query(None, description="Filter by source: simulation, yolo, ble, manual"),
    alliance: Optional[str] = Query(None, description="Filter by alliance: friendly, hostile, unknown"),
    asset_type: Optional[str] = Query(None, description="Filter by asset_type: rover, drone, person, etc."),
    since: Optional[int] = Query(None, description="Only targets seen in last N seconds"),
):
    """Filter targets by source, alliance, asset_type, and recency.

    All filters are optional and combine with AND logic.
    """
    targets = _all_target_dicts(request)
    now = time.monotonic()

    results = []
    for t in targets:
        if source and t.get("source") != source:
            continue
        if alliance and t.get("alliance") != alliance:
            continue
        if asset_type and t.get("asset_type") != asset_type:
            continue
        if since is not None:
            last_seen = t.get("last_seen", 0)
            if (now - last_seen) > since:
                continue
        results.append(t)

    return {"targets": results, "total": len(results)}


@router.get("/stats")
async def target_stats(request: Request):
    """Return target counts broken down by source, alliance, and activity windows."""
    targets = _all_target_dicts(request)
    now = time.monotonic()

    by_source: dict[str, int] = {}
    by_alliance: dict[str, int] = {}
    by_asset_type: dict[str, int] = {}
    active_5min = 0
    active_1hr = 0
    active_24hr = 0

    for t in targets:
        src = t.get("source", "unknown")
        by_source[src] = by_source.get(src, 0) + 1

        ally = t.get("alliance", "unknown")
        by_alliance[ally] = by_alliance.get(ally, 0) + 1

        atype = t.get("asset_type", "unknown")
        by_asset_type[atype] = by_asset_type.get(atype, 0) + 1

        last_seen = t.get("last_seen", 0)
        age = now - last_seen
        if age <= 300:
            active_5min += 1
        if age <= 3600:
            active_1hr += 1
        if age <= 86400:
            active_24hr += 1

    return {
        "total": len(targets),
        "by_source": by_source,
        "by_alliance": by_alliance,
        "by_asset_type": by_asset_type,
        "active_5min": active_5min,
        "active_1hr": active_1hr,
        "active_24hr": active_24hr,
    }


@router.get("/{target_id}")
async def get_target_detail(request: Request, target_id: str):
    """Get a single target with full detail including recent trail."""
    tracker = _get_tracker_raw(request)
    if tracker is not None:
        target = tracker.get_target(target_id)
        if target is not None:
            return target.to_dict(history=tracker.history)

    # Fallback: search simulation engine targets
    engine = _get_sim_engine(request)
    if engine is not None:
        for t in engine.get_targets():
            if t.target_id == target_id:
                return t.to_dict()

    return {"error": "Target not found", "target_id": target_id}
