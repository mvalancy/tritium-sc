# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""AR export endpoint — simplified target data for augmented reality overlays.

Returns target positions in a lightweight format optimized for AR headsets
and mobile AR apps. Each target includes lat/lng/alt/type/name/alliance
for spatial overlay rendering.
"""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Query, Request

router = APIRouter(prefix="/api/targets", tags=["ar-export"])


def _get_tracker(request: Request):
    """Get target tracker from Amy, or fall back to simulation engine."""
    amy = getattr(request.app.state, "amy", None)
    if amy is not None:
        tracker = getattr(amy, "target_tracker", None)
        if tracker is not None:
            return tracker
    return None


def _get_sim_engine(request: Request):
    """Get simulation engine (headless fallback)."""
    return getattr(request.app.state, "simulation_engine", None)


@router.get("/ar-export")
async def ar_export(
    request: Request,
    alliance: Optional[str] = Query(None, description="Filter by alliance: friendly, hostile, unknown"),
    max_targets: int = Query(100, ge=1, le=1000, description="Max targets to return"),
    min_confidence: float = Query(0.0, ge=0.0, le=1.0, description="Min position confidence"),
):
    """Export target data formatted for AR overlay apps.

    Returns a simplified JSON array with lat/lng/alt/type/name for each
    visible target. Designed for low-overhead consumption by AR headsets,
    mobile AR apps, and HUD overlays.

    Response format is intentionally flat and simple for easy parsing
    on resource-constrained AR devices.
    """
    tracker = _get_tracker(request)
    all_targets: list[dict] = []

    if tracker is not None:
        all_targets = [t.to_dict() for t in tracker.get_all()]
    else:
        engine = _get_sim_engine(request)
        if engine is not None:
            all_targets = [t.to_dict() for t in engine.get_targets()]

    # Filter by alliance
    if alliance:
        alliance_lower = alliance.lower()
        all_targets = [t for t in all_targets if (t.get("alliance", "") or "").lower() == alliance_lower]

    # Filter by confidence
    if min_confidence > 0:
        all_targets = [
            t for t in all_targets
            if (t.get("position_confidence", 0.0) or 0.0) >= min_confidence
        ]

    # Limit count
    all_targets = all_targets[:max_targets]

    # Build simplified AR payload
    ar_targets = []
    for t in all_targets:
        lat = t.get("lat", 0.0) or 0.0
        lng = t.get("lng", 0.0) or 0.0

        # Skip targets with no real position
        if lat == 0.0 and lng == 0.0:
            pos = t.get("position", {})
            if isinstance(pos, dict):
                # Use local coords if no geo
                lat = pos.get("y", 0.0)
                lng = pos.get("x", 0.0)

        # Derive altitude from asset type
        asset_type = (t.get("asset_type", "") or t.get("type", "") or "unknown").lower()
        alt = 0.0
        if "drone" in asset_type or "uav" in asset_type:
            alt = 30.0  # Default drone altitude
        elif "person" in asset_type:
            alt = 1.7  # Average person height
        elif "vehicle" in asset_type:
            alt = 1.5

        ar_targets.append({
            "id": t.get("target_id", ""),
            "name": t.get("name", t.get("target_id", "")),
            "type": asset_type,
            "alliance": (t.get("alliance", "") or "unknown").lower(),
            "lat": lat,
            "lng": lng,
            "alt": alt,
            "heading": t.get("heading", 0.0) or 0.0,
            "speed": t.get("speed", 0.0) or 0.0,
            "confidence": t.get("position_confidence", 0.0) or 0.0,
            "threat_score": t.get("threat_score", 0.0) or 0.0,
            "status": (t.get("status", "") or "active").lower(),
        })

    return {
        "version": "1.0",
        "target_count": len(ar_targets),
        "targets": ar_targets,
    }
