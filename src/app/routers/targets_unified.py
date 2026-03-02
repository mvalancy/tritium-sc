# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Unified target tracking endpoint — real + virtual targets."""

from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, Query, Request

router = APIRouter(prefix="/api", tags=["targets"])


def _get_tracker(request: Request):
    """Get target tracker from Amy, or fall back to simulation engine targets."""
    amy = getattr(request.app.state, "amy", None)
    if amy is not None:
        tracker = getattr(amy, "target_tracker", None)
        if tracker is not None:
            return tracker
    return None


def _get_sim_engine(request: Request):
    """Get simulation engine (headless fallback when Amy is disabled)."""
    return getattr(request.app.state, "simulation_engine", None)


@router.get("/targets")
async def get_targets(
    request: Request,
    source: Optional[str] = Query(None, description="Filter by source: real, sim, graphling"),
):
    """Return all tracked targets (simulation + YOLO detections).

    Optional query parameter ``source`` filters by target source classification:
    - ``real`` — physical hardware / YOLO-detected targets
    - ``sim`` — locally simulated targets
    - ``graphling`` — remote Graphlings agents
    """
    tracker = _get_tracker(request)
    if tracker is not None:
        targets = tracker.get_all()
        dicts = [t.to_dict() for t in targets]
        if source:
            # TrackedTarget.source uses "yolo"/"simulation"/"manual"; map to
            # SimulationTarget convention for filtering consistency.
            _source_map = {"real": "yolo", "sim": "simulation"}
            tracker_source = _source_map.get(source, source)
            dicts = [d for d in dicts if d.get("source") == tracker_source]
        return {
            "targets": dicts,
            "summary": tracker.summary(),
        }

    # Headless mode: read targets directly from simulation engine
    engine = _get_sim_engine(request)
    if engine is not None:
        targets = engine.get_targets()
        if source:
            # Apply same source mapping as tracker path for consistency.
            _source_map = {"real": "yolo", "sim": "simulation"}
            # SimulationTarget.source uses short names ("sim", "real", "graphling"),
            # while TrackedTarget uses long names ("simulation", "yolo").
            # Accept both conventions.
            mapped = _source_map.get(source, source)
            targets = [
                t for t in targets if t.source == source or t.source == mapped
            ]
        return {
            "targets": [t.to_dict() for t in targets],
            "summary": f"{len(targets)} simulation targets",
        }

    return {"targets": [], "summary": "No tracking available"}


@router.get("/targets/hostiles")
async def get_hostiles(request: Request):
    """Return only hostile targets."""
    tracker = _get_tracker(request)
    if tracker is not None:
        return {"targets": [t.to_dict() for t in tracker.get_hostiles()]}

    engine = _get_sim_engine(request)
    if engine is not None:
        targets = [t for t in engine.get_targets() if t.alliance == "hostile"]
        return {"targets": [t.to_dict() for t in targets]}

    return {"targets": []}


@router.get("/targets/friendlies")
async def get_friendlies(request: Request):
    """Return only friendly targets."""
    tracker = _get_tracker(request)
    if tracker is not None:
        return {"targets": [t.to_dict() for t in tracker.get_friendlies()]}

    engine = _get_sim_engine(request)
    if engine is not None:
        targets = [t for t in engine.get_targets() if t.alliance == "friendly"]
        return {"targets": [t.to_dict() for t in targets]}

    return {"targets": []}


@router.post("/sighting")
async def report_sighting(request: Request):
    """Accept a sighting report from camera or robot."""
    # Try Amy's engine first, then headless fallback.
    engine = None
    amy = getattr(request.app.state, "amy", None)
    if amy is not None:
        engine = getattr(amy, "simulation_engine", None)
    if engine is None:
        engine = _get_sim_engine(request)
    if engine is None:
        return {"error": "No simulation engine"}
    body = await request.json()
    from engine.simulation.vision import SightingReport
    report = SightingReport(
        observer_id=body.get("observer_id", "unknown"),
        target_id=body.get("target_id", ""),
        observer_type=body.get("observer_type", "camera"),
        confidence=body.get("confidence", 1.0),
        position=tuple(body["position"]) if "position" in body else None,
        timestamp=body.get("timestamp", 0.0),
    )
    engine.vision_system.add_sighting(report)
    return {"status": "accepted"}
