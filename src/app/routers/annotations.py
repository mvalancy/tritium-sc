# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Map annotation system — persist text labels, arrows, circles, and freehand
drawings on the tactical map for briefings and operational planning.

Annotations are stored in-memory (reset on server restart). Each annotation
has a type, position, style, and optional metadata.
"""

from __future__ import annotations

import time
import uuid
from typing import Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api/annotations", tags=["annotations"])


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------

class AnnotationCreate(BaseModel):
    """Create a new map annotation."""
    type: str = Field(..., description="text | arrow | circle | freehand | rectangle | polygon")
    lat: float = Field(..., description="Latitude of annotation anchor")
    lng: float = Field(..., description="Longitude of annotation anchor")
    # Text content (for text annotations)
    text: str = ""
    # Geometry (type-specific)
    end_lat: Optional[float] = None  # arrow end point
    end_lng: Optional[float] = None
    radius_m: Optional[float] = None  # circle radius in meters
    points: Optional[list[list[float]]] = None  # freehand/polygon: [[lat,lng], ...]
    width: Optional[float] = None  # rectangle width in meters
    height: Optional[float] = None  # rectangle height in meters
    # Style
    color: str = "#00f0ff"
    stroke_width: float = 2.0
    font_size: float = 14.0
    opacity: float = 0.8
    fill: bool = False
    fill_opacity: float = 0.2
    # Metadata
    label: str = ""
    layer: str = "default"
    locked: bool = False


class AnnotationUpdate(BaseModel):
    """Partial update for an annotation."""
    lat: Optional[float] = None
    lng: Optional[float] = None
    text: Optional[str] = None
    end_lat: Optional[float] = None
    end_lng: Optional[float] = None
    radius_m: Optional[float] = None
    points: Optional[list[list[float]]] = None
    width: Optional[float] = None
    height: Optional[float] = None
    color: Optional[str] = None
    stroke_width: Optional[float] = None
    font_size: Optional[float] = None
    opacity: Optional[float] = None
    fill: Optional[bool] = None
    fill_opacity: Optional[float] = None
    label: Optional[str] = None
    layer: Optional[str] = None
    locked: Optional[bool] = None


# ---------------------------------------------------------------------------
# In-memory store
# ---------------------------------------------------------------------------

_annotations: dict[str, dict[str, Any]] = {}


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@router.get("")
async def list_annotations(layer: Optional[str] = None):
    """List all map annotations, optionally filtered by layer."""
    items = list(_annotations.values())
    if layer:
        items = [a for a in items if a.get("layer") == layer]
    return {
        "annotations": sorted(items, key=lambda a: a.get("created_at", 0)),
        "count": len(items),
    }


@router.post("")
async def create_annotation(body: AnnotationCreate):
    """Create a new map annotation."""
    now = time.time()
    ann_id = f"ann_{uuid.uuid4().hex[:8]}"
    annotation = {
        "id": ann_id,
        "type": body.type,
        "lat": body.lat,
        "lng": body.lng,
        "text": body.text,
        "end_lat": body.end_lat,
        "end_lng": body.end_lng,
        "radius_m": body.radius_m,
        "points": body.points,
        "width": body.width,
        "height": body.height,
        "color": body.color,
        "stroke_width": body.stroke_width,
        "font_size": body.font_size,
        "opacity": body.opacity,
        "fill": body.fill,
        "fill_opacity": body.fill_opacity,
        "label": body.label,
        "layer": body.layer,
        "locked": body.locked,
        "created_at": now,
        "updated_at": now,
    }
    _annotations[ann_id] = annotation
    return annotation


@router.get("/{annotation_id}")
async def get_annotation(annotation_id: str):
    """Get a single annotation by ID."""
    ann = _annotations.get(annotation_id)
    if ann is None:
        raise HTTPException(status_code=404, detail="Annotation not found")
    return ann


@router.put("/{annotation_id}")
async def update_annotation(annotation_id: str, body: AnnotationUpdate):
    """Update an existing annotation."""
    ann = _annotations.get(annotation_id)
    if ann is None:
        raise HTTPException(status_code=404, detail="Annotation not found")
    if ann.get("locked"):
        raise HTTPException(status_code=403, detail="Annotation is locked")

    updates = body.model_dump(exclude_none=True)
    if updates:
        ann.update(updates)
        ann["updated_at"] = time.time()

    return ann


@router.delete("/{annotation_id}")
async def delete_annotation(annotation_id: str):
    """Delete an annotation."""
    if annotation_id not in _annotations:
        raise HTTPException(status_code=404, detail="Annotation not found")
    del _annotations[annotation_id]
    return {"ok": True, "deleted": annotation_id}


@router.delete("")
async def clear_annotations(layer: Optional[str] = None):
    """Clear all annotations, optionally filtered by layer."""
    if layer:
        to_delete = [k for k, v in _annotations.items() if v.get("layer") == layer]
    else:
        to_delete = list(_annotations.keys())
    for k in to_delete:
        del _annotations[k]
    return {"ok": True, "deleted_count": len(to_delete)}


@router.get("/layers/list")
async def list_layers():
    """List all annotation layers."""
    layers = set()
    for ann in _annotations.values():
        layers.add(ann.get("layer", "default"))
    return {"layers": sorted(layers)}
