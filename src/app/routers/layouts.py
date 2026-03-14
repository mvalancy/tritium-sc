# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Dashboard layout persistence API — save/load panel arrangements.

Endpoints:
    GET    /api/layouts           — list available layouts
    GET    /api/layouts/{name}    — load a specific layout
    POST   /api/layouts           — save/update a layout
    DELETE /api/layouts/{name}    — delete a layout
    POST   /api/layouts/{name}/duplicate — duplicate a layout
"""

from __future__ import annotations

import logging
from typing import Optional

from fastapi import APIRouter, HTTPException, Query, Request
from pydantic import BaseModel

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/layouts", tags=["layouts"])


class PanelConfigBody(BaseModel):
    """Panel configuration in a layout save request."""
    panel_id: str
    visible: bool = True
    position: dict = {"x": 0, "y": 0}
    size: dict = {"width": 400, "height": 300}
    order: int = 0
    collapsed: bool = False
    settings: dict = {}


class SaveLayoutRequest(BaseModel):
    """Request body for saving a dashboard layout."""
    name: str
    user: str = "default"
    description: str = ""
    panels: list[PanelConfigBody] = []
    map_settings: dict = {}


class DuplicateRequest(BaseModel):
    """Request body for duplicating a layout."""
    new_name: str
    user: str = "default"


def _get_layout_manager(request: Request):
    """Get DashboardLayoutManager from app state."""
    mgr = getattr(request.app.state, "layout_manager", None)
    if mgr is None:
        # Try from Amy
        amy = getattr(request.app.state, "amy", None)
        if amy:
            mgr = getattr(amy, "layout_manager", None)
    return mgr


@router.get("")
async def list_layouts(
    request: Request,
    user: Optional[str] = Query(None, description="Filter by user"),
):
    """List all available dashboard layouts."""
    mgr = _get_layout_manager(request)
    if mgr is None:
        return {"layouts": [], "count": 0, "error": "Layout manager not initialized"}

    layouts = mgr.list_layouts(user=user)
    return {"layouts": layouts, "count": len(layouts)}


@router.get("/{name}")
async def get_layout(
    request: Request,
    name: str,
    user: str = Query("default", description="User who owns the layout"),
):
    """Load a specific dashboard layout."""
    mgr = _get_layout_manager(request)
    if mgr is None:
        raise HTTPException(status_code=503, detail="Layout manager not initialized")

    layout = mgr.load(name, user=user)
    if layout is None:
        raise HTTPException(status_code=404, detail=f"Layout '{name}' not found")
    return layout


@router.post("")
async def save_layout(request: Request, body: SaveLayoutRequest):
    """Save or update a dashboard layout."""
    mgr = _get_layout_manager(request)
    if mgr is None:
        raise HTTPException(status_code=503, detail="Layout manager not initialized")

    panels = [p.model_dump() for p in body.panels]
    result = mgr.save(
        name=body.name,
        user=body.user,
        description=body.description,
        panels=panels,
        map_settings=body.map_settings,
    )
    return result


@router.delete("/{name}")
async def delete_layout(
    request: Request,
    name: str,
    user: str = Query("default", description="User who owns the layout"),
):
    """Delete a dashboard layout."""
    mgr = _get_layout_manager(request)
    if mgr is None:
        raise HTTPException(status_code=503, detail="Layout manager not initialized")

    deleted = mgr.delete(name, user=user)
    if not deleted:
        raise HTTPException(status_code=404, detail=f"Layout '{name}' not found")
    return {"deleted": True, "name": name}


@router.post("/{name}/duplicate")
async def duplicate_layout(
    request: Request,
    name: str,
    body: DuplicateRequest,
):
    """Duplicate an existing layout under a new name."""
    mgr = _get_layout_manager(request)
    if mgr is None:
        raise HTTPException(status_code=503, detail="Layout manager not initialized")

    result = mgr.duplicate(name, body.new_name, user=body.user)
    if result is None:
        raise HTTPException(status_code=404, detail=f"Source layout '{name}' not found")
    return result
