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

import html
import logging
import re
from typing import Optional

from fastapi import APIRouter, HTTPException, Query, Request
from pydantic import BaseModel, Field, field_validator

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/layouts", tags=["layouts"])

_MAX_NAME_LEN = 100
_MAX_DESC_LEN = 500
_MAX_USER_LEN = 100
_MAX_PANELS = 200
_HTML_TAG_RE = re.compile(r"<[^>]+>")


def _sanitize(value: str, max_len: int) -> str:
    """Strip HTML tags, escape, and enforce length limit."""
    value = _HTML_TAG_RE.sub("", value)
    value = html.escape(value)
    return value[:max_len]


class PanelConfigBody(BaseModel):
    """Panel configuration in a layout save request."""
    panel_id: str = Field(..., max_length=_MAX_NAME_LEN)
    visible: bool = True
    position: dict = {"x": 0, "y": 0}
    size: dict = {"width": 400, "height": 300}
    order: int = Field(default=0, ge=-1000, le=1000)
    collapsed: bool = False
    settings: dict = {}


class SaveLayoutRequest(BaseModel):
    """Request body for saving a dashboard layout."""
    name: str = Field(..., min_length=1, max_length=_MAX_NAME_LEN)
    user: str = Field(default="default", max_length=_MAX_USER_LEN)
    description: str = Field(default="", max_length=_MAX_DESC_LEN)
    panels: list[PanelConfigBody] = Field(default_factory=list)
    map_settings: dict = {}

    @field_validator("name", "user", "description")
    @classmethod
    def sanitize_strings(cls, v: str) -> str:
        return _sanitize(v, _MAX_DESC_LEN)

    @field_validator("panels")
    @classmethod
    def validate_panels(cls, v):
        if len(v) > _MAX_PANELS:
            raise ValueError(f"Too many panels (max {_MAX_PANELS})")
        return v


class DuplicateRequest(BaseModel):
    """Request body for duplicating a layout."""
    new_name: str = Field(..., min_length=1, max_length=_MAX_NAME_LEN)
    user: str = Field(default="default", max_length=_MAX_USER_LEN)

    @field_validator("new_name", "user")
    @classmethod
    def sanitize_strings(cls, v: str) -> str:
        return _sanitize(v, _MAX_NAME_LEN)


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
