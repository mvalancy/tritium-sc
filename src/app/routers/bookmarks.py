# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Map bookmarks — save and recall named map positions.

Bookmarks persist in an in-memory store (reset on server restart).
Each bookmark stores lat/lng/zoom plus optional target filters for
quick-jump to monitored areas.
"""

from __future__ import annotations

import time
from typing import Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api/bookmarks", tags=["bookmarks"])


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------

class BookmarkCreate(BaseModel):
    """Request to create a map bookmark."""
    name: str = Field(..., min_length=1, max_length=100)
    lat: float
    lng: float
    zoom: float = 16.0
    pitch: float = 0.0
    bearing: float = 0.0
    alliance_filter: Optional[list[str]] = None
    asset_type_filter: Optional[list[str]] = None
    description: str = ""


class BookmarkUpdate(BaseModel):
    """Partial update for a bookmark."""
    name: Optional[str] = None
    lat: Optional[float] = None
    lng: Optional[float] = None
    zoom: Optional[float] = None
    pitch: Optional[float] = None
    bearing: Optional[float] = None
    alliance_filter: Optional[list[str]] = None
    asset_type_filter: Optional[list[str]] = None
    description: Optional[str] = None


class Bookmark(BaseModel):
    """A saved map position."""
    id: str
    name: str
    lat: float
    lng: float
    zoom: float = 16.0
    pitch: float = 0.0
    bearing: float = 0.0
    alliance_filter: Optional[list[str]] = None
    asset_type_filter: Optional[list[str]] = None
    description: str = ""
    created_at: float = 0.0
    updated_at: float = 0.0


# ---------------------------------------------------------------------------
# In-memory store
# ---------------------------------------------------------------------------

_bookmarks: dict[str, dict[str, Any]] = {}
_next_id = 1


def _generate_id() -> str:
    global _next_id
    bid = f"bm_{_next_id}"
    _next_id += 1
    return bid


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@router.get("")
async def list_bookmarks():
    """List all saved map bookmarks."""
    return {
        "bookmarks": sorted(
            _bookmarks.values(),
            key=lambda b: b.get("created_at", 0),
        ),
        "count": len(_bookmarks),
    }


@router.post("")
async def create_bookmark(body: BookmarkCreate):
    """Save a new map bookmark."""
    now = time.time()
    bid = _generate_id()
    bookmark = {
        "id": bid,
        "name": body.name,
        "lat": body.lat,
        "lng": body.lng,
        "zoom": body.zoom,
        "pitch": body.pitch,
        "bearing": body.bearing,
        "alliance_filter": body.alliance_filter,
        "asset_type_filter": body.asset_type_filter,
        "description": body.description,
        "created_at": now,
        "updated_at": now,
    }
    _bookmarks[bid] = bookmark
    return bookmark


@router.get("/{bookmark_id}")
async def get_bookmark(bookmark_id: str):
    """Get a single bookmark by ID."""
    bookmark = _bookmarks.get(bookmark_id)
    if bookmark is None:
        raise HTTPException(status_code=404, detail="Bookmark not found")
    return bookmark


@router.put("/{bookmark_id}")
async def update_bookmark(bookmark_id: str, body: BookmarkUpdate):
    """Update an existing bookmark."""
    bookmark = _bookmarks.get(bookmark_id)
    if bookmark is None:
        raise HTTPException(status_code=404, detail="Bookmark not found")

    updates = body.model_dump(exclude_none=True)
    if updates:
        bookmark.update(updates)
        bookmark["updated_at"] = time.time()

    return bookmark


@router.delete("/{bookmark_id}")
async def delete_bookmark(bookmark_id: str):
    """Delete a bookmark."""
    if bookmark_id not in _bookmarks:
        raise HTTPException(status_code=404, detail="Bookmark not found")
    del _bookmarks[bookmark_id]
    return {"ok": True, "deleted": bookmark_id}
