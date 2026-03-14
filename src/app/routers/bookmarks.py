# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Map bookmarks — save and recall named map positions.

Bookmarks persist in an in-memory store (reset on server restart).
Each bookmark stores lat/lng/zoom plus optional target filters for
quick-jump to monitored areas.
"""

from __future__ import annotations

import html
import re
import time
from typing import Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field, field_validator

router = APIRouter(prefix="/api/bookmarks", tags=["bookmarks"])

# ---------------------------------------------------------------------------
# Limits
# ---------------------------------------------------------------------------
_MAX_NAME_LEN = 100
_MAX_DESC_LEN = 1000
_MAX_FILTER_ITEMS = 50
_MAX_FILTER_LEN = 100
_MAX_BOOKMARKS = 1000
_HTML_TAG_RE = re.compile(r"<[^>]+>")


def _sanitize(value: str, max_len: int) -> str:
    """Strip HTML tags, escape, and enforce length limit."""
    value = _HTML_TAG_RE.sub("", value)
    value = html.escape(value)
    return value[:max_len]


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------

class BookmarkCreate(BaseModel):
    """Request to create a map bookmark."""
    name: str = Field(..., min_length=1, max_length=_MAX_NAME_LEN)
    lat: float = Field(..., ge=-90, le=90)
    lng: float = Field(..., ge=-180, le=180)
    zoom: float = Field(default=16.0, ge=0, le=24)
    pitch: float = Field(default=0.0, ge=-90, le=90)
    bearing: float = Field(default=0.0, ge=-360, le=360)
    alliance_filter: Optional[list[str]] = None
    asset_type_filter: Optional[list[str]] = None
    description: str = Field(default="", max_length=_MAX_DESC_LEN)

    @field_validator("name", "description")
    @classmethod
    def sanitize_strings(cls, v: str) -> str:
        return _sanitize(v, _MAX_DESC_LEN)

    @field_validator("alliance_filter", "asset_type_filter")
    @classmethod
    def validate_filters(cls, v):
        if v is not None:
            if len(v) > _MAX_FILTER_ITEMS:
                raise ValueError(f"Too many filter items (max {_MAX_FILTER_ITEMS})")
            return [_sanitize(s, _MAX_FILTER_LEN) for s in v]
        return v


class BookmarkUpdate(BaseModel):
    """Partial update for a bookmark."""
    name: Optional[str] = Field(default=None, max_length=_MAX_NAME_LEN)
    lat: Optional[float] = Field(default=None, ge=-90, le=90)
    lng: Optional[float] = Field(default=None, ge=-180, le=180)
    zoom: Optional[float] = Field(default=None, ge=0, le=24)
    pitch: Optional[float] = Field(default=None, ge=-90, le=90)
    bearing: Optional[float] = Field(default=None, ge=-360, le=360)
    alliance_filter: Optional[list[str]] = None
    asset_type_filter: Optional[list[str]] = None
    description: Optional[str] = Field(default=None, max_length=_MAX_DESC_LEN)

    @field_validator("name", "description")
    @classmethod
    def sanitize_strings(cls, v):
        if v is not None:
            return _sanitize(v, _MAX_DESC_LEN)
        return v

    @field_validator("alliance_filter", "asset_type_filter")
    @classmethod
    def validate_filters(cls, v):
        if v is not None:
            if len(v) > _MAX_FILTER_ITEMS:
                raise ValueError(f"Too many filter items (max {_MAX_FILTER_ITEMS})")
            return [_sanitize(s, _MAX_FILTER_LEN) for s in v]
        return v


class Bookmark(BaseModel):
    """A saved map position."""
    id: str
    name: str
    lat: float = Field(ge=-90, le=90)
    lng: float = Field(ge=-180, le=180)
    zoom: float = Field(default=16.0, ge=0, le=24)
    pitch: float = Field(default=0.0, ge=-90, le=90)
    bearing: float = Field(default=0.0, ge=-360, le=360)
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
    if len(_bookmarks) >= _MAX_BOOKMARKS:
        raise HTTPException(status_code=429, detail=f"Bookmark limit reached ({_MAX_BOOKMARKS})")
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
