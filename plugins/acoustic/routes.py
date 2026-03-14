# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""FastAPI routes for acoustic classification plugin.

Provides REST endpoints for submitting audio features, querying
classified events, and checking plugin status.
"""

from __future__ import annotations

from typing import Any, Optional

from fastapi import APIRouter
from pydantic import BaseModel


class ClassifyRequest(BaseModel):
    """Request body for audio classification."""
    rms_energy: float = 0.0
    peak_amplitude: float = 0.0
    zero_crossing_rate: float = 0.0
    spectral_centroid: float = 0.0
    spectral_bandwidth: float = 0.0
    duration_ms: int = 0
    device_id: str = ""
    lat: Optional[float] = None
    lng: Optional[float] = None


def create_router(plugin: Any) -> APIRouter:
    """Build acoustic classification API router.

    Parameters
    ----------
    plugin:
        AcousticPlugin instance.
    """
    router = APIRouter(prefix="/api/acoustic", tags=["acoustic"])

    @router.post("/classify")
    async def classify_audio(body: ClassifyRequest):
        """Classify audio features and return the detection event."""
        location = None
        if body.lat is not None and body.lng is not None:
            location = (body.lat, body.lng)

        result = plugin.classify_audio(
            features={
                "rms_energy": body.rms_energy,
                "peak_amplitude": body.peak_amplitude,
                "zero_crossing_rate": body.zero_crossing_rate,
                "spectral_centroid": body.spectral_centroid,
                "spectral_bandwidth": body.spectral_bandwidth,
                "duration_ms": body.duration_ms,
            },
            device_id=body.device_id,
            location=location,
        )
        return result

    @router.get("/events")
    async def get_events(count: int = 50):
        """Return recent classified acoustic events."""
        events = plugin.get_recent_events(count)
        return {"events": events, "count": len(events)}

    @router.get("/stats")
    async def get_stats():
        """Return plugin statistics."""
        return plugin.get_stats()

    @router.get("/counts")
    async def get_event_counts():
        """Return event type counts."""
        return {"counts": plugin.get_event_counts()}

    @router.get("/health")
    async def get_health():
        """Return plugin health status."""
        return {
            "healthy": plugin.healthy,
            "plugin_id": plugin.plugin_id,
            "version": plugin.version,
        }

    return router
