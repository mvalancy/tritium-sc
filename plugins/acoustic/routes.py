# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""FastAPI routes for acoustic intelligence plugin.

Provides REST endpoints for audio classification (rule-based + ML),
event timeline, sound source localization, and sensor management.
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
    # MFCC features for ML classification
    mfcc: Optional[list[float]] = None
    spectral_rolloff: float = 0.0
    spectral_flatness: float = 0.0


class SensorRegisterRequest(BaseModel):
    """Register a sensor position for acoustic localization."""
    sensor_id: str
    lat: float
    lon: float


class LocalizeRequest(BaseModel):
    """Request body for manual localization from multiple observations."""
    event_type: str = "unknown"
    observers: list[dict] = []


def create_router(plugin: Any) -> APIRouter:
    """Build acoustic intelligence API router.

    Parameters
    ----------
    plugin:
        AcousticPlugin instance.
    """
    router = APIRouter(prefix="/api/acoustic", tags=["acoustic"])

    @router.post("/classify")
    async def classify_audio(body: ClassifyRequest):
        """Classify audio features and return the detection event.

        Supports both basic features and MFCC vectors for ML classification.
        """
        location = None
        if body.lat is not None and body.lng is not None:
            location = (body.lat, body.lng)

        features = {
            "rms_energy": body.rms_energy,
            "peak_amplitude": body.peak_amplitude,
            "zero_crossing_rate": body.zero_crossing_rate,
            "spectral_centroid": body.spectral_centroid,
            "spectral_bandwidth": body.spectral_bandwidth,
            "duration_ms": body.duration_ms,
            "spectral_rolloff": body.spectral_rolloff,
            "spectral_flatness": body.spectral_flatness,
        }
        if body.mfcc:
            features["mfcc"] = body.mfcc

        result = plugin.classify_audio(
            features=features,
            device_id=body.device_id,
            location=location,
        )
        return result

    @router.get("/events")
    async def get_events(count: int = 50):
        """Return recent classified acoustic events."""
        events = plugin.get_recent_events(count)
        return {"events": events, "count": len(events)}

    @router.get("/timeline")
    async def get_timeline(count: int = 100):
        """Return acoustic event timeline with severity/color data.

        Events include severity classification and color coding
        for frontend timeline rendering.
        """
        events = plugin.get_timeline(count)
        return {"events": events, "count": len(events)}

    @router.get("/localizations")
    async def get_localizations(count: int = 50):
        """Return recent sound source localization results.

        Returns triangulated positions from multi-node TDoA.
        """
        results = plugin.get_localizations(count)
        return {"localizations": results, "count": len(results)}

    @router.post("/sensors/register")
    async def register_sensor(body: SensorRegisterRequest):
        """Register a sensor's position for acoustic localization."""
        plugin.register_sensor(body.sensor_id, body.lat, body.lon)
        return {
            "status": "registered",
            "sensor_id": body.sensor_id,
            "lat": body.lat,
            "lon": body.lon,
        }

    @router.post("/localize")
    async def localize_sound(body: LocalizeRequest):
        """Manually submit multi-observer data for source localization.

        Provide a list of observers with sensor_id, lat, lon, arrival_time.
        Returns the estimated source position.
        """
        try:
            from tritium_lib.models.acoustic_intelligence import acoustic_trilaterate
            result = acoustic_trilaterate(body.observers)
        except ImportError:
            result = plugin._simple_localize(body.observers)

        if result:
            return {"localization": result, "status": "ok"}
        return {"localization": None, "status": "insufficient_data"}

    @router.get("/stats")
    async def get_stats():
        """Return plugin statistics including ML classifier status."""
        return plugin.get_stats()

    @router.get("/counts")
    async def get_event_counts():
        """Return event type counts."""
        return {"counts": plugin.get_event_counts()}

    @router.get("/health")
    async def get_health():
        """Return plugin health status."""
        stats = plugin.get_stats()
        return {
            "healthy": plugin.healthy,
            "plugin_id": plugin.plugin_id,
            "version": plugin.version,
            "ml_available": stats.get("ml_available", False),
            "sensors_registered": stats.get("sensors_registered", 0),
        }

    return router
