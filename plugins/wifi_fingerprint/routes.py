# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""FastAPI routes for the WiFi Fingerprint plugin.

Provides REST endpoints for viewing probe correlations, WiFi fingerprints,
and correlation links between WiFi and BLE devices.
"""
from __future__ import annotations

from typing import Optional

from fastapi import APIRouter, HTTPException

from .correlator import ProbeCorrelator


def create_router(correlator: ProbeCorrelator) -> APIRouter:
    """Build and return the wifi-fingerprint APIRouter."""

    router = APIRouter(prefix="/api/wifi-fingerprint", tags=["wifi-fingerprint"])

    @router.get("/status")
    async def get_status():
        """Get correlator status and statistics."""
        return correlator.get_status()

    @router.get("/links")
    async def get_links(min_score: float = 0.3):
        """Get all WiFi-BLE correlation links above threshold."""
        links = correlator.get_all_links(min_score=min_score)
        return {"links": links, "count": len(links)}

    @router.get("/fingerprint/{wifi_mac}")
    async def get_fingerprint(wifi_mac: str):
        """Get WiFi fingerprint (probed SSIDs) for a device by MAC."""
        fp = correlator.get_fingerprint(wifi_mac)
        if not fp["probed_ssids"] and fp["probe_count"] == 0:
            raise HTTPException(status_code=404, detail="WiFi MAC not found")
        return fp

    @router.get("/correlations/{ble_mac}")
    async def get_correlations(ble_mac: str):
        """Get WiFi correlations for a BLE device MAC."""
        enrichment = correlator.get_dossier_enrichment(ble_mac)
        if not enrichment:
            return {"wifi_correlations": [], "all_probed_ssids": [], "strongest_score": 0.0}
        return enrichment

    @router.get("/ble-links/{ble_mac}")
    async def get_ble_links(ble_mac: str):
        """Get all WiFi correlation links for a BLE device."""
        links = correlator.get_links_for_ble(ble_mac)
        return {"links": [l.to_dict() for l in links], "count": len(links)}

    @router.post("/prune")
    async def prune_stale(max_age: float = 3600.0):
        """Prune stale records and decay old correlation scores."""
        pruned = correlator.prune_stale(max_age=max_age)
        return {"pruned": pruned, "status": correlator.get_status()}

    return router
