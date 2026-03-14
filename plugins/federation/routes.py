# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""FastAPI routes for the Federation plugin."""

from __future__ import annotations

from typing import TYPE_CHECKING

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

if TYPE_CHECKING:
    from .plugin import FederationPlugin

try:
    from tritium_lib.models.federation import FederatedSite, SharedTarget
except ImportError:  # pragma: no cover
    FederatedSite = None  # type: ignore[assignment,misc]
    SharedTarget = None  # type: ignore[assignment,misc]


class AddSiteRequest(BaseModel):
    """Request body for adding a federated site."""
    name: str = "Remote Site"
    description: str = ""
    mqtt_host: str = ""
    mqtt_port: int = 1883
    mqtt_username: str = ""
    mqtt_password: str = ""
    role: str = "peer"
    share_policy: str = "targets_only"
    lat: float | None = None
    lng: float | None = None
    tags: list[str] = []
    enabled: bool = True


def create_router(plugin: FederationPlugin) -> APIRouter:
    """Create and return the federation API router."""
    router = APIRouter(prefix="/api/federation", tags=["federation"])

    @router.get("/sites")
    async def list_sites():
        """List all federated sites with connection status."""
        return {"sites": plugin.list_sites()}

    @router.get("/sites/{site_id}")
    async def get_site(site_id: str):
        """Get a specific federated site."""
        site = plugin.get_site(site_id)
        if site is None:
            raise HTTPException(status_code=404, detail="Site not found")
        conn = plugin.get_connection(site_id)
        result = site.model_dump()
        if conn:
            result["connection"] = conn.model_dump()
        return result

    @router.post("/sites")
    async def add_site(req: AddSiteRequest):
        """Register a new federated site."""
        if FederatedSite is None:
            raise HTTPException(
                status_code=503,
                detail="Federation models not available"
            )
        site = FederatedSite(
            name=req.name,
            description=req.description,
            mqtt_host=req.mqtt_host,
            mqtt_port=req.mqtt_port,
            mqtt_username=req.mqtt_username,
            mqtt_password=req.mqtt_password,
            role=req.role,
            share_policy=req.share_policy,
            lat=req.lat,
            lng=req.lng,
            tags=req.tags,
            enabled=req.enabled,
        )
        site_id = plugin.add_site(site)
        return {"site_id": site_id, "name": site.name}

    @router.delete("/sites/{site_id}")
    async def remove_site(site_id: str):
        """Remove a federated site."""
        if not plugin.remove_site(site_id):
            raise HTTPException(status_code=404, detail="Site not found")
        return {"status": "removed", "site_id": site_id}

    @router.get("/targets")
    async def get_shared_targets():
        """Get all targets shared from federated sites."""
        return {"targets": plugin.get_shared_targets()}

    @router.get("/stats")
    async def get_stats():
        """Get federation statistics."""
        return plugin.get_stats()

    return router
