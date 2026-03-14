# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""API version router — serves version metadata at /api/version."""

from __future__ import annotations

import platform
import sys
from datetime import datetime, timezone

from fastapi import APIRouter

router = APIRouter(tags=["version"])

_BOOT_TIME = datetime.now(timezone.utc).isoformat()

API_VERSION_INFO = {
    "api_version": "v1",
    "app": "TRITIUM-SC",
    "app_version": "0.1.0",
    "supported_versions": ["v1"],
    "deprecated_versions": [],
    "python_version": f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}",
    "platform": platform.system(),
}


@router.get("/api/version")
async def api_version():
    """Return API version information and supported version namespaces."""
    return {
        **API_VERSION_INFO,
        "server_boot": _BOOT_TIME,
    }


@router.get("/api/v1/version")
async def api_v1_version():
    """Return version info under the v1 namespace."""
    return {
        **API_VERSION_INFO,
        "namespace": "/api/v1",
        "server_boot": _BOOT_TIME,
    }
