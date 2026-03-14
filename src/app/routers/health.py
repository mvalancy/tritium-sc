# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""System health endpoint.

Returns system status, subsystem health, plugin health, uptime, and
test baselines. Used by Docker HEALTHCHECK and monitoring systems.
"""

import time
from typing import Any

from fastapi import APIRouter, Request

router = APIRouter(tags=["health"])

# Set at import time; updated to real value when lifespan starts.
_start_time: float = time.time()


def reset_start_time() -> None:
    """Reset the start time (called during lifespan startup)."""
    global _start_time
    _start_time = time.time()


def _subsystem_status(request: Request) -> dict[str, str]:
    """Check health of core subsystems attached to app.state."""
    checks: dict[str, str] = {}

    # Amy AI Commander
    amy = getattr(request.app.state, "amy", None)
    checks["amy"] = "running" if amy is not None else "disabled"

    # MQTT bridge
    mqtt = getattr(request.app.state, "mqtt_bridge", None)
    if mqtt is not None:
        connected = getattr(mqtt, "connected", False)
        checks["mqtt"] = "connected" if connected else "disconnected"
    else:
        checks["mqtt"] = "disabled"

    # Simulation engine
    sim = getattr(request.app.state, "simulation_engine", None)
    if sim is None and amy is not None:
        sim = getattr(amy, "simulation_engine", None)
    checks["simulation"] = "running" if sim is not None else "disabled"

    # Plugin manager
    pm = getattr(request.app.state, "plugin_manager", None)
    if pm is not None:
        plugins = pm.list_plugins()
        running = sum(1 for p in plugins if p.get("running", False))
        checks["plugins"] = f"{running}/{len(plugins)} running"
    else:
        checks["plugins"] = "disabled"

    # Demo mode
    demo = getattr(request.app.state, "demo_controller", None)
    if demo is not None and getattr(demo, "active", False):
        checks["demo"] = "active"

    # Fleet bridge
    fleet = getattr(request.app.state, "fleet_bridge", None)
    if fleet is not None:
        checks["fleet_bridge"] = "connected"

    # Meshtastic bridge
    mesh = getattr(request.app.state, "meshtastic_bridge", None)
    if mesh is not None:
        checks["meshtastic"] = "connected"

    return checks


def _plugin_health(request: Request) -> dict[str, Any]:
    """Detailed plugin health from PluginManager."""
    pm = getattr(request.app.state, "plugin_manager", None)
    if pm is None:
        return {}
    try:
        return pm.health_check()
    except Exception:
        return {"error": "health check failed"}


@router.get("/api/health")
async def health_check(request: Request):
    """Comprehensive health check endpoint.

    Returns system status, subsystem health, plugin health, uptime,
    and test baselines. Used by Docker HEALTHCHECK, load balancers,
    and monitoring dashboards.
    """
    uptime_seconds = time.time() - _start_time
    subsystems = _subsystem_status(request)
    plugins = _plugin_health(request)

    # Overall status: degraded if any critical subsystem is down
    # but still responding (the endpoint itself proves the app is alive)
    all_healthy = True
    for key, val in subsystems.items():
        if val == "disconnected":
            all_healthy = False
            break

    return {
        "status": "healthy" if all_healthy else "degraded",
        "version": "0.1.0",
        "system": "TRITIUM-SC",
        "uptime_seconds": round(uptime_seconds, 1),
        "subsystems": subsystems,
        "plugins": plugins,
        "test_baselines": {
            "tritium_lib": 833,
            "tritium_sc_pytest": 7666,
            "tritium_sc_js": 281,
            "tritium_edge_warnings": 0,
        },
    }
