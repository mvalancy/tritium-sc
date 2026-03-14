# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""System runtime metrics endpoint for monitoring dashboards.

    GET /api/system/metrics — uptime, memory, WebSocket connections, target count,
                              request count, error count, route count
"""

from __future__ import annotations

import os
import time

from fastapi import APIRouter, Request

router = APIRouter(prefix="/api/system", tags=["system"])


def _get_memory_mb() -> dict:
    """Get current process memory usage via /proc/self/status (Linux)."""
    try:
        import resource
        usage = resource.getrusage(resource.RUSAGE_SELF)
        rss_mb = usage.ru_maxrss / 1024  # Linux: ru_maxrss is in KB
        return {"rss_mb": round(rss_mb, 2)}
    except Exception:
        pass

    # Fallback: try tracemalloc if available
    try:
        import tracemalloc
        if tracemalloc.is_tracing():
            current, peak = tracemalloc.get_traced_memory()
            return {
                "traced_current_mb": round(current / 1024 / 1024, 2),
                "traced_peak_mb": round(peak / 1024 / 1024, 2),
            }
    except Exception:
        pass

    # Fallback: /proc/self/statm
    try:
        with open("/proc/self/statm") as f:
            parts = f.read().split()
            page_size = os.sysconf("SC_PAGE_SIZE")
            rss_pages = int(parts[1])
            return {"rss_mb": round(rss_pages * page_size / 1024 / 1024, 2)}
    except Exception:
        return {"rss_mb": -1}


def _get_ws_connections(request: Request) -> int:
    """Count active WebSocket connections from ws router."""
    try:
        from app.routers.ws import manager
        return len(manager.active_connections)
    except Exception:
        return 0


def _get_target_count(request: Request) -> int:
    """Get current tracked target count."""
    amy = getattr(request.app.state, "amy", None)
    if amy is not None:
        tracker = getattr(amy, "target_tracker", None)
        if tracker is not None:
            try:
                return len(tracker.get_all())
            except Exception:
                pass

    engine = getattr(request.app.state, "simulation_engine", None)
    if engine is not None:
        try:
            return len(engine.get_targets())
        except Exception:
            pass

    return 0


def _get_uptime() -> float:
    """Get system uptime in seconds."""
    try:
        from app.routers.health import _start_time
        return round(time.time() - _start_time, 1)
    except Exception:
        return 0.0


def _get_request_stats(request: Request) -> dict:
    """Get request/error counts from audit middleware if available."""
    try:
        from app.audit_middleware import get_audit_stats
        return get_audit_stats()
    except Exception:
        return {"total_requests": -1, "total_errors": -1}


@router.get("/metrics")
async def get_system_metrics(request: Request):
    """Runtime metrics for monitoring dashboards.

    Returns:
        - uptime_seconds: time since server start
        - memory: RSS memory usage in MB
        - websocket_connections: active WebSocket connection count
        - target_count: currently tracked targets
        - route_count: total registered API routes
        - request_stats: total requests and errors (if audit middleware active)
        - pid: server process ID
    """
    return {
        "uptime_seconds": _get_uptime(),
        "memory": _get_memory_mb(),
        "websocket_connections": _get_ws_connections(request),
        "target_count": _get_target_count(request),
        "route_count": len(request.app.routes),
        "request_stats": _get_request_stats(request),
        "pid": os.getpid(),
        "timestamp": time.time(),
    }
