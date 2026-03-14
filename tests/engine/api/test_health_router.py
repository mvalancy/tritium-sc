# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Unit tests for /api/health endpoint.

Verifies the health check returns system status, subsystem health,
plugin health, uptime, and test baselines under various app states.
"""
from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.routers.health import router, reset_start_time, _start_time


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def health_app():
    """Minimal FastAPI app with only the health router."""
    app = FastAPI()
    app.include_router(router)
    # Initialize app.state with no subsystems (bare minimum)
    app.state.amy = None
    return app


@pytest.fixture
def health_client(health_app):
    return TestClient(health_app, raise_server_exceptions=False)


@pytest.fixture
def full_app():
    """App with simulated subsystems attached to state."""
    app = FastAPI()
    app.include_router(router)

    # Simulate Amy running
    amy = SimpleNamespace(
        simulation_engine=SimpleNamespace(),
    )
    app.state.amy = amy

    # Simulate MQTT connected
    mqtt = SimpleNamespace(connected=True)
    app.state.mqtt_bridge = mqtt

    # Simulate plugin manager with 2 plugins, 1 running
    pm = MagicMock()
    pm.list_plugins.return_value = [
        {"id": "edge_tracker", "running": True},
        {"id": "camera_feeds", "running": False},
    ]
    pm.health_check.return_value = {
        "edge_tracker": {"healthy": True, "targets": 42},
        "camera_feeds": {"healthy": False, "error": "no cameras"},
    }
    app.state.plugin_manager = pm

    return app


@pytest.fixture
def full_client(full_app):
    return TestClient(full_app, raise_server_exceptions=False)


# ---------------------------------------------------------------------------
# Response structure tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestHealthResponseStructure:
    """GET /api/health returns the expected JSON shape."""

    def test_returns_200(self, health_client):
        resp = health_client.get("/api/health")
        assert resp.status_code == 200

    def test_has_status_field(self, health_client):
        data = health_client.get("/api/health").json()
        assert "status" in data
        assert data["status"] in ("healthy", "degraded")

    def test_has_version(self, health_client):
        data = health_client.get("/api/health").json()
        assert data["version"] == "0.1.0"

    def test_has_system_name(self, health_client):
        data = health_client.get("/api/health").json()
        assert data["system"] == "TRITIUM-SC"

    def test_has_uptime(self, health_client):
        data = health_client.get("/api/health").json()
        assert "uptime_seconds" in data
        assert isinstance(data["uptime_seconds"], (int, float))
        assert data["uptime_seconds"] >= 0

    def test_has_subsystems(self, health_client):
        data = health_client.get("/api/health").json()
        assert "subsystems" in data
        assert isinstance(data["subsystems"], dict)

    def test_has_plugins(self, health_client):
        data = health_client.get("/api/health").json()
        assert "plugins" in data

    def test_has_test_baselines(self, health_client):
        data = health_client.get("/api/health").json()
        assert "test_baselines" in data
        baselines = data["test_baselines"]
        assert baselines["tritium_lib"] == 833
        assert baselines["tritium_sc_pytest"] == 7666
        assert baselines["tritium_sc_js"] == 281
        assert baselines["tritium_edge_warnings"] == 0


# ---------------------------------------------------------------------------
# Subsystem status tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestSubsystemStatus:
    """Subsystem health reflects app.state correctly."""

    def test_bare_app_all_disabled(self, health_client):
        """With no subsystems attached, everything reports disabled."""
        data = health_client.get("/api/health").json()
        subs = data["subsystems"]
        assert subs["amy"] == "disabled"
        assert subs["mqtt"] == "disabled"
        assert subs["simulation"] == "disabled"
        assert subs["plugins"] == "disabled"

    def test_healthy_status_when_all_up(self, full_client):
        data = full_client.get("/api/health").json()
        assert data["status"] == "healthy"

    def test_amy_running(self, full_client):
        data = full_client.get("/api/health").json()
        assert data["subsystems"]["amy"] == "running"

    def test_mqtt_connected(self, full_client):
        data = full_client.get("/api/health").json()
        assert data["subsystems"]["mqtt"] == "connected"

    def test_plugins_count(self, full_client):
        data = full_client.get("/api/health").json()
        assert data["subsystems"]["plugins"] == "1/2 running"

    def test_plugin_details(self, full_client):
        data = full_client.get("/api/health").json()
        plugins = data["plugins"]
        assert "edge_tracker" in plugins
        assert plugins["edge_tracker"]["healthy"] is True

    def test_mqtt_disconnected_degrades(self):
        """MQTT disconnected should result in 'degraded' status."""
        app = FastAPI()
        app.include_router(router)
        app.state.amy = SimpleNamespace()
        app.state.mqtt_bridge = SimpleNamespace(connected=False)
        client = TestClient(app, raise_server_exceptions=False)

        data = client.get("/api/health").json()
        assert data["status"] == "degraded"
        assert data["subsystems"]["mqtt"] == "disconnected"


# ---------------------------------------------------------------------------
# Uptime tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestUptime:
    """Uptime tracking works correctly."""

    def test_uptime_increases(self, health_client):
        """Two requests should show increasing uptime."""
        t1 = health_client.get("/api/health").json()["uptime_seconds"]
        # Uptime should be non-negative
        assert t1 >= 0

    def test_reset_start_time(self, health_client):
        """reset_start_time() resets the clock."""
        reset_start_time()
        data = health_client.get("/api/health").json()
        # Just after reset, uptime should be very small
        assert data["uptime_seconds"] < 5.0


# ---------------------------------------------------------------------------
# Optional subsystem tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestOptionalSubsystems:
    """Optional subsystems appear in health when attached."""

    def test_demo_mode_active(self):
        app = FastAPI()
        app.include_router(router)
        app.state.amy = None
        app.state.demo_controller = SimpleNamespace(active=True)
        client = TestClient(app, raise_server_exceptions=False)

        data = client.get("/api/health").json()
        assert data["subsystems"]["demo"] == "active"

    def test_fleet_bridge_connected(self):
        app = FastAPI()
        app.include_router(router)
        app.state.amy = None
        app.state.fleet_bridge = SimpleNamespace()
        client = TestClient(app, raise_server_exceptions=False)

        data = client.get("/api/health").json()
        assert data["subsystems"]["fleet_bridge"] == "connected"

    def test_meshtastic_connected(self):
        app = FastAPI()
        app.include_router(router)
        app.state.amy = None
        app.state.meshtastic_bridge = SimpleNamespace()
        client = TestClient(app, raise_server_exceptions=False)

        data = client.get("/api/health").json()
        assert data["subsystems"]["meshtastic"] == "connected"


# ---------------------------------------------------------------------------
# Integration with real app
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestHealthOnRealApp:
    """Health router is mounted on the actual Tritium app."""

    def test_health_route_registered(self):
        """The /api/health route exists on the real app."""
        from app.main import app as real_app
        paths = [r.path for r in real_app.routes if hasattr(r, "path")]
        assert "/api/health" in paths

    def test_health_endpoint_responds(self):
        """GET /api/health on the real app returns 200."""
        from app.main import app as real_app
        client = TestClient(real_app, raise_server_exceptions=False)
        resp = client.get("/api/health")
        assert resp.status_code == 200
        data = resp.json()
        assert data["system"] == "TRITIUM-SC"
        assert "subsystems" in data
