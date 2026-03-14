# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for /api/system/metrics endpoint."""

import pytest
from fastapi.testclient import TestClient


@pytest.fixture
def client():
    from app.main import app
    return TestClient(app)


class TestSystemMetrics:
    """Verify /api/system/metrics returns expected structure."""

    def test_metrics_returns_200(self, client):
        r = client.get("/api/system/metrics")
        assert r.status_code == 200

    def test_metrics_has_uptime(self, client):
        data = client.get("/api/system/metrics").json()
        assert "uptime_seconds" in data
        assert isinstance(data["uptime_seconds"], (int, float))

    def test_metrics_has_memory(self, client):
        data = client.get("/api/system/metrics").json()
        assert "memory" in data
        assert isinstance(data["memory"], dict)

    def test_metrics_has_route_count(self, client):
        data = client.get("/api/system/metrics").json()
        assert "route_count" in data
        assert data["route_count"] >= 400  # sanity — we know there are 418+

    def test_metrics_has_ws_connections(self, client):
        data = client.get("/api/system/metrics").json()
        assert "websocket_connections" in data
        assert data["websocket_connections"] == 0  # no WS connected in test

    def test_metrics_has_target_count(self, client):
        data = client.get("/api/system/metrics").json()
        assert "target_count" in data

    def test_metrics_has_pid(self, client):
        data = client.get("/api/system/metrics").json()
        assert "pid" in data
        assert data["pid"] > 0

    def test_metrics_has_timestamp(self, client):
        data = client.get("/api/system/metrics").json()
        assert "timestamp" in data
        assert data["timestamp"] > 0


class TestMissionSecurity:
    """Verify mission input validation and sanitization."""

    def test_create_mission_sanitizes_title(self, client):
        r = client.post("/api/missions", json={
            "title": "<script>alert('xss')</script>Recon",
            "type": "custom",
        })
        assert r.status_code == 201
        data = r.json()
        assert "<script>" not in data["title"]
        # Clean up
        client.delete(f"/api/missions/{data['mission_id']}")

    def test_create_mission_rejects_invalid_priority(self, client):
        r = client.post("/api/missions", json={
            "title": "Test",
            "type": "custom",
            "priority": 999,
        })
        assert r.status_code == 422

    def test_create_mission_rejects_empty_title(self, client):
        r = client.post("/api/missions", json={
            "title": "",
            "type": "custom",
        })
        assert r.status_code == 422


class TestBookmarkSecurity:
    """Verify bookmark input validation."""

    def test_create_bookmark_lat_bounds(self, client):
        r = client.post("/api/bookmarks", json={
            "name": "Test",
            "lat": 999,
            "lng": 0,
        })
        assert r.status_code == 422

    def test_create_bookmark_zoom_bounds(self, client):
        r = client.post("/api/bookmarks", json={
            "name": "Test",
            "lat": 0,
            "lng": 0,
            "zoom": 999,
        })
        assert r.status_code == 422

    def test_create_bookmark_sanitizes_name(self, client):
        r = client.post("/api/bookmarks", json={
            "name": "<b>HQ</b>",
            "lat": 30.0,
            "lng": -97.0,
        })
        assert r.status_code == 200
        data = r.json()
        assert "<b>" not in data["name"]
