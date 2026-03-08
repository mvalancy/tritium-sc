# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Test fleet proxy endpoints — correlations, topology, heap-trends."""

import json
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from app.main import app


@pytest.fixture
def client():
    """Create test client."""
    return TestClient(app)


class TestFleetCorrelations:
    """Test /api/fleet/correlations proxy endpoint."""

    def test_correlations_unavailable(self, client):
        """Returns unavailable when fleet server is unreachable."""
        with patch("app.routers.fleet._proxy_get", return_value=None):
            response = client.get("/api/fleet/correlations")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "unavailable"
            assert data["correlations"] == []
            assert data["count"] == 0

    def test_correlations_live(self, client):
        """Returns live data when fleet server responds."""
        mock_data = {
            "correlations": [
                {
                    "type": "synchronized_reboot",
                    "affected_nodes": ["node-a", "node-b"],
                    "confidence": 0.92,
                    "timestamp": 1741400000,
                    "description": "Two nodes rebooted within 5s window",
                },
                {
                    "type": "cascading_failure",
                    "affected_nodes": ["node-c", "node-d", "node-e"],
                    "confidence": 0.78,
                    "timestamp": 1741399000,
                    "description": "Sequential WiFi disconnects across subnet",
                },
            ],
            "count": 2,
        }
        with patch("app.routers.fleet._proxy_get", return_value=mock_data):
            response = client.get("/api/fleet/correlations")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "live"
            assert len(data["correlations"]) == 2
            assert data["correlations"][0]["type"] == "synchronized_reboot"
            assert data["correlations"][0]["confidence"] == 0.92

    def test_correlations_list_response(self, client):
        """Handles list response from fleet server (no wrapper dict)."""
        mock_list = [
            {"type": "reboot_sync", "affected_nodes": ["node-a"], "confidence": 0.5},
        ]
        with patch("app.routers.fleet._proxy_get", return_value=mock_list):
            response = client.get("/api/fleet/correlations")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "live"
            assert "correlations" in data

    def test_correlations_cached_fallback(self, client):
        """Falls back to cached data from bridge."""
        mock_bridge = MagicMock()
        mock_bridge.correlations = {
            "correlations": [{"type": "cached_event", "confidence": 0.6}],
            "count": 1,
        }
        app.state.fleet_bridge = mock_bridge
        try:
            with patch("app.routers.fleet._proxy_get", return_value=None):
                response = client.get("/api/fleet/correlations")
                assert response.status_code == 200
                data = response.json()
                assert data["source"] == "cached"
                assert len(data["correlations"]) == 1
        finally:
            del app.state.fleet_bridge


class TestFleetTopology:
    """Test /api/fleet/topology proxy endpoint."""

    def test_topology_unavailable(self, client):
        """Returns unavailable when fleet server is unreachable."""
        with patch("app.routers.fleet._proxy_get", return_value=None):
            response = client.get("/api/fleet/topology")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "unavailable"
            assert data["nodes"] == []
            assert data["edges"] == []

    def test_topology_live(self, client):
        """Returns live topology data."""
        mock_data = {
            "nodes": [
                {"id": "node-a", "ip": "192.168.86.10", "role": "sensor"},
                {"id": "node-b", "ip": "192.168.86.11", "role": "sensor"},
                {"id": "server", "ip": "192.168.86.9", "role": "server"},
            ],
            "edges": [
                {"from": "node-a", "to": "server", "quality": 0.95, "latency_ms": 12, "type": "wifi"},
                {"from": "node-b", "to": "server", "quality": 0.87, "latency_ms": 18, "type": "wifi"},
            ],
        }
        with patch("app.routers.fleet._proxy_get", return_value=mock_data):
            response = client.get("/api/fleet/topology")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "live"
            assert len(data["nodes"]) == 3
            assert len(data["edges"]) == 2
            assert data["edges"][0]["quality"] == 0.95

    def test_topology_list_response(self, client):
        """Handles list response (nodes only, no wrapper dict)."""
        mock_list = [{"id": "node-a"}, {"id": "node-b"}]
        with patch("app.routers.fleet._proxy_get", return_value=mock_list):
            response = client.get("/api/fleet/topology")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "live"
            assert "nodes" in data

    def test_topology_cached_fallback(self, client):
        """Falls back to cached data from bridge."""
        mock_bridge = MagicMock()
        mock_bridge.topology = {
            "nodes": [{"id": "cached-node"}],
            "edges": [],
        }
        app.state.fleet_bridge = mock_bridge
        try:
            with patch("app.routers.fleet._proxy_get", return_value=None):
                response = client.get("/api/fleet/topology")
                assert response.status_code == 200
                data = response.json()
                assert data["source"] == "cached"
                assert len(data["nodes"]) == 1
        finally:
            del app.state.fleet_bridge


class TestFleetHeapTrends:
    """Test /api/fleet/heap-trends proxy endpoint."""

    def test_heap_trends_unavailable(self, client):
        """Returns unavailable when fleet server is unreachable."""
        with patch("app.routers.fleet._proxy_get", return_value=None):
            response = client.get("/api/fleet/heap-trends")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "unavailable"
            assert data["trends"] == []
            assert data["leak_suspects"] == []

    def test_heap_trends_live(self, client):
        """Returns live heap trend data with leak suspects."""
        mock_data = {
            "trends": [
                {
                    "device_id": "node-a",
                    "trend": "declining",
                    "current_heap": 120000,
                    "rate_bytes_per_min": -256,
                    "projected_exhaustion_min": 468,
                    "suspected_leak": True,
                },
                {
                    "device_id": "node-b",
                    "trend": "stable",
                    "current_heap": 245000,
                    "rate_bytes_per_min": 0,
                    "suspected_leak": False,
                },
            ],
            "leak_suspects": [
                {
                    "device_id": "node-a",
                    "rate_bytes_per_min": -256,
                    "projected_exhaustion_min": 468,
                    "current_heap": 120000,
                },
            ],
        }
        with patch("app.routers.fleet._proxy_get", return_value=mock_data):
            response = client.get("/api/fleet/heap-trends")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "live"
            assert len(data["trends"]) == 2
            assert len(data["leak_suspects"]) == 1
            assert data["trends"][0]["suspected_leak"] is True
            assert data["leak_suspects"][0]["device_id"] == "node-a"

    def test_heap_trends_list_response(self, client):
        """Handles list response (trends only, no wrapper dict)."""
        mock_list = [{"device_id": "node-a", "trend": "stable"}]
        with patch("app.routers.fleet._proxy_get", return_value=mock_list):
            response = client.get("/api/fleet/heap-trends")
            assert response.status_code == 200
            data = response.json()
            assert data["source"] == "live"
            assert "trends" in data

    def test_heap_trends_cached_fallback(self, client):
        """Falls back to cached data from bridge."""
        mock_bridge = MagicMock()
        mock_bridge.heap_trends = {
            "trends": [{"device_id": "cached-node", "trend": "declining"}],
            "leak_suspects": [],
        }
        app.state.fleet_bridge = mock_bridge
        try:
            with patch("app.routers.fleet._proxy_get", return_value=None):
                response = client.get("/api/fleet/heap-trends")
                assert response.status_code == 200
                data = response.json()
                assert data["source"] == "cached"
                assert len(data["trends"]) == 1
        finally:
            del app.state.fleet_bridge
