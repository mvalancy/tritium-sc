# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Unit tests for the target search/filter API endpoints.

Tests /api/targets/search, /api/targets/filter, /api/targets/stats,
and /api/targets/{id} using a mock TargetTracker.
"""
from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.routers.target_search import router


# ---------------------------------------------------------------------------
# Helpers — mock TrackedTarget and TargetTracker
# ---------------------------------------------------------------------------

class MockTrackedTarget:
    """Minimal stand-in for engine.tactical.target_tracker.TrackedTarget."""

    def __init__(self, target_id, name, alliance, asset_type, source,
                 position=(0.0, 0.0), heading=0.0, speed=0.0, battery=1.0,
                 last_seen=None, status="active"):
        self.target_id = target_id
        self.name = name
        self.alliance = alliance
        self.asset_type = asset_type
        self.source = source
        self.position = position
        self.heading = heading
        self.speed = speed
        self.battery = battery
        self.last_seen = last_seen if last_seen is not None else time.monotonic()
        self.status = status

    def to_dict(self, history=None):
        d = {
            "target_id": self.target_id,
            "name": self.name,
            "alliance": self.alliance,
            "asset_type": self.asset_type,
            "position": {"x": self.position[0], "y": self.position[1]},
            "heading": self.heading,
            "speed": self.speed,
            "battery": self.battery,
            "last_seen": self.last_seen,
            "source": self.source,
            "status": self.status,
        }
        if history is not None:
            d["trail"] = []
        return d


class MockTargetTracker:
    """Minimal stand-in for TargetTracker with controllable targets."""

    def __init__(self, targets: list[MockTrackedTarget]):
        self._targets = {t.target_id: t for t in targets}
        self.history = MagicMock()
        self.history.get_trail_dicts.return_value = []

    def get_all(self):
        return list(self._targets.values())

    def get_target(self, target_id):
        return self._targets.get(target_id)


def _make_client(targets: list[MockTrackedTarget] | None = None):
    """Create a test client with mock targets."""
    app = FastAPI()
    app.include_router(router)

    tracker = MockTargetTracker(targets or [])
    amy = MagicMock()
    amy.target_tracker = tracker
    app.state.amy = amy
    app.state.simulation_engine = None

    return TestClient(app)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def sample_targets():
    """A set of diverse targets for testing search and filter."""
    now = time.monotonic()
    return [
        MockTrackedTarget("rover_alpha", "Alpha Rover", "friendly", "rover", "simulation",
                          position=(1.0, 2.0), last_seen=now),
        MockTrackedTarget("drone_beta", "Beta Drone", "friendly", "drone", "simulation",
                          position=(3.0, 4.0), last_seen=now),
        MockTrackedTarget("det_person_1", "Person #1", "hostile", "person", "yolo",
                          position=(5.0, 6.0), last_seen=now),
        MockTrackedTarget("ble_aabbccddee", "iPhone-Matt", "unknown", "ble_device", "ble",
                          position=(7.0, 8.0), last_seen=now),
        MockTrackedTarget("det_vehicle_1", "Vehicle #1", "unknown", "vehicle", "yolo",
                          position=(9.0, 10.0), last_seen=now - 600),
    ]


@pytest.fixture
def client(sample_targets):
    return _make_client(sample_targets)


# ---------------------------------------------------------------------------
# Tests: GET /api/targets/search
# ---------------------------------------------------------------------------

class TestTargetSearch:
    def test_search_by_name(self, client):
        resp = client.get("/api/targets/search?q=Alpha")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 1
        assert data["results"][0]["target_id"] == "rover_alpha"

    def test_search_by_id(self, client):
        resp = client.get("/api/targets/search?q=ble_aabb")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 1
        assert data["results"][0]["name"] == "iPhone-Matt"

    def test_search_case_insensitive(self, client):
        resp = client.get("/api/targets/search?q=iphone")
        assert resp.status_code == 200
        assert resp.json()["total"] == 1

    def test_search_by_asset_type(self, client):
        resp = client.get("/api/targets/search?q=rover")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] >= 1

    def test_search_by_alliance(self, client):
        resp = client.get("/api/targets/search?q=hostile")
        assert resp.status_code == 200
        assert resp.json()["total"] >= 1

    def test_search_no_results(self, client):
        resp = client.get("/api/targets/search?q=zzzznotfound")
        assert resp.status_code == 200
        assert resp.json()["total"] == 0

    def test_search_empty_query_rejected(self, client):
        resp = client.get("/api/targets/search?q=")
        assert resp.status_code == 422  # validation error

    def test_search_multiple_results(self, client):
        # "drone" and "person" both have "o" but let's search for something broader
        resp = client.get("/api/targets/search?q=det_")
        assert resp.status_code == 200
        assert resp.json()["total"] == 2  # det_person_1 and det_vehicle_1


# ---------------------------------------------------------------------------
# Tests: GET /api/targets/filter
# ---------------------------------------------------------------------------

class TestTargetFilter:
    def test_filter_by_source(self, client):
        resp = client.get("/api/targets/filter?source=yolo")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 2
        for t in data["targets"]:
            assert t["source"] == "yolo"

    def test_filter_by_alliance(self, client):
        resp = client.get("/api/targets/filter?alliance=friendly")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 2
        for t in data["targets"]:
            assert t["alliance"] == "friendly"

    def test_filter_by_asset_type(self, client):
        resp = client.get("/api/targets/filter?asset_type=drone")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 1
        assert data["targets"][0]["target_id"] == "drone_beta"

    def test_filter_combined(self, client):
        resp = client.get("/api/targets/filter?source=simulation&alliance=friendly")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 2

    def test_filter_no_match(self, client):
        resp = client.get("/api/targets/filter?source=simulation&alliance=hostile")
        assert resp.status_code == 200
        assert resp.json()["total"] == 0

    def test_filter_no_params_returns_all(self, client):
        resp = client.get("/api/targets/filter")
        assert resp.status_code == 200
        assert resp.json()["total"] == 5


# ---------------------------------------------------------------------------
# Tests: GET /api/targets/stats
# ---------------------------------------------------------------------------

class TestTargetStats:
    def test_stats_totals(self, client):
        resp = client.get("/api/targets/stats")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 5

    def test_stats_by_source(self, client):
        resp = client.get("/api/targets/stats")
        data = resp.json()
        assert data["by_source"]["simulation"] == 2
        assert data["by_source"]["yolo"] == 2
        assert data["by_source"]["ble"] == 1

    def test_stats_by_alliance(self, client):
        resp = client.get("/api/targets/stats")
        data = resp.json()
        assert data["by_alliance"]["friendly"] == 2
        assert data["by_alliance"]["hostile"] == 1
        assert data["by_alliance"]["unknown"] == 2

    def test_stats_activity_windows(self, client):
        resp = client.get("/api/targets/stats")
        data = resp.json()
        # 4 targets have recent last_seen, 1 is 600s old (>5min but <1hr)
        assert data["active_5min"] == 4
        assert data["active_1hr"] == 5

    def test_stats_by_asset_type(self, client):
        resp = client.get("/api/targets/stats")
        data = resp.json()
        assert "rover" in data["by_asset_type"]
        assert "drone" in data["by_asset_type"]


# ---------------------------------------------------------------------------
# Tests: GET /api/targets/{target_id}
# ---------------------------------------------------------------------------

class TestTargetDetail:
    def test_get_known_target(self, client):
        resp = client.get("/api/targets/rover_alpha")
        assert resp.status_code == 200
        data = resp.json()
        assert data["target_id"] == "rover_alpha"
        assert data["name"] == "Alpha Rover"
        assert "trail" in data

    def test_get_unknown_target(self, client):
        resp = client.get("/api/targets/nonexistent_xyz")
        assert resp.status_code == 200
        data = resp.json()
        assert "error" in data

    def test_get_ble_target(self, client):
        resp = client.get("/api/targets/ble_aabbccddee")
        assert resp.status_code == 200
        data = resp.json()
        assert data["name"] == "iPhone-Matt"
        assert data["source"] == "ble"


# ---------------------------------------------------------------------------
# Tests: No tracker / no engine fallback
# ---------------------------------------------------------------------------

class TestNoTracker:
    def test_search_no_tracker(self):
        app = FastAPI()
        app.include_router(router)
        app.state.amy = None
        app.state.simulation_engine = None
        c = TestClient(app)
        resp = c.get("/api/targets/search?q=test")
        assert resp.status_code == 200
        assert resp.json()["total"] == 0

    def test_stats_no_tracker(self):
        app = FastAPI()
        app.include_router(router)
        app.state.amy = None
        app.state.simulation_engine = None
        c = TestClient(app)
        resp = c.get("/api/targets/stats")
        assert resp.status_code == 200
        assert resp.json()["total"] == 0

    def test_detail_no_tracker(self):
        app = FastAPI()
        app.include_router(router)
        app.state.amy = None
        app.state.simulation_engine = None
        c = TestClient(app)
        resp = c.get("/api/targets/some_id")
        assert resp.status_code == 200
        assert "error" in resp.json()
