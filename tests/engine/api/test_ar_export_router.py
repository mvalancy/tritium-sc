# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Unit tests for the AR export router — /api/targets/ar-export."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from app.routers.ar_export import router


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_app(tracker=None, engine=None, amy=None):
    """Create FastAPI app with ar_export router and optional state."""
    app = FastAPI()
    app.include_router(router)

    if amy is not None:
        app.state.amy = amy
    if engine is not None:
        app.state.simulation_engine = engine

    return app


def _mock_target(target_id="t1", alliance="friendly", name="Rover-1",
                 asset_type="rover", lat=37.7749, lng=-122.4194,
                 position_confidence=0.8, speed=1.5, heading=90.0,
                 threat_score=0.0, status="active"):
    """Create a mock target with to_dict()."""
    t = MagicMock()
    t.target_id = target_id
    t.alliance = alliance
    t.name = name
    t.to_dict.return_value = {
        "target_id": target_id,
        "alliance": alliance,
        "name": name,
        "asset_type": asset_type,
        "lat": lat,
        "lng": lng,
        "position": {"x": 10.0, "y": 20.0},
        "position_confidence": position_confidence,
        "speed": speed,
        "heading": heading,
        "threat_score": threat_score,
        "status": status,
    }
    return t


def _mock_tracker(targets=None):
    """Create a mock TargetTracker."""
    tracker = MagicMock()
    tracker.get_all.return_value = targets or []
    return tracker


def _amy_with_tracker(tracker):
    """Create a mock Amy with a target_tracker."""
    amy = MagicMock()
    amy.target_tracker = tracker
    return amy


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestARExport:
    """Tests for GET /api/targets/ar-export."""

    def test_empty_no_tracker(self):
        """Returns empty when no tracker or engine available."""
        app = _make_app()
        client = TestClient(app)
        resp = client.get("/api/targets/ar-export")
        assert resp.status_code == 200
        data = resp.json()
        assert data["target_count"] == 0
        assert data["targets"] == []
        assert data["version"] == "1.0"

    def test_returns_targets(self):
        """Returns simplified AR target data."""
        t1 = _mock_target("rover_1", "friendly", "Rover-1", "rover")
        t2 = _mock_target("hostile_1", "hostile", "Intruder", "person",
                          lat=37.78, lng=-122.42, threat_score=0.9)
        tracker = _mock_tracker([t1, t2])
        amy = _amy_with_tracker(tracker)
        app = _make_app(amy=amy)
        client = TestClient(app)

        resp = client.get("/api/targets/ar-export")
        assert resp.status_code == 200
        data = resp.json()
        assert data["target_count"] == 2

        ar = data["targets"][0]
        assert ar["id"] == "rover_1"
        assert ar["name"] == "Rover-1"
        assert ar["type"] == "rover"
        assert ar["alliance"] == "friendly"
        assert ar["lat"] == 37.7749
        assert ar["lng"] == -122.4194
        assert "alt" in ar
        assert "heading" in ar
        assert "speed" in ar
        assert "confidence" in ar

    def test_filter_by_alliance(self):
        """Filters targets by alliance."""
        t1 = _mock_target("r1", "friendly", "Rover", "rover")
        t2 = _mock_target("h1", "hostile", "Threat", "person")
        tracker = _mock_tracker([t1, t2])
        amy = _amy_with_tracker(tracker)
        app = _make_app(amy=amy)
        client = TestClient(app)

        resp = client.get("/api/targets/ar-export?alliance=hostile")
        data = resp.json()
        assert data["target_count"] == 1
        assert data["targets"][0]["alliance"] == "hostile"

    def test_filter_by_confidence(self):
        """Filters targets by minimum confidence."""
        t1 = _mock_target("r1", "friendly", "Rover", "rover", position_confidence=0.9)
        t2 = _mock_target("h1", "hostile", "Low", "person", position_confidence=0.1)
        tracker = _mock_tracker([t1, t2])
        amy = _amy_with_tracker(tracker)
        app = _make_app(amy=amy)
        client = TestClient(app)

        resp = client.get("/api/targets/ar-export?min_confidence=0.5")
        data = resp.json()
        assert data["target_count"] == 1

    def test_max_targets_limit(self):
        """Respects max_targets limit."""
        targets = [_mock_target(f"t{i}", "friendly", f"Unit-{i}", "rover") for i in range(10)]
        tracker = _mock_tracker(targets)
        amy = _amy_with_tracker(tracker)
        app = _make_app(amy=amy)
        client = TestClient(app)

        resp = client.get("/api/targets/ar-export?max_targets=3")
        data = resp.json()
        assert data["target_count"] == 3

    def test_altitude_by_asset_type(self):
        """Derives altitude from asset type."""
        t_drone = _mock_target("d1", "friendly", "Drone", "drone")
        t_person = _mock_target("p1", "hostile", "Person", "person")
        t_vehicle = _mock_target("v1", "unknown", "Vehicle", "vehicle")
        tracker = _mock_tracker([t_drone, t_person, t_vehicle])
        amy = _amy_with_tracker(tracker)
        app = _make_app(amy=amy)
        client = TestClient(app)

        resp = client.get("/api/targets/ar-export")
        data = resp.json()
        targets_by_type = {t["type"]: t for t in data["targets"]}
        assert targets_by_type["drone"]["alt"] == 30.0
        assert targets_by_type["person"]["alt"] == 1.7
        assert targets_by_type["vehicle"]["alt"] == 1.5

    def test_sim_engine_fallback(self):
        """Falls back to simulation engine when no tracker."""
        t1 = _mock_target("sim_1", "hostile", "Hostile", "person")
        engine = MagicMock()
        engine.get_targets.return_value = [t1]
        app = _make_app(engine=engine)
        client = TestClient(app)

        resp = client.get("/api/targets/ar-export")
        data = resp.json()
        assert data["target_count"] == 1
        assert data["targets"][0]["id"] == "sim_1"
