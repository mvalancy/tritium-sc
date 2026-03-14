# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for target watch list API."""

import pytest


@pytest.fixture
def _clear_store():
    """Reset watch list store before each test."""
    from app.routers.watchlist import _watch_entries, _target_snapshots, _alert_history
    _watch_entries.clear()
    _target_snapshots.clear()
    _alert_history.clear()
    yield
    _watch_entries.clear()
    _target_snapshots.clear()
    _alert_history.clear()


class TestWatchlistModels:
    """Test watch list data models."""

    @pytest.mark.unit
    def test_create_entry(self, _clear_store):
        from app.routers.watchlist import WatchEntryCreate

        body = WatchEntryCreate(
            target_id="ble_aa:bb:cc:dd:ee:ff",
            label="Suspicious phone",
            priority=1,
            alert_on_move=True,
        )
        assert body.target_id == "ble_aa:bb:cc:dd:ee:ff"
        assert body.label == "Suspicious phone"
        assert body.priority == 1

    @pytest.mark.unit
    def test_create_defaults(self, _clear_store):
        from app.routers.watchlist import WatchEntryCreate

        body = WatchEntryCreate(target_id="test_target")
        assert body.priority == 3
        assert body.alert_on_move is True
        assert body.alert_on_state_change is True
        assert body.alert_on_zone_enter is False

    @pytest.mark.unit
    def test_update_partial(self, _clear_store):
        from app.routers.watchlist import WatchEntryUpdate

        body = WatchEntryUpdate(notes="Updated note", priority=2)
        updates = body.model_dump(exclude_none=True)
        assert updates == {"notes": "Updated note", "priority": 2}


class TestWatchlistSnapshots:
    """Test target snapshot updates and alert generation."""

    @pytest.mark.unit
    def test_snapshot_no_watchers(self, _clear_store):
        from app.routers.watchlist import update_target_snapshot

        alerts = update_target_snapshot("ble_test", {"status": "active"})
        assert alerts == []

    @pytest.mark.unit
    def test_movement_alert(self, _clear_store):
        from app.routers.watchlist import (
            _watch_entries, _target_snapshots, update_target_snapshot,
        )

        # Add a watch entry
        _watch_entries["we_test"] = {
            "id": "we_test",
            "target_id": "ble_test",
            "alert_on_move": True,
            "alert_on_state_change": False,
        }

        # First snapshot
        update_target_snapshot("ble_test", {
            "position": {"x": 0, "y": 0},
            "status": "active",
        })

        # Second snapshot with movement
        alerts = update_target_snapshot("ble_test", {
            "position": {"x": 100, "y": 0},
            "status": "active",
        })
        assert len(alerts) == 1
        assert alerts[0]["type"] == "movement"

    @pytest.mark.unit
    def test_state_change_alert(self, _clear_store):
        from app.routers.watchlist import (
            _watch_entries, _target_snapshots, update_target_snapshot,
        )

        _watch_entries["we_test"] = {
            "id": "we_test",
            "target_id": "ble_test",
            "alert_on_move": False,
            "alert_on_state_change": True,
        }

        update_target_snapshot("ble_test", {"status": "active"})
        alerts = update_target_snapshot("ble_test", {"status": "offline"})
        assert len(alerts) == 1
        assert alerts[0]["type"] == "state_change"
        assert "active -> offline" in alerts[0]["details"]

    @pytest.mark.unit
    def test_no_alert_when_disabled(self, _clear_store):
        from app.routers.watchlist import (
            _watch_entries, update_target_snapshot,
        )

        _watch_entries["we_test"] = {
            "id": "we_test",
            "target_id": "ble_test",
            "alert_on_move": False,
            "alert_on_state_change": False,
        }

        update_target_snapshot("ble_test", {
            "position": {"x": 0, "y": 0},
            "status": "active",
        })
        alerts = update_target_snapshot("ble_test", {
            "position": {"x": 100, "y": 100},
            "status": "offline",
        })
        assert len(alerts) == 0
