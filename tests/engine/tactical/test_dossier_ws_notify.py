# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for WebSocket notification on new dossier creation."""

import queue
import pytest

from engine.tactical.dossier_manager import DossierManager


class MockEventBus:
    """Captures published events for assertion."""
    def __init__(self):
        self.events = []

    def publish(self, event_type: str, data: dict = None):
        self.events.append({"type": event_type, "data": data or {}})

    def subscribe(self):
        return queue.Queue()

    def unsubscribe(self, q):
        pass


class MockDossierStore:
    """Minimal mock for DossierStore."""
    def __init__(self):
        self._dossiers = {}

    def create_dossier(self, name="", entity_type="unknown",
                       identifiers=None, tags=None):
        import uuid
        did = str(uuid.uuid4())
        self._dossiers[did] = {
            "dossier_id": did,
            "name": name,
            "entity_type": entity_type,
            "identifiers": identifiers or {},
            "tags": tags or [],
        }
        return did

    def get_dossier(self, dossier_id):
        return self._dossiers.get(dossier_id)

    def find_by_identifier(self, key, value):
        return None

    def get_recent(self, limit=50, since=None):
        return list(self._dossiers.values())[:limit]

    def add_signal(self, **kwargs):
        return "signal-1"


class TestDossierWebSocketNotification:
    """Verify DossierManager publishes dossier_created events."""

    def test_new_dossier_publishes_event(self):
        bus = MockEventBus()
        store = MockDossierStore()
        mgr = DossierManager(store=store, event_bus=bus)

        dossier_id = mgr.find_or_create_for_target(
            "ble_aabbccddeeff",
            name="Test Phone",
            entity_type="device",
            identifiers={"mac": "AA:BB:CC:DD:EE:FF"},
            tags=["ble"],
        )

        # Should have published a dossier_created event
        created_events = [
            e for e in bus.events if e["type"] == "dossier_created"
        ]
        assert len(created_events) == 1
        data = created_events[0]["data"]
        assert data["dossier_id"] == dossier_id
        assert data["target_id"] == "ble_aabbccddeeff"
        assert data["name"] == "Test Phone"
        assert data["entity_type"] == "device"

    def test_existing_dossier_no_event(self):
        bus = MockEventBus()
        store = MockDossierStore()
        mgr = DossierManager(store=store, event_bus=bus)

        # First call creates
        d1 = mgr.find_or_create_for_target("ble_test123456", name="Phone")
        initial_count = len([e for e in bus.events if e["type"] == "dossier_created"])
        assert initial_count == 1

        # Second call reuses — no new event
        d2 = mgr.find_or_create_for_target("ble_test123456", name="Phone")
        assert d1 == d2
        created_count = len([e for e in bus.events if e["type"] == "dossier_created"])
        assert created_count == 1  # Still just 1

    def test_no_event_without_event_bus(self):
        store = MockDossierStore()
        mgr = DossierManager(store=store, event_bus=None)

        # Should not raise even without event_bus
        dossier_id = mgr.find_or_create_for_target(
            "ble_no_bus_test",
            name="No Bus",
        )
        assert dossier_id is not None

    def test_event_includes_tags(self):
        bus = MockEventBus()
        store = MockDossierStore()
        mgr = DossierManager(store=store, event_bus=bus)

        mgr.find_or_create_for_target(
            "det_person_1",
            name="Person Detection",
            entity_type="person",
            tags=["yolo", "person"],
        )

        created = [e for e in bus.events if e["type"] == "dossier_created"]
        assert len(created) == 1
        assert "yolo" in created[0]["data"]["tags"]
        assert "person" in created[0]["data"]["tags"]
