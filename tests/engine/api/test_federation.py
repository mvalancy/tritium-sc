# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the Federation plugin."""

import json
import os
import tempfile
import pytest

from tritium_lib.models.federation import (
    ConnectionState,
    FederatedSite,
    FederationMessage,
    FederationMessageType,
    SharedTarget,
    SharePolicy,
    SiteConnection,
    SiteRole,
)


class MockEventBus:
    """Minimal EventBus mock for testing."""
    def __init__(self):
        self.published = []

    def publish(self, event_type: str, data: dict = None):
        self.published.append({"type": event_type, "data": data or {}})

    def subscribe(self):
        import queue
        return queue.Queue()

    def unsubscribe(self, q):
        pass


class MockTracker:
    """Minimal TargetTracker mock."""
    def __init__(self):
        self.updates = []

    def update_from_federation(self, data: dict):
        self.updates.append(data)


@pytest.fixture
def temp_dir():
    with tempfile.TemporaryDirectory() as d:
        yield d


@pytest.fixture
def plugin(temp_dir):
    """Create a FederationPlugin with mocked dependencies."""
    # Import here so sys.path is set up
    import sys
    plugins_dir = os.path.join(os.path.dirname(__file__), "..", "..", "..", "plugins")
    plugins_dir = os.path.abspath(plugins_dir)
    if plugins_dir not in sys.path:
        sys.path.insert(0, plugins_dir)

    from federation.plugin import FederationPlugin

    p = FederationPlugin()
    p._event_bus = MockEventBus()
    p._tracker = MockTracker()
    p._logger = __import__("logging").getLogger("test-federation")
    p._sites_file = os.path.join(temp_dir, "federation_sites.json")
    p._running = True
    return p


class TestFederationPlugin:
    """Tests for FederationPlugin core functionality."""

    def test_plugin_identity(self, plugin):
        assert plugin.plugin_id == "tritium.federation"
        assert plugin.name == "Federation"
        assert plugin.version == "1.0.0"
        assert "bridge" in plugin.capabilities

    def test_add_site(self, plugin):
        site = FederatedSite(
            name="Alpha HQ",
            mqtt_host="10.0.0.1",
            role=SiteRole.PEER,
        )
        site_id = plugin.add_site(site)
        assert site_id == site.site_id
        assert plugin.get_site(site_id) is not None
        assert plugin.get_site(site_id).name == "Alpha HQ"

    def test_remove_site(self, plugin):
        site = FederatedSite(name="To Remove")
        plugin.add_site(site)
        assert plugin.remove_site(site.site_id) is True
        assert plugin.get_site(site.site_id) is None

    def test_remove_nonexistent(self, plugin):
        assert plugin.remove_site("nonexistent") is False

    def test_list_sites(self, plugin):
        site1 = FederatedSite(name="Site A")
        site2 = FederatedSite(name="Site B")
        plugin.add_site(site1)
        plugin.add_site(site2)
        sites = plugin.list_sites()
        assert len(sites) == 2
        names = {s["name"] for s in sites}
        assert "Site A" in names
        assert "Site B" in names

    def test_list_sites_includes_connection(self, plugin):
        site = FederatedSite(name="With Conn")
        plugin.add_site(site)
        sites = plugin.list_sites()
        assert len(sites) == 1
        assert "connection" in sites[0]

    def test_share_target(self, plugin):
        target = SharedTarget(
            target_id="ble_aabbccddeeff",
            source_site_id="site-alpha",
            name="Test Device",
        )
        plugin.share_target(target)
        shared = plugin.get_shared_targets()
        assert len(shared) == 1
        assert shared[0]["target_id"] == "ble_aabbccddeeff"

    def test_receive_target(self, plugin):
        target = SharedTarget(
            target_id="det_person_1",
            source_site_id="site-bravo",
            name="Remote Person",
            lat=37.7,
            lng=-121.9,
        )
        plugin.receive_target(target)
        shared = plugin.get_shared_targets()
        assert len(shared) == 1
        # Check tracker was updated
        assert len(plugin._tracker.updates) == 1
        assert plugin._tracker.updates[0]["target_id"] == "det_person_1"

    def test_get_stats(self, plugin):
        site = FederatedSite(name="Stats Site", enabled=True)
        plugin.add_site(site)
        stats = plugin.get_stats()
        assert stats["total_sites"] == 1
        assert stats["enabled_sites"] == 1
        assert stats["connected_sites"] == 0
        assert stats["shared_targets"] == 0

    def test_get_connection(self, plugin):
        site = FederatedSite(name="Conn Site")
        plugin.add_site(site)
        conn = plugin.get_connection(site.site_id)
        assert conn is not None
        assert conn.state == ConnectionState.DISCONNECTED

    def test_events_published_on_add(self, plugin):
        site = FederatedSite(name="Event Site")
        plugin.add_site(site)
        events = [e for e in plugin._event_bus.published if e["type"] == "federation:site_added"]
        assert len(events) == 1
        assert events[0]["data"]["name"] == "Event Site"

    def test_events_published_on_share(self, plugin):
        target = SharedTarget(
            target_id="ble_test",
            source_site_id="site-1",
        )
        plugin.share_target(target)
        events = [e for e in plugin._event_bus.published if e["type"] == "federation:target_shared"]
        assert len(events) == 1

    def test_events_published_on_receive(self, plugin):
        target = SharedTarget(
            target_id="ble_recv",
            source_site_id="site-2",
        )
        plugin.receive_target(target)
        events = [e for e in plugin._event_bus.published if e["type"] == "federation:target_received"]
        assert len(events) == 1


class TestFederationPersistence:
    """Tests for site persistence."""

    def test_save_and_load(self, temp_dir):
        import sys
        plugins_dir = os.path.join(os.path.dirname(__file__), "..", "..", "..", "plugins")
        plugins_dir = os.path.abspath(plugins_dir)
        if plugins_dir not in sys.path:
            sys.path.insert(0, plugins_dir)

        from federation.plugin import FederationPlugin

        # Create plugin and add a site
        p1 = FederationPlugin()
        p1._logger = __import__("logging").getLogger("test")
        p1._sites_file = os.path.join(temp_dir, "sites.json")
        p1._running = True

        site = FederatedSite(name="Persistent Site", mqtt_host="10.0.0.5")
        p1.add_site(site)

        # Create new plugin and load
        p2 = FederationPlugin()
        p2._logger = __import__("logging").getLogger("test")
        p2._sites_file = os.path.join(temp_dir, "sites.json")
        p2._load_sites()

        assert len(p2._sites) == 1
        loaded = list(p2._sites.values())[0]
        assert loaded.name == "Persistent Site"
        assert loaded.mqtt_host == "10.0.0.5"

    def test_load_empty_file(self, temp_dir):
        import sys
        plugins_dir = os.path.join(os.path.dirname(__file__), "..", "..", "..", "plugins")
        plugins_dir = os.path.abspath(plugins_dir)
        if plugins_dir not in sys.path:
            sys.path.insert(0, plugins_dir)

        from federation.plugin import FederationPlugin

        p = FederationPlugin()
        p._logger = __import__("logging").getLogger("test")
        p._sites_file = os.path.join(temp_dir, "nonexistent.json")
        p._load_sites()
        assert len(p._sites) == 0


class TestFederationLifecycle:
    """Tests for start/stop lifecycle."""

    def test_start_stop(self, plugin):
        # plugin._running is already True from fixture, reset it
        plugin._running = False
        plugin.start()
        assert plugin._running is True
        assert plugin.healthy is True
        plugin.stop()
        assert plugin._running is False

    def test_double_start(self, plugin):
        plugin._running = False
        plugin.start()
        plugin.start()  # Should not raise
        assert plugin._running is True
        plugin.stop()
