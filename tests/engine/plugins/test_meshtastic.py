# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the Meshtastic bridge plugin."""

import pytest

from engine.plugins.base import PluginInterface


@pytest.mark.unit
class TestMeshtasticPlugin:
    """Verify Meshtastic plugin interface and configuration."""

    def test_implements_plugin_interface(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        assert isinstance(plugin, PluginInterface)

    def test_plugin_identity(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        assert plugin.plugin_id == "tritium.meshtastic"
        assert plugin.name == "Meshtastic Bridge"
        assert plugin.version == "0.2.0"

    def test_capabilities(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        caps = plugin.capabilities
        assert "bridge" in caps
        assert "data_source" in caps
        assert "routes" in caps

    def test_default_config(self):
        from plugins.meshtastic.plugin import MeshtasticConfig
        config = MeshtasticConfig()
        assert config.connection_type == "serial"
        assert config.enabled is False
        assert config.poll_interval == 5.0

    def test_disabled_start_is_healthy(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._config.enabled = False
        plugin._running = True
        plugin.start()
        assert plugin.healthy is True

    def test_send_text_without_radio_returns_false(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        assert plugin.send_text("hello") is False

    def test_send_waypoint_without_radio_returns_false(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        assert plugin.send_waypoint(37.0, -122.0, "Base") is False

    def test_nodes_start_empty(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        assert plugin._nodes == {}

    def test_update_node_caches_state(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        plugin._update_node("!abc123", {
            "user": {"longName": "Base Station", "hwModel": "HELTEC_V3"},
            "position": {"latitude": 37.7749, "longitude": -122.4194, "altitude": 10.0},
            "deviceMetrics": {"batteryLevel": 85},
            "lastHeard": 1710300000,
            "snr": 9.5,
        })
        assert "!abc123" in plugin._nodes
        node = plugin._nodes["!abc123"]
        assert node["name"] == "Base Station"
        assert node["lat"] == 37.7749
        assert node["lng"] == -122.4194
        assert node["battery"] == 85
        assert node["snr"] == 9.5
        assert node["hw_model"] == "HELTEC_V3"

    def test_update_node_with_gps_pushes_to_tracker(self):
        from unittest.mock import MagicMock
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        tracker = MagicMock()
        plugin._tracker = tracker
        plugin._update_node("!node1", {
            "user": {"longName": "Rover"},
            "position": {"latitude": 37.0, "longitude": -122.0},
            "deviceMetrics": {"batteryLevel": 50},
        })
        tracker.update_from_simulation.assert_called_once()
        call_data = tracker.update_from_simulation.call_args[0][0]
        assert call_data["target_id"] == "mesh_node1"
        assert call_data["name"] == "[Mesh] Rover"
        assert call_data["alliance"] == "friendly"
        assert call_data["asset_type"] == "mesh_radio"
        assert call_data["battery"] == 0.5

    def test_update_node_no_gps_skips_tracker(self):
        from unittest.mock import MagicMock
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        tracker = MagicMock()
        plugin._tracker = tracker
        # lat=0 lng=0 means no GPS fix
        plugin._update_node("!nogps", {
            "user": {"longName": "NoGPS"},
            "position": {"latitude": 0.0, "longitude": 0.0},
            "deviceMetrics": {"batteryLevel": 100},
        })
        tracker.update_from_simulation.assert_not_called()

    def test_update_node_uses_shortname_fallback(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        plugin._update_node("!short", {
            "user": {"shortName": "SN01"},
            "position": {},
            "deviceMetrics": {"batteryLevel": 100},
        })
        assert plugin._nodes["!short"]["name"] == "SN01"

    def test_update_node_uses_node_id_fallback(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        plugin._update_node("!fallback", {
            "user": {},
            "position": {},
            "deviceMetrics": {"batteryLevel": 100},
        })
        assert plugin._nodes["!fallback"]["name"] == "!fallback"

    def test_config_from_env_vars(self, monkeypatch):
        monkeypatch.setenv("MESHTASTIC_CONNECTION", "tcp")
        monkeypatch.setenv("MESHTASTIC_TCP_HOST", "192.168.1.100")
        monkeypatch.setenv("MESHTASTIC_TCP_PORT", "9999")
        monkeypatch.setenv("MESHTASTIC_ENABLED", "true")
        monkeypatch.setenv("MESHTASTIC_POLL_INTERVAL", "10.0")
        from plugins.meshtastic.plugin import MeshtasticConfig
        config = MeshtasticConfig()
        assert config.connection_type == "tcp"
        assert config.tcp_host == "192.168.1.100"
        assert config.tcp_port == 9999
        assert config.enabled is True
        assert config.poll_interval == 10.0

    def test_config_disabled_by_default(self):
        from plugins.meshtastic.plugin import MeshtasticConfig
        config = MeshtasticConfig()
        assert config.enabled is False

    def test_connect_without_meshtastic_package_returns_false(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        plugin._config.enabled = True
        # _connect tries to import meshtastic which isn't installed in test env
        result = plugin._connect()
        assert result is False

    def test_start_disabled_stays_healthy(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        plugin._config.enabled = False
        plugin.start()
        assert plugin.healthy is True
        assert plugin._running is True

    def test_stop_without_start_is_safe(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        plugin = MeshtasticPlugin()
        plugin._logger = __import__("logging").getLogger("test")
        # Should not raise
        plugin.stop()


@pytest.mark.unit
class TestMeshtasticRoutes:
    """Verify Meshtastic route registration produces a valid APIRouter."""

    def test_create_router_returns_api_router(self):
        from fastapi import APIRouter
        from plugins.meshtastic.plugin import MeshtasticPlugin
        from plugins.meshtastic.routes import create_router

        plugin = MeshtasticPlugin()
        router = create_router(plugin)
        assert isinstance(router, APIRouter)

    def test_router_has_correct_prefix(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        from plugins.meshtastic.routes import create_router

        plugin = MeshtasticPlugin()
        router = create_router(plugin)
        assert router.prefix == "/api/meshtastic"

    def test_router_has_expected_endpoints(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        from plugins.meshtastic.routes import create_router

        plugin = MeshtasticPlugin()
        router = create_router(plugin)

        # Collect all route paths and methods
        routes = {}
        for route in router.routes:
            path = getattr(route, "path", None)
            methods = getattr(route, "methods", set())
            if path:
                routes[path] = methods

        pfx = "/api/meshtastic"
        assert f"{pfx}/nodes" in routes
        assert "GET" in routes[f"{pfx}/nodes"]
        assert f"{pfx}/nodes/{{node_id}}" in routes
        assert "GET" in routes[f"{pfx}/nodes/{{node_id}}"]
        assert f"{pfx}/send" in routes
        assert "POST" in routes[f"{pfx}/send"]
        assert f"{pfx}/waypoint" in routes
        assert "POST" in routes[f"{pfx}/waypoint"]
        assert f"{pfx}/status" in routes
        assert "GET" in routes[f"{pfx}/status"]

    def test_router_has_eight_routes(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        from plugins.meshtastic.routes import create_router

        plugin = MeshtasticPlugin()
        router = create_router(plugin)
        # Filter to actual API routes (exclude internal)
        api_routes = [r for r in router.routes if hasattr(r, "methods")]
        assert len(api_routes) == 8

    def test_router_tagged_meshtastic(self):
        from plugins.meshtastic.plugin import MeshtasticPlugin
        from plugins.meshtastic.routes import create_router

        plugin = MeshtasticPlugin()
        router = create_router(plugin)
        assert "meshtastic" in router.tags
