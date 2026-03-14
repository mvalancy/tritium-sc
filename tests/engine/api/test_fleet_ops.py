# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for fleet coordination, analytics, and config template endpoints."""

from __future__ import annotations

import time
from unittest.mock import MagicMock, patch

import pytest


def _make_plugin():
    """Create a FleetDashboardPlugin instance for testing."""
    from plugins.fleet_dashboard.plugin import FleetDashboardPlugin

    plugin = FleetDashboardPlugin()
    # Mock the context and logger
    plugin._logger = MagicMock()
    plugin._running = True
    return plugin


@pytest.mark.unit
class TestFleetCoordinationPlugin:
    """Test fleet coordination methods on the plugin."""

    def test_builtin_templates_initialized(self):
        plugin = _make_plugin()
        templates = plugin.get_config_templates()
        assert len(templates) == 3
        names = {t["id"] for t in templates}
        assert "perimeter_high_security" in names
        assert "indoor_normal" in names
        assert "power_saver_mobile" in names

    def test_get_config_template(self):
        plugin = _make_plugin()
        tpl = plugin.get_config_template("indoor_normal")
        assert tpl is not None
        assert tpl["name"] == "Indoor - Normal"
        assert tpl["power_mode"] == "normal"

    def test_get_missing_template_returns_none(self):
        plugin = _make_plugin()
        assert plugin.get_config_template("nonexistent") is None

    def test_create_custom_template(self):
        plugin = _make_plugin()
        tpl = plugin.create_config_template({
            "name": "Custom Fast",
            "ble_scan_interval_ms": 2000,
            "power_mode": "high_performance",
        })
        assert tpl["name"] == "Custom Fast"
        assert tpl["builtin"] is False
        assert "id" in tpl
        # Now retrievable
        assert plugin.get_config_template(tpl["id"]) is not None
        assert len(plugin.get_config_templates()) == 4

    def test_get_groups_empty(self):
        plugin = _make_plugin()
        groups = plugin.get_groups()
        assert groups == {}

    def test_get_groups_with_devices(self):
        plugin = _make_plugin()
        # Add devices with groups via heartbeat
        plugin._on_heartbeat({"device_id": "dev-1", "device_group": "perimeter"})
        plugin._on_heartbeat({"device_id": "dev-2", "device_group": "perimeter"})
        plugin._on_heartbeat({"device_id": "dev-3", "device_group": "interior"})

        groups = plugin.get_groups()
        assert "perimeter" in groups
        assert "interior" in groups
        assert len(groups["perimeter"]) == 2
        assert len(groups["interior"]) == 1

    def test_get_devices_in_group(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({"device_id": "dev-1", "device_group": "mobile"})
        plugin._on_heartbeat({"device_id": "dev-2", "device_group": "perimeter"})

        mobile = plugin.get_devices_in_group("mobile")
        assert len(mobile) == 1
        assert mobile[0]["device_id"] == "dev-1"

    def test_send_fleet_command_no_devices(self):
        plugin = _make_plugin()
        result = plugin.send_fleet_command("nonexistent", "reboot")
        assert result["status"] == "complete"
        assert result["expected_targets"] == 0

    def test_send_fleet_command_with_devices(self):
        plugin = _make_plugin()
        # Add devices
        plugin._on_heartbeat({"device_id": "dev-1", "device_group": "perimeter"})
        plugin._on_heartbeat({"device_id": "dev-2", "device_group": "perimeter"})

        # Mock event bus
        mock_bus = MagicMock()
        plugin._event_bus = mock_bus

        result = plugin.send_fleet_command("perimeter", "scan_burst")
        assert result["expected_targets"] == 2
        assert result["mqtt_published"] == 2
        assert result["command_type"] == "scan_burst"
        assert mock_bus.publish.call_count == 2

    def test_send_fleet_command_all_group(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({"device_id": "dev-1", "device_group": "perimeter"})
        plugin._on_heartbeat({"device_id": "dev-2", "device_group": "interior"})

        mock_bus = MagicMock()
        plugin._event_bus = mock_bus

        result = plugin.send_fleet_command("__all__", "reboot")
        assert result["expected_targets"] == 2

    def test_fleet_commands_history(self):
        plugin = _make_plugin()
        plugin.send_fleet_command("grp1", "reboot")
        plugin.send_fleet_command("grp2", "scan_burst")

        commands = plugin.get_fleet_commands()
        assert len(commands) == 2
        # Most recent first
        assert commands[0]["command_type"] == "scan_burst"

    def test_apply_template_to_group(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({"device_id": "dev-1", "device_group": "perimeter"})

        mock_bus = MagicMock()
        plugin._event_bus = mock_bus

        result = plugin.apply_template_to_group("perimeter_high_security", "perimeter")
        assert result["command_type"] == "apply_template"
        assert result["payload"]["template_id"] == "perimeter_high_security"
        assert result["payload"]["ble_scan_interval_ms"] == 5000

    def test_apply_missing_template(self):
        plugin = _make_plugin()
        result = plugin.apply_template_to_group("nonexistent", "perimeter")
        assert result["status"] == "failed"
        assert "not found" in result["error"]


@pytest.mark.unit
class TestFleetAnalyticsPlugin:
    """Test fleet analytics methods."""

    def test_analytics_snapshot_empty(self):
        plugin = _make_plugin()
        snap = plugin.get_analytics_snapshot()
        assert snap["total_devices"] == 0
        assert snap["online_devices"] == 0
        assert snap["groups"] == {}

    def test_analytics_snapshot_with_devices(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({
            "device_id": "dev-1",
            "device_group": "perimeter",
            "uptime_s": 3600,
            "battery_pct": 80,
            "ble_count": 10,
        })
        plugin._on_heartbeat({
            "device_id": "dev-2",
            "device_group": "interior",
            "uptime_s": 1800,
            "battery_pct": 60,
            "wifi_count": 5,
        })

        snap = plugin.get_analytics_snapshot()
        assert snap["total_devices"] == 2
        assert snap["online_devices"] == 2
        assert snap["avg_battery_pct"] == 70.0
        assert snap["total_ble_sightings"] == 10
        assert snap["total_wifi_sightings"] == 5
        assert snap["groups"]["perimeter"] == 1
        assert snap["groups"]["interior"] == 1
        assert len(snap["uptime_records"]) == 2

    def test_analytics_history(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({"device_id": "dev-1"})
        plugin.get_analytics_snapshot()
        plugin.get_analytics_snapshot()

        history = plugin.get_analytics_history(hours=1)
        assert len(history) == 2

    def test_coverage_map_empty(self):
        plugin = _make_plugin()
        assert plugin.get_coverage_map() == []

    def test_coverage_map_with_positions(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({
            "device_id": "dev-1",
            "lat": 33.0,
            "lng": -97.0,
        })
        coverage = plugin.get_coverage_map()
        assert len(coverage) == 1
        assert coverage[0]["lat"] == 33.0

    def test_heartbeat_tracks_device_group(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({
            "device_id": "dev-1",
            "device_group": "mobile",
        })
        dev = plugin.get_device("dev-1")
        assert dev is not None
        assert dev["device_group"] == "mobile"

    def test_heartbeat_tracks_position(self):
        plugin = _make_plugin()
        plugin._on_heartbeat({
            "device_id": "dev-1",
            "lat": 33.123,
            "lng": -97.456,
        })
        dev = plugin.get_device("dev-1")
        assert dev["lat"] == 33.123
        assert dev["lng"] == -97.456
