# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for device management router — /api/devices/* management endpoints.

Verifies reboot, OTA, config, screenshot, and bulk operations across
MQTT, fleet server, and direct HTTP delivery channels.
"""
from __future__ import annotations

import asyncio
import json
from io import BytesIO
from unittest.mock import MagicMock, patch, AsyncMock

import pytest


def _run(coro):
    """Run an async coroutine synchronously."""
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _make_request(mqtt_bridge=None, fleet_bridge=None, site_id="home") -> MagicMock:
    """Create a mock Request with optional bridges on app.state."""
    request = MagicMock()
    # Build spec list so getattr returns AttributeError for missing attrs
    spec_attrs = ["mqtt_site_id", "plugin_manager"]
    if mqtt_bridge is not None:
        spec_attrs.append("mqtt_bridge")
    if fleet_bridge is not None:
        spec_attrs.append("fleet_bridge")

    state = MagicMock(spec=spec_attrs)
    state.mqtt_site_id = site_id
    state.plugin_manager = None
    if mqtt_bridge is not None:
        state.mqtt_bridge = mqtt_bridge
    if fleet_bridge is not None:
        state.fleet_bridge = fleet_bridge

    request.app.state = state
    return request


def _make_mqtt_bridge() -> MagicMock:
    """Create a mock MQTT bridge."""
    bridge = MagicMock()
    bridge.publish = MagicMock()
    return bridge


def _make_fleet_bridge(devices=None) -> MagicMock:
    """Create a mock FleetBridge with device cache."""
    bridge = MagicMock()
    bridge.rest_url = "http://fleet:8080"
    bridge.devices = devices or {}
    return bridge


# ---------------------------------------------------------------------------
# POST /api/devices/{device_id}/reboot
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestRebootDevice:
    """POST /api/devices/{device_id}/reboot."""

    def test_reboot_via_mqtt(self):
        from app.routers.device_management import reboot_device
        mqtt = _make_mqtt_bridge()
        request = _make_request(mqtt_bridge=mqtt)
        result = _run(reboot_device(request, "d1"))
        assert result["status"] == "ok"
        assert result["device_id"] == "d1"
        assert result["via"] == "mqtt"
        mqtt.publish.assert_called_once()
        topic, payload = mqtt.publish.call_args[0]
        assert "d1" in topic
        assert json.loads(payload)["command"] == "reboot"

    def test_reboot_via_fleet_server(self):
        from app.routers.device_management import reboot_device
        request = _make_request()
        with patch("app.routers.device_management._publish_mqtt_command", return_value=False), \
             patch("app.routers.device_management._proxy_post", return_value={"ok": True}), \
             patch("app.routers.device_management._get_device_ip", return_value=None):
            result = _run(reboot_device(request, "d1"))
        assert result["status"] == "ok"
        assert result["via"] == "fleet_server"

    def test_reboot_no_channel(self):
        from app.routers.device_management import reboot_device
        request = _make_request()
        with patch("app.routers.device_management._publish_mqtt_command", return_value=False), \
             patch("app.routers.device_management._proxy_post", return_value=None), \
             patch("app.routers.device_management._get_device_ip", return_value=None):
            result = _run(reboot_device(request, "d1"))
        assert result.status_code == 503


# ---------------------------------------------------------------------------
# GET /api/devices/{device_id}/config
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestGetDeviceConfig:
    """GET /api/devices/{device_id}/config."""

    def test_config_from_fleet_server(self):
        from app.routers.device_management import get_device_config
        request = _make_request()
        config_data = {"wifi": {"ssid": "TestNet"}, "mqtt": {"broker": "mqtt.local"}}
        with patch("app.routers.device_management._proxy_get", return_value=config_data):
            result = _run(get_device_config(request, "d1"))
        assert result["device_id"] == "d1"
        assert result["source"] == "fleet_server"
        assert result["config"]["wifi"]["ssid"] == "TestNet"

    def test_config_from_direct_device(self):
        from app.routers.device_management import get_device_config
        config_data = {"wifi": {"ssid": "Direct"}}
        fleet_bridge = _make_fleet_bridge(devices={"d1": {"ip": "192.168.1.10"}})
        request = _make_request(fleet_bridge=fleet_bridge)
        with patch("app.routers.device_management._proxy_get", side_effect=[None, config_data]):
            result = _run(get_device_config(request, "d1"))
        assert result["source"] == "direct"

    def test_config_from_cache(self):
        from app.routers.device_management import get_device_config
        fleet_bridge = _make_fleet_bridge(devices={
            "d1": {"version": "1.2.3", "ip": "192.168.1.10", "mac": "AA:BB:CC",
                    "board": "esp32s3", "capabilities": ["wifi"]}
        })
        request = _make_request(fleet_bridge=fleet_bridge)
        with patch("app.routers.device_management._proxy_get", return_value=None):
            result = _run(get_device_config(request, "d1"))
        assert result["source"] == "cached"
        assert result["config"]["firmware"] == "1.2.3"

    def test_config_not_found(self):
        from app.routers.device_management import get_device_config
        from fastapi import HTTPException
        request = _make_request()
        with patch("app.routers.device_management._proxy_get", return_value=None), \
             patch("app.routers.device_management._get_device_ip", return_value=None):
            with pytest.raises(HTTPException) as exc_info:
                _run(get_device_config(request, "unknown"))
            assert exc_info.value.status_code == 404


# ---------------------------------------------------------------------------
# PUT /api/devices/{device_id}/config
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestUpdateDeviceConfig:
    """PUT /api/devices/{device_id}/config."""

    def test_update_via_mqtt(self):
        from app.routers.device_management import update_device_config, ConfigUpdateRequest
        mqtt = _make_mqtt_bridge()
        request = _make_request(mqtt_bridge=mqtt)
        body = ConfigUpdateRequest(wifi={"ssid": "NewNet", "password": "secret"})
        result = _run(update_device_config(request, "d1", body))
        assert result["status"] == "ok"
        assert result["via"] == "mqtt"
        assert "wifi" in result["fields_updated"]

    def test_update_empty_config(self):
        from app.routers.device_management import update_device_config, ConfigUpdateRequest
        from fastapi import HTTPException
        request = _make_request()
        body = ConfigUpdateRequest()
        with pytest.raises(HTTPException) as exc_info:
            _run(update_device_config(request, "d1", body))
        assert exc_info.value.status_code == 400

    def test_update_via_fleet_server(self):
        from app.routers.device_management import update_device_config, ConfigUpdateRequest
        request = _make_request()
        body = ConfigUpdateRequest(display={"brightness": 200})
        with patch("app.routers.device_management._publish_mqtt_command", return_value=False), \
             patch("app.routers.device_management._proxy_post", return_value={"ok": True}), \
             patch("app.routers.device_management._get_device_ip", return_value=None):
            result = _run(update_device_config(request, "d1", body))
        assert result["status"] == "ok"
        assert result["via"] == "fleet_server"


# ---------------------------------------------------------------------------
# POST /api/devices/{device_id}/screenshot
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestScreenshotDevice:
    """POST /api/devices/{device_id}/screenshot."""

    def test_screenshot_direct(self):
        from app.routers.device_management import request_screenshot
        fleet_bridge = _make_fleet_bridge(devices={"d1": {"ip": "192.168.1.10"}})
        request = _make_request(fleet_bridge=fleet_bridge)
        with patch("app.routers.device_management._proxy_get", return_value={"format": "jpeg"}):
            result = _run(request_screenshot(request, "d1"))
        assert result["status"] == "ok"
        assert result["via"] == "direct"
        assert "url" in result

    def test_screenshot_via_mqtt(self):
        from app.routers.device_management import request_screenshot
        mqtt = _make_mqtt_bridge()
        request = _make_request(mqtt_bridge=mqtt)
        with patch("app.routers.device_management._get_device_ip", return_value=None):
            result = _run(request_screenshot(request, "d1"))
        assert result["status"] == "requested"
        assert result["via"] == "mqtt"

    def test_screenshot_no_channel(self):
        from app.routers.device_management import request_screenshot
        request = _make_request()
        with patch("app.routers.device_management._publish_mqtt_command", return_value=False), \
             patch("app.routers.device_management._get_device_ip", return_value=None):
            result = _run(request_screenshot(request, "d1"))
        assert result.status_code == 503


# ---------------------------------------------------------------------------
# POST /api/devices/bulk
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestBulkDeviceAction:
    """POST /api/devices/bulk."""

    def test_bulk_reboot_via_mqtt(self):
        from app.routers.device_management import bulk_device_action, BulkDeviceRequest
        mqtt = _make_mqtt_bridge()
        request = _make_request(mqtt_bridge=mqtt)
        body = BulkDeviceRequest(device_ids=["d1", "d2", "d3"], action="reboot")
        result = _run(bulk_device_action(request, body))
        assert result["total"] == 3
        assert result["succeeded"] == 3
        assert result["failed"] == 0

    def test_bulk_unsupported_action(self):
        from app.routers.device_management import bulk_device_action, BulkDeviceRequest
        from fastapi import HTTPException
        request = _make_request()
        body = BulkDeviceRequest(device_ids=["d1"], action="explode")
        with pytest.raises(HTTPException) as exc_info:
            _run(bulk_device_action(request, body))
        assert exc_info.value.status_code == 400

    def test_bulk_empty_ids(self):
        from app.routers.device_management import bulk_device_action, BulkDeviceRequest
        from fastapi import HTTPException
        request = _make_request()
        body = BulkDeviceRequest(device_ids=[], action="reboot")
        with pytest.raises(HTTPException) as exc_info:
            _run(bulk_device_action(request, body))
        assert exc_info.value.status_code == 400

    def test_bulk_partial_failure(self):
        from app.routers.device_management import bulk_device_action, BulkDeviceRequest
        request = _make_request()
        call_count = 0

        def mock_mqtt_cmd(req, did, cmd):
            nonlocal call_count
            call_count += 1
            return call_count == 1  # Only first succeeds

        body = BulkDeviceRequest(device_ids=["d1", "d2"], action="reboot")
        with patch("app.routers.device_management._publish_mqtt_command", side_effect=mock_mqtt_cmd), \
             patch("app.routers.device_management._proxy_post", return_value=None):
            result = _run(bulk_device_action(request, body))
        assert result["succeeded"] == 1
        assert result["failed"] == 1


# ---------------------------------------------------------------------------
# Helper function tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestHelperFunctions:
    """Test utility functions in device_management module."""

    def test_get_device_ip_from_fleet_bridge(self):
        from app.routers.device_management import _get_device_ip
        fleet_bridge = _make_fleet_bridge(devices={"d1": {"ip": "10.0.0.5"}})
        request = _make_request(fleet_bridge=fleet_bridge)
        assert _get_device_ip(request, "d1") == "10.0.0.5"

    def test_get_device_ip_not_found(self):
        from app.routers.device_management import _get_device_ip
        request = _make_request()
        assert _get_device_ip(request, "unknown") is None

    def test_publish_mqtt_command_success(self):
        from app.routers.device_management import _publish_mqtt_command
        mqtt = _make_mqtt_bridge()
        request = _make_request(mqtt_bridge=mqtt)
        result = _publish_mqtt_command(request, "d1", {"command": "test"})
        assert result is True

    def test_publish_mqtt_command_no_bridge(self):
        from app.routers.device_management import _publish_mqtt_command
        request = _make_request()
        result = _publish_mqtt_command(request, "d1", {"command": "test"})
        assert result is False
