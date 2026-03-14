# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Device Management API — remote OTA, reboot, config, and screenshot for edge devices.

Provides endpoints for managing tritium-edge devices from the SC dashboard:
- Reboot via MQTT command
- OTA firmware push via fleet server proxy or direct HTTP
- Configuration read/write via fleet server or device HTTP
- Screenshot request via device HTTP
"""

from __future__ import annotations

import json
import logging
import urllib.error
import urllib.request

from fastapi import APIRouter, HTTPException, Request, UploadFile, File
from fastapi.responses import JSONResponse
from pydantic import BaseModel

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/devices", tags=["device-management"])


class ConfigUpdateRequest(BaseModel):
    """Payload for device configuration update."""
    wifi: dict | None = None
    mqtt: dict | None = None
    display: dict | None = None
    power: dict | None = None
    extra: dict | None = None


class BulkDeviceRequest(BaseModel):
    """Payload for bulk device operations."""
    device_ids: list[str]
    action: str  # "reboot" or "ota"


def _get_fleet_url(request: Request) -> str:
    """Resolve fleet server base URL from bridge or settings."""
    bridge = getattr(request.app.state, "fleet_bridge", None)
    if bridge is not None:
        return bridge.rest_url
    try:
        from app.config import settings
        return settings.fleet_server_url.rstrip("/")
    except Exception:
        return "http://localhost:8080"


def _get_mqtt_bridge(request: Request):
    """Get MQTT bridge from app state."""
    return getattr(request.app.state, "mqtt_bridge", None)


def _get_site_id(request: Request) -> str:
    """Get MQTT site ID from app state."""
    return getattr(request.app.state, "mqtt_site_id", "home")


def _proxy_get(url: str, timeout: float = 5.0) -> dict | None:
    """GET a JSON endpoint."""
    try:
        req = urllib.request.Request(url, method="GET")
        req.add_header("Accept", "application/json")
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except Exception as e:
        logger.debug("Device management GET %s failed: %s", url, e)
        return None


def _proxy_post(url: str, data: bytes | None = None,
                content_type: str = "application/json",
                timeout: float = 10.0) -> dict | None:
    """POST data to a URL and return JSON response."""
    try:
        req = urllib.request.Request(url, data=data, method="POST")
        req.add_header("Content-Type", content_type)
        req.add_header("Accept", "application/json")
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except Exception as e:
        logger.debug("Device management POST %s failed: %s", url, e)
        return None


def _publish_mqtt_command(request: Request, device_id: str, command: dict) -> bool:
    """Publish a command to a device via MQTT. Returns True on success."""
    mqtt_bridge = _get_mqtt_bridge(request)
    if mqtt_bridge is None:
        return False
    site_id = _get_site_id(request)
    topic = f"tritium/{site_id}/devices/{device_id}/command"
    try:
        mqtt_bridge.publish(topic, json.dumps(command))
        logger.info("Device management MQTT command: %s -> %s", device_id, command.get("command"))
        return True
    except Exception as e:
        logger.warning("MQTT publish failed for %s: %s", device_id, e)
        return False


def _get_device_ip(request: Request, device_id: str) -> str | None:
    """Look up device IP from fleet bridge cache."""
    bridge = getattr(request.app.state, "fleet_bridge", None)
    if bridge is not None:
        dev = bridge.devices.get(device_id)
        if dev:
            return dev.get("ip")
    # Also check fleet dashboard plugin
    app = request.app
    plugin_mgr = getattr(app.state, "plugin_manager", None)
    if plugin_mgr is not None:
        for plugin in getattr(plugin_mgr, "_plugins", {}).values():
            get_device = getattr(plugin, "get_device", None)
            if get_device:
                dev = get_device(device_id)
                if dev and dev.get("ip"):
                    return dev["ip"]
    return None


# ===================================================================
# POST /api/devices/{device_id}/reboot
# ===================================================================

@router.post("/{device_id}/reboot")
async def reboot_device(request: Request, device_id: str):
    """Send reboot command to an edge device via MQTT or fleet server.

    Tries MQTT first (direct command), then falls back to fleet server
    REST endpoint.
    """
    command = {"command": "reboot"}

    # Try MQTT
    if _publish_mqtt_command(request, device_id, command):
        return {"status": "ok", "device_id": device_id, "action": "reboot", "via": "mqtt"}

    # Fallback: fleet server REST
    base = _get_fleet_url(request)
    result = _proxy_post(
        f"{base}/api/devices/{device_id}/reboot",
        data=json.dumps(command).encode(),
    )
    if result is not None:
        return {"status": "ok", "device_id": device_id, "action": "reboot", "via": "fleet_server"}

    # Fallback: direct device HTTP
    ip = _get_device_ip(request, device_id)
    if ip:
        result = _proxy_post(
            f"http://{ip}/api/reboot",
            data=json.dumps(command).encode(),
            timeout=5.0,
        )
        if result is not None:
            return {"status": "ok", "device_id": device_id, "action": "reboot", "via": "direct"}

    return JSONResponse(
        status_code=503,
        content={"error": "No delivery channel available", "device_id": device_id},
    )


# ===================================================================
# POST /api/devices/{device_id}/ota
# ===================================================================

@router.post("/{device_id}/ota")
async def ota_flash_device(request: Request, device_id: str, firmware: UploadFile = File(...)):
    """Upload firmware binary and push OTA update to device.

    The firmware binary is forwarded to the fleet server OTA endpoint
    or pushed directly to the device via HTTP.
    """
    firmware_bytes = await firmware.read()
    if len(firmware_bytes) == 0:
        raise HTTPException(status_code=400, detail="Empty firmware file")

    filename = firmware.filename or "firmware.bin"

    # Try fleet server OTA endpoint first
    base = _get_fleet_url(request)
    try:
        import io
        import http.client
        from urllib.parse import urlparse

        parsed = urlparse(f"{base}/api/devices/{device_id}/ota")
        boundary = "----TritiumOTABoundary"
        body = (
            f"--{boundary}\r\n"
            f'Content-Disposition: form-data; name="firmware"; filename="{filename}"\r\n'
            f"Content-Type: application/octet-stream\r\n\r\n"
        ).encode() + firmware_bytes + f"\r\n--{boundary}--\r\n".encode()

        conn = http.client.HTTPConnection(parsed.hostname, parsed.port or 80, timeout=30)
        conn.request(
            "POST",
            parsed.path,
            body=body,
            headers={
                "Content-Type": f"multipart/form-data; boundary={boundary}",
                "Accept": "application/json",
            },
        )
        resp = conn.getresponse()
        if 200 <= resp.status < 300:
            resp_data = json.loads(resp.read().decode())
            return {
                "status": "ok",
                "device_id": device_id,
                "action": "ota",
                "via": "fleet_server",
                "firmware_size": len(firmware_bytes),
                "filename": filename,
                "response": resp_data,
            }
        conn.close()
    except Exception as e:
        logger.debug("Fleet server OTA push failed for %s: %s", device_id, e)

    # Fallback: direct device OTA via HTTP
    ip = _get_device_ip(request, device_id)
    if ip:
        try:
            import http.client
            boundary = "----TritiumOTABoundary"
            body = (
                f"--{boundary}\r\n"
                f'Content-Disposition: form-data; name="firmware"; filename="{filename}"\r\n'
                f"Content-Type: application/octet-stream\r\n\r\n"
            ).encode() + firmware_bytes + f"\r\n--{boundary}--\r\n".encode()

            conn = http.client.HTTPConnection(ip, 80, timeout=60)
            conn.request(
                "POST",
                "/api/ota",
                body=body,
                headers={
                    "Content-Type": f"multipart/form-data; boundary={boundary}",
                },
            )
            resp = conn.getresponse()
            if 200 <= resp.status < 300:
                return {
                    "status": "ok",
                    "device_id": device_id,
                    "action": "ota",
                    "via": "direct",
                    "firmware_size": len(firmware_bytes),
                    "filename": filename,
                }
            conn.close()
        except Exception as e:
            logger.debug("Direct OTA push failed for %s: %s", device_id, e)

    # Notify via MQTT that OTA is available (device can pull)
    mqtt_cmd = {
        "command": "ota_available",
        "firmware_size": len(firmware_bytes),
        "filename": filename,
    }
    if _publish_mqtt_command(request, device_id, mqtt_cmd):
        return {
            "status": "queued",
            "device_id": device_id,
            "action": "ota",
            "via": "mqtt_notification",
            "firmware_size": len(firmware_bytes),
            "filename": filename,
            "note": "Device notified via MQTT; firmware push requires fleet server or direct HTTP",
        }

    return JSONResponse(
        status_code=503,
        content={"error": "No OTA delivery channel available", "device_id": device_id},
    )


# ===================================================================
# GET /api/devices/{device_id}/config
# ===================================================================

@router.get("/{device_id}/config")
async def get_device_config(request: Request, device_id: str):
    """Get device configuration from fleet server or direct device HTTP."""
    # Try fleet server
    base = _get_fleet_url(request)
    data = _proxy_get(f"{base}/api/devices/{device_id}/config")
    if data is not None:
        return {"device_id": device_id, "config": data, "source": "fleet_server"}

    # Try direct device HTTP
    ip = _get_device_ip(request, device_id)
    if ip:
        data = _proxy_get(f"http://{ip}/api/config")
        if data is not None:
            return {"device_id": device_id, "config": data, "source": "direct"}

    # Fallback: construct config from cached heartbeat data
    bridge = getattr(request.app.state, "fleet_bridge", None)
    if bridge is not None:
        dev = bridge.devices.get(device_id)
        if dev:
            return {
                "device_id": device_id,
                "config": {
                    "firmware": dev.get("version", dev.get("firmware", "unknown")),
                    "ip": dev.get("ip", ""),
                    "mac": dev.get("mac", ""),
                    "board": dev.get("board", ""),
                    "capabilities": dev.get("capabilities", []),
                },
                "source": "cached",
                "note": "Partial config from heartbeat data",
            }

    raise HTTPException(status_code=404, detail=f"Device {device_id} not found or unreachable")


# ===================================================================
# PUT /api/devices/{device_id}/config
# ===================================================================

@router.put("/{device_id}/config")
async def update_device_config(request: Request, device_id: str, body: ConfigUpdateRequest):
    """Update device configuration via MQTT command or fleet server."""
    config_payload = {}
    if body.wifi is not None:
        config_payload["wifi"] = body.wifi
    if body.mqtt is not None:
        config_payload["mqtt"] = body.mqtt
    if body.display is not None:
        config_payload["display"] = body.display
    if body.power is not None:
        config_payload["power"] = body.power
    if body.extra is not None:
        config_payload.update(body.extra)

    if not config_payload:
        raise HTTPException(status_code=400, detail="No configuration fields provided")

    command = {"command": "config_update", "config": config_payload}

    # Try MQTT
    if _publish_mqtt_command(request, device_id, command):
        return {
            "status": "ok",
            "device_id": device_id,
            "action": "config_update",
            "via": "mqtt",
            "fields_updated": list(config_payload.keys()),
        }

    # Fallback: fleet server
    base = _get_fleet_url(request)
    result = _proxy_post(
        f"{base}/api/devices/{device_id}/config",
        data=json.dumps(config_payload).encode(),
    )
    if result is not None:
        return {
            "status": "ok",
            "device_id": device_id,
            "action": "config_update",
            "via": "fleet_server",
            "fields_updated": list(config_payload.keys()),
        }

    # Fallback: direct device HTTP
    ip = _get_device_ip(request, device_id)
    if ip:
        result = _proxy_post(
            f"http://{ip}/api/config",
            data=json.dumps(config_payload).encode(),
        )
        if result is not None:
            return {
                "status": "ok",
                "device_id": device_id,
                "action": "config_update",
                "via": "direct",
                "fields_updated": list(config_payload.keys()),
            }

    return JSONResponse(
        status_code=503,
        content={"error": "No delivery channel available", "device_id": device_id},
    )


# ===================================================================
# POST /api/devices/{device_id}/screenshot
# ===================================================================

@router.post("/{device_id}/screenshot")
async def request_screenshot(request: Request, device_id: str):
    """Request a screenshot from an edge device.

    Tries direct device HTTP first (returns the screenshot URL),
    then falls back to MQTT notification.
    """
    # Try direct device HTTP
    ip = _get_device_ip(request, device_id)
    if ip:
        data = _proxy_get(f"http://{ip}/api/screenshot")
        if data is not None:
            return {
                "status": "ok",
                "device_id": device_id,
                "action": "screenshot",
                "via": "direct",
                "url": f"http://{ip}/api/screenshot/latest",
                "data": data,
            }

    # Try MQTT command
    command = {"command": "screenshot"}
    if _publish_mqtt_command(request, device_id, command):
        return {
            "status": "requested",
            "device_id": device_id,
            "action": "screenshot",
            "via": "mqtt",
            "note": "Screenshot requested via MQTT; result will arrive on heartbeat topic",
        }

    return JSONResponse(
        status_code=503,
        content={"error": "No delivery channel available", "device_id": device_id},
    )


# ===================================================================
# POST /api/devices/bulk
# ===================================================================

@router.post("/bulk")
async def bulk_device_action(request: Request, body: BulkDeviceRequest):
    """Execute a bulk action (reboot or ota) on multiple devices."""
    if body.action not in ("reboot",):
        raise HTTPException(status_code=400, detail=f"Unsupported bulk action: {body.action}")

    if not body.device_ids:
        raise HTTPException(status_code=400, detail="No device IDs provided")

    results = []
    for device_id in body.device_ids:
        command = {"command": body.action}
        success = _publish_mqtt_command(request, device_id, command)
        if success:
            results.append({"device_id": device_id, "status": "ok", "via": "mqtt"})
        else:
            # Try fleet server fallback
            base = _get_fleet_url(request)
            fallback = _proxy_post(
                f"{base}/api/devices/{device_id}/{body.action}",
                data=json.dumps(command).encode(),
            )
            if fallback is not None:
                results.append({"device_id": device_id, "status": "ok", "via": "fleet_server"})
            else:
                results.append({"device_id": device_id, "status": "failed", "error": "no channel"})

    succeeded = sum(1 for r in results if r["status"] == "ok")
    return {
        "action": body.action,
        "total": len(body.device_ids),
        "succeeded": succeeded,
        "failed": len(body.device_ids) - succeeded,
        "results": results,
    }
