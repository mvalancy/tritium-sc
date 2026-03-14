#!/usr/bin/env python3
# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Meshtastic Bridge — connects a real Meshtastic radio to Tritium MQTT.

This daemon bridges a Meshtastic LoRa radio (via serial or TCP) into the
Tritium MQTT bus.  Every mesh node's position, telemetry, and messages
are published as Tritium MQTT topics for consumption by the Command Center.

Environment variables:
    MESHTASTIC_CONNECTION   serial|tcp (default: serial)
    MESHTASTIC_SERIAL_PORT  Serial port path (default: /dev/ttyACM0)
    MESHTASTIC_TCP_HOST     TCP host for remote radio (default: localhost)
    MESHTASTIC_TCP_PORT     TCP port (default: 4403)
    MQTT_HOST               MQTT broker host (default: localhost)
    MQTT_PORT               MQTT broker port (default: 1883)
    MQTT_SITE_ID            Tritium site ID (default: home)
    BRIDGE_ID               Bridge identifier (default: bridge0)
    NODE_POLL_INTERVAL      Seconds between full node polls (default: 30)
    RECONNECT_DELAY         Seconds before reconnect attempt (default: 5)
    LOG_LEVEL               Logging level (default: INFO)

Usage:
    python3 scripts/meshtastic-bridge.py

    # Or on a remote host with the radio:
    MESHTASTIC_SERIAL_PORT=/dev/ttyACM0 \\
    MQTT_HOST=192.168.86.7 \\
    python3 scripts/meshtastic-bridge.py
"""

from __future__ import annotations

import json
import logging
import os
import signal
import sys
import time
import threading
from datetime import datetime, timezone
from typing import Any, Optional

# ---------------------------------------------------------------------------
# Configuration from environment
# ---------------------------------------------------------------------------
CONNECTION_TYPE = os.environ.get("MESHTASTIC_CONNECTION", "serial")
SERIAL_PORT = os.environ.get("MESHTASTIC_SERIAL_PORT", "/dev/ttyACM0")
TCP_HOST = os.environ.get("MESHTASTIC_TCP_HOST", "localhost")
TCP_PORT = int(os.environ.get("MESHTASTIC_TCP_PORT", "4403"))
MQTT_HOST = os.environ.get("MQTT_HOST", "localhost")
MQTT_PORT = int(os.environ.get("MQTT_PORT", "1883"))
MQTT_SITE_ID = os.environ.get("MQTT_SITE_ID", "home")
BRIDGE_ID = os.environ.get("BRIDGE_ID", "bridge0")
NODE_POLL_INTERVAL = float(os.environ.get("NODE_POLL_INTERVAL", "30"))
RECONNECT_DELAY = float(os.environ.get("RECONNECT_DELAY", "5"))
LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO").upper()

# MQTT topic templates
TOPIC_PREFIX = f"tritium/{MQTT_SITE_ID}/meshtastic"

logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("meshtastic-bridge")

# --- Input sanitization ---

# Characters that are dangerous in MQTT topics: / (separator), + (single-level
# wildcard), # (multi-level wildcard), and NUL.  Node IDs from the radio could
# theoretically contain these if the radio firmware is malicious or buggy.
_MQTT_TOPIC_UNSAFE = set("/+#\x00")

# Maximum lengths to prevent memory abuse from a rogue mesh node
_MAX_NODE_ID_LEN = 64
_MAX_TEXT_LEN = 512      # LoRa max is 228, but be generous
_MAX_NAME_LEN = 128
_MAX_HW_MODEL_LEN = 64


def _sanitize_topic_segment(raw: str) -> str:
    """Remove MQTT-unsafe characters from a string used in a topic path.

    Strips /, +, #, and NUL bytes.  Also enforces a length limit so a
    rogue node cannot create arbitrarily long topics.
    """
    cleaned = "".join(ch for ch in str(raw) if ch not in _MQTT_TOPIC_UNSAFE)
    return cleaned[:_MAX_NODE_ID_LEN]


def _sanitize_text(raw: str, max_len: int = _MAX_TEXT_LEN) -> str:
    """Truncate and strip NUL bytes from user-supplied text."""
    if not isinstance(raw, str):
        raw = str(raw)
    return raw.replace("\x00", "")[:max_len]


def _safe_str(value, max_len: int = _MAX_NAME_LEN) -> str:
    """Safely convert a value to a bounded string."""
    if value is None:
        return ""
    return str(value).replace("\x00", "")[:max_len]


def _safe_float(value) -> float | None:
    """Safely convert a value to float, returning None on failure."""
    if value is None:
        return None
    try:
        f = float(value)
        # Reject NaN/Inf which could cause issues downstream
        if f != f or f == float('inf') or f == float('-inf'):
            return None
        return f
    except (ValueError, TypeError):
        return None


def _safe_int(value) -> int | None:
    """Safely convert a value to int, returning None on failure."""
    if value is None:
        return None
    try:
        return int(value)
    except (ValueError, TypeError):
        return None


class MeshtasticBridge:
    """Long-lived daemon bridging Meshtastic radio to Tritium MQTT."""

    def __init__(self) -> None:
        self._mesh_interface: Any = None
        self._mqtt_client: Any = None
        self._running = False
        self._nodes: dict[str, dict] = {}
        self._message_count = 0
        self._lock = threading.Lock()

    # -- MQTT connection -----------------------------------------------------

    def _connect_mqtt(self) -> bool:
        """Connect to the MQTT broker. Returns True on success."""
        try:
            import paho.mqtt.client as mqtt
        except ImportError:
            log.error("paho-mqtt not installed: pip install paho-mqtt")
            return False

        try:
            client = mqtt.Client(
                client_id=f"meshtastic-bridge-{BRIDGE_ID}",
                protocol=mqtt.MQTTv311,
            )
            client.will_set(
                f"{TOPIC_PREFIX}/{BRIDGE_ID}/status",
                payload=json.dumps({"online": False, "bridge_id": BRIDGE_ID}),
                qos=1,
                retain=True,
            )
            client.on_connect = self._on_mqtt_connect
            client.on_disconnect = self._on_mqtt_disconnect
            client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
            client.loop_start()
            self._mqtt_client = client
            log.info("MQTT connected to %s:%d", MQTT_HOST, MQTT_PORT)
            return True
        except Exception as exc:
            log.error("MQTT connection failed: %s", exc)
            return False

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """Publish online status on MQTT connect."""
        status = {
            "online": True,
            "bridge_id": BRIDGE_ID,
            "connection_type": CONNECTION_TYPE,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        client.publish(
            f"{TOPIC_PREFIX}/{BRIDGE_ID}/status",
            payload=json.dumps(status),
            qos=1,
            retain=True,
        )
        log.info("MQTT bridge online (rc=%d)", rc)

    def _on_mqtt_disconnect(self, client, userdata, rc):
        if rc != 0:
            log.warning("MQTT disconnected unexpectedly (rc=%d)", rc)

    # -- Meshtastic connection -----------------------------------------------

    def _connect_meshtastic(self) -> bool:
        """Connect to the Meshtastic radio. Returns True on success."""
        try:
            if CONNECTION_TYPE == "serial":
                from meshtastic.serial_interface import SerialInterface
                log.info("Connecting to serial port %s ...", SERIAL_PORT)
                self._mesh_interface = SerialInterface(
                    devPath=SERIAL_PORT, connectNow=True
                )
            elif CONNECTION_TYPE == "tcp":
                from meshtastic.tcp_interface import TCPInterface
                log.info("Connecting to TCP %s:%d ...", TCP_HOST, TCP_PORT)
                self._mesh_interface = TCPInterface(
                    hostname=TCP_HOST, portNumber=TCP_PORT
                )
            else:
                log.error("Unknown connection type: %s", CONNECTION_TYPE)
                return False

            log.info("Meshtastic radio connected via %s", CONNECTION_TYPE)
            return True

        except Exception as exc:
            log.error("Meshtastic connection failed: %s", exc)
            self._mesh_interface = None
            return False

    def _disconnect_meshtastic(self) -> None:
        """Cleanly close the Meshtastic interface."""
        if self._mesh_interface is not None:
            try:
                self._mesh_interface.close()
            except Exception as exc:
                log.warning("Error closing meshtastic interface: %s", exc)
            self._mesh_interface = None

    # -- PubSub callbacks (meshtastic library) --------------------------------

    def _setup_pubsub(self) -> None:
        """Subscribe to meshtastic library pubsub events."""
        from pubsub import pub

        pub.subscribe(self._on_receive, "meshtastic.receive")
        pub.subscribe(self._on_connection, "meshtastic.connection.established")
        pub.subscribe(self._on_connection_lost, "meshtastic.connection.lost")
        pub.subscribe(self._on_node_updated, "meshtastic.node.updated")
        log.info("PubSub callbacks registered")

    def _on_connection(self, interface, topic=None) -> None:
        """Called when meshtastic radio connection is established."""
        log.info("Meshtastic connection established")
        # Do an initial full poll
        self._poll_all_nodes()

    def _on_connection_lost(self, interface, topic=None) -> None:
        """Called when meshtastic radio connection is lost."""
        log.warning("Meshtastic connection lost — will reconnect")

    def _on_node_updated(self, node, **kwargs) -> None:
        """Called when a node in the meshtastic DB changes."""
        if not node or not isinstance(node, dict):
            return
        node_id = node.get("user", {}).get("id") or node.get("num")
        if node_id:
            self._process_node(str(node_id), node)

    def _on_receive(self, packet, interface=None) -> None:
        """Called when any packet is received from the mesh.

        Security: packet data comes from the radio and could be crafted
        by a malicious mesh node.  All handlers sanitize their inputs
        before constructing MQTT topics or payloads.
        """
        if not packet or not isinstance(packet, dict):
            return

        decoded = packet.get("decoded", {})
        if not isinstance(decoded, dict):
            return
        portnum = str(decoded.get("portnum", ""))

        # Text messages
        if portnum == "TEXT_MESSAGE_APP" or "text" in decoded:
            self._handle_text_message(packet)

        # Position updates
        elif portnum == "POSITION_APP":
            from_id = _safe_str(packet.get("fromId", packet.get("from", "")))
            self._publish_position_from_packet(from_id, decoded)

        # Telemetry updates
        elif portnum == "TELEMETRY_APP":
            from_id = _safe_str(packet.get("fromId", packet.get("from", "")))
            self._publish_telemetry_from_packet(from_id, decoded)

    # -- Message handling ----------------------------------------------------

    def _handle_text_message(self, packet: dict) -> None:
        """Process and publish a text message from the mesh.

        Security: all fields are sanitized before use in MQTT topics or
        payloads.  Node IDs are stripped of MQTT-unsafe characters (/, +, #)
        to prevent topic injection.  Text is truncated to prevent memory abuse.
        """
        decoded = packet.get("decoded", {})
        raw_text = decoded.get("text", "")
        if not raw_text and isinstance(decoded.get("payload"), bytes):
            raw_text = decoded["payload"].decode("utf-8", errors="replace")
        text = _sanitize_text(raw_text)
        from_id = _safe_str(packet.get("fromId", packet.get("from", "")))
        to_id = _safe_str(packet.get("toId", packet.get("to", "")))
        channel = _safe_int(packet.get("channel", 0)) or 0

        msg = {
            "from_id": from_id,
            "to_id": to_id,
            "text": text,
            "channel": channel,
            "hop_limit": _safe_int(packet.get("hopLimit")),
            "hop_start": _safe_int(packet.get("hopStart")),
            "rx_snr": _safe_float(packet.get("rxSnr")),
            "rx_rssi": _safe_int(packet.get("rxRssi")),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

        from_hex = _sanitize_topic_segment(from_id.replace("!", ""))
        if not from_hex:
            log.warning("Dropping text message with empty/unsafe from_id")
            return
        topic = f"{TOPIC_PREFIX}/{from_hex}/message"
        self._mqtt_publish(topic, msg)

        self._message_count += 1
        log.info(
            "Mesh message from %s: %s",
            from_id,
            text[:80] if text else "(empty)",
        )

    def _publish_position_from_packet(self, from_id: str, decoded: dict) -> None:
        """Publish position update from a decoded position packet.

        Security: coordinates are validated as floats, node ID is sanitized
        for MQTT topic safety.
        """
        position = decoded.get("position", decoded)
        from_hex = _sanitize_topic_segment(from_id.replace("!", ""))
        if not from_hex:
            return
        pos_data = {
            "node_id": _safe_str(from_id),
            "latitude": _safe_float(position.get("latitude")),
            "longitude": _safe_float(position.get("longitude")),
            "altitude": _safe_float(position.get("altitude")),
            "sats_in_view": _safe_int(position.get("satsInView")),
            "ground_speed": _safe_float(position.get("groundSpeed")),
            "ground_track": _safe_float(position.get("groundTrack")),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._mqtt_publish(f"{TOPIC_PREFIX}/{from_hex}/position", pos_data)

    def _publish_telemetry_from_packet(self, from_id: str, decoded: dict) -> None:
        """Publish telemetry from a decoded telemetry packet.

        Security: all numeric values are validated, node ID is sanitized
        for MQTT topic safety.
        """
        telemetry = decoded.get("telemetry", decoded)
        from_hex = _sanitize_topic_segment(from_id.replace("!", ""))
        if not from_hex:
            return
        # Sanitize device metrics
        raw_dm = telemetry.get("deviceMetrics", {})
        safe_dm = {
            "batteryLevel": _safe_int(raw_dm.get("batteryLevel")),
            "voltage": _safe_float(raw_dm.get("voltage")),
            "channelUtilization": _safe_float(raw_dm.get("channelUtilization")),
            "airUtilTx": _safe_float(raw_dm.get("airUtilTx")),
            "uptimeSeconds": _safe_int(raw_dm.get("uptimeSeconds")),
        }
        # Sanitize environment metrics
        raw_env = telemetry.get("environmentMetrics", {})
        safe_env = {
            "temperature": _safe_float(raw_env.get("temperature")),
            "relativeHumidity": _safe_float(raw_env.get("relativeHumidity")),
            "barometricPressure": _safe_float(raw_env.get("barometricPressure")),
        }
        telem_data = {
            "node_id": _safe_str(from_id),
            "device_metrics": safe_dm,
            "environment_metrics": safe_env,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._mqtt_publish(f"{TOPIC_PREFIX}/{from_hex}/telemetry", telem_data)

    # -- Node polling --------------------------------------------------------

    def _poll_all_nodes(self) -> None:
        """Read the full node database from the radio and publish to MQTT."""
        if self._mesh_interface is None:
            return

        try:
            nodes = self._mesh_interface.nodes
            if not nodes:
                log.debug("No nodes in radio database")
                return

            log.info("Polling %d nodes from radio database", len(nodes))
            for node_id, node_info in nodes.items():
                self._process_node(node_id, node_info)

            # Publish aggregate node list
            self._publish_node_summary(nodes)

        except Exception as exc:
            log.error("Error polling nodes: %s", exc)

    def _process_node(self, node_id: str, info: dict) -> None:
        """Process a single node and publish its data to MQTT.

        Security: all fields are sanitized.  Node IDs are stripped of
        MQTT-unsafe characters to prevent topic injection.  String fields
        are length-bounded.  Numeric fields are type-validated.
        """
        user = info.get("user", {})
        position = info.get("position", {})
        device_metrics = info.get("deviceMetrics", {})
        env_metrics = info.get("environmentMetrics", {})

        node_hex = _sanitize_topic_segment(node_id.replace("!", ""))
        if not node_hex:
            log.warning("Dropping node with empty/unsafe ID: %r", node_id[:32])
            return

        node_data = {
            "node_id": _safe_str(node_id),
            "node_num": _safe_int(user.get("num") or info.get("num")),
            "long_name": _safe_str(user.get("longName", "")),
            "short_name": _safe_str(user.get("shortName", "")),
            "hw_model": _safe_str(user.get("hwModel", ""), _MAX_HW_MODEL_LEN),
            "firmware_version": _safe_str(info.get("firmwareVersion", "")),
            "role": _safe_str(user.get("role")),
            "mac_addr": _safe_str(user.get("macaddr")),
            "is_licensed": bool(user.get("isLicensed", False)),
            "position": {
                "latitude": _safe_float(position.get("latitude")),
                "longitude": _safe_float(position.get("longitude")),
                "altitude": _safe_float(position.get("altitude")),
                "precision_bits": _safe_int(position.get("precisionBits")),
                "time": _safe_int(position.get("time")),
                "sats_in_view": _safe_int(position.get("satsInView")),
            } if position else None,
            "device_metrics": {
                "battery_level": _safe_int(device_metrics.get("batteryLevel")),
                "voltage": _safe_float(device_metrics.get("voltage")),
                "channel_utilization": _safe_float(device_metrics.get("channelUtilization")),
                "air_util_tx": _safe_float(device_metrics.get("airUtilTx")),
                "uptime_seconds": _safe_int(device_metrics.get("uptimeSeconds")),
            } if device_metrics else None,
            "environment": {
                "temperature": _safe_float(env_metrics.get("temperature")),
                "relative_humidity": _safe_float(env_metrics.get("relativeHumidity")),
                "barometric_pressure": _safe_float(env_metrics.get("barometricPressure")),
            } if env_metrics else None,
            "snr": _safe_float(info.get("snr")),
            "rssi": _safe_int(info.get("rssi")),
            "last_heard": _safe_int(info.get("lastHeard")),
            "hops_away": _safe_int(info.get("hopsAway")),
            "is_favorite": bool(info.get("isFavorite", False)),
            "via_mqtt": bool(info.get("viaMqtt", False)),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

        with self._lock:
            self._nodes[_safe_str(node_id)] = node_data

        # Publish individual node update
        topic = f"{TOPIC_PREFIX}/{node_hex}/nodes"
        self._mqtt_publish(topic, node_data)

    def _publish_node_summary(self, nodes: dict) -> None:
        """Publish a summary of all known nodes."""
        summary = {
            "bridge_id": BRIDGE_ID,
            "node_count": len(nodes),
            "nodes_with_gps": sum(
                1 for n in nodes.values()
                if n.get("position", {}).get("latitude") is not None
            ),
            "nodes_with_battery": sum(
                1 for n in nodes.values()
                if n.get("deviceMetrics", {}).get("batteryLevel") is not None
            ),
            "message_count": self._message_count,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._mqtt_publish(f"{TOPIC_PREFIX}/{BRIDGE_ID}/summary", summary)

    # -- MQTT publishing -----------------------------------------------------

    def _mqtt_publish(self, topic: str, data: dict) -> None:
        """Publish a JSON message to MQTT."""
        if self._mqtt_client is None:
            return
        try:
            payload = json.dumps(data, default=str)
            self._mqtt_client.publish(topic, payload, qos=0)
        except Exception as exc:
            log.error("MQTT publish error on %s: %s", topic, exc)

    # -- Main loop -----------------------------------------------------------

    def run(self) -> None:
        """Main entry point — connect and run forever."""
        self._running = True
        log.info("=" * 60)
        log.info("Meshtastic Bridge starting")
        log.info("  Connection: %s", CONNECTION_TYPE)
        if CONNECTION_TYPE == "serial":
            log.info("  Serial port: %s", SERIAL_PORT)
        else:
            log.info("  TCP: %s:%d", TCP_HOST, TCP_PORT)
        log.info("  MQTT: %s:%d (site=%s)", MQTT_HOST, MQTT_PORT, MQTT_SITE_ID)
        log.info("  Poll interval: %ss", NODE_POLL_INTERVAL)
        log.info("=" * 60)

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Setup pubsub before connecting
        self._setup_pubsub()

        while self._running:
            # Connect MQTT
            if self._mqtt_client is None:
                if not self._connect_mqtt():
                    log.warning("MQTT connect failed, retrying in %ss", RECONNECT_DELAY)
                    time.sleep(RECONNECT_DELAY)
                    continue

            # Connect Meshtastic
            if self._mesh_interface is None:
                if not self._connect_meshtastic():
                    log.warning(
                        "Meshtastic connect failed, retrying in %ss",
                        RECONNECT_DELAY,
                    )
                    time.sleep(RECONNECT_DELAY)
                    continue

            # Poll loop
            try:
                self._poll_all_nodes()
                # Sleep in small increments so we can respond to signals
                sleep_remaining = NODE_POLL_INTERVAL
                while self._running and sleep_remaining > 0:
                    time.sleep(min(1.0, sleep_remaining))
                    sleep_remaining -= 1.0

            except KeyboardInterrupt:
                break
            except Exception as exc:
                log.error("Bridge loop error: %s", exc)
                # Assume connection lost — tear down and reconnect
                self._disconnect_meshtastic()
                time.sleep(RECONNECT_DELAY)

        self._shutdown()

    def _signal_handler(self, signum, frame):
        """Handle SIGINT/SIGTERM gracefully."""
        log.info("Signal %d received, shutting down...", signum)
        self._running = False

    def _shutdown(self) -> None:
        """Clean shutdown of all connections."""
        log.info("Shutting down bridge...")

        # Publish offline status
        if self._mqtt_client:
            self._mqtt_publish(
                f"{TOPIC_PREFIX}/{BRIDGE_ID}/status",
                {"online": False, "bridge_id": BRIDGE_ID},
            )
            try:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
            except Exception:
                pass

        self._disconnect_meshtastic()
        log.info("Bridge stopped")


def main():
    bridge = MeshtasticBridge()
    bridge.run()


if __name__ == "__main__":
    main()
