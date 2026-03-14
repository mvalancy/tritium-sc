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
        """Called when any packet is received from the mesh."""
        if not packet:
            return

        decoded = packet.get("decoded", {})
        portnum = decoded.get("portnum", "")

        # Text messages
        if portnum == "TEXT_MESSAGE_APP" or "text" in decoded:
            self._handle_text_message(packet)

        # Position updates
        elif portnum == "POSITION_APP":
            from_id = packet.get("fromId", packet.get("from", ""))
            self._publish_position_from_packet(str(from_id), decoded)

        # Telemetry updates
        elif portnum == "TELEMETRY_APP":
            from_id = packet.get("fromId", packet.get("from", ""))
            self._publish_telemetry_from_packet(str(from_id), decoded)

    # -- Message handling ----------------------------------------------------

    def _handle_text_message(self, packet: dict) -> None:
        """Process and publish a text message from the mesh."""
        decoded = packet.get("decoded", {})
        text = decoded.get("text", decoded.get("payload", b"").decode("utf-8", errors="replace"))
        from_id = str(packet.get("fromId", packet.get("from", "")))
        to_id = str(packet.get("toId", packet.get("to", "")))
        channel = packet.get("channel", 0)

        msg = {
            "from_id": from_id,
            "to_id": to_id,
            "text": text,
            "channel": channel,
            "hop_limit": packet.get("hopLimit"),
            "hop_start": packet.get("hopStart"),
            "rx_snr": packet.get("rxSnr"),
            "rx_rssi": packet.get("rxRssi"),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

        from_hex = from_id.replace("!", "")
        topic = f"{TOPIC_PREFIX}/{from_hex}/message"
        self._mqtt_publish(topic, msg)

        self._message_count += 1
        log.info(
            "Mesh message from %s: %s",
            from_id,
            text[:80] if text else "(empty)",
        )

    def _publish_position_from_packet(self, from_id: str, decoded: dict) -> None:
        """Publish position update from a decoded position packet."""
        position = decoded.get("position", decoded)
        from_hex = from_id.replace("!", "")
        pos_data = {
            "node_id": from_id,
            "latitude": position.get("latitude"),
            "longitude": position.get("longitude"),
            "altitude": position.get("altitude"),
            "sats_in_view": position.get("satsInView"),
            "ground_speed": position.get("groundSpeed"),
            "ground_track": position.get("groundTrack"),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        self._mqtt_publish(f"{TOPIC_PREFIX}/{from_hex}/position", pos_data)

    def _publish_telemetry_from_packet(self, from_id: str, decoded: dict) -> None:
        """Publish telemetry from a decoded telemetry packet."""
        telemetry = decoded.get("telemetry", decoded)
        from_hex = from_id.replace("!", "")
        telem_data = {
            "node_id": from_id,
            "device_metrics": telemetry.get("deviceMetrics", {}),
            "environment_metrics": telemetry.get("environmentMetrics", {}),
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
        """Process a single node and publish its data to MQTT."""
        user = info.get("user", {})
        position = info.get("position", {})
        device_metrics = info.get("deviceMetrics", {})
        env_metrics = info.get("environmentMetrics", {})

        node_hex = node_id.replace("!", "")

        node_data = {
            "node_id": node_id,
            "node_num": user.get("num") or info.get("num"),
            "long_name": user.get("longName", ""),
            "short_name": user.get("shortName", ""),
            "hw_model": user.get("hwModel", ""),
            "firmware_version": info.get("firmwareVersion", ""),
            "role": user.get("role"),
            "mac_addr": user.get("macaddr"),
            "is_licensed": user.get("isLicensed", False),
            "position": {
                "latitude": position.get("latitude"),
                "longitude": position.get("longitude"),
                "altitude": position.get("altitude"),
                "precision_bits": position.get("precisionBits"),
                "time": position.get("time"),
                "sats_in_view": position.get("satsInView"),
            } if position else None,
            "device_metrics": {
                "battery_level": device_metrics.get("batteryLevel"),
                "voltage": device_metrics.get("voltage"),
                "channel_utilization": device_metrics.get("channelUtilization"),
                "air_util_tx": device_metrics.get("airUtilTx"),
                "uptime_seconds": device_metrics.get("uptimeSeconds"),
            } if device_metrics else None,
            "environment": {
                "temperature": env_metrics.get("temperature"),
                "relative_humidity": env_metrics.get("relativeHumidity"),
                "barometric_pressure": env_metrics.get("barometricPressure"),
            } if env_metrics else None,
            "snr": info.get("snr"),
            "rssi": info.get("rssi"),
            "last_heard": info.get("lastHeard"),
            "hops_away": info.get("hopsAway"),
            "is_favorite": info.get("isFavorite", False),
            "via_mqtt": info.get("viaMqtt", False),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }

        with self._lock:
            self._nodes[node_id] = node_data

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
