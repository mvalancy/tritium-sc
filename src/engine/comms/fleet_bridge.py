# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""FleetBridge — bridges tritium-edge fleet server WebSocket to internal EventBus.

Connects to the fleet server's WebSocket endpoint and injects device
heartbeats, registration events, and BLE sensor presence data into the
local EventBus.  Runs in a background thread with auto-reconnect,
mirroring the pattern used by MQTTBridge.

Fleet Server WebSocket Events:
    device_heartbeat   -> fleet.heartbeat   (device telemetry)
    device_registered  -> fleet.registered  (new device auto-registered)
    device_offline     -> fleet.offline     (missed heartbeats)
    ota_started        -> fleet.ota_started
    ota_result         -> fleet.ota_result
    command_sent       -> fleet.command_sent

BLE sensor data extracted from heartbeats:
    heartbeat.sensors.ble_scanner.devices -> fleet.ble_presence per device
"""

from __future__ import annotations

import json
import logging
import threading
import time
from typing import TYPE_CHECKING

from tritium_lib.models.correlation import (
    CorrelationEvent,
    CorrelationSummary,
    classify_correlation_severity,
    summarize_correlations,
)
from tritium_lib.models.fleet import FleetNode, FleetStatus, NodeStatus, fleet_health_score
from tritium_lib.models.topology import (
    FleetTopology,
    NetworkLink,
    analyze_connectivity,
    build_topology,
)
from tritium_lib.models.diagnostics import HeapTrend

if TYPE_CHECKING:
    from engine.comms.event_bus import EventBus

logger = logging.getLogger("amy.fleet")


class FleetBridge:
    """Bridges tritium-edge fleet server WebSocket <-> internal EventBus.

    Usage::

        bridge = FleetBridge(event_bus, ws_url="ws://192.168.86.9:8080/ws")
        bridge.start()
        ...
        bridge.stop()
    """

    def __init__(
        self,
        event_bus: EventBus,
        ws_url: str = "ws://192.168.86.9:8080/ws",
        rest_url: str | None = None,
        reconnect_interval: float = 5.0,
        ping_interval: float = 30.0,
        poll_interval: float = 10.0,
    ) -> None:
        self._event_bus = event_bus
        self._ws_url = ws_url
        # Derive REST base URL from ws_url if not explicitly provided
        if rest_url is not None:
            self._rest_url = rest_url.rstrip("/")
        else:
            # ws://host:port/ws -> http://host:port
            self._rest_url = ws_url.replace("ws://", "http://").replace("wss://", "https://").rstrip("/")
            if self._rest_url.endswith("/ws"):
                self._rest_url = self._rest_url[:-3]
        self._reconnect_interval = reconnect_interval
        self._ping_interval = ping_interval
        self._poll_interval = poll_interval
        self._ws = None
        self._connected = False
        self._running = False
        self._thread: threading.Thread | None = None
        self._poll_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        # Stats
        self._messages_received: int = 0
        self._last_error: str = ""
        self._last_heartbeat_ts: float = 0.0
        # Device tracking: device_id -> last heartbeat data
        self._devices: dict[str, dict] = {}
        # BLE presence cache from REST polling
        self._ble_presence: list[dict] = []
        # Config sync cache from REST polling
        self._config_sync: dict = {}
        # Dashboard cache from REST polling
        self._dashboard: dict = {}
        # Fleet health report cache
        self._health_report: dict = {}
        # Correlation events cache
        self._correlations: dict = {}
        # Network topology cache
        self._topology: dict = {}
        # Heap trend analysis cache
        self._heap_trends: dict = {}

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def rest_url(self) -> str:
        """Derive REST base URL from WebSocket URL."""
        return self._rest_url

    @property
    def stats(self) -> dict:
        return {
            "connected": self._connected,
            "ws_url": self._ws_url,
            "messages_received": self._messages_received,
            "last_error": self._last_error,
            "devices_tracked": len(self._devices),
            "device_ids": list(self._devices.keys()),
            "config_sync": dict(self._config_sync) if self._config_sync else {},
            "dashboard": dict(self._dashboard) if self._dashboard else {},
            "health_report": dict(self._health_report) if self._health_report else {},
            "correlations": dict(self._correlations) if self._correlations else {},
            "topology": dict(self._topology) if self._topology else {},
            "heap_trends": dict(self._heap_trends) if self._heap_trends else {},
        }

    @property
    def devices(self) -> dict[str, dict]:
        """Return dict of tracked devices (device_id -> last heartbeat data)."""
        return dict(self._devices)

    @property
    def ble_presence(self) -> list[dict]:
        """Return latest BLE presence data from REST polling."""
        return list(self._ble_presence)

    @property
    def config_sync(self) -> dict:
        """Return latest fleet config sync status from REST polling."""
        return dict(self._config_sync)

    @property
    def dashboard(self) -> dict:
        """Return latest fleet dashboard summary from REST polling."""
        return dict(self._dashboard)

    @property
    def health_report(self) -> dict:
        """Return latest fleet health report from REST polling."""
        return dict(self._health_report)

    @property
    def correlations(self) -> dict:
        """Return latest fleet correlation events from REST polling."""
        return dict(self._correlations)

    @property
    def topology(self) -> dict:
        """Return latest fleet network topology from REST polling."""
        return dict(self._topology)

    @property
    def heap_trends(self) -> dict:
        """Return latest fleet heap trend analysis from REST polling."""
        return dict(self._heap_trends)

    def start(self) -> None:
        """Connect to fleet server WebSocket in a background thread."""
        if self._running:
            return
        try:
            import websocket  # noqa: F401
        except ImportError:
            logger.warning("websocket-client not installed — fleet bridge disabled")
            self._last_error = "websocket-client not installed"
            return

        self._running = True
        self._thread = threading.Thread(
            target=self._run_loop,
            name="fleet-bridge",
            daemon=True,
        )
        self._thread.start()

        # Start REST polling thread for /api/devices and /api/presence/ble
        self._poll_thread = threading.Thread(
            target=self._poll_loop,
            name="fleet-poll",
            daemon=True,
        )
        self._poll_thread.start()

        logger.info(f"Fleet bridge starting, target: {self._ws_url}")

    def stop(self) -> None:
        """Disconnect from fleet server WebSocket."""
        self._running = False
        if self._ws is not None:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None
        self._connected = False
        logger.info("Fleet bridge stopped")

    # --- Background thread ---

    def _run_loop(self) -> None:
        """Main loop: connect, listen, reconnect on failure."""
        import websocket

        while self._running:
            try:
                self._ws = websocket.WebSocketApp(
                    self._ws_url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )
                self._ws.run_forever(
                    ping_interval=self._ping_interval,
                    ping_timeout=10,
                )
            except Exception as e:
                logger.debug(f"Fleet WebSocket error: {e}")
                self._last_error = str(e)

            self._connected = False
            if self._running:
                logger.info(
                    f"Fleet bridge reconnecting in {self._reconnect_interval}s"
                )
                time.sleep(self._reconnect_interval)

    # --- WebSocket callbacks ---

    def _on_open(self, ws) -> None:
        self._connected = True
        logger.info(f"Fleet bridge connected to {self._ws_url}")
        self._event_bus.publish("fleet.connected", {"ws_url": self._ws_url})

    def _on_close(self, ws, close_status_code, close_msg) -> None:
        self._connected = False
        logger.info(f"Fleet bridge disconnected (code={close_status_code})")
        self._event_bus.publish("fleet.disconnected", {
            "code": close_status_code,
            "message": close_msg,
        })

    def _on_error(self, ws, error) -> None:
        self._last_error = str(error)
        logger.debug(f"Fleet WebSocket error: {error}")

    def _on_message(self, ws, message: str) -> None:
        """Parse incoming fleet server message and inject into EventBus."""
        self._messages_received += 1
        try:
            msg = json.loads(message)
        except (json.JSONDecodeError, TypeError) as e:
            logger.debug(f"Fleet bridge bad payload: {e}")
            return

        event_type = msg.get("type", "")
        data = msg.get("data", {})
        timestamp = msg.get("timestamp", "")

        try:
            if event_type == "device_heartbeat":
                self._handle_heartbeat(data, timestamp)
            elif event_type == "device_registered":
                self._handle_registered(data, timestamp)
            elif event_type == "device_offline":
                self._handle_offline(data, timestamp)
            elif event_type in ("ota_started", "ota_result", "ota_scheduled",
                                "command_sent", "firmware_uploaded"):
                # Forward other fleet events with fleet.* prefix
                self._event_bus.publish(f"fleet.{event_type}", {
                    **data,
                    "server_timestamp": timestamp,
                })
            elif event_type == "pong":
                pass  # Keepalive response, ignore
            else:
                logger.debug(f"Fleet bridge unknown event: {event_type}")
        except Exception as e:
            logger.debug(f"Fleet bridge handler error for {event_type}: {e}")

    # --- Event handlers ---

    def _handle_heartbeat(self, data: dict, timestamp: str) -> None:
        """Process device_heartbeat -> fleet.heartbeat + fleet.ble_presence."""
        device_id = data.get("device_id", "unknown")
        self._devices[device_id] = data
        self._last_heartbeat_ts = time.monotonic()

        # Emit fleet.heartbeat with full device data
        self._event_bus.publish("fleet.heartbeat", {
            "device_id": device_id,
            "board": data.get("board", "unknown"),
            "version": data.get("version", "unknown"),
            "ip": data.get("ip"),
            "mac": data.get("mac"),
            "rssi": data.get("rssi"),
            "free_heap": data.get("free_heap"),
            "uptime_s": data.get("uptime_s"),
            "online": data.get("_online", True),
            "capabilities": data.get("capabilities", []),
            "server_timestamp": timestamp,
        })

        # Extract BLE sensor data and emit per-device presence events
        sensors = data.get("sensors", {})
        ble_scanner = sensors.get("ble_scanner", {})
        ble_devices = ble_scanner.get("devices", [])
        if ble_devices:
            for ble_dev in ble_devices:
                self._event_bus.publish("fleet.ble_presence", {
                    "reporter_id": device_id,
                    "ble_addr": ble_dev.get("addr", ""),
                    "ble_name": ble_dev.get("name", ""),
                    "rssi": ble_dev.get("rssi"),
                    "device_type": ble_dev.get("type", "unknown"),
                    "server_timestamp": timestamp,
                })

        # Also emit fleet.sensor for any non-BLE sensor data
        for sensor_type, sensor_data in sensors.items():
            if sensor_type == "ble_scanner":
                continue  # Already handled above
            self._event_bus.publish("fleet.sensor", {
                "device_id": device_id,
                "sensor_type": sensor_type,
                "data": sensor_data,
                "server_timestamp": timestamp,
            })

    def _handle_registered(self, data: dict, timestamp: str) -> None:
        """Process device_registered -> fleet.registered."""
        device_id = data.get("device_id", "unknown")
        self._devices[device_id] = data
        self._event_bus.publish("fleet.registered", {
            "device_id": device_id,
            "board": data.get("board", "unknown"),
            "mac": data.get("mac"),
            "tags": data.get("tags", []),
            "server_timestamp": timestamp,
        })
        logger.info(f"Fleet device registered: {device_id}")

    def _handle_offline(self, data: dict, timestamp: str) -> None:
        """Process device_offline -> fleet.offline."""
        device_id = data.get("device_id", "unknown")
        # Mark device as offline in our tracking
        if device_id in self._devices:
            self._devices[device_id]["_online"] = False
        self._event_bus.publish("fleet.offline", {
            "device_id": device_id,
            "server_timestamp": timestamp,
        })
        logger.info(f"Fleet device offline: {device_id}")

    # --- REST polling loop ---

    def _poll_loop(self) -> None:
        """Poll fleet server REST API for device list and BLE presence."""
        import urllib.request
        import urllib.error

        while self._running:
            try:
                self._poll_devices(urllib.request, urllib.error)
                self._poll_ble_presence(urllib.request, urllib.error)
                self._poll_node_diagnostics(urllib.request, urllib.error)
                self._poll_fleet_config(urllib.request, urllib.error)
                self._poll_fleet_dashboard(urllib.request, urllib.error)
                self._poll_fleet_health_report(urllib.request, urllib.error)
                self._poll_fleet_correlations(urllib.request, urllib.error)
                self._poll_fleet_topology(urllib.request, urllib.error)
                self._poll_fleet_heap_trends(urllib.request, urllib.error)
            except Exception as e:
                logger.debug(f"Fleet REST poll error: {e}")
            time.sleep(self._poll_interval)

    def _poll_devices(self, request_mod, error_mod) -> None:
        """GET /api/devices and emit fleet.device_update."""
        url = f"{self._rest_url}/api/devices"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                data = json.loads(resp.read().decode())
                devices = data if isinstance(data, list) else data.get("devices", [])
                for dev in devices:
                    device_id = dev.get("device_id") or dev.get("id", "unknown")
                    self._devices[device_id] = dev
                self._event_bus.publish("fleet.device_update", {
                    "devices": devices,
                    "count": len(devices),
                })
        except error_mod.URLError as e:
            logger.debug(f"Fleet REST /api/devices failed: {e}")
        except Exception as e:
            logger.debug(f"Fleet REST /api/devices error: {e}")

    def _poll_ble_presence(self, request_mod, error_mod) -> None:
        """GET /api/presence/ble and emit fleet.ble_presence."""
        url = f"{self._rest_url}/api/presence/ble"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                data = json.loads(resp.read().decode())
                devices = data if isinstance(data, list) else data.get("devices", [])
                self._ble_presence = devices
                self._event_bus.publish("fleet.ble_presence", {
                    "devices": devices,
                    "count": len(devices),
                })
        except error_mod.URLError:
            pass  # BLE endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/presence/ble error: {e}")

    def _poll_node_diagnostics(self, request_mod, error_mod) -> None:
        """Poll each node's /api/diag endpoint directly for real-time diagnostics.

        For each tracked device with an IP, fetch its diagnostic report and emit
        fleet.node_diag events. This enables the command center to show per-node
        health, anomalies, and diagnostic events in real time.
        """
        for device_id, dev_data in list(self._devices.items()):
            ip = dev_data.get("ip")
            port = dev_data.get("port", 80)
            if not ip:
                continue
            url = f"http://{ip}:{port}/api/diag"
            try:
                req = request_mod.Request(url, method="GET")
                req.add_header("Accept", "application/json")
                with request_mod.urlopen(req, timeout=3) as resp:
                    diag = json.loads(resp.read().decode())
                    self._event_bus.publish("fleet.node_diag", {
                        "device_id": device_id,
                        "ip": ip,
                        "diagnostics": diag,
                    })
                    # Check for anomalies and emit separate events
                    anomalies = diag.get("anomalies", [])
                    if anomalies:
                        self._event_bus.publish("fleet.node_anomaly", {
                            "device_id": device_id,
                            "anomalies": anomalies,
                            "count": len(anomalies),
                        })
            except (error_mod.URLError, Exception):
                pass  # Node may be offline or diag not enabled

    def _poll_fleet_config(self, request_mod, error_mod) -> None:
        """GET /api/fleet/config and emit fleet.config_sync.

        Fetches the fleet-wide configuration sync status so the command center
        can show which nodes are running the expected firmware version and
        settings.  The response is expected to contain at minimum:
            - config_version: current fleet config revision
            - nodes_synced: count of nodes matching the current config
            - nodes_total: total managed nodes
            - nodes_pending: list of device_ids still pending sync
        """
        url = f"{self._rest_url}/api/fleet/config"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                raw = json.loads(resp.read().decode())
                data = raw if isinstance(raw, dict) else {}
                self._config_sync = data
                self._event_bus.publish("fleet.config_sync", {
                    "config_version": data.get("config_version", "unknown"),
                    "nodes_synced": data.get("nodes_synced", 0),
                    "nodes_total": data.get("nodes_total", 0),
                    "nodes_pending": data.get("nodes_pending", []),
                })
        except error_mod.URLError:
            pass  # Config endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/fleet/config error: {e}")

    def _poll_fleet_dashboard(self, request_mod, error_mod) -> None:
        """GET /api/fleet/dashboard and emit fleet.dashboard.

        Fetches the combined dashboard summary (health + config + alerts)
        in a single call, reducing roundtrips for the command center UI.
        """
        url = f"{self._rest_url}/api/fleet/dashboard"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                raw = json.loads(resp.read().decode())
                data = raw if isinstance(raw, dict) else {}
                self._dashboard = data
                health = data.get("health", {})
                config = data.get("config", {})
                alerts = data.get("alerts", {})
                self._event_bus.publish("fleet.dashboard", {
                    "health_score": health.get("score", 0),
                    "total_nodes": health.get("total_nodes", 0),
                    "online_count": health.get("online_count", 0),
                    "synced_count": config.get("synced_count", 0),
                    "drifted_count": config.get("drifted_count", 0),
                    "sync_ratio": config.get("sync_ratio", 1.0),
                    "alert_count": alerts.get("recent_count", 0),
                    "critical_alerts": alerts.get("critical", 0),
                    "server_uptime_s": data.get("server_uptime_s", 0),
                })
        except error_mod.URLError:
            pass  # Dashboard endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/fleet/dashboard error: {e}")

    def _poll_fleet_health_report(self, request_mod, error_mod) -> None:
        """GET /api/fleet/health-report and emit fleet.health_report.

        Fetches per-node health classification and fleet-wide anomaly
        detection from tritium-lib.  Computes fleet health score via
        the tritium-lib FleetStatus model when node data is available.
        Emits separate events for anomalies to enable command center alerting.
        """
        url = f"{self._rest_url}/api/fleet/health-report"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                raw = json.loads(resp.read().decode())
                data = raw if isinstance(raw, dict) else {}
                self._health_report = data

                total = data.get("total_nodes", 0)
                healthy = data.get("healthy", 0)
                warning = data.get("warning", 0)
                critical = data.get("critical", 0)

                # Compute fleet health score from tracked devices using
                # tritium-lib's FleetStatus model when we have device data
                health_score = data.get("health_score", 0.0)
                if not health_score and self._devices:
                    try:
                        fleet_nodes = []
                        for dev_id, dev in self._devices.items():
                            online = dev.get("_online", True)
                            fleet_nodes.append(FleetNode(
                                device_id=dev_id,
                                mac=dev.get("mac", ""),
                                ip=dev.get("ip", ""),
                                firmware_version=dev.get("version", "unknown"),
                                uptime_s=dev.get("uptime_s", 0),
                                wifi_rssi=dev.get("rssi", 0) or 0,
                                free_heap=dev.get("free_heap", 0) or 0,
                                status=NodeStatus.ONLINE if online else NodeStatus.OFFLINE,
                                capabilities=dev.get("capabilities", []),
                            ))
                        fleet_status = FleetStatus(
                            nodes=fleet_nodes,
                            total_nodes=len(fleet_nodes),
                            online_count=sum(1 for n in fleet_nodes if n.status == NodeStatus.ONLINE),
                        )
                        health_score = fleet_health_score(fleet_status)
                    except Exception:
                        pass

                self._event_bus.publish("fleet.health_report", {
                    "total_nodes": total,
                    "healthy": healthy,
                    "warning": warning,
                    "critical": critical,
                    "anomaly_count": data.get("anomaly_count", 0),
                    "health_score": health_score,
                })
                # Emit individual anomaly events for alerting
                anomalies = data.get("anomalies", [])
                if anomalies:
                    self._event_bus.publish("fleet.anomalies", {
                        "anomalies": anomalies,
                        "count": len(anomalies),
                    })
        except error_mod.URLError:
            pass  # Health report endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/fleet/health-report error: {e}")

    def _poll_fleet_correlations(self, request_mod, error_mod) -> None:
        """GET /api/fleet/correlations and emit fleet.correlations.

        Fetches cross-node event correlations: synchronized reboots,
        cascading failures, and other correlated events with confidence
        scores for the command center alerting UI.  Parses into
        tritium-lib CorrelationEvent models for type-safe severity
        classification and summary generation.
        """
        url = f"{self._rest_url}/api/fleet/correlations"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                raw = json.loads(resp.read().decode())
                data = raw if isinstance(raw, dict) else {"correlations": raw}
                self._correlations = data
                raw_correlations = data.get("correlations", [])

                # Parse into typed models for severity classification
                typed_events: list[CorrelationEvent] = []
                enriched: list[dict] = []
                for c in raw_correlations:
                    try:
                        event = CorrelationEvent(**c)
                        typed_events.append(event)
                        enriched.append({
                            **c,
                            "severity": classify_correlation_severity(event),
                            "devices_involved": event.devices_involved,
                        })
                    except Exception:
                        # Keep raw dict if it doesn't parse cleanly
                        enriched.append(c)

                summary = summarize_correlations(typed_events)
                self._event_bus.publish("fleet.correlations", {
                    "correlations": enriched,
                    "count": len(enriched),
                    "summary": summary,
                })
        except error_mod.URLError:
            pass  # Correlations endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/fleet/correlations error: {e}")

    def _poll_fleet_topology(self, request_mod, error_mod) -> None:
        """GET /api/fleet/topology and emit fleet.topology.

        Fetches the fleet network topology graph with node connectivity,
        link quality metrics, and network structure for visualization
        in the command center.  Runs connectivity analysis via tritium-lib
        to provide isolated/connected node counts and transport breakdown.
        """
        url = f"{self._rest_url}/api/fleet/topology"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                raw = json.loads(resp.read().decode())
                data = raw if isinstance(raw, dict) else {"nodes": raw}
                self._topology = data
                nodes = data.get("nodes", [])
                edges = data.get("edges", [])

                # Build typed topology and run connectivity analysis
                connectivity = {}
                try:
                    links = [NetworkLink(**e) for e in edges if isinstance(e, dict)]
                    topo = build_topology(links)
                    # Merge in any nodes from the response that aren't in links
                    node_ids = set(topo.nodes)
                    for n in nodes:
                        nid = n.get("id", n) if isinstance(n, dict) else str(n)
                        if nid not in node_ids:
                            topo.nodes.append(nid)
                    report = analyze_connectivity(topo)
                    connectivity = report.model_dump()
                except Exception:
                    pass  # Topology data may not match expected schema

                self._event_bus.publish("fleet.topology", {
                    "nodes": nodes,
                    "edges": edges,
                    "connectivity": connectivity,
                })
        except error_mod.URLError:
            pass  # Topology endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/fleet/topology error: {e}")

    def _poll_fleet_heap_trends(self, request_mod, error_mod) -> None:
        """GET /api/fleet/heap-trends and emit fleet.heap_trends.

        Fetches per-node heap memory trend analysis including suspected
        memory leaks, trend direction, and projected time-to-exhaustion.
        Validates trends against tritium-lib HeapTrend model for
        consistent field names and leak detection.
        """
        url = f"{self._rest_url}/api/fleet/heap-trends"
        try:
            req = request_mod.Request(url, method="GET")
            req.add_header("Accept", "application/json")
            with request_mod.urlopen(req, timeout=5) as resp:
                raw = json.loads(resp.read().decode())
                data = raw if isinstance(raw, dict) else {"trends": raw}
                self._heap_trends = data
                raw_trends = data.get("trends", [])

                # Parse into typed models for consistent leak detection
                typed_trends: list[dict] = []
                leak_suspects: list[dict] = []
                for t in raw_trends:
                    try:
                        ht = HeapTrend(**t)
                        trend_dict = ht.model_dump()
                        typed_trends.append(trend_dict)
                        if ht.leak_suspected:
                            leak_suspects.append(trend_dict)
                    except Exception:
                        typed_trends.append(t)

                # Fall back to raw leak_suspects if model parsing didn't find any
                if not leak_suspects:
                    leak_suspects = data.get("leak_suspects", [])

                self._event_bus.publish("fleet.heap_trends", {
                    "trends": typed_trends,
                    "leak_suspects": leak_suspects,
                    "count": len(typed_trends),
                })
        except error_mod.URLError:
            pass  # Heap trends endpoint may not exist on all fleet servers
        except Exception as e:
            logger.debug(f"Fleet REST /api/fleet/heap-trends error: {e}")
