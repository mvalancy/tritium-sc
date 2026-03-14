# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""WiFiFingerprintPlugin — passive WiFi probe correlation with BLE sightings.

Listens for WiFi probe request events and BLE sighting events on the EventBus.
Correlates them temporally and spatially to build WiFi fingerprints for BLE
targets. The resulting correlations strengthen device identity and feed into
target dossiers.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

from engine.plugins.base import EventDrainPlugin, PluginContext

from .correlator import ProbeCorrelator
from .routes import create_router

log = logging.getLogger("wifi-fingerprint")

# Background prune interval (seconds)
PRUNE_INTERVAL = 600.0


class WiFiFingerprintPlugin(EventDrainPlugin):
    """Passive WiFi probe request to BLE device correlation."""

    def __init__(self) -> None:
        super().__init__()
        self._correlator = ProbeCorrelator()
        self._tracker: Any = None
        self._prune_thread: Optional[threading.Thread] = None

    # -- PluginInterface identity ----------------------------------------------

    @property
    def plugin_id(self) -> str:
        return "tritium.wifi-fingerprint"

    @property
    def name(self) -> str:
        return "WiFi Fingerprint"

    @property
    def version(self) -> str:
        return "1.0.0"

    @property
    def capabilities(self) -> set[str]:
        return {"data_source", "routes", "background"}

    # -- EventDrainPlugin overrides --------------------------------------------

    def _on_configure(self, ctx: PluginContext) -> None:
        """Store tracker reference, register routes."""
        self._tracker = ctx.target_tracker

        # Register FastAPI routes
        if self._app is not None:
            router = create_router(self._correlator)
            self._app.include_router(router)
            self._logger.info("WiFi fingerprint routes registered")

        self._logger.info("WiFi Fingerprint plugin configured")

    def _on_start(self) -> None:
        """Start background prune thread."""
        self._prune_thread = threading.Thread(
            target=self._prune_loop,
            daemon=True,
            name="wifi-fingerprint-prune",
        )
        self._prune_thread.start()
        self._logger.info("WiFi Fingerprint plugin started")

    def _on_stop(self) -> None:
        """Stop background threads."""
        self._logger.info("WiFi Fingerprint plugin stopped")

    def _handle_event(self, event: dict) -> None:
        """Process EventBus events for probe requests and BLE sightings."""
        event_type = event.get("type", event.get("event_type", ""))
        data = event.get("data", {})

        if event_type == "fleet.wifi_probe":
            self._handle_wifi_probe(data)
        elif event_type == "fleet.ble_presence":
            self._handle_ble_sighting(data)
        elif event_type == "wifi_probe_request":
            self._handle_wifi_probe(data)

    # -- Event handlers --------------------------------------------------------

    def _handle_wifi_probe(self, data: dict) -> None:
        """Process a WiFi probe request event."""
        links = self._correlator.ingest_probe(data)
        if links:
            for link in links:
                if link.score >= 0.5:
                    self._logger.debug(
                        "Strong WiFi-BLE correlation: %s <-> %s (score=%.2f)",
                        link.wifi_mac, link.ble_mac, link.score,
                    )
                    # Publish enrichment event for dossier integration
                    if self._event_bus:
                        self._event_bus.publish({
                            "type": "wifi_fingerprint.correlation",
                            "data": {
                                "wifi_mac": link.wifi_mac,
                                "ble_mac": link.ble_mac,
                                "score": link.score,
                                "ssids": sorted(link.ssids_seen),
                            },
                        })

    def _handle_ble_sighting(self, data: dict) -> None:
        """Process a BLE sighting event for correlation."""
        links = self._correlator.ingest_ble_sighting(data)
        if links:
            for link in links:
                if link.score >= 0.5:
                    self._logger.debug(
                        "BLE-WiFi correlation update: %s <-> %s (score=%.2f)",
                        link.ble_mac, link.wifi_mac, link.score,
                    )

    def _prune_loop(self) -> None:
        """Background loop to prune stale records periodically."""
        while self._running:
            time.sleep(PRUNE_INTERVAL)
            if not self._running:
                break
            try:
                pruned = self._correlator.prune_stale()
                if pruned > 0:
                    self._logger.debug("Pruned %d stale WiFi fingerprint records", pruned)
            except Exception as exc:
                self._logger.error("WiFi fingerprint prune error: %s", exc)

    # -- Public API ------------------------------------------------------------

    @property
    def correlator(self) -> ProbeCorrelator:
        """Access the probe correlator for external enrichment."""
        return self._correlator
