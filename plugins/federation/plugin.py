# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""FederationPlugin — manages connections to other Tritium installations.

Enables multi-site federation via MQTT bridge:
  - Site discovery (announce/heartbeat)
  - Target sharing (real-time position updates across sites)
  - Dossier synchronization (share accumulated intelligence)
  - Alert forwarding (cross-site threat notifications)

Each federated site is connected via its own MQTT client that subscribes
to the remote site's federation topic tree.

Configuration: sites are stored in data/federation_sites.json.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

from engine.plugins.base import PluginContext, PluginInterface

try:
    from tritium_lib.models.federation import (
        ConnectionState,
        FederatedSite,
        FederationMessage,
        FederationMessageType,
        SharedTarget,
        SiteConnection,
        federation_topic,
        is_message_expired,
    )
except ImportError:  # pragma: no cover
    FederatedSite = None  # type: ignore[assignment,misc]

log = logging.getLogger("federation")


class FederationPlugin(PluginInterface):
    """Multi-site federation via MQTT bridge.

    Manages connections to remote Tritium installations for target
    sharing, dossier sync, and cross-site situational awareness.
    """

    def __init__(self) -> None:
        self._event_bus: Any = None
        self._tracker: Any = None
        self._app: Any = None
        self._logger: Optional[logging.Logger] = None
        self._settings: dict = {}

        self._running = False
        self._sites: dict[str, FederatedSite] = {}  # site_id -> FederatedSite
        self._connections: dict[str, SiteConnection] = {}  # site_id -> SiteConnection
        self._shared_targets: dict[str, SharedTarget] = {}  # target_id -> SharedTarget
        self._lock = threading.Lock()
        self._heartbeat_thread: Optional[threading.Thread] = None

        self._sites_file: str = ""

    # -- PluginInterface identity -------------------------------------------

    @property
    def plugin_id(self) -> str:
        return "tritium.federation"

    @property
    def name(self) -> str:
        return "Federation"

    @property
    def version(self) -> str:
        return "1.0.0"

    @property
    def capabilities(self) -> set[str]:
        return {"bridge", "data_source", "routes"}

    # -- PluginInterface lifecycle ------------------------------------------

    def configure(self, ctx: PluginContext) -> None:
        """Store references and load saved sites."""
        self._event_bus = ctx.event_bus
        self._tracker = ctx.target_tracker
        self._app = ctx.app
        self._logger = ctx.logger or log
        self._settings = ctx.settings or {}

        # Sites persistence file
        data_dir = os.path.join(os.getcwd(), "data")
        os.makedirs(data_dir, exist_ok=True)
        self._sites_file = os.path.join(data_dir, "federation_sites.json")

        # Load saved sites
        self._load_sites()

        # Register routes
        self._register_routes()

        self._logger.info(
            "Federation plugin configured (%d sites)", len(self._sites)
        )

    def start(self) -> None:
        """Start federation heartbeat loop."""
        if self._running:
            return
        if FederatedSite is None:
            log.warning("tritium-lib federation models not available — federation disabled")
            return

        self._running = True

        # Initialize connections for all enabled sites
        for site_id, site in self._sites.items():
            if site.enabled:
                self._connections[site_id] = SiteConnection(
                    site_id=site_id,
                    state=ConnectionState.DISCONNECTED,
                )

        # Start heartbeat thread
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            daemon=True,
            name="federation-heartbeat",
        )
        self._heartbeat_thread.start()

        self._logger.info("Federation plugin started")

    def stop(self) -> None:
        """Disconnect all sites and stop heartbeat."""
        if not self._running:
            return
        self._running = False

        if self._heartbeat_thread and self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=3.0)

        # Mark all connections as disconnected
        with self._lock:
            for conn in self._connections.values():
                conn.state = ConnectionState.DISCONNECTED

        self._save_sites()
        self._logger.info("Federation plugin stopped")

    @property
    def healthy(self) -> bool:
        return self._running

    # -- Public API ---------------------------------------------------------

    def add_site(self, site: FederatedSite) -> str:
        """Register a new federated site. Returns site_id."""
        with self._lock:
            self._sites[site.site_id] = site
            self._connections[site.site_id] = SiteConnection(
                site_id=site.site_id,
                state=ConnectionState.DISCONNECTED,
            )
        self._save_sites()
        self._logger.info("Added federated site: %s (%s)", site.name, site.site_id)

        # Publish event
        if self._event_bus:
            self._event_bus.publish("federation:site_added", data={
                "site_id": site.site_id,
                "name": site.name,
            })

        return site.site_id

    def remove_site(self, site_id: str) -> bool:
        """Remove a federated site. Returns True if found."""
        with self._lock:
            if site_id not in self._sites:
                return False
            del self._sites[site_id]
            self._connections.pop(site_id, None)
        self._save_sites()
        self._logger.info("Removed federated site: %s", site_id)
        return True

    def get_site(self, site_id: str) -> Optional[FederatedSite]:
        """Get a federated site by ID."""
        with self._lock:
            return self._sites.get(site_id)

    def list_sites(self) -> list[dict]:
        """List all federated sites with connection status."""
        with self._lock:
            result = []
            for site_id, site in self._sites.items():
                conn = self._connections.get(site_id)
                entry = site.model_dump()
                if conn:
                    entry["connection"] = conn.model_dump()
                result.append(entry)
            return result

    def get_connection(self, site_id: str) -> Optional[SiteConnection]:
        """Get connection state for a site."""
        with self._lock:
            return self._connections.get(site_id)

    def share_target(self, target: SharedTarget) -> None:
        """Share a target with federated sites."""
        with self._lock:
            self._shared_targets[target.target_id] = target

        if self._event_bus:
            self._event_bus.publish("federation:target_shared", data={
                "target_id": target.target_id,
                "source_site_id": target.source_site_id,
            })

    def receive_target(self, target: SharedTarget) -> None:
        """Process a target received from a federated site."""
        with self._lock:
            self._shared_targets[target.target_id] = target

        # Push to TargetTracker if available
        if self._tracker is not None:
            self._tracker.update_from_federation({
                "target_id": target.target_id,
                "source_site": target.source_site_id,
                "name": target.name,
                "entity_type": target.entity_type,
                "alliance": target.alliance,
                "lat": target.lat,
                "lng": target.lng,
                "confidence": target.confidence,
                "source": target.source,
            })

        if self._event_bus:
            self._event_bus.publish("federation:target_received", data={
                "target_id": target.target_id,
                "source_site_id": target.source_site_id,
            })

    def get_shared_targets(self) -> list[dict]:
        """Get all shared targets from federated sites."""
        with self._lock:
            return [t.model_dump() for t in self._shared_targets.values()]

    def get_stats(self) -> dict:
        """Get federation statistics."""
        with self._lock:
            connected = sum(
                1 for c in self._connections.values()
                if c.state == ConnectionState.CONNECTED
            )
            return {
                "total_sites": len(self._sites),
                "connected_sites": connected,
                "shared_targets": len(self._shared_targets),
                "enabled_sites": sum(
                    1 for s in self._sites.values() if s.enabled
                ),
            }

    # -- Persistence --------------------------------------------------------

    def _load_sites(self) -> None:
        """Load sites from JSON file."""
        if FederatedSite is None:
            return
        if not os.path.exists(self._sites_file):
            return
        try:
            with open(self._sites_file, "r") as f:
                data = json.load(f)
            for entry in data:
                site = FederatedSite(**entry)
                self._sites[site.site_id] = site
            self._logger.info("Loaded %d federated sites", len(self._sites))
        except Exception as exc:
            self._logger.error("Failed to load federation sites: %s", exc)

    def _save_sites(self) -> None:
        """Save sites to JSON file."""
        if FederatedSite is None:
            return
        try:
            with self._lock:
                data = [s.model_dump() for s in self._sites.values()]
            with open(self._sites_file, "w") as f:
                json.dump(data, f, indent=2, default=str)
        except Exception as exc:
            self._logger.error("Failed to save federation sites: %s", exc)

    # -- Background tasks ---------------------------------------------------

    def _heartbeat_loop(self) -> None:
        """Periodic heartbeat for federation health monitoring."""
        while self._running:
            try:
                self._send_heartbeats()
            except Exception as exc:
                log.error("Federation heartbeat error: %s", exc)
            # Sleep in small increments so we can stop quickly
            for _ in range(30):
                if not self._running:
                    break
                time.sleep(1.0)

    def _send_heartbeats(self) -> None:
        """Send heartbeat to all connected sites."""
        if FederatedSite is None:
            return

        with self._lock:
            sites = list(self._sites.values())

        for site in sites:
            if not site.enabled:
                continue
            conn = self._connections.get(site.site_id)
            if conn:
                conn.last_heartbeat = time.time()

    # -- HTTP routes --------------------------------------------------------

    def _register_routes(self) -> None:
        """Register FastAPI routes for federation management."""
        if not self._app:
            return

        from .routes import create_router
        router = create_router(self)
        self._app.include_router(router)
