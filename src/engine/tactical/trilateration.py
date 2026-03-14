# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""BLE trilateration engine — estimate device positions from multi-node RSSI.

Maintains a sliding window of recent BLE sightings from multiple observer
nodes.  When 3+ nodes report the same MAC within a configurable window,
the engine estimates position via weighted centroid trilateration using
tritium-lib's rssi_to_distance() and trilaterate_2d().
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

from tritium_lib.models.trilateration import (
    AnchorPoint,
    RSSIFilter,
    _compute_confidence,
    rssi_to_distance,
    trilaterate_2d,
)

log = logging.getLogger("trilateration-engine")

# Defaults
DEFAULT_STALE_THRESHOLD = 60.0   # seconds — prune sightings older than this
DEFAULT_WINDOW = 30.0            # seconds — max age for trilateration input
DEFAULT_MIN_ANCHORS = 3          # minimum observer nodes required


@dataclass
class Sighting:
    """A single BLE RSSI observation from one node."""
    node_id: str
    lat: float
    lon: float
    rssi: float
    timestamp: float = field(default_factory=time.time)


@dataclass
class PositionResult:
    """Estimated position with metadata."""
    lat: float
    lon: float
    confidence: float
    anchors_used: int
    method: str = "weighted_centroid"

    def to_dict(self) -> dict:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "confidence": self.confidence,
            "anchors_used": self.anchors_used,
            "method": self.method,
        }


class TrilaterationEngine:
    """Accumulates BLE sightings from multiple observer nodes and estimates
    device positions via trilateration when sufficient data is available.

    Thread-safe: all mutations are guarded by a lock.
    """

    def __init__(
        self,
        *,
        stale_threshold: float = DEFAULT_STALE_THRESHOLD,
        window: float = DEFAULT_WINDOW,
        min_anchors: int = DEFAULT_MIN_ANCHORS,
        tx_power: float = -59.0,
        path_loss_exponent: float = 2.5,
    ) -> None:
        self._stale_threshold = stale_threshold
        self._window = window
        self._min_anchors = min_anchors
        self._tx_power = tx_power
        self._path_loss_exponent = path_loss_exponent

        # mac -> list[Sighting]  (most recent per node kept)
        self._sightings: dict[str, list[Sighting]] = {}
        # mac -> node_id -> RSSIFilter  (Kalman smoothing per link)
        self._filters: dict[str, dict[str, RSSIFilter]] = {}
        self._lock = threading.Lock()

    # -- Public API -----------------------------------------------------------

    def record_sighting(
        self,
        mac: str,
        node_id: str,
        node_lat: float,
        node_lon: float,
        rssi: float,
    ) -> None:
        """Record a BLE sighting from an observer node.

        If the same node_id already has a sighting for this MAC, the older
        entry is replaced (we keep only the freshest per node).
        """
        mac = mac.upper()
        now = time.time()

        with self._lock:
            # Kalman-filter the RSSI
            if mac not in self._filters:
                self._filters[mac] = {}
            if node_id not in self._filters[mac]:
                self._filters[mac][node_id] = RSSIFilter(initial_estimate=rssi)
            smoothed_rssi = self._filters[mac][node_id].update(rssi)

            sighting = Sighting(
                node_id=node_id,
                lat=node_lat,
                lon=node_lon,
                rssi=smoothed_rssi,
                timestamp=now,
            )

            if mac not in self._sightings:
                self._sightings[mac] = []

            # Replace existing sighting from same node
            self._sightings[mac] = [
                s for s in self._sightings[mac] if s.node_id != node_id
            ]
            self._sightings[mac].append(sighting)

    def estimate_position(self, mac: str) -> Optional[PositionResult]:
        """Estimate position of a BLE device from accumulated sightings.

        Returns None if fewer than min_anchors nodes have reported the MAC
        within the sliding window.
        """
        mac = mac.upper()
        now = time.time()
        cutoff = now - self._window

        with self._lock:
            all_sightings = self._sightings.get(mac, [])
            # Filter to recent sightings within window
            recent = [s for s in all_sightings if s.timestamp >= cutoff]

        if len(recent) < self._min_anchors:
            return None

        # Build anchor points
        anchors: list[AnchorPoint] = []
        anchor_tuples: list[tuple[float, float, float]] = []

        for s in recent:
            distance = rssi_to_distance(
                s.rssi, self._tx_power, self._path_loss_exponent
            )
            anchors.append(AnchorPoint(
                node_id=s.node_id,
                lat=s.lat,
                lon=s.lon,
                rssi=s.rssi,
                distance=distance,
            ))
            anchor_tuples.append((s.lat, s.lon, distance))

        result = trilaterate_2d(anchor_tuples)
        if result is None:
            return None

        distances = [a.distance for a in anchors]
        confidence = _compute_confidence(anchors, distances)

        return PositionResult(
            lat=round(result[0], 8),
            lon=round(result[1], 8),
            confidence=confidence,
            anchors_used=len(anchors),
        )

    def prune_stale(self) -> int:
        """Remove sightings older than the stale threshold.

        Returns the number of MACs that were fully removed (no sightings left).
        """
        now = time.time()
        cutoff = now - self._stale_threshold
        removed_macs = 0

        with self._lock:
            stale_keys = []
            for mac, sightings in self._sightings.items():
                self._sightings[mac] = [
                    s for s in sightings if s.timestamp >= cutoff
                ]
                if not self._sightings[mac]:
                    stale_keys.append(mac)

            for mac in stale_keys:
                del self._sightings[mac]
                self._filters.pop(mac, None)
                removed_macs += 1

        return removed_macs

    def get_all_estimates(self) -> dict[str, PositionResult]:
        """Return position estimates for all MACs that have enough anchors."""
        results: dict[str, PositionResult] = {}
        with self._lock:
            macs = list(self._sightings.keys())

        for mac in macs:
            est = self.estimate_position(mac)
            if est is not None:
                results[mac] = est

        return results

    @property
    def tracked_macs(self) -> int:
        """Number of MACs currently being tracked."""
        with self._lock:
            return len(self._sightings)

    def get_sighting_count(self, mac: str) -> int:
        """Number of recent sightings for a MAC."""
        mac = mac.upper()
        with self._lock:
            return len(self._sightings.get(mac, []))

    def clear(self) -> None:
        """Clear all sightings and filters."""
        with self._lock:
            self._sightings.clear()
            self._filters.clear()
