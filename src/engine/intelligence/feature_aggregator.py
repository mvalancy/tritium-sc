# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Feature Aggregation Service — collects BLE feature vectors from edge nodes.

Receives feature vectors from edge MQTT sightings, aggregates per-device,
and feeds accumulated features to the BLE classification learner.

The aggregator maintains a rolling window of feature vectors per MAC,
computes mean features, and tracks which edge nodes have contributed.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

logger = logging.getLogger("feature_aggregator")

# Maximum devices to track features for
MAX_TRACKED_DEVICES = 512

# Maximum feature vectors per device (rolling window)
MAX_VECTORS_PER_DEVICE = 50

# Feature vector TTL in seconds (drop vectors older than this)
FEATURE_TTL_S = 3600  # 1 hour


class DeviceFeatureEntry:
    """Accumulated feature vectors for a single device."""

    __slots__ = (
        "mac", "vectors", "node_ids", "first_seen", "last_seen",
        "mean_features", "_dirty",
    )

    def __init__(self, mac: str) -> None:
        self.mac = mac
        self.vectors: list[dict[str, float]] = []
        self.node_ids: set[str] = set()
        self.first_seen: float = time.time()
        self.last_seen: float = time.time()
        self.mean_features: dict[str, float] = {}
        self._dirty = True

    def add_vector(
        self,
        features: dict[str, float],
        node_id: str,
    ) -> None:
        """Add a feature vector from an edge node."""
        self.vectors.append(features)
        self.node_ids.add(node_id)
        self.last_seen = time.time()
        self._dirty = True

        # Trim to rolling window
        if len(self.vectors) > MAX_VECTORS_PER_DEVICE:
            self.vectors = self.vectors[-MAX_VECTORS_PER_DEVICE:]

    def compute_mean(self) -> dict[str, float]:
        """Compute mean feature values across all vectors."""
        if not self._dirty and self.mean_features:
            return self.mean_features

        if not self.vectors:
            return {}

        sums: dict[str, float] = {}
        counts: dict[str, int] = {}

        for v in self.vectors:
            for k, val in v.items():
                sums[k] = sums.get(k, 0.0) + val
                counts[k] = counts.get(k, 0) + 1

        self.mean_features = {k: sums[k] / counts[k] for k in sums}
        self._dirty = False
        return self.mean_features

    def to_dict(self) -> dict[str, Any]:
        """Serialize to dict for API response."""
        mean = self.compute_mean()
        return {
            "mac": self.mac,
            "vector_count": len(self.vectors),
            "node_count": len(self.node_ids),
            "node_ids": sorted(self.node_ids),
            "first_seen": self.first_seen,
            "last_seen": self.last_seen,
            "mean_features": mean,
        }


class FeatureAggregator:
    """Aggregates BLE feature vectors from edge nodes.

    Thread-safe: can be called from MQTT callback threads and
    API handler coroutines concurrently.
    """

    def __init__(self) -> None:
        self._devices: dict[str, DeviceFeatureEntry] = {}
        self._lock = threading.Lock()
        self._total_vectors: int = 0
        self._feedback_sent: int = 0

    @property
    def device_count(self) -> int:
        with self._lock:
            return len(self._devices)

    @property
    def total_vectors(self) -> int:
        return self._total_vectors

    def ingest(
        self,
        mac: str,
        features: dict[str, float],
        node_id: str,
    ) -> None:
        """Ingest a feature vector from an edge node.

        Args:
            mac: Device MAC address.
            features: Feature name-to-value dict.
            node_id: Edge node identifier.
        """
        mac = mac.upper().strip()
        if not mac or not features:
            return

        with self._lock:
            entry = self._devices.get(mac)
            if entry is None:
                # Evict oldest if at capacity
                if len(self._devices) >= MAX_TRACKED_DEVICES:
                    self._evict_oldest()
                entry = DeviceFeatureEntry(mac)
                self._devices[mac] = entry

            entry.add_vector(features, node_id)
            self._total_vectors += 1

    def ingest_batch(
        self,
        items: list[dict[str, Any]],
        node_id: str,
    ) -> int:
        """Ingest a batch of feature vectors (from MQTT JSON array).

        Each item should have "mac" and "features" keys.

        Args:
            items: List of {"mac": "...", "features": {...}} dicts.
            node_id: Edge node identifier.

        Returns:
            Number of vectors ingested.
        """
        count = 0
        for item in items:
            mac = item.get("mac", "")
            features = item.get("features")
            if mac and isinstance(features, dict):
                self.ingest(mac, features, node_id)
                count += 1
        return count

    def get_features(self, mac: str) -> Optional[dict[str, Any]]:
        """Get accumulated features for a device.

        Args:
            mac: Device MAC address.

        Returns:
            Feature summary dict or None if not tracked.
        """
        mac = mac.upper().strip()
        with self._lock:
            entry = self._devices.get(mac)
            if entry is None:
                return None
            return entry.to_dict()

    def get_all_features(self, limit: int = 100) -> list[dict[str, Any]]:
        """Get features for all tracked devices.

        Args:
            limit: Maximum number of devices to return.

        Returns:
            List of feature summary dicts, ordered by last_seen descending.
        """
        with self._lock:
            entries = sorted(
                self._devices.values(),
                key=lambda e: e.last_seen,
                reverse=True,
            )[:limit]
            return [e.to_dict() for e in entries]

    def get_classification_candidates(
        self, min_vectors: int = 3
    ) -> list[DeviceFeatureEntry]:
        """Get devices with enough feature data for ML classification.

        Args:
            min_vectors: Minimum number of feature vectors required.

        Returns:
            List of DeviceFeatureEntry objects ready for classification.
        """
        with self._lock:
            return [
                e for e in self._devices.values()
                if len(e.vectors) >= min_vectors
            ]

    def get_stats(self) -> dict[str, Any]:
        """Get aggregator statistics."""
        with self._lock:
            if not self._devices:
                return {
                    "device_count": 0,
                    "total_vectors": self._total_vectors,
                    "feedback_sent": self._feedback_sent,
                }

            node_ids: set[str] = set()
            for e in self._devices.values():
                node_ids.update(e.node_ids)

            return {
                "device_count": len(self._devices),
                "total_vectors": self._total_vectors,
                "unique_nodes": len(node_ids),
                "feedback_sent": self._feedback_sent,
                "avg_vectors_per_device": (
                    self._total_vectors / len(self._devices)
                    if self._devices else 0
                ),
            }

    def record_feedback_sent(self) -> None:
        """Increment feedback sent counter."""
        self._feedback_sent += 1

    def prune_stale(self, max_age_s: float = FEATURE_TTL_S) -> int:
        """Remove devices not seen recently.

        Args:
            max_age_s: Maximum age in seconds.

        Returns:
            Number of devices pruned.
        """
        cutoff = time.time() - max_age_s
        with self._lock:
            stale = [
                mac for mac, e in self._devices.items()
                if e.last_seen < cutoff
            ]
            for mac in stale:
                del self._devices[mac]
            return len(stale)

    def _evict_oldest(self) -> None:
        """Evict the oldest-seen device to make room. Must hold _lock."""
        if not self._devices:
            return
        oldest_mac = min(self._devices, key=lambda m: self._devices[m].last_seen)
        del self._devices[oldest_mac]


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------

_aggregator: Optional[FeatureAggregator] = None


def get_feature_aggregator() -> FeatureAggregator:
    """Get or create the singleton FeatureAggregator."""
    global _aggregator
    if _aggregator is None:
        _aggregator = FeatureAggregator()
    return _aggregator
