# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""WiFi probe request to BLE device correlator.

When a device sends WiFi probe requests, it reveals its MAC and the SSIDs it
knows. If a BLE device is seen nearby (same observer, similar time), the probe
becomes a signal in that device's dossier.

Correlation logic:
  1. Temporal proximity — probe and BLE sighting within TEMPORAL_WINDOW seconds
  2. Spatial proximity — same observer node or nearby observer
  3. MAC affinity — some devices share OUI prefix between WiFi and BLE MACs
  4. Behavioral clustering — devices that always appear/disappear together

Each correlation is scored 0.0-1.0 and accumulated over time. High-confidence
correlations fuse the WiFi fingerprint into the BLE target's identity.
"""
from __future__ import annotations

import hashlib
import logging
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Any, Optional

logger = logging.getLogger("wifi-fingerprint")

# Time window for temporal correlation (seconds)
TEMPORAL_WINDOW = 10.0

# Minimum correlation score to consider a link
MIN_CORRELATION_SCORE = 0.3

# Maximum probe history per MAC
MAX_PROBE_HISTORY = 500

# Maximum BLE sighting history per MAC
MAX_BLE_HISTORY = 500

# Decay factor for old correlations (per hour)
CORRELATION_DECAY = 0.95


@dataclass(slots=True)
class ProbeRecord:
    """A WiFi probe request observation."""
    wifi_mac: str
    ssid: str
    rssi: int
    observer_id: str
    timestamp: float
    channel: int = 0


@dataclass(slots=True)
class BleSightingRecord:
    """A BLE sighting observation for correlation purposes."""
    ble_mac: str
    name: str
    rssi: int
    observer_id: str
    timestamp: float
    device_type: str = ""


@dataclass
class CorrelationLink:
    """A correlation between a WiFi MAC and a BLE MAC."""
    wifi_mac: str
    ble_mac: str
    score: float = 0.0
    evidence_count: int = 0
    last_updated: float = field(default_factory=time.monotonic)
    ssids_seen: set[str] = field(default_factory=set)
    shared_observers: set[str] = field(default_factory=set)

    def to_dict(self) -> dict[str, Any]:
        return {
            "wifi_mac": self.wifi_mac,
            "ble_mac": self.ble_mac,
            "score": round(self.score, 3),
            "evidence_count": self.evidence_count,
            "ssids_seen": sorted(self.ssids_seen),
            "shared_observers": sorted(self.shared_observers),
        }


class ProbeCorrelator:
    """Correlates WiFi probe requests with BLE sightings.

    Feed probe requests and BLE sightings. The correlator detects when a WiFi
    device and BLE device are consistently co-located and builds correlation
    links between them.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()

        # Recent probe requests keyed by observer_id for fast temporal lookup
        self._probes_by_observer: dict[str, list[ProbeRecord]] = defaultdict(list)
        # Recent BLE sightings keyed by observer_id
        self._ble_by_observer: dict[str, list[BleSightingRecord]] = defaultdict(list)

        # WiFi fingerprints: wifi_mac -> set of probed SSIDs
        self._wifi_fingerprints: dict[str, set[str]] = defaultdict(set)

        # Correlation links: (wifi_mac, ble_mac) -> CorrelationLink
        self._links: dict[tuple[str, str], CorrelationLink] = {}

        # Probe count per wifi_mac
        self._probe_counts: dict[str, int] = defaultdict(int)

    def ingest_probe(self, probe: dict[str, Any]) -> list[CorrelationLink]:
        """Ingest a WiFi probe request and correlate with nearby BLE devices.

        Args:
            probe: Dict with keys: mac, ssid, rssi, observer_id, timestamp, channel

        Returns:
            List of new or strengthened correlation links.
        """
        wifi_mac = probe.get("mac", "").lower()
        ssid = probe.get("ssid", probe.get("ssid_probed", ""))
        rssi = probe.get("rssi", -100)
        observer_id = probe.get("observer_id", "")
        timestamp = probe.get("timestamp", time.monotonic())
        channel = probe.get("channel", 0)

        if not wifi_mac:
            return []

        record = ProbeRecord(
            wifi_mac=wifi_mac,
            ssid=ssid,
            rssi=rssi,
            observer_id=observer_id,
            timestamp=timestamp,
            channel=channel,
        )

        updated_links: list[CorrelationLink] = []

        with self._lock:
            # Store fingerprint
            if ssid:
                self._wifi_fingerprints[wifi_mac].add(ssid)
            self._probe_counts[wifi_mac] = self._probe_counts.get(wifi_mac, 0) + 1

            # Store probe record
            obs_probes = self._probes_by_observer[observer_id]
            obs_probes.append(record)
            # Trim old probes
            if len(obs_probes) > MAX_PROBE_HISTORY:
                self._probes_by_observer[observer_id] = obs_probes[-MAX_PROBE_HISTORY:]

            # Find temporally close BLE sightings on the same observer
            obs_ble = self._ble_by_observer.get(observer_id, [])
            for ble in obs_ble:
                time_diff = abs(timestamp - ble.timestamp)
                if time_diff > TEMPORAL_WINDOW:
                    continue

                # Compute correlation score for this co-occurrence
                score = self._compute_co_occurrence_score(
                    time_diff=time_diff,
                    wifi_rssi=rssi,
                    ble_rssi=ble.rssi,
                    wifi_mac=wifi_mac,
                    ble_mac=ble.ble_mac,
                )

                link = self._update_link(
                    wifi_mac=wifi_mac,
                    ble_mac=ble.ble_mac,
                    score_delta=score,
                    observer_id=observer_id,
                    ssid=ssid,
                )
                updated_links.append(link)

        return updated_links

    def ingest_ble_sighting(self, sighting: dict[str, Any]) -> list[CorrelationLink]:
        """Ingest a BLE sighting and correlate with recent WiFi probes.

        Args:
            sighting: Dict with keys: mac, name, rssi, observer_id, timestamp, device_type

        Returns:
            List of new or strengthened correlation links.
        """
        ble_mac = sighting.get("mac", "").lower()
        name = sighting.get("name", "")
        rssi = sighting.get("rssi", -100)
        observer_id = sighting.get("observer_id", sighting.get("node_id", ""))
        timestamp = sighting.get("timestamp", time.monotonic())
        device_type = sighting.get("device_type", "")

        if not ble_mac:
            return []

        record = BleSightingRecord(
            ble_mac=ble_mac,
            name=name,
            rssi=rssi,
            observer_id=observer_id,
            timestamp=timestamp,
            device_type=device_type,
        )

        updated_links: list[CorrelationLink] = []

        with self._lock:
            # Store BLE record
            obs_ble = self._ble_by_observer[observer_id]
            obs_ble.append(record)
            if len(obs_ble) > MAX_BLE_HISTORY:
                self._ble_by_observer[observer_id] = obs_ble[-MAX_BLE_HISTORY:]

            # Find temporally close WiFi probes on the same observer
            obs_probes = self._probes_by_observer.get(observer_id, [])
            for probe_rec in obs_probes:
                time_diff = abs(timestamp - probe_rec.timestamp)
                if time_diff > TEMPORAL_WINDOW:
                    continue

                score = self._compute_co_occurrence_score(
                    time_diff=time_diff,
                    wifi_rssi=probe_rec.rssi,
                    ble_rssi=rssi,
                    wifi_mac=probe_rec.wifi_mac,
                    ble_mac=ble_mac,
                )

                link = self._update_link(
                    wifi_mac=probe_rec.wifi_mac,
                    ble_mac=ble_mac,
                    score_delta=score,
                    observer_id=observer_id,
                    ssid=probe_rec.ssid,
                )
                updated_links.append(link)

        return updated_links

    def _compute_co_occurrence_score(
        self,
        time_diff: float,
        wifi_rssi: int,
        ble_rssi: int,
        wifi_mac: str,
        ble_mac: str,
    ) -> float:
        """Compute correlation score for a single co-occurrence.

        Factors:
          - Temporal proximity (closer = higher score)
          - RSSI similarity (similar signal strength = likely same device)
          - MAC OUI affinity (shared manufacturer prefix = bonus)
        """
        # Temporal factor: 1.0 at t=0, decays to ~0.3 at TEMPORAL_WINDOW
        temporal = max(0.0, 1.0 - (time_diff / TEMPORAL_WINDOW) * 0.7)

        # RSSI proximity factor: if both have similar RSSI, more likely same device
        rssi_diff = abs(wifi_rssi - ble_rssi)
        rssi_factor = max(0.3, 1.0 - (rssi_diff / 40.0))

        # MAC OUI affinity: if first 3 bytes match, bonus
        wifi_clean = wifi_mac.replace(":", "").replace("-", "")
        ble_clean = ble_mac.replace(":", "").replace("-", "")
        oui_bonus = 0.2 if len(wifi_clean) >= 6 and len(ble_clean) >= 6 and wifi_clean[:6] == ble_clean[:6] else 0.0

        # Base score is small per co-occurrence, accumulates over time
        base_score = 0.05

        return base_score * temporal * rssi_factor + oui_bonus

    def _update_link(
        self,
        wifi_mac: str,
        ble_mac: str,
        score_delta: float,
        observer_id: str,
        ssid: str,
    ) -> CorrelationLink:
        """Update or create a correlation link."""
        key = (wifi_mac, ble_mac)
        if key not in self._links:
            self._links[key] = CorrelationLink(
                wifi_mac=wifi_mac,
                ble_mac=ble_mac,
            )

        link = self._links[key]
        link.score = min(1.0, link.score + score_delta)
        link.evidence_count += 1
        link.last_updated = time.monotonic()
        link.shared_observers.add(observer_id)
        if ssid:
            link.ssids_seen.add(ssid)

        return link

    def get_links_for_ble(self, ble_mac: str) -> list[CorrelationLink]:
        """Get all WiFi correlation links for a BLE device."""
        ble_mac = ble_mac.lower()
        with self._lock:
            return [
                link for link in self._links.values()
                if link.ble_mac == ble_mac and link.score >= MIN_CORRELATION_SCORE
            ]

    def get_links_for_wifi(self, wifi_mac: str) -> list[CorrelationLink]:
        """Get all BLE correlation links for a WiFi device."""
        wifi_mac = wifi_mac.lower()
        with self._lock:
            return [
                link for link in self._links.values()
                if link.wifi_mac == wifi_mac and link.score >= MIN_CORRELATION_SCORE
            ]

    def get_fingerprint(self, wifi_mac: str) -> dict[str, Any]:
        """Get the WiFi fingerprint for a device."""
        wifi_mac = wifi_mac.lower()
        with self._lock:
            ssids = sorted(self._wifi_fingerprints.get(wifi_mac, set()))
            count = self._probe_counts.get(wifi_mac, 0)
        return {
            "mac": wifi_mac,
            "probed_ssids": ssids,
            "probe_count": count,
        }

    def get_all_links(self, min_score: float = MIN_CORRELATION_SCORE) -> list[dict[str, Any]]:
        """Get all correlation links above threshold."""
        with self._lock:
            return [
                link.to_dict()
                for link in self._links.values()
                if link.score >= min_score
            ]

    def get_dossier_enrichment(self, ble_mac: str) -> dict[str, Any]:
        """Get WiFi probe enrichment for a BLE device's dossier.

        Returns the correlated WiFi MACs, their probed SSIDs, and the
        correlation confidence.
        """
        links = self.get_links_for_ble(ble_mac)
        if not links:
            return {}

        wifi_profiles: list[dict[str, Any]] = []
        all_ssids: set[str] = set()

        for link in sorted(links, key=lambda l: l.score, reverse=True):
            fp = self.get_fingerprint(link.wifi_mac)
            all_ssids.update(link.ssids_seen)
            wifi_profiles.append({
                "wifi_mac": link.wifi_mac,
                "score": round(link.score, 3),
                "evidence_count": link.evidence_count,
                "probed_ssids": fp["probed_ssids"],
            })

        return {
            "wifi_correlations": wifi_profiles,
            "all_probed_ssids": sorted(all_ssids),
            "strongest_score": round(links[0].score, 3) if links else 0.0,
        }

    def get_status(self) -> dict[str, Any]:
        """Return correlator status for API."""
        with self._lock:
            total_probes = sum(self._probe_counts.values())
            wifi_devices = len(self._wifi_fingerprints)
            total_links = len(self._links)
            strong_links = sum(1 for l in self._links.values() if l.score >= 0.5)

        return {
            "total_probes_ingested": total_probes,
            "unique_wifi_devices": wifi_devices,
            "total_correlation_links": total_links,
            "strong_links": strong_links,
            "observer_count": len(self._probes_by_observer),
        }

    def prune_stale(self, max_age: float = 3600.0) -> int:
        """Prune old probe/sighting records and decay link scores.

        Args:
            max_age: Maximum age in seconds for records.

        Returns:
            Number of records pruned.
        """
        now = time.monotonic()
        cutoff = now - max_age
        pruned = 0

        with self._lock:
            # Prune old probe records
            for observer_id in list(self._probes_by_observer.keys()):
                old_len = len(self._probes_by_observer[observer_id])
                self._probes_by_observer[observer_id] = [
                    p for p in self._probes_by_observer[observer_id]
                    if p.timestamp > cutoff
                ]
                pruned += old_len - len(self._probes_by_observer[observer_id])
                if not self._probes_by_observer[observer_id]:
                    del self._probes_by_observer[observer_id]

            # Prune old BLE records
            for observer_id in list(self._ble_by_observer.keys()):
                old_len = len(self._ble_by_observer[observer_id])
                self._ble_by_observer[observer_id] = [
                    b for b in self._ble_by_observer[observer_id]
                    if b.timestamp > cutoff
                ]
                pruned += old_len - len(self._ble_by_observer[observer_id])
                if not self._ble_by_observer[observer_id]:
                    del self._ble_by_observer[observer_id]

            # Decay and prune weak links
            stale_links = []
            for key, link in self._links.items():
                hours_old = (now - link.last_updated) / 3600.0
                if hours_old > 1.0:
                    link.score *= CORRELATION_DECAY ** hours_old
                    if link.score < 0.01:
                        stale_links.append(key)
            for key in stale_links:
                del self._links[key]
                pruned += 1

        return pruned
