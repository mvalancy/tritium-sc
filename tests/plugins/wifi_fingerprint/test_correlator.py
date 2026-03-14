# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for WiFi Fingerprint probe correlator."""
import time

import pytest


@pytest.fixture
def correlator():
    from plugins.wifi_fingerprint.correlator import ProbeCorrelator
    return ProbeCorrelator()


class TestProbeCorrelator:
    """Unit tests for ProbeCorrelator."""

    def test_ingest_probe_records_fingerprint(self, correlator):
        """Ingesting a probe records the SSID in the fingerprint."""
        now = time.monotonic()
        correlator.ingest_probe({
            "mac": "AA:BB:CC:DD:EE:FF",
            "ssid": "HomeNet-5G",
            "rssi": -50,
            "observer_id": "node-01",
            "timestamp": now,
        })
        fp = correlator.get_fingerprint("aa:bb:cc:dd:ee:ff")
        assert "HomeNet-5G" in fp["probed_ssids"]
        assert fp["probe_count"] == 1

    def test_ingest_ble_no_crash_empty(self, correlator):
        """Ingesting BLE sighting without prior probes should not crash."""
        links = correlator.ingest_ble_sighting({
            "mac": "11:22:33:44:55:66",
            "name": "Phone",
            "rssi": -60,
            "observer_id": "node-01",
            "timestamp": time.monotonic(),
        })
        assert isinstance(links, list)

    def test_temporal_correlation(self, correlator):
        """WiFi probe and BLE sighting at same time/place correlate."""
        now = time.monotonic()
        obs = "node-01"

        # Probe first
        correlator.ingest_probe({
            "mac": "AA:BB:CC:DD:EE:FF",
            "ssid": "CorpNet",
            "rssi": -45,
            "observer_id": obs,
            "timestamp": now,
        })

        # BLE sighting shortly after
        links = correlator.ingest_ble_sighting({
            "mac": "11:22:33:44:55:66",
            "name": "iPhone",
            "rssi": -48,
            "observer_id": obs,
            "timestamp": now + 1.0,
        })

        # Should produce a correlation link
        assert len(links) > 0
        assert links[0].wifi_mac == "aa:bb:cc:dd:ee:ff"
        assert links[0].ble_mac == "11:22:33:44:55:66"
        assert links[0].score > 0

    def test_no_correlation_different_observers(self, correlator):
        """Probes and BLE on different observers should not correlate."""
        now = time.monotonic()

        correlator.ingest_probe({
            "mac": "AA:BB:CC:DD:EE:FF",
            "ssid": "HomeNet",
            "rssi": -50,
            "observer_id": "node-01",
            "timestamp": now,
        })

        links = correlator.ingest_ble_sighting({
            "mac": "11:22:33:44:55:66",
            "name": "Phone",
            "rssi": -55,
            "observer_id": "node-02",  # Different observer
            "timestamp": now + 1.0,
        })

        assert len(links) == 0

    def test_score_accumulates(self, correlator):
        """Repeated co-occurrences increase the correlation score."""
        obs = "node-01"

        for i in range(20):
            now = time.monotonic()
            correlator.ingest_probe({
                "mac": "AA:BB:CC:DD:EE:FF",
                "ssid": "Net",
                "rssi": -50,
                "observer_id": obs,
                "timestamp": now,
            })
            correlator.ingest_ble_sighting({
                "mac": "11:22:33:44:55:66",
                "name": "Phone",
                "rssi": -52,
                "observer_id": obs,
                "timestamp": now + 0.5,
            })

        links = correlator.get_links_for_ble("11:22:33:44:55:66")
        assert len(links) > 0
        assert links[0].score > 0.1

    def test_get_dossier_enrichment(self, correlator):
        """Dossier enrichment returns correlated WiFi profiles."""
        now = time.monotonic()
        obs = "node-01"

        # Create a strong correlation
        for i in range(10):
            correlator.ingest_probe({
                "mac": "AA:BB:CC:DD:EE:FF",
                "ssid": "CorpNet" if i % 2 == 0 else "HomeNet",
                "rssi": -45,
                "observer_id": obs,
                "timestamp": now + i,
            })
            correlator.ingest_ble_sighting({
                "mac": "11:22:33:44:55:66",
                "name": "Laptop",
                "rssi": -48,
                "observer_id": obs,
                "timestamp": now + i + 0.5,
            })

        enrichment = correlator.get_dossier_enrichment("11:22:33:44:55:66")
        assert "wifi_correlations" in enrichment
        assert len(enrichment["wifi_correlations"]) > 0
        assert len(enrichment["all_probed_ssids"]) > 0

    def test_status(self, correlator):
        """Status returns expected fields."""
        status = correlator.get_status()
        assert "total_probes_ingested" in status
        assert "unique_wifi_devices" in status
        assert "total_correlation_links" in status

    def test_prune_stale(self, correlator):
        """Pruning removes old records."""
        # This is just a functional test - records would need to be old
        pruned = correlator.prune_stale(max_age=0.001)
        assert isinstance(pruned, int)

    def test_get_all_links(self, correlator):
        """get_all_links returns list of dicts."""
        links = correlator.get_all_links()
        assert isinstance(links, list)

    def test_empty_mac_rejected(self, correlator):
        """Empty MAC addresses are rejected."""
        links = correlator.ingest_probe({"mac": "", "ssid": "Net"})
        assert links == []
        links = correlator.ingest_ble_sighting({"mac": ""})
        assert links == []
