# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the feature aggregation service."""

import pytest
import time

from engine.intelligence.feature_aggregator import (
    FeatureAggregator,
    DeviceFeatureEntry,
)


class TestDeviceFeatureEntry:
    """Tests for DeviceFeatureEntry."""

    def test_create(self):
        e = DeviceFeatureEntry("AA:BB:CC:DD:EE:FF")
        assert e.mac == "AA:BB:CC:DD:EE:FF"
        assert len(e.vectors) == 0
        assert len(e.node_ids) == 0

    def test_add_vector(self):
        e = DeviceFeatureEntry("XX")
        e.add_vector({"oui_hash": 0.5, "name_length": 8.0}, "node-01")
        assert len(e.vectors) == 1
        assert "node-01" in e.node_ids

    def test_add_multiple_nodes(self):
        e = DeviceFeatureEntry("XX")
        e.add_vector({"a": 1.0}, "node-01")
        e.add_vector({"a": 2.0}, "node-02")
        assert len(e.vectors) == 2
        assert len(e.node_ids) == 2

    def test_compute_mean(self):
        e = DeviceFeatureEntry("XX")
        e.add_vector({"a": 2.0, "b": 4.0}, "n1")
        e.add_vector({"a": 6.0, "b": 8.0}, "n2")
        mean = e.compute_mean()
        assert mean["a"] == pytest.approx(4.0)
        assert mean["b"] == pytest.approx(6.0)

    def test_to_dict(self):
        e = DeviceFeatureEntry("AA:BB")
        e.add_vector({"x": 1.0}, "n1")
        d = e.to_dict()
        assert d["mac"] == "AA:BB"
        assert d["vector_count"] == 1
        assert d["node_count"] == 1
        assert "mean_features" in d


class TestFeatureAggregator:
    """Tests for FeatureAggregator."""

    def test_ingest_single(self):
        agg = FeatureAggregator()
        agg.ingest("AA:BB:CC:DD:EE:FF", {"oui_hash": 0.5}, "node-01")
        assert agg.device_count == 1
        assert agg.total_vectors == 1

    def test_ingest_empty_rejected(self):
        agg = FeatureAggregator()
        agg.ingest("", {"a": 1.0}, "n1")
        agg.ingest("XX", {}, "n1")
        assert agg.device_count == 0

    def test_ingest_normalizes_mac(self):
        agg = FeatureAggregator()
        agg.ingest("aa:bb:cc:dd:ee:ff", {"a": 1.0}, "n1")
        result = agg.get_features("AA:BB:CC:DD:EE:FF")
        assert result is not None
        assert result["mac"] == "AA:BB:CC:DD:EE:FF"

    def test_ingest_batch(self):
        agg = FeatureAggregator()
        items = [
            {"mac": "AA:BB:CC:DD:EE:01", "features": {"a": 1.0}},
            {"mac": "AA:BB:CC:DD:EE:02", "features": {"a": 2.0}},
            {"mac": "AA:BB:CC:DD:EE:03", "features": {"a": 3.0}},
        ]
        count = agg.ingest_batch(items, "node-01")
        assert count == 3
        assert agg.device_count == 3

    def test_get_features_not_found(self):
        agg = FeatureAggregator()
        assert agg.get_features("XX:XX:XX:XX:XX:XX") is None

    def test_get_all_features(self):
        agg = FeatureAggregator()
        for i in range(5):
            agg.ingest(f"AA:BB:CC:DD:EE:{i:02X}", {"a": float(i)}, "n1")
        results = agg.get_all_features(limit=3)
        assert len(results) == 3

    def test_get_classification_candidates(self):
        agg = FeatureAggregator()
        for i in range(5):
            agg.ingest("AA:BB:CC:DD:EE:01", {"a": float(i)}, f"n{i}")
        agg.ingest("AA:BB:CC:DD:EE:02", {"a": 1.0}, "n1")

        candidates = agg.get_classification_candidates(min_vectors=3)
        assert len(candidates) == 1
        assert candidates[0].mac == "AA:BB:CC:DD:EE:01"

    def test_get_stats(self):
        agg = FeatureAggregator()
        agg.ingest("AA:BB:CC:DD:EE:01", {"a": 1.0}, "n1")
        agg.ingest("AA:BB:CC:DD:EE:02", {"a": 2.0}, "n2")
        stats = agg.get_stats()
        assert stats["device_count"] == 2
        assert stats["total_vectors"] == 2
        assert stats["unique_nodes"] == 2

    def test_prune_stale(self):
        agg = FeatureAggregator()
        agg.ingest("AA:BB:CC:DD:EE:01", {"a": 1.0}, "n1")
        # Force entry to be old
        entry = agg._devices["AA:BB:CC:DD:EE:01"]
        entry.last_seen = time.time() - 7200  # 2 hours ago
        pruned = agg.prune_stale(max_age_s=3600)
        assert pruned == 1
        assert agg.device_count == 0

    def test_eviction_at_capacity(self):
        """Test that oldest entry is evicted when at max capacity."""
        agg = FeatureAggregator()
        # Override max for test
        import engine.intelligence.feature_aggregator as mod
        original_max = mod.MAX_TRACKED_DEVICES
        mod.MAX_TRACKED_DEVICES = 3
        try:
            agg.ingest("AA:01", {"a": 1.0}, "n1")
            agg._devices["AA:01"].last_seen = 1000  # oldest
            agg.ingest("AA:02", {"a": 2.0}, "n1")
            agg.ingest("AA:03", {"a": 3.0}, "n1")
            # This should evict AA:01
            agg.ingest("AA:04", {"a": 4.0}, "n1")
            assert agg.device_count == 3
            assert agg.get_features("AA:01") is None
        finally:
            mod.MAX_TRACKED_DEVICES = original_max
