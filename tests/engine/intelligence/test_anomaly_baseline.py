# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for Anomaly Baseline Collector."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "src"))

from engine.intelligence.anomaly_baseline import (
    AnomalyBaselineCollector,
    BaselineStats,
    RFMetrics,
    Anomaly,
    MIN_BASELINE_SAMPLES,
)


class TestBaselineStats:
    """Test running statistics calculator."""

    def test_empty(self):
        bs = BaselineStats()
        assert bs.count == 0
        assert bs.mean() == 0.0
        assert bs.std() == 0.0

    def test_single_value(self):
        bs = BaselineStats()
        bs.add(10.0)
        assert bs.count == 1
        assert bs.mean() == 10.0
        assert bs.std() == 0.0

    def test_known_values(self):
        bs = BaselineStats()
        for v in [2.0, 4.0, 6.0, 8.0, 10.0]:
            bs.add(v)
        assert bs.mean() == 6.0
        assert bs.std() == pytest.approx(2.828, abs=0.01)

    def test_no_anomaly_insufficient_data(self):
        bs = BaselineStats()
        for i in range(10):
            bs.add(float(i))
        is_anomaly, _, _ = bs.check_anomaly(100.0)
        assert not is_anomaly  # Not enough samples

    def test_anomaly_detection(self):
        bs = BaselineStats()
        # Add 300 samples around 10.0
        for i in range(300):
            bs.add(10.0 + (i % 3) - 1.0)  # 9, 10, 11 repeating

        # Check normal value
        is_anomaly, _, _ = bs.check_anomaly(10.0)
        assert not is_anomaly

        # Check anomalous value
        is_anomaly, deviation, direction = bs.check_anomaly(20.0)
        assert is_anomaly
        assert direction == "above"
        assert deviation > 2.0

    def test_anomaly_below(self):
        bs = BaselineStats()
        for i in range(300):
            bs.add(50.0 + (i % 5))

        is_anomaly, deviation, direction = bs.check_anomaly(30.0)
        assert is_anomaly
        assert direction == "below"


class TestRFMetrics:
    """Test RF metrics dataclass."""

    def test_to_dict(self):
        m = RFMetrics(
            timestamp=1000.0,
            ble_device_count=10,
            wifi_network_count=5,
            rssi_mean=-60.0,
        )
        d = m.to_dict()
        assert d["ble_device_count"] == 10
        assert d["wifi_network_count"] == 5
        assert d["rssi_mean"] == -60.0


class TestAnomalyBaselineCollector:
    """Test the anomaly baseline collector."""

    def _make_provider(self, ble=10, wifi=5, rssi=-60.0):
        def provider():
            return RFMetrics(
                ble_device_count=ble,
                wifi_network_count=wifi,
                rssi_mean=rssi,
                rssi_std=5.0,
                rssi_min=-80.0,
                rssi_max=-40.0,
            )
        return provider

    def test_init(self):
        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(),
        )
        assert not collector.is_baseline_ready
        assert collector.baseline_progress == 0.0
        assert collector.sample_count == 0

    def test_collect_sample(self):
        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(),
        )
        metrics, anomalies = collector.collect_sample()
        assert metrics.ble_device_count == 10
        assert anomalies == []  # No baseline yet
        assert collector.sample_count == 1

    def test_no_anomaly_during_baseline(self):
        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(),
        )
        # Collect 50 samples — not enough for anomaly detection
        for _ in range(50):
            _, anomalies = collector.collect_sample()
            assert anomalies == []

    def test_anomaly_after_baseline(self):
        # Collect baseline at BLE=10
        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(ble=10),
        )
        for _ in range(MIN_BASELINE_SAMPLES):
            collector.collect_sample()

        assert collector.is_baseline_ready

        # Now inject anomalous reading
        collector._provider = self._make_provider(ble=50)
        _, anomalies = collector.collect_sample()
        assert len(anomalies) > 0
        assert any(a.metric_name == "ble_device_count" for a in anomalies)

    def test_callback_invoked(self):
        received = []

        def on_anomaly(anomalies):
            received.extend(anomalies)

        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(ble=10),
            on_anomaly=on_anomaly,
        )
        for _ in range(MIN_BASELINE_SAMPLES):
            collector.collect_sample()

        collector._provider = self._make_provider(ble=50)
        collector.collect_sample()
        assert len(received) > 0

    def test_get_status(self):
        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(),
        )
        collector.collect_sample()
        status = collector.get_status()
        assert "baseline_ready" in status
        assert "baselines" in status
        assert "sample_count" in status
        assert status["sample_count"] == 1

    def test_get_recent_anomalies_empty(self):
        collector = AnomalyBaselineCollector(
            metrics_provider=self._make_provider(),
        )
        assert collector.get_recent_anomalies() == []
