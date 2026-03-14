# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Anomaly Baseline Collector — background RF environment baselining.

Records "normal" RF environment metrics every 5 minutes:
  - BLE device count
  - WiFi network count
  - RSSI distribution (mean, std, min, max)
  - Device churn rate (new/departed per interval)

After 24 hours of baseline collection (288 samples), deviations beyond
2 standard deviations trigger anomaly alerts.

Integrates with the event bus to publish anomaly events and optionally
calls an LLM for natural language anomaly descriptions.
"""
from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

logger = logging.getLogger("anomaly_baseline")

# Collection interval in seconds (5 minutes)
COLLECTION_INTERVAL = 300

# Minimum samples before anomaly detection is active (24h / 5min = 288)
MIN_BASELINE_SAMPLES = 288

# Standard deviation threshold for anomaly detection
ANOMALY_STD_THRESHOLD = 2.0

# Maximum baseline history (7 days worth of 5-min samples)
MAX_BASELINE_HISTORY = 2016


@dataclass(slots=True)
class RFMetrics:
    """A snapshot of RF environment metrics at a point in time."""

    timestamp: float = 0.0
    ble_device_count: int = 0
    wifi_network_count: int = 0
    rssi_mean: float = -100.0
    rssi_std: float = 0.0
    rssi_min: float = -100.0
    rssi_max: float = 0.0
    device_churn_new: int = 0       # Devices seen for first time this interval
    device_churn_departed: int = 0  # Devices that disappeared this interval
    total_sightings: int = 0        # Total sighting events this interval

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "ble_device_count": self.ble_device_count,
            "wifi_network_count": self.wifi_network_count,
            "rssi_mean": round(self.rssi_mean, 2),
            "rssi_std": round(self.rssi_std, 2),
            "rssi_min": self.rssi_min,
            "rssi_max": self.rssi_max,
            "device_churn_new": self.device_churn_new,
            "device_churn_departed": self.device_churn_departed,
            "total_sightings": self.total_sightings,
        }


@dataclass(slots=True)
class Anomaly:
    """A detected anomaly in the RF environment."""

    anomaly_id: str = ""
    timestamp: float = 0.0
    metric_name: str = ""
    current_value: float = 0.0
    baseline_mean: float = 0.0
    baseline_std: float = 0.0
    deviation_sigma: float = 0.0
    direction: str = ""  # "above" or "below"
    severity: str = "low"  # low, medium, high
    description: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "anomaly_id": self.anomaly_id,
            "timestamp": self.timestamp,
            "metric_name": self.metric_name,
            "current_value": self.current_value,
            "baseline_mean": round(self.baseline_mean, 2),
            "baseline_std": round(self.baseline_std, 2),
            "deviation_sigma": round(self.deviation_sigma, 2),
            "direction": self.direction,
            "severity": self.severity,
            "description": self.description,
        }


class BaselineStats:
    """Running statistics calculator for a single metric."""

    def __init__(self, max_samples: int = MAX_BASELINE_HISTORY) -> None:
        self._values: deque[float] = deque(maxlen=max_samples)

    @property
    def count(self) -> int:
        return len(self._values)

    def add(self, value: float) -> None:
        self._values.append(value)

    def mean(self) -> float:
        if not self._values:
            return 0.0
        return sum(self._values) / len(self._values)

    def std(self) -> float:
        if len(self._values) < 2:
            return 0.0
        m = self.mean()
        variance = sum((v - m) ** 2 for v in self._values) / len(self._values)
        return math.sqrt(variance)

    def check_anomaly(
        self,
        value: float,
        threshold: float = ANOMALY_STD_THRESHOLD,
    ) -> tuple[bool, float, str]:
        """Check if a value is anomalous relative to baseline.

        Returns:
            (is_anomaly, deviation_sigma, direction)
        """
        if self.count < MIN_BASELINE_SAMPLES:
            return False, 0.0, ""

        m = self.mean()
        s = self.std()
        if s < 0.001:
            # Zero variance baseline — any change is notable
            if abs(value - m) > 0.5:
                direction = "above" if value > m else "below"
                return True, 10.0, direction
            return False, 0.0, ""

        deviation = (value - m) / s
        if abs(deviation) >= threshold:
            direction = "above" if deviation > 0 else "below"
            return True, abs(deviation), direction

        return False, abs(deviation), ""


class AnomalyBaselineCollector:
    """Background thread that collects RF environment baselines and detects anomalies.

    Uses a MetricsProvider callback to fetch current RF metrics. Starts a
    daemon thread that samples every 5 minutes. After 24h of baseline data,
    anomaly detection activates.

    Parameters
    ----------
    metrics_provider:
        Callable that returns current RFMetrics snapshot.
    on_anomaly:
        Optional callback invoked with list[Anomaly] when anomalies detected.
    llm_describer:
        Optional callable(list[Anomaly]) -> list[str] that generates
        natural language descriptions for anomalies.
    interval:
        Collection interval in seconds (default 300 = 5 minutes).
    """

    def __init__(
        self,
        metrics_provider: Callable[[], RFMetrics],
        on_anomaly: Optional[Callable[[list[Anomaly]], None]] = None,
        llm_describer: Optional[Callable[[list[Anomaly]], list[str]]] = None,
        interval: float = COLLECTION_INTERVAL,
    ) -> None:
        self._provider = metrics_provider
        self._on_anomaly = on_anomaly
        self._llm_describer = llm_describer
        self._interval = interval

        # Per-metric baseline statistics
        self._baselines: dict[str, BaselineStats] = {
            "ble_device_count": BaselineStats(),
            "wifi_network_count": BaselineStats(),
            "rssi_mean": BaselineStats(),
            "device_churn_new": BaselineStats(),
            "device_churn_departed": BaselineStats(),
            "total_sightings": BaselineStats(),
        }

        self._history: deque[RFMetrics] = deque(maxlen=MAX_BASELINE_HISTORY)
        self._anomalies: deque[Anomaly] = deque(maxlen=500)
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._anomaly_counter = 0

    @property
    def is_baseline_ready(self) -> bool:
        """Whether enough baseline data has been collected for anomaly detection."""
        return min(b.count for b in self._baselines.values()) >= MIN_BASELINE_SAMPLES

    @property
    def baseline_progress(self) -> float:
        """Progress toward baseline readiness (0.0 to 1.0)."""
        if not self._baselines:
            return 0.0
        counts = [b.count for b in self._baselines.values()]
        return min(1.0, min(counts) / MIN_BASELINE_SAMPLES)

    @property
    def sample_count(self) -> int:
        """Total samples collected."""
        return len(self._history)

    def start(self) -> None:
        """Start the background collection thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._collection_loop,
            name="anomaly-baseline",
            daemon=True,
        )
        self._thread.start()
        logger.info(
            "Anomaly baseline collector started (interval=%ds, need %d samples)",
            int(self._interval), MIN_BASELINE_SAMPLES,
        )

    def stop(self) -> None:
        """Stop the background collection thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=10)
            self._thread = None

    def collect_sample(self) -> tuple[RFMetrics, list[Anomaly]]:
        """Manually collect a single sample and check for anomalies.

        Returns:
            (metrics, anomalies) tuple.
        """
        metrics = self._provider()
        metrics.timestamp = time.time()

        anomalies = self._check_anomalies(metrics)

        # Update baselines
        with self._lock:
            self._history.append(metrics)
            self._baselines["ble_device_count"].add(float(metrics.ble_device_count))
            self._baselines["wifi_network_count"].add(float(metrics.wifi_network_count))
            self._baselines["rssi_mean"].add(metrics.rssi_mean)
            self._baselines["device_churn_new"].add(float(metrics.device_churn_new))
            self._baselines["device_churn_departed"].add(float(metrics.device_churn_departed))
            self._baselines["total_sightings"].add(float(metrics.total_sightings))

        if anomalies:
            # Try LLM descriptions
            if self._llm_describer:
                try:
                    descriptions = self._llm_describer(anomalies)
                    for i, desc in enumerate(descriptions):
                        if i < len(anomalies):
                            anomalies[i].description = desc
                except Exception as exc:
                    logger.debug("LLM anomaly description failed: %s", exc)

            with self._lock:
                self._anomalies.extend(anomalies)

            if self._on_anomaly:
                try:
                    self._on_anomaly(anomalies)
                except Exception as exc:
                    logger.warning("Anomaly callback failed: %s", exc)

        return metrics, anomalies

    def get_status(self) -> dict[str, Any]:
        """Return collector status for API response."""
        with self._lock:
            baseline_stats = {}
            for name, bs in self._baselines.items():
                baseline_stats[name] = {
                    "count": bs.count,
                    "mean": round(bs.mean(), 2),
                    "std": round(bs.std(), 2),
                }

            recent_anomalies = [a.to_dict() for a in list(self._anomalies)[-10:]]

        return {
            "running": self._running,
            "baseline_ready": self.is_baseline_ready,
            "baseline_progress": round(self.baseline_progress, 3),
            "sample_count": self.sample_count,
            "interval_seconds": self._interval,
            "baselines": baseline_stats,
            "recent_anomalies": recent_anomalies,
            "total_anomalies": len(self._anomalies),
        }

    def get_recent_anomalies(self, limit: int = 50) -> list[dict[str, Any]]:
        """Return recent anomalies as dicts."""
        with self._lock:
            return [a.to_dict() for a in list(self._anomalies)[-limit:]]

    def _check_anomalies(self, metrics: RFMetrics) -> list[Anomaly]:
        """Check current metrics against baseline for anomalies."""
        anomalies: list[Anomaly] = []

        metric_values = {
            "ble_device_count": float(metrics.ble_device_count),
            "wifi_network_count": float(metrics.wifi_network_count),
            "rssi_mean": metrics.rssi_mean,
            "device_churn_new": float(metrics.device_churn_new),
            "device_churn_departed": float(metrics.device_churn_departed),
            "total_sightings": float(metrics.total_sightings),
        }

        with self._lock:
            for name, value in metric_values.items():
                bs = self._baselines.get(name)
                if bs is None:
                    continue

                is_anomaly, deviation, direction = bs.check_anomaly(value)
                if is_anomaly:
                    self._anomaly_counter += 1
                    severity = "low"
                    if deviation >= 4.0:
                        severity = "high"
                    elif deviation >= 3.0:
                        severity = "medium"

                    anomaly = Anomaly(
                        anomaly_id=f"anomaly_{self._anomaly_counter}",
                        timestamp=metrics.timestamp,
                        metric_name=name,
                        current_value=value,
                        baseline_mean=bs.mean(),
                        baseline_std=bs.std(),
                        deviation_sigma=deviation,
                        direction=direction,
                        severity=severity,
                        description=(
                            f"{name} is {deviation:.1f} sigma {direction} baseline "
                            f"(current={value:.1f}, baseline={bs.mean():.1f} +/- {bs.std():.1f})"
                        ),
                    )
                    anomalies.append(anomaly)

        return anomalies

    def _collection_loop(self) -> None:
        """Background collection loop."""
        while self._running:
            try:
                metrics, anomalies = self.collect_sample()
                if anomalies:
                    logger.info(
                        "Detected %d anomalies: %s",
                        len(anomalies),
                        ", ".join(a.metric_name for a in anomalies),
                    )
                else:
                    logger.debug(
                        "Baseline sample %d: BLE=%d WiFi=%d RSSI=%.1f",
                        self.sample_count,
                        metrics.ble_device_count,
                        metrics.wifi_network_count,
                        metrics.rssi_mean,
                    )
            except Exception as exc:
                logger.error("Baseline collection failed: %s", exc)

            # Sleep in short intervals for responsive shutdown
            elapsed = 0.0
            while self._running and elapsed < self._interval:
                time.sleep(min(5.0, self._interval - elapsed))
                elapsed += 5.0


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------

_collector: Optional[AnomalyBaselineCollector] = None


def get_anomaly_collector() -> Optional[AnomalyBaselineCollector]:
    """Get the singleton anomaly collector (if started)."""
    return _collector


def start_anomaly_collector(
    metrics_provider: Callable[[], RFMetrics],
    on_anomaly: Optional[Callable[[list[Anomaly]], None]] = None,
    llm_describer: Optional[Callable[[list[Anomaly]], list[str]]] = None,
    interval: float = COLLECTION_INTERVAL,
) -> AnomalyBaselineCollector:
    """Start the singleton anomaly baseline collector."""
    global _collector
    if _collector is not None and _collector._running:
        return _collector
    _collector = AnomalyBaselineCollector(
        metrics_provider=metrics_provider,
        on_anomaly=on_anomaly,
        llm_describer=llm_describer,
        interval=interval,
    )
    _collector.start()
    return _collector


def stop_anomaly_collector() -> None:
    """Stop the singleton anomaly collector."""
    global _collector
    if _collector is not None:
        _collector.stop()
        _collector = None
