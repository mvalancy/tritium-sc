# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Classification Feedback Loop — sends ML classifications back to edge nodes.

When the BLE classification learner classifies a device, this service
publishes the result via MQTT so the originating edge node can cache it
and include it in future sightings. Also tracks classification consistency
over time to detect drift or misclassification.
"""
from __future__ import annotations

import json
import logging
import threading
import time
from typing import Any, Optional

logger = logging.getLogger("classification_feedback")

# MQTT topic pattern for classification feedback
FEEDBACK_TOPIC = "tritium/{node_id}/cmd/classify"


class ClassificationRecord:
    """Tracks classification history for a single device."""

    __slots__ = (
        "mac", "classifications", "last_type", "last_confidence",
        "consistency_score", "feedback_count",
    )

    def __init__(self, mac: str) -> None:
        self.mac = mac
        self.classifications: list[tuple[str, float, float]] = []  # (type, conf, timestamp)
        self.last_type: str = "unknown"
        self.last_confidence: float = 0.0
        self.consistency_score: float = 1.0
        self.feedback_count: int = 0

    def add_classification(self, device_type: str, confidence: float) -> None:
        """Record a new classification result."""
        now = time.time()
        self.classifications.append((device_type, confidence, now))
        self.last_type = device_type
        self.last_confidence = confidence

        # Keep last 20 classifications
        if len(self.classifications) > 20:
            self.classifications = self.classifications[-20:]

        # Compute consistency: fraction agreeing with most recent type
        if len(self.classifications) >= 2:
            matching = sum(1 for t, _, _ in self.classifications if t == device_type)
            self.consistency_score = matching / len(self.classifications)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mac": self.mac,
            "last_type": self.last_type,
            "last_confidence": self.last_confidence,
            "consistency_score": self.consistency_score,
            "classification_count": len(self.classifications),
            "feedback_count": self.feedback_count,
        }


class ClassificationFeedbackService:
    """Manages the classification feedback loop between SC and edge nodes.

    Flow:
    1. Feature vectors arrive from edge nodes via MQTT
    2. FeatureAggregator collects and averages them
    3. BLEClassificationLearner classifies using aggregated features
    4. This service sends classification back to edge via MQTT
    5. Edge caches classification, includes in future sightings
    6. SC verifies consistency over time
    """

    def __init__(self) -> None:
        self._records: dict[str, ClassificationRecord] = {}
        self._lock = threading.Lock()
        self._mqtt_publish_fn: Optional[Any] = None
        self._total_feedback_sent: int = 0
        self._total_inconsistencies: int = 0

    def set_mqtt_publisher(self, publish_fn: Any) -> None:
        """Set the MQTT publish function for sending feedback.

        Args:
            publish_fn: Callable(topic: str, payload: str) -> None
        """
        self._mqtt_publish_fn = publish_fn

    def classify_and_feedback(
        self,
        mac: str,
        node_id: str,
        features: dict[str, float],
    ) -> Optional[dict[str, Any]]:
        """Classify a device and optionally send feedback to edge.

        Args:
            mac: Device MAC address.
            node_id: Edge node that reported this device.
            features: Feature vector dict.

        Returns:
            Classification result dict, or None if classification failed.
        """
        try:
            from engine.intelligence.ble_classification_learner import (
                get_ble_classification_learner,
            )

            learner = get_ble_classification_learner()
            if not learner.is_trained:
                return None

            # Convert features dict to the format expected by the learner
            device_type, confidence = learner.predict(
                mac=mac,
                rssi=int(features.get("rssi_near_pct", 0) * -30
                         + features.get("rssi_mid_pct", 0) * -60
                         + features.get("rssi_far_pct", 0) * -85),
                name="" if features.get("name_length", 0) == 0 else "x" * int(features.get("name_length", 0)),
            )

            if device_type == "unknown" or confidence < 0.4:
                return None

            result = {
                "mac": mac,
                "predicted_type": device_type,
                "confidence": confidence,
                "node_id": node_id,
            }

            # Record classification
            with self._lock:
                record = self._records.get(mac)
                if record is None:
                    record = ClassificationRecord(mac)
                    self._records[mac] = record
                record.add_classification(device_type, confidence)

                # Check consistency — flag if type keeps changing
                if record.consistency_score < 0.5 and len(record.classifications) >= 5:
                    self._total_inconsistencies += 1
                    logger.warning(
                        "Classification inconsistency for %s: score=%.2f",
                        mac, record.consistency_score,
                    )
                    result["inconsistent"] = True

            # Send feedback to edge node
            if self._mqtt_publish_fn and confidence >= 0.5:
                self._send_feedback(node_id, mac, device_type, confidence)

            return result

        except Exception as exc:
            logger.debug("Classification feedback failed: %s", exc)
            return None

    def _send_feedback(
        self,
        node_id: str,
        mac: str,
        device_type: str,
        confidence: float,
    ) -> None:
        """Send classification feedback to edge node via MQTT."""
        topic = FEEDBACK_TOPIC.format(node_id=node_id)
        payload = json.dumps({
            "cmd": "classify_device",
            "mac": mac,
            "predicted_type": device_type,
            "confidence": round(confidence, 3),
            "timestamp": time.time(),
        })

        try:
            self._mqtt_publish_fn(topic, payload)
            self._total_feedback_sent += 1

            with self._lock:
                record = self._records.get(mac)
                if record:
                    record.feedback_count += 1

            logger.debug(
                "Sent classification feedback: %s -> %s (%.2f) to %s",
                mac, device_type, confidence, node_id,
            )
        except Exception as exc:
            logger.warning("Failed to send classification feedback: %s", exc)

    def get_record(self, mac: str) -> Optional[dict[str, Any]]:
        """Get classification history for a device."""
        mac = mac.upper().strip()
        with self._lock:
            record = self._records.get(mac)
            return record.to_dict() if record else None

    def get_stats(self) -> dict[str, Any]:
        """Get feedback service statistics."""
        with self._lock:
            classified = len(self._records)
            consistent = sum(
                1 for r in self._records.values()
                if r.consistency_score >= 0.8
            )

        return {
            "devices_classified": classified,
            "devices_consistent": consistent,
            "consistency_rate": (
                consistent / classified if classified > 0 else 1.0
            ),
            "total_feedback_sent": self._total_feedback_sent,
            "total_inconsistencies": self._total_inconsistencies,
            "mqtt_connected": self._mqtt_publish_fn is not None,
        }

    def get_node_metrics(self) -> dict[str, dict[str, Any]]:
        """Get per-node ML intelligence metrics.

        Returns dict of node_id -> metrics.
        """
        from engine.intelligence.feature_aggregator import get_feature_aggregator
        aggregator = get_feature_aggregator()

        node_metrics: dict[str, dict[str, Any]] = {}

        # Gather per-node stats from aggregator
        for entry_dict in aggregator.get_all_features(limit=500):
            for node_id in entry_dict.get("node_ids", []):
                if node_id not in node_metrics:
                    node_metrics[node_id] = {
                        "node_id": node_id,
                        "total_devices_seen": 0,
                        "devices_classified": 0,
                        "feedback_sent": 0,
                        "feature_vectors": 0,
                    }
                node_metrics[node_id]["total_devices_seen"] += 1
                node_metrics[node_id]["feature_vectors"] += entry_dict.get("vector_count", 0)

        # Add classification info
        with self._lock:
            for record in self._records.values():
                # We don't track which node a classification came from directly,
                # but feedback_count tells us how many were sent
                pass

        # Add feedback counts from aggregator stats
        agg_stats = aggregator.get_stats()
        total_feedback = self._total_feedback_sent

        return node_metrics


# ---------------------------------------------------------------------------
# Singleton
# ---------------------------------------------------------------------------

_service: Optional[ClassificationFeedbackService] = None


def get_classification_feedback_service() -> ClassificationFeedbackService:
    """Get or create the singleton ClassificationFeedbackService."""
    global _service
    if _service is None:
        _service = ClassificationFeedbackService()
    return _service
