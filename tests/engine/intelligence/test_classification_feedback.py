# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the classification feedback loop service."""

import pytest

from engine.intelligence.classification_feedback import (
    ClassificationFeedbackService,
    ClassificationRecord,
)


class TestClassificationRecord:
    """Tests for ClassificationRecord."""

    def test_create(self):
        r = ClassificationRecord("AA:BB:CC:DD:EE:FF")
        assert r.mac == "AA:BB:CC:DD:EE:FF"
        assert r.last_type == "unknown"
        assert r.consistency_score == 1.0

    def test_add_classification(self):
        r = ClassificationRecord("XX")
        r.add_classification("phone", 0.9)
        assert r.last_type == "phone"
        assert r.last_confidence == pytest.approx(0.9)

    def test_consistency_score_consistent(self):
        r = ClassificationRecord("XX")
        for _ in range(5):
            r.add_classification("phone", 0.9)
        assert r.consistency_score == pytest.approx(1.0)

    def test_consistency_score_inconsistent(self):
        r = ClassificationRecord("XX")
        r.add_classification("phone", 0.9)
        r.add_classification("tablet", 0.8)
        r.add_classification("laptop", 0.7)
        r.add_classification("watch", 0.6)
        # last type is "watch", only 1 out of 4 matches
        assert r.consistency_score == pytest.approx(0.25)

    def test_to_dict(self):
        r = ClassificationRecord("AA:BB")
        r.add_classification("phone", 0.85)
        d = r.to_dict()
        assert d["mac"] == "AA:BB"
        assert d["last_type"] == "phone"
        assert d["classification_count"] == 1


class TestClassificationFeedbackService:
    """Tests for ClassificationFeedbackService."""

    def test_create(self):
        svc = ClassificationFeedbackService()
        stats = svc.get_stats()
        assert stats["devices_classified"] == 0
        assert stats["mqtt_connected"] is False

    def test_set_mqtt_publisher(self):
        svc = ClassificationFeedbackService()
        svc.set_mqtt_publisher(lambda topic, payload: None)
        stats = svc.get_stats()
        assert stats["mqtt_connected"] is True

    def test_get_record_not_found(self):
        svc = ClassificationFeedbackService()
        assert svc.get_record("XX:XX:XX:XX:XX:XX") is None

    def test_get_stats_empty(self):
        svc = ClassificationFeedbackService()
        stats = svc.get_stats()
        assert stats["devices_classified"] == 0
        assert stats["consistency_rate"] == pytest.approx(1.0)
        assert stats["total_feedback_sent"] == 0

    def test_get_node_metrics_empty(self):
        svc = ClassificationFeedbackService()
        metrics = svc.get_node_metrics()
        assert isinstance(metrics, dict)
