# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the YOLO Detector plugin.

All tests run without requiring a real YOLO model — inference is either
stubbed (ultralytics not installed) or mocked.
"""

import time
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from engine.plugins.base import PluginContext, PluginInterface


# ---------------------------------------------------------------------------
# Detector tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestYOLODetector:
    """Test the YOLODetector wrapper."""

    def test_stub_mode_when_ultralytics_missing(self):
        """Detector should work in stub mode without ultralytics."""
        with patch.dict("sys.modules", {"ultralytics": None}):
            # Force reload to pick up missing module
            import importlib
            import plugins.yolo_detector.detector as det_mod
            original_available = det_mod.YOLO_AVAILABLE
            det_mod.YOLO_AVAILABLE = False

            try:
                from plugins.yolo_detector.detector import YOLODetector
                detector = YOLODetector.__new__(YOLODetector)
                detector._model_path = "yolov8n.pt"
                detector._confidence_threshold = 0.5
                detector._device = None
                detector._model = None
                detector._class_names = {}
                from plugins.yolo_detector.detector import DetectorStats
                detector.stats = DetectorStats(
                    model_name="yolov8n.pt", using_stub=True,
                )

                assert not detector.available
                assert detector.stats.using_stub

                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                result = detector.detect(frame)
                assert result.detections == []
                assert result.frame_shape == (480, 640)
                assert detector.stats.frames_processed == 1
            finally:
                det_mod.YOLO_AVAILABLE = original_available

    def test_detection_dataclass(self):
        from plugins.yolo_detector.detector import Detection
        det = Detection(
            class_id=0, class_name="person", confidence=0.85,
            bbox=(10, 20, 100, 200), center=(55, 110), area=16200,
        )
        d = det.to_dict()
        assert d["class_name"] == "person"
        assert d["confidence"] == 0.85
        assert d["bbox"] == (10, 20, 100, 200)
        assert d["center"] == (55, 110)
        assert d["area"] == 16200

    def test_frame_result_counts(self):
        from plugins.yolo_detector.detector import Detection, FrameResult
        dets = [
            Detection(0, "person", 0.9, (0, 0, 50, 50), (25, 25), 2500),
            Detection(0, "person", 0.8, (60, 0, 110, 50), (85, 25), 2500),
            Detection(2, "car", 0.7, (0, 100, 200, 200), (100, 150), 20000),
        ]
        result = FrameResult(
            timestamp=time.time(), detections=dets,
            frame_shape=(480, 640), inference_ms=12.5,
        )
        assert result.people_count == 2
        assert result.vehicle_count == 1
        d = result.to_dict()
        assert d["people_count"] == 2
        assert d["vehicle_count"] == 1
        assert len(d["detections"]) == 3

    def test_detector_stats(self):
        from plugins.yolo_detector.detector import DetectorStats
        stats = DetectorStats(model_name="test.pt")
        assert stats.avg_inference_ms == 0.0
        stats.frames_processed = 10
        stats.total_inference_ms = 100.0
        assert stats.avg_inference_ms == 10.0
        d = stats.to_dict()
        assert d["model_name"] == "test.pt"
        assert d["avg_inference_ms"] == 10.0

    def test_confidence_threshold_clamping(self):
        from plugins.yolo_detector.detector import DetectorStats

        # Build detector manually to avoid model loading
        from plugins.yolo_detector.detector import YOLODetector
        detector = YOLODetector.__new__(YOLODetector)
        detector._model = None
        detector._confidence_threshold = 0.5
        detector.stats = DetectorStats(using_stub=True)

        detector.confidence_threshold = 1.5
        assert detector.confidence_threshold == 1.0

        detector.confidence_threshold = -0.5
        assert detector.confidence_threshold == 0.0

        detector.confidence_threshold = 0.75
        assert detector.confidence_threshold == 0.75

    def test_relevant_classes_defined(self):
        from plugins.yolo_detector.detector import RELEVANT_CLASSES
        assert 0 in RELEVANT_CLASSES  # person
        assert RELEVANT_CLASSES[0] == "person"
        assert 2 in RELEVANT_CLASSES  # car
        assert 16 in RELEVANT_CLASSES  # dog


# ---------------------------------------------------------------------------
# Plugin tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestYOLODetectorPlugin:
    """Test the YOLODetectorPlugin interface and lifecycle."""

    def test_implements_plugin_interface(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        assert isinstance(plugin, PluginInterface)

    def test_plugin_identity(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        assert plugin.plugin_id == "tritium.yolo-detector"
        assert plugin.name == "YOLO Detector"
        assert plugin.version == "1.0.0"

    def test_capabilities(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        caps = plugin.capabilities
        assert "ai" in caps
        assert "data_source" in caps
        assert "routes" in caps

    def test_start_stop_lifecycle(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        assert not plugin.healthy
        plugin.start()
        assert plugin.healthy
        plugin.stop()
        assert not plugin.healthy

    def test_start_idempotent(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        plugin.start()
        plugin.start()  # second start should be safe
        assert plugin.healthy
        plugin.stop()

    def test_stop_idempotent(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        plugin.stop()  # stop before start should be safe
        plugin.start()
        plugin.stop()
        plugin.stop()  # double stop should be safe
        assert not plugin.healthy

    def test_configure_applies_settings(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        ctx = PluginContext(
            event_bus=None,
            target_tracker=None,
            simulation_engine=None,
            settings={
                "model_path": "yolov8s.pt",
                "confidence_threshold": 0.7,
                "inference_interval": 1.0,
            },
            app=None,
            logger=None,
            plugin_manager=None,
        )
        plugin.configure(ctx)
        assert plugin._model_path == "yolov8s.pt"
        assert plugin._confidence_threshold == 0.7
        assert plugin._inference_interval == 1.0

    def test_configure_stores_event_bus(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        mock_bus = MagicMock()
        ctx = PluginContext(
            event_bus=mock_bus,
            target_tracker=MagicMock(),
            simulation_engine=None,
            settings={},
            app=None,
            logger=None,
            plugin_manager=None,
        )
        plugin.configure(ctx)
        assert plugin._event_bus is mock_bus

    def test_detect_frame_requires_start(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        with pytest.raises(RuntimeError, match="not started"):
            plugin.detect_frame(frame)

    def test_detect_frame_returns_result(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        plugin.start()
        try:
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            result = plugin.detect_frame(frame)
            # In stub mode, returns empty detections
            assert result is not None
            assert result.frame_shape == (480, 640)
            assert isinstance(result.detections, list)
            assert plugin.last_result is result
        finally:
            plugin.stop()

    def test_detect_frame_publishes_to_event_bus(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        from plugins.yolo_detector.detector import Detection, FrameResult

        plugin = YOLODetectorPlugin()
        mock_bus = MagicMock()
        plugin._event_bus = mock_bus
        plugin.start()

        try:
            # Mock the detector to return a detection
            mock_result = FrameResult(
                timestamp=time.time(),
                detections=[
                    Detection(0, "person", 0.9, (10, 20, 100, 200), (55, 110), 8100),
                ],
                frame_shape=(480, 640),
                inference_ms=5.0,
            )
            plugin._detector.detect = MagicMock(return_value=mock_result)

            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            result = plugin.detect_frame(frame)

            assert len(result.detections) == 1
            mock_bus.publish.assert_called_once()
            call_args = mock_bus.publish.call_args
            assert call_args[0][0] == "yolo_detections"
        finally:
            plugin.stop()

    def test_detect_frame_updates_target_tracker(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        from plugins.yolo_detector.detector import Detection, FrameResult

        plugin = YOLODetectorPlugin()
        mock_tracker = MagicMock()
        plugin._target_tracker = mock_tracker
        plugin.start()

        try:
            mock_result = FrameResult(
                timestamp=time.time(),
                detections=[
                    Detection(0, "person", 0.9, (10, 20, 100, 200), (55, 110), 8100),
                ],
                frame_shape=(480, 640),
            )
            plugin._detector.detect = MagicMock(return_value=mock_result)

            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            plugin.detect_frame(frame)

            mock_tracker.update_target.assert_called_once()
        finally:
            plugin.stop()

    def test_no_publish_on_empty_detections(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        mock_bus = MagicMock()
        plugin._event_bus = mock_bus
        plugin.start()

        try:
            frame = np.zeros((100, 100, 3), dtype=np.uint8)
            # Stub mode returns empty detections
            plugin.detect_frame(frame)
            mock_bus.publish.assert_not_called()
        finally:
            plugin.stop()

    def test_stats_before_start(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        stats = plugin.stats
        assert stats == {"status": "not_started"}

    def test_stats_after_start(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        plugin.start()
        try:
            stats = plugin.stats
            assert "frames_processed" in stats
            assert "model_name" in stats
        finally:
            plugin.stop()

    def test_confidence_threshold_property(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        assert plugin.confidence_threshold == 0.5
        plugin.confidence_threshold = 0.8
        assert plugin.confidence_threshold == 0.8

    def test_submit_frame(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        plugin = YOLODetectorPlugin()
        plugin.start()
        try:
            frame = np.zeros((100, 100, 3), dtype=np.uint8)
            plugin.submit_frame(frame)
            with plugin._frame_lock:
                assert plugin._pending_frame is not None
        finally:
            plugin.stop()

    def test_event_bus_subscription_on_start(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        from engine.comms.event_bus import EventBus

        bus = EventBus()
        plugin = YOLODetectorPlugin()
        plugin._event_bus = bus
        plugin.start()
        try:
            assert plugin._event_queue is not None
            assert plugin._worker_thread is not None
            assert plugin._worker_thread.is_alive()
        finally:
            plugin.stop()

    def test_event_bus_unsubscription_on_stop(self):
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        from engine.comms.event_bus import EventBus

        bus = EventBus()
        plugin = YOLODetectorPlugin()
        plugin._event_bus = bus
        plugin.start()
        eq = plugin._event_queue
        plugin.stop()
        assert plugin._event_queue is None
        # Queue should be removed from bus subscribers
        assert eq not in bus._subscribers


# ---------------------------------------------------------------------------
# Routes tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestYOLODetectorRoutes:
    """Test route response patterns using TestClient."""

    def _make_app(self):
        from fastapi import FastAPI
        from fastapi.testclient import TestClient
        from plugins.yolo_detector.plugin import YOLODetectorPlugin
        from plugins.yolo_detector.routes import create_router

        app = FastAPI()
        plugin = YOLODetectorPlugin()
        app.include_router(create_router(plugin))
        return TestClient(app), plugin

    def test_status_endpoint(self):
        client, plugin = self._make_app()
        resp = client.get("/api/yolo/status")
        assert resp.status_code == 200
        data = resp.json()
        assert "healthy" in data
        assert "confidence_threshold" in data
        assert "stats" in data

    def test_stats_endpoint(self):
        client, plugin = self._make_app()
        resp = client.get("/api/yolo/stats")
        assert resp.status_code == 200

    def test_last_endpoint_no_result(self):
        client, plugin = self._make_app()
        resp = client.get("/api/yolo/last")
        assert resp.status_code == 200
        data = resp.json()
        assert data["detections"] == []

    def test_configure_threshold(self):
        client, plugin = self._make_app()
        resp = client.post("/api/yolo/configure", json={
            "confidence_threshold": 0.8,
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "updated"
        assert data["confidence_threshold"] == 0.8
        assert plugin.confidence_threshold == 0.8

    def test_configure_interval(self):
        client, plugin = self._make_app()
        resp = client.post("/api/yolo/configure", json={
            "inference_interval": 2.0,
        })
        assert resp.status_code == 200
        assert resp.json()["inference_interval"] == 2.0

    def test_configure_invalid_threshold(self):
        client, plugin = self._make_app()
        resp = client.post("/api/yolo/configure", json={
            "confidence_threshold": 1.5,
        })
        assert resp.status_code == 400

    def test_configure_negative_interval(self):
        client, plugin = self._make_app()
        resp = client.post("/api/yolo/configure", json={
            "inference_interval": -1.0,
        })
        assert resp.status_code == 400


# ---------------------------------------------------------------------------
# Loader tests
# ---------------------------------------------------------------------------

@pytest.mark.unit
class TestYOLODetectorLoader:
    """Test the loader shim."""

    def test_loader_imports(self):
        from plugins.yolo_detector_loader import YOLODetectorPlugin
        assert YOLODetectorPlugin is not None
        plugin = YOLODetectorPlugin()
        assert plugin.plugin_id == "tritium.yolo-detector"
