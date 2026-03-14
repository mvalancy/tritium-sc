# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""YOLO model wrapper with graceful degradation.

Wraps ultralytics YOLO model loading and inference. If ultralytics is not
installed, falls back to a stub that returns empty detections, allowing
the plugin to load and run without a real model.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

log = logging.getLogger("yolo-detector")

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    log.warning("ultralytics not installed — YOLO detector will use stubs")


@dataclass
class Detection:
    """A single object detection."""

    class_id: int
    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]  # x1, y1, x2, y2
    center: tuple[int, int]
    area: int

    def to_dict(self) -> dict:
        return {
            "class_id": int(self.class_id),
            "class_name": str(self.class_name),
            "confidence": round(float(self.confidence), 3),
            "bbox": tuple(int(x) for x in self.bbox),
            "center": tuple(int(x) for x in self.center),
            "area": int(self.area),
        }


@dataclass
class FrameResult:
    """Detections for a single frame."""

    timestamp: float
    detections: list[Detection]
    frame_shape: tuple[int, int]  # height, width
    inference_ms: float = 0.0

    @property
    def people_count(self) -> int:
        return sum(1 for d in self.detections if d.class_name == "person")

    @property
    def vehicle_count(self) -> int:
        return sum(
            1 for d in self.detections
            if d.class_name in ("car", "truck", "bus", "motorcycle")
        )

    def to_dict(self) -> dict:
        return {
            "timestamp": self.timestamp,
            "detections": [d.to_dict() for d in self.detections],
            "frame_shape": list(self.frame_shape),
            "inference_ms": round(self.inference_ms, 1),
            "people_count": self.people_count,
            "vehicle_count": self.vehicle_count,
        }


# COCO classes relevant for security/surveillance
RELEVANT_CLASSES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    6: "train",
    7: "truck",
    14: "bird",
    15: "cat",
    16: "dog",
    24: "backpack",
    26: "handbag",
    28: "suitcase",
}


@dataclass
class DetectorStats:
    """Running statistics for the detector."""

    frames_processed: int = 0
    total_detections: int = 0
    total_inference_ms: float = 0.0
    last_inference_ms: float = 0.0
    people_detected: int = 0
    vehicles_detected: int = 0
    model_loaded: bool = False
    model_name: str = ""
    using_stub: bool = False

    @property
    def avg_inference_ms(self) -> float:
        if self.frames_processed == 0:
            return 0.0
        return self.total_inference_ms / self.frames_processed

    def to_dict(self) -> dict:
        return {
            "frames_processed": self.frames_processed,
            "total_detections": self.total_detections,
            "avg_inference_ms": round(self.avg_inference_ms, 1),
            "last_inference_ms": round(self.last_inference_ms, 1),
            "people_detected": self.people_detected,
            "vehicles_detected": self.vehicles_detected,
            "model_loaded": self.model_loaded,
            "model_name": self.model_name,
            "using_stub": self.using_stub,
        }


class YOLODetector:
    """YOLO object detector with graceful degradation.

    If ultralytics is not installed, returns empty detections for every
    frame, allowing the rest of the plugin pipeline to operate.
    """

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.5,
        device: Optional[str] = None,
    ) -> None:
        self._model_path = model_path
        self._confidence_threshold = confidence_threshold
        self._device = device
        self._model: Any = None
        self._class_names: dict[int, str] = {}
        self.stats = DetectorStats(model_name=model_path)

        if YOLO_AVAILABLE:
            self._load_model()
        else:
            self.stats.using_stub = True
            log.info("YOLO detector running in stub mode (no ultralytics)")

    def _load_model(self) -> None:
        """Load the YOLO model."""
        try:
            self._model = YOLO(self._model_path)
            if self._device:
                self._model.to(self._device)
            self._class_names = self._model.names
            self.stats.model_loaded = True
            log.info(
                "YOLO model loaded: %s (%d classes)",
                self._model_path, len(self._class_names),
            )
        except Exception as exc:
            log.error("Failed to load YOLO model %s: %s", self._model_path, exc)
            self.stats.using_stub = True

    @property
    def available(self) -> bool:
        """True if a real YOLO model is loaded."""
        return self._model is not None

    @property
    def confidence_threshold(self) -> float:
        return self._confidence_threshold

    @confidence_threshold.setter
    def confidence_threshold(self, value: float) -> None:
        self._confidence_threshold = max(0.0, min(1.0, value))

    def detect(self, frame: np.ndarray) -> FrameResult:
        """Run detection on a single frame.

        Args:
            frame: BGR image as numpy array.

        Returns:
            FrameResult with all detections.
        """
        height, width = frame.shape[:2]
        now = time.time()

        if not self.available:
            # Stub mode: return empty result
            self.stats.frames_processed += 1
            return FrameResult(
                timestamp=now,
                detections=[],
                frame_shape=(height, width),
                inference_ms=0.0,
            )

        t0 = time.monotonic()
        results = self._model(
            frame, conf=self._confidence_threshold, verbose=False,
        )
        inference_ms = (time.monotonic() - t0) * 1000.0

        detections: list[Detection] = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue

            for i in range(len(boxes)):
                class_id = int(boxes.cls[i].item())
                confidence = float(boxes.conf[i].item())

                if class_id not in RELEVANT_CLASSES:
                    continue

                class_name = self._class_names.get(class_id, f"class_{class_id}")
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                center = ((x1 + x2) // 2, (y1 + y2) // 2)
                area = int((x2 - x1) * (y2 - y1))

                detections.append(Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    bbox=(int(x1), int(y1), int(x2), int(y2)),
                    center=center,
                    area=area,
                ))

        # Update stats
        self.stats.frames_processed += 1
        self.stats.total_detections += len(detections)
        self.stats.total_inference_ms += inference_ms
        self.stats.last_inference_ms = inference_ms
        self.stats.people_detected += sum(
            1 for d in detections if d.class_name == "person"
        )
        self.stats.vehicles_detected += sum(
            1 for d in detections
            if d.class_name in ("car", "truck", "bus", "motorcycle")
        )

        return FrameResult(
            timestamp=now,
            detections=detections,
            frame_shape=(height, width),
            inference_ms=inference_ms,
        )
