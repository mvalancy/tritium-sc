# YOLO Detector Plugin

Object detection plugin for TRITIUM-SC using YOLOv8.

## Overview

Wraps the YOLO detection pipeline into a modular plugin that:
- Subscribes to camera frame events from EventBus (e.g., from camera_feeds plugin or MQTT)
- Runs YOLOv8 inference on received frames
- Publishes detections to EventBus (`yolo_detections` events) and TargetTracker
- Gracefully degrades to stub mode if `ultralytics` is not installed

## Configuration

Settings passed via `PluginContext.settings`:

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `model_path` | str | `yolov8n.pt` | YOLO model file path |
| `confidence_threshold` | float | `0.5` | Minimum detection confidence |
| `device` | str | `None` | Inference device (`cuda`, `cpu`, or auto) |
| `inference_interval` | float | `0.5` | Minimum seconds between inferences |

## API Endpoints

| Method | Path | Description |
|--------|------|-------------|
| GET | `/api/yolo/status` | Detector status and model info |
| POST | `/api/yolo/configure` | Update threshold / interval |
| GET | `/api/yolo/stats` | Detection statistics |
| GET | `/api/yolo/last` | Most recent detection result |

## Capabilities

- `ai` — AI/ML integration
- `data_source` — Produces detection data
- `routes` — Registers FastAPI routes

## Usage

Frames can arrive via:
1. **EventBus** — publish `camera_frame` events with `{"frame": np.ndarray}`
2. **Direct API** — call `plugin.detect_frame(frame)` or `plugin.submit_frame(frame)`

## Dependencies

- `ultralytics` (optional) — required for real inference, stub mode without it
- `numpy` — frame representation
- `opencv-python` — image processing
