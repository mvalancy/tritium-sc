# Synthetic Data Engine

**Where you are:** `tritium-sc/src/engine/synthetic/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The synthetic data subsystem generates realistic fake sensor data so the entire Tritium pipeline can be exercised without hardware. Demo mode starts synthetic generators that produce BLE sightings, WiFi probes, Meshtastic node updates, camera detections, and fused multi-sensor scenarios. This data flows through the exact same code paths as real sensor data.

## Key Files

| File | Purpose |
|------|---------|
| `data_generators.py` | Synthetic data generators for BLE, WiFi, Meshtastic, and camera sightings |
| `demo_mode.py` | Demo mode orchestrator — starts/stops all generators, manages lifecycle |
| `fusion_scenario.py` | Multi-sensor fusion scenarios — coordinated synthetic events across sensor types |
| `video_gen.py` | Procedural video frame generation (synthetic camera feeds) |
| `video_library.py` | Library of synthetic video scene types and templates |

## Usage

```bash
# Start the server
cd tritium-sc && ./start.sh

# Activate demo mode via API
curl -X POST http://localhost:8000/api/demo/start

# Synthetic data now flows through the full pipeline:
# BLE sightings -> target_tracker -> map
# Camera detections -> correlator -> fused targets
# Meshtastic nodes -> mesh overlay on map
```

## Related

- [../tactical/target_tracker.py](../tactical/target_tracker.py) — Receives and tracks synthetic targets
- [../tactical/correlator.py](../tactical/correlator.py) — Fuses multi-sensor synthetic data
- [../comms/mqtt_bridge.py](../comms/mqtt_bridge.py) — Synthetic data can be published via MQTT
- [../../app/routers/](../../app/routers/) — `/api/demo/start` and `/api/synthetic/cameras` endpoints
- [../../../plugins/camera_feeds/](../../../plugins/camera_feeds/) — Camera feeds plugin consumes synthetic video
- [../../../plugins/edge_tracker/](../../../plugins/edge_tracker/) — Edge tracker consumes synthetic BLE/WiFi data
