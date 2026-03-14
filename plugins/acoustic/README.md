# Acoustic Classification Plugin

**Where you are:** `tritium-sc/plugins/acoustic/`

**Parent:** [../README.md](../README.md) | [../../CLAUDE.md](../../CLAUDE.md)

## What This Is

The acoustic plugin provides sound classification capabilities. It processes audio input to detect and classify environmental sounds (gunshots, voices, vehicles, animals) and feeds classified audio events into the tactical target tracker as a sensor source alongside BLE, WiFi, and camera.

## Key Files

| File | Purpose |
|------|---------|
| `plugin.py` | AcousticPlugin — plugin lifecycle, event handlers, classification pipeline |
| `routes.py` | FastAPI routes — API endpoints for acoustic status and configuration |

## Loader

`../acoustic_loader.py` — Registers the plugin with the Command Center plugin system.

## Related

- [../](../) — Plugin directory overview
- [../../src/engine/comms/listener.py](../../src/engine/comms/listener.py) — Audio VAD + recording that feeds raw audio
- [../../src/engine/tactical/target_tracker.py](../../src/engine/tactical/target_tracker.py) — Target tracker that receives acoustic detections
- [../../src/engine/comms/event_bus.py](../../src/engine/comms/event_bus.py) — Event bus for acoustic detection events
