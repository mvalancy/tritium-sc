# Plugins

**Where you are:** `tritium-sc/plugins/` — extensible plugin system for the Command Center.

**Parent:** [../CLAUDE.md](../CLAUDE.md) | [../../CLAUDE.md](../../CLAUDE.md) (tritium root)

## What This Is

Plugins extend the Command Center with new capabilities. Each plugin can register event handlers, add API endpoints, inject UI panels, and interact with the tactical engine. Plugins are the primary growth mechanism — every new sensor type, protocol bridge, or analysis capability is added as a plugin.

## Available Plugins

| Plugin | Directory | Purpose | Status |
|--------|-----------|---------|--------|
| **edge_tracker** | `edge_tracker/` | BLE/WiFi presence tracking from tritium-edge devices, TargetTracker integration | Active |
| **meshtastic** | `meshtastic/` | LoRa mesh radio bridge — GPS nodes, text messages, waypoints on map | Active |
| **camera_feeds** | `camera_feeds/` | Multi-source camera management (synthetic/RTSP/MJPEG/MQTT/USB) | Active |
| **fleet_dashboard** | `fleet_dashboard/` | Device registry, battery levels, uptime, sighting counts | Active |
| **yolo_detector** | `yolo_detector/` | YOLO object detection pipeline — person/vehicle/animal classification | Active |
| **tak_bridge** | `tak_bridge/` | ATAK/CoT interoperability — multicast UDP, TCP, MQTT | Active |
| **gis_layers** | `gis_layers/` | OSM/satellite/terrain/building map tile layers and providers | Active |
| **automation** | `automation/` | If-then rule engine — 9 condition operators, 6 action types | Active |
| **threat_feeds** | `threat_feeds/` | STIX/TAXII-style known-bad indicator matching | Active |
| **rf_motion** | `rf_motion/` | RF-based motion detection using stationary BLE/WiFi RSSI variance | Active |
| **acoustic** | `acoustic/` | Audio classification — gunshot/voice/vehicle/animal detection | Active |
| **graphlings** | `graphlings/` | NPC digital life agents — autonomous AI entities with memory and behavior | Active |
| **npc_thoughts** | `npc_thoughts.py` | NPC context-aware thought bubble generation and display | Active |

## Plugin Architecture

Each plugin directory contains:
- `plugin.py` — Plugin class with lifecycle hooks (startup, shutdown, tick)
- `routes.py` — FastAPI router with API endpoints
- `__init__.py` — Package init
- A loader file at `plugins/{name}_loader.py` — Registers the plugin with the system

Some plugins have additional modules:
- `graphlings/` has 12+ files (agent bridge, perception, memory, lifecycle, motor, battle mode)
- `rf_motion/` has `detector.py` and `zones.py` for zone-based motion detection
- `threat_feeds/` has `feeds.py` for feed source management
- `automation/` has `rules.py` for rule definitions
- `camera_feeds/` has `sources.py` for camera source abstraction
- `gis_layers/` has `providers.py` for tile provider implementations
- `yolo_detector/` has `detector.py` for the detection pipeline

## Related

- [../docs/PLUGIN-SPEC.md](../docs/PLUGIN-SPEC.md) — Plugin specification and development guide
- [../src/engine/plugins/](../src/engine/plugins/) — Plugin loader infrastructure
- [../src/engine/comms/event_bus.py](../src/engine/comms/event_bus.py) — Event bus that plugins subscribe to
- [../src/engine/tactical/target_tracker.py](../src/engine/tactical/target_tracker.py) — Target tracker that plugins feed data into
