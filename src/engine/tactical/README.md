# Tactical Engine

**Where you are:** `tritium-sc/src/engine/tactical/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The tactical subsystem is the core of the Tritium unified operating picture. It owns target tracking, threat classification, sensor fusion, geo-referencing, and all intelligence analysis. Every detected entity (BLE device, WiFi network, camera detection, mesh radio) flows through this module to become a unified TrackedTarget with a unique ID, position, classification, and alliance.

## Key Files

| File | Purpose |
|------|---------|
| `target_tracker.py` | Unified target registry — all sensors feed into this single registry |
| `correlator.py` | Multi-sensor target correlation — fuses BLE + camera + WiFi into single targets |
| `correlation_strategies.py` | Pluggable correlation algorithms (proximity, temporal, identity) |
| `ble_classifier.py` | BLE device type classification (phone, watch, laptop, IoT, etc.) |
| `wifi_geolocation.py` | WiFi-based geolocation using BSSID/RSSI data |
| `trilateration.py` | Multi-node position estimation from 3+ RSSI readings |
| `geo.py` | Server-side coordinate transforms (local meters <-> lat/lng) |
| `geo_protocols.py` | Protocol interfaces for geo-aware components |
| `geofence.py` | Zone boundary detection — alerts when targets enter/exit defined areas |
| `heatmap.py` | Spatial density heatmap generation from target positions |
| `enrichment.py` | Target enrichment pipeline — OUI lookup, manufacturer, device class |
| `escalation.py` | ThreatClassifier (2Hz) + AutoDispatcher — threat level assessment |
| `dossier.py` | Target dossier data model — persistent intelligence file per target |
| `dossier_manager.py` | CRUD operations for dossiers (SQLite-backed) |
| `investigation.py` | Investigation workflow — track and manage active investigations |
| `target_history.py` | Target movement history — trails and position log over time |
| `movement_patterns.py` | Movement pattern analysis — detect patrol routes, loitering, anomalies |
| `patrol.py` | Patrol route definition and monitoring |
| `network_analysis.py` | Network topology analysis — device relationships and clusters |
| `street_graph.py` | OSM road network extraction + A* pathfinding |
| `obstacles.py` | Building and terrain obstacle definitions for line-of-sight |
| `dashboard_layouts.py` | Layout presets for tactical dashboard views |
| `temporal_playback.py` | Time-scrub replay of historical target positions |

## Data Flow

```
Edge Devices (BLE/WiFi/Camera/Mesh)
        |
        v
  MQTT / WebSocket
        |
        v
  target_tracker.py  <-- correlator.py (fuse multiple detections)
        |                    ^
        v                    |
  enrichment.py        ble_classifier.py / wifi_geolocation.py
        |
        v
  escalation.py  -->  geofence.py (zone alerts)
        |
        v
  dossier_manager.py  -->  movement_patterns.py
        |
        v
  WebSocket broadcast to frontend map
```

## Related

- [../../app/routers/](../../app/routers/) — API endpoints that expose tactical data
- [../comms/](../comms/) — MQTT bridge and event bus that feed data into tactical
- [../synthetic/](../synthetic/) — Synthetic data generators for demo mode
- [../../../../tritium-lib/src/tritium_lib/classifier/](../../../../tritium-lib/src/tritium_lib/classifier/) — Shared DeviceClassifier used by BLE classification
- [../../../../tritium-lib/src/tritium_lib/geo/](../../../../tritium-lib/src/tritium_lib/geo/) — Shared coordinate transform library
- [../../../plugins/edge_tracker/](../../../plugins/edge_tracker/) — Edge tracker plugin that pushes BLE/WiFi data here
