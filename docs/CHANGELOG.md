# Tritium-SC Changelog

Changes tracked with verification status. All changes on `dev` branch.

## Verification Levels

| Level | Meaning |
|-------|---------|
| **Unit Tested** | Passes `@pytest.mark.unit` tests |
| **Integration Tested** | Passes `./test.sh fast` (tiers 1-3 + 8) |
| **Visual Tested** | Passes `./test.sh 10` or manual browser check |
| **Human Verified** | Manually tested by a human in browser |
| **Deployed** | Running on production server |

---

## 2026-03-13 — Wave 8: Smart Correlation, Dossiers, TAK Bridge & Fusion Demo

### Smart Correlator — Multi-Factor Identity Resolution
| Change | Verification |
|--------|-------------|
| `CorrelationStrategy` ABC with pluggable strategy pattern | Integration Tested |
| `SpatialStrategy` — distance-based proximity scoring | Integration Tested |
| `TemporalStrategy` — co-movement detection from position history | Integration Tested |
| `SignalPatternStrategy` — appearance/disappearance timing correlation | Integration Tested |
| `DossierStrategy` — known prior associations from DossierStore | Integration Tested |
| `TargetCorrelator` — weighted multi-strategy fusion engine | Integration Tested |

### DossierManager — Persistent Entity Intelligence
| Change | Verification |
|--------|-------------|
| `DossierManager` bridges TargetTracker (real-time) and DossierStore (persistent) | Integration Tested |
| EventBus subscriptions: correlation, BLE new device, detections, enrichment | Integration Tested |
| Periodic flush (30s) persists dirty dossiers to SQLite store | Integration Tested |
| Thread-safe public API for cross-thread access | Integration Tested |

### Dossier Panel — Frontend Entity Browser
| Change | Verification |
|--------|-------------|
| Split-view panel: dossier list (left) + detail view (right) | Integration Tested |
| Filtering, sorting, tags, notes, merge support | Integration Tested |
| Position trail mini-map in detail view | Integration Tested |
| Fetches from `/api/dossiers`, `/api/dossiers/search`, `/api/dossiers/{id}` | Integration Tested |

### TAK Bridge — Team Awareness Kit Integration
| Change | Verification |
|--------|-------------|
| `TAKBridge` — EventBus subscriber bridging TAK server protocol | Integration Tested |
| CoT XML translation (targets, geochat, spot reports, emergencies, video feeds) | Integration Tested |
| pytak async event loop in daemon thread with graceful degradation | Integration Tested |
| Publish loop reads TargetTracker and queues CoT XML to TAK server | Integration Tested |

### Fusion Demo — Multi-Sensor Correlation Showcase
| Change | Verification |
|--------|-------------|
| `FusionScenario` — scripted multi-sensor actors for demo mode | Integration Tested |
| Person A (phone + watch + camera) patrol path fusion | Integration Tested |
| Vehicle B (driver phone BLE + camera detection) road path fusion | Integration Tested |
| Person C geofence intrusion alert trigger | Integration Tested |
| Injects BLE sightings and YOLO detections into TargetTracker for correlator merge | Integration Tested |

---

## 2026-03-13 — Wave 7: Data Providers, Dossiers, Enrichment & GIS Layers

### DataProvider Plugin Architecture
| Change | Verification |
|--------|-------------|
| `DataProviderPlugin` interface for modular data sources | Integration Tested |
| `LayerRegistry` for managing and composing data layers | Integration Tested |

### Target Enrichment Pipeline
| Change | Verification |
|--------|-------------|
| OUI manufacturer lookup enrichment provider | Integration Tested |
| WiFi network association enrichment provider | Integration Tested |
| BLE device type enrichment provider | Integration Tested |
| Composable enrichment pipeline with provider chaining | Integration Tested |

### GIS Layers Plugin
| Change | Verification |
|--------|-------------|
| OSM tile provider for street-level map data | Integration Tested |
| Satellite imagery provider | Integration Tested |
| Building footprint provider | Integration Tested |
| Terrain/elevation provider | Integration Tested |
| Layer toggle and opacity controls | Integration Tested |

---

## 2026-03-13 — Wave 6: Tracking, Geofencing, Persistence & Search

### Target Correlation Engine
| Change | Verification |
|--------|-------------|
| Multi-sensor target correlation (BLE + camera + mesh fusion) | Integration Tested |
| Correlation scoring with configurable thresholds | Integration Tested |

### BLE Threat Classification
| Change | Verification |
|--------|-------------|
| Threat level classification: known/unknown/new/suspicious | Integration Tested |
| Configurable threat rules based on device history and behavior | Integration Tested |

### YOLO Detector Plugin
| Change | Verification |
|--------|-------------|
| Modular YOLO inference pipeline with pluggable backends | Integration Tested |
| Detection → TrackedTarget integration via TargetTracker | Integration Tested |

### Target Position History & Movement Trails
| Change | Verification |
|--------|-------------|
| Position history persistence per tracked target | Integration Tested |
| Movement trail rendering on tactical map | Integration Tested |

### Target Search & Filter API
| Change | Verification |
|--------|-------------|
| REST API for target search with multi-field filtering | Integration Tested |
| Command panel UI for search and filter interaction | Integration Tested |

### Geofencing Engine
| Change | Verification |
|--------|-------------|
| Zone-based geofencing with enter/exit event detection | Integration Tested |
| Zone management API (CRUD) | Integration Tested |
| Geofence alerts dispatched via EventBus | Integration Tested |
| Map panel for zone visualization and editing | Integration Tested |

---

## 2026-03-13 — Wave 4: Full Pipeline & Fleet Dashboard

### Synthetic Data Generators
| Change | Verification |
|--------|-------------|
| BLE device data generator (randomized MACs, RSSI, device types) | Integration Tested |
| Meshtastic node data generator (GPS, battery, SNR) | Integration Tested |
| Camera pipeline data generator (synthetic detections, bounding boxes) | Integration Tested |

### WebSocket Broadcast
| Change | Verification |
|--------|-------------|
| BLE targets broadcast via WebSocket `/ws/live` | Integration Tested |
| Mesh radio targets broadcast via WebSocket `/ws/live` | Integration Tested |

### Frontend — Camera Feeds Panel
| Change | Verification |
|--------|-------------|
| Live camera feeds panel with MJPEG grid layout | Integration Tested |
| Camera status indicators (online/offline/error) | Integration Tested |
| Detection info overlay on camera feeds | Integration Tested |

### Fleet Dashboard Plugin
| Change | Verification |
|--------|-------------|
| `plugins/fleet_dashboard/` — fleet-wide device registry | Integration Tested |
| Device list with status, heartbeat, battery, uptime | Integration Tested |
| REST API at `/api/fleet/*` | Integration Tested |

### Demo Mode
| Change | Verification |
|--------|-------------|
| Demo mode exercising full pipeline with synthetic data | Integration Tested |
| Synthetic BLE, Meshtastic, and camera data flowing through WebSocket | Integration Tested |

---

## 2026-03-13 — Wave 3: Integration & Hardening

### MQTT Integration
| Change | Verification |
|--------|-------------|
| MQTTBridge subscribes to `tritium/+/sighting` for edge BLE/WiFi data | Unit Tested |
| MQTTBridge `_on_edge_sighting()` → EventBus `fleet.ble_presence` / `fleet.wifi_presence` | Unit Tested |
| MQTTBridge subscribes to `cameras/+/frame`, dispatches to camera feeds plugin | Unit Tested |
| MQTTBridge `register_camera_callback()` API for plugin frame delivery | Unit Tested |
| Camera feeds MQTTSource auto-registers with MQTTBridge on startup | Unit Tested |
| Camera detection → TrackedTarget creation via MQTTSource.on_detection() | Unit Tested |

### Frontend — Asset Management
| Change | Verification |
|--------|-------------|
| `panels/assets.js` — asset placement panel (add/edit/delete/drag) | Integration Tested |
| Map placement mode via EventBus `asset:placementMode` / `map:click` | Integration Tested |
| Asset type selector (camera, sensor, mesh_radio, gateway) | Integration Tested |
| Property editor (name, height, FOV, rotation, position) | Integration Tested |
| CRUD via `/api/assets` endpoint | Integration Tested |

### Examples
| Change | Verification |
|--------|-------------|
| `examples/ros2-camera/` — ROS2 camera node with MQTT detection publisher | Unit Tested |
| 53 tests (camera node + MQTT publisher) | Unit Tested |
| Launch file, config YAML, package.xml for ROS2 integration | Unit Tested |

### Test Coverage Expansion
| Change | Verification |
|--------|-------------|
| 14 MQTT camera integration tests | Unit Tested |
| 7 edge sighting MQTT routing tests | Unit Tested |
| Camera feeds source lifecycle, error handling tests | Unit Tested |
| Meshtastic node update, config, degradation tests | Unit Tested |
| BLE tracking MAC normalization, multi-node, rapid update tests | Unit Tested |

---

## 2026-03-13 — Wave 2: Core Plugin Features

### Plugins — New
| Change | Verification |
|--------|-------------|
| `plugins/meshtastic/` — LoRa mesh radio bridge (serial/TCP, GPS, send/waypoint) | Unit Tested |
| Meshtastic routes at `/api/meshtastic/*` (nodes, send, waypoint, status) | Unit Tested |
| Meshtastic route registration tests (5 tests) | Unit Tested |
| `plugins/camera_feeds/` — multi-source camera plugin (synthetic/RTSP/MJPEG/MQTT/USB) | Unit Tested |
| Camera feeds routes at `/api/camera-feeds/*` (list, add, remove, snapshot, stream) | Unit Tested |

### Plugins — Edge Tracker Improvements
| Change | Verification |
|--------|-------------|
| EdgeTrackerPlugin pushes BLE devices to TargetTracker | Unit Tested |
| MQTT sighting subscription in MQTTBridge for `tritium/+/sighting` | Unit Tested |
| BLE data flows via both HTTP heartbeat and MQTT | Unit Tested |

### Tactical — BLE & Mesh Tracking
| Change | Verification |
|--------|-------------|
| `TargetTracker.update_from_ble()` — BLE devices as tracked targets | Unit Tested |
| RSSI → confidence mapping (-30dBm=1.0, -90dBm=0.1) | Unit Tested |
| Trilateration position support | Unit Tested |
| Node proximity fallback positioning | Unit Tested |
| 120s BLE stale timeout | Unit Tested |
| MAC normalization (case-insensitive) | Unit Tested |
| Multiple nodes same device → update not duplicate | Unit Tested |

### Frontend — Map Rendering
| Change | Verification |
|--------|-------------|
| BLE device rendering — cyan dots with bluetooth rune, confidence opacity | Integration Tested |
| Mesh radio rendering — green dots with radio wave arcs | Integration Tested |
| Asset placement panel (`panels/assets.js`) | Integration Tested |
| Unit icon routing for ble_device and mesh_radio asset types | Integration Tested |

### Geo
| Change | Verification |
|--------|-------------|
| `geo.py` latlng_to_local tuple unpacking fix in Meshtastic plugin | Unit Tested |

### Examples
| Change | Verification |
|--------|-------------|
| `examples/ros2-camera/` — ROS2 camera node → MQTT detection publisher | Unit Tested |

### Documentation
| Change | Verification |
|--------|-------------|
| Subsystem READMEs (engine, amy, app, frontend, tests, plugins, examples) | N/A (docs) |
| `docs/README.md` — updated with navigation tree and cross-project links | N/A (docs) |
| `plugins/README.md` — plugin catalog | N/A (docs) |
| `plugins/meshtastic/README.md` — data flow diagram | N/A (docs) |
| `plugins/camera_feeds/README.md` — multi-source architecture | N/A (docs) |

---

## Test Baseline

| Suite | Count | Status | Date |
|-------|-------|--------|------|
| ./test.sh fast | 81 tiers | All passing | 2026-03-13 |
| Unit tests (tier 2) | ~7845 | All passing | 2026-03-13 |
| JS tests (tier 3) | 281+ | All passing | 2026-03-13 |
| ROS2 tests (tier 8b) | 125 | All passing | 2026-03-13 |

## Known Issues

| Issue | Status | Impact |
|-------|--------|--------|
| 18 tests in `test_websocket.py` fail | Pre-existing | Missing asyncio event loop |
| No real cameras connected | Expected | Using synthetic feeds for testing |
| Meshtastic disabled by default | Expected | Set `MESHTASTIC_ENABLED=true` to activate |
