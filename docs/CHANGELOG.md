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

## 2026-03-13

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
| Unit tests (tier 2) | ~7600 | All passing | 2026-03-13 |
| JS tests (tier 3) | 281+ | All passing | 2026-03-13 |
| ROS2 tests (tier 8b) | 125 | All passing | 2026-03-13 |

## Known Issues

| Issue | Status | Impact |
|-------|--------|--------|
| 18 tests in `test_websocket.py` fail | Pre-existing | Missing asyncio event loop |
| No real cameras connected | Expected | Using synthetic feeds for testing |
| Meshtastic disabled by default | Expected | Set `MESHTASTIC_ENABLED=true` to activate |
