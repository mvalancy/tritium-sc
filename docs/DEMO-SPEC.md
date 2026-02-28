# TRITIUM-SC Demo Servers, Device Control, Map Layers & Real Combat

## 1. Purpose

This spec defines four systems that make TRITIUM-SC operational with real or
simulated hardware:

1. **Standalone demo servers** — independent processes that simulate cameras,
   robots, motion sensors, and mesh radios, speaking the same open protocols
   as real hardware.
2. **Device control modals** — a standard frontend interface for clicking any
   device on the map and controlling it through a type-specific panel.
3. **Map data layer system** — import/export/stream standard geospatial
   formats: KML/KMZ, CoT XML, GeoJSON, GPX.
4. **Real combat bridge** — routes simulation combat commands to physical
   hardware via MQTT, with dual-authority (sim + hardware) feedback.

---

## 2. Open Protocols & Formats

Every interface in this system speaks an open standard. No proprietary
protocols.

| Domain | Protocol | Standard | Reference |
|--------|----------|----------|-----------|
| Device messaging | MQTT 3.1.1 | OASIS | [mqtt.org/mqtt-specification](https://mqtt.org/mqtt-specification/) |
| Camera discovery | ONVIF Profile S | ONVIF | [onvif.org/profiles](https://www.onvif.org/profiles/) |
| Camera streaming | RTSP 2.0 | RFC 7826 | IETF RFC 7826 |
| Camera HTTP | MJPEG over HTTP | RFC 2046 multipart | IETF RFC 2046 |
| Target tracking | CoT XML | MIL-STD-2045 | [git.tak.gov/standards/cot](https://git.tak.gov/standards/cot) |
| Map overlays | KML 2.3 | OGC 12-007r2 | [ogc.org/standard/kml](https://www.ogc.org/standard/kml/) |
| Map features | GeoJSON | RFC 7946 | IETF RFC 7946 |
| Track files | GPX 1.1 | topografix.com | [topografix.com/gpx](https://www.topografix.com/gpx.asp) |
| Audio codec | Opus | RFC 6716 | IETF RFC 6716 |
| WebSocket | WS | RFC 6455 | IETF RFC 6455 |
| Sensor data | SensorThings | OGC 15-078r6 | [ogc.org/standard/sensorthings](https://www.ogc.org/standard/sensorthings/) |
| Mesh radio | Meshtastic | MIT license | [meshtastic.org/docs/developers](https://meshtastic.org/docs/developers/) |

---

## 3. Demo Servers

Each demo server is a standalone Python process in `demo/`. No dependency on
the main TRITIUM-SC app. Each has its own `requirements.txt`, `README.md`,
`main.py`, and `tests/` directory. Each can run independently with just an
MQTT broker.

### 3.1 Demo Camera Server (`demo/camera-server/`)

**Purpose:** Simulates an IP camera that produces video frames, serves MJPEG
over HTTP, publishes YOLO-style detections via MQTT, and responds to ONVIF
discovery probes.

**Protocols:**
- MJPEG over HTTP (primary — universal, works in browsers)
- MQTT 3.1.1 for detection publishing and command subscription
- ONVIF WS-Discovery for auto-discovery (optional, if `python-onvif-zeep` installed)

**Architecture:**
```
demo/camera-server/
  main.py              # Entry point, arg parsing, server startup
  camera.py            # Frame generation (procedural or from video file)
  mjpeg_server.py      # HTTP MJPEG streaming endpoint
  detector.py          # Fake YOLO detection generator
  mqtt_publisher.py    # MQTT client for detections + status
  onvif_responder.py   # ONVIF GetCapabilities/GetProfiles stub
  config.py            # Pydantic settings
  requirements.txt     # paho-mqtt, Pillow (no OpenCV required)
  tests/
    test_camera.py
    test_mjpeg.py
    test_detector.py
    test_mqtt.py
```

**Interfaces:**

| Endpoint / Topic | Direction | Format |
|------------------|-----------|--------|
| `GET /mjpeg` | OUT (HTTP) | `multipart/x-mixed-replace` JPEG stream |
| `GET /snapshot` | OUT (HTTP) | Single JPEG frame |
| `GET /status` | OUT (HTTP) | JSON `{"camera_id", "fps", "resolution", "uptime"}` |
| `tritium/{site}/cameras/{id}/detections` | OUT (MQTT QoS 0) | JSON detection array |
| `tritium/{site}/cameras/{id}/status` | OUT (MQTT QoS 1, retained) | JSON `{"status": "online"/"offline"}` |
| `tritium/{site}/cameras/{id}/command` | IN (MQTT QoS 1) | JSON `{"command": "camera_on"/"camera_off"/"set_fps"}` |

**Detection format (TRITIUM standard):**
```json
{
  "boxes": [
    {
      "label": "person",
      "confidence": 0.87,
      "center_x": 0.5,
      "center_y": 0.6,
      "bbox": [0.3, 0.2, 0.7, 0.8]
    }
  ],
  "camera_id": "demo-cam-01",
  "timestamp": "2026-02-27T12:00:00Z",
  "frame_id": 1234
}
```

**Frame generation modes:**
1. `--mode procedural` — renders colored rectangles on dark background (no deps)
2. `--mode video --file path/to/mp4` — loops a video file as camera feed
3. `--mode noise` — random noise (stress test)

**CLI:**
```bash
cd demo/camera-server
pip install -r requirements.txt
python main.py --camera-id demo-cam-01 --port 8081 \
    --mqtt-host localhost --mqtt-port 1883 --site home \
    --fps 10 --width 640 --height 480 --mode procedural
```

### 3.2 Demo Robot Server (`demo/robot-server/`)

**Purpose:** Simulates a mobile robot (rover, drone, or turret) that publishes
MQTT telemetry, receives dispatch/patrol/recall commands, and simulates
physics-based movement.

**Protocols:**
- MQTT 3.1.1 for all communication
- CoT XML for TAK interoperability (optional output)

**Architecture:**
```
demo/robot-server/
  main.py              # Entry point, arg parsing
  robot.py             # Robot state machine (idle, moving, patrolling, engaging)
  physics.py           # Differential drive kinematics, battery, thermal
  mqtt_client.py       # MQTT pub/sub (telemetry, commands, ACKs, thoughts)
  cot_publisher.py     # Optional CoT XML output for TAK
  config.py            # Pydantic settings
  requirements.txt     # paho-mqtt, pyyaml
  tests/
    test_robot.py
    test_physics.py
    test_mqtt.py
    test_commands.py
```

**MQTT Topics:**

| Topic | Direction | QoS | Format |
|-------|-----------|-----|--------|
| `tritium/{site}/robots/{id}/telemetry` | OUT | 0 | JSON telemetry (see below) |
| `tritium/{site}/robots/{id}/status` | OUT | 1 (retained) | `{"status": "online"/"offline"}` |
| `tritium/{site}/robots/{id}/command` | IN | 1 | JSON command |
| `tritium/{site}/robots/{id}/command/ack` | OUT | 1 | JSON acknowledgement |
| `tritium/{site}/robots/{id}/thoughts` | OUT | 0 | JSON thought text |

**Telemetry payload (TRITIUM standard):**
```json
{
  "name": "Demo Rover Alpha",
  "asset_type": "rover",
  "position": {"x": 12.5, "y": -8.3},
  "heading": 135.0,
  "speed": 2.1,
  "battery": 0.85,
  "status": "active",
  "turret": {"pan": 0.0, "tilt": 0.0},
  "timestamp": "2026-02-27T12:00:00Z",
  "battery_state": {
    "charge_pct": 0.85,
    "voltage": 12.3,
    "current_draw": 1.8,
    "temperature_c": 28.5
  },
  "imu": {
    "roll": 0.5,
    "pitch": -1.2,
    "yaw": 135.0,
    "accel_x": 0.1,
    "accel_y": 2.8,
    "accel_z": 9.81
  },
  "motor_temps": {"left": 32.1, "right": 31.8},
  "odometry": {"total_distance": 452.3}
}
```

**Command format:**
```json
{"command": "dispatch", "x": 50.0, "y": 100.0, "timestamp": "2026-02-27T12:00:00Z"}
{"command": "patrol", "waypoints": [{"x": 10, "y": 10}, {"x": 20, "y": 20}], "timestamp": "..."}
{"command": "recall", "timestamp": "..."}
{"command": "fire", "target_x": 30.0, "target_y": 40.0, "timestamp": "..."}
{"command": "aim", "pan": 45.0, "tilt": -10.0, "timestamp": "..."}
```

**ACK format:**
```json
{
  "command": "dispatch",
  "command_timestamp": "2026-02-27T12:00:00Z",
  "status": "accepted",
  "robot_id": "demo-rover-01",
  "timestamp": "2026-02-27T12:00:00.050Z"
}
```

**Physics model:**
- Differential drive: `turn = (left - right) / track_width`, `forward = (left + right) / 2`
- Max speed: configurable per asset_type (rover: 3.0 m/s, drone: 6.0 m/s)
- Battery: LiPo curve (12.6V full → 9.0V empty), drain proportional to motor load
- Motor thermals: warm under load (0.5 C/s), cool idle (0.2 C/s), ambient 25 C
- IMU: derived from dynamics (pitch on accel, roll on turn, yaw = heading)

**Robot types:**
| Type | Speed | Weapon | Movement |
|------|-------|--------|----------|
| `rover` | 3.0 m/s | nerf_blaster (40m range) | ground, differential drive |
| `drone` | 6.0 m/s | drone_gun (50m range) | flying, direct path |
| `turret` | 0.0 m/s | turret_cannon (80m range) | stationary, PTZ only |
| `tank` | 3.0 m/s | tank_main_gun (100m range) | ground, tracks |

**CLI:**
```bash
cd demo/robot-server
python main.py --robot-id demo-rover-01 --type rover --name "Rover Alpha" \
    --mqtt-host localhost --mqtt-port 1883 --site home \
    --start-x 0 --start-y 0 --telemetry-interval 0.5
```

### 3.3 Demo Motion Sensor (`demo/motion-sensor/`)

**Purpose:** Simulates a PIR, microwave, or acoustic motion sensor that
publishes detection events via MQTT.

**Architecture:**
```
demo/motion-sensor/
  main.py              # Entry point
  sensor.py            # Detection state machine (idle, triggered, cooldown)
  patterns.py          # Trigger pattern generators (random, scheduled, burst)
  mqtt_client.py       # MQTT publisher
  config.py            # Settings
  requirements.txt     # paho-mqtt
  tests/
    test_sensor.py
    test_patterns.py
    test_mqtt.py
```

**MQTT Topics:**

| Topic | Direction | QoS | Format |
|-------|-----------|-----|--------|
| `tritium/{site}/sensors/{id}/events` | OUT | 1 | JSON detection event |
| `tritium/{site}/sensors/{id}/status` | OUT | 1 (retained) | JSON status |
| `tritium/{site}/sensors/{id}/command` | IN | 1 | JSON enable/disable |

**Detection event format:**
```json
{
  "sensor_id": "demo-pir-01",
  "sensor_type": "pir",
  "event": "motion_detected",
  "confidence": 0.92,
  "position": {"x": 25.0, "y": -10.0},
  "zone": "front_door",
  "timestamp": "2026-02-27T12:00:00Z",
  "metadata": {
    "duration_ms": 2500,
    "peak_amplitude": 0.87
  }
}
```

**Sensor types:**
| Type | Behavior | Trigger Rate |
|------|----------|-------------|
| `pir` | Binary motion (on/off), 5s cooldown | 0.1-2 Hz |
| `microwave` | Doppler velocity, continuous | 1-10 Hz |
| `acoustic` | Sound level threshold | Event-driven |
| `tripwire` | Line-crossing (enter/exit) | Event-driven |

**CLI:**
```bash
cd demo/motion-sensor
python main.py --sensor-id demo-pir-01 --type pir --zone front_door \
    --mqtt-host localhost --mqtt-port 1883 --site home \
    --position-x 25.0 --position-y -10.0 --pattern random --rate 0.5
```

### 3.4 Demo Mesh Radio (`demo/mesh-radio/`)

**Purpose:** Simulates a Meshtastic or MeshCore LoRa mesh radio node that
publishes position and text messages via MQTT.

**Architecture:**
```
demo/mesh-radio/
  main.py              # Entry point
  node.py              # Mesh node state (position, battery, neighbors)
  mesh_sim.py          # Simulate multi-hop mesh (SNR, RSSI, path loss)
  mqtt_client.py       # MQTT publisher
  config.py            # Settings
  requirements.txt     # paho-mqtt
  tests/
    test_node.py
    test_mesh_sim.py
    test_mqtt.py
```

**MQTT Topics (follows existing meshtastic_bridge pattern):**

| Topic | Direction | QoS | Format |
|-------|-----------|-----|--------|
| `tritium/{site}/mesh/{protocol}/{id}/position` | OUT | 0 | JSON lat/lng/alt |
| `tritium/{site}/mesh/{protocol}/{id}/telemetry` | OUT | 0 | JSON battery/voltage/SNR |
| `tritium/{site}/mesh/{protocol}/{id}/text` | OUT/IN | 1 | JSON text message |
| `tritium/{site}/mesh/{protocol}/{id}/status` | OUT | 1 (retained) | JSON online/offline |

Where `{protocol}` is `meshtastic` or `meshcore`.

**Position payload:**
```json
{
  "node_id": "!aabbccdd",
  "long_name": "Hilltop Node",
  "short_name": "HT01",
  "protocol": "meshtastic",
  "position": {
    "lat": 37.7749,
    "lng": -122.4194,
    "alt": 15.0
  },
  "battery": 0.72,
  "voltage": 3.85,
  "snr": 8.5,
  "rssi": -95,
  "hops": 1,
  "hardware": "heltec_v3",
  "timestamp": "2026-02-27T12:00:00Z"
}
```

**CLI:**
```bash
cd demo/mesh-radio
python main.py --node-id "!aabbccdd" --name "Hilltop Node" \
    --protocol meshtastic --lat 37.7749 --lng -122.4194 \
    --mqtt-host localhost --mqtt-port 1883 --site home
```

### 3.5 Demo Fleet Launcher (`demo/fleet-launcher.py`)

**Purpose:** Starts multiple demo servers at once for a complete simulated
environment.

```bash
cd demo
python fleet-launcher.py --scenario neighborhood
# Starts: 4 cameras, 2 rovers, 1 drone, 1 turret, 3 PIR sensors, 2 mesh radios
# All publishing to localhost:1883

python fleet-launcher.py --scenario riot
# Starts: 8 cameras, 4 drones, 3 rovers, 2 tanks, 10 PIR sensors, 5 mesh radios

python fleet-launcher.py --list
# Lists available fleet scenarios
```

**Fleet scenario format (JSON):**
```json
{
  "name": "neighborhood",
  "description": "Small neighborhood patrol with standard coverage",
  "mqtt": {"host": "localhost", "port": 1883},
  "site": "home",
  "devices": [
    {"type": "camera", "id": "cam-front", "port": 8081, "mode": "procedural", "position": [10, 0]},
    {"type": "camera", "id": "cam-back", "port": 8082, "mode": "procedural", "position": [-10, 0]},
    {"type": "robot", "id": "rover-01", "asset_type": "rover", "start": [0, 0]},
    {"type": "robot", "id": "drone-01", "asset_type": "drone", "start": [5, 5]},
    {"type": "sensor", "id": "pir-front", "sensor_type": "pir", "position": [10, 0], "zone": "front"},
    {"type": "mesh", "id": "!aabb0001", "protocol": "meshtastic", "lat": 37.7749, "lng": -122.4194}
  ]
}
```

---

## 4. Device Control Modal System

### 4.1 Architecture

A unified modal manager that opens type-specific control panels when any
device is clicked on the map. Every device type follows the same interface
contract.

**Frontend files:**
```
frontend/js/command/
  device-modal.js      # DeviceModalManager — creates/shows/hides modals
  device-controls/
    index.js           # Registry of control panels by device type
    camera-control.js  # PTZ, snapshot, stream controls
    robot-control.js   # Dispatch, patrol, recall, fire, aim
    sensor-control.js  # Enable/disable, sensitivity, zone config
    mesh-control.js    # Send text, view neighbors, signal strength
    turret-control.js  # Pan/tilt/fire controls
```

### 4.2 DeviceModalManager Interface

```javascript
class DeviceModalManager {
  constructor(store, eventBus) { ... }

  /**
   * Open a device control modal. Called when user clicks a unit on the map.
   * @param {string} deviceId - Target ID from TritiumStore
   * @param {string} deviceType - asset_type from target data
   * @param {object} position - Screen position {x, y} for modal placement
   */
  open(deviceId, deviceType, position) { ... }

  /** Close the currently open modal */
  close() { ... }

  /** Check if a modal is currently open */
  isOpen() { ... }
}
```

### 4.3 Control Panel Interface (DeviceControl contract)

Every device type implements this interface:

```javascript
/**
 * @typedef {Object} DeviceControl
 * @property {string} type - Device type this handles (e.g., "rover", "camera")
 * @property {string} title - Human-readable title for modal header
 * @property {function} render - Returns HTML string for modal body
 * @property {function} bind - Attaches event listeners to rendered DOM
 * @property {function} update - Called when device data changes (live updates)
 * @property {function} destroy - Cleanup (remove listeners, abort fetches)
 */
const DeviceControl = {
  type: 'rover',
  title: 'Robot Control',

  render(device) {
    // Returns HTML string
    return `
      <div class="dc-stats">...</div>
      <div class="dc-actions">...</div>
    `;
  },

  bind(container, device, api) {
    // api.dispatch(x, y), api.patrol(waypoints), api.recall()
    // api.sendCommand(luaString), api.sendMqtt(topic, payload)
  },

  update(container, device) {
    // Live telemetry updates
  },

  destroy(container) {
    // Cleanup
  }
};
```

### 4.4 Control Panel Specs by Device Type

**Camera Control (`camera-control.js`):**
| Section | Controls |
|---------|----------|
| Status | FPS, resolution, frame count, connection state |
| Stream | Inline MJPEG preview (200x150), fullscreen button |
| PTZ | Pan left/right, tilt up/down, zoom in/out (if `has_ptz`) |
| Actions | Snapshot, toggle recording, night mode |
| Commands | `camera_on`, `camera_off`, `set_fps(n)` via MQTT |

**Robot Control (`robot-control.js`):**
| Section | Controls |
|---------|----------|
| Status | Position, heading, speed, battery, FSM state, motor temps |
| Health | Health bar, degradation indicator, morale bar |
| Movement | Dispatch (click map), Patrol (set waypoints), Recall |
| Turret | Pan/tilt sliders, Fire button (if armed) |
| Actions | Stop, RTB (return to base), custom Lua command input |
| Thoughts | Last robot thought (from MQTT thoughts topic) |

**Sensor Control (`sensor-control.js`):**
| Section | Controls |
|---------|----------|
| Status | Online/offline, last triggered, trigger count |
| Config | Sensitivity slider (0-100%), cooldown (1-30s) |
| Zone | Zone name, position readout |
| Actions | Enable/disable toggle, test trigger button |
| History | Last 10 detection events with timestamps |

**Mesh Radio Control (`mesh-control.js`):**
| Section | Controls |
|---------|----------|
| Status | Battery, voltage, SNR, RSSI, hops, hardware |
| Position | Lat/lng/alt readout, center-on-map button |
| Neighbors | List of heard nodes with signal strength |
| Text | Send text message input (228 char max for LoRa) |
| Channel | Channel selector (if supported) |

**Turret Control (`turret-control.js`):**
| Section | Controls |
|---------|----------|
| Status | Health, ammo, weapon range, cooldown timer |
| Targeting | Current target ID, bearing to target |
| Turret | Pan slider (-180 to +180), tilt slider (-30 to +90) |
| Fire | Fire button (with cooldown indicator), burst mode toggle |
| Actions | Manual/auto toggle, select target from list |

### 4.5 API Contract (Frontend → Backend)

All device commands go through existing endpoints:

| Action | Endpoint | Payload |
|--------|----------|---------|
| Dispatch robot | `POST /api/amy/simulation/dispatch` | `{unit_id, target: {x, y}}` |
| Lua command | `POST /api/amy/command` | `{action: "lua_string", target_id}` |
| Camera command | via MQTT `tritium/{site}/cameras/{id}/command` | `{command, ...params}` |
| Sensor command | via MQTT `tritium/{site}/sensors/{id}/command` | `{command, ...params}` |
| Mesh text | via MQTT `tritium/{site}/mesh/{proto}/{id}/text` | `{text, to}` |

For commands that need MQTT, the frontend sends a REST request to a new
thin proxy endpoint:

```
POST /api/devices/{id}/command
Body: {"topic_suffix": "command", "payload": {...}}
```

This endpoint publishes the payload to the appropriate MQTT topic. The
frontend never connects directly to MQTT.

### 4.6 CSS

Uses the existing `.cc-modal-overlay` + `.cc-modal` system from
`frontend/css/command.css` (lines 867-978). Device-specific styles use a
`.dc-` prefix (device control):

```css
.dc-stats { ... }        /* Status readout grid */
.dc-actions { ... }      /* Action button row */
.dc-slider { ... }       /* PTZ/sensitivity sliders */
.dc-stream { ... }       /* MJPEG preview container */
.dc-health-bar { ... }   /* Health/battery bars */
.dc-cmd-input { ... }    /* Lua command input */
```

### 4.7 Map Integration

File: `frontend/js/command/map.js`

**Change:** When a unit is clicked (existing `_hitTestUnit()` → `unit:selected`
event), also open the device control modal:

```javascript
// In _onMouseDown handler, after unit selection:
if (hitId) {
  const target = TritiumStore.units.get(hitId);
  if (target) {
    DeviceModalManager.open(hitId, target.asset_type, {x: screenX, y: screenY});
  }
}
```

**Double-click:** Opens modal (single click still just selects).

---

## 5. Map Data Layer System

### 5.1 Supported Formats

| Format | Import | Export | Stream | Use Case |
|--------|--------|--------|--------|----------|
| **KML 2.3** | Yes | Yes | No | Google Earth overlays, zone definitions |
| **KMZ** | Yes | Yes | No | Compressed KML with embedded resources |
| **GeoJSON** | Yes | Yes | Yes (WS) | Zone features, building footprints, POIs |
| **GPX 1.1** | Yes | Yes | No | Historical tracks, waypoint files |
| **CoT XML** | Yes | Yes | Yes (TCP/MQTT) | TAK interoperability, live target feeds |
| **CSV** | Yes | No | No | Bulk coordinate import |

### 5.2 Backend Layer Manager

Create `src/engine/layers/`:
```
src/engine/layers/
  __init__.py
  manager.py           # LayerManager — registry of active layers
  layer.py             # Layer base class
  parsers/
    __init__.py
    kml.py             # KML/KMZ → Layer (uses xml.etree, no deps)
    geojson.py         # GeoJSON → Layer (stdlib json)
    gpx.py             # GPX → Layer (xml.etree)
    cot.py             # CoT XML → Layer (uses existing cot.py parser)
    csv_import.py      # CSV lat/lng → point Layer
  exporters/
    __init__.py
    kml.py             # Layer → KML XML
    geojson.py         # Layer → GeoJSON dict
    gpx.py             # Layer → GPX XML
    cot.py             # Layer → CoT XML
  streams/
    __init__.py
    cot_stream.py      # TCP CoT ingest (TAK server feed)
    geojson_ws.py      # WebSocket GeoJSON streaming
```

### 5.3 Layer Data Model

```python
@dataclass
class LayerFeature:
    """A single feature (point, line, polygon) within a layer."""
    feature_id: str
    geometry_type: str         # "Point", "LineString", "Polygon"
    coordinates: list          # GeoJSON-style coordinate arrays
    properties: dict           # Arbitrary key-value metadata
    style: dict | None = None  # Color, icon, line width, opacity
    timestamp: str | None = None  # ISO8601 for time-series data

@dataclass
class Layer:
    """A named collection of geographic features."""
    layer_id: str
    name: str
    source_format: str         # "kml", "geojson", "gpx", "cot", "csv"
    features: list[LayerFeature]
    visible: bool = True
    opacity: float = 1.0
    z_index: int = 0           # Draw order
    metadata: dict = field(default_factory=dict)
    created_at: str = ""       # ISO8601
    updated_at: str = ""       # ISO8601

class LayerManager:
    """Registry of active map layers."""

    def add_layer(self, layer: Layer) -> str: ...
    def remove_layer(self, layer_id: str) -> bool: ...
    def get_layer(self, layer_id: str) -> Layer | None: ...
    def list_layers(self) -> list[Layer]: ...
    def import_file(self, path: str, format: str = "auto") -> Layer: ...
    def export_layer(self, layer_id: str, format: str) -> str | bytes: ...
    def set_visibility(self, layer_id: str, visible: bool) -> None: ...
```

### 5.4 API Endpoints

```
# Layer management
GET    /api/layers                          → List all layers
POST   /api/layers                          → Import layer from uploaded file
GET    /api/layers/{id}                     → Get layer details + features
DELETE /api/layers/{id}                     → Remove layer
PUT    /api/layers/{id}/visibility          → Toggle visibility
GET    /api/layers/{id}/export?format=kml   → Export layer as KML/GeoJSON/GPX

# Streaming
POST   /api/layers/stream/cot              → Start CoT TCP ingest
WS     /ws/layers                          → GeoJSON feature streaming
```

**File upload (multipart):**
```
POST /api/layers
Content-Type: multipart/form-data
  file: overlay.kml
  name: "Patrol Routes" (optional)
```

### 5.5 Frontend Layer Renderer

File: `frontend/js/command/map-layers.js`

```javascript
class MapLayerRenderer {
  constructor(canvasCtx, store) { ... }

  /**
   * Draw all visible layers on the tactical map canvas.
   * Called during the map render loop after units, before UI.
   */
  drawLayers(ctx, transform) {
    for (const layer of this.layers) {
      if (!layer.visible) continue;
      ctx.globalAlpha = layer.opacity;
      for (const feature of layer.features) {
        switch (feature.geometry_type) {
          case 'Point': this._drawPoint(ctx, feature, transform); break;
          case 'LineString': this._drawLine(ctx, feature, transform); break;
          case 'Polygon': this._drawPolygon(ctx, feature, transform); break;
        }
      }
    }
  }
}
```

**Layer panel in menu bar:**
File: `frontend/js/command/menu-bar.js`

Add "LAYERS" menu with:
- List of loaded layers (checkboxes for visibility)
- "Import File..." button (opens file picker)
- "Export..." submenu per layer (KML, GeoJSON, GPX)
- Opacity slider per layer

### 5.6 KML Feature Mapping

| KML Element | → Layer Feature |
|-------------|-----------------|
| `<Placemark><Point>` | Point feature |
| `<Placemark><LineString>` | LineString feature |
| `<Placemark><Polygon>` | Polygon feature |
| `<Placemark><name>` | `properties.name` |
| `<Placemark><description>` | `properties.description` |
| `<Style><IconStyle><color>` | `style.color` |
| `<Style><LineStyle><width>` | `style.lineWidth` |
| `<Style><PolyStyle><color>` | `style.fillColor` |
| `<GroundOverlay>` | Image overlay (raster tile) |
| `<NetworkLink>` | Streaming layer (poll URL) |

### 5.7 CoT Streaming Integration

Extends existing `src/engine/comms/cot.py` and `tak_bridge.py`:

```python
class CoTLayerStream:
    """Ingest live CoT events from a TAK server or MQTT and render as a layer."""

    def __init__(self, layer_manager: LayerManager, event_bus: EventBus):
        self._layer = Layer(layer_id="cot-live", name="TAK Live Feed", source_format="cot")
        layer_manager.add_layer(self._layer)

    def on_cot_event(self, cot_xml: str) -> None:
        """Parse CoT XML → update/add feature in the live layer."""
        # Uses existing cot.py parse_event()
        # Stale timeout: remove features not updated in 5 minutes
```

### 5.8 GPX Track Playback

For historical track files, support time-based playback:

```python
class GPXPlayer:
    """Play back a GPX track at configurable speed."""

    def __init__(self, gpx_layer: Layer, speed: float = 1.0):
        self._speed = speed  # 1.0 = real time, 10.0 = 10x
        self._position_index = 0

    def tick(self, dt: float) -> LayerFeature | None:
        """Advance playback, return current position feature."""
```

---

## 6. Real Combat Bridge

### 6.1 Concept

The simulation engine continues to run as the authoritative game state. When
a simulated unit fires, the combat bridge also sends the corresponding MQTT
command to the physical robot. Hardware feedback (ACK, actual fire
confirmation) is recorded but does not override the simulation score.

**Dual authority model:**
```
Simulation:   combat.fire(turret, hostile) → projectile → hit → score
Hardware:     MQTT publish: fire(pan=45, tilt=-10) → robot ACK
              ↓
              Hardware result logged alongside simulation result
```

The operator sees both: "Simulation says hit. Hardware says fired and
turret confirmed aim." Over time, correlation between sim and hardware
builds confidence in the model.

### 6.2 CombatBridge class

Create `src/engine/simulation/combat_bridge.py`:

```python
class CombatBridge:
    """Routes simulation combat actions to physical hardware via MQTT."""

    def __init__(self, mqtt_bridge, event_bus):
        self._mqtt = mqtt_bridge
        self._event_bus = event_bus
        self._hardware_units: dict[str, str] = {}  # sim_id → mqtt_robot_id

    def bind_unit(self, sim_target_id: str, mqtt_robot_id: str) -> None:
        """Bind a simulation target to a physical robot."""

    def on_fire(self, event: dict) -> None:
        """Called when simulation fires a projectile.
        Sends aim + fire commands to bound hardware unit."""

    def on_dispatch(self, event: dict) -> None:
        """Called when simulation dispatches a unit.
        Sends dispatch command to bound hardware unit."""
```

### 6.3 Hardware Command Mapping

| Sim Event | MQTT Command | Topic |
|-----------|-------------|-------|
| `projectile_fired` | `{"command": "aim", "pan": deg, "tilt": deg}` then `{"command": "fire"}` | `robots/{id}/command` |
| `unit_dispatched` | `{"command": "dispatch", "x": x, "y": y}` | `robots/{id}/command` |
| `unit_patrol` | `{"command": "patrol", "waypoints": [...]}` | `robots/{id}/command` |
| `unit_recall` | `{"command": "recall"}` | `robots/{id}/command` |
| `unit_stop` | `{"command": "stop"}` | `robots/{id}/command` |

### 6.4 Binding API

```
POST /api/combat/bind
Body: {"sim_target_id": "turret-1", "mqtt_robot_id": "real-turret-01"}

GET /api/combat/bindings
→ [{"sim_id": "turret-1", "mqtt_id": "real-turret-01", "status": "connected"}]

DELETE /api/combat/bind/{sim_target_id}
```

### 6.5 Frontend Integration

In the device control modal for a bound unit, show:
- "HARDWARE BOUND" indicator with green dot
- Real robot telemetry alongside simulation state
- Manual override buttons (fire, aim, stop)
- Correlation log: "Sim: fired at 12:00:00.100 | HW: ACK at 12:00:00.200"

---

## 7. Dependency Graph

```
Spec (this document)
  ├── Demo Camera Server ──────────────── independent
  ├── Demo Robot Server ───────────────── independent
  ├── Demo Motion Sensor ──────────────── independent
  ├── Demo Mesh Radio ─────────────────── independent
  ├── Demo Fleet Launcher ─────────────── depends on all demo servers
  ├── Device Control Modals ───────────── independent (frontend only)
  ├── Map Data Layers ─────────────────── independent
  └── Combat Bridge ───────────────────── depends on demo robot server (for testing)
```

Most work is independent and can run in parallel.

## 8. Testing Strategy

Every component follows TDD:

1. Write test file with failing tests
2. Run tests, verify they fail for the right reasons
3. Implement the code
4. Run tests, verify they pass
5. Run `./test.sh fast` to verify no regressions

**Test locations:**
| Component | Test Location | Count (est) |
|-----------|--------------|-------------|
| Demo camera | `demo/camera-server/tests/` | 15 |
| Demo robot | `demo/robot-server/tests/` | 20 |
| Demo sensor | `demo/motion-sensor/tests/` | 10 |
| Demo mesh | `demo/mesh-radio/tests/` | 12 |
| Fleet launcher | `demo/tests/test_fleet.py` | 5 |
| Device modals | `tests/js/test_device_modal.js` | 15 |
| Map layers | `tests/engine/layers/` | 25 |
| Combat bridge | `tests/engine/simulation/test_combat_bridge.py` | 15 |
| **Total** | | **~117** |
