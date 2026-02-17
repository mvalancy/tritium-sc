# TRITIUM-SC System Architecture

## Overview

TRITIUM-SC is a three-layer system: a **browser frontend** renders the tactical picture, a **FastAPI server** hosts Amy's consciousness and the battlespace engine, and **external devices** (cameras, robots, sensors) connect via MQTT or direct USB.

Real sensors and simulated units coexist on the same event bus, same APIs, same tactical map. Amy does not distinguish between them.

```mermaid
graph TB
    subgraph "Browser"
        WAR[War Room Canvas]
        AMY_UI[Amy Dashboard]
        ASSETS[Assets View]
        MAP3D[3D Tactical Map]
    end

    subgraph "FastAPI Server"
        WS[WebSocket Bridge]
        API[REST API]

        subgraph "Amy Commander"
            THINK[Thinking Loop]
            SENSE[Sensorium]
            VISION[Vision Thread]
            LISTEN[Listener - VAD+STT]
            SPEAK[Speaker - TTS]
            MOTOR[Motor Thread]
            MEM[Memory v3]
            AGENT[Chat Agent]
        end

        subgraph "Battlespace"
            SIM[Simulation Engine 10Hz]
            TRACK[Target Tracker]
            CLASS[Threat Classifier 2Hz]
            DISP[Auto Dispatcher]
            AMBIENT[Ambient Spawner]
        end

        BUS[Event Bus]
        MQTT[MQTT Bridge]
    end

    subgraph "External Devices"
        CAM[BCC950 PTZ Camera]
        MIC[USB Microphone]
        ROBOT[MQTT Robots]
        SENSOR[MQTT Sensors]
    end

    WAR -->|WebSocket| WS
    AMY_UI -->|WebSocket| WS
    ASSETS -->|REST| API
    MAP3D -->|WebSocket| WS

    WS --> BUS
    API --> TRACK
    API --> SIM

    THINK --> BUS
    SENSE --> TRACK
    VISION --> BUS
    LISTEN --> THINK
    SPEAK --> CAM
    MOTOR --> CAM

    SIM --> BUS
    CLASS --> TRACK
    DISP --> SIM
    DISP --> MQTT
    AMBIENT --> SIM
    TRACK --> BUS

    MQTT --> ROBOT
    MQTT --> SENSOR
    CAM --> VISION
    MIC --> LISTEN
    BUS --> MQTT
```

## Boot Sequence

The FastAPI lifespan handler in `app/main.py` orchestrates startup:

```mermaid
sequenceDiagram
    participant M as main.py lifespan
    participant DB as Database
    participant NVR as NVR Discovery
    participant SIM as SimulationEngine
    participant AMY as Commander
    participant WS as WebSocket Bridge
    participant MQTT as MQTTBridge
    participant ESC as Escalation System

    M->>DB: init_db()
    M->>NVR: discover_cameras()
    M->>SIM: SimulationEngine(event_bus)
    M->>SIM: load_layout(layout_path)
    M->>AMY: create_amy(settings, sim_engine)
    M->>SIM: Re-wire to Amy's EventBus
    M->>SIM: start() [10Hz tick]
    M->>AMY: Thread(target=run).start()

    Note over AMY: _boot() initializes 8 subsystems

    M->>WS: start_amy_event_bridge(amy, loop)
    M->>MQTT: MQTTBridge(event_bus, tracker)
    M->>MQTT: start()
    M->>ESC: ThreatClassifier(event_bus, tracker, zones)
    M->>ESC: AutoDispatcher(event_bus, tracker, sim, mqtt)
```

### Commander Boot (`_boot()`)

Amy's Commander initializes subsystems in order:

1. **Sensor nodes** -- start BCC950 or virtual nodes
2. **Speaker** -- Piper TTS, pre-cache acknowledgment WAVs
3. **Listener** -- Silero VAD + whisper.cpp GPU STT
4. **YOLO** -- VisionThread with model warmup
5. **Chat agent** -- Ollama LLM (gemma3:4b)
6. **Deep vision** -- Ollama multimodal (llava:7b)
7. **Thinking thread** -- continuous inner monologue
8. **Background threads** -- motor, audio, curiosity timer

After boot, Amy delivers a context-aware greeting, then starts the event loop.

## Event Flow

### Target Lifecycle: Spawn to Render

```mermaid
sequenceDiagram
    participant SP as Hostile Spawner
    participant SIM as SimulationEngine
    participant BUS as EventBus
    participant TT as TargetTracker
    participant TC as ThreatClassifier
    participant AD as AutoDispatcher
    participant WS as WebSocket Bridge
    participant UI as War Room Canvas

    SP->>SIM: spawn_hostile()
    SIM->>SIM: add_target(SimulationTarget)

    loop Every 100ms (10Hz)
        SIM->>SIM: target.tick(0.1)
        SIM->>BUS: publish("sim_telemetry", target.to_dict())
    end

    BUS->>TT: _sim_bridge_loop() -> update_from_simulation()
    BUS->>WS: bridge_loop() -> TelemetryBatcher.add()

    loop Every 100ms
        WS->>UI: broadcast("amy_sim_telemetry_batch", [...])
    end

    UI->>UI: updateSimTarget() in assets.js
    UI->>UI: renderTargets() on canvas

    loop Every 500ms (2Hz)
        TC->>TT: get_all()
        TC->>TC: _find_zone(position)
        TC->>BUS: publish("threat_escalation", ...)
    end

    BUS->>AD: threat_escalation event
    AD->>TT: get_friendlies()
    AD->>SIM: set waypoints on nearest unit
    AD->>BUS: publish("amy_dispatch", ...)
    AD->>MQTT: publish_dispatch()
```

### Escalation Sequence

```mermaid
sequenceDiagram
    participant H as Hostile Target
    participant TC as ThreatClassifier
    participant BUS as EventBus
    participant AD as AutoDispatcher
    participant SIM as SimulationEngine
    participant F as Friendly Unit
    participant MQTT as MQTTBridge

    Note over H: Crosses perimeter zone
    TC->>TC: threat: none -> unknown
    TC->>BUS: publish("zone_violation")
    TC->>BUS: publish("threat_escalation")

    Note over H: Enters restricted zone
    TC->>TC: threat: unknown -> suspicious
    TC->>BUS: publish("threat_escalation")

    BUS->>AD: suspicious escalation
    AD->>AD: Find nearest available friendly
    AD->>SIM: Set waypoints -> hostile position
    AD->>BUS: publish("amy_dispatch")
    AD->>MQTT: publish_dispatch(robot_id, x, y)
    AD->>BUS: publish("auto_dispatch_speech")

    Note over F: Moving to intercept
    F->>F: tick() moves along waypoints

    Note over H: Lingers > 30s
    TC->>TC: threat: suspicious -> hostile
    TC->>BUS: publish("threat_escalation")

    Note over H: Leaves all zones
    Note over H: 30s outside zones
    TC->>TC: threat: hostile -> suspicious (de-escalation)
```

## Data Formats

Position data flows through several representations:

| Layer | Format | Example |
|-------|--------|---------|
| SimulationTarget | `tuple[float, float]` | `(5.2, -3.1)` |
| SimulationTarget.to_dict() | `{"x": float, "y": float}` | `{"x": 5.2, "y": -3.1}` |
| TrackedTarget | `tuple[float, float]` | `(5.2, -3.1)` |
| TrackedTarget.to_dict() | `{"x": float, "y": float}` | `{"x": 5.2, "y": -3.1}` |
| MQTT telemetry | `{"x": float, "y": float}` | `{"x": 5.2, "y": -3.1}` |
| WebSocket to frontend | `{"x": float, "y": float}` | `{"x": 5.2, "y": -3.1}` |
| War Room canvas | pixel coordinates | `worldToScreen(5.2, -3.1)` |
| TritiumLevelFormat JSON | `{"x": float, "z": float}` | `{"x": 5.2, "z": -3.1}` |

Note: The level format uses `z` for the horizontal plane (3D convention where `y` is height). The simulation and tracker use `y` internally.

## Thread Architecture

Amy runs multiple daemon threads coordinated through the EventBus:

| Thread | Rate | Purpose |
|--------|------|---------|
| `amy` (Commander.run) | event-driven | Main event loop |
| `sim-tick` | 10 Hz | Advance all simulation targets |
| `sim-spawner` | 30-120s | Spawn hostile intruders |
| `ambient-spawner` | 15-45s | Spawn neutral neighborhood activity |
| `sim-bridge` | event-driven | Feed sim telemetry to TargetTracker |
| `threat-classifier` | 2 Hz | Classify threats against zones |
| `auto-dispatcher` | event-driven | Dispatch units to intercept threats |
| `thinking` | 8s interval | Amy's inner monologue (LLM) |
| `vision` (VisionThread) | 3 Hz | YOLO detection + perception |
| `audio` (AudioThread) | continuous | VAD speech detection |
| `motor` (MotorThread) | continuous | PTZ camera control |
| `curiosity` | 45-90s | Trigger deep visual observation |
| `amy-ws-bridge` | event-driven | Forward EventBus to WebSocket |
| `telemetry-batcher` | 100ms | Batch sim_telemetry for WebSocket |
| `mqtt` (paho loop) | continuous | MQTT broker communication |

## Key Files

| File | Purpose |
|------|---------|
| `app/main.py` | FastAPI lifespan, boot sequence |
| `amy/commander.py` | Commander, VisionThread, AudioThread |
| `amy/event_bus.py` | EventBus — thread-safe pub/sub for all internal events |
| `amy/thinking.py` | ThinkingThread, GoalStack |
| `amy/sensorium.py` | Temporal sensor fusion, narrative |
| `amy/target_tracker.py` | Unified real+virtual target registry |
| `amy/escalation.py` | ThreatClassifier, AutoDispatcher |
| `amy/mqtt_bridge.py` | MQTT broker bridge |
| `amy/simulation/engine.py` | SimulationEngine (10Hz tick loop) |
| `amy/simulation/target.py` | SimulationTarget dataclass |
| `amy/simulation/ambient.py` | AmbientSpawner (neutral targets) |
| `amy/simulation/loader.py` | TritiumLevelFormat parser |
| `app/routers/ws.py` | WebSocket bridge + TelemetryBatcher |
| `frontend/js/war.js` | War Room canvas renderer |
| `frontend/js/assets.js` | Target state + tactical map |

## Related Documentation

- [PLAN.md](PLAN.md) -- Development roadmap and phase status
- [ESCALATION.md](ESCALATION.md) -- Threat escalation system
- [MQTT.md](MQTT.md) -- MQTT communication protocol
- [SIMULATION.md](SIMULATION.md) -- Simulation engine internals
