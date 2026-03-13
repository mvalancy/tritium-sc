# Engine — System Infrastructure

**Where you are:** `tritium-sc/src/engine/` — the commander-agnostic system infrastructure that powers the Command Center.

**Parent:** [../../CLAUDE.md](../../CLAUDE.md) | [../../../CLAUDE.md](../../../CLAUDE.md) (tritium root)

## What This Is

The engine is the reusable foundation that any commander (Amy, or a future replacement) plugs into. It handles simulation, communication, perception, tactical reasoning, and hardware abstraction. The `CommanderProtocol` in `commander_protocol.py` defines the interface.

## Subsystems

```
engine/
├── commander_protocol.py     # Interface for swappable commanders
├── simulation/               # 10Hz battle simulation engine
├── comms/                    # Communication: EventBus, MQTT, CoT, TTS, STT
├── tactical/                 # Geo, threat classification, target tracking
├── inference/                # LLM model routing and Ollama fleet discovery
├── perception/               # Frame analysis, vision, fact extraction
├── actions/                  # Lua dispatch, formations, motor commands
├── nodes/                    # Sensor node abstraction (cameras, PTZ, mics)
├── units/                    # 16 unit types (robots, people, sensors)
├── audio/                    # Sound effects library and audio pipeline
├── synthetic/                # Procedural media generation
├── scenarios/                # Behavioral test framework
├── layers/                   # Map/GIS layer system
├── layouts/                  # Level format JSON files
└── plugins/                  # Plugin loader and registration
```

### simulation/ — Battle Simulation

The 10Hz tick loop that runs combat, AI behaviors, and wave progression. Used for both game mode and continuous monitoring exercises.

| File | Purpose |
|------|---------|
| `engine.py` | `SimulationEngine` — main tick loop, hostile spawner |
| `target.py` | `SimulationTarget` dataclass (all entity types) |
| `combat.py` | `CombatSystem` — projectile flight, hit detection, damage |
| `game_mode.py` | `GameMode` — 10-wave progression, scoring |
| `behaviors.py` | `UnitBehaviors` — turret, drone, rover, hostile AI |
| `ambient.py` | `AmbientSpawner` — neutral civilian activity |
| `loader.py` | `TritiumLevelFormat` JSON parser |
| `behavior/` | Behavior trees and unit AI state machines |
| `npc_intelligence/` | NPC personality, cognition, crowd dynamics |

### comms/ — Communication Layer

Thread-safe pub/sub, MQTT bridge, audio I/O, and protocol codecs.

| File | Purpose |
|------|---------|
| `event_bus.py` | `EventBus` — thread-safe pub/sub for all internal events |
| `mqtt_bridge.py` | MQTT broker bridge for distributed device mesh |
| `listener.py` | Audio VAD + recording (Silero VAD) |
| `speaker.py` | TTS output (Piper) |
| `cot.py` | Cursor on Target XML codec |
| `mqtt_cot.py` | MQTT-based CoT transport |
| `tak_bridge.py` | TAK/ATAK server bridge |

### tactical/ — Situational Awareness

Geo transforms, threat classification, and the unified target registry.

| File | Purpose |
|------|---------|
| `target_tracker.py` | Unified registry of real (YOLO) + virtual (sim) targets |
| `escalation.py` | `ThreatClassifier` (2Hz) + `AutoDispatcher` |
| `geo.py` | Server-side geo-reference and coordinate transforms |
| `street_graph.py` | OSM road extraction + A* pathfinding |
| `obstacles.py` | Building obstacles for pathfinding |

### inference/ — Model Routing

Task-aware LLM selection with multi-host Ollama fleet discovery.

| File | Purpose |
|------|---------|
| `model_router.py` | Task → model selection (vision, reasoning, fast) |
| `fleet.py` | `OllamaFleet` — multi-host discovery (conf/env/Tailscale) |
| `robot_thinker.py` | LLM-powered autonomous robot thinking |

### perception/ — Frame Analysis

Layered perception pipeline: quality gate → motion detection → object recognition.

| File | Purpose |
|------|---------|
| `perception.py` | Layered quality gate, complexity, motion detection |
| `vision.py` | Ollama chat API wrapper for visual reasoning |
| `extraction.py` | Fact extraction from conversation (regex-based) |

### actions/ — Command Dispatch

Lua-based action system with dynamic registry and formation support.

| File | Purpose |
|------|---------|
| `lua_motor.py` | Lua parser, `VALID_ACTIONS` |
| `lua_multi.py` | Multi-action sequences |
| `lua_registry.py` | Dynamic action registry |
| `formation_actions.py` | Squad formations |

### nodes/ — Sensor Hardware

Abstract sensor nodes wrapping cameras, PTZ controllers, mics, and speakers.

| File | Purpose |
|------|---------|
| `base.py` | Abstract `SensorNode` interface |
| `bcc950.py` | BCC950 PTZ camera + mic + speaker node |
| `mqtt_robot.py` | `MQTTSensorNode` — wraps MQTT robot as SensorNode |

### units/ — Unit Type Registry

16 unit types organized by category, with auto-discovery.

```
units/
├── people/     # NPC, civilian, hostile people
├── robots/     # Robot unit types
└── sensors/    # Sensor unit types
```

## Testing

```bash
# All engine tests
.venv/bin/python3 -m pytest tests/engine/ -v

# Specific subsystem
.venv/bin/python3 -m pytest tests/engine/simulation/ -v
.venv/bin/python3 -m pytest tests/engine/comms/ -v
.venv/bin/python3 -m pytest tests/engine/tactical/ -v
```

## Related

- [../../docs/SIMULATION.md](../../docs/SIMULATION.md) — Simulation engine design doc
- [../../docs/ESCALATION.md](../../docs/ESCALATION.md) — Threat escalation state machine
- [../../docs/MQTT.md](../../docs/MQTT.md) — MQTT topic hierarchy
- [../../docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md) — System architecture
- [../amy/](../amy/) — Amy AI commander (implements CommanderProtocol)
