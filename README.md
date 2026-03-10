# TRITIUM-SC

Command Center for managing IoT fleets with a tactical map, AI commander, and plugin system.

## What It Does

- **Web-based command center** with 2D/3D tactical maps on real satellite imagery, fog of war, and floating panels
- **AI commander (Amy)** with LLM-powered perception, continuous inner monologue, and autonomous decision-making
- **Plugin system** for extensibility -- Graphlings NPCs, edge fleet tracker, and more
- **26 UI panels** covering fleet telemetry, BLE tracking, mesh visualization, alerts, mission briefings, and unit control
- **Battle simulation engine** with wave-based combat, threat escalation, projectile physics, and auto-dispatch

## Quick Start

```bash
git clone git@github.com:Valpatel/tritium-sc.git
cd tritium-sc
./setup.sh install
./start.sh
# Open http://localhost:8000
```

## Architecture at a Glance

```
FastAPI backend (Python 3.12+)
    |-- EventBus (internal pub/sub)
    |-- MQTT bridge (device mesh)
    |-- WebSocket (/ws/live -> browser)
    |-- Amy (4-layer AI commander)
    |-- SimulationEngine (10Hz tick loop)

Vanilla JS frontend (no frameworks)
    |-- Canvas 2D + Three.js tactical map
    |-- CYBERCORE CSS (cyberpunk design language)
    |-- 26 floating panels, keyboard + gamepad input
```

All processing is local. No cloud. No subscriptions. SQLite for persistence, MQTT for device communication, WebSocket for real-time UI updates.

## Plugins

Plugins extend `PluginInterface` and are discovered from the `plugins/` directory, the `TRITIUM_PLUGINS` env var, or pip entry points. Each plugin can register routes, UI panels, background tasks, and event handlers.

| Plugin | Description |
|--------|-------------|
| **graphlings** | Compute-scalable digital life -- autonomous NPCs with LLM cognition distributed across your local fleet |
| **edge_tracker** | BLE presence tracking and fleet telemetry from tritium-edge IoT nodes |
| **npc_thoughts** | NPC inner monologue streaming via the event bus |

See [docs/PLUGIN-SPEC.md](docs/PLUGIN-SPEC.md) for the full plugin architecture.

## Development

```bash
./test.sh fast    # Tiers 1-3 + 8 (~60s) -- run after every change
./test.sh 3       # JS tests only (~3s)
./test.sh 9       # Integration E2E (~70s)
./setup.sh dev    # Dev server with hot reload
```

See [CLAUDE.md](CLAUDE.md) for code conventions, API reference, and testing tiers.

## Where to Go Next

| Doc | What's inside |
|-----|---------------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture and data flow |
| [docs/PLUGIN-SPEC.md](docs/PLUGIN-SPEC.md) | Plugin interface and lifecycle |
| [docs/SIMULATION.md](docs/SIMULATION.md) | Simulation engine internals |
| [docs/HOW-TO-PLAY.md](docs/HOW-TO-PLAY.md) | Player guide and controls |
| [docs/PLAN.md](docs/PLAN.md) | Development roadmap |
| [docs/USER-STORIES.md](docs/USER-STORIES.md) | UX stories and panel specs |
| [docs/MQTT.md](docs/MQTT.md) | MQTT topic reference |

## License

AGPL-3.0 -- See [LICENSE](LICENSE) for details.

Copyright 2026 Valpatel Software LLC.
