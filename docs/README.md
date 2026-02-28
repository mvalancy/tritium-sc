# TRITIUM-SC Documentation

## For Users

| Document | Description |
|----------|-------------|
| [USER-STORIES.md](USER-STORIES.md) | Player experience stories and UI intent |
| [HOW-TO-PLAY.md](HOW-TO-PLAY.md) | Player guide: setup, gameplay, controls, strategy tips |
| [CONTROLS.md](CONTROLS.md) | Complete keyboard and gamepad control reference |
| [GAMEPAD.md](GAMEPAD.md) | Gamepad setup, calibration, and troubleshooting |

## Architecture

| Document | Description |
|----------|-------------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | System architecture: layers, boot sequence, event flow, thread model |
| [PLAN.md](PLAN.md) | Development roadmap: phases, progress, technical architecture |
| [SIMULATION.md](SIMULATION.md) | Simulation engine: target lifecycle, spawners, tick loop |
| [SIMULATION-ARCHITECTURE.md](SIMULATION-ARCHITECTURE.md) | Simulation pipeline and data flow |
| [ESCALATION.md](ESCALATION.md) | Threat escalation: state machine, zones, dispatch decision tree |
| [MQTT.md](MQTT.md) | MQTT communication: topic hierarchy, payloads, device integration |
| [COORDINATE-SYSTEM.md](COORDINATE-SYSTEM.md) | Local coordinate transforms and geo-reference |
| [UNIFIED-SPEC.md](UNIFIED-SPEC.md) | Command Center data flow, render pipeline, API list |
| [UI-REDESIGN.md](UI-REDESIGN.md) | Command Center plugin architecture and layout |
| [WINDOW-MANAGER.md](WINDOW-MANAGER.md) | dockview-core window manager integration plan |
| [MODEL-ROUTING.md](MODEL-ROUTING.md) | Task-aware model selection and fleet discovery |
| [INFRASTRUCTURE.md](INFRASTRUCTURE.md) | MTIG monitoring stack (Mosquitto, Telegraf, InfluxDB, Grafana) |
| [MESHTASTIC.md](MESHTASTIC.md) | Mesh radio integration for outdoor operations |
| [LANDSCAPE-SURVEY.md](LANDSCAPE-SURVEY.md) | ML model evaluation and alternatives survey |
| [VISION.md](VISION.md) | Perception philosophy, dual-use design, privacy tiers, roadmap |
| [PLUGIN-SPEC.md](PLUGIN-SPEC.md) | Plugin system: interface, lifecycle, discovery, dependencies |
| [NPC-INTELLIGENCE.md](NPC-INTELLIGENCE.md) | NPC brain, FSM, personality, crowd dynamics, thought registry |
| [NPC-WORLD-BEHAVIOR.md](NPC-WORLD-BEHAVIOR.md) | NPC world population: routing, buildings, daily routines |
| [TAK.md](TAK.md) | TAK/ATAK integration and CoT protocol bridge |
| [TERRAIN-PLAN.md](TERRAIN-PLAN.md) | DEM terrain elevation and 3D terrain rendering plan |

## Testing

| Document | Description |
|----------|-------------|
| [TESTING-PHILOSOPHY.md](TESTING-PHILOSOPHY.md) | TDD principles and testing tiers |
| [TEST-AUTOMATION.md](TEST-AUTOMATION.md) | E2E automation patterns and visual regression |
| [UI-TESTING.md](UI-TESTING.md) | Visual regression testing with Playwright and OpenCV |
| [ITERATE-PROMPT.md](ITERATE-PROMPT.md) | Agent iteration prompt template |

## Operations

| Document | Description |
|----------|-------------|
| [AUDIT-GAPS.md](AUDIT-GAPS.md) | Infrastructure security audit findings |
| [UI-VIEWS.md](UI-VIEWS.md) | UI view specifications: design intent for every panel, button, and element |
| [WAR-ROOM-UX-REVIEW.md](WAR-ROOM-UX-REVIEW.md) | War Room frontend UX architecture review |
| [DEMO-SPEC.md](DEMO-SPEC.md) | Demo project specifications for MQTT device examples |

## Quick Links

### For Users
- **How to Play**: See [HOW-TO-PLAY.md](HOW-TO-PLAY.md)
- **Getting Started**: See main [README.md](../README.md)
- **Controls**: Press `?` in the UI or see [CONTROLS.md](CONTROLS.md)
- **Gamepad Setup**: See [GAMEPAD.md](GAMEPAD.md)

### For Developers
- **Architecture**: See [ARCHITECTURE.md](ARCHITECTURE.md)
- **Project Context**: See [CLAUDE.md](../CLAUDE.md)
- **API Reference**: See CLAUDE.md API Endpoints section
- **MQTT Protocol**: See [MQTT.md](MQTT.md)
- **Robot Integration**: See `examples/robot-template/README.md`
- **Plugin Development**: See [PLUGIN-SPEC.md](PLUGIN-SPEC.md)
- **NPC System**: See [NPC-INTELLIGENCE.md](NPC-INTELLIGENCE.md)
- **Demo Devices**: See [DEMO-SPEC.md](DEMO-SPEC.md)

### For Amy
- **Threat Model**: See [ESCALATION.md](ESCALATION.md)
- **Simulation**: See [SIMULATION.md](SIMULATION.md)
- **Battlespace Overview**: See [ARCHITECTURE.md](ARCHITECTURE.md)

## Document Conventions

- All diagrams use mermaid syntax
- API documentation uses OpenAPI/Swagger format
- Each document stands alone but cross-references related docs
- Architecture decisions include rationale ("Why X, not Y?")

## Contributing Documentation

When adding new features:
1. Update relevant docs in this folder
2. Add keyboard shortcuts to CONTROLS.md
3. Add gamepad mappings to CONTROLS.md and GAMEPAD.md
4. Add new files to the Important Files table in CLAUDE.md
5. Update the table above if adding new documents
