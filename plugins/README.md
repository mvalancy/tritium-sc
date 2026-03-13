# Plugins

**Where you are:** `tritium-sc/plugins/` — extensible plugin system for the Command Center.

**Parent:** [../CLAUDE.md](../CLAUDE.md) | [../../CLAUDE.md](../../CLAUDE.md) (tritium root)

## What This Is

Plugins extend the Command Center with new capabilities. Each plugin can register event handlers, add API endpoints, inject UI panels, and interact with the simulation engine.

## Available Plugins

| Plugin | Purpose | Status |
|--------|---------|--------|
| `edge_tracker/` | BLE presence tracking from tritium-edge devices | Active |
| `graphlings/` | Digital life NPCs (imported from graphlings project) | Active |
| `npc_thoughts.py` | NPC thought generation and display | Active |

## Plugin Architecture

Each plugin directory contains:
- Plugin code (Python modules)
- A loader file at `plugins/{name}_loader.py`
- `README.md` with plugin documentation

See [../docs/PLUGIN-SPEC.md](../docs/PLUGIN-SPEC.md) for the full plugin interface, lifecycle, and development guide.

## Related

- [../docs/PLUGIN-SPEC.md](../docs/PLUGIN-SPEC.md) — Plugin specification
- [../src/engine/plugins/](../src/engine/plugins/) — Plugin loader infrastructure
