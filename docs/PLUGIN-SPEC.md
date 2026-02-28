# TRITIUM-SC Plugin Architecture

## 1. Overview

TRITIUM-SC uses a plugin architecture to support both open-source and
closed-source extensions. The core system provides the event bus, target
tracker, simulation engine, and web framework. Plugins add capabilities
like hardware bridges, AI integrations, data sources, and UI panels.

**Business model**: The core tritium-sc repo is open source. Premium
capabilities (Graphlings AI integration, advanced analytics, fleet
management) ship as closed-source Python packages installable via pip.

## 2. Plugin Interface

Every plugin implements `PluginInterface`:

```python
class PluginInterface(ABC):
    """Base class all TRITIUM-SC plugins must extend."""

    @property
    @abstractmethod
    def plugin_id(self) -> str:
        """Unique identifier (reverse-domain: 'com.example.my-plugin')."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable name."""

    @property
    @abstractmethod
    def version(self) -> str:
        """Semantic version (e.g., '1.2.3')."""

    @property
    def capabilities(self) -> set[str]:
        """Capabilities this plugin provides. Default: empty set.

        Standard capabilities:
        - 'bridge'      — External system bridge (MQTT, WebSocket, etc.)
        - 'data_source' — Adds targets or data to the system
        - 'ai'          — AI/ML integration (LLM, vision, etc.)
        - 'routes'      — Registers FastAPI routes
        - 'ui'          — Provides frontend panels/layers
        - 'background'  — Runs a background thread/task
        """
        return set()

    @property
    def dependencies(self) -> list[str]:
        """Plugin IDs this plugin depends on. Default: empty list."""
        return []

    def configure(self, ctx: PluginContext) -> None:
        """Called once with the plugin context. Store references here.

        The context provides:
        - event_bus: EventBus instance
        - target_tracker: TargetTracker instance
        - simulation_engine: SimulationEngine or None
        - settings: Application settings dict
        - app: FastAPI app instance (for registering routes)
        - logger: Logger scoped to this plugin
        """

    @abstractmethod
    def start(self) -> None:
        """Start the plugin. Called after configure()."""

    @abstractmethod
    def stop(self) -> None:
        """Stop the plugin. Called during shutdown."""

    @property
    def healthy(self) -> bool:
        """Health check. Default: True."""
        return True
```

## 3. Plugin Context

Plugins receive a `PluginContext` during configuration:

```python
@dataclass
class PluginContext:
    event_bus: EventBus
    target_tracker: TargetTracker
    simulation_engine: SimulationEngine | None
    settings: dict               # Plugin-specific settings from config
    app: FastAPI | None          # For route registration
    logger: logging.Logger       # Scoped logger
    plugin_manager: PluginManager  # For inter-plugin communication
```

## 4. Plugin Manager

The `PluginManager` handles discovery, lifecycle, and dependency resolution:

```python
class PluginManager:
    def register(self, plugin: PluginInterface) -> None
    def discover(self, paths: list[str] = None) -> list[PluginInterface]
    def configure_all(self, ctx_factory: Callable) -> None
    def start_all(self) -> dict[str, bool]    # plugin_id -> success
    def stop_all(self) -> None
    def get_plugin(self, plugin_id: str) -> PluginInterface | None
    def list_plugins(self) -> list[dict]      # id, name, version, capabilities, healthy
    def health_check(self) -> dict[str, bool]
```

### 4.1 Discovery

Plugins are discovered from multiple sources (in priority order):

1. **Explicit registration**: `manager.register(MyPlugin())` in main.py
2. **Entry points**: `pip install` packages declare `tritium_plugins` entry point
3. **Directory scan**: Python files in `plugins/` directory (project root)
4. **Settings**: `TRITIUM_PLUGINS` env var (comma-separated module paths)

Entry point example (in closed-source package's `pyproject.toml`):

```toml
[project.entry-points."tritium_plugins"]
graphlings = "graphlings_tritium:GraphlingsBridgePlugin"
```

### 4.2 Lifecycle

```
discover() -> register() -> configure() -> start() -> [running] -> stop()
```

1. **Discover**: Find all available plugins from all sources
2. **Register**: Validate plugin interface compliance
3. **Configure**: Topological sort by dependencies, call `configure()` in order
4. **Start**: Call `start()` in dependency order, log success/failure per plugin
5. **Running**: Plugins operate, health checked periodically
6. **Stop**: Call `stop()` in reverse dependency order

### 4.3 Dependency Resolution

Dependencies are declared by `plugin_id`. The manager resolves them via
topological sort. Circular dependencies raise `PluginDependencyError`.
Missing optional dependencies log a warning; missing required dependencies
prevent the dependent plugin from starting.

### 4.4 Error Isolation

A plugin that fails to start does not crash the system. The manager:
- Catches all exceptions from `configure()`, `start()`, `stop()`
- Logs the error with plugin ID and traceback
- Marks the plugin as unhealthy
- Continues with remaining plugins
- Plugins depending on a failed plugin are skipped

## 5. Configuration

Plugin settings live in `conf/plugins/` as TOML or env vars:

```bash
# Environment variable pattern
TRITIUM_PLUGIN_<PLUGIN_ID>_<KEY>=value

# Example for graphlings bridge
TRITIUM_PLUGIN_GRAPHLINGS_BRIDGE_API_URL=http://localhost:4774
TRITIUM_PLUGIN_GRAPHLINGS_BRIDGE_API_KEY=secret123
```

Or in `conf/plugins/graphlings-bridge.toml`:

```toml
[graphlings-bridge]
api_url = "http://localhost:4774"
ws_url = "ws://localhost:4775"
api_key = "secret123"
```

## 6. API Endpoints

The plugin manager exposes:

- `GET /api/plugins` — List all plugins with status
- `GET /api/plugins/{id}` — Plugin details
- `GET /api/plugins/{id}/health` — Health check

## 7. Distribution Models

### 7.1 Open Source (in-repo)

Plugins in `src/engine/plugins/builtin/` ship with the core:
- MQTT Bridge (wraps existing `mqtt_bridge.py`)
- Meshtastic Bridge
- Graphlings Bridge (public API only)
- Synthetic Camera

### 7.2 Closed Source (pip package)

Premium plugins distributed as Python wheels:
```bash
pip install tritium-graphlings-ai     # Advanced Graphlings integration
pip install tritium-fleet-manager     # Multi-site fleet management
pip install tritium-analytics-pro     # Advanced analytics dashboard
```

These register via entry points. No source code exposed.

### 7.3 Drop-in (plugins/ directory)

For custom one-off plugins, drop a Python file in `plugins/`:

```python
# plugins/my_custom_bridge.py
from engine.plugins import PluginInterface

class MyCustomBridge(PluginInterface):
    plugin_id = "com.mycompany.custom-bridge"
    name = "Custom Bridge"
    version = "0.1.0"
    capabilities = {"bridge", "background"}

    def start(self): ...
    def stop(self): ...
```

## 8. Use Cases

### 8.1 Ambient World Population (Graphlings NPCs)

The idle world must feel alive. Neutral NPCs (pedestrians, cyclists,
delivery drivers, dog walkers) populate the neighborhood and react to
their environment. A plugin bridges to Graphlings' autonomous agent
system to drive these NPCs:

- **Spawn neutrals**: Plugin creates Graphlings entities with civilian
  personality traits, places them on streets from the road graph
- **Context injection**: Plugin sends each NPC situational awareness
  ("police car approaching", "construction noise ahead", "Amy's rover
  patrolling nearby") via Graphlings' action/say/think endpoints
- **Behavioral responses**: NPCs autonomously react — crossing the
  street, pausing to look, waving, changing route. Their thoughts and
  emotions stream back to TRITIUM-SC for display
- **Escalation transition**: When a scenario starts, some neutrals can
  become hostile (riot escalation) — the plugin changes their personality
  traits and behavior via Graphlings API
- **Bystander effects**: Neutrals flee from combat, call for help,
  record with phones — all driven by Graphlings' autonomous agent

This keeps the tactical map populated and realistic between battles.

### 8.2 Enemy AI Agents (Graphlings Hostiles)

Hostile units with their own LLM-driven decision making:

- Plugin spawns Graphlings entities with aggressive personality traits
- Each hostile has its own consciousness and tactical reasoning
- Graphlings' ethical core can be configured per entity (hostiles have
  different ethical constraints than civilians)
- Thoughts/decisions stream back for Amy's situational awareness
- Elimination events flow through MQTT to simulation engine

### 8.3 Graphlings Bridge Plugin (Two-Tier)

**Tier 1: Open Source Bridge (in this repo)**
- Uses Graphlings public REST/WebSocket APIs only
- Translates MQTT <-> Graphlings HTTP/WS
- No Graphlings source code or internals
- Ships in `src/engine/plugins/builtin/graphlings_bridge.py`

**Tier 2: Closed Source AI Plugin (separate package)**
- Uses Graphlings SDK with full internal access
- Deep AI integration: consciousness layers, personality transfer
- Advanced context injection using internal entity state
- Distributed as `tritium-graphlings-ai` pip package
- Declares dependency on Tier 1 bridge plugin

## 9. Frontend Plugin Support

Plugins with `'ui'` capability can register frontend assets:

```python
class MyPlugin(PluginInterface):
    capabilities = {"ui"}

    def get_frontend_assets(self) -> dict:
        return {
            "js": ["static/plugins/my-plugin/panel.js"],
            "css": ["static/plugins/my-plugin/styles.css"],
            "panels": [
                {"id": "my-panel", "title": "My Panel", "category": "intel"}
            ],
        }
```

The plugin manager serves these via `/static/plugins/{id}/` routes.

## 10. Security

- Plugins run in the same process (no sandbox)
- Plugin authors are trusted (installed via pip or dropped in by admin)
- Closed-source plugins are signed with Ed25519 keys
- `TRITIUM_PLUGIN_VERIFY_SIGNATURES=true` enforces signature checking
- API key management per plugin via settings
