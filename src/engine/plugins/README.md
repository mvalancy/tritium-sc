# Engine: Plugin System

Core plugin infrastructure that all Tritium plugins build on.

## Files

| File | Purpose |
|------|---------|
| `base.py` | PluginInterface, EventDrainPlugin, PluginContext — base classes |
| `manager.py` | PluginManager — lifecycle management, discovery, route registration |
| `data_provider.py` | DataProvider protocol for plugins that produce map data |
| `layer_registry.py` | Registry for map layer plugins (GIS, satellite, terrain) |

## Plugin Lifecycle

1. Plugin implements `PluginInterface` (or extends `EventDrainPlugin`)
2. `PluginManager.register()` calls `plugin.configure(ctx)` with app + event bus
3. `PluginManager.start_all()` calls `plugin.start()` on each
4. At shutdown, `PluginManager.stop_all()` calls `plugin.stop()` in reverse order
