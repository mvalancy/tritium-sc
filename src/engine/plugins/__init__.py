"""TRITIUM-SC Plugin System.

Supports open-source and closed-source extensions via a standard
plugin interface with lifecycle management, dependency resolution,
and multi-source discovery.
"""

from engine.plugins.base import PluginInterface, PluginContext
from engine.plugins.manager import PluginManager, PluginDependencyError

__all__ = [
    "PluginInterface",
    "PluginContext",
    "PluginManager",
    "PluginDependencyError",
]
