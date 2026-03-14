# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Loader shim for PluginManager auto-discovery.

PluginManager scans plugins/ for top-level *.py files. This file
re-exports FederationPlugin so it's discoverable without modifying
the plugin manager's scan logic.
"""
from __future__ import annotations

import sys
from pathlib import Path

# Ensure the plugins/ directory is on sys.path so the federation
# package can be imported.
_plugins_dir = str(Path(__file__).resolve().parent)
if _plugins_dir not in sys.path:
    sys.path.insert(0, _plugins_dir)

from federation.plugin import FederationPlugin  # noqa: E402, F401

__all__ = ["FederationPlugin"]
