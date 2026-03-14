# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Database migration system for Tritium-SC.

Versioned schema upgrades with forward and rollback support.
Each migration is a numbered Python module in this package.

Usage:
    from app.migrations import MigrationManager
    manager = MigrationManager(engine)
    await manager.migrate()       # Apply all pending
    await manager.rollback(1)     # Undo to version 1
    print(await manager.get_version())  # Current version

CLI:
    python -m app.migrations migrate
    python -m app.migrations rollback 1
    python -m app.migrations status
"""

from app.migrations.migrator import MigrationManager

__all__ = ["MigrationManager"]
