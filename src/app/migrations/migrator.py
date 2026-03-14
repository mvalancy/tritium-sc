# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""MigrationManager — versioned schema upgrade engine.

Tracks schema version in a `_migrations` table. Loads numbered migration
modules from this package directory (001_initial.py, 002_add_dossiers.py, etc.).
Each module must expose:
    DESCRIPTION: str           — human-readable description
    async def upgrade(conn)    — apply the migration (receives an async connection)
    async def downgrade(conn)  — undo the migration (receives an async connection)
"""

from __future__ import annotations

import importlib
import pkgutil
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from loguru import logger
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncEngine, async_sessionmaker, AsyncSession


@dataclass
class MigrationInfo:
    """Metadata for a single migration module."""
    version: int
    name: str
    description: str
    module: object


class MigrationManager:
    """Manages versioned database schema migrations.

    Parameters
    ----------
    engine : AsyncEngine
        SQLAlchemy async engine to run migrations against.
    """

    MIGRATIONS_TABLE = "_migrations"
    _PATTERN = re.compile(r"^(\d{3})_(.+)$")

    def __init__(self, engine: AsyncEngine):
        self._engine = engine
        self._session_factory = async_sessionmaker(
            engine, class_=AsyncSession, expire_on_commit=False,
        )
        self._migrations: list[MigrationInfo] | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    async def migrate(self, target: Optional[int] = None) -> int:
        """Run all pending migrations up to `target` (default: latest).

        Returns the number of migrations applied.
        """
        await self._ensure_table()
        migrations = self._load_migrations()
        current = await self.get_version()

        if target is None:
            target = migrations[-1].version if migrations else 0

        applied = 0
        for m in migrations:
            if m.version <= current:
                continue
            if m.version > target:
                break

            logger.info(f"Applying migration {m.version:03d}: {m.description}")
            async with self._engine.begin() as conn:
                await m.module.upgrade(conn)
                await conn.execute(
                    text(
                        f"INSERT INTO {self.MIGRATIONS_TABLE} "
                        "(version, name, description, applied_at) "
                        "VALUES (:v, :n, :d, :ts)"
                    ),
                    {
                        "v": m.version,
                        "n": m.name,
                        "d": m.description,
                        "ts": datetime.now(timezone.utc).isoformat(),
                    },
                )
            applied += 1
            logger.info(f"Migration {m.version:03d} applied successfully")

        if applied == 0:
            logger.info(f"Schema is current (version {current})")
        else:
            logger.info(f"Applied {applied} migration(s), now at version {current + applied}")

        return applied

    async def rollback(self, target_version: int) -> int:
        """Roll back migrations down to (and including) `target_version`.

        Runs downgrade() for each migration from current version down to
        target_version + 1 (i.e. the schema will be at target_version after).

        Returns the number of migrations rolled back.
        """
        await self._ensure_table()
        migrations = self._load_migrations()
        current = await self.get_version()

        if target_version < 0:
            raise ValueError("target_version must be >= 0")
        if target_version >= current:
            logger.info(f"Already at version {current}, nothing to roll back")
            return 0

        # Build a version -> migration lookup
        by_version = {m.version: m for m in migrations}

        rolled = 0
        for v in range(current, target_version, -1):
            m = by_version.get(v)
            if m is None:
                raise RuntimeError(f"Migration {v:03d} not found, cannot rollback")

            logger.info(f"Rolling back migration {m.version:03d}: {m.description}")
            async with self._engine.begin() as conn:
                await m.module.downgrade(conn)
                await conn.execute(
                    text(f"DELETE FROM {self.MIGRATIONS_TABLE} WHERE version = :v"),
                    {"v": m.version},
                )
            rolled += 1
            logger.info(f"Migration {m.version:03d} rolled back")

        logger.info(f"Rolled back {rolled} migration(s), now at version {target_version}")
        return rolled

    async def get_version(self) -> int:
        """Return the current schema version (0 if no migrations applied)."""
        await self._ensure_table()
        async with self._session_factory() as session:
            result = await session.execute(
                text(f"SELECT MAX(version) FROM {self.MIGRATIONS_TABLE}")
            )
            row = result.scalar()
            return row if row is not None else 0

    async def status(self) -> list[dict]:
        """Return a list of applied migrations with timestamps."""
        await self._ensure_table()
        async with self._session_factory() as session:
            result = await session.execute(
                text(
                    f"SELECT version, name, description, applied_at "
                    f"FROM {self.MIGRATIONS_TABLE} ORDER BY version"
                )
            )
            rows = result.fetchall()

        applied_versions = {r[0] for r in rows}
        migrations = self._load_migrations()

        entries = []
        for m in migrations:
            applied_row = next((r for r in rows if r[0] == m.version), None)
            entries.append({
                "version": m.version,
                "name": m.name,
                "description": m.description,
                "applied": m.version in applied_versions,
                "applied_at": applied_row[3] if applied_row else None,
            })

        return entries

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    async def _ensure_table(self) -> None:
        """Create the migrations tracking table if it does not exist."""
        async with self._engine.begin() as conn:
            await conn.execute(
                text(f"""
                    CREATE TABLE IF NOT EXISTS {self.MIGRATIONS_TABLE} (
                        version INTEGER PRIMARY KEY,
                        name TEXT NOT NULL,
                        description TEXT NOT NULL,
                        applied_at TEXT NOT NULL
                    )
                """)
            )

    def _load_migrations(self) -> list[MigrationInfo]:
        """Discover and load all migration modules from this package."""
        if self._migrations is not None:
            return self._migrations

        package_dir = Path(__file__).parent
        migrations: list[MigrationInfo] = []

        for info in pkgutil.iter_modules([str(package_dir)]):
            match = self._PATTERN.match(info.name)
            if not match:
                continue

            version = int(match.group(1))
            name = info.name

            mod = importlib.import_module(f"app.migrations.{name}")

            description = getattr(mod, "DESCRIPTION", name)
            if not hasattr(mod, "upgrade"):
                logger.warning(f"Migration {name} missing upgrade(), skipping")
                continue
            if not hasattr(mod, "downgrade"):
                logger.warning(f"Migration {name} missing downgrade(), skipping")
                continue

            migrations.append(MigrationInfo(
                version=version,
                name=name,
                description=description,
                module=mod,
            ))

        migrations.sort(key=lambda m: m.version)
        self._migrations = migrations
        return migrations
