# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Database migration system for Tritium-SC.

Simple version-based migrations using raw SQL. Each migration is a numbered
function that runs exactly once. The current schema version is stored in
a `schema_version` table.

Usage:
    from app.migrations import run_migrations
    await run_migrations()  # Called during app startup
"""

from loguru import logger
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession

from app.database import async_session


# Migration registry — add new migrations here
MIGRATIONS: dict[int, tuple[str, str]] = {
    # version: (description, SQL)
    1: (
        "Create schema_version tracking table",
        """
        CREATE TABLE IF NOT EXISTS schema_version (
            version INTEGER PRIMARY KEY,
            description TEXT NOT NULL,
            applied_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
        """,
    ),
    2: (
        "Add auth_users table",
        """
        CREATE TABLE IF NOT EXISTS auth_users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            role TEXT NOT NULL DEFAULT 'user',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            last_login TIMESTAMP,
            active BOOLEAN DEFAULT 1
        )
        """,
    ),
    3: (
        "Add api_keys table",
        """
        CREATE TABLE IF NOT EXISTS api_keys (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            key_hash TEXT UNIQUE NOT NULL,
            name TEXT NOT NULL,
            owner TEXT NOT NULL,
            permissions TEXT DEFAULT '[]',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            expires_at TIMESTAMP,
            active BOOLEAN DEFAULT 1
        )
        """,
    ),
    4: (
        "Add audit_log table",
        """
        CREATE TABLE IF NOT EXISTS audit_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            user TEXT,
            action TEXT NOT NULL,
            resource TEXT,
            details TEXT,
            ip_address TEXT,
            success BOOLEAN DEFAULT 1
        )
        """,
    ),
}


async def get_current_version(session: AsyncSession) -> int:
    """Get the current schema version."""
    try:
        result = await session.execute(
            text("SELECT MAX(version) FROM schema_version")
        )
        row = result.scalar()
        return row if row is not None else 0
    except Exception:
        return 0


async def run_migrations() -> int:
    """Run all pending migrations. Returns number of migrations applied."""
    async with async_session() as session:
        # Ensure schema_version table exists
        await session.execute(text(MIGRATIONS[1][1]))
        await session.commit()

        current = await get_current_version(session)
        applied = 0

        for version in sorted(MIGRATIONS.keys()):
            if version <= current:
                continue

            desc, sql = MIGRATIONS[version]
            logger.info(f"Running migration {version}: {desc}")

            try:
                await session.execute(text(sql))
                await session.execute(
                    text("INSERT INTO schema_version (version, description) VALUES (:v, :d)"),
                    {"v": version, "d": desc},
                )
                await session.commit()
                applied += 1
                logger.info(f"Migration {version} applied successfully")
            except Exception as e:
                await session.rollback()
                logger.error(f"Migration {version} failed: {e}")
                raise

        if applied > 0:
            logger.info(f"Applied {applied} migrations (now at version {version})")
        else:
            logger.info(f"Database schema is current (version {current})")

        return applied
