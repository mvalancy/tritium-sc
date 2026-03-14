# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the database migration system."""

import pytest
import pytest_asyncio
from sqlalchemy import text
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker

from app.migrations.migrator import MigrationManager


@pytest_asyncio.fixture
async def engine():
    """Create an in-memory SQLite engine for testing."""
    eng = create_async_engine("sqlite+aiosqlite:///:memory:", echo=False)
    yield eng
    await eng.dispose()


@pytest_asyncio.fixture
async def manager(engine):
    """Create a MigrationManager with in-memory engine."""
    return MigrationManager(engine)


@pytest.mark.asyncio
async def test_initial_version_is_zero(manager):
    """Fresh database should be at version 0."""
    version = await manager.get_version()
    assert version == 0


@pytest.mark.asyncio
async def test_migrate_applies_all(manager):
    """migrate() should apply all pending migrations."""
    applied = await manager.migrate()
    assert applied >= 3  # We have at least 3 migration files
    version = await manager.get_version()
    assert version >= 3


@pytest.mark.asyncio
async def test_migrate_is_idempotent(manager):
    """Running migrate() twice applies nothing the second time."""
    await manager.migrate()
    applied = await manager.migrate()
    assert applied == 0


@pytest.mark.asyncio
async def test_migrate_to_target(manager):
    """migrate(target=1) should only apply migration 001."""
    applied = await manager.migrate(target=1)
    assert applied == 1
    version = await manager.get_version()
    assert version == 1


@pytest.mark.asyncio
async def test_migrate_incremental(manager):
    """Migrating to 1, then to 3 should apply 2 more."""
    await manager.migrate(target=1)
    applied = await manager.migrate(target=3)
    assert applied == 2
    version = await manager.get_version()
    assert version == 3


@pytest.mark.asyncio
async def test_rollback(manager, engine):
    """Rollback from version 3 to version 1 should undo 2 migrations."""
    await manager.migrate(target=3)
    rolled = await manager.rollback(1)
    assert rolled == 2
    version = await manager.get_version()
    assert version == 1


@pytest.mark.asyncio
async def test_rollback_to_zero(manager, engine):
    """Rollback to 0 should undo all migrations."""
    await manager.migrate()
    version_before = await manager.get_version()
    assert version_before >= 3

    rolled = await manager.rollback(0)
    assert rolled == version_before
    version = await manager.get_version()
    assert version == 0


@pytest.mark.asyncio
async def test_rollback_noop(manager):
    """Rollback when already at target should do nothing."""
    await manager.migrate(target=2)
    rolled = await manager.rollback(2)
    assert rolled == 0


@pytest.mark.asyncio
async def test_rollback_invalid_version(manager):
    """Rollback to negative version should raise ValueError."""
    await manager.migrate()
    with pytest.raises(ValueError):
        await manager.rollback(-1)


@pytest.mark.asyncio
async def test_status(manager):
    """status() returns migration info with applied/pending flags."""
    await manager.migrate(target=2)
    entries = await manager.status()

    assert len(entries) >= 3
    # First two should be applied
    assert entries[0]["applied"] is True
    assert entries[0]["version"] == 1
    assert entries[1]["applied"] is True
    assert entries[1]["version"] == 2
    # Third should be pending
    assert entries[2]["applied"] is False
    assert entries[2]["version"] == 3


@pytest.mark.asyncio
async def test_tables_created_by_migration_001(manager, engine):
    """Migration 001 should create core tables."""
    await manager.migrate(target=1)

    async with engine.begin() as conn:
        # Check that cameras table exists
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='cameras'")
        )
        assert result.scalar() == "cameras"

        # Check events table
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='events'")
        )
        assert result.scalar() == "events"

        # Check assets table
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='assets'")
        )
        assert result.scalar() == "assets"

        # Check auth_users table
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='auth_users'")
        )
        assert result.scalar() == "auth_users"


@pytest.mark.asyncio
async def test_tables_created_by_migration_002(manager, engine):
    """Migration 002 should create dossier tables."""
    await manager.migrate(target=2)

    async with engine.begin() as conn:
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='dossiers'")
        )
        assert result.scalar() == "dossiers"

        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='dossier_signals'")
        )
        assert result.scalar() == "dossier_signals"

        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='dossier_enrichments'")
        )
        assert result.scalar() == "dossier_enrichments"


@pytest.mark.asyncio
async def test_tables_created_by_migration_003(manager, engine):
    """Migration 003 should create notifications table."""
    await manager.migrate(target=3)

    async with engine.begin() as conn:
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='notifications'")
        )
        assert result.scalar() == "notifications"


@pytest.mark.asyncio
async def test_rollback_drops_tables(manager, engine):
    """Rolling back migration 003 should drop the notifications table."""
    await manager.migrate(target=3)
    await manager.rollback(2)

    async with engine.begin() as conn:
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='notifications'")
        )
        assert result.scalar() is None

        # Dossiers should still exist
        result = await conn.execute(
            text("SELECT name FROM sqlite_master WHERE type='table' AND name='dossiers'")
        )
        assert result.scalar() == "dossiers"


@pytest.mark.asyncio
async def test_migrate_after_rollback(manager):
    """After rollback, re-migrating should re-apply dropped migrations."""
    await manager.migrate(target=3)
    await manager.rollback(1)
    assert await manager.get_version() == 1

    applied = await manager.migrate()
    assert applied == 2
    assert await manager.get_version() == 3


@pytest.mark.asyncio
async def test_status_after_rollback(manager):
    """After rollback, status should show rolled-back migrations as pending."""
    await manager.migrate(target=3)
    await manager.rollback(1)

    entries = await manager.status()
    assert entries[0]["applied"] is True
    assert entries[1]["applied"] is False
    assert entries[2]["applied"] is False
