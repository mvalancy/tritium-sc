# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for rate limiting middleware."""

import pytest


class TestRateLimitEntry:
    def test_import(self):
        from app.rate_limit import RateLimitEntry
        assert RateLimitEntry is not None

    def test_allows_within_limit(self):
        from app.rate_limit import RateLimitEntry
        entry = RateLimitEntry()
        for _ in range(10):
            allowed, remaining = entry.check(10, 60)
            assert allowed

    def test_blocks_over_limit(self):
        from app.rate_limit import RateLimitEntry
        entry = RateLimitEntry()
        for _ in range(10):
            entry.check(10, 60)
        allowed, remaining = entry.check(10, 60)
        assert not allowed
        assert remaining == 0

    def test_remaining_decreases(self):
        from app.rate_limit import RateLimitEntry
        entry = RateLimitEntry()
        _, rem1 = entry.check(5, 60)
        _, rem2 = entry.check(5, 60)
        assert rem2 < rem1


class TestRateLimitMiddleware:
    def test_import(self):
        from app.rate_limit import RateLimitMiddleware
        assert RateLimitMiddleware is not None

    def test_exempt_paths(self):
        from app.rate_limit import EXEMPT_PATHS, EXEMPT_PREFIXES
        assert "/ws/live" in EXEMPT_PATHS
        assert "/health" in EXEMPT_PATHS
        assert any(p.startswith("/static") for p in EXEMPT_PREFIXES)


class TestBackupRouter:
    def test_import(self):
        from app.routers.backup import router
        assert router is not None

    def test_manifest_builder(self):
        from app.routers.backup import _get_backup_manifest
        manifest = _get_backup_manifest()
        assert "version" in manifest
        assert manifest["version"] == "1.0"
        assert "created_at" in manifest
        assert "contents" in manifest


class TestMigrations:
    def test_import(self):
        from app.migrations import MIGRATIONS, run_migrations
        assert len(MIGRATIONS) >= 4
        assert callable(run_migrations)

    def test_migration_order(self):
        from app.migrations import MIGRATIONS
        versions = sorted(MIGRATIONS.keys())
        # Versions should be sequential starting from 1
        assert versions[0] == 1
        for i in range(1, len(versions)):
            assert versions[i] == versions[i - 1] + 1

    def test_migrations_have_descriptions(self):
        from app.migrations import MIGRATIONS
        for version, (desc, sql) in MIGRATIONS.items():
            assert len(desc) > 0, f"Migration {version} missing description"
            assert len(sql.strip()) > 0, f"Migration {version} missing SQL"
