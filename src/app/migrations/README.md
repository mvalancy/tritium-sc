# Database Migrations

**Where you are:** `tritium-sc/src/app/migrations/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

Versioned schema migration system for the Tritium-SC SQLite database. Uses a `_migrations` table to track which migrations have been applied. Migrations run automatically on server startup.

## Key Files

| File | Purpose |
|------|---------|
| `migrator.py` | MigrationManager — discovers, orders, and applies migration modules |
| `__main__.py` | CLI entry point — run migrations manually via `python -m src.app.migrations` |
| `001_initial.py` | Initial schema — core tables (cameras, events, zones, assets) |
| `002_add_dossiers.py` | Adds dossier tables for persistent target intelligence files |
| `003_add_notifications.py` | Adds notification storage tables |

## Adding a Migration

1. Create `NNN_description.py` (zero-padded 3-digit number)
2. Expose three items in the module:
   - `DESCRIPTION: str` — human-readable description
   - `async def upgrade(conn)` — apply the migration (receives async connection)
   - `async def downgrade(conn)` — undo the migration (receives async connection)
3. The migrator auto-discovers modules by filename pattern `\d{3}_*.py`

## Related

- [../main.py](../main.py) — App lifespan runs migrations on startup
- [../database.py](../database.py) — Async database setup and FTS5 tables
- [../models.py](../models.py) — SQLAlchemy models that migrations create tables for
