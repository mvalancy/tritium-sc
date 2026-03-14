# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""002 — Dossier tables for persistent entity intelligence.

Stores long-lived target dossiers with signal history and enrichment
results. These tables mirror the schema used by tritium-lib's
DossierStore but live in the main application database.
"""

from sqlalchemy import text

DESCRIPTION = "Dossier tables — persistent entity intelligence"


async def upgrade(conn) -> None:
    """Create dossier tables."""
    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS dossiers (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            entity_id TEXT UNIQUE NOT NULL,
            entity_type TEXT NOT NULL DEFAULT 'unknown',
            name TEXT,
            alias TEXT,
            alliance TEXT NOT NULL DEFAULT 'unknown',
            threat_level TEXT NOT NULL DEFAULT 'unknown',
            confidence REAL NOT NULL DEFAULT 0.0,
            first_seen TEXT NOT NULL,
            last_seen TEXT NOT NULL,
            last_position_lat REAL,
            last_position_lng REAL,
            notes TEXT,
            tags TEXT DEFAULT '[]',
            metadata_json TEXT DEFAULT '{}',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """))

    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_dossiers_entity_id ON dossiers(entity_id)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_dossiers_entity_type ON dossiers(entity_type)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_dossiers_alliance ON dossiers(alliance)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_dossiers_threat_level ON dossiers(threat_level)
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS dossier_signals (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            dossier_id INTEGER NOT NULL REFERENCES dossiers(id) ON DELETE CASCADE,
            signal_type TEXT NOT NULL,
            source TEXT NOT NULL,
            value TEXT NOT NULL,
            confidence REAL NOT NULL DEFAULT 1.0,
            timestamp TEXT NOT NULL,
            metadata_json TEXT DEFAULT '{}'
        )
    """))

    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_dossier_signals_dossier_id
            ON dossier_signals(dossier_id)
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS dossier_enrichments (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            dossier_id INTEGER NOT NULL REFERENCES dossiers(id) ON DELETE CASCADE,
            provider TEXT NOT NULL,
            enrichment_type TEXT NOT NULL,
            data_json TEXT NOT NULL DEFAULT '{}',
            confidence REAL NOT NULL DEFAULT 1.0,
            timestamp TEXT NOT NULL
        )
    """))

    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_dossier_enrichments_dossier_id
            ON dossier_enrichments(dossier_id)
    """))


async def downgrade(conn) -> None:
    """Drop dossier tables."""
    await conn.execute(text("DROP TABLE IF EXISTS dossier_enrichments"))
    await conn.execute(text("DROP TABLE IF EXISTS dossier_signals"))
    await conn.execute(text("DROP TABLE IF EXISTS dossiers"))
