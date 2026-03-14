# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""003 — Notification table for persistent cross-plugin alerts.

Stores notifications that were previously kept only in memory by the
NotificationManager. Enables notification history, filtering, and
survival across server restarts.
"""

from sqlalchemy import text

DESCRIPTION = "Notification table — persistent cross-plugin alerts"


async def upgrade(conn) -> None:
    """Create notifications table."""
    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS notifications (
            id TEXT PRIMARY KEY,
            title TEXT NOT NULL,
            message TEXT NOT NULL,
            severity TEXT NOT NULL DEFAULT 'info',
            source TEXT NOT NULL DEFAULT 'system',
            entity_id TEXT,
            read BOOLEAN NOT NULL DEFAULT 0,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """))

    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_notifications_severity
            ON notifications(severity)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_notifications_read
            ON notifications(read)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_notifications_created_at
            ON notifications(created_at)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_notifications_source
            ON notifications(source)
    """))


async def downgrade(conn) -> None:
    """Drop notifications table."""
    await conn.execute(text("DROP TABLE IF EXISTS notifications"))
