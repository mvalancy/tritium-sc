# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""001 — Baseline schema: cameras, events, zones, assets tables.

These tables are also created by SQLAlchemy's metadata.create_all()
(via app.models). This migration uses IF NOT EXISTS so it is safe to
run on a database that already has these tables.
"""

from sqlalchemy import text

DESCRIPTION = "Baseline schema — cameras, events, zones, assets"


async def upgrade(conn) -> None:
    """Create the core tables."""
    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS cameras (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            channel INTEGER UNIQUE NOT NULL,
            name VARCHAR(100) NOT NULL,
            rtsp_url VARCHAR(500),
            substream_url VARCHAR(500),
            enabled BOOLEAN DEFAULT 1,
            position_x FLOAT,
            position_y FLOAT,
            heading FLOAT,
            fov FLOAT,
            mount_height FLOAT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS events (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            camera_id INTEGER NOT NULL REFERENCES cameras(id),
            timestamp TIMESTAMP NOT NULL,
            end_timestamp TIMESTAMP,
            event_type VARCHAR(50) NOT NULL,
            confidence FLOAT NOT NULL,
            bbox_json TEXT,
            thumbnail_path VARCHAR(500),
            video_clip_path VARCHAR(500),
            description TEXT,
            zones_triggered TEXT,
            acknowledged BOOLEAN DEFAULT 0,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """))

    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_events_camera_id ON events(camera_id)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_events_timestamp ON events(timestamp)
    """))
    await conn.execute(text("""
        CREATE INDEX IF NOT EXISTS ix_events_event_type ON events(event_type)
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS zones (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            camera_id INTEGER NOT NULL REFERENCES cameras(id),
            name VARCHAR(100) NOT NULL,
            polygon_json TEXT NOT NULL,
            zone_type VARCHAR(50) NOT NULL,
            alert_on TEXT,
            cooldown_seconds INTEGER DEFAULT 60,
            enabled BOOLEAN DEFAULT 1
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS alerts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            event_id INTEGER NOT NULL REFERENCES events(id),
            zone_id INTEGER REFERENCES zones(id),
            delivery_method VARCHAR(50) NOT NULL,
            delivered_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            payload_json TEXT
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS transcripts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            camera_id INTEGER NOT NULL REFERENCES cameras(id),
            start_time TIMESTAMP NOT NULL,
            end_time TIMESTAMP NOT NULL,
            text TEXT NOT NULL,
            language VARCHAR(10)
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS settings (
            key VARCHAR(100) PRIMARY KEY,
            value TEXT NOT NULL
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS assets (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            asset_id VARCHAR(50) UNIQUE NOT NULL,
            name VARCHAR(100) NOT NULL,
            asset_type VARCHAR(50) NOT NULL,
            asset_class VARCHAR(50) NOT NULL,
            status VARCHAR(50) DEFAULT 'standby',
            battery_level FLOAT,
            ammo_level FLOAT,
            position_x FLOAT,
            position_y FLOAT,
            heading FLOAT,
            speed FLOAT,
            home_x FLOAT,
            home_y FLOAT,
            capabilities TEXT,
            connection_url VARCHAR(500),
            camera_url VARCHAR(500),
            height_meters FLOAT,
            floor_level INTEGER,
            mounting_type VARCHAR(50),
            coverage_radius_meters FLOAT,
            coverage_cone_angle FLOAT,
            enabled BOOLEAN DEFAULT 1,
            last_heartbeat TIMESTAMP,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS asset_tasks (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            task_id VARCHAR(50) UNIQUE NOT NULL,
            asset_id INTEGER NOT NULL REFERENCES assets(id),
            task_type VARCHAR(50) NOT NULL,
            priority INTEGER DEFAULT 5,
            status VARCHAR(50) DEFAULT 'pending',
            target_type VARCHAR(50),
            target_id VARCHAR(100),
            waypoints_json TEXT,
            parameters_json TEXT,
            started_at TIMESTAMP,
            completed_at TIMESTAMP,
            result TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """))

    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS asset_telemetry (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            asset_id INTEGER NOT NULL REFERENCES assets(id),
            timestamp TIMESTAMP NOT NULL,
            position_x FLOAT NOT NULL,
            position_y FLOAT NOT NULL,
            heading FLOAT,
            speed FLOAT,
            battery_level FLOAT,
            status VARCHAR(50)
        )
    """))

    # Auth tables (migrated from old migrations.py)
    await conn.execute(text("""
        CREATE TABLE IF NOT EXISTS auth_users (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL,
            role TEXT NOT NULL DEFAULT 'user',
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            last_login TIMESTAMP,
            active BOOLEAN DEFAULT 1
        )
    """))

    await conn.execute(text("""
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
    """))

    await conn.execute(text("""
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
    """))


async def downgrade(conn) -> None:
    """Drop all baseline tables (destructive)."""
    for table in [
        "audit_log", "api_keys", "auth_users",
        "asset_telemetry", "asset_tasks", "assets",
        "settings", "transcripts", "alerts", "zones", "events", "cameras",
    ]:
        await conn.execute(text(f"DROP TABLE IF EXISTS {table}"))
