# Backup Manager

**Where you are:** `tritium-sc/src/engine/backup/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The backup subsystem handles full system state export and import. It creates ZIP archives containing SQLite databases, KuzuDB graph data, configuration files, Amy's memory/transcripts, and plugin state. Supports both on-demand and scheduled periodic auto-backups.

## Key Files

| File | Purpose |
|------|---------|
| `backup.py` | BackupManager — export/import system state as ZIP archives with scheduled auto-backup |

## What Gets Backed Up

- SQLite databases (`tritium.db`, `dossiers.db`)
- KuzuDB graph data (if available)
- Configuration (automation rules, geofence zones, patrol routes, threat indicators)
- Amy's memory and transcripts (`data/amy/`)
- Plugin state files

## Related

- [../../app/main.py](../../app/main.py) — App lifespan hooks that initialize backup scheduling
- [../../app/models.py](../../app/models.py) — SQLAlchemy models for the databases being backed up
- [../tactical/dossier_manager.py](../tactical/dossier_manager.py) — Dossier database included in backups
- [../../amy/brain/memory.py](../../amy/brain/memory.py) — Amy's persistent memory included in backups
