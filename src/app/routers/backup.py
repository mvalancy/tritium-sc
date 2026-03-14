# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Backup and restore API for Tritium-SC state.

Exports/imports the SQLite database, configuration, and Amy memory
as a single archive file. Useful for migrations and disaster recovery.
"""

import io
import json
import os
import shutil
import tempfile
import zipfile
from datetime import datetime, timezone
from pathlib import Path

from fastapi import APIRouter, HTTPException, UploadFile, File
from fastapi.responses import StreamingResponse
from loguru import logger

from app.config import settings

router = APIRouter(prefix="/api/backup", tags=["backup"])


def _get_db_path() -> Path:
    """Extract SQLite file path from database URL."""
    url = settings.database_url
    # sqlite+aiosqlite:///./tritium.db -> ./tritium.db
    if ":///" in url:
        path = url.split(":///")[-1]
        return Path(path)
    return Path("tritium.db")


def _get_backup_manifest() -> dict:
    """Build manifest describing what's being backed up."""
    db_path = _get_db_path()
    amy_memory = Path("data/amy/memory.json")

    manifest = {
        "version": "1.0",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "tritium_version": "0.1.0",
        "contents": {},
    }

    if db_path.exists():
        manifest["contents"]["database"] = {
            "file": db_path.name,
            "size_bytes": db_path.stat().st_size,
        }
    if amy_memory.exists():
        manifest["contents"]["amy_memory"] = {
            "file": "amy/memory.json",
            "size_bytes": amy_memory.stat().st_size,
        }

    return manifest


@router.get("/status")
async def backup_status():
    """Get backup system status and available data."""
    db_path = _get_db_path()
    return {
        "database_exists": db_path.exists(),
        "database_size_bytes": db_path.stat().st_size if db_path.exists() else 0,
        "database_path": str(db_path),
        "amy_memory_exists": Path("data/amy/memory.json").exists(),
        "backup_ready": db_path.exists(),
    }


@router.post("/export")
async def export_backup():
    """Export a full backup as a ZIP archive.

    Returns a ZIP file containing:
    - manifest.json (backup metadata)
    - tritium.db (SQLite database)
    - amy/memory.json (Amy's persistent memory, if exists)
    - config snapshot
    """
    db_path = _get_db_path()
    if not db_path.exists():
        raise HTTPException(status_code=404, detail="No database found to backup")

    buf = io.BytesIO()

    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        # Manifest
        manifest = _get_backup_manifest()
        zf.writestr("manifest.json", json.dumps(manifest, indent=2))

        # Database — make a copy to avoid locking issues
        with tempfile.NamedTemporaryFile(suffix=".db", delete=False) as tmp:
            tmp_path = tmp.name
        try:
            shutil.copy2(str(db_path), tmp_path)
            zf.write(tmp_path, db_path.name)
        finally:
            os.unlink(tmp_path)

        # Amy memory
        amy_mem = Path("data/amy/memory.json")
        if amy_mem.exists():
            zf.write(str(amy_mem), "amy/memory.json")

        # Config snapshot (non-sensitive settings only)
        safe_config = {
            k: v for k, v in settings.model_dump().items()
            if "password" not in k.lower()
            and "secret" not in k.lower()
            and "token" not in k.lower()
            and "key" not in k.lower()
        }
        zf.writestr("config_snapshot.json", json.dumps(safe_config, indent=2, default=str))

    buf.seek(0)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"tritium_backup_{timestamp}.zip"

    logger.info(f"Backup exported: {filename} ({buf.getbuffer().nbytes} bytes)")

    return StreamingResponse(
        buf,
        media_type="application/zip",
        headers={"Content-Disposition": f"attachment; filename={filename}"},
    )


@router.post("/import")
async def import_backup(file: UploadFile = File(...)):
    """Import a backup from a ZIP archive.

    WARNING: This will overwrite the current database and Amy memory.
    The server should be restarted after import.
    """
    if not file.filename or not file.filename.endswith(".zip"):
        raise HTTPException(status_code=400, detail="File must be a ZIP archive")

    content = await file.read()
    if len(content) > 500 * 1024 * 1024:  # 500MB limit
        raise HTTPException(status_code=413, detail="Backup file too large (max 500MB)")

    buf = io.BytesIO(content)
    try:
        with zipfile.ZipFile(buf, "r") as zf:
            names = zf.namelist()

            # Validate manifest
            if "manifest.json" not in names:
                raise HTTPException(status_code=400, detail="Invalid backup: missing manifest.json")

            manifest = json.loads(zf.read("manifest.json"))
            if manifest.get("version") != "1.0":
                raise HTTPException(status_code=400, detail=f"Unsupported backup version: {manifest.get('version')}")

            # Restore database
            db_path = _get_db_path()
            db_name = db_path.name
            if db_name in names:
                # Backup current database first
                if db_path.exists():
                    backup_name = f"{db_path.stem}_pre_restore_{datetime.now().strftime('%Y%m%d%H%M%S')}{db_path.suffix}"
                    shutil.copy2(str(db_path), str(db_path.parent / backup_name))
                    logger.info(f"Current database backed up as {backup_name}")

                with zf.open(db_name) as src:
                    db_path.write_bytes(src.read())
                logger.info(f"Database restored: {db_name}")

            # Restore Amy memory
            if "amy/memory.json" in names:
                amy_dir = Path("data/amy")
                amy_dir.mkdir(parents=True, exist_ok=True)
                with zf.open("amy/memory.json") as src:
                    (amy_dir / "memory.json").write_bytes(src.read())
                logger.info("Amy memory restored")

    except zipfile.BadZipFile:
        raise HTTPException(status_code=400, detail="Invalid ZIP file")

    return {
        "status": "restored",
        "message": "Backup imported successfully. Restart the server to apply changes.",
        "manifest": manifest,
    }
