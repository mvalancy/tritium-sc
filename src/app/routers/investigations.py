# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Investigation API — create, expand, annotate, and close entity investigations.

Implements the ontology research workflow:
  Seed -> Expand -> Filter -> Analyze -> Annotate -> Share
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Body, HTTPException, Query, Request
from loguru import logger

router = APIRouter(prefix="/api/investigations", tags=["investigations"])

# Lazy-init singleton engine
_engine = None
_DB_PATH = Path("data/investigations.db")


def _get_engine():
    """Lazy-initialise the InvestigationEngine singleton."""
    global _engine
    if _engine is None:
        try:
            from engine.tactical.investigation import InvestigationEngine

            # Try to get dossier store for graph expansion
            dossier_store = None
            try:
                from tritium_lib.store.dossiers import DossierStore

                store_path = Path("data/dossiers.db")
                if store_path.exists():
                    dossier_store = DossierStore(store_path)
            except Exception:
                pass

            _DB_PATH.parent.mkdir(parents=True, exist_ok=True)
            _engine = InvestigationEngine(
                db_path=_DB_PATH,
                dossier_store=dossier_store,
            )
            logger.info(f"InvestigationEngine initialised: {_DB_PATH}")
        except Exception as e:
            logger.warning(f"InvestigationEngine init failed: {e}")
            return None
    return _engine


# -- Endpoints -------------------------------------------------------------


@router.post("")
async def create_investigation(
    title: str = Body(...),
    seed_entity_ids: list[str] = Body(default=[]),
    description: str = Body(default=""),
):
    """Create a new investigation seeded with entity IDs."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    if not title or not title.strip():
        raise HTTPException(status_code=400, detail="Title is required")

    inv = engine.create(
        title=title.strip(),
        seed_entity_ids=seed_entity_ids,
        description=description,
    )
    return inv.to_dict()


@router.get("")
async def list_investigations(
    status: Optional[str] = Query(None, description="Filter by status: open, closed, archived"),
    limit: int = Query(50, ge=1, le=500),
    offset: int = Query(0, ge=0),
):
    """List all investigations, optionally filtered by status."""
    engine = _get_engine()
    if engine is None:
        return {"investigations": [], "total": 0}

    investigations = engine.list_investigations(
        status=status, limit=limit, offset=offset,
    )
    result = [inv.to_dict() for inv in investigations]
    return {"investigations": result, "total": len(result)}


@router.get("/{inv_id}")
async def get_investigation(inv_id: str):
    """Get a full investigation with all entities and annotations."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    inv = engine.get(inv_id)
    if inv is None:
        raise HTTPException(status_code=404, detail="Investigation not found")

    result = inv.to_dict()

    # Enrich with dossier summaries if store is available
    if engine._dossier_store is not None:
        entities = []
        for eid in inv.all_entity_ids():
            dossier = engine._dossier_store.get_dossier(eid)
            if dossier:
                entities.append({
                    "dossier_id": eid,
                    "name": dossier.get("name", "Unknown"),
                    "entity_type": dossier.get("entity_type", "unknown"),
                    "threat_level": dossier.get("threat_level", "none"),
                    "confidence": dossier.get("confidence", 0.0),
                    "last_seen": dossier.get("last_seen", 0),
                    "is_seed": eid in inv.seed_entities,
                })
            else:
                entities.append({
                    "dossier_id": eid,
                    "name": "Unknown (deleted?)",
                    "entity_type": "unknown",
                    "threat_level": "none",
                    "confidence": 0.0,
                    "last_seen": 0,
                    "is_seed": eid in inv.seed_entities,
                })
        result["entities"] = entities

    return result


@router.post("/{inv_id}/expand")
async def expand_investigation(
    inv_id: str,
    entity_id: str = Body(...),
    max_hops: int = Body(default=1),
):
    """Expand the investigation graph from an entity via relationship traversal."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    inv = engine.get(inv_id)
    if inv is None:
        raise HTTPException(status_code=404, detail="Investigation not found")

    if inv.status != "open":
        raise HTTPException(status_code=400, detail="Cannot expand a closed investigation")

    newly_discovered = engine.expand(inv_id, entity_id, max_hops=max_hops)
    return {
        "inv_id": inv_id,
        "expanded_from": entity_id,
        "newly_discovered": newly_discovered,
        "count": len(newly_discovered),
    }


@router.post("/{inv_id}/annotate")
async def annotate_investigation(
    inv_id: str,
    entity_id: str = Body(...),
    note: str = Body(...),
    analyst: str = Body(default="system"),
):
    """Add an annotation to an entity within the investigation."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    if not note or not note.strip():
        raise HTTPException(status_code=400, detail="Note is required")

    ann = engine.annotate(inv_id, entity_id, note.strip(), analyst=analyst)
    if ann is None:
        raise HTTPException(status_code=404, detail="Investigation not found")

    return ann.to_dict()


@router.post("/{inv_id}/close")
async def close_investigation(inv_id: str):
    """Close an investigation."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    ok = engine.close(inv_id)
    if not ok:
        raise HTTPException(status_code=404, detail="Investigation not found")

    return {"ok": True, "inv_id": inv_id, "status": "closed"}


@router.post("/{inv_id}/archive")
async def archive_investigation(inv_id: str):
    """Archive an investigation."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    ok = engine.archive(inv_id)
    if not ok:
        raise HTTPException(status_code=404, detail="Investigation not found")

    return {"ok": True, "inv_id": inv_id, "status": "archived"}


@router.post("/{inv_id}/filter/time")
async def filter_by_time(
    inv_id: str,
    start: float = Body(...),
    end: float = Body(...),
):
    """Filter investigation entities by time range."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    if start >= end:
        raise HTTPException(status_code=400, detail="start must be before end")

    filtered = engine.filter_by_time(inv_id, start, end)
    return {"inv_id": inv_id, "filtered_entities": filtered, "count": len(filtered)}


@router.post("/{inv_id}/filter/type")
async def filter_by_type(
    inv_id: str,
    entity_types: list[str] = Body(..., embed=True),
):
    """Filter investigation entities by entity type."""
    engine = _get_engine()
    if engine is None:
        raise HTTPException(status_code=503, detail="Investigation engine unavailable")

    if not entity_types:
        raise HTTPException(status_code=400, detail="At least one entity type is required")

    filtered = engine.filter_by_type(inv_id, entity_types)
    return {"inv_id": inv_id, "filtered_entities": filtered, "count": len(filtered)}
