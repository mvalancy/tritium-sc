"""Search endpoints for people/vehicle lookup and similarity search."""

import json
from datetime import datetime
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import FileResponse
from pydantic import BaseModel
from loguru import logger

from app.config import settings

router = APIRouter(prefix="/api/search", tags=["search"])

# Storage paths
THUMBNAILS_DIR = settings.recordings_path / ".cache" / "thumbnails"
VECTORS_PATH = settings.recordings_path / ".cache" / "vectors.json"
LABELS_PATH = settings.recordings_path / ".cache" / "labels.json"
MERGES_PATH = settings.recordings_path / ".cache" / "merges.json"
FEEDBACK_PATH = settings.recordings_path / ".cache" / "feedback.json"


class MergeRequest(BaseModel):
    """Request to merge multiple detections as same object."""
    primary_id: str  # The ID to keep
    duplicate_ids: list[str]  # IDs to merge into primary


class LabelRequest(BaseModel):
    """Request to label an object."""
    thumbnail_id: str
    label: str  # User-provided name like "my car", "mailman", etc.
    notes: Optional[str] = None


class FeedbackRequest(BaseModel):
    """User feedback for a detection."""
    thumbnail_id: str
    feedback_type: str  # "correct", "wrong_type", "wrong_merge", "not_real"
    correct_type: Optional[str] = None  # If wrong_type
    notes: Optional[str] = None


def load_json(path: Path, default=None):
    """Load JSON file or return default."""
    if path.exists():
        with open(path) as f:
            return json.load(f)
    return default or {}


def save_json(path: Path, data):
    """Save data to JSON file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as f:
        json.dump(data, f, indent=2, default=str)


def get_vector_store():
    """Get the vector store instance."""
    from app.ai.embeddings import VectorStore
    return VectorStore(VECTORS_PATH)


@router.get("/people")
async def list_people(
    date_from: Optional[str] = Query(None, description="Start date YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="End date YYYY-MM-DD"),
    channel: Optional[int] = Query(None, description="Filter by channel"),
    limit: int = Query(50, le=200),
    offset: int = Query(0),
):
    """List all detected people with thumbnails."""
    store = get_vector_store()
    results = store.get_all(
        object_type="person",
        date_from=date_from,
        date_to=date_to,
        limit=limit,
        offset=offset,
    )

    # Add labels and merge info
    labels = load_json(LABELS_PATH, {})
    merges = load_json(MERGES_PATH, {"merged_into": {}, "groups": {}})

    enriched = []
    seen_groups = set()

    for r in results:
        tid = r.get("thumbnail_id", "")

        # Skip if merged into another
        if tid in merges.get("merged_into", {}):
            primary = merges["merged_into"][tid]
            if primary in seen_groups:
                continue
            seen_groups.add(primary)
            r = next((x for x in results if x.get("thumbnail_id") == primary), r)
            tid = primary

        # Add label if exists
        r["label"] = labels.get(tid, {}).get("label")
        r["notes"] = labels.get(tid, {}).get("notes")

        # Add URL for thumbnail
        r["thumbnail_url"] = f"/api/search/thumbnail/{tid}"

        # Count how many are merged into this
        r["merged_count"] = len(merges.get("groups", {}).get(tid, []))

        if channel and r.get("channel") != channel:
            continue

        enriched.append(r)

    return {
        "total": len(enriched),
        "items": enriched,
        "offset": offset,
        "limit": limit,
    }


@router.get("/vehicles")
async def list_vehicles(
    date_from: Optional[str] = Query(None, description="Start date YYYY-MM-DD"),
    date_to: Optional[str] = Query(None, description="End date YYYY-MM-DD"),
    channel: Optional[int] = Query(None, description="Filter by channel"),
    limit: int = Query(50, le=200),
    offset: int = Query(0),
):
    """List all detected vehicles with thumbnails."""
    store = get_vector_store()

    # Get cars, trucks, etc.
    results = []
    for vtype in ["car", "truck", "bus", "motorcycle", "bicycle"]:
        results.extend(store.get_all(
            object_type=vtype,
            date_from=date_from,
            date_to=date_to,
            limit=limit * 2,  # Get extra since we'll merge
        ))

    # Sort by timestamp
    results.sort(key=lambda x: x.get("timestamp", ""), reverse=True)

    # Add labels and merge info
    labels = load_json(LABELS_PATH, {})
    merges = load_json(MERGES_PATH, {"merged_into": {}, "groups": {}})

    enriched = []
    seen_groups = set()

    for r in results:
        tid = r.get("thumbnail_id", "")

        # Skip if merged into another
        if tid in merges.get("merged_into", {}):
            primary = merges["merged_into"][tid]
            if primary in seen_groups:
                continue
            seen_groups.add(primary)
            # Use primary's data
            r = next((x for x in results if x.get("thumbnail_id") == primary), r)
            tid = primary

        r["label"] = labels.get(tid, {}).get("label")
        r["notes"] = labels.get(tid, {}).get("notes")
        r["thumbnail_url"] = f"/api/search/thumbnail/{tid}"
        r["merged_count"] = len(merges.get("groups", {}).get(tid, []))

        if channel and r.get("channel") != channel:
            continue

        enriched.append(r)

    return {
        "total": len(enriched),
        "items": enriched[:limit],
        "offset": offset,
        "limit": limit,
    }


@router.get("/thumbnail/{thumbnail_id}")
async def get_thumbnail(thumbnail_id: str):
    """Get a thumbnail image."""
    # Search in all category directories
    for category in ["person", "vehicle", "animal"]:
        path = THUMBNAILS_DIR / category / f"{thumbnail_id}.jpg"
        if path.exists():
            return FileResponse(path, media_type="image/jpeg")

    raise HTTPException(status_code=404, detail="Thumbnail not found")


@router.get("/similar/{thumbnail_id}")
async def find_similar(
    thumbnail_id: str,
    limit: int = Query(10, le=50),
):
    """Find similar objects to a given thumbnail."""
    store = get_vector_store()
    results = store.search_by_thumbnail(thumbnail_id, k=limit)

    return {
        "query_id": thumbnail_id,
        "similar": [
            {
                **meta,
                "similarity": round(score, 3),
                "thumbnail_url": f"/api/search/thumbnail/{meta.get('thumbnail_id', '')}",
            }
            for meta, score in results
        ],
    }


@router.post("/merge")
async def merge_detections(request: MergeRequest):
    """Merge multiple detections as the same object.

    User feedback: 'These are all the same car/person'.
    """
    merges = load_json(MERGES_PATH, {"merged_into": {}, "groups": {}})

    # Add each duplicate to the merged_into map
    for dup_id in request.duplicate_ids:
        merges["merged_into"][dup_id] = request.primary_id

    # Update groups
    if request.primary_id not in merges["groups"]:
        merges["groups"][request.primary_id] = []
    merges["groups"][request.primary_id].extend(request.duplicate_ids)

    save_json(MERGES_PATH, merges)

    # Also log as feedback for future training
    feedback = load_json(FEEDBACK_PATH, {"items": []})
    feedback["items"].append({
        "type": "merge",
        "primary_id": request.primary_id,
        "duplicate_ids": request.duplicate_ids,
        "timestamp": datetime.now().isoformat(),
    })
    save_json(FEEDBACK_PATH, feedback)

    logger.info(f"Merged {len(request.duplicate_ids)} detections into {request.primary_id}")

    return {
        "status": "success",
        "primary_id": request.primary_id,
        "merged_count": len(request.duplicate_ids),
    }


@router.post("/unmerge/{thumbnail_id}")
async def unmerge_detection(thumbnail_id: str):
    """Remove a detection from its merge group."""
    merges = load_json(MERGES_PATH, {"merged_into": {}, "groups": {}})

    if thumbnail_id in merges["merged_into"]:
        primary = merges["merged_into"].pop(thumbnail_id)
        if primary in merges["groups"]:
            merges["groups"][primary] = [
                x for x in merges["groups"][primary] if x != thumbnail_id
            ]
        save_json(MERGES_PATH, merges)
        return {"status": "success", "unmerged": thumbnail_id}

    raise HTTPException(status_code=404, detail="Not in a merge group")


@router.post("/label")
async def label_object(request: LabelRequest):
    """Add a label to a detected object.

    Examples: 'my car', 'mailman', 'neighbor bob', 'delivery truck'.
    """
    labels = load_json(LABELS_PATH, {})

    labels[request.thumbnail_id] = {
        "label": request.label,
        "notes": request.notes,
        "labeled_at": datetime.now().isoformat(),
    }

    save_json(LABELS_PATH, labels)

    # Log for training
    feedback = load_json(FEEDBACK_PATH, {"items": []})
    feedback["items"].append({
        "type": "label",
        "thumbnail_id": request.thumbnail_id,
        "label": request.label,
        "timestamp": datetime.now().isoformat(),
    })
    save_json(FEEDBACK_PATH, feedback)

    logger.info(f"Labeled {request.thumbnail_id} as '{request.label}'")

    return {"status": "success", "thumbnail_id": request.thumbnail_id}


@router.delete("/label/{thumbnail_id}")
async def remove_label(thumbnail_id: str):
    """Remove a label from an object."""
    labels = load_json(LABELS_PATH, {})

    if thumbnail_id in labels:
        del labels[thumbnail_id]
        save_json(LABELS_PATH, labels)
        return {"status": "success"}

    raise HTTPException(status_code=404, detail="Label not found")


@router.post("/feedback")
async def submit_feedback(request: FeedbackRequest):
    """Submit feedback about a detection.

    For reinforcement learning and improving the system.
    """
    feedback = load_json(FEEDBACK_PATH, {"items": []})

    feedback["items"].append({
        "type": "detection_feedback",
        "thumbnail_id": request.thumbnail_id,
        "feedback_type": request.feedback_type,
        "correct_type": request.correct_type,
        "notes": request.notes,
        "timestamp": datetime.now().isoformat(),
    })

    save_json(FEEDBACK_PATH, feedback)

    logger.info(f"Feedback received for {request.thumbnail_id}: {request.feedback_type}")

    return {"status": "success"}


@router.get("/feedback/stats")
async def get_feedback_stats():
    """Get statistics about user feedback."""
    feedback = load_json(FEEDBACK_PATH, {"items": []})
    merges = load_json(MERGES_PATH, {"merged_into": {}, "groups": {}})
    labels = load_json(LABELS_PATH, {})

    items = feedback.get("items", [])

    return {
        "total_feedback": len(items),
        "total_labels": len(labels),
        "total_merge_groups": len(merges.get("groups", {})),
        "total_merged_items": len(merges.get("merged_into", {})),
        "feedback_by_type": {
            ftype: sum(1 for i in items if i.get("type") == ftype)
            for ftype in set(i.get("type") for i in items)
        },
    }


@router.get("/stats")
async def get_search_stats():
    """Get overall search/detection statistics."""
    store = get_vector_store()
    stats = store.stats()

    merges = load_json(MERGES_PATH, {"merged_into": {}, "groups": {}})
    labels = load_json(LABELS_PATH, {})

    # Calculate unique after merges
    unique_counts = {}
    for obj_type, count in stats.get("by_type", {}).items():
        # Count how many are primary (not merged into others)
        merged = sum(1 for tid, primary in merges.get("merged_into", {}).items()
                    if any(m.get("object_type") == obj_type
                          for m in store.get_all(object_type=obj_type)))
        unique_counts[obj_type] = count - merged

    return {
        "total_detections": stats.get("total", 0),
        "by_type": stats.get("by_type", {}),
        "unique_after_merge": unique_counts,
        "labeled_count": len(labels),
        "merge_groups": len(merges.get("groups", {})),
    }
