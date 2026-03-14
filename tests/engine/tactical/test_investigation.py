# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for InvestigationEngine — entity intelligence investigation workflows."""

import tempfile
import time

import pytest

from tritium_lib.store.dossiers import DossierStore

from src.engine.tactical.investigation import (
    Annotation,
    Investigation,
    InvestigationEngine,
)


def _make_store(tmp_path=None):
    """Create a temporary DossierStore."""
    if tmp_path is None:
        tmp = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
        tmp.close()
        return DossierStore(tmp.name)
    return DossierStore(tmp_path / "test_dossiers.db")


def _make_engine(tmp_path=None, dossier_store=None):
    """Create an InvestigationEngine with a temp database."""
    if tmp_path is None:
        tmp = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
        tmp.close()
        db_path = tmp.name
    else:
        db_path = tmp_path / "test_investigations.db"
    return InvestigationEngine(db_path=db_path, dossier_store=dossier_store)


# ---------------------------------------------------------------------------
# Investigation dataclass
# ---------------------------------------------------------------------------

class TestInvestigationDataclass:
    """Tests for the Investigation dataclass."""

    @pytest.mark.unit
    def test_default_values(self):
        inv = Investigation()
        assert inv.inv_id
        assert inv.status == "open"
        assert inv.seed_entities == []
        assert inv.discovered_entities == set()
        assert inv.annotations == []
        assert inv.analyst_notes == ""

    @pytest.mark.unit
    def test_all_entity_ids(self):
        inv = Investigation(
            seed_entities=["a", "b"],
            discovered_entities={"b", "c", "d"},
        )
        assert inv.all_entity_ids() == {"a", "b", "c", "d"}

    @pytest.mark.unit
    def test_to_dict(self):
        inv = Investigation(title="Test", seed_entities=["x"])
        d = inv.to_dict()
        assert d["title"] == "Test"
        assert d["status"] == "open"
        assert d["seed_entities"] == ["x"]
        assert d["entity_count"] == 1
        assert "discovered_entities" in d
        assert "annotations" in d

    @pytest.mark.unit
    def test_annotation_to_dict(self):
        ann = Annotation(entity_id="e1", note="suspicious", analyst="alice")
        d = ann.to_dict()
        assert d["entity_id"] == "e1"
        assert d["note"] == "suspicious"
        assert d["analyst"] == "alice"
        assert d["timestamp"] > 0


# ---------------------------------------------------------------------------
# InvestigationEngine CRUD
# ---------------------------------------------------------------------------

class TestInvestigationEngineCRUD:
    """Basic create, get, list, close, archive operations."""

    @pytest.mark.unit
    def test_create(self):
        engine = _make_engine()
        inv = engine.create("Test Investigation", ["d1", "d2"])
        assert inv.title == "Test Investigation"
        assert inv.seed_entities == ["d1", "d2"]
        assert inv.status == "open"
        engine.close.__wrapped__(engine, inv.inv_id) if hasattr(engine.close, '__wrapped__') else None

    @pytest.mark.unit
    def test_create_with_description(self):
        engine = _make_engine()
        inv = engine.create("Test", ["d1"], description="Important case")
        assert inv.description == "Important case"

    @pytest.mark.unit
    def test_get(self):
        engine = _make_engine()
        inv = engine.create("Fetch Test", ["d1"])
        loaded = engine.get(inv.inv_id)
        assert loaded is not None
        assert loaded.title == "Fetch Test"
        assert loaded.seed_entities == ["d1"]

    @pytest.mark.unit
    def test_get_nonexistent(self):
        engine = _make_engine()
        assert engine.get("nonexistent-uuid") is None

    @pytest.mark.unit
    def test_list_investigations(self):
        engine = _make_engine()
        engine.create("First", ["d1"])
        engine.create("Second", ["d2"])
        engine.create("Third", ["d3"])
        all_inv = engine.list_investigations()
        assert len(all_inv) == 3
        # Newest first
        assert all_inv[0].title == "Third"

    @pytest.mark.unit
    def test_list_by_status(self):
        engine = _make_engine()
        inv1 = engine.create("Open", ["d1"])
        inv2 = engine.create("To Close", ["d2"])
        # Use the engine's close method directly via the DB
        inv2_obj = engine.get(inv2.inv_id)
        inv2_obj.status = "closed"
        engine._save_investigation(inv2_obj)

        open_inv = engine.list_investigations(status="open")
        assert len(open_inv) == 1
        assert open_inv[0].title == "Open"

        closed_inv = engine.list_investigations(status="closed")
        assert len(closed_inv) == 1
        assert closed_inv[0].title == "To Close"

    @pytest.mark.unit
    def test_list_pagination(self):
        engine = _make_engine()
        for i in range(5):
            engine.create(f"Inv {i}", [f"d{i}"])
        page = engine.list_investigations(limit=2, offset=1)
        assert len(page) == 2

    @pytest.mark.unit
    def test_close_investigation(self):
        engine = _make_engine()
        inv = engine.create("Close Me", ["d1"])
        # close() is also the DB close method, so we call it as the investigation close
        ok = engine.close(inv.inv_id)
        assert ok is True
        loaded = engine.get(inv.inv_id)
        assert loaded.status == "closed"

    @pytest.mark.unit
    def test_close_nonexistent(self):
        engine = _make_engine()
        ok = engine.close("no-such-id")
        assert ok is False

    @pytest.mark.unit
    def test_archive(self):
        engine = _make_engine()
        inv = engine.create("Archive Me", ["d1"])
        ok = engine.archive(inv.inv_id)
        assert ok is True
        loaded = engine.get(inv.inv_id)
        assert loaded.status == "archived"


# ---------------------------------------------------------------------------
# Annotations
# ---------------------------------------------------------------------------

class TestAnnotations:
    """Tests for annotating entities within investigations."""

    @pytest.mark.unit
    def test_annotate(self):
        engine = _make_engine()
        inv = engine.create("Annotate Test", ["d1"])
        ann = engine.annotate(inv.inv_id, "d1", "Suspicious behavior", analyst="bob")
        assert ann is not None
        assert ann.note == "Suspicious behavior"
        assert ann.analyst == "bob"
        assert ann.entity_id == "d1"

    @pytest.mark.unit
    def test_annotate_persists(self):
        engine = _make_engine()
        inv = engine.create("Persist Test", ["d1"])
        engine.annotate(inv.inv_id, "d1", "Note 1")
        engine.annotate(inv.inv_id, "d1", "Note 2", analyst="alice")
        loaded = engine.get(inv.inv_id)
        assert len(loaded.annotations) == 2
        assert loaded.annotations[0].note == "Note 1"
        assert loaded.annotations[1].note == "Note 2"
        assert loaded.annotations[1].analyst == "alice"

    @pytest.mark.unit
    def test_annotate_nonexistent_investigation(self):
        engine = _make_engine()
        ann = engine.annotate("no-such-id", "d1", "note")
        assert ann is None


# ---------------------------------------------------------------------------
# Expand (graph traversal)
# ---------------------------------------------------------------------------

class TestExpand:
    """Tests for graph expansion from seed entities."""

    @pytest.mark.unit
    def test_expand_no_store(self):
        engine = _make_engine(dossier_store=None)
        inv = engine.create("No Store", ["d1"])
        result = engine.expand(inv.inv_id, "d1")
        assert result == []

    @pytest.mark.unit
    def test_expand_nonexistent_investigation(self):
        engine = _make_engine()
        result = engine.expand("no-such-id", "d1")
        assert result == []

    @pytest.mark.unit
    def test_expand_shared_source(self):
        """Two dossiers seen by the same camera should be linked."""
        store = _make_store()
        engine = _make_engine(dossier_store=store)

        # Create two dossiers with signals from the same source
        d1 = store.create_dossier("Alpha", entity_type="person")
        d2 = store.create_dossier("Beta", entity_type="person")
        ts = time.time()
        store.add_signal(d1, source="camera_north", signal_type="detection",
                         timestamp=ts)
        store.add_signal(d2, source="camera_north", signal_type="detection",
                         timestamp=ts + 5)

        inv = engine.create("Expand Test", [d1])
        discovered = engine.expand(inv.inv_id, d1)
        assert d2 in discovered

        # Verify it's persisted
        loaded = engine.get(inv.inv_id)
        assert d2 in loaded.discovered_entities
        store.close()

    @pytest.mark.unit
    def test_expand_does_not_rediscover(self):
        """Already-known entities should not be re-discovered."""
        store = _make_store()
        engine = _make_engine(dossier_store=store)

        d1 = store.create_dossier("Alpha", entity_type="person")
        d2 = store.create_dossier("Beta", entity_type="person")
        ts = time.time()
        store.add_signal(d1, source="cam1", signal_type="det", timestamp=ts)
        store.add_signal(d2, source="cam1", signal_type="det", timestamp=ts)

        inv = engine.create("Dedup Test", [d1, d2])
        discovered = engine.expand(inv.inv_id, d1)
        # d2 is already a seed, should not be "discovered"
        assert d2 not in discovered
        store.close()

    @pytest.mark.unit
    def test_expand_multi_hop(self):
        """Multi-hop expansion should find entities further out."""
        store = _make_store()
        engine = _make_engine(dossier_store=store)

        d1 = store.create_dossier("A")
        d2 = store.create_dossier("B")
        d3 = store.create_dossier("C")
        ts = time.time()

        # A and B share source "cam1"
        store.add_signal(d1, source="cam1", signal_type="det", timestamp=ts)
        store.add_signal(d2, source="cam1", signal_type="det", timestamp=ts)
        # B and C share source "cam2" (but not A)
        store.add_signal(d2, source="cam2", signal_type="det", timestamp=ts)
        store.add_signal(d3, source="cam2", signal_type="det", timestamp=ts)

        inv = engine.create("Multi-hop", [d1])
        # 1 hop from A should find B
        discovered_1 = engine.expand(inv.inv_id, d1, max_hops=1)
        assert d2 in discovered_1
        # d3 might or might not be found in 1 hop (depends on co-occurrence)
        # But with 2 hops from A, should find B then C
        inv2 = engine.create("Multi-hop 2", [d1])
        discovered_2 = engine.expand(inv2.inv_id, d1, max_hops=2)
        assert d2 in discovered_2
        assert d3 in discovered_2
        store.close()


# ---------------------------------------------------------------------------
# Filters
# ---------------------------------------------------------------------------

class TestFilters:
    """Tests for time and type filtering."""

    @pytest.mark.unit
    def test_filter_by_time(self):
        store = _make_store()
        engine = _make_engine(dossier_store=store)

        d1 = store.create_dossier("Early")
        d2 = store.create_dossier("Late")
        base = time.time()
        store.add_signal(d1, source="s", signal_type="t", timestamp=base - 100)
        store.add_signal(d2, source="s", signal_type="t", timestamp=base + 100)

        inv = engine.create("Filter Time", [d1, d2])
        # Only d1 should match the early window
        filtered = engine.filter_by_time(inv.inv_id, base - 200, base - 50)
        assert d1 in filtered
        assert d2 not in filtered
        store.close()

    @pytest.mark.unit
    def test_filter_by_time_nonexistent(self):
        engine = _make_engine()
        result = engine.filter_by_time("no-id", 0, 999)
        assert result == []

    @pytest.mark.unit
    def test_filter_by_type(self):
        store = _make_store()
        engine = _make_engine(dossier_store=store)

        d1 = store.create_dossier("Phone", entity_type="device")
        d2 = store.create_dossier("John", entity_type="person")
        d3 = store.create_dossier("Truck", entity_type="vehicle")

        inv = engine.create("Filter Type", [d1, d2, d3])
        filtered = engine.filter_by_type(inv.inv_id, ["person", "vehicle"])
        assert d2 in filtered
        assert d3 in filtered
        assert d1 not in filtered
        store.close()

    @pytest.mark.unit
    def test_filter_by_type_nonexistent(self):
        engine = _make_engine()
        result = engine.filter_by_type("no-id", ["person"])
        assert result == []

    @pytest.mark.unit
    def test_filter_no_store_returns_all(self):
        engine = _make_engine(dossier_store=None)
        inv = engine.create("No Store", ["d1", "d2"])
        filtered = engine.filter_by_type(inv.inv_id, ["person"])
        assert set(filtered) == {"d1", "d2"}


# ---------------------------------------------------------------------------
# Auto-escalation
# ---------------------------------------------------------------------------

class TestAutoEscalation:
    """Tests for auto-creating investigations on high threat level."""

    @pytest.mark.unit
    def test_auto_investigate_high(self):
        engine = _make_engine()
        inv = engine.auto_investigate_threat("d1", "high", "Suspect Alpha")
        assert inv is not None
        assert "high" in inv.title
        assert "Suspect Alpha" in inv.title
        assert "d1" in inv.seed_entities
        assert inv.status == "open"

    @pytest.mark.unit
    def test_auto_investigate_critical(self):
        engine = _make_engine()
        inv = engine.auto_investigate_threat("d2", "critical")
        assert inv is not None
        assert "critical" in inv.title

    @pytest.mark.unit
    def test_auto_investigate_low_ignored(self):
        engine = _make_engine()
        inv = engine.auto_investigate_threat("d1", "low")
        assert inv is None

    @pytest.mark.unit
    def test_auto_investigate_none_ignored(self):
        engine = _make_engine()
        inv = engine.auto_investigate_threat("d1", "none")
        assert inv is None

    @pytest.mark.unit
    def test_auto_investigate_no_duplicate(self):
        """Should not create a duplicate investigation for the same entity."""
        engine = _make_engine()
        inv1 = engine.auto_investigate_threat("d1", "high")
        inv2 = engine.auto_investigate_threat("d1", "high")
        assert inv1 is not None
        assert inv2 is None  # Already has an open investigation


# ---------------------------------------------------------------------------
# API router tests
# ---------------------------------------------------------------------------

class TestInvestigationAPI:
    """Tests for the investigation REST API endpoints."""

    @pytest.fixture(autouse=True)
    def setup_app(self, tmp_path):
        """Set up a FastAPI test app with the investigations router."""
        from fastapi import FastAPI
        from fastapi.testclient import TestClient
        from app.routers import investigations as inv_mod

        # Reset singleton
        inv_mod._engine = None

        store = DossierStore(tmp_path / "api_dossiers.db")
        engine = InvestigationEngine(
            db_path=tmp_path / "api_investigations.db",
            dossier_store=store,
        )
        inv_mod._engine = engine

        app = FastAPI()
        app.include_router(inv_mod.router)
        self.client = TestClient(app)
        self.engine = engine
        self.store = store

        yield

        inv_mod._engine = None
        store.close()

    @pytest.mark.unit
    def test_create_investigation(self):
        resp = self.client.post("/api/investigations", json={
            "title": "API Test",
            "seed_entity_ids": ["d1", "d2"],
            "description": "Testing the API",
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["title"] == "API Test"
        assert data["status"] == "open"
        assert data["seed_entities"] == ["d1", "d2"]

    @pytest.mark.unit
    def test_create_empty_title_fails(self):
        resp = self.client.post("/api/investigations", json={
            "title": "  ",
            "seed_entity_ids": [],
        })
        assert resp.status_code == 400

    @pytest.mark.unit
    def test_list_investigations(self):
        self.client.post("/api/investigations", json={"title": "A"})
        self.client.post("/api/investigations", json={"title": "B"})
        resp = self.client.get("/api/investigations")
        assert resp.status_code == 200
        data = resp.json()
        assert data["total"] == 2
        assert len(data["investigations"]) == 2

    @pytest.mark.unit
    def test_list_by_status(self):
        r1 = self.client.post("/api/investigations", json={"title": "Open"})
        inv_id = r1.json()["inv_id"]
        self.client.post(f"/api/investigations/{inv_id}/close")

        resp = self.client.get("/api/investigations?status=closed")
        data = resp.json()
        assert data["total"] == 1
        assert data["investigations"][0]["status"] == "closed"

    @pytest.mark.unit
    def test_get_investigation(self):
        r = self.client.post("/api/investigations", json={
            "title": "Get Test",
            "seed_entity_ids": ["d1"],
        })
        inv_id = r.json()["inv_id"]
        resp = self.client.get(f"/api/investigations/{inv_id}")
        assert resp.status_code == 200
        data = resp.json()
        assert data["title"] == "Get Test"

    @pytest.mark.unit
    def test_get_nonexistent(self):
        resp = self.client.get("/api/investigations/no-such-id")
        assert resp.status_code == 404

    @pytest.mark.unit
    def test_annotate(self):
        r = self.client.post("/api/investigations", json={"title": "Ann Test"})
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/annotate", json={
            "entity_id": "d1",
            "note": "Seen at north gate",
            "analyst": "bob",
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data["note"] == "Seen at north gate"
        assert data["analyst"] == "bob"

    @pytest.mark.unit
    def test_annotate_empty_note_fails(self):
        r = self.client.post("/api/investigations", json={"title": "Empty"})
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/annotate", json={
            "entity_id": "d1",
            "note": "  ",
        })
        assert resp.status_code == 400

    @pytest.mark.unit
    def test_close(self):
        r = self.client.post("/api/investigations", json={"title": "Close"})
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/close")
        assert resp.status_code == 200
        assert resp.json()["status"] == "closed"

    @pytest.mark.unit
    def test_close_nonexistent(self):
        resp = self.client.post("/api/investigations/no-such-id/close")
        assert resp.status_code == 404

    @pytest.mark.unit
    def test_archive(self):
        r = self.client.post("/api/investigations", json={"title": "Archive"})
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/archive")
        assert resp.status_code == 200
        assert resp.json()["status"] == "archived"

    @pytest.mark.unit
    def test_expand(self):
        # Create dossiers with shared source
        d1 = self.store.create_dossier("Alpha")
        d2 = self.store.create_dossier("Beta")
        ts = time.time()
        self.store.add_signal(d1, source="cam1", signal_type="det", timestamp=ts)
        self.store.add_signal(d2, source="cam1", signal_type="det", timestamp=ts)

        r = self.client.post("/api/investigations", json={
            "title": "Expand",
            "seed_entity_ids": [d1],
        })
        inv_id = r.json()["inv_id"]

        resp = self.client.post(f"/api/investigations/{inv_id}/expand", json={
            "entity_id": d1,
        })
        assert resp.status_code == 200
        data = resp.json()
        assert d2 in data["newly_discovered"]

    @pytest.mark.unit
    def test_expand_closed_fails(self):
        r = self.client.post("/api/investigations", json={"title": "Closed"})
        inv_id = r.json()["inv_id"]
        self.client.post(f"/api/investigations/{inv_id}/close")
        resp = self.client.post(f"/api/investigations/{inv_id}/expand", json={
            "entity_id": "d1",
        })
        assert resp.status_code == 400

    @pytest.mark.unit
    def test_filter_by_time(self):
        d1 = self.store.create_dossier("Early")
        base = time.time()
        self.store.add_signal(d1, source="s", signal_type="t", timestamp=base)

        r = self.client.post("/api/investigations", json={
            "title": "Time Filter",
            "seed_entity_ids": [d1],
        })
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/filter/time", json={
            "start": base - 10,
            "end": base + 10,
        })
        assert resp.status_code == 200
        assert d1 in resp.json()["filtered_entities"]

    @pytest.mark.unit
    def test_filter_by_time_bad_range(self):
        r = self.client.post("/api/investigations", json={"title": "Bad"})
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/filter/time", json={
            "start": 100,
            "end": 50,
        })
        assert resp.status_code == 400

    @pytest.mark.unit
    def test_filter_by_type(self):
        d1 = self.store.create_dossier("Dev", entity_type="device")
        d2 = self.store.create_dossier("Per", entity_type="person")

        r = self.client.post("/api/investigations", json={
            "title": "Type Filter",
            "seed_entity_ids": [d1, d2],
        })
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/filter/type", json={
            "entity_types": ["person"],
        })
        assert resp.status_code == 200
        data = resp.json()
        assert d2 in data["filtered_entities"]
        assert d1 not in data["filtered_entities"]

    @pytest.mark.unit
    def test_filter_by_type_empty_fails(self):
        r = self.client.post("/api/investigations", json={"title": "Empty"})
        inv_id = r.json()["inv_id"]
        resp = self.client.post(f"/api/investigations/{inv_id}/filter/type", json={
            "entity_types": [],
        })
        assert resp.status_code == 400
