# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for Investigation auto-escalation."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "src"))

from engine.tactical.investigation import InvestigationEngine


class _MockDossierStore:
    """Mock dossier store for testing escalation."""

    def __init__(self):
        self._dossiers = {}

    def add_dossier(self, dossier_id, threat_level="unknown"):
        self._dossiers[dossier_id] = {
            "dossier_id": dossier_id,
            "threat_level": threat_level,
        }

    def get_dossier(self, dossier_id):
        return self._dossiers.get(dossier_id)


class TestInvestigationEscalation:
    """Test auto-escalation of investigations."""

    def test_no_escalation_below_threshold(self, tmp_path):
        store = _MockDossierStore()
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
            dossier_store=store,
        )

        # Create investigation with 3 medium entities (below threshold of 5)
        for i in range(3):
            store.add_dossier(f"entity_{i}", "medium")

        inv = engine.create("Test", [f"entity_{i}" for i in range(3)])
        result = engine.check_escalation(inv.inv_id)
        assert result is False

        # Verify title unchanged
        loaded = engine.get(inv.inv_id)
        assert not loaded.title.startswith("PRIORITY:")

    def test_escalation_at_threshold(self, tmp_path):
        store = _MockDossierStore()
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
            dossier_store=store,
        )

        # Create investigation with 5 medium entities
        seeds = []
        for i in range(5):
            eid = f"entity_{i}"
            store.add_dossier(eid, "medium")
            seeds.append(eid)

        inv = engine.create("Test Investigation", seeds)
        result = engine.check_escalation(inv.inv_id)
        assert result is True

        # Verify title updated
        loaded = engine.get(inv.inv_id)
        assert loaded.title.startswith("PRIORITY:")
        assert len(loaded.annotations) > 0

    def test_mixed_threat_levels(self, tmp_path):
        store = _MockDossierStore()
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
            dossier_store=store,
        )

        # Mix of threat levels
        store.add_dossier("e1", "medium")
        store.add_dossier("e2", "high")
        store.add_dossier("e3", "critical")
        store.add_dossier("e4", "medium")
        store.add_dossier("e5", "high")
        store.add_dossier("e6", "low")  # This one doesn't count
        store.add_dossier("e7", "unknown")  # This one doesn't count

        seeds = [f"e{i}" for i in range(1, 8)]
        inv = engine.create("Mixed Threats", seeds)
        result = engine.check_escalation(inv.inv_id)
        assert result is True  # 5 medium+ entities

    def test_no_double_escalation(self, tmp_path):
        store = _MockDossierStore()
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
            dossier_store=store,
        )

        for i in range(6):
            store.add_dossier(f"e{i}", "high")

        inv = engine.create("Test", [f"e{i}" for i in range(6)])
        assert engine.check_escalation(inv.inv_id) is True
        assert engine.check_escalation(inv.inv_id) is False  # Already escalated

    def test_escalation_callback(self, tmp_path):
        store = _MockDossierStore()
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
            dossier_store=store,
        )

        for i in range(5):
            store.add_dossier(f"e{i}", "critical")

        received = []

        def on_escalate(inv, count):
            received.append((inv.inv_id, count))

        inv = engine.create("Callback Test", [f"e{i}" for i in range(5)])
        engine.check_escalation(inv.inv_id, on_escalate=on_escalate)

        assert len(received) == 1
        assert received[0][1] == 5

    def test_no_store_no_escalation(self, tmp_path):
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
            dossier_store=None,
        )
        inv = engine.create("No Store", ["e1", "e2", "e3", "e4", "e5"])
        result = engine.check_escalation(inv.inv_id)
        assert result is False

    def test_nonexistent_investigation(self, tmp_path):
        engine = InvestigationEngine(
            db_path=str(tmp_path / "inv.db"),
        )
        result = engine.check_escalation("nonexistent_id")
        assert result is False
