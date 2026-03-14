# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for DashboardLayoutManager — save/load panel arrangements."""

import json
import sys
import tempfile
from pathlib import Path

import pytest

_sc_root = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(_sc_root / "src"))

from engine.tactical.dashboard_layouts import (
    DashboardLayoutManager,
    DashboardLayout,
    PanelConfig,
    DEFAULT_LAYOUTS,
)


@pytest.fixture
def mgr():
    """Create an in-memory DashboardLayoutManager."""
    return DashboardLayoutManager()


@pytest.fixture
def persistent_mgr(tmp_path):
    """Create a persistent DashboardLayoutManager."""
    return DashboardLayoutManager(storage_path=tmp_path / "layouts.json")


# -- Default layouts -------------------------------------------------------

class TestDefaults:
    def test_default_layouts_loaded(self, mgr):
        layouts = mgr.list_layouts()
        names = {l["name"] for l in layouts}
        assert "surveillance" in names
        assert "battle" in names
        assert "analysis" in names
        assert "minimal" in names

    def test_load_default_layout(self, mgr):
        layout = mgr.load("surveillance")
        assert layout is not None
        assert layout["name"] == "surveillance"
        assert len(layout["panels"]) > 0

    def test_default_layout_count(self, mgr):
        layouts = mgr.list_layouts()
        assert len(layouts) >= len(DEFAULT_LAYOUTS)


# -- Save/load tests -------------------------------------------------------

class TestSaveLoad:
    def test_save_layout(self, mgr):
        result = mgr.save(
            name="patrol",
            user="operator1",
            description="Night patrol config",
            panels=[
                {"panel_id": "map", "visible": True, "order": 0},
                {"panel_id": "targets", "visible": True, "order": 1},
            ],
        )
        assert result["name"] == "patrol"
        assert result["user"] == "operator1"
        assert result["panel_count"] == 2

    def test_load_saved_layout(self, mgr):
        mgr.save(
            name="custom",
            user="admin",
            panels=[{"panel_id": "map", "visible": True}],
        )
        layout = mgr.load("custom", user="admin")
        assert layout is not None
        assert layout["name"] == "custom"
        assert len(layout["panels"]) == 1

    def test_load_nonexistent(self, mgr):
        result = mgr.load("nonexistent", user="nobody")
        assert result is None

    def test_load_falls_back_to_default_user(self, mgr):
        mgr.save(name="shared", user="default", panels=[])
        layout = mgr.load("shared", user="other_user")
        assert layout is not None

    def test_update_existing(self, mgr):
        mgr.save(name="test", panels=[{"panel_id": "a"}])
        mgr.save(name="test", panels=[{"panel_id": "b"}, {"panel_id": "c"}])
        layout = mgr.load("test")
        assert len(layout["panels"]) == 2

    def test_save_with_map_settings(self, mgr):
        result = mgr.save(
            name="geo",
            map_settings={"zoom": 15, "center": [33.45, -112.07]},
        )
        layout = mgr.load("geo")
        assert layout["map_settings"]["zoom"] == 15

    def test_save_with_panel_details(self, mgr):
        result = mgr.save(
            name="detailed",
            panels=[{
                "panel_id": "targets",
                "visible": True,
                "position": {"x": 100, "y": 50},
                "size": {"width": 600, "height": 400},
                "order": 1,
                "collapsed": False,
                "settings": {"sort_by": "distance"},
            }],
        )
        layout = mgr.load("detailed")
        panel = layout["panels"][0]
        assert panel["panel_id"] == "targets"
        assert panel["position"] == {"x": 100, "y": 50}
        assert panel["size"]["width"] == 600
        assert panel["settings"]["sort_by"] == "distance"


# -- List/filter tests -----------------------------------------------------

class TestListing:
    def test_list_all(self, mgr):
        mgr.save(name="a", user="user1", panels=[])
        mgr.save(name="b", user="user2", panels=[])
        layouts = mgr.list_layouts()
        names = {l["name"] for l in layouts}
        assert "a" in names
        assert "b" in names

    def test_list_by_user(self, mgr):
        mgr.save(name="mine", user="user1", panels=[])
        mgr.save(name="theirs", user="user2", panels=[])
        layouts = mgr.list_layouts(user="user1")
        names = {l["name"] for l in layouts}
        assert "mine" in names
        # Should also include default layouts
        assert "surveillance" in names

    def test_list_sorted_by_name(self, mgr):
        mgr.save(name="zzz", panels=[])
        mgr.save(name="aaa", panels=[])
        layouts = mgr.list_layouts()
        names = [l["name"] for l in layouts]
        assert names == sorted(names)


# -- Delete tests ----------------------------------------------------------

class TestDelete:
    def test_delete_layout(self, mgr):
        mgr.save(name="temp", panels=[])
        assert mgr.delete("temp") is True
        assert mgr.load("temp") is None

    def test_delete_nonexistent(self, mgr):
        assert mgr.delete("nonexistent") is False


# -- Duplicate tests -------------------------------------------------------

class TestDuplicate:
    def test_duplicate(self, mgr):
        mgr.save(
            name="original",
            panels=[{"panel_id": "map"}],
            map_settings={"zoom": 10},
        )
        result = mgr.duplicate("original", "copy")
        assert result is not None
        assert result["name"] == "copy"
        assert result["description"] == "Copy of original"

        copy = mgr.load("copy")
        assert len(copy["panels"]) == 1
        assert copy["map_settings"]["zoom"] == 10

    def test_duplicate_nonexistent(self, mgr):
        result = mgr.duplicate("nonexistent", "copy")
        assert result is None


# -- Persistence tests -----------------------------------------------------

class TestPersistence:
    def test_save_to_file(self, persistent_mgr, tmp_path):
        persistent_mgr.save(name="persistent", panels=[{"panel_id": "map"}])
        file_path = tmp_path / "layouts.json"
        assert file_path.exists()
        data = json.loads(file_path.read_text())
        assert "default:persistent" in data

    def test_reload_from_file(self, tmp_path):
        path = tmp_path / "layouts.json"
        mgr1 = DashboardLayoutManager(storage_path=path)
        mgr1.save(name="persisted", panels=[{"panel_id": "targets"}])

        mgr2 = DashboardLayoutManager(storage_path=path)
        layout = mgr2.load("persisted")
        assert layout is not None
        assert layout["name"] == "persisted"


# -- PanelConfig dataclass tests -------------------------------------------

class TestPanelConfig:
    def test_to_dict(self):
        config = PanelConfig(
            panel_id="map",
            visible=True,
            position={"x": 10, "y": 20},
            size={"width": 800, "height": 600},
            order=0,
        )
        d = config.to_dict()
        assert d["panel_id"] == "map"
        assert d["position"] == {"x": 10, "y": 20}
        assert d["size"]["width"] == 800


# -- DashboardLayout dataclass tests ---------------------------------------

class TestDashboardLayout:
    def test_to_dict(self):
        layout = DashboardLayout(
            name="test",
            user="admin",
            description="Test layout",
            panels=[PanelConfig(panel_id="map")],
        )
        d = layout.to_dict()
        assert d["name"] == "test"
        assert d["panel_count"] == 1
        assert d["user"] == "admin"
