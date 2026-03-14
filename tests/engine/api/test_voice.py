# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for voice command API router."""
import pytest

from app.routers.voice import _match_command


class TestCommandMatching:
    """Tests for voice command pattern matching."""

    def test_demo_start(self):
        action, groups = _match_command("start demo")
        assert action == "demo_start"

    def test_demo_stop(self):
        action, groups = _match_command("stop demo")
        assert action == "demo_stop"

    def test_begin_demo(self):
        action, groups = _match_command("begin demo")
        assert action == "demo_start"

    def test_panel_toggle(self):
        action, groups = _match_command("open fleet panel")
        assert action == "panel_toggle"
        assert groups[0] == "fleet"

    def test_panel_close(self):
        action, groups = _match_command("close alerts panel")
        assert action == "panel_close"
        assert groups[0] == "alerts"

    def test_target_search(self):
        action, groups = _match_command("find target alpha rover")
        assert action == "target_search"
        assert "alpha rover" in groups[0]

    def test_where_is(self):
        action, groups = _match_command("where is the drone")
        assert action == "target_search"
        assert "the drone" in groups[0]

    def test_amy_chat(self):
        action, groups = _match_command("amy what is the threat level")
        assert action == "amy_chat"
        assert "what is the threat level" in groups[0]

    def test_hey_amy(self):
        action, groups = _match_command("hey amy, how many targets")
        assert action == "amy_chat"

    def test_zoom_in(self):
        action, _ = _match_command("zoom in")
        assert action == "map_zoom_in"

    def test_zoom_out(self):
        action, _ = _match_command("zoom out")
        assert action == "map_zoom_out"

    def test_system_status(self):
        action, _ = _match_command("status")
        assert action == "system_status"

    def test_sitrep(self):
        action, _ = _match_command("sitrep")
        assert action == "system_status"

    def test_mute(self):
        action, _ = _match_command("mute")
        assert action == "audio_mute"

    def test_unmute(self):
        action, _ = _match_command("unmute")
        assert action == "audio_unmute"

    def test_unknown_falls_to_amy(self):
        """Unrecognized commands become Amy chat messages."""
        action, groups = _match_command("tell me about the weather")
        assert action == "amy_chat"
        assert "tell me about the weather" in groups[0]

    def test_case_insensitive(self):
        action, _ = _match_command("START DEMO")
        assert action == "demo_start"

    def test_whitespace_handling(self):
        action, _ = _match_command("  zoom in  ")
        assert action == "map_zoom_in"
