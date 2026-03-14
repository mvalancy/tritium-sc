# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for Target Handoff Tracking."""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "src"))

from engine.tactical.target_handoff import (
    HandoffTracker,
    HandoffEvent,
)


class TestHandoffTracker:
    """Test the handoff tracker."""

    def test_init(self):
        tracker = HandoffTracker()
        assert tracker.get_status()["sensor_count"] == 0
        assert tracker.get_status()["total_handoffs"] == 0

    def test_visibility_update(self):
        tracker = HandoffTracker()
        result = tracker.update_visibility(
            sensor_id="cam_a",
            target_id="person_1",
            position=(10.0, 20.0),
            target_type="person",
            timestamp=100.0,
        )
        # First appearance, no departure to match
        assert result is None
        coverage = tracker.get_sensor_coverage()
        assert "cam_a" in coverage
        assert coverage["cam_a"]["visible_count"] == 1

    def test_same_sensor_no_handoff(self):
        tracker = HandoffTracker()
        # Same target seen at same sensor — no handoff
        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.update_visibility("cam_a", "person_1", timestamp=101.0)
        assert tracker.get_status()["total_handoffs"] == 0

    def test_handoff_detection(self):
        tracker = HandoffTracker(visibility_timeout=5.0)

        # Target seen at camera A
        tracker.update_visibility("cam_a", "person_1", (10.0, 20.0), "person", 100.0)

        # Target leaves camera A (no update for visibility_timeout)
        departed = tracker.check_departures(timestamp=106.0)
        assert "person_1" in departed

        # Target appears at camera B
        handoff = tracker.update_visibility("cam_b", "person_1", (30.0, 20.0), "person", 108.0)
        assert handoff is not None
        assert handoff.target_id == "person_1"
        assert handoff.from_sensor == "cam_a"
        assert handoff.to_sensor == "cam_b"
        assert handoff.gap_seconds == pytest.approx(2.0, abs=0.1)
        assert handoff.confidence > 0.0

    def test_handoff_gap_too_large(self):
        tracker = HandoffTracker(max_gap=10.0, visibility_timeout=5.0)

        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.check_departures(timestamp=106.0)

        # Arrival too late (gap > max_gap)
        handoff = tracker.update_visibility("cam_b", "person_1", timestamp=120.0)
        assert handoff is None

    def test_handoff_callback(self):
        received = []

        def on_handoff(event):
            received.append(event)

        tracker = HandoffTracker(visibility_timeout=5.0, on_handoff=on_handoff)
        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.check_departures(timestamp=106.0)
        tracker.update_visibility("cam_b", "person_1", timestamp=108.0)

        assert len(received) == 1
        assert received[0].target_id == "person_1"

    def test_get_handoffs(self):
        tracker = HandoffTracker(visibility_timeout=5.0)

        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.check_departures(timestamp=106.0)
        tracker.update_visibility("cam_b", "person_1", timestamp=108.0)

        handoffs = tracker.get_handoffs()
        assert len(handoffs) == 1
        assert handoffs[0]["target_id"] == "person_1"

    def test_get_handoffs_filtered(self):
        tracker = HandoffTracker(visibility_timeout=5.0)

        # Two targets hand off
        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.update_visibility("cam_a", "person_2", timestamp=100.0)
        tracker.check_departures(timestamp=106.0)
        tracker.update_visibility("cam_b", "person_1", timestamp=108.0)
        tracker.update_visibility("cam_b", "person_2", timestamp=109.0)

        all_handoffs = tracker.get_handoffs()
        assert len(all_handoffs) == 2

        p1_handoffs = tracker.get_handoffs(target_id="person_1")
        assert len(p1_handoffs) == 1

    def test_multiple_sensors(self):
        tracker = HandoffTracker(visibility_timeout=3.0)

        # Target passes through 3 cameras
        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.check_departures(timestamp=104.0)
        tracker.update_visibility("cam_b", "person_1", timestamp=106.0)
        tracker.check_departures(timestamp=110.0)
        tracker.update_visibility("cam_c", "person_1", timestamp=112.0)

        handoffs = tracker.get_handoffs(target_id="person_1")
        assert len(handoffs) == 2

    def test_confidence_decreases_with_gap(self):
        tracker = HandoffTracker(max_gap=60.0, visibility_timeout=3.0)

        # Short gap handoff
        tracker.update_visibility("cam_a", "p1", timestamp=100.0)
        tracker.check_departures(timestamp=104.0)
        h1 = tracker.update_visibility("cam_b", "p1", timestamp=105.0)

        # Long gap handoff
        tracker.update_visibility("cam_a", "p2", timestamp=200.0)
        tracker.check_departures(timestamp=204.0)
        h2 = tracker.update_visibility("cam_b", "p2", timestamp=250.0)

        assert h1 is not None and h2 is not None
        assert h1.confidence > h2.confidence

    def test_departure_memory_pruning(self):
        tracker = HandoffTracker(visibility_timeout=3.0)

        tracker.update_visibility("cam_a", "person_1", timestamp=100.0)
        tracker.check_departures(timestamp=104.0)

        # After DEPARTURE_MEMORY_TTL, old departures are pruned
        tracker.check_departures(timestamp=400.0)
        assert tracker.get_status()["pending_departures"] == 0


class TestHandoffEvent:
    """Test HandoffEvent dataclass."""

    def test_to_dict(self):
        event = HandoffEvent(
            handoff_id="h1",
            target_id="person_1",
            from_sensor="cam_a",
            to_sensor="cam_b",
            departure_time=100.0,
            arrival_time=110.0,
            gap_seconds=10.0,
            confidence=0.83,
            target_type="person",
        )
        d = event.to_dict()
        assert d["handoff_id"] == "h1"
        assert d["gap_seconds"] == 10.0
        assert d["confidence"] == 0.83
