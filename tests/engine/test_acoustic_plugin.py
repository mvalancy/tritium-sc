# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for the acoustic classification plugin."""

import sys
import time
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Ensure plugins/ and src/ are on path
_sc_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_sc_root / "src"))
sys.path.insert(0, str(_sc_root / "plugins"))

from acoustic.plugin import (
    AcousticPlugin,
    ACOUSTIC_TARGET_TTL,
    HIGH_SEVERITY_EVENTS,
    MEDIUM_SEVERITY_EVENTS,
    MAX_EVENT_HISTORY,
)
from engine.audio.acoustic_classifier import (
    AcousticEventType,
    AudioFeatures,
)


@pytest.fixture
def plugin():
    """Create an AcousticPlugin instance (not started)."""
    return AcousticPlugin()


@pytest.fixture
def mock_event_bus():
    """Create a mock EventBus."""
    bus = MagicMock()
    bus.subscribe.return_value = MagicMock()
    return bus


@pytest.fixture
def mock_tracker():
    """Create a mock TargetTracker with a lock and _targets dict."""
    tracker = MagicMock()
    tracker._lock = threading.Lock()
    tracker._targets = {}
    return tracker


@pytest.fixture
def configured_plugin(plugin, mock_event_bus, mock_tracker):
    """Create a configured (but not started) AcousticPlugin."""
    from engine.plugins.base import PluginContext

    ctx = PluginContext(
        event_bus=mock_event_bus,
        target_tracker=mock_tracker,
        simulation_engine=None,
        settings={},
        app=None,
        logger=MagicMock(),
        plugin_manager=None,
    )
    plugin.configure(ctx)
    return plugin


# -- Identity tests --------------------------------------------------------

class TestPluginIdentity:
    def test_plugin_id(self, plugin):
        assert plugin.plugin_id == "tritium.acoustic"

    def test_name(self, plugin):
        assert plugin.name == "Acoustic Classifier"

    def test_version(self, plugin):
        assert plugin.version == "1.0.0"

    def test_capabilities(self, plugin):
        caps = plugin.capabilities
        assert "data_source" in caps
        assert "routes" in caps
        assert "background" in caps

    def test_healthy_before_start(self, plugin):
        assert not plugin.healthy


# -- Classification tests -------------------------------------------------

class TestClassification:
    def test_classify_gunshot(self, configured_plugin):
        """High energy, short duration should classify as gunshot."""
        result = configured_plugin.classify_audio(
            features={
                "rms_energy": 0.9,
                "peak_amplitude": 0.95,
                "spectral_centroid": 1000.0,
                "spectral_bandwidth": 500.0,
                "zero_crossing_rate": 0.5,
                "duration_ms": 50,
            },
            device_id="mic_01",
        )
        assert result["event_type"] == "gunshot"
        assert result["confidence"] > 0.5
        assert result["device_id"] == "mic_01"

    def test_classify_voice(self, configured_plugin):
        """Mid-frequency, moderate energy, >200ms should classify as voice."""
        result = configured_plugin.classify_audio(
            features={
                "rms_energy": 0.3,
                "peak_amplitude": 0.4,
                "spectral_centroid": 500.0,
                "spectral_bandwidth": 200.0,
                "zero_crossing_rate": 0.3,
                "duration_ms": 1000,
            },
            device_id="mic_02",
        )
        assert result["event_type"] == "voice"

    def test_classify_vehicle(self, configured_plugin):
        """Low frequency, sustained, moderate energy -> vehicle."""
        result = configured_plugin.classify_audio(
            features={
                "rms_energy": 0.4,
                "peak_amplitude": 0.5,
                "spectral_centroid": 200.0,
                "spectral_bandwidth": 100.0,
                "zero_crossing_rate": 0.1,
                "duration_ms": 2000,
            },
            device_id="mic_03",
        )
        assert result["event_type"] == "vehicle"

    def test_classify_unknown(self, configured_plugin):
        """Very low energy -> unknown."""
        result = configured_plugin.classify_audio(
            features={
                "rms_energy": 0.01,
                "peak_amplitude": 0.02,
                "spectral_centroid": 100.0,
                "spectral_bandwidth": 50.0,
                "zero_crossing_rate": 0.01,
                "duration_ms": 100,
            },
        )
        assert result["event_type"] == "unknown"

    def test_classify_with_location(self, configured_plugin):
        """Classification with location data."""
        result = configured_plugin.classify_audio(
            features={
                "rms_energy": 0.9,
                "peak_amplitude": 0.95,
                "spectral_centroid": 1000.0,
                "duration_ms": 50,
            },
            device_id="mic_01",
            location=(33.45, -112.07),
        )
        assert result["location"] == (33.45, -112.07)


# -- Event history tests ---------------------------------------------------

class TestEventHistory:
    def test_events_recorded(self, configured_plugin):
        """Classified events should appear in history."""
        configured_plugin.classify_audio(
            features={"peak_amplitude": 0.95, "duration_ms": 50},
            device_id="mic_01",
        )
        events = configured_plugin.get_recent_events(10)
        assert len(events) == 1
        assert events[0]["device_id"] == "mic_01"

    def test_multiple_events(self, configured_plugin):
        """Multiple events accumulate."""
        for i in range(5):
            configured_plugin.classify_audio(
                features={"peak_amplitude": 0.95, "duration_ms": 50},
                device_id=f"mic_{i}",
            )
        events = configured_plugin.get_recent_events(10)
        assert len(events) == 5

    def test_history_limit(self, configured_plugin):
        """History should be capped."""
        for i in range(MAX_EVENT_HISTORY + 50):
            configured_plugin.classify_audio(
                features={"peak_amplitude": 0.1, "duration_ms": 50},
                device_id=f"mic_{i}",
            )
        events = configured_plugin.get_recent_events(MAX_EVENT_HISTORY + 100)
        assert len(events) <= MAX_EVENT_HISTORY

    def test_event_counts(self, configured_plugin):
        """Event counts by type."""
        # Gunshot
        configured_plugin.classify_audio(
            features={"peak_amplitude": 0.95, "duration_ms": 50}
        )
        # Voice
        configured_plugin.classify_audio(
            features={
                "rms_energy": 0.3,
                "peak_amplitude": 0.4,
                "spectral_centroid": 500.0,
                "duration_ms": 1000,
            }
        )
        counts = configured_plugin.get_event_counts()
        assert "gunshot" in counts
        assert "voice" in counts


# -- Statistics tests ------------------------------------------------------

class TestStats:
    def test_initial_stats(self, configured_plugin):
        stats = configured_plugin.get_stats()
        assert stats["events_classified"] == 0
        assert stats["active_targets"] == 0

    def test_stats_after_classification(self, configured_plugin):
        configured_plugin.classify_audio(
            features={"peak_amplitude": 0.95, "duration_ms": 50},
            device_id="mic_01",
        )
        stats = configured_plugin.get_stats()
        assert stats["events_classified"] == 1
        assert stats["high_severity_count"] == 1  # gunshot is high severity

    def test_stats_medium_severity(self, configured_plugin):
        """Vehicle is medium severity."""
        configured_plugin.classify_audio(
            features={
                "rms_energy": 0.4,
                "peak_amplitude": 0.5,
                "spectral_centroid": 200.0,
                "duration_ms": 2000,
            },
        )
        stats = configured_plugin.get_stats()
        assert stats["events_classified"] == 1
        # Vehicle is medium, not high
        assert stats["high_severity_count"] == 0


# -- EventBus integration tests -------------------------------------------

class TestEventBusIntegration:
    def test_high_severity_publishes_alert(
        self, configured_plugin, mock_event_bus
    ):
        """Gunshot classification should publish both classified and alert."""
        configured_plugin.classify_audio(
            features={"peak_amplitude": 0.95, "duration_ms": 50},
            device_id="mic_01",
        )
        # Should have published acoustic:classified and acoustic:alert
        call_args = [
            call[0][0] for call in mock_event_bus.publish.call_args_list
        ]
        assert "acoustic:classified" in call_args
        assert "acoustic:alert" in call_args

    def test_low_severity_no_alert(self, configured_plugin, mock_event_bus):
        """Unknown sounds should not trigger an alert."""
        configured_plugin.classify_audio(
            features={
                "rms_energy": 0.01,
                "peak_amplitude": 0.02,
                "duration_ms": 100,
            },
        )
        call_args = [
            call[0][0] for call in mock_event_bus.publish.call_args_list
        ]
        # Unknown events don't create targets, so no classified event
        # (but they do still classify)
        assert "acoustic:alert" not in call_args


# -- TargetTracker integration tests ---------------------------------------

class TestTrackerIntegration:
    def test_creates_target(self, configured_plugin, mock_tracker):
        """Classified sound should create a target in tracker."""
        configured_plugin.classify_audio(
            features={"peak_amplitude": 0.95, "duration_ms": 50},
            device_id="mic_01",
        )
        # Target should exist in tracker._targets
        assert len(mock_tracker._targets) > 0
        tid = list(mock_tracker._targets.keys())[0]
        assert "acoustic" in tid
        assert "gunshot" in tid

    def test_target_has_correct_source(self, configured_plugin, mock_tracker):
        """Acoustic target should have source='acoustic'."""
        configured_plugin.classify_audio(
            features={"peak_amplitude": 0.95, "duration_ms": 50},
            device_id="mic_01",
        )
        target = list(mock_tracker._targets.values())[0]
        assert target.source == "acoustic"

    def test_unknown_does_not_create_target(
        self, configured_plugin, mock_tracker
    ):
        """Unknown classifications should not create tracker targets."""
        configured_plugin.classify_audio(
            features={
                "rms_energy": 0.01,
                "peak_amplitude": 0.02,
                "duration_ms": 100,
            },
        )
        assert len(mock_tracker._targets) == 0


# -- Event handling tests --------------------------------------------------

class TestEventHandling:
    def test_handle_audio_features_event(self, configured_plugin):
        """Plugin should classify audio:features events."""
        event = {
            "type": "audio:features",
            "data": {
                "features": {
                    "rms_energy": 0.9,
                    "peak_amplitude": 0.95,
                    "duration_ms": 50,
                },
                "device_id": "mic_01",
            },
        }
        configured_plugin._handle_event(event)
        events = configured_plugin.get_recent_events(10)
        assert len(events) == 1

    def test_handle_audio_raw_event(self, configured_plugin):
        """Plugin should classify audio:raw events with features."""
        event = {
            "type": "audio:raw",
            "data": {
                "features": {
                    "rms_energy": 0.9,
                    "peak_amplitude": 0.95,
                    "duration_ms": 50,
                },
                "device_id": "mic_02",
            },
        }
        configured_plugin._handle_event(event)
        events = configured_plugin.get_recent_events(10)
        assert len(events) == 1

    def test_ignores_unrelated_events(self, configured_plugin):
        """Plugin should ignore non-audio events."""
        event = {"type": "fleet.heartbeat", "data": {"node_id": "test"}}
        configured_plugin._handle_event(event)
        events = configured_plugin.get_recent_events(10)
        assert len(events) == 0


# -- Lifecycle tests -------------------------------------------------------

class TestLifecycle:
    def test_start_stop(self, configured_plugin):
        """Plugin should start and stop cleanly."""
        configured_plugin.start()
        assert configured_plugin.healthy
        configured_plugin.stop()
        assert not configured_plugin.healthy

    def test_double_start(self, configured_plugin):
        """Starting twice should be idempotent."""
        configured_plugin.start()
        configured_plugin.start()
        assert configured_plugin.healthy
        configured_plugin.stop()

    def test_double_stop(self, configured_plugin):
        """Stopping twice should be safe."""
        configured_plugin.start()
        configured_plugin.stop()
        configured_plugin.stop()
        assert not configured_plugin.healthy
