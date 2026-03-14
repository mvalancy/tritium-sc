# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""AcousticPlugin — sound classification for the Tritium sensing pipeline.

Subscribes to audio events from edge devices (ESP32 I2S microphones,
IP camera audio channels, dedicated acoustic sensors). Classifies sounds
into categories like gunshot, voice, vehicle, animal, glass_break, siren.

Classified events are published to EventBus and wired into TargetTracker
as temporary acoustic detection targets on the tactical map.

MQTT topics:
    IN:  tritium/{site}/audio/{device}/raw   — raw audio features
    OUT: tritium/{site}/audio/{device}/event  — classified events
"""

from __future__ import annotations

import logging
import queue as queue_mod
import threading
import time
from typing import Any, Optional

from engine.plugins.base import PluginContext, PluginInterface
from engine.audio.acoustic_classifier import (
    AcousticClassifier,
    AcousticEvent,
    AcousticEventType,
    AudioFeatures,
)

log = logging.getLogger("acoustic")

# How often to run the cleanup loop (seconds)
DEFAULT_POLL_INTERVAL = 2.0

# Stale timeout for acoustic targets in TargetTracker
ACOUSTIC_TARGET_TTL = 30.0

# Maximum event history for the plugin
MAX_EVENT_HISTORY = 500

# Severity classification for alert routing
HIGH_SEVERITY_EVENTS = {
    AcousticEventType.GUNSHOT,
    AcousticEventType.EXPLOSION,
    AcousticEventType.GLASS_BREAK,
}
MEDIUM_SEVERITY_EVENTS = {
    AcousticEventType.SIREN,
    AcousticEventType.ALARM,
    AcousticEventType.VEHICLE,
}


class AcousticPlugin(PluginInterface):
    """Acoustic classification plugin for passive sound monitoring."""

    def __init__(self) -> None:
        self._event_bus: Any = None
        self._tracker: Any = None
        self._app: Any = None
        self._logger: Optional[logging.Logger] = None

        self._classifier = AcousticClassifier()

        self._running = False
        self._poll_interval = DEFAULT_POLL_INTERVAL
        self._cleanup_thread: Optional[threading.Thread] = None
        self._event_queue: Optional[queue_mod.Queue] = None
        self._event_thread: Optional[threading.Thread] = None

        # Plugin-level event history
        self._event_history: list[dict] = []
        self._lock = threading.Lock()

        # Active acoustic targets (target_id -> expiry time)
        self._active_targets: dict[str, float] = {}

        # Statistics
        self._stats = {
            "events_processed": 0,
            "events_classified": 0,
            "high_severity_count": 0,
            "targets_created": 0,
        }

    # -- PluginInterface identity ------------------------------------------

    @property
    def plugin_id(self) -> str:
        return "tritium.acoustic"

    @property
    def name(self) -> str:
        return "Acoustic Classifier"

    @property
    def version(self) -> str:
        return "1.0.0"

    @property
    def capabilities(self) -> set[str]:
        return {"data_source", "routes", "background"}

    # -- PluginInterface lifecycle -----------------------------------------

    def configure(self, ctx: PluginContext) -> None:
        self._event_bus = ctx.event_bus
        self._tracker = ctx.target_tracker
        self._app = ctx.app
        self._logger = ctx.logger or log

        # Apply settings
        settings = ctx.settings or {}
        if "poll_interval" in settings:
            self._poll_interval = float(settings["poll_interval"])
        if "target_ttl" in settings:
            global ACOUSTIC_TARGET_TTL
            ACOUSTIC_TARGET_TTL = float(settings["target_ttl"])

        # Configure classifier thresholds
        if "gunshot_energy_threshold" in settings:
            self._classifier.GUNSHOT_ENERGY_THRESHOLD = float(
                settings["gunshot_energy_threshold"]
            )
        if "voice_centroid_min_hz" in settings:
            self._classifier.VOICE_CENTROID_MIN_HZ = float(
                settings["voice_centroid_min_hz"]
            )

        # Register routes
        self._register_routes()

        self._logger.info("Acoustic plugin configured")

    def start(self) -> None:
        if self._running:
            return
        self._running = True

        # Start cleanup thread (removes stale acoustic targets)
        self._cleanup_thread = threading.Thread(
            target=self._cleanup_loop,
            daemon=True,
            name="acoustic-cleanup",
        )
        self._cleanup_thread.start()

        # Subscribe to EventBus for audio data from edge devices
        if self._event_bus:
            self._event_queue = self._event_bus.subscribe()
            self._event_thread = threading.Thread(
                target=self._event_drain_loop,
                daemon=True,
                name="acoustic-events",
            )
            self._event_thread.start()

        self._logger.info(
            "Acoustic plugin started (poll=%.1fs)", self._poll_interval
        )

    def stop(self) -> None:
        if not self._running:
            return
        self._running = False

        if self._cleanup_thread and self._cleanup_thread.is_alive():
            self._cleanup_thread.join(timeout=3.0)

        if self._event_thread and self._event_thread.is_alive():
            self._event_thread.join(timeout=2.0)

        if self._event_bus and self._event_queue:
            self._event_bus.unsubscribe(self._event_queue)

        self._logger.info("Acoustic plugin stopped")

    @property
    def healthy(self) -> bool:
        return self._running

    # -- Classification API ------------------------------------------------

    def classify_audio(
        self,
        features: dict,
        device_id: str = "",
        location: Optional[tuple[float, float]] = None,
    ) -> dict:
        """Classify audio features and return the event.

        Args:
            features: Dict with rms_energy, peak_amplitude, zero_crossing_rate,
                      spectral_centroid, spectral_bandwidth, duration_ms.
            device_id: Source device identifier.
            location: Optional (lat, lng) of the microphone.

        Returns:
            Classified event as a dict.
        """
        audio_features = AudioFeatures(
            rms_energy=features.get("rms_energy", 0.0),
            peak_amplitude=features.get("peak_amplitude", 0.0),
            zero_crossing_rate=features.get("zero_crossing_rate", 0.0),
            spectral_centroid=features.get("spectral_centroid", 0.0),
            spectral_bandwidth=features.get("spectral_bandwidth", 0.0),
            duration_ms=features.get("duration_ms", 0),
        )

        event = self._classifier.classify(audio_features)
        event.device_id = device_id
        event.location = location

        self._on_classified(event)
        return self._event_to_dict(event)

    def get_recent_events(self, count: int = 50) -> list[dict]:
        """Get the most recent classified events."""
        with self._lock:
            return list(self._event_history[-count:])

    def get_stats(self) -> dict:
        """Get plugin statistics."""
        with self._lock:
            return {
                **self._stats,
                "active_targets": len(self._active_targets),
                "history_size": len(self._event_history),
                "classifier_history": len(
                    self._classifier.get_recent_events()
                ),
            }

    def get_event_counts(self) -> dict[str, int]:
        """Get count of each event type."""
        return self._classifier.get_event_counts()

    # -- Internal event handling -------------------------------------------

    def _on_classified(self, event: AcousticEvent) -> None:
        """Handle a newly classified acoustic event."""
        event_dict = self._event_to_dict(event)

        with self._lock:
            self._event_history.append(event_dict)
            if len(self._event_history) > MAX_EVENT_HISTORY:
                self._event_history = self._event_history[-MAX_EVENT_HISTORY:]
            self._stats["events_classified"] += 1

        # Determine severity
        severity = "low"
        if event.event_type in HIGH_SEVERITY_EVENTS:
            severity = "high"
            with self._lock:
                self._stats["high_severity_count"] += 1
        elif event.event_type in MEDIUM_SEVERITY_EVENTS:
            severity = "medium"

        # Publish to EventBus
        if self._event_bus:
            self._event_bus.publish(
                "acoustic:classified",
                data={**event_dict, "severity": severity},
            )

            # High severity events get an additional alert
            if severity == "high":
                self._event_bus.publish(
                    "acoustic:alert",
                    data={
                        "type": event.event_type.value,
                        "confidence": event.confidence,
                        "device_id": event.device_id,
                        "location": event.location,
                        "severity": severity,
                        "timestamp": event.timestamp,
                    },
                )

        # Create target in tracker
        if event.event_type != AcousticEventType.UNKNOWN:
            self._create_acoustic_target(event, severity)

    def _create_acoustic_target(
        self, event: AcousticEvent, severity: str
    ) -> None:
        """Create a temporary target in TargetTracker for the acoustic event."""
        if self._tracker is None:
            return

        target_id = f"acoustic_{event.event_type.value}_{event.device_id or 'unknown'}"

        with self._lock:
            self._active_targets[target_id] = (
                time.monotonic() + ACOUSTIC_TARGET_TTL
            )
            self._stats["targets_created"] += 1

        try:
            from engine.tactical.target_tracker import TrackedTarget

            with self._tracker._lock:
                if target_id in self._tracker._targets:
                    t = self._tracker._targets[target_id]
                    t.last_seen = time.monotonic()
                    t.status = f"{event.event_type.value}:{severity}"
                    if event.location:
                        t.position = event.location
                else:
                    self._tracker._targets[target_id] = TrackedTarget(
                        target_id=target_id,
                        name=f"Acoustic: {event.event_type.value}",
                        alliance="unknown",
                        asset_type="acoustic_detection",
                        position=event.location or (0.0, 0.0),
                        last_seen=time.monotonic(),
                        source="acoustic",
                        position_source="microphone",
                        position_confidence=event.confidence,
                        status=f"{event.event_type.value}:{severity}",
                    )
        except Exception as exc:
            log.error("Failed to create acoustic target: %s", exc)

    # -- Cleanup loop (remove stale targets) --------------------------------

    def _cleanup_loop(self) -> None:
        """Background loop: remove expired acoustic targets."""
        while self._running:
            try:
                now = time.monotonic()
                expired = []

                with self._lock:
                    for tid, expiry in self._active_targets.items():
                        if now > expiry:
                            expired.append(tid)
                    for tid in expired:
                        del self._active_targets[tid]

                # Remove from tracker
                if self._tracker and expired:
                    try:
                        with self._tracker._lock:
                            for tid in expired:
                                self._tracker._targets.pop(tid, None)
                    except Exception:
                        pass

            except Exception as exc:
                log.error("Acoustic cleanup error: %s", exc)

            # Sleep in small increments for responsive shutdown
            deadline = time.monotonic() + self._poll_interval
            while self._running and time.monotonic() < deadline:
                time.sleep(0.25)

    # -- Event bus listener (incoming audio data) --------------------------

    def _event_drain_loop(self) -> None:
        """Background loop: drain EventBus for audio data from edge."""
        while self._running:
            try:
                event = self._event_queue.get(timeout=0.5)
                self._handle_event(event)
            except queue_mod.Empty:
                pass
            except Exception as exc:
                log.error("Acoustic event error: %s", exc)

    def _handle_event(self, event: dict) -> None:
        """Process incoming events for audio data."""
        event_type = event.get("type", event.get("event_type", ""))
        data = event.get("data", {})

        with self._lock:
            self._stats["events_processed"] += 1

        if event_type == "audio:features":
            # Direct audio feature data from edge
            self.classify_audio(
                features=data.get("features", data),
                device_id=data.get("device_id", ""),
                location=data.get("location"),
            )
        elif event_type == "audio:raw":
            # Raw audio data — extract features if possible
            features = data.get("features")
            if features:
                self.classify_audio(
                    features=features,
                    device_id=data.get("device_id", ""),
                    location=data.get("location"),
                )

    # -- Routes ------------------------------------------------------------

    def _register_routes(self) -> None:
        if not self._app:
            return

        from .routes import create_router

        router = create_router(self)
        self._app.include_router(router)

    # -- Helpers -----------------------------------------------------------

    @staticmethod
    def _event_to_dict(event: AcousticEvent) -> dict:
        """Convert an AcousticEvent to a serializable dict."""
        return {
            "event_type": event.event_type.value,
            "confidence": event.confidence,
            "timestamp": event.timestamp,
            "duration_ms": event.duration_ms,
            "peak_frequency_hz": event.peak_frequency_hz,
            "peak_amplitude_db": event.peak_amplitude_db,
            "device_id": event.device_id,
            "location": event.location,
        }
