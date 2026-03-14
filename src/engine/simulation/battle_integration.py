# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""BattleIntegration — bridges simulation combat into the Wave 5-13 intelligence stack.

The simulation engine produces combat events (eliminations, projectile hits,
target movement) but these never reached the correlator, geofence engine,
dossier store, or automation system.  This module wires them together so
that simulated battles exercise the full intelligence pipeline — making
the game a genuine CI test for the real system.

Architecture
------------
BattleIntegration is an optional plugin attached to the SimulationEngine.
It subscribes to the EventBus and:

  1. Feeds sim target positions into GeofenceEngine on every telemetry batch.
  2. Syncs sim targets into TargetTracker so the correlator can see them.
  3. Publishes combat events as dossier signals (via DossierStore).
  4. Runs AutomationRules against incoming events and fires actions.

Data flow:

  Engine --[sim_telemetry_batch]--> BattleIntegration
    |
    +-> GeofenceEngine.check() --> geofence:enter/exit events
    +-> TargetTracker.update_from_simulation() --> correlator can fuse
    +-> DossierStore.create_or_update() --> persistent identity
    +-> AutomationEngine.evaluate() --> automation:alert events

AutomationEngine
----------------
A lightweight rule engine.  Each rule has:
  - trigger: event type to match (e.g. "target_eliminated", "geofence:enter")
  - conditions: dict of field->value filters on event data
  - action: dict describing what to do (publish event, escalate, etc.)

Rules are evaluated in order.  First match wins.  This is intentionally
simple — complex logic belongs in Amy's L4 deliberation, not here.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from engine.comms.event_bus import EventBus
    from engine.tactical.correlator import TargetCorrelator
    from engine.tactical.dossier import DossierStore
    from engine.tactical.geofence import GeofenceEngine
    from engine.tactical.investigation import InvestigationEngine
    from engine.tactical.target_tracker import TargetTracker

logger = logging.getLogger("engine.battle_integration")


# ---------------------------------------------------------------------------
# AutomationRule — lightweight if-then rule
# ---------------------------------------------------------------------------

@dataclass
class AutomationRule:
    """A single automation rule evaluated against incoming events."""

    rule_id: str = field(default_factory=lambda: uuid.uuid4().hex[:12])
    name: str = ""
    trigger: str = ""  # EventBus event type to match
    conditions: dict = field(default_factory=dict)
    action: dict = field(default_factory=dict)
    enabled: bool = True
    fire_count: int = 0
    last_fired: float = 0.0
    cooldown: float = 0.0  # minimum seconds between firings

    def matches(self, event_type: str, data: dict) -> bool:
        """Check if this rule matches the given event."""
        if not self.enabled:
            return False
        if event_type != self.trigger:
            return False
        # Check cooldown
        if self.cooldown > 0 and self.last_fired > 0:
            if time.time() - self.last_fired < self.cooldown:
                return False
        # Check conditions: each key in conditions must match data
        for key, expected in self.conditions.items():
            actual = data.get(key)
            if actual is None:
                return False
            if isinstance(expected, list):
                if actual not in expected:
                    return False
            elif actual != expected:
                return False
        return True

    def to_dict(self) -> dict:
        return {
            "rule_id": self.rule_id,
            "name": self.name,
            "trigger": self.trigger,
            "conditions": self.conditions,
            "action": self.action,
            "enabled": self.enabled,
            "fire_count": self.fire_count,
            "last_fired": self.last_fired,
            "cooldown": self.cooldown,
        }


class AutomationEngine:
    """Lightweight event-driven rule engine.

    Evaluates rules against incoming events and publishes actions
    on the EventBus.  Thread-safe.
    """

    def __init__(self, event_bus: EventBus) -> None:
        self._event_bus = event_bus
        self._rules: list[AutomationRule] = []
        self._lock = threading.Lock()
        self._history: list[dict] = []
        self._max_history = 500

    def add_rule(self, rule: AutomationRule) -> None:
        with self._lock:
            self._rules.append(rule)

    def remove_rule(self, rule_id: str) -> bool:
        with self._lock:
            before = len(self._rules)
            self._rules = [r for r in self._rules if r.rule_id != rule_id]
            return len(self._rules) < before

    def list_rules(self) -> list[dict]:
        with self._lock:
            return [r.to_dict() for r in self._rules]

    def get_history(self, limit: int = 50) -> list[dict]:
        with self._lock:
            return list(reversed(self._history[-limit:]))

    def evaluate(self, event_type: str, data: dict) -> list[dict]:
        """Evaluate all rules against an event. Returns list of fired actions."""
        fired: list[dict] = []
        with self._lock:
            for rule in self._rules:
                if rule.matches(event_type, data):
                    rule.fire_count += 1
                    rule.last_fired = time.time()

                    action_event = {
                        "rule_id": rule.rule_id,
                        "rule_name": rule.name,
                        "trigger_event": event_type,
                        "trigger_data": data,
                        "action": rule.action,
                        "timestamp": time.time(),
                    }
                    fired.append(action_event)

                    # Execute the action
                    self._execute_action(rule.action, data)

                    self._history.append(action_event)
                    if len(self._history) > self._max_history:
                        self._history = self._history[-self._max_history:]

        return fired

    def _execute_action(self, action: dict, trigger_data: dict) -> None:
        """Execute a rule action by publishing to EventBus."""
        action_type = action.get("type", "alert")

        if action_type == "alert":
            self._event_bus.publish("automation:alert", {
                "title": action.get("title", "Automation Alert"),
                "message": action.get("message", "Rule fired"),
                "severity": action.get("severity", "info"),
                "source": "automation",
                "target_id": trigger_data.get("target_id", ""),
            })
        elif action_type == "escalate":
            self._event_bus.publish("threat_escalation", {
                "target_id": trigger_data.get("target_id", ""),
                "level": action.get("level", "high"),
                "reason": action.get("reason", "automation rule"),
                "source": "automation",
            })
        elif action_type == "publish":
            # Generic event publish
            event_name = action.get("event", "automation:custom")
            self._event_bus.publish(event_name, {
                **trigger_data,
                "source": "automation",
            })


# ---------------------------------------------------------------------------
# Default combat automation rules
# ---------------------------------------------------------------------------

def default_combat_rules() -> list[AutomationRule]:
    """Create default automation rules for combat scenarios."""
    return [
        AutomationRule(
            name="Hostile eliminated alert",
            trigger="target_eliminated",
            conditions={"alliance": "hostile"},
            action={
                "type": "alert",
                "title": "Target Eliminated",
                "message": "Hostile target eliminated in combat",
                "severity": "info",
            },
            cooldown=1.0,
        ),
        AutomationRule(
            name="Friendly damage warning",
            trigger="projectile_hit",
            conditions={},
            action={
                "type": "alert",
                "title": "Unit Under Fire",
                "message": "Friendly unit taking damage",
                "severity": "warning",
            },
            cooldown=3.0,
        ),
        AutomationRule(
            name="Restricted zone breach",
            trigger="geofence:enter",
            conditions={"zone_type": "restricted"},
            action={
                "type": "escalate",
                "level": "high",
                "reason": "Target entered restricted zone",
            },
            cooldown=5.0,
        ),
        AutomationRule(
            name="Geofence entry notification",
            trigger="geofence:enter",
            conditions={},
            action={
                "type": "alert",
                "title": "Geofence Entry",
                "message": "Target entered monitored zone",
                "severity": "warning",
            },
            cooldown=2.0,
        ),
    ]


# ---------------------------------------------------------------------------
# BattleIntegration — the main bridge
# ---------------------------------------------------------------------------

class BattleIntegration:
    """Bridges simulation combat into the intelligence pipeline.

    Subscribes to EventBus events and routes them through:
    - GeofenceEngine (zone monitoring)
    - TargetTracker (unified target view for correlator)
    - DossierStore (persistent identity)
    - AutomationEngine (rule evaluation)
    - InvestigationEngine (auto-escalation)
    """

    def __init__(
        self,
        event_bus: EventBus,
        *,
        tracker: TargetTracker | None = None,
        geofence: GeofenceEngine | None = None,
        dossier_store: DossierStore | None = None,
        correlator: TargetCorrelator | None = None,
        investigation: InvestigationEngine | None = None,
        automation: AutomationEngine | None = None,
    ) -> None:
        self._event_bus = event_bus
        self._tracker = tracker
        self._geofence = geofence
        self._dossier_store = dossier_store
        self._correlator = correlator
        self._investigation = investigation
        self._automation = automation or AutomationEngine(event_bus)

        self._running = False
        self._thread: threading.Thread | None = None
        self._sub_queue: queue.Queue | None = None

        # Stats
        self._geofence_checks = 0
        self._tracker_syncs = 0
        self._dossier_updates = 0
        self._automation_fires = 0

    @property
    def automation(self) -> AutomationEngine:
        return self._automation

    @property
    def stats(self) -> dict:
        return {
            "geofence_checks": self._geofence_checks,
            "tracker_syncs": self._tracker_syncs,
            "dossier_updates": self._dossier_updates,
            "automation_fires": self._automation_fires,
            "running": self._running,
        }

    def start(self) -> None:
        """Start the integration bridge background thread."""
        if self._running:
            return
        self._running = True
        self._sub_queue = self._event_bus.subscribe()
        self._thread = threading.Thread(
            target=self._event_loop, name="battle-integration", daemon=True,
        )
        self._thread.start()
        logger.info("BattleIntegration started")

    def stop(self) -> None:
        """Stop the integration bridge."""
        self._running = False
        if self._sub_queue is not None:
            try:
                self._event_bus.unsubscribe(self._sub_queue)
            except Exception:
                pass
            self._sub_queue = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        logger.info("BattleIntegration stopped")

    def _event_loop(self) -> None:
        """Background thread: process EventBus messages."""
        while self._running and self._sub_queue is not None:
            try:
                msg = self._sub_queue.get(timeout=0.5)
            except Exception:
                continue

            event_type = msg.get("type", "")
            data = msg.get("data", {})

            try:
                self._handle_event(event_type, data)
            except Exception:
                logger.debug("BattleIntegration event handler error", exc_info=True)

    def _handle_event(self, event_type: str, data) -> None:
        """Route an event through all intelligence subsystems."""
        # 1. Telemetry batch -> geofence + tracker sync
        if event_type == "sim_telemetry_batch" and isinstance(data, list):
            self._on_telemetry_batch(data)
            return

        # 2. Combat events -> dossier + automation
        if event_type in (
            "target_eliminated", "target_neutralized",
            "projectile_fired", "projectile_hit",
        ):
            self._on_combat_event(event_type, data)

        # 3. Geofence events -> automation
        if event_type in ("geofence:enter", "geofence:exit"):
            self._on_geofence_event(event_type, data)

        # 4. All events -> automation engine
        fired = self._automation.evaluate(event_type, data)
        if fired:
            self._automation_fires += len(fired)

    def _on_telemetry_batch(self, batch: list[dict]) -> None:
        """Process a telemetry batch: sync tracker + check geofences."""
        for entry in batch:
            target_id = entry.get("target_id", "")
            if not target_id:
                continue

            # Sync to TargetTracker
            if self._tracker is not None:
                self._tracker.update_from_simulation(entry)
                self._tracker_syncs += 1

            # Check geofences
            if self._geofence is not None:
                pos = entry.get("position", {})
                x = pos.get("x", 0.0) if isinstance(pos, dict) else 0.0
                y = pos.get("y", 0.0) if isinstance(pos, dict) else 0.0
                self._geofence.check(target_id, (x, y))
                self._geofence_checks += 1

    def _on_combat_event(self, event_type: str, data: dict) -> None:
        """Process a combat event: update dossiers."""
        if self._dossier_store is None:
            return

        target_id = data.get("target_id", "")
        if not target_id:
            # Try other keys
            target_id = data.get("hostile_id", "") or data.get("shooter_id", "")
        if not target_id:
            return

        # Create dossier entries for combat participants
        source = f"combat_{event_type}"
        # Use a synthetic signal for the combat event
        signal_id = f"{event_type}_{target_id}_{time.time():.0f}"

        if event_type == "target_eliminated":
            # Create dossier for eliminated target
            shooter_id = data.get("shooter_id", "")
            if shooter_id:
                self._dossier_store.create_or_update(
                    signal_a=target_id,
                    source_a="combat",
                    signal_b=shooter_id,
                    source_b="combat",
                    confidence=0.9,
                    metadata={
                        "event": "elimination",
                        "target_name": data.get("target_name", ""),
                        "shooter_name": data.get("shooter_name", ""),
                    },
                )
                self._dossier_updates += 1

            # Auto-investigate high-threat eliminations
            if self._investigation is not None:
                alliance = data.get("alliance", "")
                if alliance == "friendly":
                    # Friendly was eliminated — auto-investigate
                    self._investigation.auto_investigate_threat(
                        dossier_id=target_id,
                        threat_level="high",
                        dossier_name=data.get("target_name", target_id[:8]),
                    )

    def _on_geofence_event(self, event_type: str, data: dict) -> None:
        """Process geofence events for dossier and investigation."""
        if self._dossier_store is None:
            return

        target_id = data.get("target_id", "")
        zone_id = data.get("zone_id", "")
        if not target_id or not zone_id:
            return

        # Create dossier linking target to zone
        self._dossier_store.create_or_update(
            signal_a=target_id,
            source_a="simulation",
            signal_b=f"zone_{zone_id}",
            source_b="geofence",
            confidence=0.8,
            metadata={
                "event": event_type,
                "zone_name": data.get("zone_name", ""),
                "zone_type": data.get("zone_type", ""),
            },
        )
        self._dossier_updates += 1

    def process_event_sync(self, event_type: str, data) -> None:
        """Synchronously process an event (for testing without threads)."""
        self._handle_event(event_type, data)
