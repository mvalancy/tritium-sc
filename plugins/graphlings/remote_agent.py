# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""RemoteAgentRegistry — protocol for external AI agents to join a simulation.

Allows any external agent (graphling instances, custom bots, etc.) to
register with a running tritium-sc session via HTTP or WebSocket, receive
perceptions, and send back decisions.

This is a GENERIC agent protocol — no proprietary graphlings code.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Optional

log = logging.getLogger(__name__)


class RemoteAgentRegistry:
    """Thread-safe registry tracking connected external agents.

    Each agent has:
      - agent_id: unique identifier
      - name: human-readable display name
      - capabilities: list of strings (e.g., ["think", "observe", "emote"])
      - callback_url: optional HTTP endpoint for push-based perception delivery
      - connected_at: timestamp of registration
    """

    def __init__(self, max_agents: int = 10) -> None:
        self._max_agents = max_agents
        self._lock = threading.Lock()
        # agent_id -> agent info dict
        self._agents: dict[str, dict[str, Any]] = {}

    def register_agent(
        self,
        agent_id: str,
        name: str,
        capabilities: list[str],
        callback_url: Optional[str] = None,
    ) -> bool:
        """Register an external agent.

        Returns True if registration succeeded, False if:
          - agent_id is empty
          - name is empty
          - agent_id already registered
          - max agents limit reached
        """
        if not agent_id or not name:
            return False

        with self._lock:
            if agent_id in self._agents:
                return False
            if len(self._agents) >= self._max_agents:
                return False

            self._agents[agent_id] = {
                "agent_id": agent_id,
                "name": name,
                "capabilities": list(capabilities),
                "callback_url": callback_url,
                "connected_at": time.time(),
            }

        log.info("Agent registered: %s (%s)", agent_id, name)
        return True

    def unregister_agent(self, agent_id: str) -> bool:
        """Unregister an agent. Returns True if removed, False if not found."""
        with self._lock:
            removed = self._agents.pop(agent_id, None)

        if removed is not None:
            log.info("Agent unregistered: %s (%s)", agent_id, removed["name"])
            return True
        return False

    def get_agents(self) -> list[dict[str, Any]]:
        """Return list of all registered agent info dicts."""
        with self._lock:
            return list(self._agents.values())

    def get_agent(self, agent_id: str) -> Optional[dict[str, Any]]:
        """Return agent info dict for a specific agent, or None."""
        with self._lock:
            return self._agents.get(agent_id)

    def route_perception(
        self,
        agent_id: str,
        perception: dict[str, Any],
        available_actions: list[str],
        urgency: float,
    ) -> Optional[dict[str, Any]]:
        """Build a perception packet addressed to a specific agent.

        Returns the routed perception dict, or None if:
          - agent_id not found
          - agent lacks 'think' capability (cannot process perceptions)
        """
        with self._lock:
            agent = self._agents.get(agent_id)
            if agent is None:
                return None
            caps = agent["capabilities"]

        # Agent needs 'think' capability to process perceptions
        if "think" not in caps:
            return None

        return {
            "agent_id": agent_id,
            "perception": perception,
            "available_actions": available_actions,
            "urgency": urgency,
        }

    # ── WebSocket message builders/parsers ────────────────────────

    def build_perceive_message(
        self,
        perception: dict[str, Any],
        available_actions: list[str],
        urgency: float,
    ) -> dict[str, Any]:
        """Build a 'perceive' message to send to an agent via WebSocket."""
        return {
            "type": "perceive",
            "perception": perception,
            "available_actions": available_actions,
            "urgency": urgency,
        }

    def parse_decide_message(
        self, raw: dict[str, Any]
    ) -> Optional[dict[str, Any]]:
        """Parse a 'decide' message received from an agent via WebSocket.

        Returns parsed decision dict, or None if message type is wrong.
        """
        if raw.get("type") != "decide":
            return None
        return {
            "thought": raw.get("thought", ""),
            "action": raw.get("action", ""),
            "emotion": raw.get("emotion", ""),
        }

    def build_act_result_message(
        self, success: bool, details: str = ""
    ) -> dict[str, Any]:
        """Build an 'act_result' message to confirm action execution."""
        return {
            "type": "act_result",
            "success": success,
            "details": details,
        }
