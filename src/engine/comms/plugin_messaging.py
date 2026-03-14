# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Inter-plugin messaging — typed message passing between plugins via EventBus.

Plugins can send messages to each other without knowing implementation details.
Messages flow through the EventBus with a standard envelope format, allowing
any plugin to subscribe to messages from specific senders or of specific types.

Usage:
    from engine.comms.plugin_messaging import PluginMessageBus, PluginMessage

    # In plugin A:
    bus = PluginMessageBus(event_bus)
    bus.send(PluginMessage(
        sender_id="plugin-a",
        target_id="plugin-b",
        message_type="request.data",
        payload={"query": "active_targets"},
    ))

    # In plugin B:
    def handler(msg: PluginMessage):
        if msg.message_type == "request.data":
            # process request
            bus.reply(msg, {"targets": [...]})

    bus.subscribe("plugin-b", handler)
"""

from __future__ import annotations

import time
import uuid
import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from engine.comms.event_bus import EventBus


# Event type used on the EventBus for plugin messages
PLUGIN_MESSAGE_EVENT = "plugin_message"


@dataclass
class PluginMessage:
    """A typed message between plugins.

    Attributes:
        message_id: Unique message identifier.
        sender_id: Plugin ID of the sender.
        target_id: Plugin ID of the recipient ('*' for broadcast).
        message_type: Dot-separated type string (e.g., 'request.targets').
        payload: Arbitrary data payload.
        reply_to: Message ID this is a reply to (None for originals).
        timestamp: When the message was created.
        ttl: Time-to-live in seconds (0 = no expiry).
    """
    sender_id: str
    target_id: str = "*"
    message_type: str = ""
    payload: Any = None
    message_id: str = field(default_factory=lambda: str(uuid.uuid4())[:12])
    reply_to: Optional[str] = None
    timestamp: float = field(default_factory=time.time)
    ttl: float = 0.0

    def is_expired(self) -> bool:
        """Check if this message has expired."""
        if self.ttl <= 0:
            return False
        return (time.time() - self.timestamp) > self.ttl

    def to_dict(self) -> dict:
        return {
            "message_id": self.message_id,
            "sender_id": self.sender_id,
            "target_id": self.target_id,
            "message_type": self.message_type,
            "payload": self.payload,
            "reply_to": self.reply_to,
            "timestamp": self.timestamp,
            "ttl": self.ttl,
        }

    @classmethod
    def from_dict(cls, d: dict) -> PluginMessage:
        return cls(
            message_id=d.get("message_id", str(uuid.uuid4())[:12]),
            sender_id=d.get("sender_id", ""),
            target_id=d.get("target_id", "*"),
            message_type=d.get("message_type", ""),
            payload=d.get("payload"),
            reply_to=d.get("reply_to"),
            timestamp=d.get("timestamp", time.time()),
            ttl=d.get("ttl", 0.0),
        )


# Type alias for message handlers
MessageHandler = Callable[[PluginMessage], None]


class PluginMessageBus:
    """Inter-plugin message bus that rides on top of the EventBus.

    Provides targeted message delivery (by plugin_id) and typed message
    filtering without plugins needing to know each other's internals.
    """

    def __init__(self, event_bus: EventBus) -> None:
        self._event_bus = event_bus
        self._lock = threading.Lock()
        self._handlers: dict[str, list[MessageHandler]] = {}
        # { message_type_prefix: [handler] }
        self._type_handlers: dict[str, list[MessageHandler]] = {}
        self._queue = event_bus.subscribe()
        self._running = True
        self._thread = threading.Thread(
            target=self._drain_loop, daemon=True, name="plugin-msg-bus"
        )
        self._thread.start()

    def send(self, message: PluginMessage) -> None:
        """Send a message to a target plugin (or broadcast if target='*')."""
        self._event_bus.publish(PLUGIN_MESSAGE_EVENT, message.to_dict())

    def reply(self, original: PluginMessage, payload: Any,
              message_type: Optional[str] = None, sender_id: Optional[str] = None) -> None:
        """Send a reply to a previously received message."""
        reply_msg = PluginMessage(
            sender_id=sender_id or original.target_id,
            target_id=original.sender_id,
            message_type=message_type or f"reply.{original.message_type}",
            payload=payload,
            reply_to=original.message_id,
        )
        self.send(reply_msg)

    def subscribe(self, plugin_id: str, handler: MessageHandler) -> Callable[[], None]:
        """Subscribe to messages targeted at a specific plugin_id.

        Returns an unsubscribe function.
        """
        with self._lock:
            if plugin_id not in self._handlers:
                self._handlers[plugin_id] = []
            self._handlers[plugin_id].append(handler)

        def unsub():
            with self._lock:
                try:
                    self._handlers[plugin_id].remove(handler)
                except (KeyError, ValueError):
                    pass
        return unsub

    def subscribe_type(self, message_type_prefix: str,
                       handler: MessageHandler) -> Callable[[], None]:
        """Subscribe to messages matching a type prefix (e.g., 'request.' matches 'request.data').

        Returns an unsubscribe function.
        """
        with self._lock:
            if message_type_prefix not in self._type_handlers:
                self._type_handlers[message_type_prefix] = []
            self._type_handlers[message_type_prefix].append(handler)

        def unsub():
            with self._lock:
                try:
                    self._type_handlers[message_type_prefix].remove(handler)
                except (KeyError, ValueError):
                    pass
        return unsub

    def stop(self) -> None:
        """Stop the message bus drain loop."""
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._event_bus.unsubscribe(self._queue)

    def _drain_loop(self) -> None:
        """Drain EventBus queue and dispatch plugin messages."""
        import queue
        while self._running:
            try:
                event = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if event.get("type") != PLUGIN_MESSAGE_EVENT:
                continue

            data = event.get("data")
            if not data:
                continue

            try:
                msg = PluginMessage.from_dict(data)
            except Exception:
                continue

            if msg.is_expired():
                continue

            self._dispatch(msg)

    def _dispatch(self, msg: PluginMessage) -> None:
        """Dispatch a message to matching handlers."""
        with self._lock:
            # Target-specific handlers
            target = msg.target_id
            if target == "*":
                # Broadcast: call all handlers
                for handlers in self._handlers.values():
                    for h in handlers:
                        try:
                            h(msg)
                        except Exception:
                            pass
            else:
                handlers = self._handlers.get(target, [])
                for h in handlers:
                    try:
                        h(msg)
                    except Exception:
                        pass

            # Type-prefix handlers
            for prefix, handlers in self._type_handlers.items():
                if msg.message_type.startswith(prefix):
                    for h in handlers:
                        try:
                            h(msg)
                        except Exception:
                            pass
