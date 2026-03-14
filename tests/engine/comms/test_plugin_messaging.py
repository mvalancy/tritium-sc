# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Tests for inter-plugin messaging system."""

import pytest
import time
import threading

from engine.comms.event_bus import EventBus
from engine.comms.plugin_messaging import PluginMessage, PluginMessageBus


class TestPluginMessage:
    """Test PluginMessage dataclass."""

    @pytest.mark.unit
    def test_create(self):
        msg = PluginMessage(
            sender_id="plugin-a",
            target_id="plugin-b",
            message_type="request.data",
            payload={"query": "targets"},
        )
        assert msg.sender_id == "plugin-a"
        assert msg.target_id == "plugin-b"
        assert msg.message_type == "request.data"
        assert msg.payload == {"query": "targets"}
        assert len(msg.message_id) > 0

    @pytest.mark.unit
    def test_broadcast_default(self):
        msg = PluginMessage(sender_id="plugin-a")
        assert msg.target_id == "*"

    @pytest.mark.unit
    def test_is_expired(self):
        msg = PluginMessage(
            sender_id="a",
            ttl=0.1,
            timestamp=time.time() - 1.0,
        )
        assert msg.is_expired() is True

    @pytest.mark.unit
    def test_not_expired(self):
        msg = PluginMessage(sender_id="a", ttl=60.0)
        assert msg.is_expired() is False

    @pytest.mark.unit
    def test_no_ttl_never_expires(self):
        msg = PluginMessage(
            sender_id="a",
            ttl=0.0,
            timestamp=time.time() - 9999,
        )
        assert msg.is_expired() is False

    @pytest.mark.unit
    def test_to_dict(self):
        msg = PluginMessage(
            sender_id="plugin-a",
            target_id="plugin-b",
            message_type="ping",
        )
        d = msg.to_dict()
        assert d["sender_id"] == "plugin-a"
        assert d["target_id"] == "plugin-b"
        assert d["message_type"] == "ping"

    @pytest.mark.unit
    def test_from_dict(self):
        d = {
            "sender_id": "plugin-a",
            "target_id": "plugin-b",
            "message_type": "pong",
            "payload": 42,
        }
        msg = PluginMessage.from_dict(d)
        assert msg.sender_id == "plugin-a"
        assert msg.payload == 42


class TestPluginMessageBus:
    """Test PluginMessageBus send/subscribe/dispatch."""

    @pytest.mark.unit
    def test_send_and_receive(self):
        bus = EventBus()
        pmb = PluginMessageBus(bus)
        received = []

        def handler(msg):
            received.append(msg)

        pmb.subscribe("plugin-b", handler)

        pmb.send(PluginMessage(
            sender_id="plugin-a",
            target_id="plugin-b",
            message_type="hello",
            payload="world",
        ))

        # Wait for drain
        time.sleep(0.3)
        pmb.stop()

        assert len(received) == 1
        assert received[0].message_type == "hello"
        assert received[0].payload == "world"

    @pytest.mark.unit
    def test_broadcast(self):
        bus = EventBus()
        pmb = PluginMessageBus(bus)
        received_a = []
        received_b = []

        pmb.subscribe("plugin-a", lambda m: received_a.append(m))
        pmb.subscribe("plugin-b", lambda m: received_b.append(m))

        pmb.send(PluginMessage(
            sender_id="plugin-c",
            target_id="*",
            message_type="broadcast",
        ))

        time.sleep(0.3)
        pmb.stop()

        assert len(received_a) == 1
        assert len(received_b) == 1

    @pytest.mark.unit
    def test_type_subscription(self):
        bus = EventBus()
        pmb = PluginMessageBus(bus)
        received = []

        pmb.subscribe_type("request.", lambda m: received.append(m))

        pmb.send(PluginMessage(
            sender_id="a",
            message_type="request.targets",
        ))
        pmb.send(PluginMessage(
            sender_id="a",
            message_type="response.targets",
        ))

        time.sleep(0.3)
        pmb.stop()

        assert len(received) == 1
        assert received[0].message_type == "request.targets"

    @pytest.mark.unit
    def test_unsubscribe(self):
        bus = EventBus()
        pmb = PluginMessageBus(bus)
        received = []

        unsub = pmb.subscribe("plugin-x", lambda m: received.append(m))
        unsub()

        pmb.send(PluginMessage(
            sender_id="a",
            target_id="plugin-x",
            message_type="test",
        ))

        time.sleep(0.3)
        pmb.stop()

        assert len(received) == 0

    @pytest.mark.unit
    def test_reply(self):
        bus = EventBus()
        pmb = PluginMessageBus(bus)
        replies = []

        pmb.subscribe("plugin-a", lambda m: replies.append(m))

        original = PluginMessage(
            sender_id="plugin-a",
            target_id="plugin-b",
            message_type="request.data",
        )
        pmb.reply(original, {"result": "ok"})

        time.sleep(0.3)
        pmb.stop()

        assert len(replies) == 1
        assert replies[0].reply_to == original.message_id
        assert replies[0].message_type == "reply.request.data"
        assert replies[0].payload == {"result": "ok"}

    @pytest.mark.unit
    def test_expired_messages_dropped(self):
        bus = EventBus()
        pmb = PluginMessageBus(bus)
        received = []

        pmb.subscribe("plugin-b", lambda m: received.append(m))

        pmb.send(PluginMessage(
            sender_id="a",
            target_id="plugin-b",
            message_type="old",
            ttl=0.001,
            timestamp=time.time() - 10,
        ))

        time.sleep(0.3)
        pmb.stop()

        assert len(received) == 0
