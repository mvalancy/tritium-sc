# Meshtastic Integration

## Overview

TRITIUM-SC integrates with Meshtastic LoRa mesh radios for long-range,
low-bandwidth communication between operators, robots, and Amy. Mesh radios
provide a resilient backup channel when WiFi or internet connectivity is
unavailable.

## Supported Hardware

- **Heltec V3** (ESP32-S3 + SX1262) -- compact, affordable, good range
- **T-Beam** (ESP32 + SX1276/SX1262 + GPS) -- built-in GPS for position sharing
- Any Meshtastic-compatible device with WiFi enabled

All devices must run Meshtastic firmware 2.x or later.

## Setup

1. Enable WiFi on your Meshtastic device (via Bluetooth app or serial CLI)
2. Connect the device to the same WiFi network as the TRITIUM-SC server
3. Set `MESHTASTIC_ENABLED=true` in `.env`
4. (Optional) Set `MESHTASTIC_HOST` if auto-discovery via mDNS fails
5. Restart the server

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `MESHTASTIC_ENABLED` | `false` | Enable the Meshtastic bridge |
| `MESHTASTIC_HOST` | (auto-discover) | IP or hostname of the Meshtastic device |
| `MESHTASTIC_PORT` | `4403` | TCP port for protobuf API |

## Architecture

```
[LoRa Mesh Network] <--air--> [Heltec V3 Radio]
                                      |
                              TCP port 4403 (protobuf)
                                      |
                              [MeshtasticBridge]
                                /     |      \
                        EventBus  TargetTracker  Message History
                           |          |
                      [WebSocket]  [Tactical Map]
                           |          |
                      [Browser]   [mesh_radio icons]
                           |
                    [Mesh Panel: chat + nodes]
```

**MeshtasticBridge** connects to the radio over TCP and translates protobuf
packets into EventBus events. These events flow through the same WebSocket
broadcast pipeline as Amy and simulation events, so the browser receives
mesh data in real time.

Mesh nodes with GPS positions are registered in the TargetTracker as
`source="mesh"` targets, rendering them on the tactical map alongside
YOLO detections and simulation units.

## API Endpoints

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/mesh/nodes` | All mesh nodes with position, battery, signal strength |
| `GET` | `/api/mesh/messages` | Message history (optional `?limit=50`) |
| `POST` | `/api/mesh/send` | Send text message `{ text, channel?, destination? }` |
| `GET` | `/api/mesh/status` | Bridge connection status and statistics |
| `GET` | `/api/mesh/discover` | Trigger mDNS scan for Meshtastic devices |

### Example: Send a Message

```bash
curl -X POST http://localhost:8000/api/mesh/send \
  -H "Content-Type: application/json" \
  -d '{"text": "INTRUDER NORTH FENCE", "channel": 1}'
```

### Example: List Nodes

```bash
curl http://localhost:8000/api/mesh/nodes
```

Response:

```json
[
  {
    "id": "!a1b2c3d4",
    "long_name": "Rover-1",
    "short_name": "RV1",
    "position": { "lat": 33.123, "lon": -117.456, "alt": 45.0 },
    "battery": 87,
    "snr": 12.5,
    "last_heard": "2026-02-21T14:30:00"
  }
]
```

## WebSocket Events

All mesh events are broadcast on `ws://localhost:8000/ws/live` with the
following types:

| Event Type | Payload | Description |
|------------|---------|-------------|
| `mesh_text` | `{ from, to, text, channel, timestamp }` | Text message received |
| `mesh_position` | `{ node_id, lat, lon, alt, timestamp }` | Node position update |
| `mesh_telemetry` | `{ node_id, battery, voltage, channel_util, air_util }` | Node telemetry |
| `mesh_connected` | `{ host, port }` | Bridge connected to radio |
| `mesh_disconnected` | `{ reason }` | Bridge lost connection |

## Message Limits

- **228 characters** per message (LoRa packet size limit)
- **Throughput**: ~1--5 messages per minute depending on channel settings
- **Latency**: seconds to tens of seconds depending on hop count
- **Range**: 1--10+ km line-of-sight depending on antenna and terrain

Keep messages short and critical. This is not a chat channel -- it is a
tactical communication backup.

## Use Cases

- **Commander to Operator messaging** -- backup channel when WiFi is down
- **Robot status via mesh** -- failover from MQTT/WiFi for position reports
- **Position sharing** -- GPS-equipped nodes appear on the tactical map
- **Emergency alerts** -- Amy can broadcast threat alerts over LoRa
- **Off-grid operations** -- mesh works without any internet infrastructure

## Channel Configuration

- **Channel 0 (primary)**: default Meshtastic mesh communication
- **Recommended**: create a `TRITIUM` channel on index 1 with a random PSK
- Configure channels via the Meshtastic Bluetooth app, Python CLI, or web UI

### Setting Up a Dedicated Channel

```bash
# Using meshtastic Python CLI
meshtastic --ch-index 1 --ch-set name TRITIUM
meshtastic --ch-index 1 --ch-set psk random
```

All TRITIUM-SC mesh traffic should use the dedicated channel to avoid
interference with default mesh traffic.

## Keyboard Shortcut

Press `5` in the Command Center to toggle the Mesh Radio panel. The panel
shows:

- Connected mesh nodes with signal strength and battery
- Message history with timestamps
- Compose field for sending messages (228 char limit enforced in UI)

## Troubleshooting

| Problem | Fix |
|---------|-----|
| Bridge cannot connect | Verify device WiFi is on and on same network. Check `MESHTASTIC_HOST`. |
| No nodes appear | Ensure at least one other Meshtastic device is powered on and in range. |
| Messages not sending | Check channel configuration matches between devices. |
| mDNS discovery fails | Set `MESHTASTIC_HOST` explicitly to the device IP. |
| Position not updating | Verify the device has GPS lock (T-Beam) or manual position set. |
