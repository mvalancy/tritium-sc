# Communications Engine

**Where you are:** `tritium-sc/src/engine/comms/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The communications subsystem handles all message passing, event routing, and external protocol bridges. It connects the Command Center to edge devices (MQTT), other tactical systems (CoT/TAK), audio I/O (listener/speaker), and internal components (EventBus). Every piece of data entering or leaving the system passes through this layer.

## Key Files

| File | Purpose |
|------|---------|
| `event_bus.py` | Thread-safe pub/sub event bus — the internal nervous system for all components |
| `mqtt_bridge.py` | MQTT broker bridge — connects to edge devices for telemetry, commands, sightings |
| `cot.py` | Cursor on Target XML protocol — encode/decode CoT events |
| `cot_types/` | CoT type registry — `atoms.json` and `tak.json` define the CoT type hierarchy |
| `mqtt_cot.py` | MQTT-to-CoT codec — translates MQTT messages into CoT format |
| `tak_bridge.py` | TAK server bridge — interop with ATAK/WinTAK devices via multicast UDP/TCP |
| `notifications.py` | Notification dispatch — broadcasts alerts to UI via WebSocket |
| `fleet_bridge.py` | Fleet server communication — REST heartbeats and config sync with edge fleet |
| `meshtastic_bridge.py` | Meshtastic LoRa radio bridge — GPS nodes, text messages, waypoints |
| `meshcore_bridge.py` | MeshCore radio bridge — alternative mesh protocol support |
| `mesh_web_source.py` | Web-based mesh data source — pulls mesh node data via HTTP |
| `listener.py` | Audio input — VAD (voice activity detection) + recording |
| `speaker.py` | Audio output — TTS via Piper, sound effect playback |
| `robot_fsm_bridge.py` | Robot FSM bridge — translates robot state machine events to/from MQTT |

## Architecture

```
External                    Internal
--------                    --------
MQTT Broker  <--->  mqtt_bridge.py  --->  event_bus.py  <--->  All Components
TAK Devices  <--->  tak_bridge.py   --->       |
LoRa Radios  <--->  meshtastic_bridge.py ->    |
Fleet Server <--->  fleet_bridge.py --->       |
Microphone   --->   listener.py     --->       |
Speaker      <---   speaker.py      <---       v
                                         notifications.py  --->  WebSocket
```

## MQTT Topics

Topics follow the pattern `tritium/{site_or_device_id}/{type}`:
- `tritium/{device}/heartbeat` — periodic device telemetry
- `tritium/{device}/sighting` — BLE/WiFi device sightings
- `tritium/{device}/cmd` — commands from SC to devices
- `tritium/{device}/chat` — mesh chat bridged to/from devices
- `tritium/{site}/cameras/{id}/detections` — YOLO detection boxes
- `tritium/{site}/cameras/{id}/frame` — camera JPEG frames
- `tritium/{site}/robots/{id}/telemetry` — robot position/battery

## Related

- [../tactical/](../tactical/) — Tactical engine that consumes comms data
- [../simulation/](../simulation/) — Battle simulation that uses event_bus for tick events
- [../../app/routers/ws.py](../../app/routers/ws.py) — WebSocket broadcast endpoint
- [../../../../tritium-lib/src/tritium_lib/mqtt/](../../../../tritium-lib/src/tritium_lib/mqtt/) — Shared MQTT topic definitions
- [../../../../tritium-lib/src/tritium_lib/cot/](../../../../tritium-lib/src/tritium_lib/cot/) — Shared CoT XML codec
- [../../../plugins/tak_bridge/](../../../plugins/tak_bridge/) — TAK bridge plugin
- [../../../plugins/meshtastic/](../../../plugins/meshtastic/) — Meshtastic plugin
