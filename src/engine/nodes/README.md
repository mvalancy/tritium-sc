# Engine: Nodes

Distributed sensor node abstractions for cameras, microphones, PTZ, and robots.

## Files

| File | Purpose |
|------|---------|
| `base.py` | Abstract SensorNode interface (camera, mic, PTZ, speaker capabilities) |
| `bcc950.py` | BCC950 USB PTZ camera + mic + speaker node implementation |
| `audio.py` | Audio-only sensor node |
| `frame_buffer.py` | Frame buffer for camera nodes |
| `mqtt_robot.py` | MQTTSensorNode — wraps MQTT robot as a SensorNode |

## Pattern

All nodes implement the SensorNode base class with standardized
capability flags. Amy and the tactical engine interact with nodes
through this uniform interface regardless of underlying hardware.
