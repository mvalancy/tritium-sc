# Examples — Reference Implementations

**Where you are:** `tritium-sc/examples/` — standalone reference projects showing how to integrate with the Tritium Command Center via MQTT.

**Parent:** [../CLAUDE.md](../CLAUDE.md) | [../../CLAUDE.md](../../CLAUDE.md) (tritium root)

## Available Examples

| Example | Language | Purpose | Has Tests |
|---------|----------|---------|-----------|
| `robot-template/` | Python | **Start here** — MQTT robot brain template for real hardware | Yes |
| `ros2-robot/` | Python/ROS2 | ROS2 Humble robot with Nav2 + MQTT bridge | Yes |
| `camera-server/` | Python | Demo camera simulator with YOLO detection | Yes |
| `hostile-agent/` | Python | Demo hostile LLM agent for threat testing | Yes |
| `mesh-radio/` | Python | Demo Meshtastic mesh radio node | Yes |
| `motion-sensor/` | Python | Demo motion sensor with pattern detection | Yes |
| `robot-server/` | Python | Demo robot simulator with physics | Yes |
| `swarm-drone/` | Python | Demo drone swarm with fleet coordination | Yes |

## How They Work

Each example is a standalone Python project that:
1. Connects to the MQTT broker
2. Publishes telemetry on `tritium/{site}/{type}/{id}/telemetry`
3. Subscribes to commands on `tritium/{site}/{type}/{id}/command`
4. Shows up as a live unit in the Command Center tactical map

## Getting Started

```bash
# Start with the robot template
cd robot-template
pip install -r requirements.txt
python robot.py
```

See [../docs/DEMO-SPEC.md](../docs/DEMO-SPEC.md) for detailed example specifications and [../docs/MQTT.md](../docs/MQTT.md) for the MQTT topic reference.
