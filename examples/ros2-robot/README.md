# TRITIUM-SC ROS2 Robot Template

A ROS2 robot node that bridges Nav2 navigation with TRITIUM-SC via MQTT.
Deploy on any ROS2-capable robot to appear as a friendly unit on Amy's
tactical map. Amy can dispatch it, assign patrols, and monitor its state --
just like simulated units, but running on real hardware with real Nav2.

## Prerequisites

- **ROS2 Humble** (or later) with `nav2_bringup` installed
- **Python 3.10+**
- **paho-mqtt** (`pip install paho-mqtt>=1.6.0,<2.0.0`)
- **MQTT broker** (Mosquitto) reachable from both this robot and TRITIUM-SC

## Build

```bash
# From your ROS2 workspace src/ directory
cd ~/ros2_ws/src
ln -s /path/to/tritium-sc/examples/ros2-robot ros2_robot

# Build
cd ~/ros2_ws
colcon build --packages-select ros2_robot
source install/setup.bash
```

## Run

### Simulated Mode (no hardware needed)

```bash
# Uses built-in simulated odometry -- good for testing the MQTT pipeline
ros2 launch ros2_robot robot.launch.py
```

### With Real Hardware

```bash
# Disable simulated odom when your robot provides real /odom
ros2 launch ros2_robot robot.launch.py use_sim_odom:=false
```

### Connecting to TRITIUM-SC

Set the MQTT broker address. NO hardcoded IPs -- use env vars or launch args:

```bash
# Via environment variable (recommended for Tailscale / multi-machine)
export MQTT_HOST=my-server       # hostname, Tailscale name, or IP
ros2 launch ros2_robot robot.launch.py

# Via launch argument
ros2 launch ros2_robot robot.launch.py mqtt_host:=my-server

# Custom site ID (must match TRITIUM-SC's MQTT_SITE_ID)
ros2 launch ros2_robot robot.launch.py site_id:=backyard
```

### Parameter File

Override defaults in `config/robot_params.yaml`:

```yaml
mqtt_bridge:
  ros__parameters:
    mqtt_host: "localhost"       # Override via env: MQTT_HOST
    mqtt_port: 1883
    site_id: "home"
    robot_id: "ros2-rover-alpha"
    robot_name: "ROS2 Rover Alpha"
    asset_type: "rover"
    telemetry_rate: 2.0          # Hz
    use_sim_odom: true           # False for real hardware
    home_x: 0.0                  # Recall destination X
    home_y: 0.0                  # Recall destination Y
```

## Architecture

```
                    MQTT Broker
                   /           \
    TRITIUM-SC Server          ROS2 Robot
    (MQTTBridge)               (this template)
         |                          |
    TargetTracker            +--------------+
    EventBus                 | mqtt_bridge  |---> Nav2 Goals
    WebSocket                |   _node.py   |<--- Command ACK
         |                   +--------------+
    Browser UI                      |
    (tactical map)           +--------------+
                             | telemetry    |<--- /odom
                             |   _node.py   |<--- /battery_state
                             +--------------+<--- /joint_states
                                    |
                             +--------------+
                             | simulated    |---> /odom (fake)
                             |   _odom.py   |---> /battery_state
                             +--------------+
```

### Nodes

| Node | Purpose |
|------|---------|
| `mqtt_bridge` | Subscribes to MQTT commands, sends Nav2 goals, publishes ACKs |
| `telemetry_publisher` | Reads /odom + /battery_state, publishes MQTT telemetry |
| `simulated_odom` | Fake odometry for testing (10 Hz, kinematic model) |

## MQTT Topic Reference

All topics follow the TRITIUM-SC protocol. The `{site}` and `{robot_id}`
segments are configurable via parameters.

### Published by Robot

| Topic | QoS | Retain | Content |
|-------|-----|--------|---------|
| `tritium/{site}/robots/{id}/telemetry` | 0 | no | Position, heading, speed, battery, status |
| `tritium/{site}/robots/{id}/status` | 1 | yes | `{"status": "online/offline", "robot_id": "..."}` |
| `tritium/{site}/robots/{id}/command/ack` | 1 | no | Command acknowledgment |

### Subscribed by Robot

| Topic | QoS | Content |
|-------|-----|---------|
| `tritium/{site}/robots/{id}/command` | 1 | Dispatch, patrol, recall, stop |

### Telemetry Payload

```json
{
  "name": "ROS2 Rover Alpha",
  "asset_type": "rover",
  "position": {"x": 3.5, "y": -2.1},
  "heading": 127.4,
  "speed": 1.2,
  "battery": 0.85,
  "status": "navigating",
  "timestamp": "2026-02-20T12:00:00+00:00"
}
```

### Command Payloads

```json
{"command": "dispatch", "x": 10.0, "y": -5.0, "timestamp": "..."}
{"command": "patrol", "waypoints": [{"x": 0, "y": 0}, {"x": 10, "y": 10}], "timestamp": "..."}
{"command": "recall", "timestamp": "..."}
{"command": "stop", "timestamp": "..."}
```

### Command ACK Payload

```json
{
  "command": "dispatch",
  "command_timestamp": "2026-02-20T10:30:00+00:00",
  "status": "accepted",
  "robot_id": "ros2-rover-alpha",
  "timestamp": "2026-02-20T10:30:00.123+00:00"
}
```

ACK status values: `accepted`, `rejected`, `completed`.

## ROS2 Topics Used

| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `/odom` | `nav_msgs/Odometry` | Subscribe | Robot position + velocity |
| `/battery_state` | `sensor_msgs/BatteryState` | Subscribe | Battery level |
| `/joint_states` | `sensor_msgs/JointState` | Subscribe | Turret pan/tilt |
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe (sim only) | Velocity commands |

## Nav2 Actions Used

| Action | Type | Purpose |
|--------|------|---------|
| `navigate_to_pose` | `NavigateToPose` | Single-point dispatch/recall |
| `follow_waypoints` | `FollowWaypoints` | Multi-point patrol |

## Testing

Tests work WITHOUT ROS2 installed (all ROS2 imports are mocked):

```bash
# From tritium-sc root
.venv/bin/python3 -m pytest examples/ros2-robot/tests/ -v

# Or from the ros2-robot directory
cd examples/ros2-robot
python -m pytest tests/ -v
```

## Differences from robot-template

| Feature | `robot-template` | `ros2-robot` |
|---------|-------------------|--------------|
| Navigation | Custom waypoint navigator | Nav2 (NavigateToPose, FollowWaypoints) |
| Odometry | Simulated hardware class | ROS2 /odom topic (or simulated node) |
| Battery | Simulated drain | ROS2 /battery_state topic |
| Turret | Direct hardware control | ROS2 /joint_states (pan/tilt) |
| Runtime | Standalone Python script | ROS2 node (rclpy, launch files) |
| MQTT protocol | Identical | Identical |

Both templates speak the exact same MQTT protocol. TRITIUM-SC cannot tell
them apart -- a ROS2 robot and a plain Python robot appear identically on
the tactical map.
