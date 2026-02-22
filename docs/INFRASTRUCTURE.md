# TRITIUM-SC Infrastructure — MTIG Stack

**Mosquitto + Telegraf + InfluxDB + (optional) Grafana**

MQTT, Telegraf, and InfluxDB are foundational assumptions across the entire
TRITIUM-SC system. Every robot and every node should assume these services
are available. Grafana is optional for dashboarding.

## Architecture

```
Robots/Nodes                  Central Server (GB10)
+----------------+           +----------------------------------+
| Robot (ROS2)   |           |                                  |
|  +- MQTT client|--publish--|---> Mosquitto (port 1883)        |
|  +- Telegraf   |--write----|---> InfluxDB v2 (port 8086)      |
|  +- ROS2 nodes |           |       ^                          |
+----------------+           |       |                          |
                             |  Telegraf (MQTT consumer)        |
+----------------+           |    subscribes to tritium/#       |
| Camera/Sensor  |           |    parses JSON -> InfluxDB       |
|  +- MQTT client|--publish--|-->                               |
|  +- Telegraf   |--write----|-->                               |
+----------------+           |                                  |
                             |  FastAPI (/api/telemetry/*)      |
                             |    queries InfluxDB -> JSON      |
                             +----------------------------------+
```

```mermaid
flowchart LR
    MQTT[Mosquitto<br/>MQTT Broker] --> TEL[Telegraf<br/>Collector]
    TEL --> IDB[InfluxDB<br/>Time Series DB]
    IDB --> GF[Grafana<br/>Dashboards]

    DEVICES[IoT Devices<br/>Robots, Sensors] --> MQTT
    GF --> ALERTS[Alerts<br/>Notifications]

    style MQTT fill:#1a1a2e,stroke:#00f0ff,color:#00f0ff
    style TEL fill:#1a1a2e,stroke:#05ffa1,color:#05ffa1
    style IDB fill:#1a1a2e,stroke:#fcee0a,color:#fcee0a
    style GF fill:#1a1a2e,stroke:#ff2a6d,color:#ff2a6d
    style DEVICES fill:#1a1a2e,stroke:#00f0ff,color:#00f0ff
    style ALERTS fill:#1a1a2e,stroke:#ff2a6d,color:#ff2a6d
```

### Data Flows

1. **Robots** publish telemetry JSON to MQTT (existing, already working)
2. **Central Telegraf** subscribes to `tritium/#`, parses JSON, writes to InfluxDB
3. **Robot-local Telegraf** collects system metrics (CPU, mem, disk) and writes to central InfluxDB
4. **FastAPI** `/api/telemetry/*` proxies InfluxDB queries to JSON (tokens never in browser)
5. **(Optional) Grafana** connects to InfluxDB directly for rich dashboards

## Quick Start

### With Docker (recommended)

```bash
# Start the infrastructure stack
docker compose up -d mosquitto influxdb telegraf

# Verify services
mosquitto_pub -h localhost -t "test" -m "hello"    # MQTT works
curl http://localhost:8086/health                    # InfluxDB health
curl http://localhost:8000/api/telemetry/health      # API proxy check
```

### Without Docker (manual setup)

Install each service natively:

```bash
# Mosquitto
sudo apt install mosquitto mosquitto-clients
sudo cp conf/mosquitto/mosquitto.conf /etc/mosquitto/conf.d/tritium.conf
sudo systemctl restart mosquitto

# InfluxDB v2
# See https://docs.influxdata.com/influxdb/v2/install/
# After install, run initial setup:
influx setup \
  --username admin \
  --password tritium-sc-2026 \
  --org tritium \
  --bucket telemetry \
  --retention 30d \
  --token tritium-dev-token \
  --force

# Telegraf
sudo apt install telegraf
sudo cp conf/telegraf/telegraf.conf /etc/telegraf/telegraf.conf
# Set the token:
echo 'INFLUX_TOKEN=tritium-dev-token' | sudo tee /etc/default/telegraf
sudo systemctl restart telegraf
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `MQTT_ENABLED` | `true` | Enable MQTT bridge |
| `MQTT_HOST` | `localhost` | Mosquitto broker hostname |
| `MQTT_PORT` | `1883` | Mosquitto broker port |
| `MQTT_SITE_ID` | `home` | Site prefix in topic hierarchy |
| `INFLUX_ENABLED` | `true` | Enable InfluxDB telemetry storage |
| `INFLUX_URL` | `http://localhost:8086` | InfluxDB v2 API URL |
| `INFLUX_TOKEN` | `tritium-dev-token` | InfluxDB API token (override in production) |
| `INFLUX_ORG` | `tritium` | InfluxDB organization |
| `INFLUX_BUCKET` | `telemetry` | InfluxDB bucket name |
| `INFLUX_USER` | `admin` | InfluxDB admin username (Docker init only) |
| `INFLUX_PASSWORD` | `tritium-sc-2026` | InfluxDB admin password (Docker init only) |
| `INFLUX_RETENTION` | `30d` | Data retention period (Docker init only) |

### Graceful Degradation

If Mosquitto or InfluxDB are not running, the server starts normally:
- **MQTT absent**: Logs a warning, continues without fleet communication
- **InfluxDB absent**: `/api/telemetry/*` endpoints return empty results
- No crashes, no blocked boot

## MQTT Topic to InfluxDB Measurement Mapping

Telegraf parses MQTT topics and routes them to named measurements:

| MQTT Topic Pattern | InfluxDB Measurement | Tags Extracted |
|-------------------|---------------------|----------------|
| `tritium/+/robots/+/telemetry` | `robot_telemetry` | `site`, `robot_id` |
| `tritium/+/cameras/+/detections` | `camera_detections` | `site`, `camera_id` |
| `tritium/+/sensors/+/events` | `sensor_events` | `site`, `sensor_id` |
| `tritium/+/+/+/status` | `device_status` | (from payload) |

System metrics from Telegraf's built-in inputs:

| Measurement | Source | Fields |
|-------------|--------|--------|
| `cpu` | Central + robots | `usage_idle`, `usage_system`, `usage_user` |
| `mem` | Central + robots | `used_percent`, `available`, `total` |
| `disk` | Central + robots | `used_percent`, `free`, `total` |
| `net` | Central + robots | `bytes_recv`, `bytes_sent` |

## Telemetry API Endpoints

All endpoints are at `/api/telemetry/*`. Tokens are server-side only.

### `GET /api/telemetry/health`

Check InfluxDB connection status.

```json
{"status": "pass", "message": "ok", "influx_url": "http://localhost:8086", "bucket": "telemetry"}
```

### `GET /api/telemetry/robot/{robot_id}?hours=24&field=battery`

Robot metric history. Returns up to 1000 data points.

| Param | Default | Description |
|-------|---------|-------------|
| `hours` | 24 | Lookback window (1-168) |
| `field` | (all) | Optional field filter |

### `GET /api/telemetry/detections?hours=24`

Detection counts per camera, aggregated by hour.

### `GET /api/telemetry/system?hours=24`

CPU, memory, disk metrics across all nodes (5-minute averages).

### `GET /api/telemetry/summary`

Fleet status snapshot: online robots, detection count, latest values.

```json
{
  "robots_online": 2,
  "robot_ids": ["rover-alpha", "drone-bravo"],
  "detections_last_hour": 47,
  "latest_robot_data": [...]
}
```

## Adding a New Robot to the Fleet

1. **Configure MQTT** on the robot — set broker address, site ID, robot ID
2. **Publish telemetry** to `tritium/{site}/robots/{robot_id}/telemetry` (JSON)
3. **Install Telegraf** on the robot — use `conf/telegraf/telegraf-robot.conf`
4. Set `INFLUX_HOST` and `INFLUX_TOKEN` environment variables on the robot
5. The robot will appear automatically in:
   - TRITIUM-SC tactical map (via MQTT -> TargetTracker)
   - InfluxDB (via central Telegraf MQTT consumer)
   - `/api/telemetry/summary` endpoint

See `examples/robot-template/` for a complete reference implementation.

## Retention and Storage Estimates

Default retention: **30 days** (configurable via `INFLUX_RETENTION`).

| Source | Rate | Daily Storage |
|--------|------|--------------|
| 3 robots @ 2 Hz telemetry | 518K points/day | ~50 MB |
| 2 cameras @ 5 Hz detections | 864K points/day | ~80 MB |
| System metrics (5 nodes @ 10s) | 43K points/day | ~4 MB |
| **Total** | | **~134 MB/day** |

At 30-day retention: **~4 GB steady-state**. InfluxDB's TSM engine compresses
well; actual on-disk size is typically 30-50% of raw estimate.

## Grafana (Optional)

Grafana can connect directly to InfluxDB for rich dashboards:

```bash
# Add to docker-compose.yml if desired:
# grafana:
#   image: grafana/grafana:latest
#   ports: ["3000:3000"]
#   environment:
#     - GF_SECURITY_ADMIN_PASSWORD=tritium
#   volumes:
#     - grafana_data:/var/lib/grafana

# Then add InfluxDB as a data source in Grafana UI:
# URL: http://influxdb:8086
# Organization: tritium
# Token: (your INFLUX_TOKEN)
# Default Bucket: telemetry
```

## Telegraf Configuration Files

| File | Location | Purpose |
|------|----------|---------|
| `conf/telegraf/telegraf.conf` | Central server | MQTT consumer + system metrics -> InfluxDB |
| `conf/telegraf/telegraf-robot.conf` | Each robot | Local system metrics -> central InfluxDB |

Robot-side Telegraf requires two environment variables:
- `INFLUX_HOST` — central server hostname (e.g., `gb10-01`)
- `INFLUX_TOKEN` — InfluxDB API token
