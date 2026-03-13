# App — FastAPI Backend

**Where you are:** `tritium-sc/src/app/` — the FastAPI web server that serves the Command Center.

**Parent:** [../../CLAUDE.md](../../CLAUDE.md) | [../../../CLAUDE.md](../../../CLAUDE.md) (tritium root)

## What This Is

The backend server that ties everything together: serves the frontend, exposes REST/WebSocket APIs, manages the database, runs YOLO detection, and bridges to MQTT devices.

## Structure

```
app/
├── main.py               # FastAPI app entry point, lifespan, boot sequence
├── config.py             # Pydantic settings (app + Amy + MQTT + simulation)
├── models.py             # SQLAlchemy models (Camera, Event, Zone, Asset, etc.)
├── database.py           # Async database setup, FTS5 tables
├── routers/              # 25+ API endpoint modules
│   ├── ws.py             # WebSocket broadcast + TelemetryBatcher + Amy event bridge
│   ├── audio.py          # /api/audio/effects — sound effects API
│   ├── synthetic_feed.py # /api/synthetic/cameras — MJPEG streaming
│   └── ...               # See CLAUDE.md for full API reference
├── ai/                   # YOLO detection pipeline, tracker, embeddings
├── zones/                # Zone management and alerting
└── discovery/            # NVR auto-discovery
```

## Key Files

| File | Purpose |
|------|---------|
| `main.py` | App entry, lifespan hooks, boot sequence, middleware |
| `config.py` | All environment variables and settings |
| `models.py` | Database schema (Camera, Event, Zone, Asset, etc.) |
| `database.py` | SQLite + FTS5 setup |
| `routers/ws.py` | WebSocket for real-time UI + Amy events + sim telemetry |

## Running

```bash
./start.sh                    # Production mode on :8000
./setup.sh dev                # Dev mode with hot reload
```

## Testing

```bash
# API router tests
.venv/bin/python3 -m pytest tests/engine/api/ -v

# Integration E2E (starts headless server)
./test.sh 9
```

## Related

- [../engine/](../engine/) — System infrastructure (simulation, comms, tactical)
- [../amy/](../amy/) — AI commander (plugs in via CommanderProtocol)
- [../frontend/](../frontend/) — Vanilla JS frontend served as static files
- [../../docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md) — System architecture
- [../../.env.example](../../.env.example) — Environment variable reference
