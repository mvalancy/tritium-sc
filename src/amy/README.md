# Amy — AI Commander

**Where you are:** `tritium-sc/src/amy/` — the AI commander personality plugin that implements `CommanderProtocol`.

**Parent:** [../../CLAUDE.md](../../CLAUDE.md) | [../../../CLAUDE.md](../../../CLAUDE.md) (tritium root)

## What This Is

Amy is an autonomous AI consciousness with 4 cognitive layers. She sees through cameras, hears through mics, thinks in continuous inner monologue, dispatches assets, and acts when she decides to. When no battle is running, she monitors the neighborhood — tracking who comes and goes, learning rhythms, noticing deviations.

## Cognitive Layers

```
L4: Deliberation  ← brain/thinking.py   (LLM inner monologue, strategic reasoning)
L3: Awareness     ← brain/sensorium.py  (temporal sensor fusion, narrative)
L2: Instinct      ← commander.py        (threat response, dispatch rules)
L1: Reflex        ← commander.py        (immediate reactions, safety)
```

## Structure

```
amy/
├── __init__.py           # create_amy() factory, version
├── commander.py          # Main orchestrator (implements CommanderProtocol)
├── router.py             # FastAPI: /api/amy/* endpoints
├── brain/
│   ├── thinking.py       # L4 deliberation: LLM inner monologue
│   ├── sensorium.py      # L3 awareness: temporal sensor fusion + narrative
│   ├── memory.py         # Persistent long-term memory (JSON file)
│   └── agent.py          # LLM agent with tool use
├── actions/
│   ├── motor.py          # PTZ motor programs, MotorCommand generators
│   ├── tools.py          # Tool definitions for agent mode
│   └── announcer.py      # WarAnnouncer — Smash TV style combat commentary
└── comms/
    └── transcript.py     # Conversation logging (daily JSONL files)
```

## Key Files

| File | Size | Purpose |
|------|------|---------|
| `commander.py` | 92KB | Main Amy orchestrator — the largest file in the project |
| `router.py` | 26KB | All /api/amy/* FastAPI endpoints |
| `brain/thinking.py` | — | Inner monologue via fast LLM (continuous stream of consciousness) |
| `brain/sensorium.py` | — | Fuses camera feeds, audio, sensors into temporal narrative |
| `brain/memory.py` | — | Persistent memory stored in `data/amy/memory.json` |

## API Endpoints

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/amy/status` | GET | Amy state, mood, active nodes |
| `/api/amy/thoughts` | GET (SSE) | Stream of consciousness |
| `/api/amy/sensorium` | GET | Temporal narrative + mood |
| `/api/amy/chat` | POST | Talk to Amy |
| `/api/amy/command` | POST | Send Lua action |
| `/api/amy/nodes/{id}/video` | GET | MJPEG from camera node |
| `/api/amy/simulation/targets` | GET | List simulation targets |
| `/api/amy/simulation/spawn` | POST | Spawn hostile target |

## Testing

```bash
# Amy unit tests
.venv/bin/python3 -m pytest tests/amy/ -m unit -v

# Amy brain tests
.venv/bin/python3 -m pytest tests/amy/brain/ -v

# Amy API tests
.venv/bin/python3 -m pytest tests/amy/api/ -v
```

## Related

- [../engine/](../engine/) — System infrastructure Amy plugs into
- [../engine/commander_protocol.py](../engine/commander_protocol.py) — Interface Amy implements
- [../../docs/SIMULATION.md](../../docs/SIMULATION.md) — Battle simulation Amy commands
- [../../docs/ESCALATION.md](../../docs/ESCALATION.md) — Threat escalation Amy monitors
- [../../docs/USER-STORIES.md](../../docs/USER-STORIES.md) — UX specs for Amy's panels
