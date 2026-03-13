# Tests

**Where you are:** `tritium-sc/tests/` — the test suite for the Command Center.

**Parent:** [../CLAUDE.md](../CLAUDE.md) | [../../CLAUDE.md](../../CLAUDE.md) (tritium root)

## Test Tiers

Run with `./test.sh`:

| Tier | Command | What | Count | Time |
|------|---------|------|-------|------|
| 1 | `./test.sh 1` | Syntax check (Python + JS) | 31 files | ~2s |
| 2 | `./test.sh 2` | Python unit tests | 1666 | ~45s |
| 3 | `./test.sh 3` | JS tests (math, audio, fog, geo, panels) | 281 | ~3s |
| 8 | `./test.sh 8` | Test infrastructure | 62 | ~1s |
| 8b | `./test.sh 8b` | ROS2 robot tests | 125 | ~1s |
| 9 | `./test.sh 9` | Integration (headless server E2E) | 23 | ~70s |
| 10 | `./test.sh 10` | Visual quality (Playwright) | 7 | ~30s |
| 7 | `./test.sh 7` | Visual E2E (three-layer verification) | 23 | ~13min |

**Quick:** `./test.sh fast` — tiers 1-3 + 8 + 8b (~60s)

**Everything:** `./test.sh all` (~15 min)

## Directory Structure

```
tests/
├── engine/                  # System infrastructure tests
│   ├── simulation/          # Simulation engine (48 files)
│   ├── comms/               # CoT, MQTT, event bus, speaker (20+)
│   ├── tactical/            # Geo, escalation
│   ├── api/                 # FastAPI router tests (21)
│   ├── nodes/               # Sensor nodes, MQTT, ML (16)
│   ├── actions/             # Lua, dispatch, formation
│   ├── inference/           # Model router, fleet
│   ├── perception/          # Perception, extraction
│   ├── units/               # Unit type registry
│   ├── synthetic/           # Video, audio generation (14)
│   ├── scenarios/           # Behavioral tests (6)
│   ├── audio/               # Audio pipeline
│   └── models/              # Data models (10)
├── amy/                     # Amy personality tests
│   ├── core/                # Commander, thinking, memory, sensorium
│   ├── brain/               # Thinking battle tests
│   └── api/                 # Amy API tests
├── js/                      # JavaScript tests (281)
├── lib/                     # Test infrastructure (62)
├── integration/             # Server E2E (23)
├── visual/                  # Three-layer E2E (23)
├── ui/                      # Vision audit, gameplay, battle
├── scenarios/               # Behavioral test scenarios (JSON)
├── e2e/                     # End-to-end tests
├── perf/                    # Performance benchmarks
├── docs/                    # Test documentation tools
├── .baselines/              # Golden test baselines
└── .test-results/           # Visual test result snapshots
```

## Running Individual Tests

```bash
# Specific file
.venv/bin/python3 -m pytest tests/engine/simulation/test_combat.py -v

# Specific subsystem
.venv/bin/python3 -m pytest tests/amy/ -m unit -v

# With coverage
.venv/bin/python3 -m pytest tests/ --cov=src/
```

## Related

- [../docs/TESTING-PHILOSOPHY.md](../docs/TESTING-PHILOSOPHY.md) — TDD principles
- [../docs/TEST-AUTOMATION.md](../docs/TEST-AUTOMATION.md) — E2E automation
- [../docs/UI-TESTING.md](../docs/UI-TESTING.md) — Visual regression
