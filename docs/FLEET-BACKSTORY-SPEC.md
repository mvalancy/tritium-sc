<!-- Created by Matthew Valancy -->
<!-- Copyright 2026 Valpatel Software LLC -->
<!-- Licensed under AGPL-3.0 — see LICENSE for details. -->

# Fleet Backstory Spec: Distributed Backstory Generation & Structured Vision Prompts

**Status**: SPECIFICATION — not yet implemented.

**Dependencies**: OllamaFleet (`src/engine/inference/fleet.py`),
ModelRouter (`src/engine/inference/model_router.py`),
MissionDirector (`src/engine/simulation/mission_director.py`),
SimulationTarget (`src/engine/simulation/target.py`),
unit type registry (`src/engine/units/`).

---

## 1. Problem Statement

Units currently get generic names or short scripted backstories from a template pool in `UnitMissionSystem`. The `request_llm_backstory()` method queues targets for LLM backstory generation, but the actual LLM call is never executed — `_pending_backstories` is populated but never drained. No background worker picks up pending requests.

Vision model prompts are currently ad-hoc. `ollama_chat()` in `src/engine/perception/vision.py` accepts raw messages with no structure or validation. Each caller constructs its own prompt string. No standard exists for what we ask llava/minicpm-v to analyze, how to format requests, or how to parse responses.

---

## 2. Feature 1: Distributed Unit Backstory Generation

### Architecture: BackstoryGenerator

**New file**: `src/engine/simulation/backstory.py`

The `BackstoryGenerator` orchestrates distributed backstory generation as a background daemon, plugging into `UnitMissionSystem` as the backstory fulfillment backend.

```
engine.add_target()
  -> unit_missions.request_llm_backstory(target)
    -> backstory_generator.enqueue(target, priority)
      -> (background workers)
        -> fleet.generate(model, prompt)
          -> parse + validate JSON
            -> unit_missions._backstories[target_id] = result
            -> event_bus.publish("backstory_generated", {...})
              -> WebSocket bridge -> frontend
              -> thought_registry.set_thought() (intro thought)
```

### Class Design

```python
class BackstoryGenerator:
    def __init__(
        self,
        fleet: OllamaFleet,
        event_bus: EventBus,
        cache_dir: Path = Path("data/backstories"),
        max_concurrent: int = 3,
        bulk_model: str = "gemma3:1b",
        key_character_model: str = "gemma3:4b",
    ): ...
```

Workers are threads (matching `LLMThinkScheduler` pattern). Priority queue uses sorted dataclass items with `order=True`. Rate limiting uses `_min_interval` + `_call_lock` pattern.

### Priority System

| Priority | Value | Category | Model |
|----------|-------|----------|-------|
| Critical | 0.1 | Named defenders (turrets, rovers, drones) | `gemma3:4b` |
| High | 0.3 | Hostile leaders, tanks, APCs | `gemma3:4b` |
| Medium | 0.5 | Regular hostiles | `gemma3:1b` |
| Low | 0.7 | Named NPCs (delivery drivers, joggers) | `gemma3:1b` |
| Background | 0.9 | Generic neutral traffic | `gemma3:1b` |

### Backstory Schema

**Shared required fields** (all alliance types):
- `name` (string): Real-sounding name or callsign
- `background` (string): 1-2 sentences of history
- `motivation` (string): Why they are here right now
- `personality_traits` (list of strings): 2-4 trait words
- `speech_pattern` (string): How they talk/communicate

**Type-specific fields**:
- Defenders: + `neighborhood_relationship`, `tactical_preference`
- Hostiles: + `tactical_preference`
- Neutrals: + `daily_routine`, `neighborhood_relationship`

### Structured Prompts

**System prompt (shared)**:
```
You are a narrative designer for a neighborhood security simulation set in
West Dublin, California. Generate a backstory for a simulation unit.
Respond with valid JSON only. No markdown, no extra text.
```

Per-alliance templates include unit type, name, position, approach direction, and required JSON schema in the prompt.

### Validation & Retry

1. Strip markdown code blocks
2. Try `json.loads()`
3. Regex for `{...}` within response
4. Validate required keys per alliance type
5. On failure: retry once with same host
6. On second failure: fall back to scripted backstory

### Disk Cache

**Location**: `data/backstories/` (gitignored)

Cache key: `(alliance, asset_type, name_hash)` — same unit type + name gets same backstory across sessions. Loaded lazily. O(1) lookup via in-memory dict from `index.json`.

### Fleet Distribution

Uses `OllamaFleet` for discovery (conf file, env var, Tailscale scan, localhost fallback). Weighted round-robin for load distribution instead of `best_host()`:

```python
def _select_host(self, model):
    hosts = self._fleet.hosts_with_model(model)
    weights = [1.0 / max(h.latency_ms, 1.0) for h in hosts]
    return random.choices(hosts, weights=weights, k=1)[0]
```

Fallback: retry different host, then scripted backstory after 2 failures.

### Privacy

All generation is local. Fleet communicates only with Ollama instances on operator-controlled machines. Backstories cached to local disk only.

### Integration Points

- **UnitMissionSystem**: Add `set_backstory_generator()`, delegate `request_llm_backstory()`
- **SimulationEngine**: Create `BackstoryGenerator` in `__init__`, wire fleet + event_bus
- **WebSocket Bridge**: Forward `backstory_generated` and `backstory_progress` events
- **Frontend**: `websocket.js` handles events, `units.js` shows details, thought bubbles via `ThoughtRegistry`
- **Graphlings Exclusion**: Skip units controlled by Graphlings plugin

### Configuration

```python
backstory_enabled: bool = True
backstory_bulk_model: str = "gemma3:1b"
backstory_key_model: str = "gemma3:4b"
backstory_max_concurrent: int = 3
backstory_cache_dir: str = "data/backstories"
```

### Performance Budget

- Defenders: backstories within 10s of game start
- Hostiles per wave: within 30s of spawn
- Neutral NPCs: minutes (background)
- Cache lookup: O(1)

---

## 3. Feature 2: Clean Structured Vision Model Prompts

### VisionPromptTemplate

**New file**: `src/engine/perception/vision_prompts.py`

```python
@dataclass(frozen=True)
class VisionPromptTemplate:
    template_id: str           # "scene_description", "person_count", etc.
    system_prompt: str
    user_prompt: str           # Template with {placeholders}
    response_schema: dict      # JSON schema for expected response
    temperature: float = 0.3
    max_tokens: int = 256
    model_overrides: dict[str, dict] = field(default_factory=dict)
```

### 6 Analysis Types

1. **scene_description** — General scene understanding
2. **person_count** — Count people and describe activities
3. **threat_assessment** — Evaluate hostile indicators
4. **equipment_id** — Identify vehicles, packages, tools
5. **weather_conditions** — Assess environmental conditions
6. **change_detection** — Compare frames and describe changes

### VisionPromptManager

```python
class VisionPromptManager:
    def get_template(self, template_id) -> VisionPromptTemplate | None
    def register_template(self, template) -> None
    def build_messages(self, template_id, model, images, context=None) -> list[dict]
    def parse_response(self, template_id, raw_response) -> dict | None
    def analyze(self, template_id, model, images, context=None,
                fleet=None, max_retries=2) -> dict | None
```

### Model-Specific Tuning

- **llava:7b**: Good at scene description/spatial reasoning. Emphasize "JSON only" in prompts. Temp 0.2-0.3.
- **minicpm-v:8b**: Better structured output, more precise counts. Can handle complex schemas. Temp 0.3-0.4.

### Retry Logic

1. Standard prompt → parse
2. Add "Your previous response was not valid JSON" → re-parse
3. Replace with schema-only prompt → re-parse
4. Return None after exhaustion

### Integration with Perception Pipeline

VisionPromptManager slots into L3+ analysis. Template selection is context-dependent:
- Motion detected → `scene_description`
- People detected by YOLO → `person_count`
- Elevated threat level → `threat_assessment`
- New object → `equipment_id`
- Periodic (5 min) → `weather_conditions`
- Reference aged > 30 min → `change_detection`

---

## 4. Implementation Plan

### New Files

| File | Lines (est.) | Purpose |
|------|-------------|---------|
| `src/engine/simulation/backstory.py` | ~400 | BackstoryGenerator, cache, prompts |
| `src/engine/perception/vision_prompts.py` | ~500 | VisionPromptTemplate, Manager, 6 templates |
| `tests/engine/simulation/test_backstory.py` | ~600 | 28 unit tests |
| `tests/engine/perception/test_vision_prompts.py` | ~500 | 18 unit tests |

### Modified Files

| File | Changes |
|------|---------|
| `src/engine/simulation/target.py` | Add `backstory: dict \| None = None` field (+5 lines) |
| `src/engine/simulation/mission_director.py` | Call `generate_all()` after scenario completion (+15 lines) |
| `src/engine/simulation/engine.py` | Store BackstoryGenerator, wire events (+10 lines) |
| `src/engine/simulation/unit_missions.py` | Add `set_backstory_generator()` method (+10 lines) |
| `src/app/routers/game.py` | Add `GET /api/game/backstories` endpoint (+20 lines) |
| `src/app/routers/ws.py` | Forward backstory events (+5 lines) |
| `src/app/config.py` | Add backstory settings (+5 lines) |
| `src/frontend/js/command/panels/units.js` | Display backstory details (+25 lines) |
| `src/frontend/js/command/websocket.js` | Handle backstory events (+10 lines) |

### Phase Ordering

1. **Phase 1**: BackstoryGenerator core (standalone, no integration)
2. **Phase 2**: Fleet distribution (wire to OllamaFleet)
3. **Phase 3**: SimulationTarget integration (add field, wire events)
4. **Phase 4**: Frontend display (tooltips, units panel)
5. **Phase 5**: Amy integration (thinking context, announcer)
6. **Phase 6**: Vision prompt templates (independent of Phases 1-5)
7. **Phase 7**: Vision pipeline integration

---

## 5. Fleet Topology Considerations

| Hosts | Behavior |
|-------|----------|
| **1 host** | Sequential, 5-15s for 5 units. Backstories trickle in during loading. |
| **3 hosts** | Full parallelism, ~1-3s total. Sweet spot. |
| **10 hosts** | More hosts than units. Each backstory on different host. ~1-2s total. |
| **0 hosts** | Scripted fallbacks used immediately. No LLM calls. |
| **Partition** | Timeout (30s), retry other host, scripted fallback per-unit on total failure. |

---

## 6. Test Plans

### Backstory Tests (28 tests) — `tests/engine/simulation/test_backstory.py`

- Generator init, priority assignment per alliance (5 tests)
- Cache hit/miss, write, stable keys (4 tests)
- Validation per alliance type, JSON parsing (5 tests)
- Retry on malformed, fallback after failures (3 tests)
- Prompt building per alliance (3 tests)
- Host selection round-robin (2 tests)
- Event emission, thought bubble, name update (3 tests)
- Concurrent limits, rate limiting, stop/drain (3 tests)

### Fleet Integration Tests (7 tests, `@pytest.mark.network`)

- Real LLM generates valid JSON per alliance type (3 tests)
- Fleet distributes across hosts (1 test)
- Full game: 10 units backstories within 60s (1 test)
- Cache survives restart (1 test)
- Model fallback on missing (1 test)

### Vision Prompt Tests (18 tests) — `tests/engine/perception/test_vision_prompts.py`

- Template management (4 tests)
- Message building with placeholders and model overrides (4 tests)
- Response parsing and validation (5 tests)
- Retry logic escalation (3 tests)
- End-to-end with mock Ollama (2 tests)

### JS Tests (4 tests) — `tests/js/test_backstory_frontend.js`

- WebSocket event updates store
- Panel shows backstory
- Progress event handled
- Missing backstory graceful fallback
