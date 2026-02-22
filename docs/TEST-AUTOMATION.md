# TRITIUM-SC Automated Test Infrastructure

## Design Philosophy

Every claim must be machine-verified. No manual steps. No "looks right".
If a human has to look at it, it's not verified.

### Three-Layer Verification (AND, never OR)

Every visual test runs ALL three layers. A test FAILS if ANY layer fails.

```
Layer 1: OpenCV Pixel Assertions (fast, deterministic, 0ms)
  - assert_color_present(image, BGR, tolerance, min_pixels)
  - count_color_blobs(image, BGR, tolerance, min_area)
  - assert_region_not_blank(image, x, y, w, h)
  - assert_region_has_text(image, x, y, w, h)

Layer 2: Structured LLM Queries (llava:7b via fleet, ~2-8s each)
  - ask_yes_no(image, question) -> bool
  - ask_count(image, question) -> int
  - ask_choice(image, question, choices) -> str
  - ask_describe(image, question) -> str

Layer 3: API Cross-Validation (ground truth, ~50ms)
  - GET /api/game/state -> wave, score, kills
  - GET /api/amy/simulation/targets -> target counts by alliance
  - POST /api/game/begin, /api/game/reset, /api/game/place
```

### Why AND, not OR

The old `test_gameplay.py` used `passed = api_ok or visual_ok`. This means:
- API says "active" but canvas renders nothing? PASS.
- Canvas shows shapes but API is dead? PASS.
- Both lie in complementary ways? PASS.

With AND logic: if the canvas doesn't show green turrets, the test fails even
if the API says they exist. This catches render bugs the API can't see.

### Structured LLM Prompts

llava:7b is unreliable with open-ended questions. Structured prompts force
specific answer formats that can be parsed deterministically:

```
BAD:  "Describe this image"  -> unparseable prose
GOOD: "Are green squares visible? Answer YES or NO."  -> parse first word
GOOD: "How many red shapes? Answer with one number."  -> extract first int
```

Parsing rules:
- YES/NO: check if "yes" in first 20 chars of lowered response
- COUNT: regex `\d+` on response, default 0 if no match
- CHOICE: check which choice word appears in lowered response

### LLM Reliability (Validated 2026-02-20)

**Measured false negative rates** (3 hosts, synthetic + real screenshots):

| Query Type | Single Shot | Majority Vote (3) |
|------------|-------------|-------------------|
| YES/NO on dark bg | 20-83% false negatives | ~4-12% false negatives |
| Counting (30px shapes) | Non-deterministic (0-13 for 2) | Median stabilizes |
| Choice | Chose "triangles" for squares | Majority helps somewhat |

**Mitigation strategy** (in `visual_assert.py`):
- All YES/NO queries use majority vote (3 calls, 2 must agree)
- All counting queries use median of 3 calls
- Tiered pass logic: LLM advisory when OpenCV + API both pass
- OpenCV + API are deterministic — they gate the test result
- LLM failures logged for human review but don't block

**Known limitations** (unfixable with llava:7b):
- Neon text on dark backgrounds (20%+ false negative rate persists)
- Small shapes (<15px) often missed entirely
- Counting above 5 is unreliable even with median
- "Diamond" shape often returned as "triangle" or "polygon"
- Different hosts give different answers for same image
- Cross-host consistency: remote-a hallucinated 13 shapes where 2 existed

## Architecture

### File Layout

```
tests/
  lib/
    ollama_fleet.py     # Fleet discovery + parallel inference (existing)
    test_db.py          # SQLite results store
    visual_assert.py    # Three-layer verification
    server_manager.py   # Automated server lifecycle
    report_gen.py       # HTML report generator
    test_test_db.py     # Tests for TestDB
    test_ollama_fleet.py # Tests for OllamaFleet
    test_visual_assert.py # Tests for VisualAssert
  visual/
    conftest.py         # Shared fixtures (fleet, db, server, va)
    test_war_room.py    # War Room three-layer tests
    test_battle_e2e.py  # Full battle E2E with SQLite recording
    test_regression.py  # SSIM screenshot baselines
  .test-results/        # SQLite DB + HTML reports (gitignored)
  .baselines/           # Golden screenshots for regression (gitignored except golden/)
```

### Color Constants (BGR for OpenCV)

Source of truth: `tests/lib/visual_assert.py`

```python
FRIENDLY_GREEN = (161, 255, 5)    # CSS #05ffa1
HOSTILE_RED    = (109, 42, 255)   # CSS #ff2a6d
CYAN_PRIMARY   = (255, 240, 0)    # CSS #00f0ff
YELLOW_UNKNOWN = (10, 238, 252)   # CSS #fcee0a
VOID_BLACK     = (15, 10, 10)     # CSS #0a0a0f
DARK_BG        = (26, 18, 18)     # CSS #12121a
```

### Ollama Fleet

3 hosts discovered automatically:
- `local`: 13 models, llava:7b, ~6ms latency
- `remote-a`: 13 models, llava:7b, ~12ms latency
- `remote-b`: 39 models, llava:7b/13b/34b, ~648ms latency

`parallel_vision()` round-robins tasks across all hosts with the model.

### Server Manager

`TritiumServer` starts uvicorn on an auto-assigned port with:
- `AMY_ENABLED=false` (no audio/camera hardware needed)
- `MQTT_ENABLED=false` (no broker needed)
- `SIMULATION_ENABLED=true` (game engine active)
- `CUDA_VISIBLE_DEVICES=""` (avoid GPU crashes)

Health polling at 0.5s intervals, 30s timeout.

### SQLite Test Database

Schema: `runs` -> `results` + `screenshots`

Every test run gets a `run_id`. Every screenshot gets recorded with:
- OpenCV assertion results
- LLM response text + host + latency
- API state at time of capture

Reports generated as self-contained HTML with inline base64 thumbnails.

## Test Tiers

```
Tier 1: Syntax Check (py_compile, node --check)
Tier 2: Unit Tests (pytest tests/amy/ -m unit) — 1353 tests
Tier 3: JS Tests (node tests/js/test_war_math.js) — 28 tests
Tier 4: Vision Audit (llava multi-model, multi-resolution)
Tier 5: E2E (Playwright spec files)
Tier 6: Battle Verification (fleet-parallel llava, 8 phases)
Tier 7: Visual E2E (three-layer, pytest tests/visual/)
Tier 8: Test Infrastructure (pytest tests/lib/ -m unit) — 47 tests
```

`./test.sh fast` = tiers 1+2+3+8
`./test.sh all` = all 8 tiers
`./test.sh --dist` = distributed across local + $REMOTE_HOST

## Distributed Execution

`./test.sh --dist` syncs code to `$REMOTE_HOST` via rsync, then runs:
- local: unit tests + vision audit (first 5 views)
- remote: unit tests + vision audit (last 5 views) + lib tests

Both machines run in parallel. Results merged in summary.

## Report Generation

```bash
# Generate HTML report for latest run
.venv/bin/python3 -m tests.lib.report_gen --latest

# Generate for specific run
.venv/bin/python3 -m tests.lib.report_gen --run-id 42

# Open in browser
.venv/bin/python3 -m tests.lib.report_gen --latest --serve
```

Optional LLM executive summary via qwen2.5:7b on any fleet host.

## Regression Baselines

```bash
# Capture initial baselines
pytest tests/visual/test_regression.py --update-baselines

# Compare against baselines (SSIM threshold 0.85)
pytest tests/visual/test_regression.py -v
```

Below 0.85 SSIM = visual regression detected.
