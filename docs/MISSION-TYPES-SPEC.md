<!-- Created by Matthew Valancy -->
<!-- Copyright 2026 Valpatel Software LLC -->
<!-- Licensed under AGPL-3.0 — see LICENSE for details. -->

# Mission Types Specification: Civil Unrest & Drone Swarm

**Status**: Draft
**Author**: Matthew Valancy
**Date**: 2026-02-28
**Depends on**: `src/engine/simulation/mission_director.py`, `src/engine/simulation/game_mode.py`, `src/engine/simulation/behaviors.py`, `src/engine/simulation/engine.py`

---

## Table of Contents

1. [Overview](#1-overview)
2. [Civil Unrest](#2-civil-unrest)
   - [Scenario Concept](#21-scenario-concept)
   - [Wave Composition](#22-wave-composition)
   - [Unique Mechanics](#23-unique-mechanics)
   - [Defender Composition](#24-defender-composition)
   - [Win/Loss Conditions](#25-winloss-conditions)
   - [Scripted Context Templates](#26-scripted-context-templates)
   - [LLM Prompt Additions](#27-llm-prompt-additions)
   - [Test Plan](#28-test-plan)
3. [Drone Swarm](#3-drone-swarm)
   - [Scenario Concept](#31-scenario-concept)
   - [Wave Composition](#32-wave-composition)
   - [Unique Mechanics](#33-unique-mechanics)
   - [Defender Composition](#34-defender-composition)
   - [Win/Loss Conditions](#35-winloss-conditions)
   - [Scripted Context Templates](#36-scripted-context-templates)
   - [LLM Prompt Additions](#37-llm-prompt-additions)
   - [Test Plan](#38-test-plan)
4. [Implementation Plan](#4-implementation-plan)
   - [Python Backend Changes](#41-python-backend-changes)
   - [Frontend Changes](#42-frontend-changes)
   - [Test File Locations and Counts](#43-test-file-locations-and-counts)
   - [Phase Ordering](#44-phase-ordering)

---

## 1. Overview

This document specifies two new game modes for the TRITIUM-SC simulation engine, extending the existing four modes (battle, defense, patrol, escort). Both modes integrate into the existing `MissionDirector` (scripted + LLM generation), `GameMode` (wave progression + scoring), and `UnitBehaviors` (per-tick AI) systems.

The frontend modal (`mission-modal.js`) already lists both modes in its `GAME_MODES` constant:

```js
civil_unrest: { label: 'CIVIL UNREST', description: 'Riots and crowd control scenario', icon: 'U' },
drone_swarm: { label: 'DRONE SWARM', description: 'Mass drone attack defense', icon: 'S' },
```

Backend registration in `mission_director.py`'s `GAME_MODES` dict is the primary gating factor. Both modes also require new unit behaviors, new hostile asset sub-types, and mode-specific scoring rules in `GameMode`.

### Design Constraints

- Both modes must work with scripted generation (no LLM required).
- Both modes must work with LLM generation when Ollama is available.
- Both modes must produce a valid `BattleScenario` (see `scenario.py`: `WaveDefinition`, `SpawnGroup`, `DefenderConfig`).
- Both modes must integrate with existing subsystems: combat, morale, squads, comms, cover, terrain, degradation, difficulty scaling, replay recording.
- Wave composition data flows through `_briefings_to_composition()` for scripted and through LLM JSON for generated.
- No new frontend framework dependencies.

### Existing Patterns

The `MissionDirector` follows an 8-step generation pipeline (`GENERATION_STEPS`):
1. scenario_context -- backstory, attacker, stakes
2. unit_composition -- defenders + wave hostile plan
3. unit_motives -- per-unit objectives and ROE
4. win_conditions -- victory/defeat/bonus
5. weather_atmosphere -- conditions and mood
6. loading_messages -- briefing screen status lines
7. wave_briefings -- narrative per-wave intel
8. wave_composition -- concrete spawn data (types, counts, multipliers)

Each step has a prompt template for LLM generation and scripted fallback data. The `generate_scripted()` method resolves context templates against POI data (`{center_name}`, `{center_address}`, `{streets}`), places defenders around buildings, and constructs wave compositions from briefings via `_briefings_to_composition()`.

The `GameMode` class manages state transitions (setup -> countdown -> active -> wave_complete -> ... -> victory/defeat), wave spawning via background threads with staggered timing, elimination scoring, and EventBus event publishing.

The `UnitBehaviors` class runs per-tick AI: turret rotation and fire, drone strafe runs, rover advance and engage, hostile flanking/dodging/group rush/cover-seeking. It integrates with vision state, terrain LOS, spatial grid, and comms signals.

---

## 2. Civil Unrest

### 2.1 Scenario Concept

A crowd control scenario where large groups of civilians are gathering, some turning violent. The challenge is maintaining order without excessive force. The operator commands non-lethal assets (rovers, drones for overwatch) to contain crowds, identify instigators hidden within groups, and protect critical infrastructure.

Unlike standard battle mode where all hostiles are valid targets, civil unrest requires discrimination: most "hostiles" are actually civilians who must not be harmed. Only identified instigators are valid engagement targets. Collateral damage to civilians reduces the score and can trigger defeat.

**Thematic influence**: The neighborhood around a government building, school, or community center becomes the staging ground. POI data drives the focal point -- a town hall hearing gone wrong, a school board meeting that escalated, a park gathering that turned into a protest march.

**Time scale**: 8 waves, each representing an escalation phase (peaceful assembly -> heated protest -> isolated scuffles -> coordinated riot -> vehicular chaos -> looting -> standoff -> resolution attempt). Total play time target: 8-12 minutes.

### 2.2 Wave Composition

Each wave spawns a mix of `person` entities classified into three sub-roles that modify their behavior. Sub-role is encoded via the existing `motive` field on `SimulationTarget` (already present but unused by current hostile behavior) and a new `crowd_role` field.

| Wave | Name | Civilians | Instigators | Vehicles | Speed Mult | Health Mult |
|------|------|-----------|-------------|----------|------------|-------------|
| 1 | Peaceful Assembly | 12 | 0 | 0 | 0.5 | 0.8 |
| 2 | Heated Protest | 15 | 2 | 0 | 0.6 | 0.8 |
| 3 | Isolated Scuffles | 18 | 4 | 0 | 0.7 | 1.0 |
| 4 | Coordinated Riot | 20 | 6 | 1 | 0.8 | 1.0 |
| 5 | Vehicular Chaos | 15 | 5 | 3 | 1.0 | 1.2 |
| 6 | Looting Surge | 25 | 8 | 2 | 1.1 | 1.0 |
| 7 | Armed Standoff | 10 | 8 | 2 | 0.6 | 1.5 |
| 8 | Final Escalation | 20 | 10 | 4 | 1.2 | 1.5 |

Total hostiles: 135 civilians + 43 instigators + 12 vehicles = 190 entities across all 8 waves.

**Crowd role definitions**:

- **Civilian** (`crowd_role: "civilian"`): `alliance: "hostile"` for targeting engine compatibility but with `is_combatant: false`. Moves in random directions within the crowd zone. Does not attack. Scoring penalty if eliminated by defenders.
- **Instigator** (`crowd_role: "instigator"`): `alliance: "hostile"`, `is_combatant: true`. Blends with civilians, periodically "activates" (throws objects, starts fires = low-damage ranged attack). Once activated, can be identified by vision system. Higher damage than standard person but low health.
- **Rioter** (`crowd_role: "rioter"`): Converted civilian. When an instigator activates near civilians, nearby civilians have a 20% chance per tick of converting to rioter status (`is_combatant: true`, low damage, melee range only). Rioters can be de-escalated back to civilian status by sustained rover presence within 15m (3 seconds of proximity without firing).

**Vehicles in later waves**: `hostile_vehicle` type representing commandeered cars used as barricades or ramming weapons. Low speed (0.5), high health (250), does not fire but deals contact damage to defenders within 3m.

### 2.3 Unique Mechanics

#### 2.3.1 Crowd Density Zones

A new spatial concept tracked per-tick in a `CrowdDensityTracker` (new class). The map is divided into a grid (10m cells). Each cell tracks the count of person-type entities present. Density levels:

| Density | Persons/Cell | Effect |
|---------|-------------|--------|
| Sparse | 0-2 | Normal movement, easy identification |
| Moderate | 3-5 | Reduced identification accuracy, civilian conversion risk |
| Dense | 6-10 | Instigators invisible to vision, civilian conversion 2x risk |
| Critical | 11+ | No identification possible, maximum conversion risk, property damage ticks |

Crowd density is published as a `crowd_density` event on EventBus each second for frontend heatmap rendering.

#### 2.3.2 De-escalation Scoring

A parallel scoring track alongside the standard elimination scoring:

- **De-escalation points**: +200 per rioter de-escalated back to civilian
- **Instigator identified**: +150 per instigator correctly identified (vision confirms activation before engagement)
- **Civilian protected**: +50 per civilian that exits the map safely (despawns without harm)
- **Civilian harmed**: -500 per civilian eliminated by friendly fire
- **Property damage prevented**: +100 per wave where no critical density zones form near POI buildings

The `de_escalation_score` is tracked separately on the `GameMode` instance and reported in `get_state()`. Final scoring is a weighted combination: `total_score = combat_score * 0.3 + de_escalation_score * 0.7`. This weighting makes restraint more valuable than firepower.

#### 2.3.3 Civilian Protection Priority

Defenders are tagged with `rules_of_engagement: "restrictive"` in their objectives. This modifies behavior:

- Turrets are excluded from this mode (no turrets in default defender composition).
- Rovers only fire at positively identified instigators (`crowd_role == "instigator"` AND `instigator_activated == True`).
- Drones never fire; they serve as identification platforms (vision cone reveals instigator status).
- Scout drones mark instigators for rover engagement (comms signal `SIGNAL_CONTACT` with `instigator_id`).

If a defender fires on a civilian, the `civilian_harm_count` increments and the event `civilian_harmed` is published on EventBus. At 5+ civilian casualties, an automatic defeat is triggered (excessive force).

#### 2.3.4 Instigator Activation Cycle

Instigators follow a state cycle: `hidden` -> `activating` (2s telegraph) -> `active` (5s visible, attacking) -> `hidden` (cooldown 8s). During `activating`, they display a visual tell (animation/status change) that the vision system can detect. During `active`, they throw projectiles (reusing combat system with a new `thrown_object` projectile type: range 15m, damage 5, cooldown 3s). During `hidden`, they are indistinguishable from civilians.

This cycle is managed by extending the hostile behavior in `behaviors.py` with a new `_instigator_behavior()` method.

#### 2.3.5 Rover De-escalation Mechanic

Rovers gain a `de_escalate_mode` toggle (default `True` in civil unrest). When enabled and within 15m of a rioter, the rover emits a `SIGNAL_REGROUP` comms signal that triggers de-escalation instead of firing. The rioter's `crowd_role` transitions from `"rioter"` back to `"civilian"` after 3 seconds of sustained proximity. The rover must not fire during this period. If the rover fires, the de-escalation resets and nearby civilians have a 30% conversion-to-rioter chance (escalation feedback).

### 2.4 Defender Composition

```python
"civil_unrest": {
    "description": "Crowd control and de-escalation scenario",
    "prompt": (
        "Design a crowd control and de-escalation scenario for a neighborhood security system. "
        "A large crowd has gathered, with hidden instigators trying to incite violence. "
        "The security system must identify instigators, protect civilians, and restore order "
        "WITHOUT lethal force. Only rovers, drones, and scout drones are available. "
        "Respond in JSON format: "
        '{"trigger_event": "...", "crowd_size": 100, "instigator_count": 10, '
        '"escalation_phases": 8, "civilian_sentiment": "angry|scared|mixed", '
        '"time_of_day": "...", "theme": "..."}'
    ),
    "default_waves": 8,
    "default_defenders": [
        {"type": "rover", "count": 3},       # Primary crowd control
        {"type": "drone", "count": 2},        # Overwatch / identification
        {"type": "scout_drone", "count": 2},  # Perimeter / instigator marking
    ],
    "default_hostiles_per_wave": 15,
}
```

No turrets, no heavy weapons, no tanks, no APCs. This is deliberate. The mode tests restraint and identification, not firepower. The defender composition enforces this at the scenario generation level.

POI-based placement: rovers are placed on approach streets (crowd control cordons), drones over the center POI (overwatch), scout drones at the perimeter (early identification of approaching crowds).

### 2.5 Win/Loss Conditions

**Victory**: All 8 waves completed with `civilian_harm_count < 5` and at least 60% of total civilians safely despawned.

**Defeat** (any of):
- 5 or more civilians eliminated by friendly fire (excessive force)
- Critical density zone persists on a POI building for 60+ seconds (infrastructure overwhelmed)
- All defender units eliminated

**Bonus objectives**:
- "Zero Collateral" -- complete with 0 civilian casualties (+2000 pts)
- "Master De-escalator" -- de-escalate 20+ rioters back to civilian (+1500 pts)
- "All Instigators Identified" -- identify and neutralize every instigator before wave ends (+1000 pts per wave achieved)
- "Quick Containment" -- no critical density zones form at all (+1000 pts)

Win conditions as structured data for `generate_scripted()`:
```python
{
    "victory": {
        "condition": "Survive all 8 escalation phases with fewer than 5 civilian casualties",
        "description": "De-escalate the situation and restore order to the neighborhood.",
    },
    "defeat": {
        "condition": "5+ civilian casualties OR infrastructure overwhelmed OR all defenders eliminated",
        "description": "Excessive force, unchecked destruction, or total defensive failure.",
    },
    "bonus_objectives": [
        {"name": "Zero Collateral", "description": "Complete with 0 civilian casualties", "reward": 2000},
        {"name": "Master De-escalator", "description": "De-escalate 20+ rioters back to civilian", "reward": 1500},
        {"name": "All Instigators Identified", "description": "Neutralize every instigator per wave", "reward": 1000},
        {"name": "Quick Containment", "description": "No critical density zones form", "reward": 1000},
    ],
}
```

### 2.6 Scripted Context Templates

Four narrative variations using POI data `{center_name}`, `{center_address}`, `{streets}`:

```python
_CIVIL_UNREST_CONTEXT_TEMPLATES = [
    {
        "reason": "A contentious zoning decision has drawn hundreds of protesters to {center_name}. "
                  "Agitators are embedded in the crowd, turning a legal assembly into a flashpoint.",
        "attacker_name": "Embedded Agitators",
        "attacker_motivation": "Provoke overreaction from security forces",
        "stakes": "Protecting both the crowd and {center_address} without a PR disaster",
        "urgency": "high",
        "atmosphere": "Chanting echoes off the buildings, punctuated by breaking glass",
    },
    {
        "reason": "A viral social media post has mobilized a flash mob at {center_name}. "
                  "What started as a peaceful vigil is being hijacked by outside agitators arriving via {streets}.",
        "attacker_name": "Outside Agitators Network",
        "attacker_motivation": "Create chaos for media attention",
        "stakes": "De-escalate before the situation becomes national news",
        "urgency": "medium",
        "atmosphere": "Phone flashlights flicker like a sea of stars, but the mood is shifting",
    },
    {
        "reason": "A power outage has triggered looting near {center_name}. Opportunistic criminals are "
                  "using the blackout as cover. Most people on the street are scared residents, not looters.",
        "attacker_name": "Opportunistic Looters",
        "attacker_motivation": "Theft under cover of darkness",
        "stakes": "Restoring order near {center_address} while protecting displaced residents",
        "urgency": "critical",
        "atmosphere": "Emergency lights cast red shadows. Car alarms compete with distant sirens",
    },
    {
        "reason": "A labor dispute at {center_name} has escalated. Striking workers have blocked {streets} "
                  "and unknown provocateurs are inciting violence to discredit the movement.",
        "attacker_name": "Unidentified Provocateurs",
        "attacker_motivation": "Sabotage legitimate labor action through violence",
        "stakes": "Identify provocateurs without suppressing the legal protest at {center_address}",
        "urgency": "medium",
        "atmosphere": "Bullhorns and chanting mix with the smell of smoke from burning tires",
    },
]
```

Fallback templates (no POI data) follow the same structure with generic location references:

```python
_CIVIL_UNREST_CONTEXTS_FALLBACK = [
    {
        "reason": "A contentious public decision has drawn hundreds of protesters. "
                  "Agitators are embedded in the crowd, turning a legal assembly into a flashpoint.",
        "attacker_name": "Embedded Agitators",
        "attacker_motivation": "Provoke overreaction from security forces",
        "stakes": "Protecting the crowd and infrastructure without escalating",
        "urgency": "high",
        "atmosphere": "Chanting echoes off the buildings, punctuated by breaking glass",
    },
    # ... 3 more fallback variants matching the above without {center_name} etc.
]
```

Mode-specific loading messages:

```python
_CIVIL_UNREST_LOADING = [
    "Activating crowd monitoring sensors...",
    "Calibrating non-lethal response protocols...",
    "Loading facial recognition watchlist...",
    "Mapping crowd density zones...",
    "Establishing communication cordons...",
    "Deploying overwatch drones...",
    "Analyzing social media feeds for flash mob indicators...",
    "Loading de-escalation playbook...",
    "Reviewing rules of engagement: RESTRICTIVE...",
    "Identifying critical infrastructure in area...",
    "Scanning for known agitator signatures...",
    "Establishing safe corridors for civilian egress...",
]
```

Mode-specific weather presets:

```python
_CIVIL_UNREST_WEATHER = [
    {"weather": "Warm evening, streetlights on", "visibility": "good", "temperature": "Warm",
     "wind": "Still", "special_conditions": "Urban heat island", "mood_description": "Charged atmosphere"},
    {"weather": "Overcast afternoon, drizzle", "visibility": "fair", "temperature": "Cool",
     "wind": "Light", "special_conditions": "Slick sidewalks", "mood_description": "Tension building under gray skies"},
    {"weather": "Hot midday sun", "visibility": "good", "temperature": "Hot",
     "wind": "None", "special_conditions": "Heat stress on crowd", "mood_description": "Tempers rising with the heat"},
]
```

### 2.7 LLM Prompt Additions

The `GAME_MODES["civil_unrest"]["prompt"]` (shown in section 2.4) provides the top-level generation prompt.

Mode-specific modifications to `GENERATION_STEPS` prompts are injected when `game_mode == "civil_unrest"`. These are appended to the base prompt_template for each step:

**scenario_context step**: Append:
> "This is a CIVIL UNREST scenario. The crowd is mostly civilians. Instigators are hidden among them. The security response MUST be non-lethal and proportionate. Include the trigger event, the crowd's grievance, and what's at stake if force is used disproportionately."

**unit_composition step**: Append:
> "Available defender types for civil unrest: rover (crowd control, de-escalation), drone (overwatch, identification), scout_drone (perimeter, marking). NO turrets, tanks, or heavy weapons. Rovers should be positioned on approach streets. Drones over the center."

**unit_motives step**: Append:
> "All units operate under RESTRICTIVE rules of engagement. Rovers may only engage positively identified instigators. Drones and scouts are observation-only. Civilian safety is the primary objective."

**win_conditions step**: Append:
> "Victory requires restraint. Civilian casualties cause defeat at 5+. Include de-escalation bonus objectives. The scoring formula weights de-escalation 70% and combat 30%."

**wave_composition step**: Replace hostile types instruction with:
> "Hostile types for civil unrest: person with crowd_role 'civilian' (non-combatant, scoring penalty if harmed), person with crowd_role 'instigator' (hidden, activates periodically, valid target when active), hostile_vehicle (commandeered vehicle, later waves only). Civilians outnumber instigators at least 2:1 in every wave."

### 2.8 Test Plan

Tests are organized across unit tests (pure logic), behavior tests (tick simulation), and integration tests (full scenario flow).

#### Unit Tests (`tests/engine/simulation/test_civil_unrest.py`)

1. **test_civil_unrest_game_mode_registered** -- `GAME_MODES["civil_unrest"]` exists with correct keys (description, prompt, default_waves=8, default_defenders, default_hostiles_per_wave).
2. **test_civil_unrest_no_turrets_in_defenders** -- `default_defenders` contains only rover, drone, scout_drone types. No turret, heavy_turret, missile_turret, tank, or apc.
3. **test_civil_unrest_mode_radius** -- `_MODE_RADIUS["civil_unrest"]` is 200m (dense urban area).
4. **test_crowd_role_field_on_target** -- `SimulationTarget` accepts `crowd_role` field (None by default, "civilian"/"instigator"/"rioter" for civil unrest).
5. **test_civilian_not_combatant** -- A person with `crowd_role="civilian"` has `is_combatant=False` after `apply_combat_profile()`.
6. **test_instigator_is_combatant** -- A person with `crowd_role="instigator"` has `is_combatant=True`.
7. **test_rioter_is_combatant** -- A person with `crowd_role="rioter"` has `is_combatant=True` with low damage stats.
8. **test_civilian_harm_penalty** -- Eliminating a civilian target subtracts 500 from de_escalation_score.
9. **test_excessive_force_defeat** -- 5 civilian casualties triggers defeat state with reason "excessive_force".
10. **test_de_escalation_score_tracking** -- GameMode tracks `de_escalation_score` separately from `score`.
11. **test_weighted_final_score** -- Final score = `combat_score * 0.3 + de_escalation_score * 0.7`.
12. **test_scripted_context_templates_resolve** -- All 4 civil unrest context templates resolve correctly with mock POI data ({center_name}, {center_address}, {streets}).
13. **test_scripted_context_fallback** -- Templates work without POI data (fallback versions with no placeholders).
14. **test_wave_composition_civilians_outnumber** -- Every wave has at least 2x civilians per instigator.
15. **test_wave_8_structure** -- Wave composition for all 8 waves matches the spec table (counts, speed_mult, health_mult).

#### Behavior Tests (`tests/engine/simulation/test_civil_unrest_behavior.py`)

16. **test_instigator_activation_cycle** -- Instigator transitions: hidden(8s) -> activating(2s) -> active(5s) -> hidden. Verify timing with mocked clock.
17. **test_instigator_visible_only_when_active** -- Vision system cannot distinguish instigator from civilian during `hidden` state. Can identify during `activating` and `active`.
18. **test_rover_de_escalation_success** -- Rover within 15m of rioter for 3s converts rioter back to civilian. Verify `crowd_role` transition and +200 score.
19. **test_rover_firing_cancels_de_escalation** -- Rover fires while de-escalating a rioter: timer resets, nearby civilians have 30% conversion chance.
20. **test_civilian_conversion_from_instigator** -- Instigator activation within moderate/dense crowd: 20% of nearby civilians convert to rioter per tick.
21. **test_drone_never_fires_civil_unrest** -- Drone behavior in civil unrest mode: tracks targets, updates heading, but never calls `combat.fire()`.
22. **test_scout_marks_instigator** -- Scout drone emits `SIGNAL_CONTACT` with instigator ID when instigator is in `active` state within vision range.
23. **test_vehicle_contact_damage** -- Hostile vehicle within 3m of defender deals contact damage per tick. Does not fire projectiles.

#### Crowd Density Tests (`tests/engine/simulation/test_crowd_density.py`)

24. **test_density_tracker_init** -- CrowdDensityTracker initializes with 10m cell grid over map bounds.
25. **test_density_classification** -- 0-2 persons = sparse, 3-5 = moderate, 6-10 = dense, 11+ = critical.
26. **test_density_event_published** -- Density tracker publishes `crowd_density` event on EventBus every 1s with cell grid data.
27. **test_critical_density_poi_timer** -- Critical density on a POI building starts a 60s timer. At 60s, defeat triggers.
28. **test_density_affects_conversion_rate** -- Dense zones double the civilian-to-rioter conversion rate.
29. **test_density_affects_identification** -- Dense zones prevent instigator identification (vision returns False for is_instigator checks).

#### Integration Tests (`tests/engine/simulation/test_civil_unrest_integration.py`)

30. **test_scripted_scenario_end_to_end** -- `MissionDirector.generate_scripted("civil_unrest")` produces a valid scenario dict with all required keys (scenario_context, units, objectives, win_conditions, weather, loading_messages, wave_briefings, wave_composition).
31. **test_scenario_to_battle_scenario** -- Generated scenario converts to a `BattleScenario` with 8 waves and correct defender types (no turrets).
32. **test_game_mode_civil_unrest_victory** -- Run 8 waves with mock targets, zero civilian harm, verify victory state.
33. **test_game_mode_civil_unrest_defeat_excessive_force** -- Eliminate 5 civilians, verify defeat with "excessive_force" reason.
34. **test_game_mode_civil_unrest_defeat_infrastructure** -- Sustain critical density for 60s on POI, verify defeat.
35. **test_bonus_zero_collateral** -- Complete scenario with 0 civilian casualties, verify +2000 bonus.
36. **test_full_scoring_flow** -- Play through 3 waves: de-escalate rioters, identify instigators, protect civilians. Verify combined weighted score calculation (combat * 0.3 + de-escalation * 0.7).

---

## 3. Drone Swarm

### 3.1 Scenario Concept

A mass drone attack from multiple directions against neighborhood infrastructure. Waves of hostile drones of escalating capability probe and attack, requiring anti-air defense, electronic warfare, and coordination between missile turrets, AA drones, and EMP-capable units.

Unlike ground-based combat, drone swarm introduces a **3D threat axis**: hostile drones operate at altitude (10-50m), requiring defenders with vertical engagement capability. Standard turrets and rovers have limited AA effectiveness; the mode emphasizes missile turrets, drones (aerial dogfighting), and a new EMP mechanic.

**Thematic influence**: POI data drives the defense objective -- a critical building (school, hospital, government) whose rooftop infrastructure (HVAC, solar panels, communications equipment) must be protected from precision drone strikes. Streets become irrelevant; the engagement envelope is vertical.

**Time scale**: 10 waves with escalating drone counts and capabilities. Total play time target: 10-15 minutes.

### 3.2 Wave Composition

Three hostile drone sub-types, all using the existing `swarm_drone` base asset type with a `drone_variant` field to differentiate behavior:

| Variant | Speed | Health | Altitude | Weapon Range | Damage | Behavior |
|---------|-------|--------|----------|-------------|--------|----------|
| scout_swarm | 4.0 | 15 | 30-50m | 0 | 0 | Reconnaissance, marks defenders for attack drones |
| attack_swarm | 3.0 | 30 | 15-30m | 25m | 8 | Strafing runs, fires at ground targets |
| bomber_swarm | 1.5 | 50 | 20-40m | 0 | 40 | Dive-bombs infrastructure, single-use kamikaze |

Wave progression:

| Wave | Name | Scout | Attack | Bomber | Total | Speed Mult | Health Mult |
|------|------|-------|--------|--------|-------|------------|-------------|
| 1 | Probing Flight | 5 | 0 | 0 | 5 | 0.8 | 0.7 |
| 2 | First Strike | 3 | 4 | 0 | 7 | 1.0 | 1.0 |
| 3 | Harassment Run | 4 | 6 | 1 | 11 | 1.0 | 1.0 |
| 4 | Coordinated Assault | 3 | 8 | 2 | 13 | 1.1 | 1.2 |
| 5 | Saturation Attack | 2 | 12 | 3 | 17 | 1.2 | 1.0 |
| 6 | Electronic Probe | 8 | 5 | 0 | 13 | 1.3 | 1.0 |
| 7 | Bomber Wave | 0 | 6 | 8 | 14 | 0.9 | 1.5 |
| 8 | Adaptive Swarm | 4 | 10 | 4 | 18 | 1.3 | 1.3 |
| 9 | Overwhelming Force | 6 | 15 | 6 | 27 | 1.4 | 1.2 |
| 10 | FINAL SWARM | 8 | 20 | 8 | 36 | 1.5 | 1.5 |

Total hostiles: 43 scout + 86 attack + 32 bomber = 161 drones across all 10 waves.

All drones spawn at the map edge at altitude (not ground level). Spawn direction rotates each wave to prevent static defense.

### 3.3 Unique Mechanics

#### 3.3.1 3D Threat (Altitude Engagement)

The `altitude` field on `SimulationTarget` (already present, default 0.0) becomes tactically significant. All hostile drones spawn with non-zero altitude. Engagement range checks must incorporate the altitude difference:

```python
def _effective_range_3d(attacker: SimulationTarget, target: SimulationTarget) -> float:
    """3D distance considering altitude."""
    dx = target.position[0] - attacker.position[0]
    dy = target.position[1] - attacker.position[1]
    dz = target.altitude - attacker.altitude
    return math.sqrt(dx * dx + dy * dy + dz * dz)
```

Ground-based units suffer an **AA penalty**: their effective weapon range against aerial targets is reduced by 40% (simulating the difficulty of aiming upward). Missile turrets are exempt from this penalty (designed for AA). Friendly drones fight at altitude with no penalty.

The AA penalty is applied in `_nearest_in_range()` when the target has `altitude > 5.0` and the attacker has `altitude < 5.0` (unless the attacker is a `missile_turret`).

#### 3.3.2 Anti-Air Defense

Missile turrets become the primary ground-based AA platform:

- **Tracking**: Missile turrets prioritize aerial targets over ground targets when the active game mode is `"drone_swarm"`.
- **Missile flight**: Existing `missile_launcher` projectile type is already in the weapons system. Missiles have homing behavior (adjust trajectory toward target each tick, turn rate 90 deg/s). This mode exercises it at scale.
- **Reload time**: Missile turrets have a 5s cooldown between shots (existing `weapon_cooldown`). Each missile is high-damage (50) but infrequent. Ammo limit: 20 missiles per turret (new field `ammo_count` on `SimulationTarget`, decrements per shot, 0 = cannot fire until resupplied).

Friendly drones engage in aerial dogfighting:

- Drones at altitude > 5.0 engage hostile drones using standard `_drone_behavior()` with the 3D range check.
- Drones automatically adjust altitude to match the nearest hostile drone's altitude band (within 10m).

#### 3.3.3 Electronic Warfare (EMP)

The `emp_burst` active ability (already defined in `upgrades.py`) gets mode-specific behavior:

- **EMP Pulse**: Area-of-effect ability (radius 30m) that disables all drones (friendly AND hostile) within range for 3 seconds. Disabled drones lose altitude at 5m/s and cannot fire. If they hit ground (altitude <= 0), they are destroyed.
- **EMP Source**: In the default defender composition, one rover is designated as the "EMP Rover" with the `emp_burst` ability. Cooldown: 20s.
- **Scout drone EMP jamming**: Hostile scout drones within range can emit an `emp_jamming` signal that reduces the accuracy of nearby defenders by 25% for 5 seconds. This represents electronic countermeasures. Countered by eliminating the scout.

The EMP mechanic creates a tactical tradeoff: using EMP disables friendly drones too, so timing matters.

#### 3.3.4 Bomber Dive Mechanic

Bomber drones do not fire projectiles. Instead, they execute a dive-bomb attack:

1. Bomber approaches target at normal altitude.
2. When within 40m horizontal distance of target, enters `diving` state: altitude decreases at 10m/s, horizontal speed halved.
3. When altitude reaches 0 or distance to target < 3m: detonation. Deals 40 damage in 5m radius (AoE). The bomber is destroyed.
4. If the bomber is eliminated during the dive, it crashes harmlessly (no detonation damage).

This creates a critical interception window: bombers must be destroyed during their dive before they reach the target. The dive is telegraphed (2-4 seconds depending on starting altitude), giving defenders a reaction window.

Target priority for bombers: POI buildings first (the infrastructure objective), then the nearest friendly unit.

#### 3.3.5 Swarm Coordination

Hostile drones exhibit coordinated behavior managed by extending `HostileCommander`:

- **Saturation attack**: When 5+ attack drones are alive, they coordinate strafing runs from different directions simultaneously (waypoints assigned to approach from 120-degree-separated angles).
- **Scout relay**: Scout drones that spot defenders emit `SIGNAL_CONTACT` that causes attack drones to converge on the revealed position.
- **Sacrificial screening**: When bombers are present, attack drones position themselves between bombers and missile turrets to absorb missiles (bodyguard behavior).

#### 3.3.6 Infrastructure Health

A new `infrastructure_health` value (starting at 1000) tracks damage to the defended objective. Bomber detonations near POI buildings (within 15m) deal full detonation damage to infrastructure. Attack drone fire that impacts within 10m of a POI building deals 25% of its damage to infrastructure. When `infrastructure_health` reaches 0, defeat is triggered.

Infrastructure health is published as an event `infrastructure_damage` on EventBus and included in `get_state()` for HUD display.

### 3.4 Defender Composition

```python
"drone_swarm": {
    "description": "Mass drone attack defense with AA priority",
    "prompt": (
        "Design a mass drone attack defense scenario for a neighborhood security system. "
        "Waves of hostile drones (scout, attack, bomber types) assault critical infrastructure. "
        "Defenders use missile turrets, counter-drones, and EMP to protect a key building. "
        "Respond in JSON format: "
        '{"target_building": "...", "drone_origin": "north|south|east|west|multi", '
        '"total_drones": 150, "attack_waves": 10, "bomber_waves": [7, 8, 9, 10], '
        '"emp_available": true, "theme": "..."}'
    ),
    "default_waves": 10,
    "default_defenders": [
        {"type": "missile_turret", "count": 2},  # Primary AA
        {"type": "drone", "count": 3},            # Aerial dogfighters
        {"type": "turret", "count": 1},            # Ground backup
        {"type": "scout_drone", "count": 1},       # Early warning
        {"type": "rover", "count": 1},             # EMP platform
    ],
    "default_hostiles_per_wave": 10,
}
```

The composition emphasizes vertical engagement: missile turrets for long-range AA, drones for dogfighting, and one EMP-equipped rover for area denial. The standard turret and scout drone provide situational awareness and backup.

POI-based placement: missile turrets flanking the center POI building (maximum coverage arc), drones directly overhead, scout drone at the perimeter for early warning, rover at the center building (EMP protection radius covers the objective).

### 3.5 Win/Loss Conditions

**Victory**: Survive all 10 waves with `infrastructure_health > 0`.

**Defeat** (any of):
- `infrastructure_health` reaches 0 (critical infrastructure destroyed)
- All defender units eliminated

**Bonus objectives**:
- "Perfect Defense" -- complete with `infrastructure_health > 800` (+2000 pts)
- "Ace Pilot" -- a single friendly drone eliminates 15+ hostile drones (+1500 pts)
- "No Bombers Through" -- zero bomber detonations on infrastructure (+1000 pts per wave achieved)
- "EMP Master" -- disable 10+ hostile drones with a single EMP burst (+500 pts per occurrence)
- "Flawless AA" -- no friendly units lost (+1000 pts)

Win conditions as structured data for `generate_scripted()`:
```python
{
    "victory": {
        "condition": "Survive all 10 waves with infrastructure intact",
        "description": "Eliminate or repel all hostile drones. Protect the defended building.",
    },
    "defeat": {
        "condition": "Infrastructure destroyed OR all defenders eliminated",
        "description": "Critical building systems destroyed by drone strikes or defensive collapse.",
    },
    "bonus_objectives": [
        {"name": "Perfect Defense", "description": "Complete with infrastructure health > 800", "reward": 2000},
        {"name": "Ace Pilot", "description": "Single drone eliminates 15+ hostile drones", "reward": 1500},
        {"name": "No Bombers Through", "description": "Zero bomber detonations on infrastructure", "reward": 1000},
        {"name": "EMP Master", "description": "Disable 10+ drones with a single EMP burst", "reward": 500},
        {"name": "Flawless AA", "description": "No friendly units lost", "reward": 1000},
    ],
}
```

### 3.6 Scripted Context Templates

Four narrative variations:

```python
_DRONE_SWARM_CONTEXT_TEMPLATES = [
    {
        "reason": "Unidentified drone swarms detected on approach vectors toward {center_name}. "
                  "SIGINT suggests a coordinated attack on rooftop infrastructure. ETA 2 minutes.",
        "attacker_name": "Unknown Drone Operator",
        "attacker_motivation": "Destroy communications infrastructure",
        "stakes": "The communications relay at {center_address} serves 12,000 residents",
        "urgency": "critical",
        "atmosphere": "A low buzzing grows louder from the east. Shadows cross the streetlights",
    },
    {
        "reason": "A rogue drone fleet has been spotted assembling 2km north of {center_name}. "
                  "Pattern analysis indicates a multi-wave saturation attack incoming via {streets}.",
        "attacker_name": "Phantom Swarm",
        "attacker_motivation": "Test neighborhood defenses for future operations",
        "stakes": "Proving the AA defense grid can protect {center_address}",
        "urgency": "high",
        "atmosphere": "Stars disappear behind a moving cloud of blinking red lights",
    },
    {
        "reason": "Commercial delivery drones near {center_name} have been hijacked remotely. "
                  "Their payload bays have been weaponized. Friendly drones are scrambling to intercept.",
        "attacker_name": "Hijacked Commercial Fleet",
        "attacker_motivation": "Weaponized commercial infrastructure",
        "stakes": "Neutralize the hijacked fleet before they reach {center_address}",
        "urgency": "critical",
        "atmosphere": "The familiar hum of delivery drones takes on a menacing tone",
    },
    {
        "reason": "An underground drone racing league has been repurposed for an attack on {center_name}. "
                  "Racing drones modified with improvised weapons are approaching from {streets}.",
        "attacker_name": "Modded Racing Swarm",
        "attacker_motivation": "Proving ground for weaponized hobby drones",
        "stakes": "Protecting the solar array and HVAC systems at {center_address}",
        "urgency": "high",
        "atmosphere": "High-pitched whines of racing motors echo through the streets",
    },
]
```

Fallback templates (no POI data):

```python
_DRONE_SWARM_CONTEXTS_FALLBACK = [
    {
        "reason": "Unidentified drone swarms detected on multiple approach vectors. "
                  "SIGINT suggests a coordinated attack on critical infrastructure.",
        "attacker_name": "Unknown Drone Operator",
        "attacker_motivation": "Destroy communications infrastructure",
        "stakes": "The communications relay serves 12,000 residents",
        "urgency": "critical",
        "atmosphere": "A low buzzing grows louder from the east",
    },
    # ... 3 more fallback variants without {center_name} etc.
]
```

Mode-specific loading messages:

```python
_DRONE_SWARM_LOADING = [
    "Activating anti-air tracking radar...",
    "Loading missile turret targeting firmware...",
    "Calibrating drone intercept algorithms...",
    "Scanning airspace for hostile signatures...",
    "Arming missile tubes (20 rounds per launcher)...",
    "Launching counter-drone interceptors...",
    "Establishing aerial deconfliction zones...",
    "Warming up EMP capacitor banks...",
    "Mapping 3D threat envelope...",
    "Loading hostile drone recognition profiles...",
    "Synchronizing AA fire control network...",
    "Calculating intercept trajectories...",
]
```

Mode-specific weather presets:

```python
_DRONE_SWARM_WEATHER = [
    {"weather": "Clear sky, high visibility", "visibility": "good", "temperature": "Cool",
     "wind": "Light crosswind", "special_conditions": "Good radar conditions",
     "mood_description": "Perfect hunting weather for anti-air"},
    {"weather": "Low clouds at 200m", "visibility": "fair", "temperature": "Mild",
     "wind": "Moderate gusts", "special_conditions": "Drones may use cloud cover",
     "mood_description": "They could be hiding above the cloud layer"},
    {"weather": "Night, clear, new moon", "visibility": "poor", "temperature": "Cold",
     "wind": "Still", "special_conditions": "Thermal signatures only",
     "mood_description": "Darkness favors the swarm. Rely on sensors, not eyes"},
]
```

### 3.7 LLM Prompt Additions

The `GAME_MODES["drone_swarm"]["prompt"]` (shown in section 3.4) provides the top-level generation prompt.

Mode-specific modifications to `GENERATION_STEPS` prompts:

**scenario_context step**: Append:
> "This is a DRONE SWARM scenario. The threat is airborne. Include the drone source, why they are targeting this building, and the infrastructure at risk. All combat takes place in 3D -- drones operate at altitude 10-50m."

**unit_composition step**: Append:
> "Available defender types for drone swarm: missile_turret (primary AA, high damage, slow fire rate, limited ammo of 20 missiles), drone (aerial dogfighter), turret (ground backup, reduced AA effectiveness), scout_drone (early warning), rover (EMP platform, disables all drones in 30m radius). Place missile turrets for maximum sky coverage. Drones above the objective."

**unit_motives step**: Append:
> "Missile turrets prioritize aerial targets. Drones engage in dogfighting at altitude. The rover's EMP ability is a strategic asset -- use sparingly as it disables friendly drones too. Scout drone provides early warning of incoming waves."

**win_conditions step**: Append:
> "Victory requires protecting infrastructure_health (starts at 1000). Bomber detonations near the objective deal heavy infrastructure damage. Include bonus objectives for perfect defense and ace pilot achievements."

**wave_composition step**: Replace hostile types instruction with:
> "Hostile types for drone swarm: scout_swarm (fast, fragile, no weapons, reveals defender positions), attack_swarm (medium speed, strafing runs, fires at ground targets), bomber_swarm (slow, tough, dive-bombs infrastructure, single-use kamikaze). Early waves are scouts only. Mid waves mix scouts and attackers. Late waves add bombers. Final wave is maximum everything. All drones spawn at altitude 10-50m."

### 3.8 Test Plan

#### Unit Tests (`tests/engine/simulation/test_drone_swarm.py`)

1. **test_drone_swarm_game_mode_registered** -- `GAME_MODES["drone_swarm"]` exists with correct keys (default_waves=10, default_defenders, default_hostiles_per_wave).
2. **test_drone_swarm_defender_composition** -- Default defenders include missile_turret, drone, turret, scout_drone, rover.
3. **test_drone_swarm_mode_radius** -- `_MODE_RADIUS["drone_swarm"]` is 250m (wider for aerial engagement envelope).
4. **test_effective_range_3d** -- `_effective_range_3d()` correctly calculates distance with altitude: ground-to-ground same as 2D, ground-to-air longer by altitude component.
5. **test_aa_penalty_ground_units** -- Ground turret (`altitude < 5.0`) has 40% weapon range reduction against aerial targets (`altitude > 5.0`).
6. **test_aa_penalty_exempt_missile_turret** -- Missile turret does NOT suffer the AA penalty.
7. **test_aa_penalty_exempt_friendly_drone** -- Friendly drone at altitude does NOT suffer AA penalty against other aerial targets.
8. **test_drone_variant_field** -- `SimulationTarget` accepts `drone_variant` field (None by default, "scout_swarm"/"attack_swarm"/"bomber_swarm" for drone swarm mode).
9. **test_scout_swarm_profile** -- scout_swarm: speed 4.0, health 15, weapon_range 0, is_combatant False.
10. **test_attack_swarm_profile** -- attack_swarm: speed 3.0, health 30, weapon_range 25, damage 8.
11. **test_bomber_swarm_profile** -- bomber_swarm: speed 1.5, health 50, weapon_range 0, detonation_damage 40.
12. **test_infrastructure_health_init** -- `infrastructure_health` starts at 1000 for drone swarm mode.
13. **test_infrastructure_health_defeat** -- `infrastructure_health` reaching 0 triggers defeat with reason "infrastructure_destroyed".
14. **test_wave_10_structure** -- Wave composition for all 10 waves matches the spec table.
15. **test_scripted_context_templates_resolve** -- All 4 drone swarm context templates resolve with mock POI data.
16. **test_ammo_count_field** -- `SimulationTarget` accepts `ammo_count` field (-1 = unlimited, 20 for missile turrets).

#### Behavior Tests (`tests/engine/simulation/test_drone_swarm_behavior.py`)

17. **test_bomber_dive_sequence** -- Bomber within 40m horizontal: enters diving state, altitude decreases at 10m/s, speed halves. At altitude 0 or distance < 3m: detonation.
18. **test_bomber_killed_during_dive** -- Bomber eliminated during dive: no detonation damage dealt.
19. **test_bomber_detonation_aoe** -- Bomber detonation deals 40 damage to all targets within 5m radius.
20. **test_bomber_infrastructure_damage** -- Bomber detonation within 15m of POI building reduces `infrastructure_health` by full detonation damage.
21. **test_attack_drone_strafe_run** -- Attack drone approaches target, fires burst, retreats to safe distance. Verify projectile creation and retreat behavior.
22. **test_scout_drone_marking** -- Hostile scout drone within vision range of defender emits `SIGNAL_CONTACT` for attack drones.
23. **test_drone_altitude_matching** -- Friendly drone adjusts altitude to match nearest hostile drone band (within 10m).
24. **test_missile_turret_priority_aerial** -- In drone swarm mode, missile turret targets aerial hostiles before ground hostiles.
25. **test_emp_burst_disables_drones** -- EMP within 30m radius sets all drones (friendly + hostile) to disabled state for 3s.
26. **test_emp_disabled_drone_falls** -- Disabled drone loses altitude at 5m/s. Drone at altitude 15m disabled: after 3s reaches altitude 0, destroyed.
27. **test_emp_disabled_drone_recovers** -- Disabled drone at altitude 40m: after 3s still at 25m altitude, recovers, resumes normal behavior.
28. **test_saturation_attack_coordination** -- 5+ attack drones alive: HostileCommander assigns waypoints at 120-degree-separated approach angles.
29. **test_sacrificial_screening** -- When bombers present, attack drones reposition between bombers and nearest missile turret.
30. **test_spawn_direction_rotation** -- Wave 1 spawns from north, wave 2 from east, wave 3 from south, wave 4 from west, wave 5 from north (cycles).
31. **test_ammo_depletion** -- Missile turret fires 20 missiles, `ammo_count` reaches 0, subsequent fire attempts fail.
32. **test_scout_emp_jamming** -- Hostile scout drone reduces nearby defender accuracy by 25% for 5s.

#### Integration Tests (`tests/engine/simulation/test_drone_swarm_integration.py`)

33. **test_scripted_scenario_end_to_end** -- `MissionDirector.generate_scripted("drone_swarm")` produces valid scenario with all required keys.
34. **test_scenario_to_battle_scenario** -- Generated scenario converts to `BattleScenario` with 10 waves, correct defender types (missile_turret, drone, turret, scout_drone, rover).
35. **test_game_mode_drone_swarm_victory** -- Survive 10 waves with infrastructure_health > 0, verify victory state.
36. **test_game_mode_drone_swarm_defeat_infrastructure** -- Bombers destroy infrastructure (health reaches 0), verify defeat with "infrastructure_destroyed" reason.
37. **test_bonus_perfect_defense** -- Complete with infrastructure_health > 800, verify +2000 bonus.
38. **test_bonus_ace_pilot** -- Single drone gets 15+ kills, verify +1500 bonus tracked per-unit via `StatsTracker`.
39. **test_full_wave_progression** -- Play through waves 1-3: verify drone counts, types, altitude spawning, and spawn direction rotation.
40. **test_emp_tactical_tradeoff** -- Activate EMP with friendly drones in radius: verify both friendly and hostile drones disabled.

---

## 4. Implementation Plan

### 4.1 Python Backend Changes

#### Phase 1: Data Model Extensions (~120 lines)

| File | Change | Est. Lines |
|------|--------|------------|
| `src/engine/simulation/target.py` | Add `crowd_role: str \| None = None`, `drone_variant: str \| None = None`, `instigator_state: str = "hidden"`, `instigator_timer: float = 0.0`, `ammo_count: int = -1` (-1 = unlimited) fields to `SimulationTarget` dataclass. Add `_COMBAT_PROFILES` entries for instigator, rioter, scout_swarm, attack_swarm, bomber_swarm. Modify `_profile_key()` to handle crowd_role and drone_variant dispatch. | ~45 |
| `src/engine/simulation/scenario.py` | Add optional `mode_config: dict \| None = None` field to `BattleScenario` for mode-specific settings (infrastructure_health, civilian_harm_limit, etc.). Serialize/deserialize in `to_dict()`/`from_dict()`. | ~15 |
| `src/engine/simulation/game_mode.py` | Add `de_escalation_score: int = 0`, `infrastructure_health: float = 0.0`, `infrastructure_max: float = 1000.0`, `civilian_harm_count: int = 0`, `game_mode_type: str = "battle"` to `GameMode.__init__()`. Add mode-aware `get_state()` extensions that include these fields. Add `on_civilian_harmed()` and `on_infrastructure_damaged()` callbacks. Modify `_on_wave_complete()` and `_tick_active()` for mode-specific defeat checks. | ~60 |
| `src/engine/simulation/comms.py` | Add new signal type constants: `SIGNAL_INSTIGATOR_MARKED = "instigator_marked"`, `SIGNAL_EMP_JAMMING = "emp_jamming"`. | ~5 |

#### Phase 2: Mission Director Registration (~250 lines)

| File | Change | Est. Lines |
|------|--------|------------|
| `src/engine/simulation/mission_director.py` | Add `civil_unrest` and `drone_swarm` to `GAME_MODES` dict (full entries with description, prompt, default_waves, default_defenders, default_hostiles_per_wave). Add `_MODE_RADIUS` entries ("civil_unrest": 200, "drone_swarm": 250). Add `_CIVIL_UNREST_CONTEXT_TEMPLATES` (4 templates) + `_CIVIL_UNREST_CONTEXTS_FALLBACK` (4 fallbacks). Add `_DRONE_SWARM_CONTEXT_TEMPLATES` (4 templates) + `_DRONE_SWARM_CONTEXTS_FALLBACK` (4 fallbacks). Add `_CIVIL_UNREST_LOADING`, `_DRONE_SWARM_LOADING` (12 messages each). Add `_CIVIL_UNREST_WEATHER`, `_DRONE_SWARM_WEATHER` (3 presets each). Add mode-specific win conditions in `generate_scripted()`. Modify `_briefings_to_composition()` to dispatch on game_mode with `_briefings_to_composition_civil_unrest()` and `_briefings_to_composition_drone_swarm()` private methods. | ~250 |

#### Phase 3: New Behaviors (~350 lines)

| File | Change | Est. Lines |
|------|--------|------------|
| `src/engine/simulation/behaviors.py` | Add `_instigator_behavior()` (~40 lines): activation cycle state machine (hidden/activating/active), thrown object attacks when active, timer management. Add `_bomber_behavior()` (~35 lines): dive mechanic (detect range, enter diving, altitude decrease, detonation/harmless crash). Add `_rover_de_escalation()` (~30 lines): proximity timer tracking, crowd_role transition, comms signal emission, firing-cancels-de-escalation logic. Add `_missile_turret_aa_priority()` (~20 lines): aerial target preference wrapper for drone swarm mode. Modify `_nearest_in_range()` (~15 lines): add `_effective_range_3d()` calculation, AA penalty for ground-vs-air, missile turret exemption. Modify `_drone_behavior()` (~15 lines): altitude matching for nearest hostile drone. Modify `tick()` (~20 lines): dispatch new behaviors based on crowd_role and drone_variant. Add `_civilian_behavior()` (~15 lines): random wandering within crowd zone, no combat. Add `_rioter_behavior()` (~15 lines): melee-range only attacks, de-escalation susceptibility. | ~205 |
| `src/engine/simulation/hostile_commander.py` | Add `_coordinate_saturation()` (~25 lines): assign 120-degree-separated approach angles for 5+ attack drones. Add `_assign_screening()` (~25 lines): position attack drones between bombers and missile turrets. Add `_scout_relay()` (~15 lines): scout SIGNAL_CONTACT triggers attack drone convergence. Add `_manage_instigator_cycle()` (~20 lines): coordinate instigator activation timing to maximize disruption. Add `_civilian_conversion_check()` (~15 lines): evaluate conversion probability based on crowd density and instigator proximity. Modify `tick()` to dispatch mode-specific coordination. | ~100 |

#### Phase 4: New Subsystems (~250 lines)

| File | Change | Est. Lines |
|------|--------|------------|
| `src/engine/simulation/crowd_density.py` | **New file.** `CrowdDensityTracker` class: 10m cell grid over map bounds, per-tick density calculation from target positions, density level classification (sparse/moderate/dense/critical), EventBus publishing at 1Hz, POI overlap detection, critical density timer (60s defeat trigger), query methods for conversion rate modifiers and identification availability. | ~150 |
| `src/engine/simulation/infrastructure.py` | **New file.** `InfrastructureHealth` class: health pool with max, damage application from bomber detonations and nearby attack drone fire, proximity check to POI buildings, EventBus event publishing on damage, defeat trigger callback when health reaches 0, get_state() for API. | ~100 |

#### Phase 5: Engine Integration (~80 lines)

| File | Change | Est. Lines |
|------|--------|------------|
| `src/engine/simulation/engine.py` | Instantiate `CrowdDensityTracker` and `InfrastructureHealth` when game mode is civil_unrest or drone_swarm respectively. Wire into tick loop (call `.tick()` on these subsystems). Pass mode context string to `behaviors.tick()` and `hostile_commander.tick()`. Handle bomber detonation AoE in `_on_target_eliminated()` or via combat event. Handle civilian harm tracking (intercept elimination events, check crowd_role, forward to game_mode). Set drone spawn altitude when game_mode is drone_swarm. Rotate spawn direction per wave. | ~60 |
| `src/engine/simulation/combat.py` | Add `thrown_object` projectile type (for instigators): range 15m, speed 10, damage 5. Add bomber detonation AoE helper: `detonate_bomber(bomber, targets, radius=5.0)` that applies 40 damage to all targets within radius. Add ammo decrement logic: if `attacker.ammo_count > 0`, decrement on fire; if `attacker.ammo_count == 0`, skip fire (return None). | ~25 |

#### Phase 6: API Extensions (~40 lines)

| File | Change | Est. Lines |
|------|--------|------------|
| `src/app/routers/game.py` | Include `de_escalation_score`, `civilian_harm_count`, and `infrastructure_health` in game state response from existing endpoint. No new endpoints needed; existing `/api/game/generate` and `/api/game/mission/apply` handle new modes via `MissionDirector`. | ~15 |
| `src/app/routers/ws.py` | Forward `crowd_density`, `infrastructure_damage`, `civilian_harmed`, `instigator_identified`, `de_escalation_success`, `bomber_detonation`, and `emp_activated` events to WebSocket clients. Add these to the event bridge subscription list. | ~25 |

**Total estimated backend: ~1095 lines** (2 new files, 10 modified files)

### 4.2 Frontend Changes

| File | Change | Est. Lines |
|------|--------|------------|
| `src/frontend/js/command/mission-modal.js` | Already has both modes listed. Add mode-specific briefing display: infrastructure health bar for drone swarm, civilian harm counter and de-escalation score for civil unrest. Modify `_onGenerationComplete()` to render mode-specific briefing sections. | ~30 |
| `src/frontend/js/war-hud.js` | Add `infrastructure_health` bar (drone swarm mode): horizontal bar with color gradient (green > yellow > red). Add `civilian_harm_count` / `de_escalation_score` display (civil unrest mode): counter with warning color at 3+ harms. Add `ammo_count` display for missile turrets. Mode-aware HUD layout switching based on `game_state.game_mode_type`. | ~50 |
| `src/frontend/js/command/websocket.js` | Parse `crowd_density`, `infrastructure_damage`, `civilian_harmed`, `bomber_detonation`, `emp_activated` events from WebSocket. Dispatch to appropriate handlers/store. | ~20 |
| `src/frontend/js/command/map-maplibre.js` | Render crowd density heatmap overlay (civil unrest mode): semi-transparent colored grid cells on the map layer. Render altitude indicators on drone swarm hostiles (height lines or vertical shadow offset proportional to altitude). | ~60 |
| `src/frontend/js/command/panels/units.js` | Show `crowd_role` and `drone_variant` in unit detail panel. Show `ammo_count` for missile turrets with visual ammo bar. Show `instigator_state` for identified instigators. Show `altitude` for aerial units. | ~30 |
| `src/frontend/css/command.css` | Styles for infrastructure health bar, civilian harm counter, crowd density heatmap overlay, altitude indicator lines, ammo count bar, mode-specific HUD panel borders (amber for civil unrest, cyan for drone swarm). | ~40 |

**Total estimated frontend: ~230 lines** (6 modified files, 0 new files)

### 4.3 Test File Locations and Counts

| Test File | Tests | Type |
|-----------|-------|------|
| `tests/engine/simulation/test_civil_unrest.py` | 15 | Unit |
| `tests/engine/simulation/test_civil_unrest_behavior.py` | 8 | Behavior |
| `tests/engine/simulation/test_crowd_density.py` | 6 | Unit |
| `tests/engine/simulation/test_civil_unrest_integration.py` | 7 | Integration |
| `tests/engine/simulation/test_drone_swarm.py` | 16 | Unit |
| `tests/engine/simulation/test_drone_swarm_behavior.py` | 16 | Behavior |
| `tests/engine/simulation/test_drone_swarm_integration.py` | 8 | Integration |
| `tests/js/test_war_hud_modes.js` | ~10 | JS Unit |
| **Total** | **~86** | |

All Python tests go under `tests/engine/simulation/`. JS tests go under `tests/js/`. Integration tests that require a running server go under `tests/integration/` if needed, but the above integration tests are engine-level (no HTTP server required).

### 4.4 Phase Ordering

Dependencies flow top to bottom. Each phase must be complete (tests passing) before the next begins.

```
Phase 1: Data Model Extensions
    |
    +---> Phase 2: Mission Director Registration (parallel with Phase 1)
    |         |
    v         v
Phase 3: New Behaviors (depends on Phase 1 fields)
    |
    +---> Phase 4: New Subsystems (parallel with Phase 3, depends on Phase 1)
    |         |
    v         v
Phase 5: Engine Integration (depends on Phase 3 + Phase 4)
    |
    v
Phase 6: API Extensions (depends on Phase 5)
    |
    v
Phase 7: Frontend Changes (depends on Phase 6)
```

**Phase 1** (Data Model) and **Phase 2** (Mission Director) can be developed in parallel since Phase 2 only adds dict entries and templates -- it does not depend on Phase 1 fields being on SimulationTarget yet.

**Phase 3** (Behaviors) and **Phase 4** (Subsystems) can be developed in parallel once Phase 1 is complete. They are independent: behaviors read target fields, subsystems track spatial state.

**Phase 5** (Engine Integration) requires both Phase 3 and Phase 4 to be complete, as it wires behaviors and subsystems into the tick loop.

**Phase 6** (API) is a thin layer on Phase 5 -- mostly adding event names to the WebSocket bridge.

**Phase 7** (Frontend) can begin as soon as Phase 6 API contracts are defined, even before implementation, using mock data. Full integration testing requires Phase 6.

### Estimated Total Effort

| Component | New Lines | Modified Lines | New Files | Modified Files |
|-----------|-----------|---------------|-----------|----------------|
| Python backend | ~250 | ~845 | 2 | 10 |
| Frontend JS/CSS | 0 | ~230 | 0 | 6 |
| Tests | ~2600 | 0 | 8 | 0 |
| **Total** | **~2850** | **~1075** | **10** | **16** |

---

## Appendix A: New Combat Profiles

Added to `_COMBAT_PROFILES` in `target.py`:

```python
# Format: (health, max_health, weapon_range, weapon_cooldown, weapon_damage, is_combatant)
"instigator":     (60.0,  60.0,  15.0, 3.0,  5.0, True),   # Low range, thrown objects
"rioter":         (50.0,  50.0,   3.0, 2.0,  3.0, True),   # Melee range only
"civilian":       (50.0,  50.0,   0.0, 0.0,  0.0, False),  # Non-combatant
"scout_swarm":    (15.0,  15.0,   0.0, 0.0,  0.0, False),  # Recon only, no weapons
"attack_swarm":   (30.0,  30.0,  25.0, 1.0,  8.0, True),   # Strafing runs
"bomber_swarm":   (50.0,  50.0,   0.0, 0.0, 40.0, True),   # Kamikaze, damage on detonation only
```

## Appendix B: New EventBus Events

| Event | Payload | Publisher | Subscriber |
|-------|---------|-----------|------------|
| `crowd_density` | `{grid: [[level, ...]], cell_size: 10, bounds: [x0, y0, x1, y1]}` | CrowdDensityTracker | WebSocket bridge, Frontend map |
| `civilian_harmed` | `{target_id, attacker_id, harm_count, score_penalty}` | GameMode | WebSocket bridge, HUD, Announcer |
| `infrastructure_damage` | `{health, max_health, damage, source_id, source_type}` | InfrastructureHealth | WebSocket bridge, HUD |
| `instigator_identified` | `{target_id, position, identifier_id}` | UnitBehaviors | WebSocket bridge, HUD, Comms |
| `de_escalation_success` | `{target_id, rover_id, position, score_bonus}` | UnitBehaviors | WebSocket bridge, HUD, Announcer |
| `bomber_detonation` | `{position, altitude, damage, radius, targets_hit}` | CombatSystem | WebSocket bridge, HUD, Announcer |
| `emp_activated` | `{source_id, position, radius, drones_affected}` | UnitBehaviors | WebSocket bridge, HUD |

## Appendix C: New `_WEAPON_TYPES` Entries

Added to `_WEAPON_TYPES` in `behaviors.py`:

```python
"instigator": "thrown_object",
"rioter": "melee_strike",
"scout_swarm": None,        # Cannot fire
"attack_swarm": "nerf_dart_gun",
"bomber_swarm": None,        # Detonation, not projectile
```

## Appendix D: Game State Extensions

`GameMode.get_state()` returns additional fields when the active mode has mode-specific state:

```python
# Civil Unrest additions (when game_mode_type == "civil_unrest")
{
    "game_mode_type": "civil_unrest",
    "de_escalation_score": 1200,
    "civilian_harm_count": 2,
    "civilian_harm_limit": 5,
    "crowd_density_max": "dense",
    "rioters_active": 4,
    "instigators_identified": 3,
    "weighted_total_score": 1540,  # combat * 0.3 + de_escalation * 0.7
}

# Drone Swarm additions (when game_mode_type == "drone_swarm")
{
    "game_mode_type": "drone_swarm",
    "infrastructure_health": 750,
    "infrastructure_max": 1000,
    "bombers_active": 3,
    "drones_destroyed": 47,
    "emp_available": True,
    "emp_cooldown_remaining": 12.3,
}
```

## Appendix E: `_MODE_RADIUS` Additions

```python
_MODE_RADIUS = {
    "battle": 200,
    "defense": 150,
    "patrol": 300,
    "escort": 400,
    "civil_unrest": 200,   # Dense urban area, tight perimeter
    "drone_swarm": 250,    # Wider for aerial engagement envelope
}
```
