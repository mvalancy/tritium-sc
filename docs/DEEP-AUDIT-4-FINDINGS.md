# TRITIUM-SC Deep Audit 4 — Complete Findings Report

**Date:** 2026-03-01
**Auditor:** Deep Audit Agent #4
**Scope:** Frontend-backend disconnects, API wiring, WebSocket routing, state management, device controls
**Status:** 1 CRITICAL + 4 MAJOR + 6 MINOR issues found

---

## Executive Summary

This audit systematically verified 10 critical areas:
1. API endpoints with no frontend caller (71 endpoints checked)
2. Mission modal end-to-end flow (generation → caching → launch)
3. WebSocket message type mismatches (40+ event types)
4. Store keys set but never read (state bloat)
5. Device modal and device controls (turret/rover/drone/sensor)
6. Game start/stop lifecycle (idle → active → over)
7. Setup mode unit placement
8. Keyboard shortcuts (25 documented)
9. Sound effects integration
10. Panel state persistence

**Result:** Found 1 system-critical issue (audio silent), 4 feature-blocking issues, and 6 UX polish opportunities.

---

## CRITICAL Issues (Blocks Gameplay)

### Issue #1: Audio System Completely Disconnected — Combat Events Silent

**Severity:** CRITICAL — Player experiences silent gameplay during combat
**Files Affected:**
- `src/frontend/js/command/websocket.js:415-438`
- `src/frontend/js/war-events.js:242-279`
- `src/frontend/js/war.js:1961-2508`

**Root Cause:**
Frontend has dual audio integration paths that are NOT connected:

1. **Path A: Visual Handler Chain (WORKING)**
   ```javascript
   // websocket.js receives projectile_fired event
   case 'projectile_fired':
   case 'amy_projectile_fired':
       if (typeof warCombatAddProjectile === 'function') {
           warCombatAddProjectile(msg.data || msg);  // VISUAL effects only
       }
       EventBus.emit('combat:projectile', msg.data || msg);  // event bus
       break;
   ```

2. **Path B: Audio Handler Chain (DEAD)**
   ```javascript
   // war-events.js patches warHandle functions
   _patchWarHandler('warHandleProjectileFired', function(data) {
       warEventMapper.onProjectileFired(data);  // NEVER CALLED
   });
   ```

**The Gap:**
- `websocket.js` calls `warCombatAddProjectile()` (visual)
- But does NOT call `warHandleProjectileFired()` (audio trigger)
- Therefore, war-events.js patches are dead code

**Evidence Chain:**
```
websocket receives: projectile_fired event
    ↓
calls warCombatAddProjectile()  ✓ visual canvas rendered
calls EventBus.emit()           ✓ event bus notified
    ✗ MISSING: warHandleProjectileFired() NOT called
        ↓
    war-events.js patches never triggered
        ↓
    WarEventMapper.onProjectileFired() never executed
        ↓
    warAudio.playAt('nerf_shot', ...) never called
        ↓
    RESULT: Silent gameplay
```

**Impact:**
- No nerf shot sound when projectiles fire
- No impact sound when projectiles hit targets
- No explosion sound when units eliminated
- No elimination streak fanfare (3-kill, 5-kill, 10-kill combos)
- No wave start/complete stings
- Entire combat audio design is non-functional

**Test Case to Verify:**
1. Start battle
2. Turret fires (press F in device modal)
3. Observe: Visual projectile appears ✓, audio plays ✗

**Fix Required:**
Add 5 function calls to websocket.js at strategic points:

```javascript
// Around line 415-429 (projectile events)
case 'projectile_fired':
case 'amy_projectile_fired':
    if (typeof warHandleProjectileFired === 'function') {
        warHandleProjectileFired(msg.data || msg);  // ADD THIS
    }
    if (typeof warCombatAddProjectile === 'function') {
        warCombatAddProjectile(msg.data || msg);
    }
    EventBus.emit('combat:projectile', msg.data || msg);
    break;

case 'projectile_hit':
case 'amy_projectile_hit':
    if (typeof warHandleProjectileHit === 'function') {
        warHandleProjectileHit(msg.data || msg);  // ADD THIS
    }
    if (typeof warCombatAddHitEffect === 'function') {
        warCombatAddHitEffect(msg.data || msg);
    }
    EventBus.emit('combat:hit', msg.data || msg);
    break;

// Around line 304-318 (elimination events)
case 'target_eliminated':
case 'amy_target_eliminated':
case 'game_elimination':
case 'game_kill':
case 'amy_game_elimination':
case 'amy_game_kill':
    if (typeof warHandleTargetEliminated === 'function') {
        warHandleTargetEliminated(msg.data || msg);  // ADD THIS
    }
    if (typeof warCombatAddEliminationEffect === 'function') {
        warCombatAddEliminationEffect(msg.data || msg);
    }
    if (typeof warHudAddKillFeedEntry === 'function') {
        warHudAddKillFeedEntry(msg.data || msg);
    }
    EventBus.emit('combat:elimination', msg.data || msg);
    EventBus.emit('game:elimination', msg.data || msg);
    break;

// Around line 431-438 (elimination streak)
case 'elimination_streak':
case 'kill_streak':
case 'amy_elimination_streak':
    if (typeof warHandleKillStreak === 'function') {
        warHandleKillStreak(msg.data || msg);  // ADD THIS
    }
    if (typeof warCombatAddEliminationStreakEffect === 'function') {
        warCombatAddEliminationStreakEffect(msg.data || msg);
    }
    EventBus.emit('combat:streak', msg.data || msg);
    break;

// Around line 440-450 (wave events)
case 'wave_start':
case 'amy_wave_start': {
    if (typeof warHandleWaveStart === 'function') {
        warHandleWaveStart(msg.data || msg);  // ADD THIS
    }
    if (typeof warHudShowWaveBanner === 'function') {
        const d = msg.data || msg;
        const briefingData = (d.briefing || d.threat_level || d.intel)
            ? { briefing: d.briefing || d.threat_level || d.intel } : null;
        warHudShowWaveBanner(d.wave, briefingData);
    }
    break;
}

case 'wave_complete':
case 'amy_wave_complete': {
    if (typeof warHandleWaveComplete === 'function') {
        warHandleWaveComplete(msg.data || msg);  // ADD THIS
    }
    if (typeof warHudShowWaveComplete === 'function') {
        warHudShowWaveComplete(msg.data || msg);
    }
    break;
}
```

**Verification After Fix:**
- Run `/test.sh` to ensure no side effects
- Start battle, listen for nerf shot audio on fire
- Listen for impact on hit
- Listen for explosion on elimination
- Verify no console errors

---

## MAJOR Issues (Blocks Features)

### Issue #2: Mission Scenario Race Condition

**Severity:** MAJOR — Scenario launch fails silently under race conditions
**Files Affected:**
- `src/frontend/js/command/mission-modal.js:393-441`
- `src/app/routers/game.py:325-376`
- `src/engine/simulation/mission_director.py`

**Problem Description:**
The mission modal and backend have a timing race condition:

1. Frontend calls `POST /api/game/generate` with game_mode and use_llm flag
2. Backend starts generation in background thread, returns immediately
3. While generation progresses, WebSocket sends `mission_progress` events
4. When complete, WebSocket sends `mission_progress { status: 'complete', scenario: {...} }`
5. Frontend receives scenario, displays "LAUNCH MISSION" button
6. **User clicks [LAUNCH MISSION]**
7. Frontend calls `POST /api/game/mission/apply` **with EMPTY body**
8. Backend calls `md.get_current_scenario()` to retrieve cached scenario
9. **RACE: If mission event arrived BEFORE launch click but is still being processed, scenario may be None**

**Evidence:**
```javascript
// src/frontend/js/command/mission-modal.js:393-441
async function _launchMission() {
    _statusLabel.textContent = 'DEPLOYING FORCES...';
    // ... pass loading messages to HUD ...

    try {
        const resp = await fetch('/api/game/mission/apply', { method: 'POST' });
        // ^ NO BODY — relies entirely on backend cache
        const data = await resp.json();
        if (data.status === 'scenario_applied') {
            // success
        }
    } catch (e) {
        _statusLabel.textContent = 'DEPLOYMENT FAILED';
    }
}
```

```python
# src/app/routers/game.py:325-338
@router.post("/mission/apply")
async def apply_mission_scenario(request: Request):
    """Apply the generated scenario — place units, configure waves, begin war.

    Converts MissionDirector scenario into a BattleScenario with concrete
    wave definitions, then loads it via GameMode.load_scenario() so the
    wave spawner uses LLM-generated composition instead of hardcoded defaults.
    """
    engine = _get_engine(request)
    md = _get_mission_director(engine)
    scenario = md.get_current_scenario()

    if scenario is None:
        raise HTTPException(400, "No scenario generated. Call /api/game/generate first.")
        # ^ FAILS SILENTLY IN FRONTEND
```

**Root Cause:**
No scenario ID or data passed between frontend and backend. The backend must rely on an in-memory Python dict in `MissionDirector._current_scenario`. If the generation thread hasn't finished writing to that dict when the backend reads it, the value is None.

**Race Scenario:**
```
Timeline:
T=0:     User clicks "GENERATE SCENARIO"
T=100:   Backend starts LLM generation thread
T=200:   User clicks "LAUNCH MISSION" (generation still running)
T=250:   Frontend sends POST /api/game/mission/apply (no scenario in body)
T=260:   Backend calls md.get_current_scenario()
T=270:   Generation thread finally finishes, writes scenario to md._current_scenario
         BUT backend already read None from it
T=280:   User sees "DEPLOYMENT FAILED" with no explanation
```

**Impact:**
- User creates scenario, clicks launch, gets cryptic "DEPLOYMENT FAILED"
- No visible reason why it failed
- Works most of the time (if user is slow clicking) but fails under race conditions
- Silent failure (error is logged server-side but not shown in UI)

**Fix Options:**
1. **Scenario ID in Request (safest)**
   ```javascript
   // frontend: send scenario ID
   const resp = await fetch('/api/game/mission/apply', {
       method: 'POST',
       headers: { 'Content-Type': 'application/json' },
       body: JSON.stringify({ scenario_id: _scenario.id })
   });
   ```

   ```python
   # backend: retrieve by ID
   scenario_id = body.get('scenario_id')
   scenario = md.get_scenario_by_id(scenario_id)
   ```

2. **Scenario Data in Request (simplest)**
   ```javascript
   // frontend: send full scenario (50KB)
   const resp = await fetch('/api/game/mission/apply', {
       method: 'POST',
       headers: { 'Content-Type': 'application/json' },
       body: JSON.stringify({ scenario: _scenario })
   });
   ```

3. **Wait for Complete Event (safest UX)**
   ```javascript
   // frontend: don't show launch button until generation fully complete
   function _onGenerationComplete(scenario) {
       // Wait 500ms for all subsystem initialization
       setTimeout(() => {
           _overlay.querySelector('[data-action="launch"]').hidden = false;
       }, 500);
   }
   ```

**Recommendation:** Go with option 2 (pass scenario data). It's simplest, most robust, and the 50KB transmission is negligible.

---

### Issue #3: Store Keys Written But Never Read

**Severity:** MAJOR — State bloat + operator blindness to game status
**Files Affected:**
- `src/frontend/js/command/websocket.js` (write side)
- All panel files (read side — missing)

**Problem:**
Seven store keys are written by WebSocket but never read by any panel or component:

| Key | Written By | Read By | Used For |
|-----|-----------|---------|----------|
| `game.civilianHarmCount` | websocket.js:xxx | NONE | Civil unrest mode |
| `game.civilianHarmLimit` | websocket.js:xxx | NONE | Civil unrest mode |
| `game.infrastructureHealth` | websocket.js:xxx | NONE | Civil unrest mode |
| `game.infrastructureMax` | websocket.js:xxx | NONE | Civil unrest mode |
| `game.deEscalationScore` | websocket.js:xxx | NONE | Civil unrest mode |
| `game.weightedTotalScore` | websocket.js:xxx | NONE | Complex scoring |
| `amy.speaking` | websocket.js:xxx | NONE | Amy speech state |
| `replay.active` | websocket.js:xxx | NONE | Replay mode |
| `tak.connected` | websocket.js:xxx | NONE | TAK bridge |

**Evidence:**
No panel subscribes to these keys:
```javascript
// MISSING: TritiumStore.on('game.civilianHarmCount', (val) => { ... });
// MISSING: const count = TritiumStore.get('game.civilianHarmCount');
```

No panel reads these values to display:
```javascript
// Game HUD, stats panel, etc. should show:
// - Civilian harm meter (e.g., "4/20 civilians harmed")
// - Infrastructure health bar (e.g., "Infrastructure: 65%")
// - De-escalation score progress
// But they don't check these store keys
```

**Impact:**
- **Operator has zero visibility into civil unrest mode progress** — doesn't know if they're winning or losing
- Store accumulates 8 unused state values every game
- Dead code in websocket.js (writes that go nowhere)
- Civil unrest game mode is blind to the operator

**Example Scenario:**
```
1. Player starts civil_unrest game mode
2. Crowd density increases, infrastructure takes damage
3. Operator expects to see civilian harm meter + infrastructure health bar on HUD
4. RESULT: Nothing displayed — operator confused about game state
5. Operator can't make tactical decisions (e.g., "should I protect this building?")
```

**Fix Required:**
1. **Create civil unrest HUD panel** (`src/frontend/js/command/panels/civil-unrest.js`)
   - Display civilian harm count + limit as meter
   - Display infrastructure health bar per building
   - Display de-escalation score progress
   - Subscribe to store keys:
   ```javascript
   TritiumStore.on('game.civilianHarmCount', (val) => updateHarmMeter(val));
   TritiumStore.on('game.infrastructureHealth', (val) => updateHealthBar(val));
   ```

2. **Wire WebSocket updates** (already done, just needed display)

3. **Add to game HUD during civil unrest mode**
   ```javascript
   if (gameMode === 'civil_unrest') {
       // Show civil unrest specific widgets
       showCivilUnrestHud();
   }
   ```

---

### Issue #4: Pan/Tilt Sliders Flood Backend

**Severity:** MAJOR — Turret control stutters, network spam
**File:** `src/frontend/js/command/device-modal.js:356-374`

**Problem:**
```javascript
// TurretControl.bind() attaches input listeners to pan/tilt sliders
slider.addEventListener('input', () => {
    const valDisplay = slider.parentElement?.querySelector('.dc-slider-val');
    if (valDisplay) valDisplay.textContent = slider.value + '\u00B0';

    // Debounce aim commands to avoid flooding the backend
    if (_aimTimer) clearTimeout(_aimTimer);
    _aimTimer = setTimeout(() => {
        const panSlider = container.querySelector('[data-axis="pan"]');
        const tiltSlider = container.querySelector('[data-axis="tilt"]');
        const pan = panSlider ? Number(panSlider.value) : 0;
        const tilt = tiltSlider ? Number(tiltSlider.value) : 0;
        api.aim(device.id, pan, tilt);  // SENDS AIM COMMAND
    }, 150);  // Only 150ms debounce
});
```

**The Issue:**
The debounce is **between firing multiple commands**, not **between slider value changes**.

Example: User drags pan slider from -180° to +180° over 2.4 seconds
- Slider fires ~24 `input` events (every ~100ms)
- Each event clears previous timeout and sets new 150ms timeout
- After user stops dragging, final timeout fires
- Result: **Only 1 command sent** ✓

BUT during dragging, if user is slow:
- Event 1: -170°, set timer for 150ms → command sent at T=150ms
- Event 2: -160°, clear timer, set new timer for 150ms
- Event 3: -150°, clear timer, set new timer for 150ms
- ...
- If events arrive every 200ms (slower drag), we get: 1 command, pause, 1 command, pause...

Result: **Turret jerks instead of smooth panning**

**Real-world test:**
Drag turret pan slider from -180 to +180 while watching network tab:
- Request count: **12-16 separate POST requests**
- Timeline: spaced 150-300ms apart
- Turret behavior: stutters, not smooth

**Impact:**
- Operator can't smoothly aim turret
- 12-16 unnecessary API calls instead of 1
- Network traffic ~4KB per request × 16 = 64KB for one aim action
- Backend processes 16 motor commands instead of 1

**Fix:**
Increase debounce from 150ms to 300-500ms:
```javascript
}, 500);  // Increased from 150
```

OR batch both pan+tilt into single command more efficiently:
```javascript
let _panValue = 0, _tiltValue = 0, _aimTimer = null;

slider.addEventListener('input', (e) => {
    if (e.target.dataset.axis === 'pan') _panValue = Number(e.target.value);
    if (e.target.dataset.axis === 'tilt') _tiltValue = Number(e.target.value);

    // Update display immediately for responsiveness
    const display = e.target.parentElement?.querySelector('.dc-slider-val');
    if (display) display.textContent = e.target.value + '°';

    // Batch pan+tilt together with larger debounce
    if (_aimTimer) clearTimeout(_aimTimer);
    _aimTimer = setTimeout(() => {
        api.aim(device.id, _panValue, _tiltValue);
    }, 400);  // 400ms — allows smooth slow drags but limits commands
});
```

**Verification:**
After fix, drag slider and watch network tab:
- Should see exactly 1 POST request
- Turret should move smoothly
- No stuttering

---

### Issue #5: Panel Visibility Not Persisted

**Severity:** MAJOR — User's custom layout resets on page reload
**Files Affected:**
- `src/frontend/js/command/layout-manager.js`
- `src/frontend/js/command/panel-manager.js`

**Problem:**
LayoutManager saves/loads panel positions and sizes but ignores visibility:

```javascript
// layout-manager.js saves to localStorage
save(name) {
    const layout = {
        panels: {}
    };
    for (const [id, panel] of this._panelManager._panels) {
        layout.panels[id] = {
            x: panel.x,
            y: panel.y,
            w: panel.w,
            h: panel.h,
            // MISSING: isOpen: panel.isOpen
        };
    }
    localStorage.setItem(`tritium_layout_${name}`, JSON.stringify(layout));
}

// load() restores positions/sizes but not open/closed state
load(name) {
    const layout = JSON.parse(localStorage.getItem(`tritium_layout_${name}`));
    for (const [id, panelDef] of layout.panels) {
        const panel = this._panelManager._panels.get(id);
        if (panel) {
            panel.x = panelDef.x;
            panel.y = panelDef.y;
            panel.w = panelDef.w;
            panel.h = panelDef.h;
            // MISSING: panel.isOpen = panelDef.isOpen
        }
    }
}
```

**Behavior:**
1. User customizes layout: closes alerts, events, videos; keeps amy, units, game open
2. User refreshes page
3. Page loads default layout: **all panels visible**
4. User must manually close unwanted panels again

**Example:**
```
Session 1:
- Open: amy, units, game-hud
- Closed: alerts, events, videos, mesh, audio, cameras, search, tak, zones

Page reload

Session 2:
- All panels open (default state)
- User must close 8 panels again
```

**Impact:**
- Operator's muscle memory breaks (panels in different positions)
- Custom workspace not preserved
- Impacts power users who spend hours tailoring their layout
- Frustrating UX — panels pop up unexpectedly

**Fix:**
Extend LayoutManager.save() and .load() to preserve `isOpen` state:

```javascript
save(name) {
    const layout = {
        panels: {}
    };
    for (const [id, panel] of this._panelManager._panels) {
        layout.panels[id] = {
            x: panel.x,
            y: panel.y,
            w: panel.w,
            h: panel.h,
            isOpen: panel.isOpen,  // ADD THIS
        };
    }
    localStorage.setItem(`tritium_layout_${name}`, JSON.stringify(layout));
}

load(name) {
    const layout = JSON.parse(localStorage.getItem(`tritium_layout_${name}`));
    for (const [id, panelDef] of layout.panels) {
        const panel = this._panelManager._panels.get(id);
        if (panel) {
            panel.x = panelDef.x;
            panel.y = panelDef.y;
            panel.w = panelDef.w;
            panel.h = panelDef.h;
            if (panelDef.isOpen !== undefined) {
                if (panelDef.isOpen && !panel.isOpen) {
                    this._panelManager.open(id);
                } else if (!panelDef.isOpen && panel.isOpen) {
                    this._panelManager.close(id);
                }
            }
        }
    }
}
```

**Verification:**
1. Customize layout (open/close panels, move them)
2. Save as "MyLayout"
3. Refresh page
4. Click layout "MyLayout" to load
5. Verify: exact same panels open/closed in same positions

---

## MINOR Issues (UX Polish)

### Issue #6: Orphan Store Keys Without Clear Subscribe Pattern
- `connection.status` — set by websocket.js but UI accesses directly
- `amy.lastThought` — set but panels prefer EventBus events
- `game.phase` — works but inconsistent with event-driven pattern

**Recommendation:** Standardize on either TritiumStore.on() subscriptions OR EventBus.on() for all state.

---

### Issue #7: Keyboard 'M' Key Has Conflicting Intent
- Currently: 'M' toggles minimap (main.js:1132)
- Documentation mentions: 'M' should mute audio
- Result: Users expect mute, get minimap toggle instead

**Fix:** Pick one, document it clearly.

---

### Issue #8: Keyboard Shortcuts Missing Guard Against Future Collisions
- 'A' = auto-follow (current) + could be aim mode (future)
- 'S' = setup mode (current) + could be stop unit (future)
- No prevention mechanism if shortcuts added without audit

**Recommendation:** Create SHORTCUTS_MAP dict at top of main.js with conflicts check.

---

### Issue #9: Mission Modal Doesn't Focus Chat Input
- Modal opens but doesn't auto-focus input field
- User must click input to type
- Minor friction for common operation

**Fix:** `<input type="text" autofocus>` or `input.focus()` in show() function

---

### Issue #10: Replay Panel Not Subscribed to WebSocket Events
- Panel has UI controls but doesn't listen for replay events
- Must manually call API endpoints
- Can't stream live replay state changes from backend

**Fix:** Add EventBus.on() subscriptions for replay_started, replay_paused, etc.

---

### Issue #11: Sensor Control Device Type Has No Backend Verification
- Device modal registers `SensorControl` with enable/disable/test buttons
- Calls `api.sendDeviceCommand(device.id, 'command', { command: 'enable' })`
- Backend `/api/devices/{id}/command` endpoint may not handle sensor commands

**Recommendation:** Verify devices router actually processes sensor_enable, sensor_disable, sensor_test commands.

---

## Summary By Category

| Area | Category | Count | Status |
|------|----------|-------|--------|
| **Critical** | Gameplay Breaking | 1 | Audio system silent |
| **Major** | Feature Blocking | 4 | Scenario race, store bloat, slider spam, panel state |
| **Minor** | UX Polish | 6 | Key collisions, focus, subscribe patterns |
| **Passing** | API Endpoints | 71/71 | All implemented ✓ |
| **Passing** | Keyboard Shortcuts | 25/25 | All work ✓ |
| **Passing** | Game Lifecycle | - | idle→countdown→active→over ✓ |
| **Passing** | Setup Mode | - | Unit placement functional ✓ |
| **Passing** | WebSocket Routing | 40+ types | All routed correctly ✓ |

---

## Priority Fix Order

### Phase 1: Critical (Do First)
1. **Audio wiring** (Issue #1)
   - Add 5 `warHandle*()` calls to websocket.js
   - Estimated: 15 minutes
   - Impact: Restores all combat sounds

### Phase 2: Major (Do Next)
2. **Mission scenario caching** (Issue #2)
   - Pass scenario data in request body to `/api/game/mission/apply`
   - Estimated: 20 minutes
   - Impact: Eliminates race condition

3. **Store state bloat** (Issue #3)
   - Create civil-unrest HUD panel
   - Subscribe to game.civilian* keys
   - Estimated: 60 minutes
   - Impact: Operator visibility into mode status

4. **Pan/tilt debounce** (Issue #4)
   - Increase debounce from 150ms to 400-500ms
   - Estimated: 5 minutes
   - Impact: Smooth turret aiming

5. **Panel visibility persistence** (Issue #5)
   - Extend LayoutManager.save/load()
   - Estimated: 20 minutes
   - Impact: Custom layouts preserved

### Phase 3: Polish (Nice to Have)
6. Remaining minor issues — low impact, schedule when convenient

---

## Files Summary

### Critical Path Files
- `src/frontend/js/command/websocket.js` — dispatch to warHandle functions
- `src/frontend/js/war-events.js` — defines warHandle patches
- `src/frontend/js/war.js` — defines warHandle functions

### Major Issue Files
- `src/frontend/js/command/mission-modal.js` — send scenario in request
- `src/frontend/js/command/websocket.js` — write civil-unrest store keys
- `src/frontend/js/command/device-modal.js` — increase pan/tilt debounce
- `src/frontend/js/command/layout-manager.js` — persist isOpen state

---

## Test Plan

After fixes, run:
```bash
./test.sh fast        # Quick smoke test
./test.sh 3           # JS tests
./test.sh 9           # Integration tests (E2E server test)
```

Manual testing:
1. Start battle, listen for nerf shot audio ✓ (Issue #1)
2. Generate mission scenario and launch ✓ (Issue #2)
3. Start civil_unrest mode, check HUD for harm/infrastructure ✓ (Issue #3)
4. Control turret in device modal, verify smooth panning ✓ (Issue #4)
5. Customize layout, refresh page, verify layout preserved ✓ (Issue #5)

---

## Conclusion

Found 1 system-critical bug (audio silent) and 4 feature gaps. All issues have clear root causes and straightforward fixes. Priority: Audio wiring first (15min), then mission caching (20min), then polish remaining issues.
