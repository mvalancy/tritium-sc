# TRITIUM-SC Command Center -- UI Features Reference

Comprehensive catalog of every interactive element in the Command Center UI.
This document covers buttons, menus, toggles, inputs, modals, and overlays
with their backend connection status.

**Source of truth**: This file is generated from the frontend source code.
Update it when UI elements change.

**Connection status key**:
- **WORKING** -- Backend endpoint exists and is fully wired. User action produces a real effect.
- **PARTIAL** -- Frontend sends the event/request, but backend handling is incomplete or stubbed.
- **BROKEN** -- The frontend calls an endpoint that does not exist. Fails silently.
- **COSMETIC** -- UI-only. No backend communication needed (local state, CSS toggle, etc.).


---

## 1. Menu Bar

Desktop-style dropdown menu bar across the top of the viewport. Six menus on
the left side, quick-access panel toggle buttons on the right side.

Source: `src/frontend/js/command/menu-bar.js`

### 1.1 FILE Menu

| Item | Shortcut | What It Does | Status | Backend |
|------|----------|-------------|--------|---------|
| Save Layout... | `Ctrl+Shift+S` | Opens inline text input to name and save current panel layout to localStorage | COSMETIC | -- |
| Export Layout JSON | -- | Downloads the current layout as a `.json` file | COSMETIC | -- |
| Import Layout JSON | -- | Opens file picker, loads a `.json` layout file, applies it | COSMETIC | -- |

### 1.2 VIEW Menu

| Item | Shortcut | What It Does | Status | Backend |
|------|----------|-------------|--------|---------|
| Amy | `1` | Toggle Amy panel visibility | COSMETIC | -- |
| Units | `2` | Toggle Units panel visibility | COSMETIC | -- |
| Alerts | `3` | Toggle Alerts panel visibility | COSMETIC | -- |
| Game Status | `4` | Toggle Game HUD panel visibility | COSMETIC | -- |
| *(all registered panels)* | -- | Each registered panel appears as a checkable toggle | COSMETIC | -- |
| Show All | -- | Opens all registered panels | COSMETIC | -- |
| Hide All | -- | Closes all registered panels | COSMETIC | -- |
| Fullscreen | `F11` | Toggles browser fullscreen mode | COSMETIC | -- |

### 1.3 LAYOUT Menu

| Item | Shortcut | What It Does | Status | Backend |
|------|----------|-------------|--------|---------|
| Commander | -- | Apply built-in "commander" layout | COSMETIC | -- |
| Observer | -- | Apply built-in "observer" layout | COSMETIC | -- |
| Tactical | -- | Apply built-in "tactical" layout | COSMETIC | -- |
| Battle | -- | Apply built-in "battle" layout | COSMETIC | -- |
| *(user layouts)* | -- | Apply a user-saved layout. Each has a delete (x) button. | COSMETIC | -- |
| Save Current... | -- | Opens save input (same as FILE > Save Layout) | COSMETIC | -- |

### 1.4 MAP Menu

| Item | Shortcut | What It Does | Status | Backend |
|------|----------|-------------|--------|---------|
| Toggle All | -- | Toggles all map layers on/off | COSMETIC | -- |
| **Base Map Layers** | | | | |
| Satellite | `I` | Toggle satellite imagery layer | COSMETIC | -- |
| Roads | `G` | Toggle road overlay | COSMETIC | -- |
| Buildings | `K` | Toggle building outlines | COSMETIC | -- |
| Waterways | -- | Toggle waterway features | COSMETIC | -- |
| Parks | -- | Toggle park/green space features | COSMETIC | -- |
| Grid | -- | Toggle coordinate grid | COSMETIC | -- |
| **Unit Layers** | | | | |
| Unit Markers | `U` | Toggle unit icon rendering on the map | COSMETIC | -- |
| 3D Models | -- | Toggle 3D model rendering | COSMETIC | -- |
| Labels | -- | Toggle unit name labels | COSMETIC | -- |
| Mesh Network | -- | Toggle mesh radio link lines | COSMETIC | -- |
| Thought Bubbles | -- | Toggle NPC thought bubble display | COSMETIC | -- |
| **Combat FX Layers** | | | | |
| Tracers | -- | Toggle projectile tracer rendering | COSMETIC | -- |
| Explosions | -- | Toggle explosion effect rendering | COSMETIC | -- |
| Particles | -- | Toggle particle system rendering | COSMETIC | -- |
| Hit Flashes | -- | Toggle hit flash effects | COSMETIC | -- |
| Floating Text | -- | Toggle floating damage/kill text | COSMETIC | -- |
| **Overlay Layers** | | | | |
| Kill Feed | -- | Toggle kill feed overlay | COSMETIC | -- |
| Screen FX | -- | Toggle full-screen combat effects | COSMETIC | -- |
| Banners | -- | Toggle wave/streak banner display | COSMETIC | -- |
| Layer HUD | -- | Toggle layer state HUD indicator | COSMETIC | -- |
| **Unit Decorations** | | | | |
| Health Bars | -- | Toggle health bars above units | COSMETIC | -- |
| Selection FX | -- | Toggle selection highlight ring | COSMETIC | -- |
| **Environment** | | | | |
| Fog | `V` | Toggle fog of war | COSMETIC | -- |
| Terrain | `H` | Toggle terrain overlay | COSMETIC | -- |
| 3D Mode | -- | Toggle tilted/flat camera perspective | COSMETIC | -- |
| **Camera Controls** | | | | |
| Center on Action | `F` | Fly camera to active combat area | COSMETIC | -- |
| Reset Camera | `R` | Reset camera to default position and zoom | COSMETIC | -- |
| Zoom In | `]` | Increase zoom level | COSMETIC | -- |
| Zoom Out | `[` | Decrease zoom level | COSMETIC | -- |

### 1.5 GAME Menu

| Item | Shortcut | What It Does | Status | Backend |
|------|----------|-------------|--------|---------|
| New Mission | `B` | Opens the Mission Modal (same as pressing B) | WORKING | Opens modal, which calls `POST /api/game/generate` |
| Reset Game | `R` | Resets the current game to idle state | WORKING | `POST /api/game/reset` |

### 1.6 HELP Menu

| Item | Shortcut | What It Does | Status | Backend |
|------|----------|-------------|--------|---------|
| Keyboard Shortcuts | `?` | Toggles the help overlay showing all keyboard shortcuts | COSMETIC | -- |
| About TRITIUM-SC | -- | Shows a toast with version info | COSMETIC | -- |

### 1.7 Right-Side Quick-Access Buttons

One button per registered panel. Each button toggles its panel visibility.
The button highlights (active class) when its panel is open.
Panel toggle buttons for core panels use number key shortcuts: AMY (1),
UNITS (2), ALERTS (3), GAME (4), MESH (5).

Status: **COSMETIC** -- all local panel state management.


---

## 2. Map Mode Buttons

Three mode buttons in the top-left corner of the tactical map area.

Source: `src/frontend/unified.html`, `src/frontend/js/command/main.js`

| Button | Shortcut | What It Does | Status | Backend |
|--------|----------|-------------|--------|---------|
| OBSERVE | `O` | Observation mode -- passive monitoring, no placement | COSMETIC | -- |
| TACTICAL | `T` | Tactical mode -- combat overlays, threat indicators | COSMETIC | -- |
| SETUP | `S` | Setup mode -- enables unit placement via click-to-place. Auto-opens Game HUD panel | COSMETIC | -- |

Pressing ESC while in Setup mode switches back to Observe mode.


---

## 3. Device Control Modals

Opened by clicking a unit on the map or from the Unit Inspector. Each
device type gets a type-specific control panel with relevant buttons.

Source: `src/frontend/js/command/device-modal.js`

### 3.1 DeviceAPI (shared backend calls)

| Method | Endpoint | Status |
|--------|----------|--------|
| `dispatch(unitId, x, y)` | `POST /api/amy/simulation/dispatch` | **WORKING** |
| `recall(unitId)` | `POST /api/amy/command` (Lua: `recall()`) | **WORKING** |
| `patrol(unitId, waypoints)` | `POST /api/amy/command` (Lua: `patrol(...)`) | **WORKING** |
| `fire(unitId)` | `POST /api/amy/command` (Lua: `fire_nerf()`) | **WORKING** |
| `aim(unitId, pan, tilt)` | `POST /api/amy/command` (Lua: `motor.aim(pan,tilt)`) | **WORKING** |
| `stop(unitId)` | `POST /api/amy/command` (Lua: `stop()`) | **WORKING** |
| `sendCommand(unitId, lua)` | `POST /api/amy/command` | **WORKING** |
| `sendDeviceCommand(deviceId, topic, payload)` | `POST /api/devices/{id}/command` | **WORKING** | Routes to MQTT, Lua fallback, or accepted |

### 3.2 RoverControl (type: rover, tank, apc)

Title: "ROBOT CONTROL"

| Button | What It Does | Status | Notes |
|--------|-------------|--------|-------|
| DISPATCH | Emits `unit:dispatch-mode` event. Map enters click-to-dispatch mode. | **WORKING** | Map listens for event, next click dispatches via API |
| PATROL | Emits `unit:patrol-mode` event. Map enters click-to-set-waypoints mode. | **WORKING** | Map listens for event |
| RECALL | Sends `recall()` Lua command via `POST /api/amy/command` | **WORKING** | |
| STOP | Sends `stop()` Lua command via `POST /api/amy/command` | **WORKING** | |
| FIRE | Sends `fire_nerf()` Lua command via `POST /api/amy/command` | **WORKING** | |
| AIM | Emits `unit:aim-mode` event. Map enters click-to-aim mode. | **WORKING** | Map listens for event |
| SEND (Lua input) | Sends arbitrary Lua command string via `POST /api/amy/command` | **WORKING** | Enter key also triggers send |

Display fields: NAME, TYPE, STATUS, POSITION, HEADING, SPEED, BATTERY, HEALTH (with color bar).

### 3.3 DroneControl (type: drone, scout_drone, swarm_drone)

Title: "DRONE CONTROL"

| Button | What It Does | Status | Notes |
|--------|-------------|--------|-------|
| DISPATCH | Same as RoverControl DISPATCH | **WORKING** | |
| PATROL | Same as RoverControl PATROL | **WORKING** | |
| RECALL | Same as RoverControl RECALL | **WORKING** | |
| STOP | Same as RoverControl STOP | **WORKING** | |
| FIRE | Same as RoverControl FIRE | **WORKING** | |
| SEND (Lua input) | Same as RoverControl SEND | **WORKING** | |

Display fields: NAME, TYPE, STATUS, ALTITUDE, POSITION, BATTERY, HEALTH (with color bar).

Note: Uses RoverControl.bind, so all button wiring is identical to rover.

### 3.4 TurretControl (type: turret, heavy_turret, missile_turret)

Title: "TURRET CONTROL"

| Button | What It Does | Status | Notes |
|--------|-------------|--------|-------|
| FIRE | Sends `fire_nerf()` Lua command | **WORKING** | |
| STOP | Sends `stop()` Lua command | **WORKING** | |
| AUTO TARGET | Sends `auto_target()` Lua command | **WORKING** | Recently fixed |
| MANUAL | Sends `manual_target()` Lua command | **WORKING** | Recently fixed |
| SEND (Lua input) | Sends arbitrary Lua command | **WORKING** | |
| PAN slider | Range input -180 to 180 degrees | **WORKING** | Sends debounced `motor.aim(pan,tilt)` command on change |
| TILT slider | Range input -30 to 90 degrees | **WORKING** | Sends debounced `motor.aim(pan,tilt)` command on change |

Display fields: NAME, TYPE, STATUS, WEAPON RANGE, HEALTH (with color bar).

### 3.5 SensorControl (type: sensor, pir, microwave, acoustic, tripwire)

Title: "SENSOR CONTROL"

| Button | What It Does | Status | Notes |
|--------|-------------|--------|-------|
| ENABLE | Calls `DeviceAPI.sendDeviceCommand(id, 'command', {command:'enable'})` | **WORKING** | Routes to MQTT bridge, Lua fallback |
| DISABLE | Calls `DeviceAPI.sendDeviceCommand(id, 'command', {command:'disable'})` | **WORKING** | Routes to MQTT bridge, Lua fallback |
| TEST TRIGGER | Calls `DeviceAPI.sendDeviceCommand(id, 'command', {command:'test_trigger'})` | **WORKING** | Routes to MQTT bridge, Lua fallback |

Display fields: NAME, TYPE, STATUS, POSITION.

### 3.6 NPCControl (type: person, animal, vehicle when neutral)

Title: "NPC INTEL"

| Button / Input | What It Does | Status | Notes |
|----------------|-------------|--------|-------|
| TAKE CONTROL | `POST /api/npc/{id}/control` with `{controller_id:'operator'}` | **WORKING** | Sets `controlledUnitId` in store, enables WASD + click-to-dispatch, shows map ring indicator |
| RELEASE | `DELETE /api/npc/{id}/control` | **WORKING** | Clears `controlledUnitId`, releases WASD controls, ESC also releases |
| SET (thought) | `POST /api/npc/{id}/thought` with text, emotion, duration | **WORKING** | Endpoint exists in npc.py |
| Emotion dropdown | Select emotion: neutral, curious, afraid, angry, happy | **COSMETIC** | Used by SET thought |
| Thought text input | Free text input for thought content | **COSMETIC** | Used by SET thought |

On mount, fetches `GET /api/npc/{id}` to populate personality bars (curiosity,
caution, sociability, aggression) and brain state (danger, interest, bound).

Display fields: NAME, TYPE, ALLIANCE, FSM STATE, POSITION, SPEED, HEADING,
HEALTH, CURRENT THOUGHT, THOUGHT HISTORY, PERSONALITY, BRAIN STATE.

Note: "person" type now correctly resolves to NPC controls (not Sensor controls).
The alias `'person': 'npc'` was recently added to DeviceControlRegistry.

### 3.7 CameraControl (type: camera, ip_camera, ptz_camera, synthetic_camera)

Title: "CAMERA CONTROL"

| Button | What It Does | Status | Notes |
|--------|-------------|--------|-------|
| STREAM | `<img>` tag loads MJPEG from `/api/amy/nodes/{id}/video` | **WORKING** | Endpoint exists in amy/router.py |
| LEFT | Sends `motor.aim(-10,0)` Lua command | **WORKING** | Via `POST /api/amy/command` |
| UP | Sends `motor.aim(0,10)` Lua command | **WORKING** | Via `POST /api/amy/command` |
| DOWN | Sends `motor.aim(0,-10)` Lua command | **WORKING** | Via `POST /api/amy/command` |
| RIGHT | Sends `motor.aim(10,0)` Lua command | **WORKING** | Via `POST /api/amy/command` |
| SNAPSHOT | Opens `/api/amy/nodes/{id}/snapshot` in a new tab | **PARTIAL** | Endpoint path not confirmed in router |
| OFF | Calls `DeviceAPI.sendDeviceCommand(id, 'command', {command:'camera_off'})` | **WORKING** | Routes to MQTT bridge, Lua fallback |

PTZ buttons only render if `device.hasPtz === true`.

### 3.8 MeshRadioControl (type: mesh_radio, meshtastic, meshcore)

Title: "MESH RADIO"

| Button / Input | What It Does | Status | Notes |
|----------------|-------------|--------|-------|
| SEND (text) | Calls `DeviceAPI.sendDeviceCommand(id, 'text', {text:...})` | **WORKING** | Routes to meshtastic bridge |
| CENTER ON MAP | Emits `mesh:center-on-node` event | **COSMETIC** | Map pans to node position |
| Text input | Free text message input (228 char max) | **COSMETIC** | Used by SEND |

Display fields: NAME, PROTOCOL, BATTERY, SNR, RSSI, HOPS.

### 3.9 GenericControl (fallback for unknown types)

Title: "DEVICE"

| Button | What It Does | Status | Notes |
|--------|-------------|--------|-------|
| SEND (Lua input) | Sends arbitrary Lua command via `POST /api/amy/command` | **WORKING** | |

Display fields: NAME, TYPE, STATUS, POSITION.

### 3.10 Modal Chrome

| Element | What It Does | Status |
|---------|-------------|--------|
| [ESC] close button | Closes the device modal | COSMETIC |
| Click overlay backdrop | Closes the device modal | COSMETIC |
| ESC key | Closes the device modal | COSMETIC |

### 3.11 Device Type Alias Map

The DeviceControlRegistry resolves these aliases to their parent control type:

| Unit Type | Resolves To | Control Panel |
|-----------|-------------|---------------|
| scout_drone | drone | DroneControl |
| swarm_drone | drone | DroneControl |
| heavy_turret | turret | TurretControl |
| missile_turret | turret | TurretControl |
| tank | rover | RoverControl |
| apc | rover | RoverControl |
| person | npc | NPCControl |
| animal | npc | NPCControl |
| vehicle | npc | NPCControl |
| pir | sensor | SensorControl |
| microwave | sensor | SensorControl |
| acoustic | sensor | SensorControl |
| tripwire | sensor | SensorControl |
| meshtastic | mesh_radio | MeshRadioControl |
| meshcore | mesh_radio | MeshRadioControl |
| ip_camera | camera | CameraControl |
| ptz_camera | camera | CameraControl |
| synthetic_camera | camera | CameraControl |


---

## 4. Units Panel

Filterable list of all units on the tactical map with a detail view for
the selected unit.

Source: `src/frontend/js/command/panels/units.js`

### 4.1 Filter Dropdown

| Option | What It Does | Status |
|--------|-------------|--------|
| ALL | Show all units | COSMETIC |
| FRIENDLY | Show only friendly alliance units | COSMETIC |
| HOSTILE | Show only hostile alliance units | COSMETIC |
| NEUTRAL | Show only neutral alliance units | COSMETIC |
| UNKNOWN | Show only unknown alliance units | COSMETIC |
| SOURCE: REAL | Show only units from real hardware | COSMETIC |
| SOURCE: SIM | Show only simulated units | COSMETIC |
| SOURCE: GRAPHLING | Show only Graphling AI agent units | COSMETIC |

### 4.2 Unit List

| Interaction | What It Does | Status | Notes |
|-------------|-------------|--------|-------|
| Click unit row | Selects the unit, sets `map.selectedUnitId` in store, emits `unit:selected` event. Opens detail view below. | COSMETIC | |

Each row shows: type icon badge, short ID, name, status badges (JAM/EMPTY/LOW),
source badge (SIM/REAL/GRAPHLING), FSM state badge, health.

### 4.3 Unit Detail View (selected unit)

Displayed below the unit list when a unit is selected. Shows comprehensive
stats and controls.

**Display fields**: TYPE, ALLIANCE, SOURCE, FSM STATE, HEALTH (with bar),
BATTERY, MORALE, DEGRADATION, WEAPON RANGE, WEAPON STATUS (JAMMED),
AMMO STATUS, KILLS, COMBAT STATS (shots/damage/distance/time/assists),
HEADING, POSITION, SQUAD, CROWD ROLE, INSTIGATOR STATE, DRONE TYPE,
ALTITUDE, AMMO (with bar), THOUGHT, IDENTITY (person/device/vehicle/robot details),
BACKSTORY, INVENTORY.

| Interactive Element | What It Does | Status | Backend |
|---------------------|-------------|--------|---------|
| Ability buttons | Fetch abilities from `GET /api/game/unit/{id}/upgrades`, render ability buttons. Clicking activates via `POST /api/game/ability` | **WORKING** | Both endpoints exist in game.py |
| SEND (Lua command) | Sends arbitrary Lua command to selected unit via `POST /api/amy/command` | **WORKING** | Endpoint exists |
| Lua command input | Text input for Lua command, Enter key triggers send | **WORKING** | |
| VIEW NPC DETAILS | Opens device modal for neutral person/animal/vehicle units. Emits `device:open-modal` event | **WORKING** | NPC API exists |

Ability buttons are only shown for friendly-alliance units. The ability bar
fetches both `GET /api/game/unit/{id}/upgrades` (for granted abilities and cooldowns)
and `GET /api/game/abilities` (for ability definitions/names).


---

## 5. Unit Inspector Panel

Inline inspector panel with navigation between units and integrated
device controls. Replaces the blocking device modal with a persistent
side panel.

Source: `src/frontend/js/command/panels/unit-inspector.js`

### 5.1 Navigation Bar

| Element | What It Does | Status |
|---------|-------------|--------|
| `<<` (prev) button | Navigate to previous unit in filtered list (wraps around) | COSMETIC |
| `>>` (next) button | Navigate to next unit in filtered list (wraps around) | COSMETIC |
| Nav label | Shows "N / M" (current index / total filtered) | COSMETIC |

### 5.2 Filter Row

| Element | What It Does | Status |
|---------|-------------|--------|
| Search input | Free-text filter by unit name or ID (case-insensitive) | COSMETIC |
| Type filter dropdown | Filter by unit type (ALL, turret, heavy_turret, missile_turret, rover, drone, scout_drone, tank, apc, sensor, camera, person) | COSMETIC |
| Alliance filter dropdown | Filter by alliance (ALL, friendly, hostile, neutral) | COSMETIC |

### 5.3 Content Area

Renders the appropriate DeviceControl panel (from DeviceControlRegistry) for
the currently selected unit. Resolves neutral person/animal/vehicle to NPC
controls automatically. Includes all buttons and inputs from the relevant
device control (see Section 3).

Also appends a COMBAT STATS section (shots, damage, distance, time, assists)
from the unit's store data.


---

## 6. Replay Panel

VCR-style battle replay with timeline scrubber, transport controls, speed
selector, wave jump, and event log.

Source: `src/frontend/js/command/panels/replay.js`

### 6.1 Status Bar

| Element | What It Does | Status |
|---------|-------------|--------|
| Mode badge | Shows "LIVE" or "REPLAY" depending on mode | COSMETIC |
| Time display | Shows current position and total duration (M:SS / M:SS) | COSMETIC |

### 6.2 Timeline Bar

| Interaction | What It Does | Status | Backend |
|-------------|-------------|--------|---------|
| Click on timeline bar | Seeks to clicked position. `POST /api/game/replay/seek` | **WORKING** | Endpoint exists |
| Timeline fill | Visual progress indicator | COSMETIC | |
| Playhead | Current position marker | COSMETIC | |
| Wave markers | Vertical markers at wave start times | COSMETIC | |

### 6.3 Transport Controls

| Button | Label | What It Does | Status | Backend |
|--------|-------|-------------|--------|---------|
| Rewind | `\|<<` | Seek to start (time 0), stop playback | **WORKING** | `POST /api/game/replay/stop` + `POST /api/game/replay/seek` |
| Step Back | `<<` | Step one frame backward | **WORKING** | `POST /api/game/replay/step-backward` |
| Play/Pause | `PLAY` / `PAUSE` | Toggle playback. Enters replay mode on first press. | **WORKING** | `POST /api/game/replay/play` or `POST /api/game/replay/pause` |
| Step Forward | `>>` | Step one frame forward | **WORKING** | `POST /api/game/replay/step-forward` |
| Jump End | `>>\|` | Seek to end of replay | **WORKING** | `POST /api/game/replay/seek` |

### 6.4 Speed Controls

| Button | What It Does | Status | Backend |
|--------|-------------|--------|---------|
| 0.25x | Set playback speed to 0.25x | **WORKING** | `POST /api/game/replay/speed` |
| 0.5x | Set playback speed to 0.5x | **WORKING** | `POST /api/game/replay/speed` |
| 1x | Set playback speed to 1x (default, highlighted) | **WORKING** | `POST /api/game/replay/speed` |
| 2x | Set playback speed to 2x | **WORKING** | `POST /api/game/replay/speed` |
| 4x | Set playback speed to 4x | **WORKING** | `POST /api/game/replay/speed` |

### 6.5 Wave Jump Buttons

Dynamically generated from timeline data. One button per wave (W1, W2, etc.).

| Button | What It Does | Status | Backend |
|--------|-------------|--------|---------|
| W{N} | Seek to the start of wave N | **WORKING** | `POST /api/game/replay/seek-wave` |

### 6.6 Event Log

Scrolling log of events near the current playback time (2-second window).
Shows timestamp and event summary. Up to 8 events displayed.

Status: **WORKING** -- data comes from `GET /api/game/replay/timeline`.

### 6.7 Data Loading

On entering replay mode, loads:
- `GET /api/game/replay` -- full replay data
- `GET /api/game/replay/timeline` -- event timeline

During playback, polls `GET /api/game/replay/frame` at 4Hz for state and
frame data.


---

## 7. Sensor Net Panel

Live sensor network activation log. Displays motion detector, door sensor,
and tripwire events.

Source: `src/frontend/js/command/panels/sensors.js`

### 7.1 Toolbar

| Element | What It Does | Status |
|---------|-------------|--------|
| Active count | Shows number of currently triggered sensors (e.g., "3 active") | COSMETIC |
| CLEAR button | Clears the sensor event log | COSMETIC |

### 7.2 Event Log

Scrolling list showing sensor events (most recent first). Each entry shows
timestamp, sensor type badge (M/D/T), sensor name, and state (TRIGGERED/CLEAR).
Maximum 50 entries retained.

Data arrives via EventBus events: `sensor:triggered` and `sensor:cleared`.
These events originate from the WebSocket connection.

Status: **WORKING** -- real-time data from WebSocket telemetry.


---

## 8. Battle Stats Panel

Live during-battle statistics panel. Shows per-unit leaderboard, accuracy,
damage, and elimination sparkline.

Source: `src/frontend/js/command/panels/stats.js`

### 8.1 Header Stat Cards

| Card | What It Shows | Status |
|------|---------------|--------|
| ACCURACY | Overall team accuracy percentage | WORKING |
| TOTAL DAMAGE | Cumulative damage dealt | WORKING |
| FRIENDLIES LOST | Count of eliminated friendly units | WORKING |

### 8.2 Unit Leaderboard

Table showing friendly units ranked by kills descending. Columns: RANK,
NAME, KILLS, ACC%, DAMAGE. The #1 unit gets an MVP badge.

### 8.3 Elimination Timeline Sparkline

SVG sparkline showing cumulative eliminations over time. Two lines:
- Green solid: hostile eliminations (kills by friendlies)
- Red dashed: friendly losses

### 8.4 Data Source

Polls `GET /api/game/stats` every 3 seconds during active game phases.
Stops polling when game is idle. One final poll on victory/defeat.

Status: **WORKING** -- endpoint exists in game.py.


---

## 9. Game HUD Panel

Shows game state (wave, score, eliminations) and provides game control
buttons. Auto-opens on game state changes.

Source: `src/frontend/js/command/panels/game-hud.js`

### 9.1 Status Display

| Field | What It Shows | Status |
|-------|---------------|--------|
| PHASE | Current game phase (IDLE/SETUP/ACTIVE/WAVE_COMPLETE/VICTORY/DEFEAT/COUNTDOWN) | COSMETIC |
| WAVE | Current wave / total waves (e.g., "3/10") | COSMETIC |
| SCORE | Current score | COSMETIC |
| ELIMS | Total eliminations | COSMETIC |

### 9.2 Combat Dashboard Sections

These sections render during active game phases, refreshing every 2 seconds:

| Section | What It Shows | Status |
|---------|---------------|--------|
| WAVE PROGRESS | Progress bar showing hostile elimination progress for current wave, with elapsed time | COSMETIC |
| FRIENDLY ROSTER | List of friendly units with health bars (unicode block characters) | COSMETIC |
| COMBAT METRICS | Accuracy, DPS, active threats, morale with trend arrow | COSMETIC |
| MVP | Current highest-kill friendly unit | COSMETIC |
| GAME METRICS | Aggregate shots/distance/damage across all friendly units | COSMETIC |

### 9.3 Placement Toolbar (Setup mode only)

Visible only during IDLE or SETUP phase. Allows selecting which unit type
to place when clicking the map in Setup mode.

| Button | Label | What It Does | Status |
|--------|-------|-------------|--------|
| T | Turret (80m range) | Set placement type to turret | COSMETIC |
| H | Heavy Turret (120m range) | Set placement type to heavy_turret | COSMETIC |
| M | Missile Turret (150m range) | Set placement type to missile_turret | COSMETIC |
| R | Rover (60m range) | Set placement type to rover | COSMETIC |
| D | Drone (50m range) | Set placement type to drone | COSMETIC |
| S | Scout Drone (40m range) | Set placement type to scout_drone | COSMETIC |
| K | Tank (100m range) | Set placement type to tank | COSMETIC |
| A | APC (60m range) | Set placement type to apc | COSMETIC |

Selected type is stored in `window._setupPlacementType`. The map reads this
value when the user clicks in Setup mode to place the correct unit type.

### 9.4 Action Buttons

| Button | Visible When | What It Does | Status | Backend |
|--------|-------------|-------------|--------|---------|
| BEGIN WAR | phase = idle/setup | Opens the Mission Modal | **WORKING** | Modal handles generation/launch |
| SPAWN HOSTILE | always | Spawns a hostile unit for testing | **WORKING** | `POST /api/amy/simulation/spawn` |
| RESET | phase = victory/defeat | Resets game to idle state | **WORKING** | `POST /api/game/reset` |

### 9.5 Upgrade Picker (wave_complete phase only)

During the wave_complete phase, an upgrade grid appears. Fetches available
upgrades from the API, displays them as clickable cards.

| Interaction | What It Does | Status | Backend |
|-------------|-------------|--------|---------|
| Click upgrade card | Selects the upgrade (highlight). Stores in `window._selectedUpgradeId`. | COSMETIC | -- |
| Click friendly unit (with upgrade selected) | Applies upgrade to unit | **WORKING** | `POST /api/game/upgrade` |
| Upgrade list fetch | Loads available upgrades | **WORKING** | `GET /api/game/upgrades` |


---

## 10. Mission Modal

Full-screen modal for mission generation and launch. Shows game mode
selection, LLM generation progress, and mission briefing.

Source: `src/frontend/js/command/mission-modal.js`

### 10.1 Game Mode Selection

| Button | Label | Description | Status |
|--------|-------|-------------|--------|
| B | BATTLE | 10-wave combat defense | COSMETIC (selection only) |
| D | DEFENSE | Hold position against assault | COSMETIC |
| P | PATROL | Patrol and secure perimeter | COSMETIC |
| E | ESCORT | Escort VIP through hostile territory | COSMETIC |
| U | CIVIL UNREST | Riots and crowd control scenario | COSMETIC |
| S | DRONE SWARM | Mass drone attack defense | COSMETIC |

### 10.2 AI Model Selector

| Element | What It Does | Status | Backend |
|---------|-------------|--------|---------|
| Model dropdown | Select which Ollama model to use for LLM generation. Default: "Auto (recommended)" | **WORKING** | `GET /api/game/models` to populate list |
| Model recommendation | Shows count of available models or "Ollama offline" | COSMETIC | |

### 10.3 Action Buttons

| Button | What It Does | Status | Backend |
|--------|-------------|--------|---------|
| [ GENERATE SCENARIO ] | Starts LLM-powered scenario generation with progress tracking | **WORKING** | `POST /api/game/generate` with `{use_llm: true}` |
| [ QUICK START ] | Generates a scripted scenario immediately (no LLM) | **WORKING** | `POST /api/game/generate` with `{use_llm: false}` |
| [ LAUNCH MISSION ] | Deploys the generated scenario and starts the game. Hidden until generation completes. | **WORKING** | `POST /api/game/mission/apply` |
| [ CANCEL ] | Closes the modal | COSMETIC | -- |

### 10.4 Generation Progress

During LLM generation, shows step-by-step progress:
- Step icons for each generation phase (scenario_context, unit_composition, etc.)
- Progress bar (0-100%)
- Status label with current step name
- Prompt preview and result preview per step

Progress data arrives via WebSocket `mission_progress` events.

### 10.5 Mission Briefing

After generation completes, shows:
- SITUATION: reason and stakes
- HOSTILE FORCE: attacker name, motivation, urgency
- CONDITIONS: weather, visibility, mood
- THREAT ANALYSIS: wave count, total hostiles, per-wave breakdown
- BONUS OBJECTIVES: optional scoring objectives with point rewards
- VICTORY CONDITION: win criteria
- YOUR FORCES: unit count and types
- Generated by: source (LLM model or "scripted")


---

## 11. Right-Click Context Menu

Context-sensitive right-click menu on the tactical map. Menu items change
based on whether a unit is selected.

Source: `src/frontend/js/command/context-menu.js`

### 11.1 With Unit Selected

| Item | Icon | What It Does | Status | Backend |
|------|------|-------------|--------|---------|
| DISPATCH HERE | `>` | Emits `unit:dispatch` event with unit ID and click coordinates | **WORKING** | Event triggers `POST /api/amy/simulation/dispatch` |
| SUGGEST TO AMY | `?` | Sends suggestion command to Amy: `suggest: dispatch(unitId, x, y)` | **WORKING** | `POST /api/amy/command` |
| SET WAYPOINT | `+` | Emits `map:waypoint` event with coordinates and unit ID | **COSMETIC** | Map-local waypoint marker |
| CANCEL | `-` | Closes the menu | COSMETIC | -- |

### 11.2 Without Unit Selected

| Item | Icon | What It Does | Status | Backend |
|------|------|-------------|--------|---------|
| DROP MARKER | `x` | Emits `map:marker` event with coordinates | **COSMETIC** | Map-local marker |
| SUGGEST TO AMY: INVESTIGATE | `?` | Sends suggestion command: `suggest: investigate(x, y)` | **WORKING** | `POST /api/amy/command` |
| CANCEL | `-` | Closes the menu | COSMETIC | -- |

ESC key also closes the context menu.


---

## 12. Chat Panel

Slide-out chat panel for communicating with Amy. Hidden by default,
slides in from the right side.

Source: `src/frontend/unified.html`, `src/frontend/js/command/main.js`

| Element | Shortcut | What It Does | Status | Backend |
|---------|----------|-------------|--------|---------|
| Chat input | `/` (focus) | Type message to send to Amy | **WORKING** | `POST /api/amy/chat` |
| SEND button | -- | Send the typed message to Amy | **WORKING** | `POST /api/amy/chat` |
| Enter key | -- | Send message (same as SEND button) | **WORKING** | |
| Close (X) button | -- | Close chat panel | COSMETIC | |
| ESC key | -- | Close chat panel | COSMETIC | |
| Toggle | `C` | Open/close chat panel | COSMETIC | |

**Context area** at top of chat shows Amy's latest thought and current mood.
Updated in real-time from store.

**Message log** shows conversation history. Amy's responses arrive via
WebSocket `chat:amy_response` events. Amy's autonomous thoughts appear as
dimmed system messages when the chat is open.


---

## 13. Game Over Overlay

Full-screen overlay shown on victory or defeat. Fetches after-action stats
from the backend.

Source: `src/frontend/unified.html`, `src/frontend/js/command/main.js`,
`src/frontend/js/command/game-over-stats.js`

### 13.1 Static Display

| Element | What It Shows | Status |
|---------|---------------|--------|
| Title | "VICTORY" (green) or "DEFEAT" (magenta) | COSMETIC |
| FINAL SCORE | Game score from store | COSMETIC |
| WAVES SURVIVED | Wave count from store (e.g., "7/10") | COSMETIC |
| TOTAL ELIMINATIONS | Elimination count from store | COSMETIC |

### 13.2 Dynamic Stats Sections

Fetched in parallel on overlay open:

| Section | Data Source | Status | Backend |
|---------|-----------|--------|---------|
| MVP Spotlight | `GET /api/game/stats/summary` (includes MVP) | **WORKING** | Endpoint exists |
| Combat Stats Grid | `GET /api/game/stats/summary` | **WORKING** | Endpoint exists |
| Unit Performance Table | `GET /api/game/stats` | **WORKING** | Endpoint exists |

### 13.3 Action Button

| Button | What It Does | Status | Backend |
|--------|-------------|--------|---------|
| [ PLAY AGAIN ] | Resets game, hides overlay | **WORKING** | `POST /api/game/reset` |


---

## 14. Help Overlay

Keyboard shortcuts reference overlay.

Source: `src/frontend/unified.html`

| Shortcut | What It Does | Status |
|----------|-------------|--------|
| Close (X) button | Closes help overlay | COSMETIC |
| Click backdrop | Closes help overlay | COSMETIC |
| ESC key | Closes help overlay | COSMETIC |

Toggle via `?` key or HELP > Keyboard Shortcuts menu item.


---

## 15. Keyboard Shortcuts (Complete Reference)

Source: `src/frontend/js/command/main.js`

### 15.1 General

| Key | Action | Status |
|-----|--------|--------|
| `?` | Toggle help overlay | COSMETIC |
| `ESC` | Close all overlays/modals/chat. Exit setup mode to observe. | COSMETIC |
| `C` | Toggle Amy chat panel | COSMETIC |
| `/` | Focus chat input (opens chat if closed) | COSMETIC |
| `M` | Toggle minimap panel | COSMETIC |
| `Tab` | Cycle panel focus (unified) / toggle sidebar (legacy) | COSMETIC |

### 15.2 Map Modes

| Key | Action | Status |
|-----|--------|--------|
| `O` | Switch to Observe mode | COSMETIC |
| `T` | Switch to Tactical mode | COSMETIC |
| `S` | Switch to Setup mode | COSMETIC |

### 15.3 Map Controls

| Key | Action | Status |
|-----|--------|--------|
| `F` | Switch to tactical mode and center on combat action | COSMETIC |
| `A` | Toggle auto-follow (camera tracks combat) | COSMETIC |
| `R` | Toggle replay panel | COSMETIC |
| `[` | Zoom out | COSMETIC |
| `]` | Zoom in | COSMETIC |

### 15.4 Map Layer Toggles

| Key | Action | Status |
|-----|--------|--------|
| `U` | Toggle unit markers | COSMETIC |
| `V` | Toggle fog of war | COSMETIC |
| `K` | Toggle buildings | COSMETIC |
| `G` | Toggle roads | COSMETIC |
| `I` | Toggle satellite imagery | COSMETIC |
| `H` | Toggle terrain overlay | COSMETIC |

### 15.5 Panels

| Key | Action | Status |
|-----|--------|--------|
| `1` | Toggle Amy panel | COSMETIC |
| `2` | Toggle Units panel | COSMETIC |
| `3` | Toggle Alerts panel | COSMETIC |
| `4` | Toggle Game HUD panel | COSMETIC |
| `5` | Toggle Mesh panel | COSMETIC |

### 15.6 Layouts

| Key | Action | Status |
|-----|--------|--------|
| `Ctrl+1` | Apply Commander layout | COSMETIC |
| `Ctrl+2` | Apply Observer layout | COSMETIC |
| `Ctrl+3` | Apply Tactical layout | COSMETIC |
| `Ctrl+4` | Apply Battle layout | COSMETIC |
| `Ctrl+Shift+S` | Open save layout input | COSMETIC |

### 15.7 Game

| Key | Action | Status | Backend |
|-----|--------|--------|---------|
| `B` | Open Mission Modal (when game is idle/setup) | **WORKING** | Modal calls generation APIs |
| `N` | Toggle Mission Modal open/close | **WORKING** | Same |


---

## 16. Header Bar

Fixed header across the top of the viewport.

Source: `src/frontend/unified.html`

| Element | What It Shows | Status |
|---------|---------------|--------|
| Game score area | WAVE, SCORE, ELIMS counters (hidden when no game active) | COSMETIC |
| Connection indicator | ONLINE/OFFLINE with colored dot, reflects WebSocket state | COSMETIC |


---

## 17. Status Bar

Fixed 20px bar at the bottom of the viewport.

Source: `src/frontend/unified.html`

| Element | What It Shows | Status |
|---------|---------------|--------|
| FPS counter | Map rendering frames per second | COSMETIC |
| Alive count | Number of active (non-eliminated) units | COSMETIC |
| Threats count | Number of active hostile units | COSMETIC |
| WS status | WebSocket connection state (OK / --) | COSMETIC |
| Version label | "TRITIUM-SC v0.1.0" | COSMETIC |
| Help hint | `?` key badge with "HELP" label | COSMETIC |


---

## 18. Map Overlays (HUD Layer)

Canvas and DOM overlays rendered on top of the tactical map during gameplay.

Source: `src/frontend/unified.html`, `src/frontend/js/war-hud.js`

| Element | What It Does | Status |
|---------|-------------|--------|
| War countdown | Shows countdown timer before wave starts | COSMETIC |
| War wave banner | Full-width banner announcing wave number | COSMETIC |
| War elimination feed | Scrolling kill feed showing recent eliminations | COSMETIC |
| War score | Floating score display during active game | COSMETIC |
| War begin button | `[ BEGIN WAR ]` button rendered on the map canvas | **WORKING** (`POST /api/game/begin`) |
| War game over | Game over text on the map canvas | COSMETIC |
| War Amy toast | Amy announcement toast overlay | COSMETIC |
| Center banner | Large center-screen text for announcer events (wave banners, kill streaks) | COSMETIC |
| Map coordinates | Shows cursor X/Y game coordinates | COSMETIC |
| FPS counter | Map-level FPS display | COSMETIC |


---

## 19. Toast Notifications

Pop-up notifications in the top-right corner. Auto-dismiss after a few seconds.

Source: `src/frontend/js/command/main.js`

| Event Source | Content | Status |
|-------------|---------|--------|
| `amy:thought` | Amy's autonomous thoughts | WORKING (via WebSocket) |
| `robot:thought` | Robot LLM thoughts | WORKING (via WebSocket) |
| `alert:new` | Alert messages | WORKING (via WebSocket) |
| `announcer` | War commentary banners | WORKING (via WebSocket) |
| `game:elimination` | Kill notifications ("X neutralized Y") | WORKING (via WebSocket) |
| `mesh:text` | Mesh radio text messages | WORKING (via WebSocket) |
| `toast:show` | Programmatic toasts from any module | COSMETIC |


---

## 20. Known Issues Summary

### Recently Fixed

- **Device command endpoint**: `POST /api/devices/{id}/command` now exists
  (`src/app/routers/devices.py`). Routes sensor/camera commands to MQTT
  bridge with Lua fallback, mesh text to meshtastic bridge. All 5 formerly
  broken buttons (Sensor ENABLE/DISABLE/TEST, Camera OFF, Mesh SEND) now work.
- **DISPATCH/PATROL/AIM buttons**: Now correctly emit EventBus events
  (`unit:dispatch-mode`, `unit:patrol-mode`, `unit:aim-mode`) instead of
  directly calling non-existent API endpoints.
- **Turret AUTO TARGET and MANUAL**: Now send Lua commands (`auto_target()`,
  `manual_target()`) via `POST /api/amy/command`.
- **Person type routing**: Now correctly shows NPC controls instead of
  Sensor controls (alias `'person': 'npc'` added to DeviceControlRegistry).
- **Unit Markers toggle**: Added with shortcut `U`.
- **2D/3D indicator independence**: Toggling "3D Models" off now restores full
  2D circle icons instead of showing tiny dots. Rendering path uses
  `use3DPath = has3D && modelsVisible` so toggling either works independently.
- **Detail panel thought text**: Fixed regression where `_lastDetailId` guard
  prevented ALL updates to the selected unit's detail panel. Now uses
  `_updateDetailFields()` for incremental DOM updates on the same unit.
- **Thought store notification**: `amy_npc_thought` and `amy_npc_thought_clear`
  WebSocket handlers now call `TritiumStore._scheduleNotify('units')` so the
  UI reactively updates when thoughts change.
- **Turret PAN/TILT sliders**: Now send debounced `motor.aim(pan, tilt)` Lua
  commands (150ms debounce) instead of only updating display values.
- **TAKE CONTROL**: Now sets `controlledUnitId` in store, enables WASD movement
  (via `/api/npc/{id}/action`), click-to-dispatch on map, ESC to release, and
  shows a pulsing cyan ring indicator on the controlled unit.

### Camera SNAPSHOT

The SNAPSHOT button opens `/api/amy/nodes/{id}/snapshot` in a new tab.
The `/api/amy/nodes/{id}/video` endpoint exists (MJPEG stream), but
the `/snapshot` variant is not confirmed in the router. Status: **PARTIAL**.

### PAN/TILT Sliders (Turret)

The turret control has PAN and TILT range sliders that update their display
value in real-time, but **do not automatically send** aim commands to the
backend. The user must manually use the AIM button or send a Lua command
to move the turret. Status: **COSMETIC** (display-only).
