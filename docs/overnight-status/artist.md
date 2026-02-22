# Artist Agent -- Overnight Build Status

## Completed

### Task 1: Procedural Unit Icons
- Created `frontend/js/command/unit-icons.js` (508 lines)
- 8 distinct unit type icons: rover (rounded rect + 4 wheels), drone (X + spinning rotors), turret (pentagon + barrel), hostile_person (pulsing diamond), neutral_person (circle + walking legs), tank (hull + turret barrel), sensor/camera (circle + FOV cone)
- All icons rotate with heading via ctx.save/rotate/restore
- All icons scale with zoom parameter
- Selected units get pulsing cyan (#00f0ff) ring
- Health bar (green->yellow->red gradient) drawn below damaged units
- Neutralized units fade to 35% alpha with red X overlay
- 29 unit tests in `tests/js/test_unit_icons.js` -- all pass
- Integrated into `frontend/js/command/map.js` as primary renderer, replacing `warCombatDrawTargetShape` for unit drawing

### Task 2: Combat Visual Improvements
- **Projectile trails**: 8 segments (was 4), 2px wider radius, brighter alpha curve (0.8 max vs 0.6), white bright core on projectile head
- **Muzzle flash**: White-yellow flash (radius 6-10px, 0.1s duration) spawned at firing position on each shot
- **Explosion particles**: 29 particles on elimination (15 red + 8 orange + 6 yellow, was 20 total), sizes 2-8px (was 1-6px), downward gravity (15 units/s^2) for arcing trajectories
- **Screen shake**: 3px intensity, 0.2s duration, decaying random offset applied as canvas translate, restored before HUD layers

### Task 3: Fog of War
- Integrated existing `war-fog.js` fogDraw() into Command Center map.js rendering pipeline
- Rendered between zones (Layer 4) and targets (Layer 5)
- Updated vision radii to spec: rovers 40m, drones 60m, turrets 50m, cameras 30m
- Dark overlay rgba(5,5,15,0.50) with radial gradient vision circles punched out
- Neon cyan glow edges on vision boundaries
- Added `toggleFog()` export and `fogEnabled` state
- Updated `test_war_fog.js` assertions for new radii -- all 45 tests pass

### Task 4: Playwright Visual Tests
- Created `tests/visual/test_unit_graphics.py` with 7 OpenCV-based visual tests:
  1. Friendly units produce green pixels (>= 30 in alliance color range)
  2. Hostile units produce red/magenta pixels
  3. Canvas is not blank (std dev > 5)
  4. Header shows unit count
  5. Kill feed DOM element exists
  6. Fog gradient: center vs corner brightness comparison
  7. Minimap canvas exists with non-zero dimensions

## Test Results
- `./test.sh 3` (JS tests): 6/6 files pass, 310 total assertions
- New test_unit_icons.js: 29/29 pass
- Updated test_war_fog.js: 45/45 pass

## Files Changed
- `frontend/js/command/unit-icons.js` -- NEW (508 lines)
- `frontend/js/command/map.js` -- import unit icons, integrate fog, screen shake
- `frontend/js/command/main.js` -- import toggleFog, add to mapActions
- `frontend/js/war-combat.js` -- enhanced trails, muzzle flash, particles, gravity, screen shake
- `frontend/js/war-fog.js` -- updated vision radii
- `tests/js/test_unit_icons.js` -- NEW (347 lines, 29 tests)
- `tests/js/test_war_fog.js` -- updated expected values
- `tests/visual/test_unit_graphics.py` -- NEW (7 visual tests)

## Commits
1. `396221d` Add procedural unit icons with distinct shapes per unit type
2. `b3c5b8b` Enhance combat visuals: wider trails, muzzle flash, more particles, screen shake
3. `c56cd75` Integrate fog of war into Command Center tactical map
