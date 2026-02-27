# 3D Terrain Implementation Plan

## Goal

Add real 3D terrain to the MapLibre GL JS map so hills, valleys, and slopes
deform the entire view surface. All layers (satellite imagery, buildings,
roads, vector overlays) and all Three.js tactical objects (unit models,
projectiles, effects) must project correctly onto the terrain mesh.

The terrain toggle must be a first-class layer control alongside the existing
satellite/buildings/roads/fog toggles, accessible from the Map menu and
keyboard shortcut.

---

## 1. DEM Tile Source Selection

### Requirements
- Free, no API key (matches project philosophy: "All data sources are FREE
  with no API keys")
- Terrarium or MapboxRGB encoding (the two formats MapLibre raster-dem supports)
- Global coverage (the operator might relocate the geo-reference)
- Reasonable zoom depth (z12+ for neighborhood-scale terrain)

### Candidates

| Source | URL Template | Encoding | Max Zoom | API Key | Notes |
|--------|-------------|----------|----------|---------|-------|
| **MapLibre Demo Tiles** | `https://demotiles.maplibre.org/terrain-tiles/{z}/{x}/{y}.png` | MapboxRGB | 12 | None | JAXA AW3D30 30m DEM. Hosted on GitHub Pages. Used in official MapLibre examples. Has a `tiles.json` TileJSON endpoint. |
| **AWS Terrain Tiles (Mapzen)** | `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png` | Terrarium | 15 | None | Mapzen/Tilezen open data on AWS Public Datasets. Higher max zoom (15 vs 12). S3 direct access, no auth. |
| MapTiler Terrain | `https://api.maptiler.com/tiles/terrain-rgb-v2/{z}/{x}/{y}.webp` | MapboxRGB | 14 | **Required** | Excluded -- requires API key. |

### Decision: AWS Terrain Tiles (Mapzen Terrarium)

**Primary:** AWS Terrain Tiles at `s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`
- Higher max zoom (15) gives better terrain detail at neighborhood scale
- Terrarium encoding with 3mm precision
- AWS Public Dataset, free, no sign-up
- Well-tested with MapLibre (Terrarium encoding is natively supported)

**Fallback:** MapLibre Demo Tiles at `demotiles.maplibre.org/terrain-tiles/{z}/{x}/{y}.png`
- Lower max zoom (12) but guaranteed to stay available (MapLibre project hosts it)
- Useful as fallback if AWS S3 has issues or for offline testing

### Proxy Consideration

The existing tile proxy pattern (`/api/geo/tile/{z}/{x}/{y}` for ESRI satellite)
should be extended with a new endpoint `/api/geo/terrain-tile/{z}/{x}/{y}` that:
1. Proxies requests to the AWS S3 URL
2. Caches tiles on disk (same pattern as `_TILE_CACHE`)
3. Avoids CORS issues from the frontend
4. Enables future offline operation (pre-cache a region)

Alternatively, since both AWS S3 and MapLibre demo tiles serve with permissive
CORS headers, the frontend can fetch directly. Start with direct fetch, add
proxy later if needed for offline support.

---

## 2. MapLibre Terrain Setup

### 2.1 Add raster-dem Source to Style

In `_buildStyle()`, add a new source to the `sources` object:

```javascript
// In _buildStyle(), inside the sources: { ... } block:
'terrain-dem': {
    type: 'raster-dem',
    tiles: [
        'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png',
    ],
    encoding: 'terrarium',
    tileSize: 256,
    maxzoom: 15,
    attribution: '&copy; <a href="https://registry.opendata.aws/terrain-tiles/">Mapzen/AWS Terrain Tiles</a>',
},
```

The source is always declared but terrain is only activated when the user
toggles it on (via `map.setTerrain()`).

### 2.2 Enable Terrain

Terrain is enabled/disabled at runtime, not in the initial style:

```javascript
// Enable
map.setTerrain({
    source: 'terrain-dem',
    exaggeration: 1.0,   // 1.0 = true-to-life, 2.0 = dramatic
});

// Disable
map.setTerrain(null);
```

The `exaggeration` parameter controls vertical scale:
- `1.0` = true elevation (realistic)
- `1.5-2.0` = exaggerated (more dramatic, better for flat areas like Dublin, CA)
- `0.5` = subdued (useful if terrain is distracting)

For Dublin, CA (mostly flat suburbia with gentle hills to the east), an
exaggeration of `1.5` makes the terrain visible without being cartoonish.
Store the exaggeration value in `_state` so it can be tuned.

### 2.3 Hillshade Layer (Optional Enhancement)

A hillshade layer adds shading that makes terrain relief visible even when
viewed top-down. Add it as a MapLibre layer below the satellite imagery:

```javascript
// Insert BEFORE 'satellite-layer' in the layers array
{
    id: 'terrain-hillshade',
    type: 'hillshade',
    source: 'terrain-dem',
    paint: {
        'hillshade-illumination-direction': 315,  // NW sun
        'hillshade-shadow-color': '#000000',
        'hillshade-highlight-color': '#ffffff',
        'hillshade-accent-color': '#000000',
        'hillshade-exaggeration': 0.3,  // subtle
    },
    layout: { 'visibility': 'none' },  // off by default, toggled with terrain
},
```

Note: The hillshade source can reuse the same `terrain-dem` source. MapLibre
handles the dual use (terrain mesh + hillshade rendering) from one source.

### 2.4 Sky / Atmosphere Configuration

When terrain is enabled, a sky layer creates the horizon/atmosphere effect
that prevents the map from looking like it floats in a void:

```javascript
// When enabling terrain:
map.setSky({
    'sky-color': '#060609',           // match dark background
    'horizon-color': '#0a1020',       // dark blue horizon
    'fog-color': '#060609',           // fog matches background
    'fog-ground-blend': 0.5,
    'horizon-fog-blend': 0.8,
    'sky-horizon-blend': 0.9,
    'atmosphere-blend': 0.6,          // subtle atmosphere
});

// When disabling terrain:
map.setSky(null);  // or map.setSky({})
```

The sky colors are tuned to match the CYBERCORE dark theme (#060609 background).
The atmosphere should be subtle -- this is a tactical display, not a flight
simulator.

---

## 3. Three.js Custom Layer: Terrain-Aware Positioning

This is the critical integration challenge. Currently, the Three.js custom
layer uses a static reference transform:

```javascript
// Current code (line ~441):
const rootMatrix = new THREE.Matrix4()
    .makeTranslation(refMc.x, refMc.y, refMc.z || 0)
    .scale(new THREE.Vector3(ms, -ms, ms));
```

This pins everything to sea level (z=0). When terrain is enabled, objects
must be placed at the terrain surface elevation.

### 3.1 The Problem

When `map.setTerrain()` is active:
- MapLibre's built-in layers (satellite, buildings, roads) automatically
  drape over the terrain mesh. This is handled internally.
- `fill-extrusion` layers (buildings-3d) also automatically sit on terrain.
  The extrusion height is added on top of the terrain elevation. No change
  needed.
- DOM markers (`maplibregl.Marker`) automatically clamp to terrain. No
  change needed.
- **Custom layers (Three.js) do NOT automatically account for terrain.**
  The projection matrix from MapLibre includes the terrain mesh deformation,
  but the Three.js objects are positioned in a flat coordinate space. Objects
  will appear to float above valleys and sink into hills.

### 3.2 The Solution: Dynamic Reference Transform

The official MapLibre Three.js-on-terrain example demonstrates the pattern:

**On every render frame**, re-query the terrain elevation at the scene origin
and rebuild the reference transform:

```javascript
render(gl, args) {
    // Query current terrain elevation at geo center
    const terrainElev = _state.terrainEnabled
        ? (map.queryTerrainElevation(
            new maplibregl.LngLat(_state.geoCenter.lng, _state.geoCenter.lat)
          ) || 0)
        : 0;

    // Rebuild reference Mercator coordinate WITH terrain elevation
    const refLngLat = [_state.geoCenter.lng, _state.geoCenter.lat];
    const refMc = maplibregl.MercatorCoordinate.fromLngLat(refLngLat, terrainElev);
    const ms = refMc.meterInMercatorCoordinateUnits();

    // Update root group transform
    const rootMatrix = new THREE.Matrix4()
        .makeTranslation(refMc.x, refMc.y, refMc.z || 0)
        .scale(new THREE.Vector3(ms, -ms, ms));
    _state.threeRoot.matrix.copy(rootMatrix);
    _state.threeRoot.matrixWorld.copy(rootMatrix);

    // ... rest of render (camera projection, etc.)
}
```

This makes the scene origin track the terrain surface. Objects near the
origin will be approximately correct, but objects far from the origin
(hundreds of meters away) will have terrain slope error.

### 3.3 Per-Unit Terrain Elevation

For accurate placement of individual units on terrain slopes, each unit's
Z position should be adjusted by the terrain elevation at its location:

```javascript
function _sync3DUnits() {
    // ... existing code ...

    units.forEach((unit, id) => {
        const gx = pos.x || 0;
        const gy = pos.y || 0;
        const lngLat = _gameToLngLat(gx, gy);

        // Query terrain elevation for this unit's position
        let terrainZ = 0;
        if (_state.terrainEnabled && _state.map) {
            const unitElev = _state.map.queryTerrainElevation(
                new maplibregl.LngLat(lngLat[0], lngLat[1])
            );
            // Elevation relative to scene origin elevation
            const originElev = _state.map.queryTerrainElevation(
                new maplibregl.LngLat(_state.geoCenter.lng, _state.geoCenter.lat)
            ) || 0;
            terrainZ = (unitElev || 0) - originElev;
        }

        const baseAlt = type.includes('drone') ? UNIT_3D.DRONE_ALT : UNIT_3D.ALT;
        group.position.set(gx, gy, terrainZ + baseAlt);
    });
}
```

**Performance note:** `queryTerrainElevation()` is a synchronous read from
the terrain mesh already in GPU memory. It does NOT trigger network requests.
Calling it per-unit (10-30 units) per frame is acceptable.

### 3.4 Combat Effects on Terrain

The same elevation query should apply to combat effects (tracers, impacts,
explosions). The `_gameToMercator()` helper should be updated:

```javascript
function _gameToMercator(gx, gy, altMeters) {
    let terrainZ = 0;
    if (_state.terrainEnabled && _state.map) {
        const lngLat = _gameToLngLat(gx, gy);
        const unitElev = _state.map.queryTerrainElevation(
            new maplibregl.LngLat(lngLat[0], lngLat[1])
        ) || 0;
        const originElev = _state.map.queryTerrainElevation(
            new maplibregl.LngLat(_state.geoCenter.lng, _state.geoCenter.lat)
        ) || 0;
        terrainZ = unitElev - originElev;
    }
    return {
        x: gx,
        y: gy,
        z: (altMeters || 0) + terrainZ,
        meterInMercatorCoordinateUnits() { return 1.0; },
    };
}
```

---

## 4. Toggle Integration

### 4.1 Module State

Add to `_state`:

```javascript
showTerrain: false,          // terrain mesh enabled
terrainEnabled: false,       // runtime flag (true after setTerrain succeeds)
terrainExaggeration: 1.5,    // default exaggeration
```

### 4.2 Toggle Function

```javascript
export function toggleTerrain() {
    _state.showTerrain = !_state.showTerrain;
    if (!_state.map) return;

    if (_state.showTerrain) {
        // Enable terrain
        _state.map.setTerrain({
            source: 'terrain-dem',
            exaggeration: _state.terrainExaggeration,
        });
        // Enable sky atmosphere for terrain horizon
        _state.map.setSky({
            'sky-color': '#060609',
            'horizon-color': '#0a1020',
            'fog-color': '#060609',
            'fog-ground-blend': 0.5,
            'horizon-fog-blend': 0.8,
            'sky-horizon-blend': 0.9,
            'atmosphere-blend': 0.6,
        });
        // Enable hillshade if layer exists
        if (_state.map.getLayer('terrain-hillshade')) {
            _state.map.setLayoutProperty('terrain-hillshade', 'visibility', 'visible');
        }
        _state.terrainEnabled = true;
    } else {
        // Disable terrain
        _state.map.setTerrain(null);
        _state.map.setSky(null);
        if (_state.map.getLayer('terrain-hillshade')) {
            _state.map.setLayoutProperty('terrain-hillshade', 'visibility', 'none');
        }
        _state.terrainEnabled = false;
    }

    _updateLayerHud();
    console.log(`[MAP-ML] Terrain ${_state.showTerrain ? 'ON' : 'OFF'}`);
}
```

### 4.3 Layer HUD Update

In `_updateLayerHud()`, add:

```javascript
if (_state.showTerrain) layers.push('TERRAIN');
```

### 4.4 Map State Export

In `getMapState()`, add:

```javascript
showTerrain: _state.showTerrain,
```

### 4.5 setLayers() Support

In `setLayers()`, add:

```javascript
if (layers.terrain !== undefined) {
    const want = !!layers.terrain;
    if (_state.showTerrain !== want) toggleTerrain();
}
```

### 4.6 Menu Bar Integration

In `menu-bar.js` `_mapMenuItems()`, add after the Fog entry:

```javascript
{ label: 'Terrain', shortcut: 'H', checkable: true,
  checked: () => s().showTerrain, action: () => mapActions.toggleTerrain() },
```

Shortcut `H` for "Height" (available, not used by other shortcuts).

### 4.7 main.js Wiring

In `main.js`:
- Add `toggleTerrain` to the import from `map-maplibre.js`
- Add `toggleTerrain` to the `mapActions` object
- Add keyboard shortcut `H` to the key handler

---

## 5. Performance Considerations

### 5.1 Tile Loading

- DEM tiles are 256x256 PNG images (~30-80KB each at z12-z15)
- At neighborhood zoom (z16), the DEM source maxes out at z15 so tiles are
  upscaled (one z15 tile covers a larger area). This is fine -- terrain
  doesn't need pixel-precision at close zoom.
- Browser cache handles repeat visits. No special caching needed beyond
  standard HTTP cache headers (AWS S3 sets them properly).

### 5.2 GPU Impact

- Terrain mesh adds ~10-20% GPU overhead for triangle rasterization
- The mesh is generated from DEM tiles and cached per tile
- At low zoom (z10-12), larger terrain mesh = more triangles
- At high zoom (z16+), fewer visible tiles = less terrain geometry
- Neighborhood zoom (z16) is the sweet spot

### 5.3 queryTerrainElevation() Cost

- Synchronous read from already-loaded terrain mesh
- Per-call cost: negligible (~microseconds)
- Called per-unit per-frame: 10-30 units x 60fps = 600-1800 calls/sec
- This is well within budget

### 5.4 Three.js Render Loop

- The reference transform rebuild (Section 3.2) adds one Matrix4
  construction per frame. Cost: negligible.
- Per-unit elevation query (Section 3.3) adds one `queryTerrainElevation()`
  call per unit per sync cycle (10Hz, not 60fps). Cost: negligible.

### 5.5 Recommendation

- Default terrain OFF (flat areas like Dublin, CA show minimal terrain)
- Let the operator toggle it on when interesting
- Exaggeration adjustable (future: slider in settings panel)
- If frame rate drops below 30fps with terrain ON, log a warning

---

## 6. Edge Cases and Gotchas

### 6.1 Terrain Tiles Not Yet Loaded

`queryTerrainElevation()` returns `null` if terrain tiles haven't loaded for
that area yet. Always use `|| 0` fallback:

```javascript
const elev = map.queryTerrainElevation(lngLat) || 0;
```

### 6.2 Terrain + Fog Interaction

The existing `toggleFog()` uses `map.setFog()`. When terrain is enabled, the
sky configuration (via `map.setSky()`) provides its own fog parameters. If
both fog and terrain are on, the fog settings from `setFog()` will combine
with the sky fog. Test the visual interaction carefully. May need to disable
the manual fog when terrain is on, or merge the settings.

### 6.3 Fill-Extrusion + Terrain

Building extrusions (`buildings-3d` layer) automatically sit on terrain when
`setTerrain()` is active. The `fill-extrusion-base` property becomes relative
to the terrain surface, not sea level. No code changes needed for buildings.

### 6.4 Map Pitch Limits

When terrain is enabled, MapLibre may adjust the maximum pitch to prevent
viewing "under" the terrain mesh. The current max pitch is not explicitly set
(default 85). This should work fine. If issues arise, set `maxPitch: 85` on
the Map constructor.

### 6.5 Coordinate System Consistency

The game coordinate system (x=east, y=north, z=up, origin=geoCenter) remains
unchanged. Terrain elevation is an additive Z offset on top of the game
coordinate Z. The simulation engine does NOT need to know about terrain --
it operates on a flat plane. Only the visual renderer accounts for terrain.

### 6.6 Exaggeration and queryTerrainElevation

Per the MapLibre docs: "if terrain is enabled with exaggeration, the value
returned [by queryTerrainElevation] is multiplied by that exaggeration value."
This means the elevation returned already includes the exaggeration factor.
This is correct for visual positioning -- objects should match the visually
exaggerated terrain, not the real terrain.

---

## 7. Implementation Phases

### Phase 1: Basic Terrain (Minimal Viable)
1. Add `terrain-dem` source to `_buildStyle()`
2. Add `showTerrain` / `terrainEnabled` to `_state`
3. Implement `toggleTerrain()` with `setTerrain()` / `setSky()`
4. Wire into menu bar, keyboard shortcut, layer HUD, exports
5. Test: toggle terrain on/off, verify satellite/buildings drape correctly

**No Three.js changes yet.** Unit models will float/sink slightly but
the terrain itself will be visible and all MapLibre-native layers will
be correct.

### Phase 2: Three.js Terrain Alignment
1. Update `render()` in the custom layer to re-query terrain elevation at
   scene origin every frame and rebuild the reference transform
2. Update `_sync3DUnits()` to query per-unit terrain elevation
3. Update `_gameToMercator()` for combat effects terrain offset
4. Test: units follow terrain slopes, projectiles fly over hills

### Phase 3: Polish
1. Add hillshade layer for subtle shading
2. Add exaggeration control (store in _state, maybe a slider later)
3. Handle fog + terrain interaction
4. Add terrain proxy endpoint to geo.py (for offline/caching)
5. Performance profiling and optimization

---

## 8. Testing Approach

### 8.1 Unit Tests (JS -- test.sh tier 3)

```javascript
// test_terrain.js (add to tests/js/)

// Toggle state management
test('toggleTerrain flips showTerrain state', ...);
test('getMapState includes showTerrain', ...);
test('setLayers with terrain toggles correctly', ...);

// Elevation helpers
test('_gameToMercator returns zero terrainZ when terrain disabled', ...);
test('_gameToMercator adds terrain elevation when enabled', ...);
```

### 8.2 Visual Smoke Tests (Playwright -- test.sh tier 10/11)

```python
# In tests/visual/test_unified_smoke.py or new test_terrain.py

def test_terrain_toggle_renders():
    """Toggle terrain on via JS, verify no console errors."""
    page.evaluate('window._mapActions.toggleTerrain()')
    # Wait for terrain tiles to load
    page.wait_for_timeout(3000)
    # Verify no errors
    assert len(console_errors) == 0
    # Verify terrain state
    state = page.evaluate('window._mapActions.getMapState()')
    assert state['showTerrain'] == True

def test_terrain_toggle_off():
    """Toggle terrain off, verify clean disable."""
    page.evaluate('window._mapActions.toggleTerrain()')  # on
    page.evaluate('window._mapActions.toggleTerrain()')  # off
    state = page.evaluate('window._mapActions.getMapState()')
    assert state['showTerrain'] == False
```

### 8.3 Visual Proof (Screenshot comparison)

After implementation, capture screenshots with terrain on/off for the
project report. Use the same pattern as `tests/visual/test_battle_proof.py`.

### 8.4 Manual Verification

Dublin, CA is mostly flat. To verify terrain is working:
1. Pan to the Dublin hills east of town (~37.72, -121.86)
2. Tilt to 60+ degrees pitch
3. The hills should visibly rise above the flat valley
4. Buildings on hillsides should sit on the terrain slope
5. Units patrolling on sloped terrain should follow the ground

Alternatively, temporarily set `exaggeration: 3.0` to make even small
elevation differences dramatically visible for verification.

---

## 9. Files to Modify

| File | Changes |
|------|---------|
| `frontend/js/command/map-maplibre.js` | Add terrain-dem source, `toggleTerrain()`, state vars, render() terrain elevation, `_sync3DUnits()` terrain Z, `_gameToMercator()` terrain Z, layer HUD, exports |
| `frontend/js/command/menu-bar.js` | Add Terrain menu item with shortcut H |
| `frontend/js/command/main.js` | Import `toggleTerrain`, add to `mapActions`, add keyboard shortcut |
| `src/app/routers/geo.py` | (Phase 3) Add `/api/geo/terrain-tile/{z}/{x}/{y}` proxy endpoint |
| `tests/js/test_terrain.js` | New: terrain toggle state tests |
| `tests/visual/test_terrain.py` | New: Playwright terrain smoke tests |

---

## 10. Reference Links

- MapLibre 3D Terrain example: https://maplibre.org/maplibre-gl-js/docs/examples/3d-terrain/
- MapLibre Three.js on terrain: https://maplibre.org/maplibre-gl-js/docs/examples/adding-3d-models-using-threejs-on-terrain/
- MapLibre Terrain spec: https://maplibre.org/maplibre-style-spec/terrain/
- MapLibre Sky spec: https://maplibre.org/maplibre-style-spec/sky/
- MapLibre Map API (setTerrain, queryTerrainElevation): https://maplibre.org/maplibre-gl-js/docs/API/classes/Map/
- AWS Terrain Tiles (Mapzen): https://registry.opendata.aws/terrain-tiles/
- MapLibre Demo Tiles: https://demotiles.maplibre.org/terrain-tiles/
- TerrainControl widget: https://maplibre.org/maplibre-gl-js/docs/API/classes/TerrainControl/
- Sky, Fog, Terrain combined example: https://maplibre.org/maplibre-gl-js/docs/examples/sky-fog-terrain/
