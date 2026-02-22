/**
 * TRITIUM-SC War Room — Fog of War + Minimap tests
 * Run: node tests/js/test_war_fog.js
 *
 * Tests fog calculation, vision radius per unit type, minimap coordinate
 * mapping, viewport rectangle, and toggle state management.
 */

const fs = require('fs');
const vm = require('vm');

// Simple test runner
let passed = 0, failed = 0;
function assert(cond, msg) {
    if (!cond) { console.error('FAIL:', msg); failed++; }
    else { console.log('PASS:', msg); passed++; }
}
function assertClose(a, b, eps, msg) {
    assert(Math.abs(a - b) < (eps || 0.001), msg + ` (got ${a}, expected ${b})`);
}

// Load war-fog.js into a sandboxed context
const code = fs.readFileSync(__dirname + '/../../frontend/js/war-fog.js', 'utf8');
let perfNow = 1000;
const ctx = vm.createContext({
    Math, Date, console, Map, Array, Object, Number, Infinity, Boolean,
    performance: { now: () => perfNow },
    document: {
        createElement: () => ({ getContext: () => null }),
    },
});
vm.runInContext(code, ctx);

const {
    FOG_VISION_RADII,
    fogGetVisionRadius,
    fogBuildVisionMap,
    fogState,
    minimapWorldToMinimap,
    minimapGetViewportRect,
} = ctx;

// ============================================================
// Vision radius per unit type
// ============================================================

console.log('\n--- Vision Radii ---');

(function testTurretVisionRadius() {
    const r = fogGetVisionRadius({ asset_type: 'turret' });
    assertClose(r, 50, 0.001, 'Turret vision radius is 50');
})();

(function testDroneVisionRadius() {
    const r = fogGetVisionRadius({ asset_type: 'drone' });
    assertClose(r, 60, 0.001, 'Drone vision radius is 60');
})();

(function testRoverVisionRadius() {
    const r = fogGetVisionRadius({ asset_type: 'rover' });
    assertClose(r, 40, 0.001, 'Rover vision radius is 40');
})();

(function testCameraVisionRadius() {
    const r = fogGetVisionRadius({ asset_type: 'camera' });
    assertClose(r, 30, 0.001, 'Camera vision radius is 30');
})();

(function testSensorVisionRadius() {
    const r = fogGetVisionRadius({ asset_type: 'sensor' });
    assertClose(r, 30, 0.001, 'Sensor vision radius is 30');
})();

(function testPersonVisionRadius() {
    const r = fogGetVisionRadius({ asset_type: 'person' });
    assertClose(r, 15, 0.001, 'Person vision radius is 15');
})();

(function testUnknownTypeGetsDefault() {
    const r = fogGetVisionRadius({ asset_type: 'unknown_thing' });
    assertClose(r, 20, 0.001, 'Unknown type gets default vision radius 20');
})();

(function testVisionRadiusWithCustomOverride() {
    const r = fogGetVisionRadius({ asset_type: 'turret', vision_range: 30 });
    assertClose(r, 30, 0.001, 'Custom vision_range overrides default');
})();

(function testVisionRadiusCaseInsensitive() {
    const r = fogGetVisionRadius({ asset_type: 'Turret' });
    assertClose(r, 50, 0.001, 'Vision radius lookup is case-insensitive');
})();

// ============================================================
// Fog vision map (list of circles)
// ============================================================

console.log('\n--- Fog Vision Map ---');

(function testBuildVisionMapEmpty() {
    const circles = fogBuildVisionMap({});
    assert(Array.isArray(circles), 'Vision map returns array');
    assert(circles.length === 0, 'Empty targets gives empty vision map');
})();

(function testBuildVisionMapFriendlyOnly() {
    const targets = {
        'turret-1': { alliance: 'friendly', asset_type: 'turret', position: { x: 5, y: 10 } },
        'hostile-1': { alliance: 'hostile', asset_type: 'drone', position: { x: -5, y: -5 } },
    };
    const circles = fogBuildVisionMap(targets);
    assert(circles.length === 1, 'Only friendly units contribute to vision map (got ' + circles.length + ')');
    assertClose(circles[0].x, 5, 0.001, 'Vision circle x matches turret position');
    assertClose(circles[0].y, 10, 0.001, 'Vision circle y matches turret position');
    assertClose(circles[0].r, 50, 0.001, 'Vision circle radius matches turret type');
})();

(function testBuildVisionMapMultipleFriendlies() {
    const targets = {
        'turret-1': { alliance: 'friendly', asset_type: 'turret', position: { x: 0, y: 0 } },
        'drone-1': { alliance: 'friendly', asset_type: 'drone', position: { x: 10, y: 10 } },
        'rover-1': { alliance: 'friendly', asset_type: 'rover', position: { x: -5, y: 3 } },
    };
    const circles = fogBuildVisionMap(targets);
    assert(circles.length === 3, 'Three friendlies produce three vision circles');
})();

(function testBuildVisionMapSkipsNeutralized() {
    const targets = {
        'turret-1': { alliance: 'friendly', asset_type: 'turret', position: { x: 0, y: 0 }, status: 'neutralized' },
    };
    const circles = fogBuildVisionMap(targets);
    assert(circles.length === 0, 'Neutralized units do not contribute to vision');
})();

(function testBuildVisionMapUsesXYFallback() {
    const targets = {
        'rover-1': { alliance: 'friendly', asset_type: 'rover', x: 7, y: 8 },
    };
    const circles = fogBuildVisionMap(targets);
    assert(circles.length === 1, 'Unit with x/y (no position obj) is included');
    assertClose(circles[0].x, 7, 0.001, 'X from direct property');
    assertClose(circles[0].y, 8, 0.001, 'Y from direct property');
})();

// ============================================================
// Fog toggle state
// ============================================================

console.log('\n--- Fog Toggle ---');

(function testFogStartsEnabled() {
    assert(fogState.fogEnabled === true, 'Fog starts enabled by default');
})();

(function testMinimapStartsEnabled() {
    assert(fogState.minimapEnabled === true, 'Minimap starts enabled by default');
})();

// ============================================================
// Minimap coordinate mapping
// ============================================================

console.log('\n--- Minimap Coordinates ---');

(function testMinimapWorldCenter() {
    // Map center (0,0) should map to center of minimap
    const p = minimapWorldToMinimap(0, 0, 200, 150, -30, 30);
    assertClose(p.x, 100, 0.5, 'World center x -> minimap center x (100)');
    assertClose(p.y, 75, 0.5, 'World center y -> minimap center y (75)');
})();

(function testMinimapWorldCornerNW() {
    // NW corner: mapMin, mapMax -> minimap (0, 0)
    const p = minimapWorldToMinimap(-30, 30, 200, 150, -30, 30);
    assertClose(p.x, 0, 0.5, 'NW corner x -> minimap left (0)');
    assertClose(p.y, 0, 0.5, 'NW corner y -> minimap top (0)');
})();

(function testMinimapWorldCornerSE() {
    // SE corner: mapMax, mapMin -> minimap (200, 150)
    const p = minimapWorldToMinimap(30, -30, 200, 150, -30, 30);
    assertClose(p.x, 200, 0.5, 'SE corner x -> minimap right (200)');
    assertClose(p.y, 150, 0.5, 'SE corner y -> minimap bottom (150)');
})();

(function testMinimapWorldCornerNE() {
    // NE corner: mapMax, mapMax -> minimap (200, 0)
    const p = minimapWorldToMinimap(30, 30, 200, 150, -30, 30);
    assertClose(p.x, 200, 0.5, 'NE corner x -> minimap right (200)');
    assertClose(p.y, 0, 0.5, 'NE corner y -> minimap top (0)');
})();

(function testMinimapWorldCornerSW() {
    // SW corner: mapMin, mapMin -> minimap (0, 150)
    const p = minimapWorldToMinimap(-30, -30, 200, 150, -30, 30);
    assertClose(p.x, 0, 0.5, 'SW corner x -> minimap left (0)');
    assertClose(p.y, 150, 0.5, 'SW corner y -> minimap bottom (150)');
})();

// ============================================================
// Minimap viewport rectangle
// ============================================================

console.log('\n--- Minimap Viewport Rect ---');

(function testViewportRectCentered() {
    // Camera at origin, zoom 1, canvas 800x600
    const cam = { x: 0, y: 0, zoom: 1 };
    const rect = minimapGetViewportRect(cam, 800, 600, 200, 150, -30, 30);
    // rect should be centered around minimap center
    assert(rect.x < 100 && rect.x + rect.w > 100, 'Viewport rect straddles minimap center x');
    assert(rect.y < 75 && rect.y + rect.h > 75, 'Viewport rect straddles minimap center y');
})();

(function testViewportRectShrinkOnZoomIn() {
    const cam1 = { x: 0, y: 0, zoom: 1 };
    const cam2 = { x: 0, y: 0, zoom: 2 };
    const rect1 = minimapGetViewportRect(cam1, 800, 600, 200, 150, -30, 30);
    const rect2 = minimapGetViewportRect(cam2, 800, 600, 200, 150, -30, 30);
    assert(rect2.w < rect1.w, 'Zooming in shrinks viewport width on minimap');
    assert(rect2.h < rect1.h, 'Zooming in shrinks viewport height on minimap');
})();

(function testViewportRectGrowOnZoomOut() {
    const cam1 = { x: 0, y: 0, zoom: 1 };
    const cam2 = { x: 0, y: 0, zoom: 0.5 };
    const rect1 = minimapGetViewportRect(cam1, 800, 600, 200, 150, -30, 30);
    const rect2 = minimapGetViewportRect(cam2, 800, 600, 200, 150, -30, 30);
    assert(rect2.w > rect1.w, 'Zooming out grows viewport width on minimap');
    assert(rect2.h > rect1.h, 'Zooming out grows viewport height on minimap');
})();

(function testViewportRectMoveWithCamera() {
    const cam1 = { x: 0, y: 0, zoom: 1 };
    const cam2 = { x: 10, y: 5, zoom: 1 };
    const rect1 = minimapGetViewportRect(cam1, 800, 600, 200, 150, -30, 30);
    const rect2 = minimapGetViewportRect(cam2, 800, 600, 200, 150, -30, 30);
    assert(rect2.x > rect1.x, 'Camera panning right moves viewport rect right on minimap');
    assert(rect2.y < rect1.y, 'Camera panning up moves viewport rect up on minimap');
})();

// ============================================================
// Point-in-vision test
// ============================================================

console.log('\n--- Point in Vision ---');

const { fogIsPointVisible } = ctx;

(function testPointInsideVisionCircle() {
    const circles = [{ x: 0, y: 0, r: 10 }];
    assert(fogIsPointVisible(5, 5, circles), 'Point (5,5) is inside vision circle at origin r=10');
})();

(function testPointOutsideVisionCircle() {
    const circles = [{ x: 0, y: 0, r: 10 }];
    assert(!fogIsPointVisible(15, 15, circles), 'Point (15,15) is outside vision circle at origin r=10');
})();

(function testPointInsideOneOfMultipleCircles() {
    const circles = [
        { x: 0, y: 0, r: 5 },
        { x: 20, y: 20, r: 5 },
    ];
    assert(fogIsPointVisible(20, 22, circles), 'Point near second circle is visible');
    assert(!fogIsPointVisible(10, 10, circles), 'Point between both circles is not visible');
})();

(function testPointOnBoundary() {
    const circles = [{ x: 0, y: 0, r: 10 }];
    // Point exactly on boundary should be visible (<=)
    assert(fogIsPointVisible(10, 0, circles), 'Point on boundary is visible');
})();

// ============================================================
// Summary
// ============================================================

console.log('\n' + '='.repeat(40));
console.log(`Results: ${passed} passed, ${failed} failed`);
console.log('='.repeat(40));
process.exit(failed > 0 ? 1 : 0);
