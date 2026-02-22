# TRITIUM-SC Window Manager & Plugin Architecture

**Status**: SPEC — Ready for implementation
**Author**: Claude Code + Human Direction
**Date**: 2026-02-21

## 1. Vision

The Command Center becomes a **composable workspace** where operators build custom
views from pluggable panels: 3D world views, camera streams, status dashboards,
Amy's chat, game HUD — all snappable, resizable, tabbable, and saveable.

Think Blender's workspace system: switch between "Layout", "Modeling", "Animation"
with one click. Each workspace remembers which panels are open, where they're docked,
and their individual configuration. The operator saves their own workspaces too.

**Core requirement**: Layout save/recall is a **first-class feature**, not a nice-to-have.
Operators must be able to:
1. Arrange panels into a custom view
2. Save that arrangement with a name
3. Recall it instantly with a keyboard shortcut or menu selection
4. Ship default layouts that match common scenarios (commander, observer, tactical, battle)

## 2. Library Choice: dockview-core

After evaluating 12 libraries (golden-layout, flexlayout-react, lumino, winbox.js,
dock-spawn-ts, rc-dock, allotment, react-mosaic, w2ui, jsPanel4, floating-ui, dockview),
**dockview-core** is the recommended choice.

| Criterion | dockview-core | Why it wins |
|-----------|--------------|-------------|
| Framework | Zero dependencies | We use vanilla JS, no React/Vue |
| Maintenance | v5.0.0 (Feb 2026) | Actively maintained, 5 days old |
| Layout serialization | `toJSON()`/`fromJSON()` | Built-in, no custom code |
| Docking | Tabs, splits, grids, floating, popout | Full IDE-style layout |
| Theming | CSS variables | Trivial to map to cybercore-v2 |
| Stars | 3,016 | Healthy community |
| License | MIT | No restrictions |
| CDN | jsDelivr ESM | No build step needed |

**Runner-up**: dock-spawn-ts (truly vanilla, but 144 stars, weaker serialization).
**Eliminated**: golden-layout (3+ years stale), all React-only libs, lumino (too heavy).

### Loading Strategy

```html
<!-- In unified.html, loaded as ES module from jsDelivr CDN -->
<script type="importmap">
{
  "imports": {
    "dockview-core": "https://cdn.jsdelivr.net/npm/dockview-core@5/+esm"
  }
}
</script>
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/dockview-core@5/dist/styles/dockview.css">
```

### Fallback

If dockview-core proves problematic, the existing `PanelManager` (panel-manager.js)
already handles floating panels with drag, resize, snap, minimize, and localStorage
persistence. The plugin API (Section 4) is designed to work with either backend.

## 3. Three.js Multi-Viewport Architecture

### Strategy: Virtual Display List (Single Canvas + Scissor)

One full-page `<canvas>` behind all panels, one `WebGLRenderer`, one WebGL context.
Each 3D panel contains a transparent placeholder `<div>`. The render loop reads
placeholder positions and renders into those regions using viewport/scissor.

**Why not multiple canvases?**
- Browser limits: 8-16 WebGL contexts per page
- GPU memory: each context duplicates framebuffers, shaders, textures
- Performance: context switching overhead

**Why not OffscreenCanvas?**
- Complexity: Worker message passing for camera controls
- Browser support: inconsistent (Safari edge cases)
- Not worth it for 1-4 viewports

### ViewportRegistry

```javascript
// frontend/js/command/viewport-registry.js

export class ViewportRegistry {
    constructor(canvas) {
        this.canvas = canvas;
        this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.autoClear = false;
        this._viewports = new Map();  // panelId -> { scene, camera, element, controls }
    }

    register(panelId, scene, camera, placeholderEl) {
        this._viewports.set(panelId, { scene, camera, element: placeholderEl });
    }

    unregister(panelId) {
        this._viewports.delete(panelId);
    }

    renderAll() {
        this.renderer.setScissorTest(false);
        this.renderer.clear(true, true);
        this.renderer.setScissorTest(true);

        const dpr = window.devicePixelRatio || 1;
        const canvasH = this.canvas.clientHeight;

        for (const [id, vp] of this._viewports) {
            const rect = vp.element.getBoundingClientRect();
            if (rect.width === 0 || rect.height === 0) continue;

            // Cull offscreen
            if (rect.bottom < 0 || rect.top > canvasH) continue;

            const bottom = canvasH - rect.bottom;
            this.renderer.setViewport(
                rect.left * dpr, bottom * dpr,
                rect.width * dpr, rect.height * dpr
            );
            this.renderer.setScissor(
                rect.left * dpr, bottom * dpr,
                rect.width * dpr, rect.height * dpr
            );

            // Update camera aspect
            const aspect = rect.width / rect.height;
            if (vp.camera.isPerspectiveCamera) {
                vp.camera.aspect = aspect;
                vp.camera.updateProjectionMatrix();
            } else if (vp.camera.isOrthographicCamera) {
                const halfH = vp.camera.top;
                vp.camera.left = -halfH * aspect;
                vp.camera.right = halfH * aspect;
                vp.camera.updateProjectionMatrix();
            }

            this.renderer.render(vp.scene, vp.camera);
        }
    }
}
```

### Canvas Layering

```
z-index:  0  → #viewport-canvas (full-page, pointer-events: none)
z-index: 10  → #panel-container (dockview or floating panels)
z-index: 50  → #minimap
z-index: 90  → #toast-container
z-index: 99  → #modal-overlay
```

3D viewport panels have transparent backgrounds. The scissor rendering on the canvas
behind them makes the 3D content visible through the panel body.

### Camera Stream Panels

MJPEG and WebRTC streams remain as HTML elements (`<img>` / `<video>`), NOT drawn
to the Three.js canvas. This avoids texture upload overhead and layout reflow.

```css
.camera-panel-content {
    position: relative;
    width: 100%; height: 100%;
    overflow: clip;              /* NOT hidden — no scroll container */
    contain: layout style;      /* Isolate reflow */
}
.camera-panel-content img,
.camera-panel-content video {
    position: absolute;
    inset: 0;
    width: 100%; height: 100%;
    object-fit: contain;
    display: block;
}
```

### Render Loop

Single `requestAnimationFrame` coordinates everything:

```javascript
const clock = new THREE.Clock();

function animate() {
    requestAnimationFrame(animate);
    const dt = clock.getDelta();

    // 1. Update simulation state (unit positions, animations)
    updateScenes(dt);

    // 2. Render all 3D viewports via scissor
    viewportRegistry.renderAll();

    // 3. Render Canvas 2D tactical map (if in Canvas 2D mode)
    if (activeMapRenderer === 'canvas2d') {
        renderTacticalMap(dt);
    }

    // 4. Update FPS
    updateFps();
}
```

### Performance Budget (60fps target)

| Component | Budget | Notes |
|-----------|--------|-------|
| Scene update (all viewports) | 2ms | Position updates, animations |
| 3D render (4 viewports, scissor) | 8ms | Shared context, one draw batch |
| DOM reads (`getBoundingClientRect` x4) | 0.5ms | Cheap if layout not dirty |
| MJPEG decode (browser compositor) | 0ms | Off main thread |
| Panel DOM updates (alerts, status) | 1ms | Only on data change |
| **Total** | **~11.5ms** | Within 16.7ms budget |

## 4. Plugin Architecture

### Plugin Definition (Extended Panel Spec)

Every panel is a **plugin**. Each plugin is an ES6 module that exports a
definition object. The definition is the plugin's entire contract with the system.

```javascript
// frontend/js/command/plugins/camera-stream.js

export const CameraStreamPlugin = {
    // --- Identity ---
    id: 'camera-stream',
    title: 'CAMERA',
    icon: 'camera',                    // Icon name for toolbar/menu
    category: 'intel',                 // Grouping: command | intel | game | system

    // --- Layout ---
    defaultSize: { w: 320, h: 240 },
    minSize: { w: 160, h: 120 },
    resizable: true,
    closable: true,
    singleton: false,                  // Multiple instances allowed (one per feed)

    // --- Configuration ---
    config: {
        feedUrl: '',                   // Set per-instance
        objectFit: 'contain',          // 'contain' | 'cover'
    },

    // --- Data Contract ---
    storeKeys: [],                     // Store paths this plugin reads
    events: {
        listens: ['unit:selected'],    // Events consumed
        emits: [],                     // Events produced
    },

    // --- Lifecycle ---
    create(panel) {
        // Return DOM content. Called once when panel is created.
        const container = document.createElement('div');
        container.className = 'camera-panel-content';
        const img = document.createElement('img');
        img.loading = 'lazy';
        container.appendChild(img);
        return container;
    },

    mount(bodyEl, panel) {
        // Wire live data. Called when panel becomes visible.
        const img = bodyEl.querySelector('img');
        if (panel.config.feedUrl) {
            img.src = panel.config.feedUrl;
        }
    },

    unmount(bodyEl) {
        // Clean up. Called when panel is hidden or destroyed.
        const img = bodyEl.querySelector('img');
        if (img) img.src = '';
    },

    onResize(w, h) {
        // Optional: respond to resize
    },
};
```

### Plugin Categories

| Category | Plugins | Description |
|----------|---------|-------------|
| **command** | Amy Commander, Chat | AI interaction and command interface |
| **intel** | Units, Alerts, Camera Stream, Targets | Intelligence and surveillance |
| **map** | Tactical Map (2D), 3D Viewport, Minimap | Spatial views |
| **game** | Game HUD, Kill Feed, Wave Banner, Scoreboard | Combat system |
| **system** | Settings, Audio Mixer, Network Status | System configuration |

### Plugin Registry

```javascript
// frontend/js/command/plugin-registry.js

export class PluginRegistry {
    constructor() {
        this._plugins = new Map();     // id -> plugin definition
        this._categories = new Map();  // category -> [plugin ids]
    }

    register(plugin) {
        if (!plugin.id) throw new Error('Plugin must have an id');
        this._plugins.set(plugin.id, plugin);

        const cat = plugin.category || 'uncategorized';
        if (!this._categories.has(cat)) this._categories.set(cat, []);
        this._categories.get(cat).push(plugin.id);
    }

    get(id) {
        return this._plugins.get(id);
    }

    getByCategory(category) {
        const ids = this._categories.get(category) || [];
        return ids.map(id => this._plugins.get(id));
    }

    all() {
        return [...this._plugins.values()];
    }

    categories() {
        return [...this._categories.keys()];
    }
}
```

### Plugin Discovery

Plugins self-register by being imported. `main.js` imports all plugin modules,
and each module registers with the global registry:

```javascript
// frontend/js/command/main.js

import { PluginRegistry } from './plugin-registry.js';
import { AmyPlugin } from './plugins/amy.js';
import { UnitsPlugin } from './plugins/units.js';
import { AlertsPlugin } from './plugins/alerts.js';
import { GameHudPlugin } from './plugins/game-hud.js';
import { CameraStreamPlugin } from './plugins/camera-stream.js';
import { TacticalMapPlugin } from './plugins/tactical-map.js';
import { Viewport3DPlugin } from './plugins/viewport-3d.js';

const registry = new PluginRegistry();
registry.register(AmyPlugin);
registry.register(UnitsPlugin);
registry.register(AlertsPlugin);
registry.register(GameHudPlugin);
registry.register(CameraStreamPlugin);
registry.register(TacticalMapPlugin);
registry.register(Viewport3DPlugin);
```

Future: dynamic import via `import()` for lazy-loaded plugins.

## 5. Layout System (Core Feature)

### Layout Data Structure

A layout is a complete snapshot of the workspace: which plugins are open, where
they're positioned, their sizes, and their per-instance configuration.

```javascript
// Layout schema
{
    name: 'Commander',                   // Display name
    id: 'commander',                     // Unique identifier
    builtin: true,                       // false for user-created layouts
    created: '2026-02-21T10:00:00Z',
    modified: '2026-02-21T10:00:00Z',

    // dockview serialized state (or PanelManager state)
    dockState: { ... },                  // dockview.toJSON() output

    // Per-panel instance configuration
    panelConfigs: {
        'camera-stream-1': { feedUrl: '/api/amy/nodes/cam1/video' },
        'camera-stream-2': { feedUrl: '/api/amy/nodes/cam2/video' },
        'viewport-3d-1': { cameraPosition: [0, 50, 50], target: [0, 0, 0] },
    },

    // Global view settings
    viewSettings: {
        mapMode: 'tactical',            // observe | tactical | setup
        showMinimap: true,
        showGrid: true,
        audioEnabled: false,
    },
}
```

### Built-in Layouts

| Layout | Description | Panels |
|--------|-------------|--------|
| **Commander** | Full battle command with Amy advisor | Amy, Units, Alerts, Tactical Map, Game HUD |
| **Observer** | Minimal view for watching the battlefield | Tactical Map, Alerts |
| **Tactical** | Intelligence-focused with unit tracking | Tactical Map, Units, Alerts |
| **Battle** | Full combat mode with game HUD prominent | Tactical Map, Game HUD, Kill Feed, Amy |
| **Surveillance** | Multi-camera view | 4x Camera Stream, Alerts, Minimap |
| **Custom 3D** | 3D viewport with camera streams | 3D Viewport, 2x Camera Stream, Units |

### Layout Persistence

```javascript
// frontend/js/command/layout-manager.js

export class LayoutManager {
    constructor(dockManager, registry) {
        this._dock = dockManager;        // dockview or PanelManager
        this._registry = registry;       // PluginRegistry
        this._builtins = new Map();      // built-in layouts
        this._userLayouts = new Map();   // user-created layouts
        this._activeLayoutId = null;

        this._loadBuiltins();
        this._loadUserLayouts();
    }

    // --- Accessors ---

    allLayouts() {
        return [
            ...this._builtins.values(),
            ...this._userLayouts.values(),
        ];
    }

    activeLayout() {
        return this._activeLayoutId;
    }

    // --- Apply ---

    apply(layoutId) {
        const layout = this._builtins.get(layoutId)
                     || this._userLayouts.get(layoutId);
        if (!layout) return false;

        // Restore dockview state
        this._dock.fromJSON(layout.dockState);

        // Apply per-panel configs
        for (const [panelId, config] of Object.entries(layout.panelConfigs || {})) {
            const panel = this._dock.getPanel(panelId);
            if (panel) panel.applyConfig(config);
        }

        // Apply view settings
        if (layout.viewSettings) {
            this._applyViewSettings(layout.viewSettings);
        }

        this._activeLayoutId = layoutId;
        this._saveActiveLayoutId();
        return true;
    }

    // --- Save ---

    saveCurrent(name) {
        const id = name.toLowerCase().replace(/\s+/g, '-');
        const layout = {
            name,
            id,
            builtin: false,
            created: new Date().toISOString(),
            modified: new Date().toISOString(),
            dockState: this._dock.toJSON(),
            panelConfigs: this._collectPanelConfigs(),
            viewSettings: this._collectViewSettings(),
        };

        this._userLayouts.set(id, layout);
        this._persistUserLayouts();
        return layout;
    }

    // --- Delete ---

    deleteLayout(id) {
        if (this._builtins.has(id)) return false;  // Can't delete builtins
        this._userLayouts.delete(id);
        this._persistUserLayouts();
        return true;
    }

    // --- Export/Import ---

    exportLayout(id) {
        const layout = this._builtins.get(id) || this._userLayouts.get(id);
        return layout ? JSON.stringify(layout, null, 2) : null;
    }

    importLayout(json) {
        const layout = JSON.parse(json);
        layout.builtin = false;
        layout.id = layout.id || layout.name.toLowerCase().replace(/\s+/g, '-');
        this._userLayouts.set(layout.id, layout);
        this._persistUserLayouts();
        return layout;
    }

    // --- Persistence ---

    _persistUserLayouts() {
        const data = Object.fromEntries(this._userLayouts);
        localStorage.setItem('tritium-user-layouts', JSON.stringify(data));
    }

    _loadUserLayouts() {
        try {
            const raw = localStorage.getItem('tritium-user-layouts');
            if (raw) {
                const data = JSON.parse(raw);
                for (const [id, layout] of Object.entries(data)) {
                    this._userLayouts.set(id, layout);
                }
            }
        } catch (_) {}
    }

    _saveActiveLayoutId() {
        localStorage.setItem('tritium-active-layout', this._activeLayoutId);
    }

    _loadActiveLayoutId() {
        return localStorage.getItem('tritium-active-layout') || 'commander';
    }
}
```

### Layout Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Ctrl+1` through `Ctrl+9` | Switch to layout 1-9 |
| `Ctrl+Shift+S` | Save current layout (prompt for name) |
| `Ctrl+Shift+L` | Open layout selector |

### Layout Selector UI

A modal or dropdown showing all layouts (built-in + user-created) with:
- Thumbnail preview (optional — screenshot on save)
- Name and description
- Last modified date
- Edit/Delete buttons for user layouts
- Keyboard shortcut badge (Ctrl+N)

## 6. Panel Types — Full Plugin List

### Phase 1 (Minimum Viable)

These plugins replace the current hardcoded panels:

| Plugin | Replaces | Config |
|--------|----------|--------|
| `amy` | Amy Commander panel | — |
| `units` | Units panel | — |
| `alerts` | Alerts panel | — |
| `game-hud` | Game HUD panel | — |
| `chat` | Slide-out chat overlay | — |
| `tactical-map` | Canvas 2D tactical map | mode: observe/tactical/setup |
| `toast` | Toast notification service | position: top-right/top-left |

### Phase 2 (Full Feature)

| Plugin | Description | Multi-instance |
|--------|-------------|---------------|
| `camera-stream` | MJPEG / WebRTC camera feed | Yes (one per feed) |
| `viewport-3d` | Three.js 3D world view | Yes (1-4) |
| `kill-feed` | Combat kill notifications | No |
| `scoreboard` | Game score and wave info | No |
| `audio-mixer` | Sound effect controls | No |
| `minimap` | Tactical minimap overlay | No |
| `unit-detail` | Selected unit inspection | No |
| `analytics` | Performance metrics | No |
| `scenario-runner` | Behavioral test controls | No |

### Phase 3 (Advanced)

| Plugin | Description |
|--------|-------------|
| `video-player` | Recorded footage playback |
| `timeline` | Event timeline with scrubbing |
| `network-graph` | Device mesh visualization |
| `mqtt-inspector` | Live MQTT message viewer |
| `layout-editor` | Visual layout configuration |

## 7. Dockview Integration

### Theme Mapping (Cybercore v2)

```css
/* Override dockview's CSS variables with cybercore-v2 values */
.dv-dockview {
    --dv-background-color: #0a0a0f;                    /* --void */
    --dv-tabs-and-actions-container-background-color: #111118;  /* --surface-1 */
    --dv-activegroup-visiblepanel-tab-background-color: #191922; /* --surface-2 */
    --dv-activegroup-hiddenpanel-tab-background-color: #111118;  /* --surface-1 */
    --dv-inactivegroup-visiblepanel-tab-background-color: #111118;
    --dv-inactivegroup-hiddenpanel-tab-background-color: #0a0a0f;
    --dv-tab-divider-color: rgba(0, 240, 255, 0.08);  /* --border */
    --dv-tabs-container-scrollbar-color: #00f0ff;      /* --cyan */
    --dv-separator-border: rgba(0, 240, 255, 0.08);
    --dv-paneview-header-border-color: rgba(0, 240, 255, 0.08);

    font-family: 'Inter', system-ui, sans-serif;
}

/* Tab text styling */
.dv-tab .dv-default-tab-content {
    font-family: 'JetBrains Mono', 'Fira Code', monospace;
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.05em;
    color: rgba(255, 255, 255, 0.5);                   /* --text-secondary */
}

/* Active tab accent */
.dv-tab.dv-active-tab .dv-default-tab-content {
    color: #00f0ff;                                     /* --cyan */
}

/* Panel body */
.dv-content-container {
    background: linear-gradient(135deg, #191922, #111118);
    border: 1px solid rgba(0, 240, 255, 0.08);
}

/* Drop target indicators */
.dv-drop-target-dropzone {
    border: 2px solid #00f0ff;
    background: rgba(0, 240, 255, 0.05);
}
```

### Component Factory

dockview uses a `createComponent` callback to build panel content:

```javascript
import { DockviewComponent } from 'dockview-core';

const dock = new DockviewComponent({
    parentElement: document.getElementById('panel-container'),
    createComponent: (options) => {
        const plugin = registry.get(options.name);
        if (!plugin) {
            console.error(`Unknown plugin: ${options.name}`);
            return { element: document.createElement('div'), dispose: () => {} };
        }

        const element = document.createElement('div');
        element.className = `plugin-panel plugin-${plugin.id}`;

        // Create content
        const content = plugin.create({ config: options.params || {} });
        if (content instanceof HTMLElement) {
            element.appendChild(content);
        } else if (typeof content === 'string') {
            element.innerHTML = content;
        }

        return {
            element,
            init: (params) => {
                if (plugin.mount) plugin.mount(element, { config: params.params || {} });
            },
            update: (event) => {
                if (plugin.update) plugin.update(element, event.params);
            },
            dispose: () => {
                if (plugin.unmount) plugin.unmount(element);
            },
        };
    },
});

// Open a panel
dock.addPanel({
    id: 'amy-1',
    component: 'amy',
    title: 'AMY COMMANDER',
});

// Open a camera stream (with config)
dock.addPanel({
    id: 'cam-1',
    component: 'camera-stream',
    title: 'CAMERA 1',
    params: { feedUrl: '/api/amy/nodes/cam1/video' },
});
```

### Serialization

```javascript
// Save layout
const state = dock.toJSON();
localStorage.setItem('tritium-dock-state', JSON.stringify(state));

// Restore layout
const saved = JSON.parse(localStorage.getItem('tritium-dock-state'));
dock.fromJSON(saved);
```

## 8. Migration Path

### Phase 0: Current State (Today)

- `panel-manager.js` handles 4 floating panels
- Canvas 2D tactical map in `map.js`
- Hardcoded chat, toast, banner in `main.js`
- 3 preset layouts + 1 auto-save slot

### Phase 1: Plugin API (Non-Breaking)

**Goal**: Refactor existing panels into plugin format without changing the layout engine.

1. Create `PluginRegistry` class
2. Create `LayoutManager` class with named layout save/recall
3. Refactor Amy/Units/Alerts/GameHud panels into plugin format
4. Extract chat, toast, banner from `main.js` into plugins
5. Add "Save Layout" / "Load Layout" UI
6. Add `Ctrl+1..9` keyboard shortcuts for layout switching

**PanelManager stays as the layout engine.** No dockview yet.
The plugin API is designed to work with both PanelManager and dockview.

### Phase 2: Dockview Integration

**Goal**: Replace PanelManager's positioning with dockview's docking/tabbing.

1. Load dockview-core from CDN
2. Map cybercore-v2 CSS variables to dockview theme
3. Wire `createComponent` callback to PluginRegistry
4. Migrate layout data from PanelManager format to dockview JSON
5. Add tab grouping, split views, dock targets
6. Keep PanelManager code as fallback (feature-detect dockview)

### Phase 3: Three.js Viewports

**Goal**: Multiple 3D world views via scissor rendering.

1. Create `ViewportRegistry` with single full-page canvas
2. Create `Viewport3DPlugin` that registers with ViewportRegistry
3. Single `requestAnimationFrame` loop coordinates all rendering
4. Current `map.js` Canvas 2D continues working alongside (separate canvas)
5. Mouse event forwarding for 3D panel interaction (orbit, select)

### Phase 4: Camera Streams + Multi-Instance

**Goal**: Multiple camera feeds as dockable panels.

1. Create `CameraStreamPlugin` with `singleton: false`
2. Add "Add Camera" button/menu that spawns new instance with feed URL config
3. WebRTC support via `<video>` element in plugin
4. Picture-in-picture support for floating camera windows

## 9. Files to Create/Modify

### New Files

| File | Purpose |
|------|---------|
| `frontend/js/command/plugin-registry.js` | Plugin registration and discovery |
| `frontend/js/command/layout-manager.js` | Named layout save/recall/export/import |
| `frontend/js/command/viewport-registry.js` | Three.js multi-viewport scissor rendering |
| `frontend/js/command/plugins/amy.js` | Amy Commander plugin (refactored from panels/) |
| `frontend/js/command/plugins/units.js` | Units plugin (refactored from panels/) |
| `frontend/js/command/plugins/alerts.js` | Alerts plugin (refactored from panels/) |
| `frontend/js/command/plugins/game-hud.js` | Game HUD plugin (refactored from panels/) |
| `frontend/js/command/plugins/chat.js` | Chat plugin (extracted from main.js) |
| `frontend/js/command/plugins/camera-stream.js` | Camera feed plugin |
| `frontend/js/command/plugins/tactical-map.js` | Canvas 2D tactical map plugin |
| `frontend/js/command/plugins/viewport-3d.js` | Three.js 3D viewport plugin |
| `frontend/css/dockview-cybercore.css` | Dockview theme override for cybercore-v2 |

### Modified Files

| File | Change |
|------|--------|
| `frontend/unified.html` | Add importmap for dockview, viewport canvas, layout selector UI |
| `frontend/js/command/main.js` | Import plugins, create registry, init layout manager |
| `frontend/js/command/panel-manager.js` | Kept as fallback; add LayoutManager compatibility |
| `frontend/css/command.css` | Add viewport canvas layer, layout selector styles |
| `frontend/css/panels.css` | Plugin-specific styles (keep existing, add new) |

### Not Modified

| File | Why |
|------|-----|
| `frontend/js/command/store.js` | Already plugin-friendly (dot-path subscriptions) |
| `frontend/js/command/events.js` | Already plugin-friendly (simple pub/sub) |
| `frontend/js/command/websocket.js` | No changes needed |
| `frontend/js/war.js` | Legacy war room stays separate |

## 10. Testing Strategy

### Unit Tests (JS — test.sh 3)

- `tests/js/test_plugin_registry.js` — register, get, getByCategory, all
- `tests/js/test_layout_manager.js` — save, load, apply, delete, export, import
- `tests/js/test_viewport_registry.js` — register, unregister, renderAll bounds

### Integration Tests (Playwright — test.sh 9/10)

- Layout save/recall round-trip: save layout, reload page, verify restored
- Plugin open/close lifecycle: mount, unmount, re-mount
- Multiple camera streams: open 4 camera panels, verify all receive frames
- 3D viewport: verify canvas renders content at correct position
- Keyboard shortcuts: `Ctrl+1..9` switch layouts

### Visual Tests (test.sh 10/11)

- Screenshot comparison of each built-in layout
- Verify cybercore theming applied to dockview (no default blue/gray)
- Verify no visual regression in tactical map when dockview wraps it

## 11. Open Questions

1. **Canvas 2D tactical map as a plugin**: Currently `map.js` renders to a fixed
   `#tactical-canvas`. Should it become a dockview panel, or stay as a fixed
   background? If fixed background, it's always visible and panels overlay it.
   If a dockview panel, it can be tabbed/hidden/moved — more flexible but
   potentially confusing since "the map is always visible" is a core principle.

   **Recommendation**: The PRIMARY tactical map stays as a fixed background canvas
   (principle: "the map is the game"). Additional 3D viewports are dockview panels.
   This gives the best of both worlds.

2. **dockview CDN reliability**: Should we vendor the library (copy into
   `frontend/vendor/`) or rely on jsDelivr CDN? Vendoring avoids external
   dependency but adds maintenance burden.

   **Recommendation**: Vendor it. Copy the ESM build into `frontend/vendor/dockview/`.
   Update when needed. No external dependency at runtime.

3. **Panel state persistence granularity**: Should per-panel config (camera URLs,
   3D camera positions) persist with the layout or independently?

   **Recommendation**: With the layout. Each saved layout is a complete workspace
   snapshot. Switching layouts switches everything.

## 12. References

- [dockview documentation](https://dockview.dev/)
- [dockview-core on jsDelivr](https://www.jsdelivr.com/package/npm/dockview-core)
- [Three.js Virtual Display List](https://threejsfundamentals.org/threejs/lessons/threejs-multiple-scenes.html)
- [Three.js Scissor/Viewport Example](https://threejs.org/examples/webgl_multiple_views.html)
- [WebGL Context Limits (Chromium)](https://github.com/nicktaylorweb/webgl-context-limits)
- [CSS Containment](https://developer.mozilla.org/en-US/docs/Web/CSS/contain)
- Existing docs: `docs/UI-REDESIGN.md`, `docs/UNIFIED-SPEC.md`, `docs/USER-STORIES.md`
