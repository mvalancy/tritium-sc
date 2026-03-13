# Frontend — Vanilla JS Command Center

**Where you are:** `tritium-sc/src/frontend/` — the browser-based Command Center UI.

**Parent:** [../../CLAUDE.md](../../CLAUDE.md) | [../../../CLAUDE.md](../../../CLAUDE.md) (tritium root)

## What This Is

A full-screen tactical command center with real satellite imagery, live unit positions, and AI Commander Amy running continuously. No frameworks — vanilla JavaScript, Canvas 2D, Three.js, and the CYBERCORE CSS design language.

**Read [../../docs/USER-STORIES.md](../../docs/USER-STORIES.md) before writing ANY frontend code.**

## Structure

```
frontend/
├── unified.html              # PRIMARY — Command Center (new)
├── index.html                # LEGACY — Original 10-tab SPA
├── command.html              # Command Center variant
├── js/
│   ├── app.js                # Main app state, WebSocket, keyboard shortcuts
│   ├── war.js                # War Room — Canvas 2D RTS tactical map
│   ├── war3d.js              # War Room — Three.js WebGL 3D renderer
│   ├── amy.js                # Amy panel integration
│   ├── grid.js               # Grid/map rendering
│   ├── input.js              # Input handling (keyboard, gamepad)
│   ├── assets.js             # Asset management
│   ├── models.js             # Data models
│   ├── player.js             # Player state
│   ├── targets.js            # Target tracking
│   ├── scenarios.js          # Scenario management
│   ├── analytics.js          # Analytics tracking
│   ├── geo.js                # Geospatial utilities
│   ├── zones.js              # Zone management
│   ├── zone-editor.js        # Zone editing UI
│   ├── war-audio.js          # War Room audio
│   ├── war-combat.js         # War Room combat rendering
│   ├── war-editor.js         # War Room map editor
│   ├── war-events.js         # War Room event handling
│   ├── war-fx.js             # War Room visual effects
│   ├── war-fog.js            # Fog of war
│   ├── war-hud.js            # War Room HUD overlay
│   ├── command/              # Command Center modules
│   │   ├── main.js           # Entry point, view orchestration
│   │   ├── store.js          # Centralized state store
│   │   ├── events.js         # Event system
│   │   ├── websocket.js      # WebSocket client
│   │   ├── menu-bar.js       # Top menu bar
│   │   ├── command-bar.js    # Command input bar
│   │   ├── layout-manager.js # Panel layout management
│   │   ├── panel-manager.js  # Panel lifecycle
│   │   ├── context-menu.js   # Right-click menus
│   │   ├── map.js            # 2D map (Canvas)
│   │   ├── map3d.js          # 3D map (Three.js)
│   │   ├── map-maplibre.js   # MapLibre GL integration
│   │   ├── mesh-layer.js     # Mesh network overlay
│   │   ├── unit-icons.js     # Unit type icons
│   │   ├── vision-system.js  # AI vision overlay
│   │   ├── label-collision.js # Label placement
│   │   ├── device-modal.js   # Device detail modal
│   │   ├── mission-modal.js  # Mission detail modal
│   │   ├── game-over-stats.js # End-of-battle stats
│   │   ├── panels/           # 20+ floating panels
│   │   └── unit-types/       # Unit type rendering
│   └── common/               # Shared JS (submodule → games-common)
└── css/
    ├── cybercore.css          # CYBERCORE design language
    └── tritium.css            # Custom styles + Amy + War Room panels
```

## Design Language

**CYBERCORE** — cyberpunk aesthetic applied consistently:

| Token | Value | Use |
|-------|-------|-----|
| Cyan | `#00f0ff` | Primary accent, links, active states |
| Magenta | `#ff2a6d` | Alerts, hostiles, danger |
| Green | `#05ffa1` | Success, friendlies, health |
| Yellow | `#fcee0a` | Warnings, caution |
| Void | `#0a0a0f` | Background |
| Surface 1 | `#0e0e14` | Cards, panels |
| Surface 2 | `#12121a` | Elevated surfaces |
| Surface 3 | `#1a1a2e` | Modal backgrounds |

## Key Patterns

1. **No frameworks** — vanilla JavaScript only. No React, Vue, Angular.
2. **Modular JS** — each view has its own file with clear exports.
3. **Keyboard + gamepad** — every interaction is accessible via keyboard and gamepad.
4. **Canvas 2D + Three.js** — dual rendering: 2D tactical map and 3D WebGL view.
5. **WebSocket** — real-time updates via `WS /ws/live`.
6. **MJPEG layout rules** — see CLAUDE.md for the 3-layer fix (view-content flex, overflow clip, absolute img).

## Keyboard Shortcuts

Press `?` in the UI for full list. Key bindings:
- `B` — Begin 10-wave battle
- `O/T/S` — Map modes (Observe, Tactical, Setup)
- `F` — Center camera on action
- `V` — Toggle synthetic camera PIP
- `M` — Mute/unmute audio
- `ESC` — Close modals

## Testing

```bash
# JS tests (fast)
./test.sh 3

# Visual regression (requires running server)
./test.sh 10

# Full visual E2E
./test.sh 7
```

## Related

- [../../docs/USER-STORIES.md](../../docs/USER-STORIES.md) — UX specs (THE source of truth)
- [../../docs/CONTROLS.md](../../docs/CONTROLS.md) — Control reference
- [../../docs/GAMEPAD.md](../../docs/GAMEPAD.md) — Gamepad mapping
- [../../docs/UI-VIEWS.md](../../docs/UI-VIEWS.md) — Panel design intent
- [../../docs/UI-TESTING.md](../../docs/UI-TESTING.md) — Visual regression testing
- [../app/](../app/) — FastAPI backend serving these files
- [../engine/](../engine/) — Engine the frontend connects to via WebSocket
