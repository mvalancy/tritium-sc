# Engine: Actions

Lua-based action dispatch system for commanding units, robots, and assets.

## Files

| File | Purpose |
|------|---------|
| `lua_motor.py` | Lua parser and VALID_ACTIONS registry for Amy and robots |
| `lua_multi.py` | Multi-action sequence execution (chain actions) |
| `lua_registry.py` | Dynamic Lua action registry for extensible commands |
| `formation_actions.py` | Squad formation actions (line, wedge, column, diamond) |

## Usage

Actions are dispatched via Lua-format strings parsed by `lua_motor.py`.
The registry pattern allows plugins and robots to register custom actions
at runtime via `lua_registry.py`.
