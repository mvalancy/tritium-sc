# Simulation Engine

**Where you are:** `tritium-sc/src/engine/simulation/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The battle simulation engine runs 10Hz tick-based combat simulations with projectile physics, unit AI, wave-based game progression, and Smash TV-style commentary. It exercises the exact same pipelines (target tracking, threat classification, dispatch) that operate in real-world mode. The simulation is the system's continuous integration test — if the battle works, the real sensors work.

This is the largest subsystem in tritium-sc with 50+ files covering combat mechanics, pathfinding, AI behaviors, terrain, squads, morale, inventory, and spectator modes.

## Key Files

| File | Purpose |
|------|---------|
| `engine.py` | SimulationEngine — 10Hz tick loop, hostile spawner, main orchestrator |
| `target.py` | SimulationTarget dataclass — position, health, alliance, type |
| `combat.py` | CombatSystem — projectile flight, hit detection, damage calculation |
| `game_mode.py` | GameMode — wave-based game progression, difficulty scaling |
| `behaviors.py` | UnitBehaviors — turret, drone, rover, hostile AI decision trees |
| `pathfinding.py` | A* pathfinding on the simulation grid |
| `grid_pathfinder.py` | Grid-based pathfinding with movement profiles per unit type |
| `spatial.py` | SpatialGrid — O(1) spatial queries for collision and proximity |
| `terrain.py` | TerrainCell/TerrainMap — movement costs, cover, elevation |
| `scenario.py` | BattleScenario — wave definitions, spawn groups, defender configs |
| `scenario_gen.py` | Procedural scenario generation from map data |
| `loader.py` | TritiumLevelFormat JSON parser — loads map layouts |
| `state_machine.py` | Generic FSM (State/Transition/StateMachine) used by all units |
| `unit_states.py` | Pre-built FSMs for turret, rover, drone, hostile unit types |
| `squads.py` | Squad/SquadManager — group unit coordination |
| `combat_bridge.py` | CombatBridge — connects combat events to the event bus |
| `battle_integration.py` | AutomationEngine — if-then rules triggered by combat events |
| `hostile_commander.py` | AI commander for hostile forces |
| `mission_director.py` | High-level mission flow and objective sequencing |

## Subdirectories

| Directory | Purpose |
|-----------|---------|
| `behavior/` | Modular unit behavior implementations (turret, drone, rover, hostile, coordinator) |
| `npc_intelligence/` | NPC AI system — brain, FSM, thought scheduling, world model, crowd simulation |

## Other Files

| File | Purpose |
|------|---------|
| `ambient.py` | AmbientSpawner — neutral civilian activity during peacetime |
| `backstory.py` | BackstoryGenerator — procedural character backstories |
| `cover.py` | CoverObject/CoverSystem — cover mechanics for combat |
| `crowd_density.py` | Crowd density simulation and heat tracking |
| `degradation.py` | Equipment degradation over time |
| `difficulty.py` | DifficultyScaler — adaptive difficulty based on performance |
| `fake_robot.py` | FakeRobot/FakeRobotFleet — simulated robots for testing |
| `hazards.py` | Hazard/HazardManager — environmental hazards |
| `intercept.py` | Lead target calculation and intercept prediction |
| `inventory.py` | InventoryItem/UnitInventory — loadouts and weapon selection |
| `lod.py` | LODSystem — level-of-detail for performance optimization |
| `morale.py` | MoraleSystem — unit morale affects behavior |
| `movement.py` | MovementController — smooth unit movement with acceleration |
| `objectives.py` | ObjectiveTracker — mission objectives and scoring |
| `poi_data.py` | POI/MissionArea — OSM points of interest for scenario placement |
| `pursuit.py` | PursuitSystem — chase behavior logic |
| `replay.py` | ReplayRecorder — record and replay battle sessions |
| `sensors.py` | SensorDevice/SensorSimulator — simulated sensor readings |
| `spectator.py` | SpectatorMode — cinematic camera for watching battles |
| `stats.py` | StatsTracker — kill counts, accuracy, wave performance |
| `swarm.py` | Swarm coordination for drone groups |
| `upgrades.py` | Upgrade/Ability — unit upgrade and ability system |
| `vision.py` | VisionSystem — line-of-sight and visibility |
| `weapons.py` | Weapon/WeaponSystem — weapon definitions and ballistics |

## Related

- [../comms/event_bus.py](../comms/event_bus.py) — Event bus that simulation publishes tick events to
- [../tactical/target_tracker.py](../tactical/target_tracker.py) — Target tracker that receives simulation targets
- [../../amy/actions/announcer.py](../../amy/actions/announcer.py) — War commentary driven by combat events
- [../../app/routers/](../../app/routers/) — API endpoints for simulation control
- [../../../plugins/graphlings/](../../../plugins/graphlings/) — NPC digital life plugin
- [../../frontend/js/war.js](../../frontend/js/war.js) — Canvas 2D tactical map renderer
- [../../frontend/js/war3d.js](../../frontend/js/war3d.js) — Three.js WebGL 3D renderer
