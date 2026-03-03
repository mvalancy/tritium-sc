# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Grid-based A* pathfinder with per-unit-type movement profiles.

Routes units on the TerrainMap's cell grid instead of the OSM street graph.
This ensures ground vehicles never drive through buildings and heavy vehicles
stay on roads, even when no StreetGraph is loaded.

Movement profiles:
    pedestrian:    Roads fast, yards/open normal, buildings expensive (can enter), water impassable
    light_vehicle: Roads fast, yards/open passable, buildings impassable
    heavy_vehicle: Roads only — everything else impassable
    aerial:        Low cost everywhere, slight preference to avoid buildings

The A* uses 8-directional neighbors (cardinal + diagonal) on the TerrainMap grid.
Diagonal moves cost sqrt(2) x cell cost.  Heuristic is Euclidean distance.
A circuit breaker (max_iterations) prevents unbounded search on large maps.

After A*, smooth_path() removes redundant collinear waypoints so the unit
doesn't zigzag through every grid cell.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from .terrain import TerrainMap
    from engine.tactical.obstacles import BuildingObstacles

# ---------------------------------------------------------------------------
# Movement profiles
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MovementProfile:
    """Cost multipliers for each terrain type."""
    road: float       # cost on road cells
    yard: float       # cost on yard/park cells
    open_: float      # cost on open terrain
    building: float   # cost through buildings (inf = impassable)
    water: float      # cost through water (inf = impassable)


PROFILES: dict[str, MovementProfile] = {
    "pedestrian":    MovementProfile(road=0.7, yard=1.0, open_=1.0, building=25.0, water=999.0),
    "light_vehicle": MovementProfile(road=0.7, yard=1.5, open_=2.0, building=999.0, water=999.0),
    "heavy_vehicle": MovementProfile(road=0.7, yard=999.0, open_=999.0, building=999.0, water=999.0),
    "aerial":        MovementProfile(road=1.0, yard=1.0, open_=1.0, building=5.0, water=1.0),
    "graphling":     MovementProfile(road=0.8, yard=1.0, open_=1.0, building=999.0, water=999.0),
}

# Terrain type -> profile field mapping
_TERRAIN_TO_FIELD = {
    "road": "road",
    "yard": "yard",
    "open": "open_",
    "building": "building",
    "water": "water",
}

# 8-directional neighbor offsets (dx, dy, distance_multiplier)
_NEIGHBORS = [
    (-1,  0, 1.0),      # west
    ( 1,  0, 1.0),      # east
    ( 0, -1, 1.0),      # south
    ( 0,  1, 1.0),      # north
    (-1, -1, 1.4142),   # SW
    ( 1, -1, 1.4142),   # SE
    (-1,  1, 1.4142),   # NW
    ( 1,  1, 1.4142),   # NE
]


# ---------------------------------------------------------------------------
# Profile selection
# ---------------------------------------------------------------------------

def profile_for_unit(asset_type: str, alliance: str = "friendly") -> str:
    """Map a unit's asset_type + alliance to a movement profile name.

    Uses the unit type registry's MovementCategory when available,
    falls back to hardcoded mapping for common types.
    """
    try:
        from engine.units import get_type as _get_unit_type
        from engine.units.base import MovementCategory
        utype = _get_unit_type(asset_type)
        if utype is not None:
            cat = utype.category
            if cat is MovementCategory.AIR:
                return "aerial"
            if cat is MovementCategory.STATIONARY:
                return "pedestrian"  # won't actually pathfind, but safe default
            if cat is MovementCategory.GROUND:
                # Heavy vehicle: only tank
                if asset_type == "tank":
                    return "heavy_vehicle"
                return "light_vehicle"
            if cat is MovementCategory.FOOT:
                return "pedestrian"
    except Exception:
        pass

    # Fallback for unregistered types
    _FALLBACK = {
        "drone": "aerial",
        "scout_drone": "aerial",
        "swarm_drone": "aerial",
        "rover": "light_vehicle",
        "apc": "light_vehicle",
        "tank": "heavy_vehicle",
        "vehicle": "light_vehicle",
        "hostile_vehicle": "light_vehicle",
        "person": "pedestrian",
        "animal": "pedestrian",
        "graphling": "graphling",
    }
    return _FALLBACK.get(asset_type, "pedestrian")


# ---------------------------------------------------------------------------
# Cost function
# ---------------------------------------------------------------------------

def _cell_cost(terrain_map: TerrainMap, col: int, row: int, profile: MovementProfile) -> float:
    """Get the movement cost for a cell under the given profile.

    Returns float("inf") for impassable cells.
    """
    terrain_type = terrain_map.get_terrain_at(col, row)
    if terrain_type == "out_of_bounds":
        return float("inf")
    field_name = _TERRAIN_TO_FIELD.get(terrain_type, "open_")
    return getattr(profile, field_name)


# ---------------------------------------------------------------------------
# A* pathfinder
# ---------------------------------------------------------------------------

def grid_find_path(
    terrain_map: TerrainMap,
    start: tuple[float, float],
    end: tuple[float, float],
    profile_name: str,
    max_iterations: int = 2000,
    obstacles: Optional[BuildingObstacles] = None,
) -> Optional[list[tuple[float, float]]]:
    """Find a path on the terrain grid using A* with the given movement profile.

    Args:
        terrain_map: The TerrainMap with cell data.
        start: (x, y) world-coordinate start position.
        end: (x, y) world-coordinate end position.
        profile_name: Key into PROFILES (or a profile name from profile_for_unit).
        max_iterations: Circuit breaker to prevent unbounded search.
        obstacles: Optional BuildingObstacles for post-smoothing validation.
            When provided, the smoothed path is checked against building
            polygons.  If smoothing introduced a building crossing, the
            unsmoothed (grid-cell) path is returned instead.

    Returns:
        List of (x, y) world-coordinate waypoints, or None if no path found.
    """
    profile = PROFILES.get(profile_name)
    if profile is None:
        profile = PROFILES["pedestrian"]

    # Convert world -> grid
    start_col, start_row = terrain_map._world_to_grid(start[0], start[1])
    end_col, end_row = terrain_map._world_to_grid(end[0], end[1])

    # Clamp to grid bounds
    gs = terrain_map.grid_size
    start_col = max(0, min(gs - 1, start_col))
    start_row = max(0, min(gs - 1, start_row))
    end_col = max(0, min(gs - 1, end_col))
    end_row = max(0, min(gs - 1, end_row))

    # Same cell — trivial
    if start_col == end_col and start_row == end_row:
        wx, wy = terrain_map._grid_to_world(start_col, start_row)
        return [(wx, wy)]

    # Check if destination cell is reachable for this profile
    dest_cost = _cell_cost(terrain_map, end_col, end_row, profile)
    if dest_cost >= 999.0:
        return None

    # A* open set: (f_score, counter, col, row)
    counter = 0
    open_set: list[tuple[float, int, int, int]] = []
    g_score: dict[tuple[int, int], float] = {}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    start_node = (start_col, start_row)
    end_node = (end_col, end_row)

    g_score[start_node] = 0.0
    h = math.hypot(end_col - start_col, end_row - start_row)
    heapq.heappush(open_set, (h, counter, start_col, start_row))
    counter += 1

    iterations = 0

    while open_set and iterations < max_iterations:
        iterations += 1
        f, _, col, row = heapq.heappop(open_set)
        node = (col, row)

        if node == end_node:
            # Reconstruct path
            path_grid = _reconstruct(came_from, end_node)
            path_world = [terrain_map._grid_to_world(c, r) for c, r in path_grid]
            smoothed = smooth_path(path_world)
            # Validate smoothed path against building polygons.
            # Smoothing removes intermediate waypoints, which can create
            # line segments that cut through buildings even though the
            # original grid path went around them.  When that happens,
            # fall back to the unsmoothed grid-cell path.
            if obstacles is not None and obstacles.path_crosses_building(smoothed):
                return path_world
            return smoothed

        current_g = g_score.get(node, float("inf"))

        for dx, dy, dist_mult in _NEIGHBORS:
            nc, nr = col + dx, row + dy
            neighbor = (nc, nr)

            cell_c = _cell_cost(terrain_map, nc, nr, profile)
            if cell_c >= 999.0:
                continue

            tentative_g = current_g + cell_c * dist_mult

            if tentative_g < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = node
                h = math.hypot(end_col - nc, end_row - nr)
                heapq.heappush(open_set, (tentative_g + h, counter, nc, nr))
                counter += 1

    # Budget exhausted or no path
    return None


def _reconstruct(
    came_from: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
) -> list[tuple[int, int]]:
    """Walk came_from links backward to build the path."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Path smoothing
# ---------------------------------------------------------------------------

def smooth_path(path: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Remove collinear intermediate waypoints from a grid path.

    Keeps start, end, and any point where the direction changes.
    """
    if len(path) <= 2:
        return list(path)

    result = [path[0]]
    for i in range(1, len(path) - 1):
        # Check if point i is collinear with i-1 and i+1
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        # Cross product of vectors (i-1 -> i) and (i -> i+1)
        cross = (x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)
        if abs(cross) > 1e-6:
            result.append(path[i])
    result.append(path[-1])
    return result
