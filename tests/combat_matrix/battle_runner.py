# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""Run a single battle, collect all telemetry into BattleMetrics."""

from __future__ import annotations

import json
import logging
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import requests

from tests.combat_matrix.config_matrix import BattleConfig
from tests.combat_matrix.metrics import BattleMetrics, UnitSnapshot
from tests.combat_matrix.scenario_factory import write_scenario

logger = logging.getLogger(__name__)

RESULTS_DIR = Path(__file__).resolve().parents[1] / ".test-results" / "combat-matrix"
POLL_INTERVAL = 2.0  # seconds between target polls
MAX_BATTLE_TIME = 180.0  # timeout per battle (3 min for large force ratios)


class WebSocketCollector:
    """Collects WebSocket events in a background thread."""

    def __init__(self, ws_url: str) -> None:
        self.ws_url = ws_url
        self.projectile_fired = 0
        self.target_eliminated = 0
        self.game_over = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def _classify(self, event_type: str) -> None:
        """Classify a WS event type (amy_-prefixed or raw)."""
        # Server broadcasts with amy_ prefix (e.g. amy_projectile_fired)
        if event_type in ("projectile_fired", "amy_projectile_fired"):
            self.projectile_fired += 1
        elif event_type in ("target_eliminated", "amy_target_eliminated"):
            self.target_eliminated += 1
        elif event_type in ("game_over", "amy_game_over"):
            self.game_over += 1

    def _run(self) -> None:
        try:
            import websocket
            ws = websocket.WebSocket()
            ws.settimeout(1.0)
            ws.connect(self.ws_url)
            while not self._stop.is_set():
                try:
                    raw = ws.recv()
                    if not raw:
                        continue
                    data = json.loads(raw)
                    event_type = data.get("type", "")
                    self._classify(event_type)
                    # Also check telemetry batches
                    if event_type in ("telemetry_batch", "amy_sim_telemetry_batch"):
                        for evt in data.get("data", data.get("events", [])):
                            if isinstance(evt, dict):
                                self._classify(evt.get("type", ""))
                except (TimeoutError, websocket.WebSocketTimeoutException):
                    continue
                except Exception:
                    break
            ws.close()
        except Exception as exc:
            logger.warning("WebSocket collector error: %s", exc)


def _get_targets(base_url: str) -> list[dict]:
    """GET /api/amy/simulation/targets"""
    try:
        resp = requests.get(f"{base_url}/api/amy/simulation/targets", timeout=5)
        resp.raise_for_status()
        data = resp.json()
        if isinstance(data, list):
            return data
        return data.get("targets", [])
    except Exception as exc:
        logger.warning("Failed to get targets: %s", exc)
        return []


def _get_game_state(base_url: str) -> dict:
    """GET /api/game/state"""
    try:
        resp = requests.get(f"{base_url}/api/game/state", timeout=5)
        resp.raise_for_status()
        return resp.json()
    except Exception as exc:
        logger.warning("Failed to get game state: %s", exc)
        return {}


def _get_stats(base_url: str) -> dict:
    """GET /api/game/stats"""
    try:
        resp = requests.get(f"{base_url}/api/game/stats", timeout=5)
        resp.raise_for_status()
        return resp.json()
    except Exception as exc:
        logger.warning("Failed to get stats: %s", exc)
        return {}


def _count_by_alliance(targets: list[dict], include_eliminated: bool = False) -> dict[str, int]:
    """Count targets by alliance.

    Args:
        include_eliminated: If True, count eliminated/destroyed units too.
            Useful for initial hostile count where fast kills can eliminate
            units before the next poll.
    """
    counts: dict[str, int] = {"friendly": 0, "hostile": 0, "neutral": 0}
    for t in targets:
        alliance = t.get("alliance", "neutral")
        status = t.get("status", "")
        if not include_eliminated and status in ("eliminated", "destroyed"):
            continue
        counts[alliance] = counts.get(alliance, 0) + 1
    return counts


def _take_screenshot(page, config_id: str, index: int) -> str | None:
    """Capture a screenshot and return path."""
    try:
        RESULTS_DIR.mkdir(parents=True, exist_ok=True)
        path = RESULTS_DIR / f"ss_{config_id}_{index:03d}.png"
        page.screenshot(path=str(path))
        return str(path)
    except Exception as exc:
        logger.warning("Screenshot failed: %s", exc)
        return None


def _analyze_screenshot(path: str) -> dict:
    """OpenCV analysis: green/red blobs, bright FX, motion delta."""
    try:
        import cv2
        img = cv2.imread(path)
        if img is None:
            return {}
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Green blobs (friendlies): H=35-85, S>50, V>50
        green_mask = cv2.inRange(hsv, (35, 50, 50), (85, 255, 255))
        green_pixels = int(cv2.countNonZero(green_mask))

        # Red blobs (hostiles): H=0-10 or H=170-180, S>50, V>50
        red_low = cv2.inRange(hsv, (0, 50, 50), (10, 255, 255))
        red_high = cv2.inRange(hsv, (170, 50, 50), (180, 255, 255))
        red_pixels = int(cv2.countNonZero(red_low) + cv2.countNonZero(red_high))

        # Bright FX: V > 240 and S < 50 (white-ish bright pixels)
        bright_mask = cv2.inRange(hsv, (0, 0, 240), (180, 50, 255))
        bright_pixels = int(cv2.countNonZero(bright_mask))

        return {
            "green_pixels": green_pixels,
            "red_pixels": red_pixels,
            "bright_pixels": bright_pixels,
        }
    except ImportError:
        return {}
    except Exception as exc:
        logger.warning("OpenCV analysis failed: %s", exc)
        return {}


def _check_frame_motion(path_a: str, path_b: str) -> bool:
    """Compare two screenshots for motion."""
    try:
        import cv2
        img_a = cv2.imread(path_a, cv2.IMREAD_GRAYSCALE)
        img_b = cv2.imread(path_b, cv2.IMREAD_GRAYSCALE)
        if img_a is None or img_b is None:
            return False
        if img_a.shape != img_b.shape:
            return False
        diff = cv2.absdiff(img_a, img_b)
        return int(cv2.countNonZero(diff > 30)) > 100
    except ImportError:
        return False
    except Exception:
        return False


def _fetch_audio_rms(base_url: str) -> float:
    """Fetch a combat sound effect WAV and compute RMS."""
    try:
        # Try nerf_fire or projectile_impact
        for name in ("nerf_fire", "projectile_impact", "turret_fire"):
            try:
                resp = requests.get(
                    f"{base_url}/api/audio/effects/{name}",
                    timeout=5,
                )
                if resp.status_code == 200 and len(resp.content) > 44:
                    # WAV header is 44 bytes, rest is PCM data
                    pcm = np.frombuffer(resp.content[44:], dtype=np.int16)
                    if len(pcm) > 0:
                        rms = float(np.sqrt(np.mean(pcm.astype(np.float64) ** 2)))
                        return rms
            except Exception:
                continue
    except Exception:
        pass
    return 0.0


def run_battle(
    config: BattleConfig,
    base_url: str,
    page: Any = None,
) -> BattleMetrics:
    """Execute a full battle and collect all telemetry.

    Args:
        config: Battle configuration.
        base_url: Server URL (e.g. http://127.0.0.1:8765).
        page: Playwright page (optional, for screenshots + visual analysis).

    Returns:
        BattleMetrics with all collected data.
    """
    metrics = BattleMetrics(config_id=config.config_id)
    metrics.total_ammo_pool = config.total_ammo
    # Scale timeout with battle size: base 180s + 3s per hostile
    battle_timeout = MAX_BATTLE_TIME + config.hostile_count * 3.0
    start_time = time.monotonic()

    try:
        # 1. Write scenario file
        write_scenario(config)
        scenario_name = f"_matrix_{config.config_id}"

        # 2. Reset game
        requests.post(f"{base_url}/api/game/reset", timeout=5)
        time.sleep(0.5)

        # 3. Start WebSocket collector BEFORE battle (captures all events)
        ws_url = base_url.replace("http://", "ws://").replace("https://", "wss://")
        ws_collector = WebSocketCollector(f"{ws_url}/ws/live")
        ws_collector.start()
        time.sleep(0.3)  # Let WS connect

        # 4. Start battle
        resp = requests.post(
            f"{base_url}/api/game/battle/{scenario_name}",
            timeout=10,
        )
        resp.raise_for_status()
        battle_info = resp.json()
        logger.info(
            "Battle started: %s (defenders=%d, waves=%d)",
            config.config_id,
            battle_info.get("defender_count", 0),
            battle_info.get("wave_count", 0),
        )

        # 5. Navigate browser if page available, zoom into action
        if page is not None:
            page.goto(f"{base_url}/", wait_until="networkidle", timeout=15000)
            time.sleep(1)
            # Randomize layer visibility per config for varied screenshots
            import random
            rng = random.Random(config.seed)
            show_sat = rng.choice([True, True, False])  # 2/3 chance satellite
            show_roads = rng.choice([True, False, False])  # 1/3 chance roads
            try:
                page.evaluate(f"""() => {{
                    if (window.store) {{
                        // Zoom to fit map bounds — action fills the screen
                        const bounds = {config.map_bounds};
                        const zoom = Math.max(1, 200 / bounds);
                        window.store.set('camera.zoom', zoom);
                        window.store.set('camera.x', 0);
                        window.store.set('camera.y', 0);
                        // Randomize layers for visual variety
                        window.store.set('showSatellite', {'true' if show_sat else 'false'});
                        window.store.set('showRoads', {'true' if show_roads else 'false'});
                    }}
                }}""")
            except Exception:
                pass  # Non-fatal if zoom/layer toggle fails

        # 6. Wait 1s, check initial friendlies
        time.sleep(1)
        targets = _get_targets(base_url)
        counts = _count_by_alliance(targets)
        metrics.initial_friendly_count = counts["friendly"]

        # 7. Poll for hostile spawn (countdown ~5s + staggered spawn)
        # Track max hostile count including eliminated (fast kills can
        # eliminate hostiles before the next poll interval)
        max_hostile_seen = 0
        spawn_deadline = time.monotonic() + 20.0
        while time.monotonic() < spawn_deadline:
            targets = _get_targets(base_url)
            # Count including eliminated to catch fast kills
            counts_all = _count_by_alliance(targets, include_eliminated=True)
            counts_active = _count_by_alliance(targets, include_eliminated=False)
            if counts_all["hostile"] > max_hostile_seen:
                max_hostile_seen = counts_all["hostile"]
            if max_hostile_seen >= config.hostile_count:
                break
            state = _get_game_state(base_url)
            if state.get("state") in ("victory", "defeat"):
                break
            time.sleep(0.5)

        # Final count: use max seen (including eliminated) for initial count
        targets = _get_targets(base_url)
        counts_all = _count_by_alliance(targets, include_eliminated=True)
        counts_active = _count_by_alliance(targets, include_eliminated=False)
        if counts_all["hostile"] > max_hostile_seen:
            max_hostile_seen = counts_all["hostile"]
        metrics.initial_hostile_count = max_hostile_seen
        metrics.max_total_units = counts_active["friendly"] + max_hostile_seen
        metrics.neutral_count_max = counts_active["neutral"]

        # 8. Poll loop: every 2s until game_over or timeout
        screenshot_idx = 0
        prev_screenshot: str | None = None
        poll_start = time.monotonic()

        while time.monotonic() - poll_start < battle_timeout:
            # Check game state
            state = _get_game_state(base_url)
            game_state = state.get("state", "")
            if game_state in ("victory", "defeat"):
                metrics.game_result = game_state
                metrics.final_score = state.get("score", 0)
                break

            # Poll targets for snapshots
            targets = _get_targets(base_url)
            ts = time.monotonic() - start_time
            counts = _count_by_alliance(targets)
            total_active = counts["friendly"] + counts["hostile"]
            if total_active > metrics.max_total_units:
                metrics.max_total_units = total_active
            if counts["neutral"] > metrics.neutral_count_max:
                metrics.neutral_count_max = counts["neutral"]

            # Record snapshots
            for t in targets:
                status = t.get("status", "")
                if status in ("eliminated", "destroyed"):
                    continue
                # Position is {"x": ..., "y": ...} dict from SimulationTarget.to_dict()
                pos = t.get("position", {})
                if isinstance(pos, dict):
                    px, py = pos.get("x", 0), pos.get("y", 0)
                elif isinstance(pos, (list, tuple)) and len(pos) >= 2:
                    px, py = pos[0], pos[1]
                else:
                    px, py = 0, 0
                metrics.snapshots.append(UnitSnapshot(
                    timestamp=ts,
                    target_id=t.get("target_id", t.get("id", "")),
                    name=t.get("name", ""),
                    alliance=t.get("alliance", "neutral"),
                    asset_type=t.get("asset_type", ""),
                    health=t.get("health", 0),
                    ammo_count=t.get("ammo_count", 0),
                    position=(px, py),
                    status=status,
                    fsm_state=t.get("fsm_state", ""),
                ))

            # Screenshot + visual analysis
            if page is not None:
                ss_path = _take_screenshot(page, config.config_id, screenshot_idx)
                if ss_path:
                    metrics.screenshots.append(ss_path)
                    analysis = _analyze_screenshot(ss_path)
                    if analysis.get("green_pixels", 0) > 50:
                        metrics.green_blob_detected = True
                    if analysis.get("red_pixels", 0) > 50:
                        metrics.red_blob_detected = True
                    if analysis.get("bright_pixels", 0) > metrics.bright_fx_max:
                        metrics.bright_fx_max = analysis.get("bright_pixels", 0)
                    if prev_screenshot:
                        if _check_frame_motion(prev_screenshot, ss_path):
                            metrics.frame_motion_detected = True
                    prev_screenshot = ss_path
                    screenshot_idx += 1

            time.sleep(POLL_INTERVAL)
        else:
            # Timeout
            state = _get_game_state(base_url)
            metrics.game_result = state.get("state", "timeout")
            metrics.final_score = state.get("score", 0)

        # 9. Collect after-action stats
        stats = _get_stats(base_url)
        summary = stats.get("summary", {})
        metrics.total_shots_fired = summary.get("total_shots_fired", 0)
        metrics.total_shots_hit = summary.get("total_shots_hit", 0)
        metrics.total_eliminations = summary.get("total_kills", 0)
        metrics.total_damage_dealt = summary.get("total_damage_dealt", 0.0)
        metrics.unit_stats = stats.get("units", [])

        # 10. Stop WS collector
        ws_collector.stop()
        metrics.ws_projectile_fired = ws_collector.projectile_fired
        metrics.ws_target_eliminated = ws_collector.target_eliminated
        metrics.ws_game_over = ws_collector.game_over

        # 11. Audio RMS
        metrics.audio_rms = _fetch_audio_rms(base_url)

    except Exception as exc:
        metrics.errors.append(f"Battle execution error: {exc}")
        logger.exception("Battle %s failed", config.config_id)

    metrics.battle_duration = time.monotonic() - start_time
    return metrics
