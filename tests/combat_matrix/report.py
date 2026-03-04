# Created by Matthew Valancy
# Copyright 2026 Valpatel Software LLC
# Licensed under AGPL-3.0 — see LICENSE for details.
"""HTML report generator for combat matrix results."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path

from tests.combat_matrix.config_matrix import BattleConfig
from tests.combat_matrix.metrics import BattleMetrics

RESULTS_DIR = Path(__file__).resolve().parents[1] / ".test-results" / "combat-matrix"


def _severity_color(severity: str) -> str:
    if severity == "critical":
        return "#ff2a6d"
    if severity == "major":
        return "#fcee0a"
    return "#888"


def _pass_color(passed: bool) -> str:
    return "#05ffa1" if passed else "#ff2a6d"


def _config_status_color(metrics: BattleMetrics) -> str:
    if metrics.critical_pass and metrics.major_pass:
        return "#05ffa1"  # green
    if metrics.critical_pass:
        return "#fcee0a"  # yellow - major failures only
    return "#ff2a6d"  # red - critical failures


def generate_report(
    configs: list[BattleConfig],
    results: list[BattleMetrics],
) -> Path:
    """Generate an HTML report and return the file path."""
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_path = RESULTS_DIR / f"report_{timestamp}.html"

    # Also write latest symlink
    latest_path = RESULTS_DIR / "report.html"

    # Aggregate stats
    total_configs = len(results)
    critical_pass = sum(1 for r in results if r.critical_pass)
    all_pass = sum(1 for r in results if r.critical_pass and r.major_pass)
    total_shots = sum(r.total_shots_fired for r in results)
    total_hits = sum(r.total_shots_hit for r in results)
    total_kills = sum(r.total_eliminations for r in results)
    avg_accuracy = total_hits / total_shots if total_shots > 0 else 0.0
    total_duration = sum(r.battle_duration for r in results)

    # Build assertion heatmap data
    assertion_names: list[str] = []
    if results:
        assertion_names = [a.name for a in results[0].assertions
                          if not a.name.startswith("accuracy_")]

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Combat Matrix Report — {timestamp}</title>
<style>
  :root {{
    --bg: #0a0a0f;
    --panel: #111118;
    --border: #1a1a2e;
    --text: #c8c8d0;
    --cyan: #00f0ff;
    --magenta: #ff2a6d;
    --green: #05ffa1;
    --yellow: #fcee0a;
    --mono: 'JetBrains Mono', 'Fira Code', monospace;
  }}
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{
    background: var(--bg);
    color: var(--text);
    font-family: var(--mono);
    font-size: 13px;
    line-height: 1.5;
    padding: 20px;
  }}
  h1 {{
    color: var(--cyan);
    font-size: 22px;
    margin-bottom: 4px;
    letter-spacing: 2px;
  }}
  h2 {{
    color: var(--cyan);
    font-size: 16px;
    margin: 24px 0 12px;
    border-bottom: 1px solid var(--border);
    padding-bottom: 4px;
  }}
  .subtitle {{
    color: #666;
    font-size: 12px;
    margin-bottom: 20px;
  }}
  .stat-cards {{
    display: flex;
    gap: 12px;
    flex-wrap: wrap;
    margin-bottom: 20px;
  }}
  .stat-card {{
    background: var(--panel);
    border: 1px solid var(--border);
    padding: 12px 16px;
    min-width: 140px;
  }}
  .stat-card .label {{
    color: #666;
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 1px;
  }}
  .stat-card .value {{
    font-size: 24px;
    font-weight: bold;
    margin-top: 4px;
  }}
  table {{
    width: 100%;
    border-collapse: collapse;
    margin-bottom: 20px;
  }}
  th, td {{
    text-align: left;
    padding: 6px 10px;
    border-bottom: 1px solid var(--border);
    font-size: 12px;
  }}
  th {{
    color: var(--cyan);
    font-size: 10px;
    text-transform: uppercase;
    letter-spacing: 1px;
  }}
  tr:hover {{ background: #151520; }}
  .pass {{ color: var(--green); }}
  .fail {{ color: var(--magenta); }}
  .warn {{ color: var(--yellow); }}
  .heatmap-cell {{
    width: 20px;
    height: 20px;
    display: inline-block;
    border: 1px solid var(--border);
  }}
  details {{
    background: var(--panel);
    border: 1px solid var(--border);
    margin-bottom: 8px;
    padding: 8px 12px;
  }}
  details summary {{
    cursor: pointer;
    color: var(--cyan);
    font-weight: bold;
  }}
  details summary:hover {{ color: var(--green); }}
  .assertion-row {{ font-size: 11px; }}
  .thumb {{
    width: 120px;
    height: 68px;
    object-fit: cover;
    border: 1px solid var(--border);
    margin: 2px;
  }}
</style>
</head>
<body>
<h1>COMBAT MATRIX REPORT</h1>
<div class="subtitle">{timestamp} | {total_configs} configurations | {total_duration:.0f}s total</div>

<div class="stat-cards">
  <div class="stat-card">
    <div class="label">Configs Tested</div>
    <div class="value" style="color: var(--cyan)">{total_configs}</div>
  </div>
  <div class="stat-card">
    <div class="label">Critical Pass</div>
    <div class="value" style="color: {'var(--green)' if critical_pass == total_configs else 'var(--magenta)'}">{critical_pass}/{total_configs}</div>
  </div>
  <div class="stat-card">
    <div class="label">All Pass</div>
    <div class="value" style="color: {'var(--green)' if all_pass == total_configs else 'var(--yellow)'}">{all_pass}/{total_configs}</div>
  </div>
  <div class="stat-card">
    <div class="label">Total Shots</div>
    <div class="value" style="color: var(--cyan)">{total_shots:,}</div>
  </div>
  <div class="stat-card">
    <div class="label">Total Kills</div>
    <div class="value" style="color: var(--magenta)">{total_kills:,}</div>
  </div>
  <div class="stat-card">
    <div class="label">Avg Accuracy</div>
    <div class="value" style="color: var(--yellow)">{avg_accuracy:.1%}</div>
  </div>
</div>

<h2>Summary Table</h2>
<table>
<thead>
<tr>
  <th>Config</th>
  <th>Forces</th>
  <th>Loadout</th>
  <th>Map</th>
  <th>Shots</th>
  <th>Hits</th>
  <th>Kills</th>
  <th>Accuracy</th>
  <th>Duration</th>
  <th>Result</th>
  <th>Pass%</th>
</tr>
</thead>
<tbody>
"""
    for cfg, met in zip(configs, results):
        acc = met.total_shots_hit / met.total_shots_fired if met.total_shots_fired > 0 else 0.0
        status_color = _config_status_color(met)
        result_class = "pass" if met.game_result == "victory" else "fail" if met.game_result == "defeat" else "warn"
        html += f"""<tr>
  <td style="color: {status_color}">{cfg.config_id[:40]}</td>
  <td>{cfg.defender_count}v{cfg.hostile_count}</td>
  <td>{cfg.loadout_profile}</td>
  <td>{cfg.map_bounds:.0f}m</td>
  <td>{met.total_shots_fired}</td>
  <td>{met.total_shots_hit}</td>
  <td>{met.total_eliminations}</td>
  <td>{acc:.1%}</td>
  <td>{met.battle_duration:.0f}s</td>
  <td class="{result_class}">{met.game_result}</td>
  <td style="color: {status_color}">{met.pass_rate:.0%}</td>
</tr>
"""
    html += """</tbody>
</table>

<h2>Assertion Heatmap</h2>
<table>
<thead>
<tr>
  <th>Assertion</th>
"""
    for i in range(len(results)):
        html += f'  <th style="writing-mode:vertical-lr;font-size:9px">#{i}</th>\n'
    html += "</tr>\n</thead>\n<tbody>\n"

    for aname in assertion_names:
        html += f'<tr><td style="font-size:11px">{aname}</td>\n'
        for met in results:
            match = next((a for a in met.assertions if a.name == aname), None)
            if match:
                color = _pass_color(match.passed)
                html += f'  <td><span class="heatmap-cell" style="background:{color}" title="{match.message}"></span></td>\n'
            else:
                html += '  <td><span class="heatmap-cell" style="background:#333"></span></td>\n'
        html += "</tr>\n"
    html += "</tbody>\n</table>\n"

    # Per-config expandable details
    html += "<h2>Per-Config Details</h2>\n"
    for cfg, met in zip(configs, results):
        status_color = _config_status_color(met)
        html += f"""<details>
<summary style="color: {status_color}">{cfg.config_id} — {met.pass_rate:.0%} pass ({cfg.defender_count}v{cfg.hostile_count} {cfg.loadout_profile})</summary>
<table>
<tr><td>Result</td><td>{met.game_result}</td></tr>
<tr><td>Score</td><td>{met.final_score}</td></tr>
<tr><td>Duration</td><td>{met.battle_duration:.1f}s</td></tr>
<tr><td>Shots Fired</td><td>{met.total_shots_fired}</td></tr>
<tr><td>Shots Hit</td><td>{met.total_shots_hit}</td></tr>
<tr><td>Eliminations</td><td>{met.total_eliminations}</td></tr>
<tr><td>WS projectile_fired</td><td>{met.ws_projectile_fired}</td></tr>
<tr><td>WS target_eliminated</td><td>{met.ws_target_eliminated}</td></tr>
<tr><td>WS game_over</td><td>{met.ws_game_over}</td></tr>
<tr><td>Audio RMS</td><td>{met.audio_rms:.6f}</td></tr>
</table>
"""
        # Unit stats table
        if met.unit_stats:
            html += """<h3 style="color:var(--cyan);font-size:12px;margin:8px 0 4px">Unit Stats</h3>
<table>
<thead><tr><th>Name</th><th>Type</th><th>Alliance</th><th>Fired</th><th>Hit</th><th>Kills</th><th>Deaths</th><th>Accuracy</th></tr></thead>
<tbody>
"""
            for u in met.unit_stats:
                html += f"""<tr>
  <td>{u.get('name','')}</td>
  <td>{u.get('asset_type','')}</td>
  <td>{u.get('alliance','')}</td>
  <td>{u.get('shots_fired',0)}</td>
  <td>{u.get('shots_hit',0)}</td>
  <td>{u.get('kills',0)}</td>
  <td>{u.get('deaths',0)}</td>
  <td>{u.get('accuracy',0):.2%}</td>
</tr>
"""
            html += "</tbody></table>\n"

        # Assertions
        html += """<h3 style="color:var(--cyan);font-size:12px;margin:8px 0 4px">Assertions</h3>
<table>
<thead><tr><th>Name</th><th>Severity</th><th>Expected</th><th>Actual</th><th>Pass</th><th>Message</th></tr></thead>
<tbody>
"""
        for a in met.assertions:
            color = _pass_color(a.passed)
            sev_color = _severity_color(a.severity)
            html += f"""<tr class="assertion-row">
  <td>{a.name}</td>
  <td style="color:{sev_color}">{a.severity}</td>
  <td>{a.expected}</td>
  <td>{a.actual}</td>
  <td style="color:{color}">{'PASS' if a.passed else 'FAIL'}</td>
  <td style="color:#666">{a.message}</td>
</tr>
"""
        html += "</tbody></table>\n"

        # Screenshot thumbnails
        if met.screenshots:
            html += '<div style="margin:8px 0">\n'
            for ss in met.screenshots[:6]:
                html += f'  <img class="thumb" src="file://{ss}" alt="screenshot">\n'
            html += "</div>\n"

        # Errors
        if met.errors:
            html += '<div style="color:var(--magenta);margin:8px 0">\n'
            for err in met.errors:
                html += f"  <div>{err}</div>\n"
            html += "</div>\n"

        html += "</details>\n"

    html += """
</body>
</html>
"""

    report_path.write_text(html)

    # Write latest copy
    latest_path.write_text(html)

    # Also write JSON metrics
    json_path = RESULTS_DIR / f"metrics_{timestamp}.json"
    json_data = {
        "timestamp": timestamp,
        "total_configs": total_configs,
        "critical_pass": critical_pass,
        "all_pass": all_pass,
        "total_shots": total_shots,
        "total_kills": total_kills,
        "avg_accuracy": round(avg_accuracy, 4),
        "total_duration": round(total_duration, 2),
        "configs": [r.to_dict() for r in results],
    }
    json_path.write_text(json.dumps(json_data, indent=2))

    return report_path
