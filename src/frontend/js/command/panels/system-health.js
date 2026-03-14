// Created by Matthew Valancy
// Copyright 2026 Valpatel Software LLC
// Licensed under AGPL-3.0 — see LICENSE for details.
// System Health Panel
// At-a-glance view of the entire Tritium system: plugin statuses,
// API endpoint counts, test results, notification counts, active
// targets, dossier counts. One panel to see everything.

import { EventBus } from '../events.js';
import { _esc } from '../panel-utils.js';


function _statusDot(healthy) {
    const cls = healthy ? 'panel-dot-green' : 'panel-dot-neutral';
    return `<span class="panel-dot ${cls}"></span>`;
}

export const SystemHealthPanelDef = {
    id: 'system-health',
    title: 'SYSTEM HEALTH',
    defaultPosition: { x: 8, y: 60 },
    defaultSize: { w: 340, h: 520 },

    create(_panel) {
        const el = document.createElement('div');
        el.className = 'system-health-panel-inner';
        el.innerHTML = `
            <div class="sys-health-toolbar">
                <button class="panel-action-btn panel-action-btn-primary" data-action="refresh-health">REFRESH</button>
                <span class="sys-health-timestamp mono" data-bind="timestamp" style="font-size:0.4rem;color:var(--text-ghost);margin-left:auto"></span>
            </div>
            <div class="sys-health-content" data-bind="content">
                <div class="panel-empty">Loading system health...</div>
            </div>
        `;
        return el;
    },

    mount(bodyEl, panel) {
        const contentEl = bodyEl.querySelector('[data-bind="content"]');
        const timestampEl = bodyEl.querySelector('[data-bind="timestamp"]');

        async function fetchHealth() {
            if (!contentEl) return;
            contentEl.innerHTML = '<div class="panel-empty">Scanning...</div>';

            // Fetch from all available endpoints in parallel
            const results = {};
            const fetches = [
                ['plugins', '/api/plugins'],
                ['pluginHealth', '/api/plugins/health'],
                ['targets', '/api/targets'],
                ['dossiers', '/api/dossiers'],
                ['notifications', '/api/notifications'],
                ['health', '/api/health'],
                ['demo', '/api/demo/status'],
                ['testing', '/api/testing/report'],
                ['amy', '/api/amy/status'],
            ];

            await Promise.allSettled(
                fetches.map(async ([key, url]) => {
                    try {
                        const resp = await fetch(url);
                        if (resp.ok) {
                            results[key] = await resp.json();
                        }
                    } catch (_) { /* endpoint unavailable */ }
                })
            );

            let html = '';

            // --- Overall status ---
            const healthData = results.health || {};
            const serverStatus = healthData.status || 'unknown';
            const serverColor = serverStatus === 'ok' || serverStatus === 'healthy'
                ? 'var(--green)' : 'var(--amber)';
            html += `<div class="panel-section-label">SERVER</div>`;
            html += `<div class="panel-stat-row">
                <span class="panel-stat-label">STATUS</span>
                <span class="panel-stat-value" style="color:${serverColor}">${_esc(serverStatus.toUpperCase())}</span>
            </div>`;
            if (healthData.uptime) {
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">UPTIME</span>
                    <span class="panel-stat-value mono">${_esc(healthData.uptime)}</span>
                </div>`;
            }
            if (healthData.version) {
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">VERSION</span>
                    <span class="panel-stat-value mono">${_esc(healthData.version)}</span>
                </div>`;
            }

            // --- Plugins ---
            const plugins = results.plugins || [];
            const pluginHealth = results.pluginHealth || {};
            if (plugins.length > 0) {
                const healthyCount = plugins.filter(p => p.healthy).length;
                const totalCount = plugins.length;
                const allHealthy = healthyCount === totalCount;
                const summaryColor = allHealthy ? 'var(--green)' : 'var(--amber)';

                html += `<div class="panel-section-label">PLUGINS (${healthyCount}/${totalCount})</div>`;
                for (const p of plugins) {
                    const dot = _statusDot(p.healthy);
                    const name = p.name || p.id || '?';
                    const version = p.version || '';
                    const statusText = p.healthy ? 'OK' : 'DOWN';
                    const statusColor = p.healthy ? 'var(--green)' : 'var(--magenta)';
                    html += `<div class="panel-stat-row">
                        ${dot}
                        <span class="panel-stat-label" style="flex:1">${_esc(name)}</span>
                        <span class="mono" style="font-size:0.4rem;color:var(--text-ghost);margin-right:8px">${_esc(version)}</span>
                        <span class="panel-stat-value" style="color:${statusColor};min-width:30px">${statusText}</span>
                    </div>`;
                }
            }

            // --- Targets ---
            const targets = results.targets;
            if (targets) {
                const targetList = Array.isArray(targets) ? targets : (targets.targets || []);
                const hostiles = targetList.filter(t => t.alliance === 'hostile').length;
                const friendlies = targetList.filter(t => t.alliance === 'friendly').length;
                const unknowns = targetList.filter(t => t.alliance === 'unknown' || !t.alliance).length;

                html += `<div class="panel-section-label">TARGETS</div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">TOTAL</span>
                    <span class="panel-stat-value" style="color:var(--cyan)">${targetList.length}</span>
                </div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">FRIENDLY</span>
                    <span class="panel-stat-value" style="color:var(--green)">${friendlies}</span>
                </div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">HOSTILE</span>
                    <span class="panel-stat-value" style="color:var(--magenta)">${hostiles}</span>
                </div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">UNKNOWN</span>
                    <span class="panel-stat-value" style="color:var(--amber)">${unknowns}</span>
                </div>`;
            }

            // --- Dossiers ---
            const dossiers = results.dossiers;
            if (dossiers) {
                const dossierList = Array.isArray(dossiers) ? dossiers : (dossiers.dossiers || []);
                html += `<div class="panel-section-label">DOSSIERS</div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">TOTAL</span>
                    <span class="panel-stat-value" style="color:var(--cyan)">${dossierList.length}</span>
                </div>`;
            }

            // --- Notifications ---
            const notifications = results.notifications;
            if (notifications) {
                const notifList = Array.isArray(notifications) ? notifications
                    : (notifications.notifications || []);
                const unread = notifList.filter(n => !n.read).length;
                html += `<div class="panel-section-label">NOTIFICATIONS</div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">TOTAL</span>
                    <span class="panel-stat-value">${notifList.length}</span>
                </div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">UNREAD</span>
                    <span class="panel-stat-value" style="color:${unread > 0 ? 'var(--amber)' : 'var(--green)'}">${unread}</span>
                </div>`;
            }

            // --- Amy ---
            const amy = results.amy;
            if (amy) {
                const amyStatus = amy.status || amy.state || 'unknown';
                const amyColor = amyStatus === 'active' || amyStatus === 'thinking'
                    ? 'var(--cyan)' : 'var(--text-ghost)';
                html += `<div class="panel-section-label">AMY (AI COMMANDER)</div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">STATUS</span>
                    <span class="panel-stat-value" style="color:${amyColor}">${_esc(amyStatus.toUpperCase())}</span>
                </div>`;
                if (amy.mood) {
                    html += `<div class="panel-stat-row">
                        <span class="panel-stat-label">MOOD</span>
                        <span class="panel-stat-value">${_esc(amy.mood)}</span>
                    </div>`;
                }
            }

            // --- Demo mode ---
            const demo = results.demo;
            if (demo) {
                const demoActive = demo.running || demo.active || false;
                const demoColor = demoActive ? 'var(--green)' : 'var(--text-ghost)';
                html += `<div class="panel-section-label">DEMO MODE</div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">STATUS</span>
                    <span class="panel-stat-value" style="color:${demoColor}">${demoActive ? 'ACTIVE' : 'OFF'}</span>
                </div>`;
            }

            // --- Test Results ---
            const testing = results.testing;
            if (testing) {
                const passed = testing.passed || testing.tests_passed || 0;
                const failed = testing.failed || testing.tests_failed || 0;
                const total = testing.total || testing.tests_total || (passed + failed);
                const testColor = failed > 0 ? 'var(--magenta)' : 'var(--green)';
                html += `<div class="panel-section-label">TEST RESULTS</div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">PASSED</span>
                    <span class="panel-stat-value" style="color:var(--green)">${passed}</span>
                </div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">FAILED</span>
                    <span class="panel-stat-value" style="color:${testColor}">${failed}</span>
                </div>`;
                html += `<div class="panel-stat-row">
                    <span class="panel-stat-label">TOTAL</span>
                    <span class="panel-stat-value">${total}</span>
                </div>`;
            }

            contentEl.innerHTML = html || '<div class="panel-empty">No health data available</div>';

            // Update timestamp
            if (timestampEl) {
                const now = new Date();
                timestampEl.textContent = `Updated ${now.toLocaleTimeString()}`;
            }
        }

        bodyEl.querySelector('[data-action="refresh-health"]')?.addEventListener('click', fetchHealth);

        // Initial fetch
        fetchHealth();

        // Auto-refresh every 30s
        const interval = setInterval(fetchHealth, 30000);
        panel._unsubs.push(() => clearInterval(interval));
    },
};
