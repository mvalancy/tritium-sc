// Created by Matthew Valancy
// Copyright 2026 Valpatel Software LLC
// Licensed under AGPL-3.0 — see LICENSE for details.
// Geofence Panel — polygon zone management with alert history.
// CRUD via /api/geofence/zones, events via /api/geofence/events.
// Emits geofence:drawZone to enter polygon drawing mode on the map.
// Emits zone:selected when a geofence zone is clicked (for map highlighting).

import { EventBus } from '../events.js';

function _esc(text) {
    if (!text) return '';
    const div = document.createElement('div');
    div.textContent = String(text);
    return div.innerHTML;
}

const TYPE_COLORS = {
    restricted: '#ff2a6d',
    monitored: '#00f0ff',
    safe: '#05ffa1',
};

export const GeofencePanelDef = {
    id: 'geofence',
    title: 'GEOFENCE',
    defaultPosition: { x: 8, y: 360 },
    defaultSize: { w: 300, h: 420 },

    create(panel) {
        const el = document.createElement('div');
        el.className = 'geofence-panel-inner';
        el.innerHTML = `
            <div class="zone-toolbar">
                <button class="panel-action-btn panel-action-btn-primary" data-action="refresh">REFRESH</button>
                <button class="panel-action-btn" data-action="draw-zone">+ DRAW ZONE</button>
            </div>
            <div class="geofence-tab-bar" style="display:flex;gap:2px;margin:4px 0">
                <button class="panel-action-btn panel-action-btn-primary geofence-tab" data-tab="zones" style="flex:1;font-size:0.45rem">ZONES</button>
                <button class="panel-action-btn geofence-tab" data-tab="events" style="flex:1;font-size:0.45rem">EVENTS</button>
            </div>
            <ul class="panel-list zone-list" data-bind="zone-list" role="listbox" aria-label="Geofence zones">
                <li class="panel-empty">Loading zones...</li>
            </ul>
            <div class="geofence-events" data-bind="event-list" style="display:none">
                <ul class="panel-list" data-bind="event-items" role="list" aria-label="Geofence events">
                    <li class="panel-empty">Loading events...</li>
                </ul>
            </div>
        `;
        return el;
    },

    mount(bodyEl, panel) {
        const zoneListEl = bodyEl.querySelector('[data-bind="zone-list"]');
        const eventListEl = bodyEl.querySelector('[data-bind="event-list"]');
        const eventItemsEl = bodyEl.querySelector('[data-bind="event-items"]');
        const refreshBtn = bodyEl.querySelector('[data-action="refresh"]');
        const drawBtn = bodyEl.querySelector('[data-action="draw-zone"]');
        const tabs = bodyEl.querySelectorAll('.geofence-tab');

        let zones = [];
        let events = [];
        let activeTab = 'zones';

        // Tab switching
        tabs.forEach(tab => {
            tab.addEventListener('click', () => {
                activeTab = tab.dataset.tab;
                tabs.forEach(t => t.classList.toggle('panel-action-btn-primary', t.dataset.tab === activeTab));
                zoneListEl.style.display = activeTab === 'zones' ? '' : 'none';
                eventListEl.style.display = activeTab === 'events' ? '' : 'none';
                if (activeTab === 'events') fetchEvents();
            });
        });

        function renderZones() {
            if (!zoneListEl) return;
            if (zones.length === 0) {
                zoneListEl.innerHTML = '<li class="panel-empty">No geofence zones defined</li>';
                return;
            }

            zoneListEl.innerHTML = zones.map(z => {
                const color = TYPE_COLORS[z.zone_type] || '#00f0ff';
                const vertices = (z.polygon || []).length;
                const flags = [];
                if (z.alert_on_enter) flags.push('ENTER');
                if (z.alert_on_exit) flags.push('EXIT');
                return `<li class="panel-list-item zone-item" data-zone-id="${_esc(z.zone_id)}" role="option">
                    <span class="panel-dot" style="background:${color}"></span>
                    <span class="zone-item-info" style="flex:1;min-width:0">
                        <span class="zone-item-name">${_esc(z.name)}</span>
                        <span class="mono" style="font-size:0.42rem;color:var(--text-ghost)">${_esc(z.zone_type)} | ${vertices} pts | ${flags.join('+') || 'no alerts'}</span>
                    </span>
                    <button class="panel-btn zone-delete-btn" data-action="delete-zone" data-zone-id="${_esc(z.zone_id)}" title="Delete zone">&times;</button>
                </li>`;
            }).join('');

            // Click to select / highlight on map
            zoneListEl.querySelectorAll('.zone-item').forEach(item => {
                item.addEventListener('click', (e) => {
                    if (e.target.closest('[data-action="delete-zone"]')) return;
                    const zid = item.dataset.zoneId;
                    const zone = zones.find(z => z.zone_id === zid);
                    if (zone) {
                        EventBus.emit('zone:selected', { id: zid, polygon: zone.polygon });
                        // Center map on zone centroid
                        const pts = zone.polygon || [];
                        if (pts.length > 0) {
                            const cx = pts.reduce((s, p) => s + (p[0] || 0), 0) / pts.length;
                            const cy = pts.reduce((s, p) => s + (p[1] || 0), 0) / pts.length;
                            EventBus.emit('map:centerOnUnit', { id: null, x: cx, y: cy });
                        }
                    }
                });
            });

            // Delete
            zoneListEl.querySelectorAll('[data-action="delete-zone"]').forEach(btn => {
                btn.addEventListener('click', async (e) => {
                    e.stopPropagation();
                    const zid = btn.dataset.zoneId;
                    try {
                        await fetch(`/api/geofence/zones/${zid}`, { method: 'DELETE' });
                        fetchZones();
                        EventBus.emit('toast:show', { message: 'Geofence zone deleted', type: 'info' });
                    } catch (_) {
                        EventBus.emit('toast:show', { message: 'Failed to delete zone', type: 'alert' });
                    }
                });
            });
        }

        function renderEvents() {
            if (!eventItemsEl) return;
            if (events.length === 0) {
                eventItemsEl.innerHTML = '<li class="panel-empty">No geofence events yet</li>';
                return;
            }

            eventItemsEl.innerHTML = events.slice(0, 50).map(ev => {
                const time = new Date(ev.timestamp * 1000).toLocaleTimeString().substring(0, 8);
                const color = ev.event_type === 'enter' ? '#ff2a6d' : '#05ffa1';
                const arrow = ev.event_type === 'enter' ? '>' : '<';
                return `<li class="panel-list-item" style="font-size:0.48rem">
                    <span style="color:${color};font-weight:bold;width:12px">${arrow}</span>
                    <span style="flex:1;min-width:0">
                        <span style="color:var(--text-main)">${_esc(ev.target_id?.substring(0, 8))}</span>
                        <span style="color:var(--text-dim)"> ${_esc(ev.event_type)} </span>
                        <span style="color:var(--text-ghost)">${_esc(ev.zone_name)}</span>
                    </span>
                    <span class="mono" style="color:var(--text-ghost);font-size:0.42rem">${time}</span>
                </li>`;
            }).join('');
        }

        async function fetchZones() {
            try {
                const resp = await fetch('/api/geofence/zones');
                if (!resp.ok) { zones = []; renderZones(); return; }
                zones = await resp.json();
                if (!Array.isArray(zones)) zones = [];
                renderZones();
            } catch (_) {
                zones = [];
                renderZones();
            }
        }

        async function fetchEvents() {
            try {
                const resp = await fetch('/api/geofence/events?limit=50');
                if (!resp.ok) { events = []; renderEvents(); return; }
                events = await resp.json();
                if (!Array.isArray(events)) events = [];
                renderEvents();
            } catch (_) {
                events = [];
                renderEvents();
            }
        }

        function drawZone() {
            // Emit event to start polygon drawing on the map
            EventBus.emit('geofence:drawZone', {});
            EventBus.emit('toast:show', { message: 'Click map vertices, then press Enter to finish', type: 'info' });
        }

        // Listen for completed polygon from map
        const onZoneDrawn = async (data) => {
            if (!data || !data.polygon || data.polygon.length < 3) return;
            try {
                const resp = await fetch('/api/geofence/zones', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        name: `Zone ${zones.length + 1}`,
                        polygon: data.polygon,
                        zone_type: data.zone_type || 'monitored',
                        alert_on_enter: true,
                        alert_on_exit: true,
                    }),
                });
                if (resp.ok) {
                    fetchZones();
                    EventBus.emit('toast:show', { message: 'Geofence zone created', type: 'info' });
                }
            } catch (_) {
                EventBus.emit('toast:show', { message: 'Failed to create zone', type: 'alert' });
            }
        };
        EventBus.on('geofence:zoneDrawn', onZoneDrawn);
        panel._unsubs.push(() => EventBus.off('geofence:zoneDrawn', onZoneDrawn));

        if (refreshBtn) refreshBtn.addEventListener('click', () => {
            fetchZones();
            if (activeTab === 'events') fetchEvents();
        });
        if (drawBtn) drawBtn.addEventListener('click', drawZone);

        // Auto-refresh every 30s
        const refreshInterval = setInterval(() => {
            fetchZones();
            if (activeTab === 'events') fetchEvents();
        }, 30000);
        panel._unsubs.push(() => clearInterval(refreshInterval));

        fetchZones();
    },

    unmount(bodyEl) {
        // _unsubs cleaned up by Panel base class
    },
};
