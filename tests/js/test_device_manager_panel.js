// Created by Matthew Valancy
// Copyright 2026 Valpatel Software LLC
// Licensed under AGPL-3.0 — see LICENSE for details.
/**
 * TRITIUM-SC Device Manager Panel tests
 * Tests DeviceManagerPanelDef structure, DOM creation, toolbar, device cards,
 * detail view, and bulk operation controls.
 * Run: node tests/js/test_device_manager_panel.js
 */

const fs = require('fs');
const vm = require('vm');

// Simple test runner
let passed = 0, failed = 0;
function assert(cond, msg) {
    if (!cond) { console.error('FAIL:', msg); failed++; }
    else { console.log('PASS:', msg); passed++; }
}

// ============================================================
// DOM + browser mocks
// ============================================================

function createMockElement(tag) {
    const children = [];
    const classList = new Set();
    const eventListeners = {};
    const dataset = {};
    const style = {};
    let _innerHTML = '';
    let _textContent = '';

    const el = {
        tagName: (tag || 'DIV').toUpperCase(),
        className: '',
        get innerHTML() { return _innerHTML; },
        set innerHTML(val) {
            _innerHTML = val;
            el._parsedBinds = {};
            const bindMatches = val.matchAll(/data-bind="([^"]+)"/g);
            for (const m of bindMatches) el._parsedBinds[m[1]] = true;
            el._parsedActions = {};
            const actionMatches = val.matchAll(/data-action="([^"]+)"/g);
            for (const m of actionMatches) el._parsedActions[m[1]] = true;
        },
        get textContent() { return _textContent; },
        set textContent(val) {
            _textContent = String(val);
            _innerHTML = String(val);
        },
        style,
        dataset,
        classList: {
            add: (c) => classList.add(c),
            remove: (c) => classList.delete(c),
            contains: (c) => classList.has(c),
            toggle: (c) => { if (classList.has(c)) classList.delete(c); else classList.add(c); },
        },
        children,
        childNodes: children,
        appendChild: (child) => { children.push(child); child.parentElement = el; return child; },
        removeChild: (child) => {
            const i = children.indexOf(child);
            if (i >= 0) children.splice(i, 1);
        },
        addEventListener: (type, fn) => {
            if (!eventListeners[type]) eventListeners[type] = [];
            eventListeners[type].push(fn);
        },
        removeEventListener: () => {},
        querySelector: (sel) => {
            // Simple attribute selector
            const attrMatch = sel.match(/\[data-bind="([^"]+)"\]/);
            if (attrMatch) {
                const bindName = attrMatch[1];
                if (el._parsedBinds && el._parsedBinds[bindName]) {
                    return createMockElement('span');
                }
            }
            const actionMatch = sel.match(/\[data-action="([^"]+)"\]/);
            if (actionMatch) {
                const actionName = actionMatch[1];
                if (el._parsedActions && el._parsedActions[actionName]) {
                    const btn = createMockElement('button');
                    btn.dataset.action = actionName;
                    return btn;
                }
            }
            return null;
        },
        querySelectorAll: () => [],
        getBoundingClientRect: () => ({ top: 0, left: 0, width: 100, height: 100 }),
        _eventListeners: eventListeners,
        _parsedBinds: {},
        _parsedActions: {},
        parentElement: null,
    };
    return el;
}

const mockDocument = {
    createElement: (tag) => createMockElement(tag),
    createTextNode: (text) => ({ textContent: text }),
    body: createMockElement('body'),
    querySelector: () => null,
    querySelectorAll: () => [],
    addEventListener: () => {},
    removeEventListener: () => {},
    documentElement: createMockElement('html'),
};

const mockWindow = {
    document: mockDocument,
    addEventListener: () => {},
    removeEventListener: () => {},
    requestAnimationFrame: (cb) => setTimeout(cb, 16),
    cancelAnimationFrame: () => {},
    setTimeout,
    clearTimeout,
    setInterval,
    clearInterval,
    innerWidth: 1920,
    innerHeight: 1080,
    getComputedStyle: () => ({}),
    navigator: { userAgent: 'test' },
    location: { href: 'http://localhost:8000/', pathname: '/' },
    history: { pushState: () => {}, replaceState: () => {} },
    fetch: () => Promise.resolve({ ok: true, json: () => Promise.resolve({}) }),
    open: () => {},
    FormData: class { append() {} },
};

// ============================================================
// Load the module
// ============================================================

const src = fs.readFileSync(
    __dirname + '/../../src/frontend/js/command/panels/device-manager.js',
    'utf-8'
);

// Strip import/export to run in CJS context
const transformed = src
    .replace(/^import\s+.*$/gm, '')
    .replace(/^export\s+/gm, '');

const sandbox = {
    document: mockDocument,
    window: mockWindow,
    console,
    setTimeout,
    clearTimeout,
    setInterval,
    clearInterval,
    fetch: mockWindow.fetch,
    FormData: mockWindow.FormData,
    Date,
    Math,
    JSON,
    Array,
    Object,
    String,
    Number,
    encodeURIComponent,
    Promise,
    Set,
    Map,
    EventBus: {
        on: () => (() => {}),
        off: () => {},
        emit: () => {},
        publish: () => {},
    },
    DeviceManagerPanelDef: null,
};

const script = new vm.Script(transformed + '\n;DeviceManagerPanelDef;', {
    filename: 'device-manager.js',
});
const ctx = vm.createContext(sandbox);
const def = script.runInContext(ctx);

// ============================================================
// Tests
// ============================================================

// 1. PanelDef exists and has correct shape
assert(def !== null && def !== undefined, 'DeviceManagerPanelDef exported');
assert(def.id === 'device-manager', 'id is device-manager');
assert(def.title === 'DEVICE MANAGER', 'title is DEVICE MANAGER');
assert(typeof def.create === 'function', 'has create()');
assert(typeof def.mount === 'function', 'has mount()');
assert(typeof def.unmount === 'function', 'has unmount()');

// 2. Default size
assert(def.defaultSize.w === 680, 'default width 680');
assert(def.defaultSize.h === 520, 'default height 520');

// 3. create() returns a DOM element
const panel = { _unsubs: [] };
const el = def.create(panel);
assert(el !== null, 'create() returns element');
assert(el.className === 'dm-inner', 'element class is dm-inner');

// 4. Toolbar elements present
assert(el._parsedActions['refresh'], 'has refresh button');
assert(el._parsedActions['reboot-selected'], 'has reboot-selected button');
assert(el._parsedActions['select-all'], 'has select-all checkbox');
assert(el._parsedActions['back-to-list'], 'has back-to-list button');

// 5. Data bind elements present
assert(el._parsedBinds['device-list'], 'has device-list bind');
assert(el._parsedBinds['detail-pane'], 'has detail-pane bind');
assert(el._parsedBinds['detail-title'], 'has detail-title bind');
assert(el._parsedBinds['detail-body'], 'has detail-body bind');
assert(el._parsedBinds['device-count'], 'has device-count bind');
assert(el._parsedBinds['status-msg'], 'has status-msg bind');
assert(el._parsedBinds['list-wrap'], 'has list-wrap bind');

// 6. mount() does not throw
let mountError = null;
try {
    def.mount(el, panel);
} catch (err) {
    mountError = err;
}
assert(mountError === null, 'mount() does not throw');

// 7. Panel registers unsub callbacks
assert(panel._unsubs.length > 0, 'mount() registers cleanup callbacks');

// 8. unmount() does not throw
let unmountError = null;
try {
    def.unmount(el);
} catch (err) {
    unmountError = err;
}
assert(unmountError === null, 'unmount() does not throw');

// ============================================================
// Summary
// ============================================================
console.log(`\n${passed} passed, ${failed} failed`);
process.exit(failed > 0 ? 1 : 0);
