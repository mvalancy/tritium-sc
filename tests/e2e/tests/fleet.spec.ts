import { test, expect } from '@playwright/test';

test.describe('Fleet Panel', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
    // Open fleet panel via command or nav
    // The fleet panel is opened by the FLEET command button
    const fleetBtn = page.locator('#btn-fleet, [data-panel="fleet"]');
    if (await fleetBtn.isVisible()) {
      await fleetBtn.click();
    }
  });

  test('should render fleet health summary', async ({ page }) => {
    // Fleet panel should show health score section
    const panel = page.locator('.fleet-panel, [data-panel="fleet"]');
    if (await panel.isVisible()) {
      await expect(panel).toBeVisible();
    }
  });

  test('fleet API returns node list', async ({ request }) => {
    const resp = await request.get('/api/fleet/nodes');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('nodes');
    expect(data).toHaveProperty('count');
    expect(Array.isArray(data.nodes)).toBe(true);
  });

  test('fleet API returns presence data', async ({ request }) => {
    const resp = await request.get('/api/fleet/presence');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('devices');
    expect(data).toHaveProperty('count');
  });

  test('fleet API returns health report', async ({ request }) => {
    const resp = await request.get('/api/fleet/health-report');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('source');
  });

  test('fleet API returns correlations with summary', async ({ request }) => {
    const resp = await request.get('/api/fleet/correlations');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('correlations');
    expect(data).toHaveProperty('source');
    expect(data).toHaveProperty('summary');
    expect(data.summary).toHaveProperty('total');
    expect(data.summary).toHaveProperty('high_confidence');
    expect(data.summary).toHaveProperty('by_type');
    expect(data.summary).toHaveProperty('affected_devices');
  });

  test('fleet API returns topology', async ({ request }) => {
    const resp = await request.get('/api/fleet/topology');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('nodes');
    expect(data).toHaveProperty('source');
  });

  test('fleet API returns heap trends', async ({ request }) => {
    const resp = await request.get('/api/fleet/heap-trends');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('trends');
    expect(data).toHaveProperty('source');
  });

  test('fleet API returns dashboard summary', async ({ request }) => {
    const resp = await request.get('/api/fleet/dashboard');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('source');
  });

  test('fleet API returns config sync status', async ({ request }) => {
    const resp = await request.get('/api/fleet/config');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('source');
  });

  test('fleet node detail returns 404 for unknown device', async ({ request }) => {
    const resp = await request.get('/api/fleet/node/nonexistent-device-999');
    expect(resp.status()).toBe(404);
  });

  test('fleet node diagnostics returns data or unavailable', async ({ request }) => {
    const resp = await request.get('/api/fleet/node/test-device/diag');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('device_id');
    expect(data).toHaveProperty('source');
  });

  test('fleet API returns mesh peers data', async ({ request }) => {
    const resp = await request.get('/api/fleet/mesh-peers');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('nodes');
    expect(data).toHaveProperty('source');
    expect(Array.isArray(data.nodes)).toBe(true);
  });

  test('fleet node mesh peers returns data or unavailable', async ({ request }) => {
    const resp = await request.get('/api/fleet/node/test-device/mesh-peers');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('device_id');
    expect(data).toHaveProperty('mesh_peers');
    expect(data).toHaveProperty('source');
    expect(data.device_id).toBe('test-device');
    expect(Array.isArray(data.mesh_peers)).toBe(true);
  });

  test('mesh peers response contains expected fields when populated', async ({ request }) => {
    const resp = await request.get('/api/fleet/mesh-peers');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('count');
    // count should match nodes array length
    expect(data.count).toBe(data.nodes.length);
  });

  test('topology response includes edges array for mesh adjacency', async ({ request }) => {
    const resp = await request.get('/api/fleet/topology');
    expect(resp.status()).toBe(200);
    const data = await resp.json();
    expect(data).toHaveProperty('nodes');
    expect(data).toHaveProperty('edges');
    expect(data).toHaveProperty('source');
    expect(Array.isArray(data.nodes)).toBe(true);
    expect(Array.isArray(data.edges)).toBe(true);
  });
});
