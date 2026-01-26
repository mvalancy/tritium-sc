import { test, expect } from '@playwright/test';

test.describe('Zone Management', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('should load zones API', async ({ request }) => {
    const response = await request.get('/api/zones/');
    expect(response.ok()).toBeTruthy();

    const zones = await response.json();
    expect(Array.isArray(zones)).toBeTruthy();
  });

  test('should create a zone via API', async ({ request }) => {
    const response = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Test Zone',
        polygon: [[100, 100], [200, 100], [200, 200], [100, 200]],
        zone_type: 'activity',
      },
    });

    expect(response.ok()).toBeTruthy();

    const zone = await response.json();
    expect(zone.name).toBe('Test Zone');
    expect(zone.zone_type).toBe('activity');
    expect(zone.zone_id).toBeTruthy();

    // Clean up
    await request.delete(`/api/zones/${zone.zone_id}`);
  });

  test('should get zone events', async ({ request }) => {
    // Create a test zone
    const createResponse = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Event Test Zone',
        polygon: [[50, 50], [150, 50], [150, 150], [50, 150]],
        zone_type: 'activity',
      },
    });

    const zone = await createResponse.json();

    // Get events (should be empty)
    const eventsResponse = await request.get(`/api/zones/${zone.zone_id}/events`);
    expect(eventsResponse.ok()).toBeTruthy();

    const events = await eventsResponse.json();
    expect(Array.isArray(events)).toBeTruthy();

    // Clean up
    await request.delete(`/api/zones/${zone.zone_id}`);
  });

  test('should get zone summary', async ({ request }) => {
    // Create a test zone
    const createResponse = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Summary Test Zone',
        polygon: [[0, 0], [100, 0], [100, 100], [0, 100]],
        zone_type: 'object_monitor',
        monitored_object: 'dumpster',
      },
    });

    const zone = await createResponse.json();

    // Get summary
    const summaryResponse = await request.get(`/api/zones/${zone.zone_id}/summary`);
    expect(summaryResponse.ok()).toBeTruthy();

    const summary = await summaryResponse.json();
    expect(summary.zone_name).toBe('Summary Test Zone');
    expect(summary.zone_type).toBe('object_monitor');

    // Clean up
    await request.delete(`/api/zones/${zone.zone_id}`);
  });

  test('should update a zone', async ({ request }) => {
    // Create a test zone
    const createResponse = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Update Test Zone',
        polygon: [[0, 0], [50, 0], [50, 50], [0, 50]],
        zone_type: 'activity',
      },
    });

    const zone = await createResponse.json();

    // Update it
    const updateResponse = await request.put(`/api/zones/${zone.zone_id}`, {
      data: {
        name: 'Updated Zone Name',
        enabled: false,
      },
    });

    expect(updateResponse.ok()).toBeTruthy();

    const updated = await updateResponse.json();
    expect(updated.name).toBe('Updated Zone Name');
    expect(updated.enabled).toBe(false);

    // Clean up
    await request.delete(`/api/zones/${zone.zone_id}`);
  });

  test('should delete a zone', async ({ request }) => {
    // Create a test zone
    const createResponse = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Delete Test Zone',
        polygon: [[0, 0], [10, 0], [10, 10], [0, 10]],
        zone_type: 'tripwire',
      },
    });

    const zone = await createResponse.json();

    // Delete it
    const deleteResponse = await request.delete(`/api/zones/${zone.zone_id}`);
    expect(deleteResponse.ok()).toBeTruthy();

    // Verify it's gone
    const getResponse = await request.get(`/api/zones/${zone.zone_id}`);
    expect(getResponse.status()).toBe(404);
  });
});

test.describe('Zone Types', () => {
  test('should create entry_exit zone', async ({ request }) => {
    const response = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Entry Exit Zone',
        polygon: [[0, 0], [100, 0], [100, 100], [0, 100]],
        zone_type: 'entry_exit',
        alert_on_enter: true,
        alert_on_exit: true,
      },
    });

    expect(response.ok()).toBeTruthy();
    const zone = await response.json();
    expect(zone.zone_type).toBe('entry_exit');

    await request.delete(`/api/zones/${zone.zone_id}`);
  });

  test('should create object_monitor zone', async ({ request }) => {
    const response = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Dumpster Monitor',
        polygon: [[200, 200], [300, 200], [300, 300], [200, 300]],
        zone_type: 'object_monitor',
        monitored_object: 'dumpster',
      },
    });

    expect(response.ok()).toBeTruthy();
    const zone = await response.json();
    expect(zone.zone_type).toBe('object_monitor');
    expect(zone.monitored_object).toBe('dumpster');

    await request.delete(`/api/zones/${zone.zone_id}`);
  });

  test('should create tripwire zone', async ({ request }) => {
    const response = await request.post('/api/zones/', {
      data: {
        camera_id: 1,
        name: 'Sidewalk Tripwire',
        polygon: [[0, 500], [1920, 500], [1920, 510], [0, 510]],
        zone_type: 'tripwire',
      },
    });

    expect(response.ok()).toBeTruthy();
    const zone = await response.json();
    expect(zone.zone_type).toBe('tripwire');

    await request.delete(`/api/zones/${zone.zone_id}`);
  });
});
