# TRITIUM-SC Development Plan

## Vision
A comprehensive Security Central platform that tracks **targets** (people, vehicles, objects) across time and space, with zone-based activity monitoring and human-in-the-loop learning.

---

## Phase 1: Foundation (COMPLETE)
- [x] FastAPI backend with async SQLite
- [x] YOLO detection with ByteTrack
- [x] Video browsing and streaming
- [x] Basic 3D property view
- [x] NVR auto-discovery

## Phase 2: Target Tracking Core (IN PROGRESS)
- [x] Rename "objects" → "targets" throughout codebase
- [ ] Thumbnail extraction for all targets
- [ ] Target gallery view in UI
- [ ] Manual merge/consolidation interface
- [ ] Label assignment with persistence
- [ ] Feedback logging for reinforcement learning

## Phase 3: Zone-Based Monitoring (IN PROGRESS)
- [x] Zone data models (Zone, ZoneEvent, ZoneType)
- [x] Zone manager with CRUD operations
- [x] Zone API endpoints (/api/zones)
- [x] Zone definition UI (draw polygons on camera view)
- [x] Zone types: entry/exit, activity area, tripwire, object_monitor
- [x] Zone event detection during video analysis
- [x] Zone activity summary endpoint
- [x] Zones view in frontend with zone editor
- [x] Zone events timeline panel
- [ ] Zone-specific alerts (webhook/MQTT)
- [ ] Group clips by zone activity
- [ ] Zone heatmaps and visualizations

## Phase 4: Deep Target Intelligence (PLANNED)
- [ ] Target re-identification across cameras
- [ ] Target history view (all appearances of "this person")
- [ ] Movement patterns (where do they go, when)
- [ ] Behavioral analysis (how long do they stay, what do they interact with)
- [ ] Cross-reference: "who opened the dumpster" → show the person

## Phase 5: Test Automation (IN PROGRESS)
- [x] Playwright E2E test suite setup
- [x] Dashboard UI tests
- [x] Zone API integration tests
- [x] Zone UI component tests
- [x] Keyboard navigation tests
- [ ] Visual regression tests
- [ ] Performance benchmarks
- [ ] CI/CD pipeline

## Phase 6: Advanced Features (FUTURE)
- [ ] Real-time RTSP stream analysis
- [ ] WebSocket live alerts
- [ ] MQTT/Home Assistant integration
- [ ] Natural language search ("show me everyone who visited yesterday")
- [ ] Satellite imagery overlay for 3D view
- [ ] Multi-sensor fusion (door sensors, motion, etc.)

---

## Technical Architecture

### Target Model
```
Target:
  - target_id: unique identifier
  - target_type: person | vehicle | zone_object
  - first_seen: timestamp
  - last_seen: timestamp
  - appearances: list of sightings
  - thumbnails: list of cropped images
  - labels: user-assigned names
  - merged_with: linked targets (same entity)
  - zones_visited: which zones they've been in
```

### Zone Model
```
Zone:
  - zone_id: unique identifier
  - camera_id: which camera
  - name: user-assigned name
  - polygon: list of [x,y] points
  - zone_type: entry_exit | activity | tripwire | object_monitor
  - monitored_object: for object_monitor type (e.g., "dumpster")
  - events: list of zone events
```

### Zone Event Model
```
ZoneEvent:
  - event_id: unique identifier
  - zone_id: which zone
  - event_type: enter | exit | activity | state_change
  - target_id: who/what triggered it
  - timestamp: when
  - video_clip: short clip of event
  - thumbnail: snapshot
```

---

## UI Components Needed

### 1. Target Gallery
- Grid of all detected targets with thumbnails
- Filter by type (person, vehicle)
- Filter by date range
- Filter by zone
- Click to see all appearances
- Drag to merge duplicates
- Right-click to label

### 2. Zone Editor
- Overlay on camera view
- Draw polygon/rectangle tools
- Name zones
- Set zone type and rules
- Preview zone coverage

### 3. Zone Activity View
- Timeline of zone events
- Filter by zone
- "Dumpster opened 5 times" with clips
- Who triggered each event

### 4. Target Detail View
- All appearances of a target
- Movement timeline
- Zones visited
- Associated events
- Link to merge with other targets

---

## Test Plan

### E2E Tests (Playwright)
1. Navigation tests
   - Load dashboard
   - Switch between views
   - Select cameras

2. Video playback tests
   - Play video
   - Seek timeline
   - Switch dates

3. Target management tests
   - View target gallery
   - Merge targets
   - Label targets
   - Submit feedback

4. Zone management tests
   - Create zone
   - Edit zone polygon
   - Delete zone
   - View zone events

5. Search tests
   - Search by target type
   - Search by date
   - Search by zone

### API Tests
1. Camera endpoints
2. Video endpoints
3. AI analysis endpoints
4. Target/search endpoints
5. Zone endpoints (new)

### Performance Tests
1. Video streaming latency
2. Detection throughput
3. Gallery load time with 1000+ targets
4. Zone event query performance
