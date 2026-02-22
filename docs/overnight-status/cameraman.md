# Cameraman Agent Status

## Completed Tasks

### Task 1: Enhanced CCTV Frame Renderer
- Added `render_cctv_frame()` to `src/amy/synthetic/video_gen.py`
- 5 CCTV scene types: front_door, back_yard, street_view, parking, driveway
- 3 time-of-day modes: day, dusk, night (with proper brightness ordering)
- Realism features:
  - Gaussian sensor noise (sigma 3-8, camera-specific)
  - Barrel distortion (subtle fisheye, strength 0.15)
  - JPEG compression artifacts (quality 60-75, camera-specific)
  - Per-camera color temperature shift (via hash of camera name)
  - Occasional horizontal line artifacts (10% probability)
  - Composited silhouette figures (people, cars, animals)
- Each scene has unique composition:
  - front_door: door + porch + porch light + wall texture + welcome mat
  - back_yard: grass + fence + trees + animals
  - street_view: vanishing point + parked cars + streetlights + depth lines
  - parking: overhead view + parking lines + parked cars + light pools
  - driveway: garage + driveway angle + sidewalk + yard
- Overlay: camera name, timestamp (YYYY-MM-DD HH:MM:SS), REC dot, frame counter
- Deterministic given same seed + camera_name + scene_type + time_of_day + frame_number

### Task 2: Video Clip Generation
- Added `generate_cctv_clip()` method to `SyntheticVideoLibrary`
- Generates MP4 clip + individual JPEG frames + metadata JSON
- Clips at 10fps with subtle motion (noise variation, composited figures)
- Files saved to `data/synthetic/cameras/{camera_name}/`

### Task 3: Batch Generation Script
- Created `scripts/generate_feeds.py`
- Generates 6 cameras x 50 frames + 6 clips
- Cameras:
  1. CAM-01 Front Door (front_door, day)
  2. CAM-02 Back Yard (back_yard, day)
  3. CAM-03 Street East (street_view, dusk)
  4. CAM-04 Street West (street_view, night)
  5. CAM-05 Driveway (driveway, day)
  6. CAM-06 Parking (parking, night)
- Progress output, summary JSON

### Task 4: Parallel Generation
- Script supports `--parallel` flag
- Checks GB10-02 via SSH
- Splits 3+3 cameras if remote is reachable
- Falls back to local-only if remote unavailable

### Task 5: Wire Into Existing API
- Added "cctv" as valid scene type in `SyntheticFeedManager`
- `render_cctv_frame` added to `_RENDERERS` in `synthetic_feed.py`
- POST /api/synthetic/cameras with scene_type="cctv" creates a CCTV feed
- MJPEG streaming and snapshot endpoints work with CCTV feeds

### Task 6: Tests
- Created `tests/amy/synthetic/test_cctv_feeds.py` — 25 tests, all passing
- Test classes:
  - TestCCTVFrameResolution — 640x480 and 1280x720
  - TestCCTVTimestampOverlay — bright pixels in top-left region
  - TestCCTVNoise — frame std deviation > 5
  - TestCCTVDayVsNight — day > 80 brightness, night < 40
  - TestCCTVUniqueFrames — 10 frames mostly distinct (SSIM < 0.99)
  - TestCCTVDeterministic — same seed = identical frame
  - TestCCTVSceneTypes — all 5 scene types render, invalid raises ValueError
  - TestCCTVRealism — barrel distortion + JPEG artifacts
  - TestCCTVClipGeneration — duration, motion, frame count, metadata, JPEG frames
  - TestCCTVFeedAPI — cctv scene accepted, snapshot valid JPEG, MJPEG streams

## Test Results
- 25/25 CCTV tests pass
- 445/445 all synthetic tests pass (0 regressions)

## Files Modified
- `src/amy/synthetic/video_gen.py` — added CCTV_SCENE_TYPES, render_cctv_frame, 5 scene renderers, barrel distortion, JPEG compression
- `src/amy/synthetic/video_library.py` — added generate_cctv_clip method
- `src/app/routers/synthetic_feed.py` — added cctv scene type support

## Files Created
- `tests/amy/synthetic/test_cctv_feeds.py` — 25 unit tests
- `scripts/generate_feeds.py` — batch generation script
- `docs/overnight-status/cameraman.md` — this file
