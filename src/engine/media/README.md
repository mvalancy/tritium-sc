# Media Recording

**Where you are:** `tritium-sc/src/engine/media/`

**Parent:** [../](../) | [../../../CLAUDE.md](../../../CLAUDE.md)

## What This Is

The media subsystem handles video recording from camera feeds. It captures JPEG frames into time-segmented directories, indexes them by timestamp and camera ID, and provides an API for listing and playing back recorded segments.

## Key Files

| File | Purpose |
|------|---------|
| `recorder.py` | VideoRecordingManager — records camera JPEG frames into time-segmented directories |

## Storage Layout

```
{storage_root}/
    {camera_id}/
        {YYYY-MM-DD}/
            {HH-MM-SS}/
                meta.json       — segment metadata
                frame_0000.jpg  — sequential JPEG frames
                frame_0001.jpg
                ...
```

## Related

- [../../../plugins/camera_feeds/](../../../plugins/camera_feeds/) — Camera feeds plugin that produces frames
- [../synthetic/video_gen.py](../synthetic/video_gen.py) — Synthetic video generation for demo mode
- [../../app/routers/](../../app/routers/) — API endpoints for recording control and playback
