"""Video browsing and streaming endpoints."""

import re
from datetime import datetime
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from pydantic import BaseModel

from app.config import settings

router = APIRouter(prefix="/api/videos", tags=["videos"])


class VideoFile(BaseModel):
    """Schema for a video file."""

    channel: int
    filename: str
    path: str
    size: int
    duration_seconds: Optional[int] = None
    timestamp: Optional[datetime] = None
    date: str


class DateFolder(BaseModel):
    """Schema for a date folder."""

    channel: int
    date: str
    video_count: int


class ChannelInfo(BaseModel):
    """Schema for channel information."""

    channel: int
    name: str
    path: str
    date_count: int
    total_videos: int


def parse_video_timestamp(filename: str) -> Optional[datetime]:
    """Parse timestamp from video filename.

    Expected format: fragment_XX_YYYYMMDDHHMMSS.mp4 or YYYYMMDDHHMMSS.mp4
    """
    match = re.search(r"(\d{14})", filename)
    if match:
        try:
            return datetime.strptime(match.group(1), "%Y%m%d%H%M%S")
        except ValueError:
            pass
    return None


def get_channel_dirs() -> list[Path]:
    """Get all channel directories from recordings path.

    Supports formats:
    - channel_01, channel_02, etc.
    - channel1, channel2, etc.
    - ch01, ch02, etc.
    """
    recordings_path = settings.recordings_path
    if not recordings_path.exists():
        return []

    channel_dirs = []
    for item in recordings_path.iterdir():
        if item.is_dir():
            # Match channel_01, channel_1, channel1, ch01, ch1
            if re.match(r"^(ch|channel)[_]?\d+$", item.name, re.IGNORECASE):
                channel_dirs.append(item)

    return sorted(channel_dirs, key=lambda x: parse_channel_number(x.name))


def parse_channel_number(dirname: str) -> int:
    """Extract channel number from directory name."""
    match = re.search(r"(\d+)", dirname)
    return int(match.group(1)) if match else 0


def find_channel_dir(channel: int) -> Optional[Path]:
    """Find channel directory by number."""
    recordings_path = settings.recordings_path

    # Try various naming formats
    formats = [
        f"channel_{channel:02d}",
        f"channel_{channel}",
        f"channel{channel:02d}",
        f"channel{channel}",
        f"ch{channel:02d}",
        f"ch{channel}",
    ]

    for fmt in formats:
        candidate = recordings_path / fmt
        if candidate.exists():
            return candidate

    return None


def get_date_dirs(channel_dir: Path) -> list[tuple[str, Path]]:
    """Get all date directories from a channel directory.

    Handles both formats:
    - YYYY-MM-DD (flat)
    - YYYY/MM/DD (nested)

    Returns list of (date_string, path) tuples.
    """
    dates = []

    for item in channel_dir.iterdir():
        if not item.is_dir():
            continue

        # Check for flat YYYY-MM-DD format
        if re.match(r"^\d{4}-\d{2}-\d{2}$", item.name):
            dates.append((item.name, item))

        # Check for nested YYYY/MM/DD format
        elif re.match(r"^\d{4}$", item.name):  # Year directory
            for month_dir in item.iterdir():
                if month_dir.is_dir() and re.match(r"^\d{2}$", month_dir.name):
                    for day_dir in month_dir.iterdir():
                        if day_dir.is_dir() and re.match(r"^\d{2}$", day_dir.name):
                            date_str = f"{item.name}-{month_dir.name}-{day_dir.name}"
                            dates.append((date_str, day_dir))

    return sorted(dates, key=lambda x: x[0], reverse=True)


def find_date_dir(channel_dir: Path, date: str) -> Optional[Path]:
    """Find date directory by date string (YYYY-MM-DD format)."""
    # Try flat format first
    flat_path = channel_dir / date
    if flat_path.exists():
        return flat_path

    # Try nested YYYY/MM/DD format
    try:
        parts = date.split("-")
        if len(parts) == 3:
            nested_path = channel_dir / parts[0] / parts[1] / parts[2]
            if nested_path.exists():
                return nested_path
    except Exception:
        pass

    return None


@router.get("/channels", response_model=list[ChannelInfo])
async def list_channels():
    """List all available channels with their video counts."""
    channels = []

    for channel_dir in get_channel_dirs():
        channel_num = parse_channel_number(channel_dir.name)
        date_dirs = get_date_dirs(channel_dir)
        total_videos = sum(
            len(list(path.glob("*.mp4"))) for _, path in date_dirs
        )

        channels.append(
            ChannelInfo(
                channel=channel_num,
                name=f"Channel {channel_num}",
                path=str(channel_dir),
                date_count=len(date_dirs),
                total_videos=total_videos,
            )
        )

    return channels


@router.get("/channels/{channel}/dates", response_model=list[DateFolder])
async def list_dates(channel: int):
    """List all available dates for a channel."""
    channel_dir = find_channel_dir(channel)
    if not channel_dir:
        raise HTTPException(status_code=404, detail="Channel not found")

    dates = []
    for date_str, date_path in get_date_dirs(channel_dir):
        video_count = len(list(date_path.glob("*.mp4")))
        if video_count > 0:  # Only include dates with videos
            dates.append(
                DateFolder(channel=channel, date=date_str, video_count=video_count)
            )

    return dates


@router.get("/channels/{channel}/dates/{date}", response_model=list[VideoFile])
async def list_videos(channel: int, date: str):
    """List all videos for a specific channel and date."""
    # Validate date format
    if not re.match(r"^\d{4}-\d{2}-\d{2}$", date):
        raise HTTPException(status_code=400, detail="Invalid date format (use YYYY-MM-DD)")

    channel_dir = find_channel_dir(channel)
    if not channel_dir:
        raise HTTPException(status_code=404, detail="Channel not found")

    date_dir = find_date_dir(channel_dir, date)
    if not date_dir:
        raise HTTPException(status_code=404, detail="Date not found")

    recordings_path = settings.recordings_path
    videos = []

    for video_file in sorted(date_dir.glob("*.mp4")):
        stat = video_file.stat()
        timestamp = parse_video_timestamp(video_file.name)

        videos.append(
            VideoFile(
                channel=channel,
                filename=video_file.name,
                path=str(video_file.relative_to(recordings_path)),
                size=stat.st_size,
                timestamp=timestamp,
                date=date,
            )
        )

    return videos


@router.get("/stream/{channel}/{date}/{filename}")
async def stream_video(channel: int, date: str, filename: str):
    """Stream a video file."""
    # Validate inputs
    if not re.match(r"^\d{4}-\d{2}-\d{2}$", date):
        raise HTTPException(status_code=400, detail="Invalid date format")

    if not filename.endswith(".mp4"):
        raise HTTPException(status_code=400, detail="Invalid file type")

    # Sanitize filename to prevent directory traversal
    if "/" in filename or "\\" in filename or ".." in filename:
        raise HTTPException(status_code=400, detail="Invalid filename")

    channel_dir = find_channel_dir(channel)
    if not channel_dir:
        raise HTTPException(status_code=404, detail="Channel not found")

    date_dir = find_date_dir(channel_dir, date)
    if not date_dir:
        raise HTTPException(status_code=404, detail="Date not found")

    video_path = date_dir / filename
    if not video_path.exists():
        raise HTTPException(status_code=404, detail="Video not found")

    return FileResponse(
        video_path,
        media_type="video/mp4",
        filename=filename,
    )


@router.get("/thumbnail/{channel}/{date}/{filename}")
async def get_thumbnail(channel: int, date: str, filename: str):
    """Get thumbnail for a video (generated on demand, cached)."""
    import cv2

    # Validate inputs
    if not re.match(r"^\d{4}-\d{2}-\d{2}$", date):
        raise HTTPException(status_code=400, detail="Invalid date format")

    if not filename.endswith(".mp4"):
        raise HTTPException(status_code=400, detail="Invalid file type")

    if "/" in filename or "\\" in filename or ".." in filename:
        raise HTTPException(status_code=400, detail="Invalid filename")

    channel_dir = find_channel_dir(channel)
    if not channel_dir:
        raise HTTPException(status_code=404, detail="Channel not found")

    date_dir = find_date_dir(channel_dir, date)
    if not date_dir:
        raise HTTPException(status_code=404, detail="Date not found")

    video_path = date_dir / filename
    if not video_path.exists():
        raise HTTPException(status_code=404, detail="Video not found")

    # Check cache
    cache_dir = settings.recordings_path / ".cache" / "thumbnails" / "videos"
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_path = cache_dir / f"{channel}_{date}_{filename}.jpg"

    if not cache_path.exists():
        # Generate thumbnail from first frame
        cap = cv2.VideoCapture(str(video_path))
        ret, frame = cap.read()
        cap.release()

        if not ret:
            raise HTTPException(status_code=500, detail="Could not read video")

        # Resize to reasonable thumbnail size
        height, width = frame.shape[:2]
        max_width = 320
        if width > max_width:
            scale = max_width / width
            frame = cv2.resize(frame, (max_width, int(height * scale)))

        cv2.imwrite(str(cache_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

    return FileResponse(cache_path, media_type="image/jpeg")
