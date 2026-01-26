/**
 * SENTINEL - Video Player
 * Enhanced video playback with timeline and controls
 */

// Player state
const playerState = {
    playing: false,
    currentTime: 0,
    duration: 0,
    volume: 1,
    muted: false,
    playbackRate: 1,
};

// DOM elements
const playerElements = {
    video: null,
    playBtn: null,
    timeline: null,
    timelineProgress: null,
    currentTimeDisplay: null,
    durationDisplay: null,
};

/**
 * Initialize player
 */
document.addEventListener('DOMContentLoaded', () => {
    playerElements.video = document.getElementById('main-player');
    playerElements.playBtn = document.getElementById('play-btn');
    playerElements.timeline = document.getElementById('timeline');
    playerElements.timelineProgress = document.getElementById('timeline-progress');
    playerElements.currentTimeDisplay = document.getElementById('current-time');
    playerElements.durationDisplay = document.getElementById('duration');

    if (playerElements.video) {
        initPlayerEvents();
    }
});

/**
 * Initialize player event listeners
 */
function initPlayerEvents() {
    const video = playerElements.video;

    // Video events
    video.addEventListener('loadedmetadata', () => {
        playerState.duration = video.duration;
        playerElements.durationDisplay.textContent = formatDuration(video.duration);
    });

    video.addEventListener('timeupdate', () => {
        playerState.currentTime = video.currentTime;
        updateTimeDisplay();
        updateProgress();
    });

    video.addEventListener('play', () => {
        playerState.playing = true;
        updatePlayButton();
    });

    video.addEventListener('pause', () => {
        playerState.playing = false;
        updatePlayButton();
    });

    video.addEventListener('ended', () => {
        playerState.playing = false;
        updatePlayButton();
        playNextVideo();
    });

    // Timeline click
    playerElements.timeline.addEventListener('click', (e) => {
        seekTo(e);
    });

    // Keyboard shortcuts for player
    document.addEventListener('keydown', handlePlayerKeyboard);
}

/**
 * Toggle play/pause
 */
function togglePlay() {
    const video = playerElements.video;

    if (video.paused) {
        video.play().catch(e => console.log('Play failed:', e));
    } else {
        video.pause();
    }
}

/**
 * Seek to position on timeline
 */
function seekTo(event) {
    const video = playerElements.video;
    const timeline = playerElements.timeline;
    const rect = timeline.getBoundingClientRect();
    const percent = (event.clientX - rect.left) / rect.width;
    const seekTime = percent * video.duration;

    video.currentTime = seekTime;
}

/**
 * Update play button icon
 */
function updatePlayButton() {
    if (playerElements.playBtn) {
        playerElements.playBtn.textContent = playerState.playing ? '⏸' : '▶';
    }
}

/**
 * Update timeline progress bar
 */
function updateProgress() {
    const video = playerElements.video;
    if (!video.duration) return;

    const percent = (video.currentTime / video.duration) * 100;
    playerElements.timelineProgress.style.width = `${percent}%`;
}

/**
 * Update time display
 */
function updateTimeDisplay() {
    playerElements.currentTimeDisplay.textContent = formatDuration(playerState.currentTime);
}

/**
 * Format duration as HH:MM:SS
 */
function formatDuration(seconds) {
    if (!seconds || isNaN(seconds)) return '00:00:00';

    const hrs = Math.floor(seconds / 3600);
    const mins = Math.floor((seconds % 3600) / 60);
    const secs = Math.floor(seconds % 60);

    return [
        hrs.toString().padStart(2, '0'),
        mins.toString().padStart(2, '0'),
        secs.toString().padStart(2, '0'),
    ].join(':');
}

/**
 * Play next video in list
 */
function playNextVideo() {
    const state = SENTINEL.state;
    if (state.selectedVideo !== null && state.selectedVideo < state.videos.length - 1) {
        SENTINEL.playVideo(state.selectedVideo + 1);
    }
}

/**
 * Play previous video in list
 */
function playPreviousVideo() {
    const state = SENTINEL.state;
    if (state.selectedVideo !== null && state.selectedVideo > 0) {
        SENTINEL.playVideo(state.selectedVideo - 1);
    }
}

/**
 * Set playback rate
 */
function setPlaybackRate(rate) {
    playerState.playbackRate = rate;
    playerElements.video.playbackRate = rate;
    SENTINEL.showNotification('SPEED', `${rate}x`, 'info');
}

/**
 * Skip forward/backward
 */
function skip(seconds) {
    const video = playerElements.video;
    video.currentTime = Math.max(0, Math.min(video.duration, video.currentTime + seconds));
}

/**
 * Toggle fullscreen
 */
function toggleFullscreen() {
    const playerWrapper = document.querySelector('.player-wrapper');

    if (document.fullscreenElement) {
        document.exitFullscreen();
    } else {
        playerWrapper.requestFullscreen().catch(e => {
            console.log('Fullscreen failed:', e);
        });
    }
}

/**
 * Handle keyboard shortcuts for player
 */
function handlePlayerKeyboard(e) {
    // Only handle if player view is active
    if (SENTINEL.state.currentView !== 'player') return;

    // Don't handle if typing in input
    if (e.target.tagName === 'INPUT') return;

    switch (e.key) {
        case ' ':
        case 'k':
            e.preventDefault();
            togglePlay();
            break;

        case 'ArrowLeft':
            e.preventDefault();
            skip(-10);
            break;

        case 'ArrowRight':
            e.preventDefault();
            skip(10);
            break;

        case 'ArrowUp':
            e.preventDefault();
            adjustVolume(0.1);
            break;

        case 'ArrowDown':
            e.preventDefault();
            adjustVolume(-0.1);
            break;

        case 'j':
            skip(-10);
            break;

        case 'l':
            skip(10);
            break;

        case 'm':
            toggleMute();
            break;

        case 'f':
            toggleFullscreen();
            break;

        case 'n':
            playNextVideo();
            break;

        case 'b':
            playPreviousVideo();
            break;

        case '>':
            if (e.shiftKey) {
                setPlaybackRate(Math.min(2, playerState.playbackRate + 0.25));
            }
            break;

        case '<':
            if (e.shiftKey) {
                setPlaybackRate(Math.max(0.25, playerState.playbackRate - 0.25));
            }
            break;

        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            // Seek to percentage of video
            const percent = parseInt(e.key) / 10;
            playerElements.video.currentTime = playerState.duration * percent;
            break;
    }
}

/**
 * Adjust volume
 */
function adjustVolume(delta) {
    const video = playerElements.video;
    video.volume = Math.max(0, Math.min(1, video.volume + delta));
    playerState.volume = video.volume;
    SENTINEL.showNotification('VOLUME', `${Math.round(video.volume * 100)}%`, 'info');
}

/**
 * Toggle mute
 */
function toggleMute() {
    const video = playerElements.video;
    video.muted = !video.muted;
    playerState.muted = video.muted;
    SENTINEL.showNotification('AUDIO', video.muted ? 'MUTED' : 'UNMUTED', 'info');
}

/**
 * Add event markers to timeline
 */
function addTimelineEvent(time, type) {
    const duration = playerState.duration;
    if (!duration) return;

    const percent = (time / duration) * 100;
    const marker = document.createElement('div');
    marker.className = 'timeline-event';
    marker.style.left = `${percent}%`;
    marker.title = `${type} at ${formatDuration(time)}`;

    document.getElementById('timeline-events').appendChild(marker);
}

/**
 * Clear timeline events
 */
function clearTimelineEvents() {
    document.getElementById('timeline-events').innerHTML = '';
}

// Export functions
window.togglePlay = togglePlay;
window.seekTo = seekTo;
window.playNextVideo = playNextVideo;
window.playPreviousVideo = playPreviousVideo;
window.setPlaybackRate = setPlaybackRate;
window.toggleFullscreen = toggleFullscreen;
window.addTimelineEvent = addTimelineEvent;
window.clearTimelineEvents = clearTimelineEvents;
