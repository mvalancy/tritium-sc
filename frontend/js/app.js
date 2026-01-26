/**
 * TRITIUM - Main Application
 * Security Camera Intelligence Platform
 */

// Application State
const state = {
    channels: [],
    selectedChannel: null,
    selectedDate: null,
    videos: [],
    selectedVideo: null,
    currentView: 'grid',
    websocket: null,
    connected: false,
};

// DOM Elements
const elements = {
    channelList: document.getElementById('channel-list'),
    dateList: document.getElementById('date-list'),
    eventList: document.getElementById('event-list'),
    videoGrid: document.getElementById('video-grid'),
    videoList: document.getElementById('video-list'),
    systemTime: document.getElementById('system-time'),
    connectionDot: document.getElementById('connection-dot'),
    connectionText: document.getElementById('connection-text'),
    notifications: document.getElementById('notifications'),
    commandInput: document.getElementById('command-input'),
};

// Initialize application
document.addEventListener('DOMContentLoaded', () => {
    console.log('%c[TRITIUM] Initializing...', 'color: #00f0ff; font-weight: bold;');

    updateSystemTime();
    setInterval(updateSystemTime, 1000);

    initWebSocket();
    loadChannels();
    initKeyboardShortcuts();
    initCommandInput();

    showNotification('SYSTEM', 'TRITIUM v0.1.0 initialized', 'info');
});

// System Time
function updateSystemTime() {
    const now = new Date();
    const utc = now.toISOString().substr(11, 8);
    elements.systemTime.textContent = `${utc} UTC`;
}

// WebSocket Connection
function initWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws/live`;

    console.log(`%c[WS] Connecting to ${wsUrl}`, 'color: #fcee0a;');

    try {
        state.websocket = new WebSocket(wsUrl);

        state.websocket.onopen = () => {
            console.log('%c[WS] Connected', 'color: #05ffa1;');
            state.connected = true;
            updateConnectionStatus(true);
        };

        state.websocket.onclose = () => {
            console.log('%c[WS] Disconnected', 'color: #ff2a6d;');
            state.connected = false;
            updateConnectionStatus(false);
            // Reconnect after 5 seconds
            setTimeout(initWebSocket, 5000);
        };

        state.websocket.onerror = (error) => {
            console.error('[WS] Error:', error);
            updateConnectionStatus(false);
        };

        state.websocket.onmessage = (event) => {
            handleWebSocketMessage(JSON.parse(event.data));
        };
    } catch (error) {
        console.error('[WS] Failed to connect:', error);
        updateConnectionStatus(false);
    }
}

function updateConnectionStatus(connected) {
    elements.connectionDot.classList.toggle('disconnected', !connected);
    elements.connectionText.textContent = connected ? 'UPLINK ACTIVE' : 'DISCONNECTED';
}

function handleWebSocketMessage(message) {
    console.log('[WS] Message:', message);

    switch (message.type) {
        case 'connected':
            showNotification('UPLINK', message.message, 'success');
            break;
        case 'event':
            handleEvent(message.data);
            break;
        case 'alert':
            handleAlert(message.data);
            break;
        case 'camera_status':
            updateCameraStatus(message.camera_id, message.status);
            break;
    }
}

// API Functions
async function apiGet(endpoint) {
    try {
        const response = await fetch(`/api${endpoint}`);
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return await response.json();
    } catch (error) {
        console.error(`[API] GET ${endpoint} failed:`, error);
        throw error;
    }
}

async function apiPost(endpoint, data) {
    try {
        const response = await fetch(`/api${endpoint}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data),
        });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        return await response.json();
    } catch (error) {
        console.error(`[API] POST ${endpoint} failed:`, error);
        throw error;
    }
}

// Channel Management
async function loadChannels() {
    try {
        // Fetch both recorded channels and registered cameras
        const [recordedChannels, registeredCameras, discoveryStatus] = await Promise.all([
            apiGet('/videos/channels').catch(() => []),
            apiGet('/cameras').catch(() => []),
            apiGet('/discovery/status').catch(() => ({ status: 'unknown' })),
        ]);

        // Merge data: recorded channels + registered cameras
        const channelMap = new Map();

        // Add recorded channels
        for (const ch of recordedChannels) {
            channelMap.set(ch.channel, {
                ...ch,
                hasRecordings: true,
                registered: false,
                online: false,
            });
        }

        // Merge registered cameras
        for (const cam of registeredCameras) {
            if (channelMap.has(cam.channel)) {
                const existing = channelMap.get(cam.channel);
                existing.registered = true;
                existing.online = cam.enabled;
                existing.name = cam.name;
                existing.rtsp_url = cam.rtsp_url;
            } else {
                channelMap.set(cam.channel, {
                    channel: cam.channel,
                    name: cam.name,
                    total_videos: 0,
                    hasRecordings: false,
                    registered: true,
                    online: cam.enabled,
                    rtsp_url: cam.rtsp_url,
                });
            }
        }

        state.channels = Array.from(channelMap.values()).sort((a, b) => a.channel - b.channel);
        state.nvrStatus = discoveryStatus;

        renderChannelList();

        if (state.channels.length === 0) {
            showEmptyState('No channels found. Click "Scan NVR" to discover cameras.');
        } else {
            // Auto-select first channel with recordings
            const firstWithRecordings = state.channels.find(c => c.hasRecordings);
            if (firstWithRecordings) {
                selectChannel(firstWithRecordings.channel);
            }
        }

        // Update NVR status display
        updateNvrStatus(discoveryStatus);

    } catch (error) {
        showNotification('ERROR', 'Failed to load channels', 'error');
        showEmptyState('Failed to connect to API');
    }
}

function updateNvrStatus(status) {
    const statusEl = document.getElementById('nvr-status');
    if (!statusEl) return;

    if (status.status === 'connected') {
        statusEl.innerHTML = `<span class="status status-online">NVR: ${status.online}/${status.total_channels}</span>`;
    } else if (status.status === 'not_configured') {
        statusEl.innerHTML = `<span class="status status-warning">NVR: Not configured</span>`;
    } else {
        statusEl.innerHTML = `<span class="status status-offline">NVR: Offline</span>`;
    }
}

async function scanNvr() {
    showNotification('SCANNING', 'Discovering cameras from NVR...', 'info');
    try {
        const result = await apiPost('/discovery/register', {});
        showNotification('DISCOVERY', `Added ${result.added}, updated ${result.updated} cameras`, 'success');
        await loadChannels();
    } catch (error) {
        showNotification('ERROR', 'NVR scan failed', 'error');
    }
}

function renderChannelList() {
    elements.channelList.innerHTML = state.channels.map(ch => {
        const statusClass = ch.online ? 'online' : (ch.hasRecordings ? 'syncing' : 'offline');
        const label = ch.name || `CH${String(ch.channel).padStart(2, '0')}`;
        const meta = ch.hasRecordings ? `${ch.total_videos} files` : (ch.online ? 'LIVE' : 'offline');

        return `
            <li class="channel-item ${state.selectedChannel === ch.channel ? 'active' : ''}"
                onclick="selectChannel(${ch.channel})" data-channel="${ch.channel}">
                <span class="channel-name">
                    <span class="channel-status ${statusClass}"></span>
                    ${label}
                </span>
                <span class="channel-meta">${meta}</span>
            </li>
        `;
    }).join('');
}

async function selectChannel(channel) {
    state.selectedChannel = channel;
    state.selectedDate = null;
    state.videos = [];

    renderChannelList();

    try {
        const dates = await apiGet(`/videos/channels/${channel}/dates`);
        renderDateList(dates);

        if (dates.length > 0) {
            // Auto-select most recent date
            selectDate(dates[0].date);
        }
    } catch (error) {
        showNotification('ERROR', 'Failed to load dates', 'error');
    }

    // If in zones view, initialize zone editor for this channel
    if (state.currentView === 'zones' && typeof initZoneEditorForCamera !== 'undefined') {
        initZoneEditorForCamera(channel);
    }
}

function renderDateList(dates) {
    elements.dateList.innerHTML = dates.map(d => `
        <li class="date-item ${state.selectedDate === d.date ? 'active' : ''}"
            onclick="selectDate('${d.date}')">
            <span>${d.date}</span>
            <span class="date-count">${d.video_count}</span>
        </li>
    `).join('');
}

async function selectDate(date) {
    state.selectedDate = date;
    document.getElementById('timeline-date').textContent = date;

    // Update date list highlighting
    document.querySelectorAll('.date-item').forEach(el => {
        el.classList.toggle('active', el.textContent.includes(date));
    });

    try {
        state.videos = await apiGet(`/videos/channels/${state.selectedChannel}/dates/${date}`);
        renderVideoList();
        updateDayTimeline();

        // Switch to list view when date selected
        switchView('list');
    } catch (error) {
        showNotification('ERROR', 'Failed to load videos', 'error');
    }
}

function renderVideoList() {
    if (state.videos.length === 0) {
        elements.videoList.innerHTML = `
            <div class="empty-state">
                <div class="empty-state-icon">📼</div>
                <div class="empty-state-title">NO RECORDINGS</div>
                <div class="empty-state-message">No video files found for this date.</div>
            </div>
        `;
        return;
    }

    elements.videoList.innerHTML = state.videos.map((video, index) => `
        <div class="video-list-item ${state.selectedVideo === index ? 'active' : ''}"
             onclick="playVideo(${index})">
            <div class="video-thumbnail">
                ${video.timestamp ? formatTime(video.timestamp) : '--:--'}
            </div>
            <div class="video-info">
                <div class="video-filename">${video.filename}</div>
                <div class="video-meta">
                    <span>${formatFileSize(video.size)}</span>
                    <span>${video.timestamp ? new Date(video.timestamp).toLocaleTimeString() : ''}</span>
                </div>
            </div>
        </div>
    `).join('');
}

function updateDayTimeline() {
    const timeline = document.getElementById('day-timeline');
    timeline.innerHTML = '';

    state.videos.forEach(video => {
        if (video.timestamp) {
            const date = new Date(video.timestamp);
            const hour = date.getHours();
            const minute = date.getMinutes();
            const position = ((hour * 60 + minute) / (24 * 60)) * 100;

            const segment = document.createElement('div');
            segment.className = 'timeline-segment';
            segment.style.left = `${position}%`;
            segment.style.width = '2px';
            segment.title = video.filename;
            segment.onclick = () => {
                const index = state.videos.indexOf(video);
                if (index !== -1) playVideo(index);
            };

            timeline.appendChild(segment);
        }
    });
}

// Video Playback
function playVideo(index) {
    state.selectedVideo = index;
    const video = state.videos[index];

    if (!video) return;

    const videoUrl = `/api/videos/stream/${state.selectedChannel}/${state.selectedDate}/${video.filename}`;
    const player = document.getElementById('main-player');

    player.src = videoUrl;
    player.load();
    player.play().catch(e => console.log('Autoplay blocked:', e));

    // Update list highlighting
    document.querySelectorAll('.video-list-item').forEach((el, i) => {
        el.classList.toggle('active', i === index);
    });

    switchView('player');
    showNotification('PLAYING', video.filename, 'info');
}

// View Management
function switchView(view) {
    state.currentView = view;

    // Update buttons
    document.querySelectorAll('.view-controls .btn').forEach(btn => {
        btn.classList.toggle('active', btn.id === `btn-${view}`);
    });

    // Show/hide views
    document.querySelectorAll('.view-content').forEach(el => {
        el.classList.add('hidden');
    });

    const viewEl = document.getElementById(`view-${view}`);
    if (viewEl) {
        viewEl.classList.remove('hidden');
    }

    // Initialize 3D view if needed
    if (view === '3d' && typeof initThreeJS !== 'undefined') {
        initThreeJS();
    }

    // Initialize zones view if needed
    if (view === 'zones' && typeof initZonesView !== 'undefined') {
        initZonesView();
        // If a channel is selected, initialize zone editor for it
        if (state.selectedChannel && typeof initZoneEditorForCamera !== 'undefined') {
            initZoneEditorForCamera(state.selectedChannel);
        }
    }
}

function showEmptyState(message) {
    const emptyEl = document.getElementById('view-empty');
    emptyEl.querySelector('.empty-state-message').textContent = message;
    switchView('empty');
}

// Notifications
function showNotification(title, message, type = 'info') {
    const notification = document.createElement('div');
    notification.className = `notification ${type === 'error' ? 'alert' : ''}`;
    notification.innerHTML = `
        <div class="notification-title text-${type === 'error' ? 'magenta' : type === 'success' ? 'green' : 'cyan'}">${title}</div>
        <div class="notification-message">${message}</div>
    `;

    elements.notifications.appendChild(notification);

    // Remove after 5 seconds
    setTimeout(() => {
        notification.style.opacity = '0';
        notification.style.transform = 'translateX(100%)';
        setTimeout(() => notification.remove(), 300);
    }, 5000);
}

// Section Toggle
function toggleSection(section) {
    const toggle = document.getElementById(`${section}-toggle`);
    if (toggle) {
        toggle.classList.toggle('collapsed');
    }

    // Map section names to list IDs
    const listIdMap = {
        'channels': 'channel-list',
        'dates': 'date-list',
        'events': 'event-list',
        'sidebar-zones': 'sidebar-zone-list',
    };

    const listId = listIdMap[section];
    if (listId) {
        const list = document.getElementById(listId);
        if (list) {
            list.classList.toggle('hidden');
        }
    }
}

// Command Input
function initCommandInput() {
    elements.commandInput.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
            const command = e.target.value.trim();
            if (command) {
                executeCommand(command);
                e.target.value = '';
            }
        }
    });
}

function executeCommand(command) {
    console.log('[CMD]', command);

    // Parse command
    if (command.startsWith('search:')) {
        const query = command.substring(7).trim();
        showNotification('SEARCH', `Searching for: "${query}"`, 'info');
        // Search functionality will be added in Phase 8
    } else {
        showNotification('UNKNOWN', `Command not recognized: ${command}`, 'error');
    }
}

// Keyboard Shortcuts
function initKeyboardShortcuts() {
    document.addEventListener('keydown', (e) => {
        // Don't trigger if typing in input
        if (e.target.tagName === 'INPUT') return;

        switch (e.key) {
            case '1':
                document.getElementById('grid-size').value = '1';
                updateGridSize(1);
                break;
            case '2':
                document.getElementById('grid-size').value = '4';
                updateGridSize(4);
                break;
            case '3':
                document.getElementById('grid-size').value = '9';
                updateGridSize(9);
                break;
            case 'g':
                switchView('grid');
                break;
            case 'p':
                switchView('player');
                break;
            case 'd':
                switchView('3d');
                break;
            case 'z':
                switchView('zones');
                break;
            case '/':
                e.preventDefault();
                elements.commandInput.focus();
                break;
            case 'Escape':
                elements.commandInput.blur();
                break;
        }
    });
}

// Grid Size
document.getElementById('grid-size').addEventListener('change', (e) => {
    updateGridSize(parseInt(e.target.value));
});

function updateGridSize(size) {
    const grid = document.getElementById('video-grid');
    grid.className = `video-grid grid-${size}`;
}

// Event Handlers
function handleEvent(data) {
    console.log('[EVENT]', data);
    // Add to event list
    addEventToList(data);
}

function handleAlert(data) {
    console.log('[ALERT]', data);
    showNotification('ALERT', data.message || 'Detection alert', 'error');
    // Play alert sound (optional)
    playAlertSound();
}

function addEventToList(event) {
    const li = document.createElement('li');
    li.textContent = `${formatTime(event.timestamp)} ${event.type}`;
    elements.eventList.insertBefore(li, elements.eventList.firstChild);

    // Keep only last 10 events
    while (elements.eventList.children.length > 10) {
        elements.eventList.removeChild(elements.eventList.lastChild);
    }
}

function updateCameraStatus(cameraId, status) {
    console.log(`[CAMERA ${cameraId}] Status: ${status}`);
    // Update channel status in list
    const channelItem = document.querySelector(`.channel-item[data-channel="${cameraId}"]`);
    if (channelItem) {
        const statusDot = channelItem.querySelector('.channel-status');
        statusDot.className = `channel-status ${status}`;
    }
}

// Utility Functions
function formatTime(timestamp) {
    if (!timestamp) return '--:--';
    const date = new Date(timestamp);
    return date.toLocaleTimeString('en-US', { hour12: false });
}

function formatFileSize(bytes) {
    if (bytes === 0) return '0 B';
    const k = 1024;
    const sizes = ['B', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
}

function playAlertSound() {
    // Optional: Play a beep sound for alerts
    try {
        const audioContext = new (window.AudioContext || window.webkitAudioContext)();
        const oscillator = audioContext.createOscillator();
        const gainNode = audioContext.createGain();

        oscillator.connect(gainNode);
        gainNode.connect(audioContext.destination);

        oscillator.frequency.value = 440;
        oscillator.type = 'sine';
        gainNode.gain.value = 0.1;

        oscillator.start();
        oscillator.stop(audioContext.currentTime + 0.2);
    } catch (e) {
        // Audio not available
    }
}

// Export for other modules
window.TRITIUM = {
    state,
    selectChannel,
    selectDate,
    playVideo,
    switchView,
    showNotification,
};
