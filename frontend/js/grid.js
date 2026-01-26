/**
 * TRITIUM - 3D Camera Grid with Live Previews
 * Interactive Three.js visualization for camera spatial layout
 */

let threeInitialized = false;
let scene, camera, renderer, controls, dragControls;
let cameraCards = [];
let animationId = null;
let selectedCard = null;
let cameraTextures = {};  // Cache for camera preview textures

// Cyberpunk color palette
const COLORS = {
    cyan: 0x00f0ff,
    magenta: 0xff2a6d,
    green: 0x05ffa1,
    yellow: 0xfcee0a,
    void: 0x0a0a0f,
    dark: 0x12121a,
    gray: 0x1a1a2e,
};

// Camera position storage
let cameraPositions = JSON.parse(localStorage.getItem('tritium_camera_positions') || '{}');

/**
 * Initialize Three.js scene
 */
function initThreeJS() {
    if (threeInitialized) {
        animate();
        return;
    }

    const canvas = document.getElementById('three-canvas');
    if (!canvas) return;

    console.log('%c[THREE] Initializing 3D property view...', 'color: #00f0ff;');

    // Scene - represents your property from above
    scene = new THREE.Scene();
    scene.background = new THREE.Color(COLORS.void);

    // Camera - top-down isometric view
    const aspect = canvas.clientWidth / canvas.clientHeight;
    camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 1000);
    camera.position.set(0, 15, 15);
    camera.lookAt(0, 0, 0);

    // Renderer
    renderer = new THREE.WebGLRenderer({
        canvas: canvas,
        antialias: true,
        alpha: true,
    });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    // Orbit controls - let user rotate/pan/zoom the view
    if (typeof THREE.OrbitControls !== 'undefined') {
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.05;
        controls.maxPolarAngle = Math.PI / 2.2;  // Prevent going below ground
        controls.minDistance = 5;
        controls.maxDistance = 50;
    }

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 20, 10);
    scene.add(directionalLight);

    // Create property layout
    createPropertyGrid();
    createCameraCards();
    createInfoPanel();

    // Handle resize
    window.addEventListener('resize', onWindowResize);

    // Setup drag controls for camera positioning
    setupDragControls(canvas);

    threeInitialized = true;
    animate();

    // Load camera previews
    loadCameraPreviews();
}

/**
 * Create property grid (represents your property from above)
 */
function createPropertyGrid() {
    // Ground plane
    const groundGeometry = new THREE.PlaneGeometry(30, 30);
    const groundMaterial = new THREE.MeshBasicMaterial({
        color: 0x0d0d15,
        side: THREE.DoubleSide,
    });
    const ground = new THREE.Mesh(groundGeometry, groundMaterial);
    ground.rotation.x = -Math.PI / 2;
    ground.position.y = -0.01;
    scene.add(ground);

    // Grid lines
    const gridHelper = new THREE.GridHelper(30, 30, COLORS.cyan, 0x1a1a2e);
    gridHelper.material.opacity = 0.3;
    gridHelper.material.transparent = true;
    scene.add(gridHelper);

    // Add house representation (simple box)
    const houseGeometry = new THREE.BoxGeometry(6, 3, 8);
    const houseMaterial = new THREE.MeshBasicMaterial({
        color: 0x1a1a2e,
        transparent: true,
        opacity: 0.7,
    });
    const house = new THREE.Mesh(houseGeometry, houseMaterial);
    house.position.set(0, 1.5, 0);
    scene.add(house);

    // House edges
    const houseEdges = new THREE.EdgesGeometry(houseGeometry);
    const houseLine = new THREE.LineSegments(houseEdges, new THREE.LineBasicMaterial({
        color: COLORS.cyan,
        opacity: 0.5,
        transparent: true,
    }));
    houseLine.position.copy(house.position);
    scene.add(houseLine);

    // Add driveway
    const driveGeometry = new THREE.PlaneGeometry(3, 8);
    const driveMaterial = new THREE.MeshBasicMaterial({
        color: 0x15151f,
        side: THREE.DoubleSide,
    });
    const driveway = new THREE.Mesh(driveGeometry, driveMaterial);
    driveway.rotation.x = -Math.PI / 2;
    driveway.position.set(-6, 0.01, 0);
    scene.add(driveway);

    // Labels
    addTextLabel('HOUSE', 0, 3.5, 0);
    addTextLabel('DRIVEWAY', -6, 0.1, 0);
}

/**
 * Add text label to scene
 */
function addTextLabel(text, x, y, z) {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');

    ctx.fillStyle = 'transparent';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.font = 'bold 24px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = '#00f0ff';
    ctx.globalAlpha = 0.5;
    ctx.fillText(text, canvas.width / 2, canvas.height / 2);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
    const sprite = new THREE.Sprite(material);
    sprite.scale.set(3, 0.75, 1);
    sprite.position.set(x, y, z);
    scene.add(sprite);
}

/**
 * Create camera cards with preview capability
 */
function createCameraCards() {
    const channels = TRITIUM.state.channels;
    const cardWidth = 3;
    const cardHeight = 2;

    // Default positions in a circle around the property
    const defaultPositions = [
        { x: -10, z: -8 },   // CH01 - back left
        { x: 10, z: -8 },    // CH02 - back right
        { x: -10, z: 8 },    // CH03 - front left
        { x: 10, z: 8 },     // CH04 - front right
        { x: 0, z: -12 },    // CH05 - back center
        { x: 0, z: 12 },     // CH06 - front center
        { x: -12, z: 0 },    // CH07 - left side
        { x: 12, z: 0 },     // CH08 - right side
    ];

    channels.forEach((channel, index) => {
        // Use saved position or default
        const savedPos = cameraPositions[`ch${channel.channel}`];
        const defaultPos = defaultPositions[index] || { x: (index % 4 - 1.5) * 5, z: Math.floor(index / 4) * 5 - 5 };
        const pos = savedPos || defaultPos;

        // Card group
        const cardGroup = new THREE.Group();
        cardGroup.position.set(pos.x, 2, pos.z);
        cardGroup.userData = {
            channel: channel.channel,
            name: channel.name,
            index,
            isDraggable: true,
        };

        // Camera preview plane (will be textured with actual footage)
        const previewGeometry = new THREE.PlaneGeometry(cardWidth, cardHeight);
        const previewMaterial = new THREE.MeshBasicMaterial({
            color: 0x1a1a2e,
            side: THREE.DoubleSide,
        });
        const preview = new THREE.Mesh(previewGeometry, previewMaterial);
        preview.userData.isPreview = true;
        cardGroup.add(preview);

        // Card border (neon glow)
        const borderGeometry = new THREE.EdgesGeometry(previewGeometry);
        const borderMaterial = new THREE.LineBasicMaterial({
            color: COLORS.cyan,
            linewidth: 2,
        });
        const border = new THREE.LineSegments(borderGeometry, borderMaterial);
        cardGroup.add(border);

        // Status indicator (online/offline)
        const statusColor = channel.enabled ? COLORS.green : COLORS.magenta;
        const statusGeometry = new THREE.CircleGeometry(0.15, 16);
        const statusMaterial = new THREE.MeshBasicMaterial({ color: statusColor });
        const status = new THREE.Mesh(statusGeometry, statusMaterial);
        status.position.set(cardWidth / 2 - 0.25, cardHeight / 2 - 0.25, 0.01);
        cardGroup.add(status);

        // Channel label
        const labelCanvas = document.createElement('canvas');
        labelCanvas.width = 256;
        labelCanvas.height = 64;
        const ctx = labelCanvas.getContext('2d');
        ctx.fillStyle = '#0a0a0f';
        ctx.fillRect(0, 0, labelCanvas.width, labelCanvas.height);
        ctx.font = 'bold 28px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = '#00f0ff';
        ctx.fillText(`CH${String(channel.channel).padStart(2, '0')} - ${channel.name || 'Camera'}`, labelCanvas.width / 2, labelCanvas.height / 2);

        const labelTexture = new THREE.CanvasTexture(labelCanvas);
        const labelMaterial = new THREE.SpriteMaterial({ map: labelTexture });
        const label = new THREE.Sprite(labelMaterial);
        label.scale.set(3, 0.6, 1);
        label.position.set(0, -cardHeight / 2 - 0.5, 0);
        cardGroup.add(label);

        // View cone (shows camera field of view direction)
        const coneGeometry = new THREE.ConeGeometry(1.5, 4, 4);
        const coneMaterial = new THREE.MeshBasicMaterial({
            color: COLORS.cyan,
            transparent: true,
            opacity: 0.15,
            wireframe: true,
        });
        const cone = new THREE.Mesh(coneGeometry, coneMaterial);
        cone.rotation.x = Math.PI / 2;
        cone.position.set(0, 0, 2.5);
        cardGroup.add(cone);

        // Make card face camera
        cardGroup.lookAt(0, cardGroup.position.y, 0);
        cardGroup.rotation.x = 0;  // Keep upright

        scene.add(cardGroup);
        cameraCards.push(cardGroup);
    });
}

/**
 * Create info panel for selected camera
 */
function createInfoPanel() {
    // Info panel in corner of screen (HTML overlay)
    const panel = document.createElement('div');
    panel.id = 'camera-info-panel';
    panel.className = 'neon-box';
    panel.style.cssText = `
        position: absolute;
        bottom: 20px;
        left: 20px;
        padding: 15px;
        background: rgba(10, 10, 15, 0.9);
        border: 1px solid #00f0ff;
        color: #00f0ff;
        font-family: 'JetBrains Mono', monospace;
        font-size: 12px;
        z-index: 100;
        display: none;
        max-width: 250px;
    `;
    panel.innerHTML = `
        <div style="color: #ff2a6d; font-weight: bold; margin-bottom: 10px;">CAMERA INFO</div>
        <div id="camera-info-content"></div>
        <div style="margin-top: 10px; font-size: 10px; color: #666;">
            Drag to reposition • Click to select
        </div>
    `;

    const container = document.getElementById('three-canvas').parentElement;
    container.style.position = 'relative';
    container.appendChild(panel);
}

/**
 * Setup drag controls for repositioning cameras
 */
function setupDragControls(canvas) {
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();
    const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -2);  // Horizontal plane at y=2
    const intersection = new THREE.Vector3();
    let isDragging = false;
    let draggedCard = null;

    canvas.addEventListener('mousedown', (event) => {
        updateMouse(event);
        raycaster.setFromCamera(mouse, camera);

        const intersects = raycaster.intersectObjects(cameraCards, true);
        if (intersects.length > 0) {
            let card = intersects[0].object;
            while (card.parent && !card.userData.channel) {
                card = card.parent;
            }
            if (card.userData.channel) {
                isDragging = true;
                draggedCard = card;
                if (controls) controls.enabled = false;  // Disable orbit while dragging
                selectCamera(card);
            }
        }
    });

    canvas.addEventListener('mousemove', (event) => {
        updateMouse(event);

        if (isDragging && draggedCard) {
            raycaster.setFromCamera(mouse, camera);
            if (raycaster.ray.intersectPlane(plane, intersection)) {
                draggedCard.position.x = intersection.x;
                draggedCard.position.z = intersection.z;

                // Update view cone direction
                draggedCard.lookAt(0, draggedCard.position.y, 0);
                draggedCard.rotation.x = 0;
            }
        } else {
            // Hover effect
            raycaster.setFromCamera(mouse, camera);
            const intersects = raycaster.intersectObjects(cameraCards, true);

            cameraCards.forEach(card => {
                const border = card.children[1];
                if (border && border.material) {
                    border.material.color.setHex(COLORS.cyan);
                }
            });

            if (intersects.length > 0) {
                let card = intersects[0].object;
                while (card.parent && !card.userData.channel) {
                    card = card.parent;
                }
                if (card.userData.channel) {
                    const border = card.children[1];
                    if (border && border.material) {
                        border.material.color.setHex(COLORS.magenta);
                    }
                    canvas.style.cursor = 'grab';
                }
            } else {
                canvas.style.cursor = 'default';
            }
        }
    });

    canvas.addEventListener('mouseup', () => {
        if (isDragging && draggedCard) {
            // Save position
            cameraPositions[`ch${draggedCard.userData.channel}`] = {
                x: draggedCard.position.x,
                z: draggedCard.position.z,
            };
            localStorage.setItem('tritium_camera_positions', JSON.stringify(cameraPositions));
            TRITIUM.showNotification('SAVED', `Camera ${draggedCard.userData.channel} position saved`, 'info');
        }
        isDragging = false;
        draggedCard = null;
        if (controls) controls.enabled = true;
    });

    canvas.addEventListener('dblclick', (event) => {
        updateMouse(event);
        raycaster.setFromCamera(mouse, camera);
        const intersects = raycaster.intersectObjects(cameraCards, true);

        if (intersects.length > 0) {
            let card = intersects[0].object;
            while (card.parent && !card.userData.channel) {
                card = card.parent;
            }
            if (card.userData.channel) {
                // Double-click opens camera in list view
                TRITIUM.selectChannel(card.userData.channel);
                TRITIUM.switchView('list');
            }
        }
    });

    function updateMouse(event) {
        const rect = canvas.getBoundingClientRect();
        mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
    }
}

/**
 * Select a camera and show info
 */
function selectCamera(card) {
    selectedCard = card;

    // Update info panel
    const panel = document.getElementById('camera-info-panel');
    const content = document.getElementById('camera-info-content');
    if (panel && content) {
        panel.style.display = 'block';
        content.innerHTML = `
            <div><strong>Channel:</strong> ${card.userData.channel}</div>
            <div><strong>Name:</strong> ${card.userData.name || 'Camera ' + card.userData.channel}</div>
            <div><strong>Position:</strong> (${card.position.x.toFixed(1)}, ${card.position.z.toFixed(1)})</div>
            <div style="margin-top: 10px;">
                <button onclick="resetCameraPosition(${card.userData.channel})"
                        style="background: #1a1a2e; color: #00f0ff; border: 1px solid #00f0ff; padding: 5px 10px; cursor: pointer;">
                    Reset Position
                </button>
            </div>
        `;
    }

    // Highlight selected card
    cameraCards.forEach(c => {
        const border = c.children[1];
        if (border && border.material) {
            border.material.color.setHex(c === card ? COLORS.yellow : COLORS.cyan);
        }
    });
}

/**
 * Reset camera position to default
 */
window.resetCameraPosition = function(channel) {
    delete cameraPositions[`ch${channel}`];
    localStorage.setItem('tritium_camera_positions', JSON.stringify(cameraPositions));

    // Recreate cards
    cameraCards.forEach(card => scene.remove(card));
    cameraCards = [];
    createCameraCards();
    loadCameraPreviews();

    TRITIUM.showNotification('RESET', `Camera ${channel} position reset`, 'info');
};

/**
 * Load camera preview images
 */
async function loadCameraPreviews() {
    const loader = new THREE.TextureLoader();

    for (const card of cameraCards) {
        const channel = card.userData.channel;

        try {
            // Try to get a recent frame from the API
            const response = await fetch(`/api/videos/${channel}/dates`);
            if (response.ok) {
                const dates = await response.json();
                if (dates.length > 0) {
                    const latestDate = dates[0];
                    const videosResponse = await fetch(`/api/videos/${channel}/${latestDate}`);
                    if (videosResponse.ok) {
                        const videos = await videosResponse.json();
                        if (videos.length > 0) {
                            // Get thumbnail from latest video
                            const thumbUrl = `/api/videos/${channel}/${latestDate}/${videos[0].filename}/thumbnail`;

                            loader.load(thumbUrl, (texture) => {
                                const preview = card.children.find(c => c.userData.isPreview);
                                if (preview) {
                                    preview.material = new THREE.MeshBasicMaterial({
                                        map: texture,
                                        side: THREE.DoubleSide,
                                    });
                                }
                            }, undefined, (err) => {
                                console.log(`No thumbnail for CH${channel}`);
                            });
                        }
                    }
                }
            }
        } catch (e) {
            console.log(`Could not load preview for CH${channel}:`, e.message);
        }
    }
}

/**
 * Animation loop
 */
function animate() {
    if (TRITIUM.state.currentView !== '3d') {
        cancelAnimationFrame(animationId);
        return;
    }

    animationId = requestAnimationFrame(animate);

    // Update orbit controls
    if (controls) {
        controls.update();
    }

    // Subtle animation for cards
    const time = Date.now() * 0.001;
    cameraCards.forEach((card, index) => {
        // Gentle bob
        card.position.y = 2 + Math.sin(time * 0.5 + index) * 0.05;
    });

    renderer.render(scene, camera);
}

/**
 * Handle window resize
 */
function onWindowResize() {
    const canvas = document.getElementById('three-canvas');
    if (!canvas) return;

    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
}

/**
 * Clean up Three.js resources
 */
function disposeThreeJS() {
    if (!threeInitialized) return;

    cancelAnimationFrame(animationId);

    scene.traverse((object) => {
        if (object.geometry) object.geometry.dispose();
        if (object.material) {
            if (Array.isArray(object.material)) {
                object.material.forEach(m => m.dispose());
            } else {
                object.material.dispose();
            }
        }
    });

    if (controls) controls.dispose();
    renderer.dispose();

    // Remove info panel
    const panel = document.getElementById('camera-info-panel');
    if (panel) panel.remove();

    threeInitialized = false;
}

// Export for global access
window.initThreeJS = initThreeJS;
window.disposeThreeJS = disposeThreeJS;
