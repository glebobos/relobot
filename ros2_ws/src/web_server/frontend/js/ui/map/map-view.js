import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { rosService } from '../../services/ros-service.js';
import { DynamicLine } from './dynamic-line.js';
import { TOPICS, MSG_TYPES } from '../../shared/constants.js';
import { MapInteractionHandler } from './map-interaction.js';

// Configuration Constants
const ROBOT_SCALE = 0.68;
const ROBOT_FORWARD_OFFSET = 0.20;
const WALL_HEIGHT = 0.35;
const BOUNDARY_HEIGHT = 0.22;
const CELL_SHRINK_FACTOR = 0.96;
const PLAYBACK_DELAY_MS = 200;
const CHASE_CLOSE_TIME = 0.15;
const EMA_DECAY = 0.93;
const EMA_GROWTH = 0.07;
const ROTATION_SMOOTH_FACTOR = 3.0;
const ARRIVAL_DISTANCE = 0.20;
const CUTOFF_TIME_MS = 1500;
const MAP_THROTTLE_RATE = 2000;
const WALL_COLOR = 0x1f242e;
const UNEXPLORED_COLOR = 0x00e676;

export class MapView {
    constructor(containerId) {
        this.containerId = containerId;
        this.mapContainer = document.getElementById(containerId);

        // Callbacks
        this.onPublishCommand = null;
        this.onNavigateToPoint = null;
        this.onZoneDrawModeChange = null;
        this.onNavPointModeChange = null;

        // Three.js Core components
        this.renderer = null;
        this.scene = null;
        this.camera = null;
        this.cameraControls = null;
        this.animationFrameId = null;
        this.resizeListener = null;

        // Map components
        this.mapGroup = null;
        this.floorMesh = null;

        // Cached map geometries/materials & Instanced Meshes
        this.wallGeometry = null;
        this.wallMaterial = null;
        this.wallMesh = null;
        this.unexploredGeometry = null;
        this.unexploredMaterial = null;
        this.unexploredMesh = null;
        this._isUpdatingMap = false;

        // Robot
        this.robotMarker = null;
        this.mixer = null;

        // ROS topic subscriptions list for clean unmounting
        this.topics = [];

        // Layer groups (kept for compatibility with path/obstacle rendering)
        this.previewLayer = new THREE.Group();
        this.mapPolygonLayer = new THREE.Group();
        this.obstacleLayer = new THREE.Group();
        this.zoneLayer = new THREE.Group();
        this.navTargetLayer = new THREE.Group();

        // Dynamic lines for high-frequency updates
        this.previewLine = new DynamicLine(0xd17a00, 5000, 1);
        this.previewLayer.add(this.previewLine.line);

        this.mapPolygonLine = new DynamicLine(0x1a6fcc, 5000, 1);
        this.mapPolygonLayer.add(this.mapPolygonLine.line);

        this.zoneLine = new DynamicLine(0x00e676, 100, 1);
        this.zoneLayer.add(this.zoneLine.line);

        this.obstacleLines = [];

        // Overlay Canvas
        this.overlayCanvas = document.getElementById('zone-draw-canvas');
        this.overlayCtx = this.overlayCanvas ? this.overlayCanvas.getContext('2d') : null;

        // Animation and state
        this.savedCameraState = null;
        this.cameraAnimId = null;
        this.zoneDrawMode = false;
        this.activeZoneCorners = null;

        this.navPointMode = false;
        this.activeNavTarget = null;
        this.arrivalTimer = null;
        this.robotPosition = null;
        this.poseBuffer = [];
        this._chaseSpeed = 0;       // EMA-smoothed speed for constant-velocity rendering
        this._lastAnimTime = 0;     // previous animation frame timestamp

        // Animation states for growing meshes
        this.wallCells = [];
        this.wallResolution = 0.05;
        this.wallCellsMap = new Map();
        this.wallMeshAnimating = false;
        this.wallMeshReady = false;

        this.unexploredCells = [];
        this.unexploredResolution = 0.05;
        this.unexploredCellsMap = new Map();
        this.unexploredMeshAnimating = false;
        this.unexploredMeshReady = false;

        // Obstacles (3D)
        this.obstacleSegments = [];
        this.obstacleSegmentsMap = new Map();
        this.obstacleMesh = null;
        this.obstacleGeometry = null;
        this.obstacleMaterial = null;
        this.obstacleMeshAnimating = false;
        this.obstacleMeshReady = false;

        // Initialize consolidated interaction handler
        this.interactionHandler = new MapInteractionHandler(this);

        this.init();
    }

    init() {
        const containerWidth = this.mapContainer.offsetWidth;
        const containerHeight = this.mapContainer.offsetHeight;

        // Clear container and setup our standard WebGL Canvas
        this.mapContainer.innerHTML = '';
        this.canvas = document.createElement('canvas');
        this.canvas.id = 'scene';
        this.canvas.style.width = '100%';
        this.canvas.style.height = '100%';
        this.canvas.style.display = 'block';
        this.canvas.style.touchAction = 'none';
        this.mapContainer.appendChild(this.canvas);

        // Renderer
        this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, preserveDrawingBuffer: true });
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.setSize(containerWidth, containerHeight, false);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.05;

        // WebGL Context Loss Handling
        this._onContextLost = (event) => {
            event.preventDefault();
            console.warn('[MapView] WebGL Context Lost!');
            if (this.animationFrameId) {
                cancelAnimationFrame(this.animationFrameId);
                this.animationFrameId = null;
            }
        };
        this._onContextRestored = () => {
            console.log('[MapView] WebGL Context Restored! Reinitializing...');
            this._lastAnimTime = performance.now();
            this.animate();
        };
        this.canvas.addEventListener('webglcontextlost', this._onContextLost, false);
        this.canvas.addEventListener('webglcontextrestored', this._onContextRestored, false);

        // Scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0c0f14);

        // Camera - Z is UP in ROS
        this.camera = new THREE.PerspectiveCamera(52, containerWidth / containerHeight, 0.1, 100);
        this.camera.position.set(-3, -5, 5.5);
        this.camera.up.set(0, 0, 1);

        // Camera Controls
        this.cameraControls = new OrbitControls(this.camera, this.renderer.domElement);
        this.cameraControls.enableDamping = true;
        this.cameraControls.maxPolarAngle = Math.PI * 0.46;
        this.cameraControls.minDistance = 2;
        this.cameraControls.maxDistance = 30;
        this.cameraControls.target.set(0, 0, 0);

        // Lights
        const sun = new THREE.DirectionalLight(0xfff5d6, 3.4);
        sun.position.set(-10, -15, 20);
        sun.castShadow = true;
        sun.shadow.mapSize.set(1024, 1024);
        sun.shadow.camera.near = 1;
        sun.shadow.camera.far = 40;
        sun.shadow.camera.left = -15;
        sun.shadow.camera.right = 15;
        sun.shadow.camera.top = 15;
        sun.shadow.camera.bottom = -15;
        this.scene.add(sun);

        const ambient = new THREE.HemisphereLight(0xddefff, 0x2c3427, 1.25);
        this.scene.add(ambient);

        // Map Group
        this.mapGroup = new THREE.Group();
        this.scene.add(this.mapGroup);

        // Add layer groups
        this.scene.add(this.previewLayer);
        this.scene.add(this.mapPolygonLayer);
        this.scene.add(this.obstacleLayer);
        this.scene.add(this.zoneLayer);
        this.scene.add(this.navTargetLayer);

        // Robot marker
        this.robotMarker = new THREE.Group();
        this.scene.add(this.robotMarker);

        // Load GLB Robot Model
        const loader = new GLTFLoader();
        loader.load(
            '/models/robot.glb',
            (gltf) => {
                const robot = gltf.scene;

                // 1. Orient model so that GLTF coordinates match ROS (Z-up, X-forward)
                const q1 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2); // Y-up to Z-up
                const q2 = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2); // Keep default ROS orientation
                robot.quaternion.multiplyQuaternions(q2, q1);

                // 2. Traverse and configure shadows
                robot.traverse((child) => {
                    if (child.isMesh) {
                        child.castShadow = true;
                        child.receiveShadow = true;
                        if (child.material && child.material.map) {
                            child.material.map.colorSpace = THREE.SRGBColorSpace;
                        }
                    }
                });

                // 3. Scale the model dynamically based on rotated bounding box
                const box = new THREE.Box3().setFromObject(robot);
                const size = new THREE.Vector3();
                box.getSize(size);
                const maxDim = Math.max(size.x, size.y, size.z, 0.001);
                const scale = ROBOT_SCALE / maxDim; // Keep size at 0.52
                robot.scale.setScalar(scale);

                // 4. Center model horizontally and offset base to Z = 0, shifting wheels to pivot point
                const scaledBox = new THREE.Box3().setFromObject(robot);
                const center = new THREE.Vector3();
                scaledBox.getCenter(center);

                robot.position.x = -center.x + ROBOT_FORWARD_OFFSET; // Shift forward to align rear wheel axis with the pivot (0,0)
                robot.position.y = -center.y;
                robot.position.z = -scaledBox.min.z;

                this.robotMarker.add(robot);

                // Load animations
                if (gltf.animations.length > 0) {
                    this.mixer = new THREE.AnimationMixer(robot);
                    gltf.animations.forEach((clip) => {
                        this.mixer.clipAction(clip).play();
                    });
                }
                console.log('[MapView] GLB Robot model loaded and normalized successfully.');
            },
            undefined,
            (err) => {
                console.error('[MapView] Failed to load GLB Robot model:', err);
                const fallbackRobot = this.createFallbackRobotModel();
                this.robotMarker.add(fallbackRobot);
                console.log('[MapView] Fallback robot model loaded successfully.');
            }
        );

        this.resizeOverlay();
        this.setupEventListeners();
        this.setupSubscriptions();

        // Start unified animation loop
        this.animate();
    }

    createFallbackRobotModel() {
        const fallbackGroup = new THREE.Group();

        // Base: a simple circular chassis (cylinder) lying flat
        const baseGeom = new THREE.CylinderGeometry(0.26, 0.26, 0.15, 16);
        const baseMat = new THREE.MeshStandardMaterial({
            color: 0x3f51b5, // Sleek indigo
            roughness: 0.4,
            metalness: 0.2
        });
        const baseMesh = new THREE.Mesh(baseGeom, baseMat);
        baseMesh.rotation.x = Math.PI / 2; // Lie flat on Z plane
        baseMesh.position.z = 0.075; // Align bottom wheels to Z = 0
        baseMesh.castShadow = true;
        baseMesh.receiveShadow = true;
        fallbackGroup.add(baseMesh);

        // Heading indicator: a small arrow/cone pointing forward (+X direction in ROS)
        const dirGeom = new THREE.ConeGeometry(0.06, 0.18, 4);
        const dirMat = new THREE.MeshBasicMaterial({ color: 0xffeb3b }); // Bright yellow
        const dirMesh = new THREE.Mesh(dirGeom, dirMat);
        dirMesh.rotation.z = -Math.PI / 2; // Point in +X direction
        dirMesh.position.set(0.18, 0, 0.15); // Offset forward and up
        fallbackGroup.add(dirMesh);

        return fallbackGroup;
    }

    animate() {
        const now = performance.now();
        const dt = this._lastAnimTime ? Math.min((now - this._lastAnimTime) / 1000, 0.05) : 0;
        this._lastAnimTime = now;

        if (this.cameraControls) {
            this.cameraControls.update();
        }

        if (this.mixer) {
            // Speed up wheel animation based on chase speed
            const speedFactor = 0.35 + Math.min(1.5, this._chaseSpeed * 2);
            this.mixer.update(dt * speedFactor);
        }

        this.animateRobotSmoothing(dt);
        this.animateMapGrowth();

        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }

        this.animationFrameId = requestAnimationFrame(() => this.animate());
    }

    animateMapGrowth() {
        const now = performance.now();

        // Animate new floor fade-in and dispose of old floor only after it's complete
        if (this.floorMesh && this.floorMesh.material && this.floorMesh.material.opacity < 1.0) {
            const elapsed = now - this.newFloorStartTime;
            const t = Math.min(elapsed / this.newFloorDuration, 1.0);
            this.floorMesh.material.opacity = t;

            if (t >= 1.0) {
                this.floorMesh.material.transparent = false;
                this.floorMesh.material.needsUpdate = true;
                if (this.oldFloorMesh) {
                    this.mapGroup.remove(this.oldFloorMesh);
                    if (this.oldFloorMesh.geometry) this.oldFloorMesh.geometry.dispose();
                    if (this.oldFloorMesh.material) {
                        if (this.oldFloorMesh.material.map) this.oldFloorMesh.material.map.dispose();
                        this.oldFloorMesh.material.dispose();
                    }
                    this.oldFloorMesh = null;
                }
            }
        }

        if (this.wallMeshAnimating || !this.wallMeshReady) {
            this.updateWallMeshInstances(this.wallResolution);
        }
        if (this.unexploredMeshAnimating || !this.unexploredMeshReady) {
            this.updateUnexploredMeshInstances(this.unexploredResolution);
        }
        if (this.obstacleMeshAnimating || !this.obstacleMeshReady) {
            this.updateObstacleMeshInstances();
        }
    }

    animateRobotSmoothing(dt) {
        if (this.robotMarker && this.poseBuffer && this.poseBuffer.length > 0) {
            const now = performance.now();
            const playbackTime = now - PLAYBACK_DELAY_MS;
            let targetPos = null;
            let targetQuat = null;

            // Find bracketing frames for interpolation
            for (let i = 0; i < this.poseBuffer.length - 1; i++) {
                const a = this.poseBuffer[i], b = this.poseBuffer[i + 1];
                if (a.timestamp <= playbackTime && b.timestamp > playbackTime) {
                    const span = b.timestamp - a.timestamp;
                    const t = span > 0 ? (playbackTime - a.timestamp) / span : 1;
                    targetPos = a.position.clone().lerp(b.position, t);
                    targetQuat = a.quaternion.clone().slerp(b.quaternion, t);
                    break;
                }
            }

            // Fallback when no bracketing frames found
            if (!targetPos) {
                if (playbackTime < this.poseBuffer[0].timestamp) {
                    targetPos = this.poseBuffer[0].position;
                    targetQuat = this.poseBuffer[0].quaternion;
                } else {
                    const last = this.poseBuffer[this.poseBuffer.length - 1];
                    targetPos = last.position;
                    targetQuat = last.quaternion;
                }
            }

            // Move toward target at constant speed
            const dx = targetPos.x - this.robotMarker.position.x;
            const dy = targetPos.y - this.robotMarker.position.y;
            const dist = Math.sqrt(dx * dx + dy * dy);

            const desiredSpeed = dist / CHASE_CLOSE_TIME;
            this._chaseSpeed = this._chaseSpeed * EMA_DECAY + desiredSpeed * EMA_GROWTH;
            const step = this._chaseSpeed * dt;

            if (dist < 0.001) {
                this.robotMarker.position.x = targetPos.x;
                this.robotMarker.position.y = targetPos.y;
            } else if (step >= dist) {
                this.robotMarker.position.x = targetPos.x;
                this.robotMarker.position.y = targetPos.y;
            } else {
                const ratio = step / dist;
                this.robotMarker.position.x += dx * ratio;
                this.robotMarker.position.y += dy * ratio;
            }
            this.robotMarker.position.z = 0.01;

            // Smooth rotation
            this.robotMarker.quaternion.slerp(targetQuat, Math.min(ROTATION_SMOOTH_FACTOR * dt, 1.0));

            // Sync visual position
            this.robotPosition = {
                x: this.robotMarker.position.x,
                y: this.robotMarker.position.y
            };
        }
    }

    arraysEqual(a, b) {
        if (!a || !b) return false;
        if (a.length !== b.length) return false;
        for (let i = 0; i < a.length; i++) {
            if (a[i] !== b[i]) return false;
        }
        return true;
    }

    handleMapUpdate(msg) {
        if (!msg || !msg.info || !msg.data) return;
        if (this._isUpdatingMap) return;

        // Check if map data has actually changed
        const info = msg.info;
        const origin = info.origin;
        if (
            this._lastMapWidth === info.width &&
            this._lastMapHeight === info.height &&
            this._lastMapResolution === info.resolution &&
            this._lastMapOrigin &&
            this._lastMapOrigin.position.x === origin.position.x &&
            this._lastMapOrigin.position.y === origin.position.y &&
            this._lastMapOrigin.orientation.z === origin.orientation.z &&
            this._lastMapOrigin.orientation.w === origin.orientation.w &&
            this._lastMapData &&
            this.arraysEqual(this._lastMapData, msg.data)
        ) {
            return;
        }

        // Cache new map metadata and data
        this._lastMapWidth = info.width;
        this._lastMapHeight = info.height;
        this._lastMapResolution = info.resolution;
        this._lastMapOrigin = {
            position: { x: origin.position.x, y: origin.position.y, z: origin.position.z },
            orientation: { x: origin.orientation.x, y: origin.orientation.y, z: origin.orientation.z, w: origin.orientation.w }
        };
        this._lastMapData = new Int8Array(msg.data);

        this._isUpdatingMap = true;

        try {
            const width = msg.info.width;
            const height = msg.info.height;
            const resolution = msg.info.resolution;
            const data = msg.data;
            const origin = msg.info.origin;

            // Position map group at the origin of the occupancy grid
            this.mapGroup.position.set(origin.position.x, origin.position.y, origin.position.z);
            this.mapGroup.quaternion.set(
                origin.orientation.x,
                origin.orientation.y,
                origin.orientation.z,
                origin.orientation.w
            );

            // Scan the data for occupied, free, and unexplored boundary cells
            const occupiedCells = [];
            const unexploredBoundaryCells = [];

            for (let r = 0; r < height; r++) {
                for (let c = 0; c < width; c++) {
                    const idx = r * width + c;
                    const val = data[idx];

                    if (val === 100) {
                        occupiedCells.push({ r, c });
                    } else if (val === -1) {
                        // Check if it borders a free cell (value === 0)
                        let bordersFree = false;
                        if (r > 0 && data[(r - 1) * width + c] === 0) bordersFree = true;
                        else if (r < height - 1 && data[(r + 1) * width + c] === 0) bordersFree = true;
                        else if (c > 0 && data[r * width + (c - 1)] === 0) bordersFree = true;
                        else if (c < width - 1 && data[r * width + (c + 1)] === 0) bordersFree = true;

                        if (bordersFree) {
                            unexploredBoundaryCells.push({ r, c });
                        }
                    }
                }
            }

            // Draw flat floor texture representing explored free space
            this.updateFloorPlane(width, height, resolution, data);

            // Render 3D Walls (occupied cells)
            this.updateWallMesh(resolution, occupiedCells, origin);

            // Render unexplored boundary cells
            this.updateUnexploredMesh(resolution, unexploredBoundaryCells, origin);
        } finally {
            this._isUpdatingMap = false;
        }
    }

    updateFloorPlane(width, height, resolution, data) {
        const mapWidth = width * resolution;
        const mapHeight = height * resolution;

        // Save current floorMesh as old floor for cross-fade
        if (this.floorMesh) {
            if (this.oldFloorMesh) {
                // If there's an older one still fading, dispose it immediately
                this.mapGroup.remove(this.oldFloorMesh);
                if (this.oldFloorMesh.geometry) this.oldFloorMesh.geometry.dispose();
                if (this.oldFloorMesh.material) {
                    if (this.oldFloorMesh.material.map) this.oldFloorMesh.material.map.dispose();
                    this.oldFloorMesh.material.dispose();
                }
            }
            this.oldFloorMesh = this.floorMesh;
            // Push old floor slightly down in Z so new floor renders on top
            this.oldFloorMesh.position.z = -0.002;
            this.oldFloorMesh.renderOrder = -11;
            
            // Keep old floor fully opaque! No fade out needed!
            if (this.oldFloorMesh.material) {
                this.oldFloorMesh.material.transparent = true;
                this.oldFloorMesh.material.opacity = 1.0;
            }
            this.oldFloorStartTime = performance.now();
            this.oldFloorDuration = 800;
        }

        const size = width * height;
        const textureData = new Uint8Array(4 * size);
        for (let r = 0; r < height; r++) {
            const canvasRow = height - 1 - r; // Flip vertically to match ROS Y-up coordinates
            for (let c = 0; c < width; c++) {
                const mapIdx = r * width + c;
                const texIdx = (canvasRow * width + c) * 4;
                const val = data[mapIdx];

                let red = 12, green = 15, blue = 20; // Default: unexplored (#0c0f14)

                if (val === 0) {
                    // Explored free space: beautiful POC green (#6e9868)
                    red = 110;
                    green = 152;
                    blue = 104;
                } else if (val === 100) {
                    // Occupied space: dark charcoal base
                    red = 20;
                    green = 22;
                    blue = 24;
                }

                // Add a subtle grid/border pattern overlay for high-tech aesthetics directly in buffer
                // (Grid lines every 10 pixels, blending 2% white: val_new = val_old * 0.98 + 5.1)
                const isGridLine = (c % 10 === 0) || (canvasRow % 10 === 0);
                if (isGridLine) {
                    red = Math.round(red * 0.98 + 5.1);
                    green = Math.round(green * 0.98 + 5.1);
                    blue = Math.round(blue * 0.98 + 5.1);
                }

                textureData[texIdx] = red;
                textureData[texIdx + 1] = green;
                textureData[texIdx + 2] = blue;
                textureData[texIdx + 3] = 255;
            }
        }

        const floorTexture = new THREE.DataTexture(textureData, width, height, THREE.RGBAFormat);
        floorTexture.minFilter = THREE.NearestFilter;
        floorTexture.magFilter = THREE.NearestFilter;
        floorTexture.colorSpace = THREE.SRGBColorSpace;
        floorTexture.flipY = true;
        floorTexture.needsUpdate = true;

        const floorGeom = new THREE.PlaneGeometry(mapWidth, mapHeight);
        const floorMat = new THREE.MeshStandardMaterial({
            map: floorTexture,
            roughness: 0.8,
            metalness: 0.05,
            transparent: true,
            opacity: 0.0 // Start fully transparent for fade-in
        });

        this.floorMesh = new THREE.Mesh(floorGeom, floorMat);
        this.floorMesh.receiveShadow = true;
        this.floorMesh.position.set(mapWidth / 2, mapHeight / 2, 0);
        this.floorMesh.renderOrder = -10;
        this.mapGroup.add(this.floorMesh);

        this.newFloorStartTime = performance.now();
        this.newFloorDuration = 800; // 800ms fade in
    }

    easeOutBack(t) {
        const c1 = 2.0;
        const c3 = c1 + 1;
        return 1 + c3 * Math.pow(t - 1, 3) + c1 * Math.pow(t - 1, 2);
    }

    updateWallMesh(resolution, cells, origin) {
        this.wallResolution = resolution;
        this.wallCells = cells;
        this.wallOrigin = origin;

        if (cells.length === 0) {
            if (this.wallMesh) {
                this.mapGroup.remove(this.wallMesh);
                this.wallMesh = null;
            }
            this.wallCellsMap.clear();
            this.wallMeshReady = true;
            this.wallMeshAnimating = false;
            return;
        }

        const size = resolution * CELL_SHRINK_FACTOR;

        // Create or update shared geometry
        if (!this.wallGeometry || this.wallGeometry.parameters.width !== size) {
            if (this.wallGeometry) this.wallGeometry.dispose();
            this.wallGeometry = new THREE.BoxGeometry(size, size, WALL_HEIGHT);
        }
        // Create shared material
        if (!this.wallMaterial) {
            this.wallMaterial = new THREE.MeshStandardMaterial({
                color: WALL_COLOR,
                roughness: 0.5,
                metalness: 0.1
            });
        }

        // Recreate InstancedMesh ONLY when the instance count changes
        let needsRecreate = !this.wallMesh || this.wallMesh.count !== cells.length;
        if (needsRecreate) {
            if (this.wallMesh) {
                this.mapGroup.remove(this.wallMesh);
                this.wallMesh = null;
            }
            this.wallMesh = new THREE.InstancedMesh(this.wallGeometry, this.wallMaterial, cells.length);
            this.wallMesh.castShadow = true;
            this.wallMesh.receiveShadow = true;
            this.wallMesh.frustumCulled = false;
            this.mapGroup.add(this.wallMesh);
        }

        // Update animation tracking map
        const now = performance.now();
        const incomingKeys = new Set();

        for (let i = 0; i < cells.length; i++) {
            const cell = cells[i];
            const cellMapX = origin.position.x + (cell.c + 0.5) * resolution;
            const cellMapY = origin.position.y + (cell.r + 0.5) * resolution;
            const key = cellMapX.toFixed(3) + ',' + cellMapY.toFixed(3);
            incomingKeys.add(key);

            if (!this.wallCellsMap.has(key)) {
                // Determine delay based on distance to robot
                let delay = 0;
                if (this.robotPosition) {
                    const dx = cellMapX - this.robotPosition.x;
                    const dy = cellMapY - this.robotPosition.y;
                    const dist = Math.sqrt(dx * dx + dy * dy);
                    delay = Math.min(dist * 150, 1000); // 150ms per meter, max 1s
                } else {
                    delay = Math.random() * 300;
                }

                this.wallCellsMap.set(key, {
                    r: cell.r,
                    c: cell.c,
                    startTime: now,
                    delay: delay,
                    duration: 600 + Math.random() * 200 // 600-800ms duration
                });
            }
        }

        // Remove stale cells
        for (const key of this.wallCellsMap.keys()) {
            if (!incomingKeys.has(key)) {
                this.wallCellsMap.delete(key);
            }
        }

        this.wallMeshReady = false; // Trigger matrix updates
        this.updateWallMeshInstances(resolution);
    }

    updateWallMeshInstances(resolution) {
        if (!this.wallMesh || !this.wallCells) return;
        const now = performance.now();
        const dummy = new THREE.Object3D();
        let anyAnimating = false;

        for (let i = 0; i < this.wallCells.length; i++) {
            const cell = this.wallCells[i];
            const cellMapX = this.wallOrigin ? (this.wallOrigin.position.x + (cell.c + 0.5) * resolution) : (cell.c + 0.5) * resolution;
            const cellMapY = this.wallOrigin ? (this.wallOrigin.position.y + (cell.r + 0.5) * resolution) : (cell.r + 0.5) * resolution;
            const key = this.wallOrigin ? (cellMapX.toFixed(3) + ',' + cellMapY.toFixed(3)) : (cell.r + ',' + cell.c);
            const anim = this.wallCellsMap.get(key);

            let scaleZ = 1.0;
            if (anim) {
                const elapsed = now - anim.startTime;
                if (elapsed < anim.delay) {
                    scaleZ = 0.0001; // Avoid exact 0 scale to prevent singular matrix warning
                    anyAnimating = true;
                } else {
                    const progress = elapsed - anim.delay;
                    if (progress < anim.duration) {
                        const t = progress / anim.duration;
                        scaleZ = Math.max(0.0001, this.easeOutBack(t));
                        anyAnimating = true;
                    } else {
                        scaleZ = 1.0;
                    }
                }
            }

            dummy.position.set(
                (cell.c + 0.5) * resolution,
                (cell.r + 0.5) * resolution,
                (scaleZ * WALL_HEIGHT) / 2
            );
            dummy.scale.set(1, 1, scaleZ);
            dummy.updateMatrix();
            this.wallMesh.setMatrixAt(i, dummy.matrix);
        }
        this.wallMesh.instanceMatrix.needsUpdate = true;
        this.wallMeshAnimating = anyAnimating;
        this.wallMeshReady = !anyAnimating;
    }

    updateUnexploredMesh(resolution, cells, origin) {
        this.unexploredResolution = resolution;
        this.unexploredCells = cells;
        this.unexploredOrigin = origin;

        if (cells.length === 0) {
            if (this.unexploredMesh) {
                this.mapGroup.remove(this.unexploredMesh);
                this.unexploredMesh = null;
            }
            this.unexploredCellsMap.clear();
            this.unexploredMeshReady = true;
            this.unexploredMeshAnimating = false;
            return;
        }

        const size = resolution * CELL_SHRINK_FACTOR;

        // Create or update shared geometry
        if (!this.unexploredGeometry || this.unexploredGeometry.parameters.width !== size) {
            if (this.unexploredGeometry) this.unexploredGeometry.dispose();
            this.unexploredGeometry = new THREE.BoxGeometry(size, size, BOUNDARY_HEIGHT);
        }
        // Create shared material
        if (!this.unexploredMaterial) {
            this.unexploredMaterial = new THREE.MeshStandardMaterial({
                color: UNEXPLORED_COLOR,
                roughness: 0.1,
                metalness: 0.9,
                transparent: true,
                opacity: 0.32,
                depthWrite: false
            });
        }

        // Recreate InstancedMesh ONLY when the instance count changes
        let needsRecreate = !this.unexploredMesh || this.unexploredMesh.count !== cells.length;
        if (needsRecreate) {
            if (this.unexploredMesh) {
                this.mapGroup.remove(this.unexploredMesh);
                this.unexploredMesh = null;
            }
            this.unexploredMesh = new THREE.InstancedMesh(this.unexploredGeometry, this.unexploredMaterial, cells.length);
            this.unexploredMesh.frustumCulled = false;
            this.unexploredMesh.renderOrder = 10;
            this.mapGroup.add(this.unexploredMesh);
        }

        // Update animation tracking map
        const now = performance.now();
        const incomingKeys = new Set();

        for (let i = 0; i < cells.length; i++) {
            const cell = cells[i];
            const cellMapX = origin.position.x + (cell.c + 0.5) * resolution;
            const cellMapY = origin.position.y + (cell.r + 0.5) * resolution;
            const key = cellMapX.toFixed(3) + ',' + cellMapY.toFixed(3);
            incomingKeys.add(key);

            if (!this.unexploredCellsMap.has(key)) {
                // Determine delay based on distance to robot
                let delay = 0;
                if (this.robotPosition) {
                    const dx = cellMapX - this.robotPosition.x;
                    const dy = cellMapY - this.robotPosition.y;
                    const dist = Math.sqrt(dx * dx + dy * dy);
                    delay = Math.min(dist * 150, 1000); // 150ms per meter, max 1s
                } else {
                    delay = Math.random() * 300;
                }

                this.unexploredCellsMap.set(key, {
                    r: cell.r,
                    c: cell.c,
                    startTime: now,
                    delay: delay,
                    duration: 600 + Math.random() * 200 // 600-800ms duration
                });
            }
        }

        // Remove stale cells
        for (const key of this.unexploredCellsMap.keys()) {
            if (!incomingKeys.has(key)) {
                this.unexploredCellsMap.delete(key);
            }
        }

        this.unexploredMeshReady = false; // Trigger matrix updates
        this.updateUnexploredMeshInstances(resolution);
    }

    updateUnexploredMeshInstances(resolution) {
        if (!this.unexploredMesh || !this.unexploredCells) return;
        const now = performance.now();
        const dummy = new THREE.Object3D();
        let anyAnimating = false;

        for (let i = 0; i < this.unexploredCells.length; i++) {
            const cell = this.unexploredCells[i];
            const cellMapX = this.unexploredOrigin ? (this.unexploredOrigin.position.x + (cell.c + 0.5) * resolution) : (cell.c + 0.5) * resolution;
            const cellMapY = this.unexploredOrigin ? (this.unexploredOrigin.position.y + (cell.r + 0.5) * resolution) : (cell.r + 0.5) * resolution;
            const key = this.unexploredOrigin ? (cellMapX.toFixed(3) + ',' + cellMapY.toFixed(3)) : (cell.r + ',' + cell.c);
            const anim = this.unexploredCellsMap.get(key);

            let scaleZ = 1.0;
            if (anim) {
                const elapsed = now - anim.startTime;
                if (elapsed < anim.delay) {
                    scaleZ = 0.0001;
                    anyAnimating = true;
                } else {
                    const progress = elapsed - anim.delay;
                    if (progress < anim.duration) {
                        const t = progress / anim.duration;
                        scaleZ = Math.max(0.0001, this.easeOutBack(t));
                        anyAnimating = true;
                    } else {
                        scaleZ = 1.0;
                    }
                }
            }

            dummy.position.set(
                (cell.c + 0.5) * resolution,
                (cell.r + 0.5) * resolution,
                (scaleZ * BOUNDARY_HEIGHT) / 2
            );
            dummy.scale.set(1, 1, scaleZ);
            dummy.updateMatrix();
            this.unexploredMesh.setMatrixAt(i, dummy.matrix);
        }
        this.unexploredMesh.instanceMatrix.needsUpdate = true;
        this.unexploredMeshAnimating = anyAnimating;
        this.unexploredMeshReady = !anyAnimating;
    }

    clearGroup(group) {
        while (group.children.length > 0) {
            const child = group.children[0];
            group.remove(child);
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
                if (Array.isArray(child.material)) {
                    child.material.forEach(m => m.dispose());
                } else {
                    child.material.dispose();
                }
            }
        }
    }

    renderPreviewPath(pathMsg) {
        if (!pathMsg || !pathMsg.poses || pathMsg.poses.length === 0) {
            this.previewLine.clear();
            return;
        }

        const pathPoints = pathMsg.poses
            .filter((poseStamped) => {
                const x = poseStamped.pose.position.x;
                const y = poseStamped.pose.position.y;
                return Number.isFinite(x) && Number.isFinite(y);
            })
            .map((poseStamped) => {
                return new THREE.Vector3(
                    poseStamped.pose.position.x,
                    poseStamped.pose.position.y,
                    0.05
                );
            });

        const MAX_COORD = 100.0;
        const hasGlitch = pathPoints.some((p) => Math.abs(p.x) > MAX_COORD || Math.abs(p.y) > MAX_COORD);
        if (hasGlitch) {
            console.warn('[MapView] renderPreviewPath: rejected path containing coordinates exceeding 100m (potential glitch).');
            this.previewLine.clear();
            return;
        }

        if (pathPoints.length < 2) {
            this.previewLine.clear();
            return;
        }
        this.previewLine.updatePoints(pathPoints);
    }

    renderMapPolygon(msg) {
        if (!msg || !msg.polygon || !msg.polygon.points || msg.polygon.points.length < 3) {
            this.mapPolygonLine.clear();
            return;
        }

        const pts = msg.polygon.points
            .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
            .map((p) => new THREE.Vector3(p.x, p.y, 0.06));

        const MAX_COORD = 100.0;
        const hasGlitch = pts.some((p) => Math.abs(p.x) > MAX_COORD || Math.abs(p.y) > MAX_COORD);
        if (hasGlitch) {
            console.warn('[MapView] renderMapPolygon: rejected polygon containing coordinates exceeding 100m (potential glitch).');
            this.mapPolygonLine.clear();
            return;
        }

        if (pts.length < 3) {
            this.mapPolygonLine.clear();
            return;
        }

        const first = pts[0], last = pts[pts.length - 1];
        if (Math.abs(first.x - last.x) > 1e-4 || Math.abs(first.y - last.y) > 1e-4) {
            pts.push(first.clone());
        }

        this.mapPolygonLine.updatePoints(pts);
    }

    renderObstacles(msg) {
        if (!msg || !msg.data) {
            this.obstacleLines.forEach(l => l.clear());
            if (this.obstacleMesh) {
                this.obstacleLayer.remove(this.obstacleMesh);
                this.obstacleMesh = null;
            }
            this.obstacleSegments = [];
            this.obstacleSegmentsMap.clear();
            this.obstacleMeshReady = true;
            this.obstacleMeshAnimating = false;
            return;
        }

        let polygons;
        try {
            polygons = JSON.parse(msg.data);
        } catch (e) {
            console.error('[MapView] renderObstacles failed to parse msg.data JSON:', e);
            this.obstacleLines.forEach(l => l.clear());
            return;
        }
        if (!Array.isArray(polygons)) {
            this.obstacleLines.forEach(l => l.clear());
            return;
        }

        // Draw flat 2D boundary lines (existing logic)
        const MAX_COORD = 100.0;
        let lineIdx = 0;

        polygons.forEach((poly, polyIdx) => {
            if (!Array.isArray(poly) || poly.length < 3) {
                return;
            }

            const pts = poly
                .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
                .map((p) => new THREE.Vector3(p.x, p.y, 0.07));

            const hasGlitch = pts.some((p) => Math.abs(p.x) > MAX_COORD || Math.abs(p.y) > MAX_COORD);
            if (hasGlitch) {
                console.warn(`[MapView] renderObstacles: poly[${polyIdx}] rejected due to coordinates exceeding 100m (potential glitch).`);
                return;
            }

            if (pts.length < 3) {
                return;
            }
            if (Math.abs(pts[0].x - pts[pts.length - 1].x) > 1e-4 ||
                Math.abs(pts[0].y - pts[pts.length - 1].y) > 1e-4) {
                pts.push(pts[0].clone());
            }

            if (lineIdx >= this.obstacleLines.length) {
                const newLine = new DynamicLine(0xff6600, 1000, 1);
                this.obstacleLines.push(newLine);
                this.obstacleLayer.add(newLine.line);
            }
            this.obstacleLines[lineIdx].updatePoints(pts);
            lineIdx++;
        });

        for (let i = lineIdx; i < this.obstacleLines.length; i++) {
            this.obstacleLines[i].clear();
        }

        // Now process 3D obstacles
        const obstacleHeight = WALL_HEIGHT * 1.25; // slightly taller to stand out
        const thickness = 0.04; // 4cm thick laser fence / wall

        // Convert polygons to segments
        const newSegments = [];
        polygons.forEach((poly, polyIdx) => {
            if (!Array.isArray(poly) || poly.length < 3) return;
            const validPts = poly.filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y));
            if (validPts.length < 3) return;

            // Ensure closed loop
            const pts = [...validPts];
            if (Math.abs(pts[0].x - pts[pts.length - 1].x) > 1e-4 ||
                Math.abs(pts[0].y - pts[pts.length - 1].y) > 1e-4) {
                pts.push(pts[0]);
            }

            for (let i = 0; i < pts.length - 1; i++) {
                const A = pts[i];
                const B = pts[i + 1];
                const k1 = `${A.x.toFixed(2)},${A.y.toFixed(2)}`;
                const k2 = `${B.x.toFixed(2)},${B.y.toFixed(2)}`;
                const key = k1 < k2 ? `${k1}->${k2}` : `${k2}->${k1}`;

                newSegments.push({
                    key,
                    A,
                    B,
                    polyIdx,
                    segIdx: i
                });
            }
        });

        this.obstacleSegments = newSegments;

        if (newSegments.length === 0) {
            if (this.obstacleMesh) {
                this.obstacleLayer.remove(this.obstacleMesh);
                this.obstacleMesh = null;
            }
            this.obstacleSegmentsMap.clear();
            this.obstacleMeshReady = true;
            this.obstacleMeshAnimating = false;
            return;
        }

        // Geometry & Material
        if (!this.obstacleGeometry) {
            // Unit box centered at (0,0,0)
            this.obstacleGeometry = new THREE.BoxGeometry(1, 1, 1);
        }
        if (!this.obstacleMaterial) {
            // Glowing neon orange/red hologram barrier
            this.obstacleMaterial = new THREE.MeshStandardMaterial({
                color: 0xff4d00,
                emissive: 0x4a1400,
                roughness: 0.1,
                metalness: 0.9,
                transparent: true,
                opacity: 0.72,
                depthWrite: false
            });
        }

        // Recreate InstancedMesh ONLY when the instance count changes
        let needsRecreate = !this.obstacleMesh || this.obstacleMesh.count !== newSegments.length;
        if (needsRecreate) {
            if (this.obstacleMesh) {
                this.obstacleLayer.remove(this.obstacleMesh);
                this.obstacleMesh = null;
            }
             this.obstacleMesh = new THREE.InstancedMesh(this.obstacleGeometry, this.obstacleMaterial, newSegments.length);
             this.obstacleMesh.frustumCulled = false;
             this.obstacleMesh.renderOrder = 20;
             this.obstacleLayer.add(this.obstacleMesh);
        }

        // Update animation map
        const now = performance.now();
        const incomingKeys = new Set();

        newSegments.forEach((seg) => {
            incomingKeys.add(seg.key);

            if (!this.obstacleSegmentsMap.has(seg.key)) {
                // Winding domino delay effect
                const delay = seg.segIdx * 100; // 100ms per segment along the polygon path

                this.obstacleSegmentsMap.set(seg.key, {
                    key: seg.key,
                    startTime: now,
                    delay: delay,
                    duration: 500 // 500ms growth duration
                });
            }
        });

        // Remove stale segments
        for (const key of this.obstacleSegmentsMap.keys()) {
            if (!incomingKeys.has(key)) {
                this.obstacleSegmentsMap.delete(key);
            }
        }

        this.obstacleMeshReady = false; // Trigger matrix updates
        this.updateObstacleMeshInstances();
    }

    updateObstacleMeshInstances() {
        if (!this.obstacleMesh || !this.obstacleSegments) return;
        const now = performance.now();
        const dummy = new THREE.Object3D();
        let anyAnimating = false;
        const obstacleHeight = WALL_HEIGHT * 1.25;
        const thickness = 0.04;

        for (let i = 0; i < this.obstacleSegments.length; i++) {
            const seg = this.obstacleSegments[i];
            const anim = this.obstacleSegmentsMap.get(seg.key);

            let scaleZ = 1.0;
            if (anim) {
                const elapsed = now - anim.startTime;
                if (elapsed < anim.delay) {
                    scaleZ = 0.0001;
                    anyAnimating = true;
                } else {
                    const progress = elapsed - anim.delay;
                    if (progress < anim.duration) {
                        const t = progress / anim.duration;
                        scaleZ = Math.max(0.0001, this.easeOutBack(t));
                        anyAnimating = true;
                    } else {
                        scaleZ = 1.0;
                    }
                }
            }

            const dx = seg.B.x - seg.A.x;
            const dy = seg.B.y - seg.A.y;
            const len = Math.sqrt(dx * dx + dy * dy);
            const angle = Math.atan2(dy, dx);
            const cx = seg.A.x + dx / 2;
            const cy = seg.A.y + dy / 2;

            dummy.position.set(cx, cy, (scaleZ * obstacleHeight) / 2);
            dummy.scale.set(len, thickness, scaleZ * obstacleHeight);
            dummy.rotation.set(0, 0, angle);
            dummy.updateMatrix();
            this.obstacleMesh.setMatrixAt(i, dummy.matrix);
        }
        this.obstacleMesh.instanceMatrix.needsUpdate = true;
        this.obstacleMeshAnimating = anyAnimating;
        this.obstacleMeshReady = !anyAnimating;
    }

    renderZoneRect(corners) {
        if (!corners || corners.length < 4) {
            this.zoneLine.clear();
            return;
        }
        const pts = corners.map((c) => {
            return new THREE.Vector3(c.x, c.y, 0.08);
        });
        pts.push(pts[0].clone());
        this.zoneLine.updatePoints(pts);
    }

    resizeOverlay() {
        if (!this.overlayCanvas) return;
        const rect = this.mapContainer.getBoundingClientRect();
        this.overlayCanvas.width = rect.width;
        this.overlayCanvas.height = rect.height;
    }

    clearOverlay() {
        if (!this.overlayCtx) return;
        this.overlayCtx.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);
    }

    drawOverlayRect(x1, y1, x2, y2) {
        if (!this.overlayCtx) return;
        this.clearOverlay();
        this.overlayCtx.strokeStyle = '#00e676';
        this.overlayCtx.lineWidth = 2;
        this.overlayCtx.setLineDash([6, 3]);
        this.overlayCtx.strokeRect(
            Math.min(x1, x2), Math.min(y1, y2),
            Math.abs(x2 - x1), Math.abs(y2 - y1)
        );
    }

    screenToWorld(screenX, screenY) {
        const rect = this.mapContainer.getBoundingClientRect();
        const ndcX = ((screenX - rect.left) / rect.width) * 2 - 1;
        const ndcY = -((screenY - rect.top) / rect.height) * 2 + 1;
        const raycaster = new THREE.Raycaster();
        raycaster.setFromCamera({ x: ndcX, y: ndcY }, this.camera);
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const target = new THREE.Vector3();
        const hit = raycaster.ray.intersectPlane(plane, target);
        return hit ? { x: target.x, y: target.y } : null;
    }

    animateCamera(from, to, duration, onDone) {
        if (this.cameraAnimId !== null) {
            cancelAnimationFrame(this.cameraAnimId);
            this.cameraAnimId = null;
        }
        const cam = this.camera;
        let start = null;
        const ease = (t) => t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;

        const step = (ts) => {
            if (start === null) start = ts;
            const raw = Math.min((ts - start) / duration, 1.0);
            const t = ease(raw);

            cam.position.lerpVectors(from.position, to.position, t);
            cam.up.lerpVectors(from.up, to.up, t);
            cam.quaternion.copy(from.quaternion).slerp(to.quaternion, t);
            cam.updateMatrixWorld();

            if (raw < 1.0) {
                this.cameraAnimId = requestAnimationFrame(step);
            } else {
                this.cameraAnimId = null;
                if (typeof onDone === 'function') onDone();
            }
        };
        this.cameraAnimId = requestAnimationFrame(step);
    }

    snapCameraTopDown() {
        if (!this.camera || !this.cameraControls) return;
        const cam = this.camera;
        const controls = this.cameraControls;
        const tgt = controls.target;
        if (!tgt) return;

        this.savedCameraState = {
            position: cam.position.clone(),
            up: cam.up.clone(),
            quaternion: cam.quaternion.clone(),
            target: tgt.clone(),
        };

        controls.enabled = false;

        const dist = cam.position.distanceTo(tgt);
        const toPos = new THREE.Vector3(tgt.x, tgt.y, tgt.z + dist);
        const toUp = new THREE.Vector3(0, 1, 0);

        const tempCam = cam.clone();
        tempCam.position.copy(toPos);
        tempCam.up.copy(toUp);
        tempCam.lookAt(tgt.x, tgt.y, tgt.z);
        const toQuat = tempCam.quaternion.clone();

        this.animateCamera(
            { position: cam.position.clone(), up: cam.up.clone(), quaternion: cam.quaternion.clone() },
            { position: toPos, up: toUp, quaternion: toQuat },
            420,
            () => {
                cam.position.copy(toPos);
                cam.up.copy(toUp);
                cam.lookAt(tgt.x, tgt.y, tgt.z);
            }
        );
    }

    restoreCamera() {
        if (!this.savedCameraState || !this.camera || !this.cameraControls) return;
        const cam = this.camera;
        const controls = this.cameraControls;
        const saved = this.savedCameraState;
        this.savedCameraState = null;

        this.animateCamera(
            { position: cam.position.clone(), up: cam.up.clone(), quaternion: cam.quaternion.clone() },
            { position: saved.position, up: saved.up, quaternion: saved.quaternion },
            420,
            () => {
                cam.position.copy(saved.position);
                cam.up.copy(saved.up);
                cam.quaternion.copy(saved.quaternion);
                controls.target.copy(saved.target);
                controls.enabled = true;
                controls.update();
            }
        );
    }

    setZoneDrawMode(enabled) {
        if (enabled && this.navPointMode) this.setNavPointMode(false);
        this.zoneDrawMode = enabled;
        if (this.overlayCanvas) {
            this.overlayCanvas.style.pointerEvents = (enabled || this.navPointMode) ? 'auto' : 'none';
        }
        this.mapContainer.style.cursor = enabled ? 'crosshair' : '';
        if (enabled) {
            this.resizeOverlay();
            this.snapCameraTopDown();
        } else {
            if (!this.navPointMode) this.restoreCamera();
            this.clearOverlay();
            if (this.interactionHandler) this.interactionHandler.reset();
        }

        if (this.onZoneDrawModeChange) {
            this.onZoneDrawModeChange(enabled);
        }
    }

    renderNavTarget(worldX, worldY, yawRad) {
        this.clearGroup(this.navTargetLayer);

        const z = 0.02; // Close to ground to lie flat
        const color = 0x9c27ff; // Navigation purple

        // 1. Outer Ring
        const ring = new THREE.Mesh(
            new THREE.RingGeometry(0.3, 0.34, 48),
            new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.8, side: THREE.DoubleSide })
        );
        ring.position.set(worldX, worldY, z);
        this.navTargetLayer.add(ring);

        // 2. Center Dot
        const centerDot = new THREE.Mesh(
            new THREE.CircleGeometry(0.05, 16),
            new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.9 })
        );
        centerDot.position.set(worldX, worldY, z);
        this.navTargetLayer.add(centerDot);

        // 3. Direction Pointer (only if yawRad is specified)
        if (yawRad !== undefined) {
            const pointer = new THREE.Mesh(
                new THREE.ConeGeometry(0.08, 0.25, 4),
                new THREE.MeshBasicMaterial({ color: 0xf1c84b, transparent: true, opacity: 0.95 }) // gold pointer
            );
            // Since Z is UP in our map frame, the cone (initially along Y) is already flat on the X-Y plane.
            // We only need to rotate it around the vertical Z-axis to align with yawRad.
            pointer.rotation.z = yawRad - Math.PI / 2;

            // Offset along the yaw vector to place it on the ring edge
            const radius = 0.42;
            pointer.position.set(
                worldX + Math.cos(yawRad) * radius,
                worldY + Math.sin(yawRad) * radius,
                z
            );
            this.navTargetLayer.add(pointer);
        }
    }

    clearNavTarget() {
        this.clearGroup(this.navTargetLayer);
        this.activeNavTarget = null;
        if (this.arrivalTimer) {
            clearTimeout(this.arrivalTimer);
            this.arrivalTimer = null;
        }
        console.log('[MapView] Navigation target pin cleared.');
    }

    setNavPointMode(enabled) {
        if (enabled && this.zoneDrawMode) this.setZoneDrawMode(false);
        this.navPointMode = enabled;
        if (this.overlayCanvas) {
            this.overlayCanvas.style.pointerEvents = (enabled || this.zoneDrawMode) ? 'auto' : 'none';
        }
        this.mapContainer.style.cursor = enabled ? 'crosshair' : '';
        if (enabled) {
            this.resizeOverlay();
            this.clearNavTarget();
            if (this.interactionHandler) this.interactionHandler.reset();
            this.snapCameraTopDown();
        } else {
            if (this.interactionHandler) this.interactionHandler.reset();
            this.clearOverlay();
            if (!this.zoneDrawMode) this.restoreCamera();
        }

        if (this.onNavPointModeChange) {
            this.onNavPointModeChange(enabled);
        }
    }

    commitNavPoint(worldX, worldY, yawRad) {
        if (this.arrivalTimer) {
            clearTimeout(this.arrivalTimer);
            this.arrivalTimer = null;
        }
        this.activeNavTarget = { x: worldX, y: worldY };

        this.renderNavTarget(worldX, worldY, yawRad);

        if (this.onNavigateToPoint) {
            this.onNavigateToPoint(worldX, worldY, yawRad);
        }

        this.navPointMode = false;
        if (this.interactionHandler) this.interactionHandler.reset();
        this.clearOverlay();
        if (this.overlayCanvas) this.overlayCanvas.style.pointerEvents = 'none';
        this.mapContainer.style.cursor = '';
        if (!this.zoneDrawMode) this.restoreCamera();

        if (this.onNavPointModeChange) {
            this.onNavPointModeChange(false);
        }
        console.log('[MapView] committed world=(', worldX.toFixed(3), ',', worldY.toFixed(3), ') yaw=', (yawRad * 180 / Math.PI).toFixed(1), '°');
    }

    commitZone(x1Screen, y1Screen, x2Screen, y2Screen) {
        const tl = this.screenToWorld(Math.min(x1Screen, x2Screen), Math.min(y1Screen, y2Screen));
        const br = this.screenToWorld(Math.max(x1Screen, x2Screen), Math.max(y1Screen, y2Screen));
        if (!tl || !br) {
            console.warn('[MapView] screenToWorld returned null');
            return;
        }

        const corners = [
            { x: tl.x, y: tl.y },
            { x: br.x, y: tl.y },
            { x: br.x, y: br.y },
            { x: tl.x, y: br.y },
        ];
        this.activeZoneCorners = corners;
        this.clearOverlay();
        this.renderZoneRect(corners);

        if (this.onPublishCommand) {
            this.onPublishCommand('set_zone:' + JSON.stringify(corners));
        }
        console.log('[MapView] Zone committed:', corners);

        this.setZoneDrawMode(false);
    }

    setupEventListeners() {
        if (this.interactionHandler) {
            this.interactionHandler.init();
        }
    }

    setupSubscriptions() {
        const previewPathTopic = rosService.createTopicV2(TOPICS.COVERAGE_PREVIEW_PATH, MSG_TYPES.PATH);
        if (previewPathTopic) {
            previewPathTopic.subscribe((msg) => this.renderPreviewPath(msg));
            this.topics.push(previewPathTopic);
        }

        const polygonActiveTopic = rosService.createTopicV2(TOPICS.COVERAGE_POLYGON_ACTIVE, MSG_TYPES.POLYGON_STAMPED, {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (polygonActiveTopic) {
            polygonActiveTopic.subscribe((msg) => this.renderMapPolygon(msg));
            this.topics.push(polygonActiveTopic);
        }

        const obstaclesTopic = rosService.createTopicV2(TOPICS.COVERAGE_OBSTACLES_ACTIVE, MSG_TYPES.STRING, {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (obstaclesTopic) {
            obstaclesTopic.subscribe((msg) => this.renderObstacles(msg));
            this.topics.push(obstaclesTopic);
        }

        // The backend `robot_pose_publisher` node (nav2 package) already
        // resolves map -> base_link via tf2 and republishes it here, so the
        // UI no longer needs to combine /tf (map->odom) and
        // /odometry/filtered (odom->base_link) itself.
        const robotPoseSub = rosService.createTopicV2(TOPICS.ROBOT_POSE, MSG_TYPES.POSE_STAMPED, {
            throttle_rate: 33
        });
        if (robotPoseSub) {
            robotPoseSub.subscribe((message) => {
                const p = message.pose.position;
                const q = message.pose.orientation;

                this.poseBuffer.push({
                    position: new THREE.Vector3(p.x, p.y, 0.01),
                    quaternion: new THREE.Quaternion(q.x, q.y, q.z, q.w),
                    timestamp: performance.now()
                });
                const cutoff = performance.now() - CUTOFF_TIME_MS;
                while (this.poseBuffer.length > 1 && this.poseBuffer[0].timestamp < cutoff) {
                    this.poseBuffer.shift();
                }

                if (this.activeNavTarget) {
                    const dx = p.x - this.activeNavTarget.x;
                    const dy = p.y - this.activeNavTarget.y;
                    const dist = Math.sqrt(dx * dx + dy * dy);

                    if (dist < ARRIVAL_DISTANCE) {
                        if (!this.arrivalTimer) {
                            console.log('[MapView] Arrived at target point. Clearing pin in 5 seconds.');
                            this.arrivalTimer = setTimeout(() => {
                                this.clearNavTarget();
                            }, 5000);
                        }
                    }
                }
            });
            this.topics.push(robotPoseSub);
        }

        const mapTopic = rosService.createTopicV2('/map', 'nav_msgs/msg/OccupancyGrid', {
            throttle_rate: MAP_THROTTLE_RATE
        });
        if (mapTopic) {
            mapTopic.subscribe((message) => this.handleMapUpdate(message));
            this.topics.push(mapTopic);
        }

        this.resizeListener = () => {
            if (this.mapContainer && this.camera && this.renderer) {
                const newWidth = this.mapContainer.offsetWidth;
                const newHeight = this.mapContainer.offsetHeight;
                this.camera.aspect = newWidth / newHeight;
                this.camera.updateProjectionMatrix();
                this.renderer.setSize(newWidth, newHeight);
            }
            this.resizeOverlay();
        };
        window.addEventListener("resize", this.resizeListener);
    }

    startZoneDraw() {
        this.setZoneDrawMode(!this.zoneDrawMode);
        return this.zoneDrawMode;
    }

    clearZone() {
        this.zoneLine.clear();
        this.clearOverlay();
        this.activeZoneCorners = null;
        this.setZoneDrawMode(false);
        if (this.onPublishCommand) {
            this.onPublishCommand('clear_zone');
        }
    }

    clearMapPolygon() {
        this.mapPolygonLine.clear();
        this.obstacleLines.forEach(l => l.clear());
    }

    startNavPoint() {
        this.setNavPointMode(!this.navPointMode);
        return this.navPointMode;
    }

    cancelNavPoint() {
        this.setNavPointMode(false);
    }

    isZoneDrawMode() { return this.zoneDrawMode; }
    hasActiveZone() { return this.activeZoneCorners !== null; }
    isNavPointMode() { return this.navPointMode; }

    destroy() {
        console.log('[MapView] Destroying and cleaning up MapView...');

        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }

        if (this.arrivalTimer) {
            clearTimeout(this.arrivalTimer);
            this.arrivalTimer = null;
        }

        if (this.topics) {
            this.topics.forEach(t => {
                try {
                    t.unsubscribe();
                } catch (e) {
                    console.error('[MapView] Error unsubscribing from topic:', t.name, e);
                }
            });
            this.topics = [];
        }

        if (this.resizeListener) {
            window.removeEventListener("resize", this.resizeListener);
            this.resizeListener = null;
        }

        if (this.canvas) {
            if (this._onContextLost) this.canvas.removeEventListener('webglcontextlost', this._onContextLost);
            if (this._onContextRestored) this.canvas.removeEventListener('webglcontextrestored', this._onContextRestored);
        }

        // Dispose Three.js objects
        if (this.renderer) {
            this.renderer.dispose();
            if (this.renderer.domElement && this.renderer.domElement.parentNode) {
                this.renderer.domElement.parentNode.removeChild(this.renderer.domElement);
            }
            this.renderer = null;
        }

        if (this.cameraControls) {
            this.cameraControls.dispose();
            this.cameraControls = null;
        }

        // Dispose cached geometries and materials
        if (this.wallGeometry) {
            this.wallGeometry.dispose();
            this.wallGeometry = null;
        }
        if (this.wallMaterial) {
            this.wallMaterial.dispose();
            this.wallMaterial = null;
        }
        if (this.unexploredGeometry) {
            this.unexploredGeometry.dispose();
            this.unexploredGeometry = null;
        }
        if (this.unexploredMaterial) {
            this.unexploredMaterial.dispose();
            this.unexploredMaterial = null;
        }
        if (this.obstacleGeometry) {
            this.obstacleGeometry.dispose();
            this.obstacleGeometry = null;
        }
        if (this.obstacleMaterial) {
            this.obstacleMaterial.dispose();
            this.obstacleMaterial = null;
        }

        if (this.floorMesh) {
            if (this.floorMesh.geometry) this.floorMesh.geometry.dispose();
            if (this.floorMesh.material) {
                if (this.floorMesh.material.map) this.floorMesh.material.map.dispose();
                this.floorMesh.material.dispose();
            }
            this.floorMesh = null;
        }

        if (this.oldFloorMesh) {
            if (this.oldFloorMesh.geometry) this.oldFloorMesh.geometry.dispose();
            if (this.oldFloorMesh.material) {
                if (this.oldFloorMesh.material.map) this.oldFloorMesh.material.map.dispose();
                this.oldFloorMesh.material.dispose();
            }
            this.oldFloorMesh = null;
        }

        if (this.wallMesh) {
            this.wallMesh = null;
        }
        if (this.unexploredMesh) {
            this.unexploredMesh = null;
        }
        if (this.obstacleMesh) {
            this.obstacleMesh = null;
        }

        if (this.scene) {
            while (this.scene.children.length > 0) {
                const child = this.scene.children[0];
                this.scene.remove(child);
            }
            this.scene = null;
        }
    }
}
