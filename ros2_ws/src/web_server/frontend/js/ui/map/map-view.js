import { rosService } from '../../services/ros-service.js';
import { DynamicLine } from './dynamic-line.js';
import { installMapColorOverride } from './map-color-override.js';
import { MapInteractionHandler } from './map-interaction.js';
import { buildRobotModelFromUrdf } from './robot-model.js';
import { TOPICS, MSG_TYPES } from '../../shared/constants.js';

export class MapView {
    constructor(containerId) {
        this.containerId = containerId;
        this.mapContainer = document.getElementById(containerId);
        
        // Callbacks
        this.onPublishCommand = null;
        this.onNavigateToPoint = null;
        this.onZoneDrawModeChange = null;
        this.onNavPointModeChange = null;

        // ROS3D components
        this.viewer = null;
        this.gridClient = null;
        this.robotMarker = null;

        // Grass background
        this.grassPlane = null;
        this.grassTexture = null;

        // Layer groups
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

        // Initialize consolidated interaction handler
        this.interactionHandler = new MapInteractionHandler(this);

        this.init();
    }

    init() {
        if (!window.ROS3D) {
            console.error("ROS3D is not available!");
            return;
        }

        const containerWidth = this.mapContainer.offsetWidth;
        const containerHeight = this.mapContainer.offsetHeight;

        // Create the main 3D viewer (suppressing THREE.WebGLRenderer library log)
        const originalLog = console.log;
        console.log = function(...args) {
            if (args[0] === 'THREE.WebGLRenderer') return;
            originalLog.apply(console, args);
        };

        this.viewer = new window.ROS3D.Viewer({
            divID: this.containerId,
            width: containerWidth,
            height: containerHeight,
            antialias: true,
            background: '#0c0f14',
        });
        if (this.viewer.renderer) {
            this.viewer.renderer.setClearColor(0x0c0f14, 1.0);
        }

        console.log = originalLog;

        // ROS3D.Viewer automatically adds a ROS3D.Axes helper (6 cylinders/cones at
        // the world origin) to the scene. When the camera pans to show the origin,
        // these appear as a blue bar + orange cone that look like a broken robot model.
        // Remove everything the viewer added — we manage the scene ourselves.
        while (this.viewer.scene.children.length > 0) {
            this.viewer.scene.remove(this.viewer.scene.children[0]);
        }

        // Lock the camera to pan/zoom only — prevent accidental rotation that
        // would show the robot model from a confusing side angle.
        if (this.viewer.cameraControls) {
            this.viewer.cameraControls.enableRotate = false;
            this.viewer.cameraControls.enablePan   = true;
            this.viewer.cameraControls.enableZoom  = true;
        }

        // Add Ambient and Directional Lights for realistic 3D shading of the robot model
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.75);
        this.viewer.scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xffffff, 0.85);
        dirLight.position.set(5, -5, 10);
        this.viewer.scene.add(dirLight);

        this.viewer.scene.add(this.previewLayer);
        this.viewer.scene.add(this.mapPolygonLayer);
        this.viewer.scene.add(this.obstacleLayer);
        this.viewer.scene.add(this.zoneLayer);
        this.viewer.scene.add(this.navTargetLayer);

        // Define our custom ShaderMaterial for seamless grass rendering
        this.customGrassMaterial = new THREE.ShaderMaterial({
            uniforms: {
                uMapTex: { value: null },
                uGrassTex: { value: null },
            },
            vertexShader: `
                varying vec2 vUv;
                varying vec3 vWorldPosition;
                void main() {
                    vUv = uv;
                    vec4 worldPosition = modelMatrix * vec4(position, 1.0);
                    vWorldPosition = worldPosition.xyz;
                    gl_Position = projectionMatrix * viewMatrix * worldPosition;
                }
            `,
            fragmentShader: `
                uniform sampler2D uMapTex;
                uniform sampler2D uGrassTex;
                varying vec2 vUv;
                varying vec3 vWorldPosition;
                void main() {
                    vec4 mapColor = texture2D(uMapTex, vUv);
                    
                    // Tiling in world space (x, y). Scale factor 0.5 tiles every 2 meters.
                    vec2 grassUv = vWorldPosition.xy * 0.5;
                    vec4 grassColor = texture2D(uGrassTex, grassUv);
                    
                    if (mapColor.a == 0.0) {
                        // Free space (value = 0) -> render grass texture
                        gl_FragColor = grassColor;
                    } else if (mapColor.a < 0.9) {
                        // Unknown space (value < 0) -> blend into background color
                        gl_FragColor = vec4(0.0470588, 0.0588235, 0.0784314, 1.0);
                    } else {
                        // Occupied space (value > 0) -> render occupied wall color (fully opaque)
                        gl_FragColor = vec4(mapColor.rgb, 1.0);
                    }
                }
            `,
            transparent: true,
            depthWrite: false,
            side: THREE.DoubleSide
        });

        // Install custom colour override before subscribing to map topic
        installMapColorOverride();

        // Build the grass background plane (shows through transparent map cells)
        this._buildGrassPlane();

        // Setup the map client using ROS3D (using legacy rosV1 connection)
        this.gridClient = new window.ROS3D.OccupancyGridClient({
            ros: rosService.rosV1,
            rootObject: this.viewer.scene,
            continuous: true,
            opacity: 1.0,
        });

        // After every map update, patch the material for transparency + smooth filtering
        this.gridClient.on('change', () => this._patchMapMaterial());

        // Setup robot marker as an empty group initially (swapped with URDF once loaded)
        this.robotMarker = new THREE.Group();
        this.viewer.scene.add(this.robotMarker);


        this.resizeOverlay();
        this.setupEventListeners();
        this.setupSubscriptions();
    }

    /**
     * Load the grass texture and assign it to our custom ShaderMaterial uniform.
     */
    _buildGrassPlane() {
        const loader = new THREE.TextureLoader();
        loader.load('/grass_texture.png', (texture) => {
            this.grassTexture = texture;
            texture.wrapS = THREE.RepeatWrapping;
            texture.wrapT = THREE.RepeatWrapping;
            texture.minFilter = THREE.LinearMipMapLinearFilter;
            texture.magFilter = THREE.LinearFilter;
            texture.anisotropy = 4;

            if (this.customGrassMaterial) {
                this.customGrassMaterial.uniforms.uGrassTex.value = texture;
            }
        });
    }

    /**
     * After the OccupancyGridClient rebuilds its mesh, apply our custom ShaderMaterial
     * and smooth (linear) texture filtering.
     */
    _patchMapMaterial() {
        const grid = this.gridClient && this.gridClient.currentGrid;
        if (!grid) return;

        const tex = grid.texture;

        if (tex) {
            // Switch from NearestFilter to LinearFilter for smooth map edges
            tex.minFilter = THREE.LinearFilter;
            tex.magFilter = THREE.LinearFilter;
            tex.needsUpdate = true;
        }

        // Apply our custom shader material
        if (grid.material !== this.customGrassMaterial) {
            this.customGrassMaterial.uniforms.uMapTex.value = tex;
            grid.material = this.customGrassMaterial;
            grid.material.needsUpdate = true;
        }
    }

    clearGroup(group) {
        while (group.children.length > 0) {
            const child = group.children[0];
            group.remove(child);
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
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
                    0.1
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
            .map((p) => new THREE.Vector3(p.x, p.y, 0.05));

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

        const MAX_COORD = 100.0;
        let lineIdx = 0;

        polygons.forEach((poly, polyIdx) => {
            if (!Array.isArray(poly) || poly.length < 3) {
                return;
            }

            const pts = poly
                .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
                .map((p) => new THREE.Vector3(p.x, p.y, 0.06));
            
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

            // Get or create dynamic line from pool
            if (lineIdx >= this.obstacleLines.length) {
                const newLine = new DynamicLine(0xff6600, 1000, 1);
                this.obstacleLines.push(newLine);
                this.obstacleLayer.add(newLine.line);
            }
            this.obstacleLines[lineIdx].updatePoints(pts);
            lineIdx++;
        });

        // Clear remaining pooled lines that were not used in this update
        for (let i = lineIdx; i < this.obstacleLines.length; i++) {
            this.obstacleLines[i].clear();
        }
    }

    renderZoneRect(corners) {
        if (!corners || corners.length < 4) {
            this.zoneLine.clear();
            return;
        }
        const pts = corners.map((c) => {
            return new THREE.Vector3(c.x, c.y, 0.12);
        });
        pts.push(pts[0].clone()); // close loop
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
        raycaster.setFromCamera({ x: ndcX, y: ndcY }, this.viewer.camera);
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
        const cam = this.viewer.camera;
        let start = null;
        const ease = (t) => t < 0.5 ? 4 * t * t * t : 1 - Math.pow(-2 * t + 2, 3) / 2;

        const step = (ts) => {
            if (start === null) start = ts;
            const raw = Math.min((ts - start) / duration, 1.0);
            const t = ease(raw);

            cam.position.lerpVectors(from.position, to.position, t);
            cam.up.lerpVectors(from.up, to.up, t);
            THREE.Quaternion.slerp(from.quaternion, to.quaternion, cam.quaternion, t);
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
        if (!this.viewer || !this.viewer.camera || !this.viewer.cameraControls) return;
        const cam = this.viewer.camera;
        const controls = this.viewer.cameraControls;
        const tgt = controls.target || controls.center;
        if (!tgt) {
            console.warn('[MapView] controls target/center is undefined');
            return;
        }

        // Save camera state
        this.savedCameraState = {
            position: cam.position.clone(),
            up: cam.up.clone(),
            quaternion: cam.quaternion.clone(),
            target: tgt.clone(),
        };

        controls.enabled = false;

        // Compute top-down target pose
        const dist = cam.position.distanceTo(tgt);
        const toPos = new THREE.Vector3(tgt.x, tgt.y, tgt.z + dist);
        const toUp = new THREE.Vector3(0, -1, 0);

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
        if (!this.savedCameraState || !this.viewer || !this.viewer.camera || !this.viewer.cameraControls) return;
        const cam = this.viewer.camera;
        const controls = this.viewer.cameraControls;
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
                if (controls.target) {
                    controls.target.copy(saved.target);
                } else if (controls.center) {
                    controls.center.copy(saved.target);
                }
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
        const r = 0.45; // radius in meters
        const z = 0.2; // height above occupancy grid
        const col = 0x9c27ff;
        const mat = new THREE.LineBasicMaterial({ color: col, linewidth: 2 });

        const theta = yawRad !== undefined ? yawRad : 0.0;
        const cosT = Math.cos(theta);
        const sinT = Math.sin(theta);

        // Crosshair rotated with arrow
        this.navTargetLayer.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(worldX - r * cosT, worldY - r * sinT, z),
                new THREE.Vector3(worldX + r * cosT, worldY + r * sinT, z),
            ]), mat));
        this.navTargetLayer.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(worldX + r * sinT, worldY - r * cosT, z),
                new THREE.Vector3(worldX - r * sinT, worldY + r * cosT, z),
            ]), mat.clone()));

        // Circle
        const segs = 40;
        const circPts = [];
        for (let i = 0; i <= segs; i++) {
            const a = (i / segs) * Math.PI * 2;
            circPts.push(new THREE.Vector3(worldX + Math.cos(a) * r, worldY + Math.sin(a) * r, z));
        }
        this.navTargetLayer.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints(circPts), mat.clone()));

        // Direction arrow
        if (yawRad !== undefined) {
            const arrowLen = r * 1.6;
            const edgeX = worldX + Math.cos(yawRad) * r;
            const edgeY = worldY + Math.sin(yawRad) * r;
            const tipX = worldX + Math.cos(yawRad) * (r + arrowLen);
            const tipY = worldY + Math.sin(yawRad) * (r + arrowLen);
            const headLen = 0.2;
            const headAngle = 0.45;
            this.navTargetLayer.add(new THREE.Line(
                new THREE.BufferGeometry().setFromPoints([
                    new THREE.Vector3(edgeX, edgeY, z),
                    new THREE.Vector3(tipX, tipY, z),
                ]), mat.clone()));
            // Arrowhead
            this.navTargetLayer.add(new THREE.Line(
                new THREE.BufferGeometry().setFromPoints([
                    new THREE.Vector3(tipX + Math.cos(yawRad + Math.PI - headAngle) * headLen,
                                      tipY + Math.sin(yawRad + Math.PI - headAngle) * headLen, z),
                    new THREE.Vector3(tipX, tipY, z),
                    new THREE.Vector3(tipX + Math.cos(yawRad + Math.PI + headAngle) * headLen,
                                      tipY + Math.sin(yawRad + Math.PI + headAngle) * headLen, z),
                ]), mat.clone()));
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
        const previewPathTopic = rosService.createTopicV1(TOPICS.COVERAGE_PREVIEW_PATH, MSG_TYPES.PATH);
        if (previewPathTopic) {
            previewPathTopic.subscribe((msg) => this.renderPreviewPath(msg));
        }

        const polygonActiveTopic = rosService.createTopicV1(TOPICS.COVERAGE_POLYGON_ACTIVE, MSG_TYPES.POLYGON_STAMPED, {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (polygonActiveTopic) {
            polygonActiveTopic.subscribe((msg) => this.renderMapPolygon(msg));
        }

        const obstaclesTopic = rosService.createTopicV1(TOPICS.COVERAGE_OBSTACLES_ACTIVE, MSG_TYPES.STRING, {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (obstaclesTopic) {
            obstaclesTopic.subscribe((msg) => this.renderObstacles(msg));
        }

        // Subscribe to robot description to load the 3D model dynamically
        const robotDescTopic = rosService.createTopicV1(TOPICS.ROBOT_DESCRIPTION, MSG_TYPES.STRING, {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (robotDescTopic) {
            robotDescTopic.subscribe((msg) => {
                if (msg && msg.data) {
                    const newModel = buildRobotModelFromUrdf(msg.data);
                    if (newModel) {
                        this.viewer.scene.remove(this.robotMarker);
                        this.robotMarker = newModel;
                        this.viewer.scene.add(this.robotMarker);
                        console.log('[MapView] Robot model loaded from URDF.');
                    } else {
                        console.error('[MapView] buildRobotModelFromUrdf returned null.');
                    }
                } else {
                    console.error('[MapView] Empty /robot_description message.');
                }
                robotDescTopic.unsubscribe();
            });
        }

        const odomSub = rosService.createTopicV1(TOPICS.ODOMETRY, MSG_TYPES.ODOMETRY, {
            throttle_rate: 100
        });
        if (odomSub) {
            odomSub.subscribe((message) => {
                const robotX = message.pose.pose.position.x;
                const robotY = message.pose.pose.position.y;

                this.robotMarker.position.x = robotX;
                this.robotMarker.position.y = robotY;

                this.robotPosition = { x: robotX, y: robotY };

                // Handle arrival timer for navigation pin
                if (this.activeNavTarget) {
                    const dx = robotX - this.activeNavTarget.x;
                    const dy = robotY - this.activeNavTarget.y;
                    const dist = Math.sqrt(dx * dx + dy * dy);

                    if (dist < 0.25) { // 25 cm threshold for arrival
                        if (!this.arrivalTimer) {
                            console.log('[MapView] Arrived at target point. Clearing pin in 5 seconds.');
                            this.arrivalTimer = setTimeout(() => {
                                this.clearNavTarget();
                            }, 5000);
                        }
                    }
                }

                const q = message.pose.pose.orientation;
                const quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);

                // Apply odometry pose to the 3D robot model
                this.robotMarker.position.z = 0.01;
                this.robotMarker.quaternion.copy(quaternion);
            });
        }

        // Handle window resize to make the viewer responsive
        window.addEventListener("resize", () => {
            const newWidth = this.mapContainer.offsetWidth;
            const newHeight = this.mapContainer.offsetHeight;

            if (this.viewer && this.viewer.camera && this.viewer.renderer) {
                this.viewer.camera.aspect = newWidth / newHeight;
                this.viewer.camera.updateProjectionMatrix();
                this.viewer.renderer.setSize(newWidth, newHeight);
            }
            this.resizeOverlay();
        });
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
}
