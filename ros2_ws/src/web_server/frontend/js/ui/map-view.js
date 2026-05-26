import { rosService } from '../services/ros-service.js';

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

        // Layer groups
        this.previewLayer = new THREE.Group();
        this.mapPolygonLayer = new THREE.Group();
        this.obstacleLayer = new THREE.Group();
        this.zoneLayer = new THREE.Group();
        this.navTargetLayer = new THREE.Group();

        // Overlay Canvas
        this.overlayCanvas = document.getElementById('zone-draw-canvas');
        this.overlayCtx = this.overlayCanvas ? this.overlayCanvas.getContext('2d') : null;

        // Animation and state
        this.savedCameraState = null;
        this.cameraAnimId = null;
        this.zoneDrawMode = false;
        this.zoneDrawing = false;
        this.zoneDragStart = null;
        this.activeZoneCorners = null;

        this.navPointMode = false;
        this.navAnchor = null;

        this.init();
    }

    init() {
        if (!window.ROS3D) {
            console.error("ROS3D is not available!");
            return;
        }

        const containerWidth = this.mapContainer.offsetWidth;
        const containerHeight = this.mapContainer.offsetHeight;

        // Create the main 3D viewer
        this.viewer = new window.ROS3D.Viewer({
            divID: this.containerId,
            width: containerWidth,
            height: containerHeight,
            antialias: true,
        });

        this.viewer.scene.add(this.previewLayer);
        this.viewer.scene.add(this.mapPolygonLayer);
        this.viewer.scene.add(this.obstacleLayer);
        this.viewer.scene.add(this.zoneLayer);
        this.viewer.scene.add(this.navTargetLayer);

        // Setup the map client using ROS3D (using legacy rosV1 connection)
        this.gridClient = new window.ROS3D.OccupancyGridClient({
            ros: rosService.rosV1,
            rootObject: this.viewer.scene,
            continuous: true,
        });

        // Setup robot marker as an arrow for odometry
        this.robotMarker = new window.ROS3D.Arrow({
            length: 0.5,
            headLength: 0.25,
            shaftDiameter: 0.4,
            headDiameter: 0.4,
            material: new THREE.MeshBasicMaterial({ color: 0xff0000 }),
        });
        this.viewer.scene.add(this.robotMarker);

        this.resizeOverlay();
        this.setupEventListeners();
        this.setupSubscriptions();
    }

    clearGroup(group) {
        while (group.children.length > 0) {
            const child = group.children.pop();
            group.remove(child);
            if (child.geometry) child.geometry.dispose();
            if (child.material) child.material.dispose();
        }
    }

    renderPreviewPath(pathMsg) {
        this.clearGroup(this.previewLayer);
        if (!pathMsg || !pathMsg.poses || pathMsg.poses.length === 0) {
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
        if (pathPoints.length < 2) {
            return;
        }
        const pathGeometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
        const pathMaterial = new THREE.LineBasicMaterial({ color: 0xd17a00 });
        this.previewLayer.add(new THREE.Line(pathGeometry, pathMaterial));
    }

    renderMapPolygon(msg) {
        this.clearGroup(this.mapPolygonLayer);
        if (!msg || !msg.polygon || !msg.polygon.points || msg.polygon.points.length < 3) {
            return;
        }
        const pts = msg.polygon.points
            .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
            .map((p) => new THREE.Vector3(p.x, p.y, 0.05));

        if (pts.length < 3) {
            console.warn('renderMapPolygon: no finite points, skipping draw.');
            return;
        }

        const first = pts[0], last = pts[pts.length - 1];
        if (Math.abs(first.x - last.x) > 1e-4 || Math.abs(first.y - last.y) > 1e-4) {
            pts.push(first.clone());
        }

        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        const mat = new THREE.LineBasicMaterial({ color: 0x1a6fcc });
        this.mapPolygonLayer.add(new THREE.Line(geo, mat));
    }

    renderObstacles(msg) {
        this.clearGroup(this.obstacleLayer);
        if (!msg || !msg.data) return;
        let polygons;
        try { polygons = JSON.parse(msg.data); } catch (e) { return; }
        if (!Array.isArray(polygons)) return;
        polygons.forEach((poly) => {
            if (!Array.isArray(poly) || poly.length < 3) return;
            const pts = poly
                .filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y))
                .map((p) => new THREE.Vector3(p.x, p.y, 0.06));
            if (pts.length < 3) return;
            if (Math.abs(pts[0].x - pts[pts.length - 1].x) > 1e-4 ||
                Math.abs(pts[0].y - pts[pts.length - 1].y) > 1e-4) {
                pts.push(pts[0].clone());
            }
            const geo = new THREE.BufferGeometry().setFromPoints(pts);
            const mat = new THREE.LineBasicMaterial({ color: 0xff6600 });
            this.obstacleLayer.add(new THREE.Line(geo, mat));
        });
    }

    renderZoneRect(corners) {
        this.clearGroup(this.zoneLayer);
        if (!corners || corners.length < 4) return;
        const pts = corners.map((c) => {
            return new THREE.Vector3(c.x, c.y, 0.12);
        });
        pts.push(pts[0].clone()); // close loop
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        const mat = new THREE.LineBasicMaterial({ color: 0x00e676 });
        this.zoneLayer.add(new THREE.Line(geo, mat));
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
            this.zoneDrawing = false;
            this.zoneDragStart = null;
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

        // Crosshair
        this.navTargetLayer.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(worldX - r, worldY, z),
                new THREE.Vector3(worldX + r, worldY, z),
            ]), mat));
        this.navTargetLayer.add(new THREE.Line(
            new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(worldX, worldY - r, z),
                new THREE.Vector3(worldX, worldY + r, z),
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

    setNavPointMode(enabled) {
        if (enabled && this.zoneDrawMode) this.setZoneDrawMode(false);
        this.navPointMode = enabled;
        if (this.overlayCanvas) {
            this.overlayCanvas.style.pointerEvents = (enabled || this.zoneDrawMode) ? 'auto' : 'none';
        }
        this.mapContainer.style.cursor = enabled ? 'crosshair' : '';
        if (enabled) {
            this.resizeOverlay();
            this.clearGroup(this.navTargetLayer);
            this.navAnchor = null;
            this.snapCameraTopDown();
        } else {
            this.navAnchor = null;
            this.clearOverlay();
            if (!this.zoneDrawMode) this.restoreCamera();
        }

        if (this.onNavPointModeChange) {
            this.onNavPointModeChange(enabled);
        }
    }

    commitNavPoint(worldX, worldY, yawRad) {
        this.renderNavTarget(worldX, worldY, yawRad);

        if (this.onNavigateToPoint) {
            this.onNavigateToPoint(worldX, worldY, yawRad);
        }

        this.navPointMode = false;
        this.navAnchor = null;
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
        if (!this.overlayCanvas) return;

        // ---- mouse ----
        this.overlayCanvas.addEventListener('mousedown', (e) => {
            if (this.navPointMode) {
                const world = this.screenToWorld(e.clientX, e.clientY);
                if (!world) return;
                this.navAnchor = { screenX: e.clientX, screenY: e.clientY, worldX: world.x, worldY: world.y };
                this.renderNavTarget(world.x, world.y);
                e.preventDefault();
                return;
            }
            if (!this.zoneDrawMode) return;
            this.zoneDrawing = true;
            this.zoneDragStart = { x: e.clientX, y: e.clientY };
            e.preventDefault();
        });

        this.overlayCanvas.addEventListener('mousemove', (e) => {
            if (this.navPointMode && this.navAnchor) {
                const endWorld = this.screenToWorld(e.clientX, e.clientY);
                if (endWorld) {
                    const yaw = Math.atan2(endWorld.y - this.navAnchor.worldY, endWorld.x - this.navAnchor.worldX);
                    this.renderNavTarget(this.navAnchor.worldX, this.navAnchor.worldY, yaw);
                }
                e.preventDefault();
                return;
            }
            if (!this.zoneDrawMode || !this.zoneDrawing || !this.zoneDragStart) return;
            const r = this.mapContainer.getBoundingClientRect();
            this.drawOverlayRect(
                this.zoneDragStart.x - r.left, this.zoneDragStart.y - r.top,
                e.clientX - r.left, e.clientY - r.top
            );
            e.preventDefault();
        });

        this.overlayCanvas.addEventListener('mouseup', (e) => {
            if (this.navPointMode && this.navAnchor) {
                const endWorld = this.screenToWorld(e.clientX, e.clientY);
                let yaw = 0;
                if (endWorld) {
                    const ddx = endWorld.x - this.navAnchor.worldX;
                    const ddy = endWorld.y - this.navAnchor.worldY;
                    const sdx = e.clientX - this.navAnchor.screenX;
                    const sdy = e.clientY - this.navAnchor.screenY;
                    if (sdx * sdx + sdy * sdy > 100) yaw = Math.atan2(ddy, ddx);
                }
                this.commitNavPoint(this.navAnchor.worldX, this.navAnchor.worldY, yaw);
                e.preventDefault();
                return;
            }
            if (!this.zoneDrawMode || !this.zoneDrawing || !this.zoneDragStart) return;
            this.zoneDrawing = false;
            const dx = Math.abs(e.clientX - this.zoneDragStart.x);
            const dy = Math.abs(e.clientY - this.zoneDragStart.y);
            if (dx > 10 && dy > 10) {
                this.commitZone(this.zoneDragStart.x, this.zoneDragStart.y, e.clientX, e.clientY);
            } else {
                this.clearOverlay();
            }
            this.zoneDragStart = null;
            e.preventDefault();
        });

        // ---- touch ----
        this.overlayCanvas.addEventListener('touchstart', (e) => {
            if (this.navPointMode) {
                const t = e.touches[0];
                const world = this.screenToWorld(t.clientX, t.clientY);
                if (!world) return;
                this.navAnchor = { screenX: t.clientX, screenY: t.clientY, worldX: world.x, worldY: world.y };
                this.renderNavTarget(world.x, world.y);
                e.preventDefault();
                return;
            }
            if (!this.zoneDrawMode) return;
            const t = e.touches[0];
            this.zoneDrawing = true;
            this.zoneDragStart = { x: t.clientX, y: t.clientY };
            e.preventDefault();
        }, { passive: false });

        this.overlayCanvas.addEventListener('touchmove', (e) => {
            if (this.navPointMode && this.navAnchor) {
                const t = e.touches[0];
                const endWorld = this.screenToWorld(t.clientX, t.clientY);
                if (endWorld) {
                    const yaw = Math.atan2(endWorld.y - this.navAnchor.worldY, endWorld.x - this.navAnchor.worldX);
                    this.renderNavTarget(this.navAnchor.worldX, this.navAnchor.worldY, yaw);
                }
                e.preventDefault();
                return;
            }
            if (!this.zoneDrawMode || !this.zoneDrawing || !this.zoneDragStart) return;
            const t = e.touches[0];
            const r = this.mapContainer.getBoundingClientRect();
            this.drawOverlayRect(
                this.zoneDragStart.x - r.left, this.zoneDragStart.y - r.top,
                t.clientX - r.left, t.clientY - r.top
            );
            e.preventDefault();
        }, { passive: false });

        this.overlayCanvas.addEventListener('touchend', (e) => {
            if (this.navPointMode && this.navAnchor) {
                const t = e.changedTouches[0];
                const endWorld = this.screenToWorld(t.clientX, t.clientY);
                let yaw = 0;
                if (endWorld) {
                    const sdx = t.clientX - this.navAnchor.screenX;
                    const sdy = t.clientY - this.navAnchor.screenY;
                    if (sdx * sdx + sdy * sdy > 100) yaw = Math.atan2(endWorld.y - this.navAnchor.worldY, endWorld.x - this.navAnchor.worldX);
                }
                this.commitNavPoint(this.navAnchor.worldX, this.navAnchor.worldY, yaw);
                e.preventDefault();
                return;
            }
            if (!this.zoneDrawMode || !this.zoneDrawing || !this.zoneDragStart) return;
            this.zoneDrawing = false;
            const t = e.changedTouches[0];
            const dx = Math.abs(t.clientX - this.zoneDragStart.x);
            const dy = Math.abs(t.clientY - this.zoneDragStart.y);
            if (dx > 10 && dy > 10) {
                this.commitZone(this.zoneDragStart.x, this.zoneDragStart.y, t.clientX, t.clientY);
            } else {
                this.clearOverlay();
            }
            this.zoneDragStart = null;
            e.preventDefault();
        }, { passive: false });
    }

    setupSubscriptions() {
        const previewPathTopic = rosService.createTopicV1('/coverage/preview_path', 'nav_msgs/Path');
        if (previewPathTopic) {
            previewPathTopic.subscribe((msg) => this.renderPreviewPath(msg));
        }

        const polygonActiveTopic = rosService.createTopicV1('/coverage/polygon_active', 'geometry_msgs/PolygonStamped', {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (polygonActiveTopic) {
            polygonActiveTopic.subscribe((msg) => this.renderMapPolygon(msg));
        }

        const obstaclesTopic = rosService.createTopicV1('/coverage/obstacles_active', 'std_msgs/String', {
            durability: 'transient_local',
            reliability: 'reliable'
        });
        if (obstaclesTopic) {
            obstaclesTopic.subscribe((msg) => this.renderObstacles(msg));
        }

        const odomSub = rosService.createTopicV1("/odometry/filtered", "nav_msgs/Odometry", {
            throttle_rate: 100
        });
        if (odomSub) {
            odomSub.subscribe((message) => {
                this.robotMarker.position.x = message.pose.pose.position.x;
                this.robotMarker.position.y = message.pose.pose.position.y;
                this.robotMarker.position.z = 0.1;

                const q = message.pose.pose.orientation;
                const quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);

                const direction = new THREE.Vector3(1, 0, 0);
                direction.applyQuaternion(quaternion);

                this.robotMarker.setDirection(direction);
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
        this.clearGroup(this.zoneLayer);
        this.clearOverlay();
        this.activeZoneCorners = null;
        this.setZoneDrawMode(false);
        if (this.onPublishCommand) {
            this.onPublishCommand('clear_zone');
        }
    }

    clearMapPolygon() {
        this.clearGroup(this.mapPolygonLayer);
        this.clearGroup(this.obstacleLayer);
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
