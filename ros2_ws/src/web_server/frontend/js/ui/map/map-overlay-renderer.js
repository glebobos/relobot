import * as THREE from 'three';
import { DynamicLine } from './dynamic-line.js';
import { disposeObjectTree } from './three-disposal.js';

const MAX_COORDINATE = 100;
const OBSTACLE_HEIGHT = 0.35 * 1.25;
const OBSTACLE_THICKNESS = 0.04;

function finitePoint(point) {
    return Number.isFinite(point?.x) && Number.isFinite(point?.y);
}

function validCoordinates(points) {
    return points.every(point => (
        Math.abs(point.x) <= MAX_COORDINATE && Math.abs(point.y) <= MAX_COORDINATE
    ));
}

function closePoints(points) {
    if (points.length === 0) return points;
    const first = points[0];
    const last = points[points.length - 1];
    if (Math.abs(first.x - last.x) > 1e-4 || Math.abs(first.y - last.y) > 1e-4) {
        points.push(first.clone ? first.clone() : { ...first });
    }
    return points;
}

export class MapOverlayRenderer {
    constructor(scene) {
        this.scene = scene;
        this.previewLayer = new THREE.Group();
        this.polygonLayer = new THREE.Group();
        this.obstacleLayer = new THREE.Group();
        this.zoneLayer = new THREE.Group();
        this.targetLayer = new THREE.Group();
        [this.previewLayer, this.polygonLayer, this.obstacleLayer, this.zoneLayer, this.targetLayer]
            .forEach(layer => this.scene.add(layer));

        this.previewLine = new DynamicLine(0xd17a00, 5000, 1);
        this.polygonLine = new DynamicLine(0x1a6fcc, 5000, 1);
        this.zoneLine = new DynamicLine(0x00e676, 100, 1);
        this.previewLayer.add(this.previewLine.line);
        this.polygonLayer.add(this.polygonLine.line);
        this.zoneLayer.add(this.zoneLine.line);
        this.obstacleLines = [];
        this.obstacleSegments = [];
        this.obstacleAnimations = new Map();
        this.obstacleReady = true;
    }

    renderPreviewPath(message) {
        const points = message?.poses
            ?.map(pose => pose?.pose?.position)
            .filter(finitePoint)
            .map(point => new THREE.Vector3(point.x, point.y, 0.05)) || [];
        if (points.length < 2 || !validCoordinates(points)) {
            if (points.length >= 2) console.warn('[MapOverlayRenderer] Rejected invalid preview path.');
            this.previewLine.clear();
            return;
        }
        this.previewLine.updatePoints(points);
    }

    renderMapPolygon(message) {
        const points = message?.polygon?.points
            ?.filter(finitePoint)
            .map(point => new THREE.Vector3(point.x, point.y, 0.06)) || [];
        if (points.length < 3 || !validCoordinates(points)) {
            if (points.length >= 3) console.warn('[MapOverlayRenderer] Rejected invalid map polygon.');
            this.polygonLine.clear();
            return;
        }
        this.polygonLine.updatePoints(closePoints(points));
    }

    renderObstacles(message) {
        if (!message?.data) {
            this.clearObstacles();
            return;
        }
        let polygons;
        try {
            polygons = JSON.parse(message.data);
        } catch (error) {
            console.error('[MapOverlayRenderer] Invalid obstacle JSON:', error);
            this.clearObstacles();
            return;
        }
        if (!Array.isArray(polygons)) {
            this.clearObstacles();
            return;
        }

        const validPolygons = polygons
            .map(polygon => Array.isArray(polygon) ? polygon.filter(finitePoint) : [])
            .filter(points => points.length >= 3 && validCoordinates(points));
        this.updateObstacleLines(validPolygons);
        this.updateObstacleMesh(validPolygons);
    }

    updateObstacleLines(polygons) {
        polygons.forEach((polygon, index) => {
            const points = closePoints(
                polygon.map(point => new THREE.Vector3(point.x, point.y, 0.07)),
            );
            if (!this.obstacleLines[index]) {
                this.obstacleLines[index] = new DynamicLine(0xff6600, 1000, 1);
                this.obstacleLayer.add(this.obstacleLines[index].line);
            }
            this.obstacleLines[index].updatePoints(points);
        });
        for (let index = polygons.length; index < this.obstacleLines.length; index++) {
            this.obstacleLines[index].clear();
        }
    }

    updateObstacleMesh(polygons) {
        const segments = [];
        polygons.forEach((polygon, polygonIndex) => {
            const points = closePoints(polygon.map(point => ({ ...point })));
            for (let index = 0; index < points.length - 1; index++) {
                const start = points[index];
                const end = points[index + 1];
                const firstKey = `${start.x.toFixed(2)},${start.y.toFixed(2)}`;
                const secondKey = `${end.x.toFixed(2)},${end.y.toFixed(2)}`;
                segments.push({
                    key: firstKey < secondKey
                        ? `${firstKey}->${secondKey}`
                        : `${secondKey}->${firstKey}`,
                    start,
                    end,
                    polygonIndex,
                    segmentIndex: index,
                });
            }
        });
        this.obstacleSegments = segments;

        if (segments.length === 0) {
            if (this.obstacleMesh) this.obstacleLayer.remove(this.obstacleMesh);
            this.obstacleMesh = null;
            this.obstacleAnimations.clear();
            this.obstacleReady = true;
            return;
        }

        this.obstacleGeometry ||= new THREE.BoxGeometry(1, 1, 1);
        this.obstacleMaterial ||= new THREE.MeshStandardMaterial({
            color: 0xff4d00,
            emissive: 0x4a1400,
            roughness: 0.1,
            metalness: 0.9,
            transparent: true,
            opacity: 0.72,
            depthWrite: false,
        });
        if (!this.obstacleMesh || this.obstacleMesh.count !== segments.length) {
            if (this.obstacleMesh) this.obstacleLayer.remove(this.obstacleMesh);
            this.obstacleMesh = new THREE.InstancedMesh(
                this.obstacleGeometry,
                this.obstacleMaterial,
                segments.length,
            );
            this.obstacleMesh.frustumCulled = false;
            this.obstacleMesh.renderOrder = 20;
            this.obstacleLayer.add(this.obstacleMesh);
        }

        const now = performance.now();
        const incoming = new Set();
        segments.forEach((segment) => {
            incoming.add(segment.key);
            if (!this.obstacleAnimations.has(segment.key)) {
                this.obstacleAnimations.set(segment.key, {
                    startedAt: now,
                    delay: segment.segmentIndex * 100,
                    duration: 500,
                });
            }
        });
        for (const key of this.obstacleAnimations.keys()) {
            if (!incoming.has(key)) this.obstacleAnimations.delete(key);
        }
        this.obstacleReady = false;
        this.updateObstacleInstances();
    }

    updateObstacleInstances(now = performance.now()) {
        if (!this.obstacleMesh) return;
        const object = new THREE.Object3D();
        let animating = false;
        this.obstacleSegments.forEach((segment, index) => {
            const animation = this.obstacleAnimations.get(segment.key);
            let scaleZ = 1;
            if (animation) {
                const elapsed = now - animation.startedAt;
                if (elapsed < animation.delay) {
                    scaleZ = 0.0001;
                    animating = true;
                } else if (elapsed - animation.delay < animation.duration) {
                    scaleZ = Math.max(0.0001, this.easeOutBack(
                        (elapsed - animation.delay) / animation.duration,
                    ));
                    animating = true;
                }
            }
            const dx = segment.end.x - segment.start.x;
            const dy = segment.end.y - segment.start.y;
            object.position.set(
                segment.start.x + dx / 2,
                segment.start.y + dy / 2,
                scaleZ * OBSTACLE_HEIGHT / 2,
            );
            object.scale.set(Math.hypot(dx, dy), OBSTACLE_THICKNESS, scaleZ * OBSTACLE_HEIGHT);
            object.rotation.set(0, 0, Math.atan2(dy, dx));
            object.updateMatrix();
            this.obstacleMesh.setMatrixAt(index, object.matrix);
        });
        this.obstacleMesh.instanceMatrix.needsUpdate = true;
        this.obstacleReady = !animating;
    }

    easeOutBack(value) {
        const c1 = 2;
        return 1 + (c1 + 1) * Math.pow(value - 1, 3) + c1 * Math.pow(value - 1, 2);
    }

    animate(now = performance.now()) {
        if (!this.obstacleReady) this.updateObstacleInstances(now);
    }

    renderZone(corners) {
        if (!corners || corners.length < 4) {
            this.zoneLine.clear();
            return;
        }
        const points = corners.map(corner => new THREE.Vector3(corner.x, corner.y, 0.08));
        points.push(points[0].clone());
        this.zoneLine.updatePoints(points);
    }

    renderTarget(worldX, worldY, yaw) {
        this.clearTargetObjects();
        const z = 0.02;
        const color = 0x9c27ff;
        const ring = new THREE.Mesh(
            new THREE.RingGeometry(0.3, 0.34, 48),
            new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.8, side: THREE.DoubleSide }),
        );
        ring.position.set(worldX, worldY, z);
        this.targetLayer.add(ring);
        const center = new THREE.Mesh(
            new THREE.CircleGeometry(0.05, 16),
            new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }),
        );
        center.position.set(worldX, worldY, z);
        this.targetLayer.add(center);
        if (yaw !== undefined) {
            const pointer = new THREE.Mesh(
                new THREE.ConeGeometry(0.08, 0.25, 4),
                new THREE.MeshBasicMaterial({ color: 0xf1c84b, transparent: true, opacity: 0.95 }),
            );
            pointer.rotation.z = yaw - Math.PI / 2;
            pointer.position.set(
                worldX + Math.cos(yaw) * 0.42,
                worldY + Math.sin(yaw) * 0.42,
                z,
            );
            this.targetLayer.add(pointer);
        }
    }

    clearTargetObjects() {
        this.targetLayer.children.slice().forEach((child) => {
            this.targetLayer.remove(child);
            disposeObjectTree(child);
        });
    }

    clearTarget() {
        this.clearTargetObjects();
    }

    clearObstacles() {
        this.obstacleLines.forEach(line => line.clear());
        if (this.obstacleMesh) this.obstacleLayer.remove(this.obstacleMesh);
        this.obstacleMesh = null;
        this.obstacleSegments = [];
        this.obstacleAnimations.clear();
        this.obstacleReady = true;
    }

    clearCoverage() {
        this.polygonLine.clear();
        this.obstacleLines.forEach(line => line.clear());
    }

    destroy() {
        this.clearTargetObjects();
        [this.previewLine, this.polygonLine, this.zoneLine, ...this.obstacleLines]
            .forEach(line => line.dispose());
        this.obstacleGeometry?.dispose();
        this.obstacleMaterial?.dispose();
        [this.previewLayer, this.polygonLayer, this.obstacleLayer, this.zoneLayer, this.targetLayer]
            .forEach((layer) => {
                layer.clear();
                this.scene?.remove(layer);
            });
        this.obstacleLines = [];
        this.obstacleSegments = [];
        this.obstacleAnimations.clear();
        this.scene = null;
    }
}
