import * as THREE from 'three';
import { detachAndDispose, disposeMaterial } from './three-disposal.js';
import {
    classifyOccupancyCells,
    cloneOccupancyOrigin,
    isValidOccupancyGrid,
    occupancyDataEqual,
    occupancyOriginEqual,
} from './occupancy-grid-model.js';

const WALL_HEIGHT = 0.35;
const BOUNDARY_HEIGHT = 0.22;
const CELL_SHRINK_FACTOR = 0.96;
const WALL_COLOR = 0x1f242e;
const UNEXPLORED_COLOR = 0x00e676;

export class OccupancyGridRenderer {
    constructor(scene, getRobotPosition) {
        this.scene = scene;
        this.getRobotPosition = getRobotPosition;
        this.group = new THREE.Group();
        this.scene.add(this.group);
        this.wallCells = [];
        this.unexploredCells = [];
        this.wallAnimations = new Map();
        this.unexploredAnimations = new Map();
        this.wallResolution = 0.05;
        this.unexploredResolution = 0.05;
        this.wallReady = true;
        this.unexploredReady = true;
        this.destroyed = false;
    }

    handleMessage(message) {
        if (this.destroyed) return;
        const { info, data } = message;
        if (!isValidOccupancyGrid(message)) {
            console.warn('[OccupancyGridRenderer] Rejected malformed occupancy grid.');
            return;
        }

        if (this.lastWidth === info.width
            && this.lastHeight === info.height
            && this.lastResolution === info.resolution
            && occupancyOriginEqual(this.lastOrigin, info.origin)
            && occupancyDataEqual(this.lastData, data)) {
            return;
        }

        this.lastWidth = info.width;
        this.lastHeight = info.height;
        this.lastResolution = info.resolution;
        this.lastOrigin = cloneOccupancyOrigin(info.origin);
        this.lastData = new Int8Array(data);

        this.group.position.set(
            info.origin.position.x,
            info.origin.position.y,
            info.origin.position.z,
        );
        this.group.quaternion.set(
            info.origin.orientation.x,
            info.origin.orientation.y,
            info.origin.orientation.z,
            info.origin.orientation.w,
        );

        const { occupied, unexploredBoundary } = classifyOccupancyCells(
            data,
            info.width,
            info.height,
        );

        this.updateFloor(info.width, info.height, info.resolution, data);
        this.updateCellLayer('wall', info.resolution, occupied, info.origin);
        this.updateCellLayer('unexplored', info.resolution, unexploredBoundary, info.origin);
    }

    updateFloor(width, height, resolution, data) {
        if (this.floorMesh) {
            if (this.oldFloorMesh) this.disposeFloor(this.oldFloorMesh);
            this.oldFloorMesh = this.floorMesh;
            this.oldFloorMesh.position.z = -0.002;
            this.oldFloorMesh.renderOrder = -11;
            this.oldFloorMesh.material.transparent = true;
            this.oldFloorMesh.material.opacity = 1;
        }

        const textureData = new Uint8Array(width * height * 4);
        for (let row = 0; row < height; row++) {
            const textureRow = height - 1 - row;
            for (let column = 0; column < width; column++) {
                const value = data[row * width + column];
                const offset = (textureRow * width + column) * 4;
                let red = 12;
                let green = 15;
                let blue = 20;
                if (value === 0) {
                    red = 110; green = 152; blue = 104;
                } else if (value === 100) {
                    red = 20; green = 22; blue = 24;
                }
                if (column % 10 === 0 || textureRow % 10 === 0) {
                    red = Math.round(red * 0.98 + 5.1);
                    green = Math.round(green * 0.98 + 5.1);
                    blue = Math.round(blue * 0.98 + 5.1);
                }
                textureData.set([red, green, blue, 255], offset);
            }
        }

        const texture = new THREE.DataTexture(textureData, width, height, THREE.RGBAFormat);
        texture.minFilter = THREE.NearestFilter;
        texture.magFilter = THREE.NearestFilter;
        texture.colorSpace = THREE.SRGBColorSpace;
        texture.flipY = true;
        texture.needsUpdate = true;

        const mapWidth = width * resolution;
        const mapHeight = height * resolution;
        const material = new THREE.MeshStandardMaterial({
            map: texture,
            roughness: 0.8,
            metalness: 0.05,
            transparent: true,
            opacity: 0,
        });
        this.floorMesh = new THREE.Mesh(new THREE.PlaneGeometry(mapWidth, mapHeight), material);
        this.floorMesh.receiveShadow = true;
        this.floorMesh.position.set(mapWidth / 2, mapHeight / 2, 0);
        this.floorMesh.renderOrder = -10;
        this.group.add(this.floorMesh);
        this.floorStartedAt = performance.now();
        this.floorDuration = 800;
    }

    updateCellLayer(kind, resolution, cells, origin) {
        const wall = kind === 'wall';
        const meshKey = wall ? 'wallMesh' : 'unexploredMesh';
        const geometryKey = wall ? 'wallGeometry' : 'unexploredGeometry';
        const materialKey = wall ? 'wallMaterial' : 'unexploredMaterial';
        const cellsKey = wall ? 'wallCells' : 'unexploredCells';
        const animations = wall ? this.wallAnimations : this.unexploredAnimations;
        const height = wall ? WALL_HEIGHT : BOUNDARY_HEIGHT;

        this[cellsKey] = cells;
        this[wall ? 'wallResolution' : 'unexploredResolution'] = resolution;
        this[wall ? 'wallOrigin' : 'unexploredOrigin'] = cloneOccupancyOrigin(origin);

        if (cells.length === 0) {
            if (this[meshKey]) this.group.remove(this[meshKey]);
            this[meshKey] = null;
            animations.clear();
            this[wall ? 'wallReady' : 'unexploredReady'] = true;
            return;
        }

        const size = resolution * CELL_SHRINK_FACTOR;
        if (!this[geometryKey] || this[geometryKey].parameters.width !== size) {
            const previousGeometry = this[geometryKey];
            this[geometryKey] = new THREE.BoxGeometry(size, size, height);
            if (this[meshKey]) this[meshKey].geometry = this[geometryKey];
            previousGeometry?.dispose();
        }
        if (!this[materialKey]) {
            this[materialKey] = wall
                ? new THREE.MeshStandardMaterial({ color: WALL_COLOR, roughness: 0.5, metalness: 0.1 })
                : new THREE.MeshStandardMaterial({
                    color: UNEXPLORED_COLOR,
                    roughness: 0.1,
                    metalness: 0.9,
                    transparent: true,
                    opacity: 0.32,
                    depthWrite: false,
                });
        }

        if (!this[meshKey] || this[meshKey].count !== cells.length) {
            if (this[meshKey]) this.group.remove(this[meshKey]);
            this[meshKey] = new THREE.InstancedMesh(
                this[geometryKey],
                this[materialKey],
                cells.length,
            );
            this[meshKey].frustumCulled = false;
            if (wall) {
                this[meshKey].castShadow = true;
                this[meshKey].receiveShadow = true;
            } else {
                this[meshKey].renderOrder = 10;
            }
            this.group.add(this[meshKey]);
        }

        const now = performance.now();
        const incomingKeys = new Set();
        cells.forEach((cell) => {
            const world = this.cellWorldPosition(cell, resolution, origin);
            const key = `${world.x.toFixed(3)},${world.y.toFixed(3)}`;
            incomingKeys.add(key);
            if (!animations.has(key)) {
                const robot = this.getRobotPosition?.();
                const delay = robot
                    ? Math.min(Math.hypot(world.x - robot.x, world.y - robot.y) * 150, 1000)
                    : Math.random() * 300;
                animations.set(key, {
                    startedAt: now,
                    delay,
                    duration: 600 + Math.random() * 200,
                });
            }
        });
        for (const key of animations.keys()) {
            if (!incomingKeys.has(key)) animations.delete(key);
        }
        this[wall ? 'wallReady' : 'unexploredReady'] = false;
        this.updateCellInstances(kind);
    }

    cellWorldPosition(cell, resolution, origin) {
        const local = new THREE.Vector3(
            (cell.column + 0.5) * resolution,
            (cell.row + 0.5) * resolution,
            0,
        );
        const rotation = new THREE.Quaternion(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        );
        return local.applyQuaternion(rotation).add(new THREE.Vector3(
            origin.position.x,
            origin.position.y,
            origin.position.z,
        ));
    }

    updateCellInstances(kind) {
        const wall = kind === 'wall';
        const mesh = this[wall ? 'wallMesh' : 'unexploredMesh'];
        const cells = this[wall ? 'wallCells' : 'unexploredCells'];
        const animations = wall ? this.wallAnimations : this.unexploredAnimations;
        const resolution = this[wall ? 'wallResolution' : 'unexploredResolution'];
        const origin = this[wall ? 'wallOrigin' : 'unexploredOrigin'];
        const height = wall ? WALL_HEIGHT : BOUNDARY_HEIGHT;
        if (!mesh || !cells) return;

        const now = performance.now();
        const object = new THREE.Object3D();
        let animating = false;
        cells.forEach((cell, index) => {
            const world = this.cellWorldPosition(cell, resolution, origin);
            const animation = animations.get(`${world.x.toFixed(3)},${world.y.toFixed(3)}`);
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
            object.position.set(
                (cell.column + 0.5) * resolution,
                (cell.row + 0.5) * resolution,
                scaleZ * height / 2,
            );
            object.scale.set(1, 1, scaleZ);
            object.updateMatrix();
            mesh.setMatrixAt(index, object.matrix);
        });
        mesh.instanceMatrix.needsUpdate = true;
        this[wall ? 'wallReady' : 'unexploredReady'] = !animating;
    }

    easeOutBack(value) {
        const c1 = 2;
        const c3 = c1 + 1;
        return 1 + c3 * Math.pow(value - 1, 3) + c1 * Math.pow(value - 1, 2);
    }

    animate(now = performance.now()) {
        if (this.floorMesh?.material?.opacity < 1) {
            const ratio = Math.min((now - this.floorStartedAt) / this.floorDuration, 1);
            this.floorMesh.material.opacity = ratio;
            if (ratio >= 1) {
                this.floorMesh.material.transparent = false;
                this.floorMesh.material.needsUpdate = true;
                if (this.oldFloorMesh) {
                    this.disposeFloor(this.oldFloorMesh);
                    this.oldFloorMesh = null;
                }
            }
        }
        if (!this.wallReady) this.updateCellInstances('wall');
        if (!this.unexploredReady) this.updateCellInstances('unexplored');
    }

    disposeFloor(mesh) {
        detachAndDispose(this.group, mesh);
    }

    destroy() {
        if (this.destroyed) return;
        this.destroyed = true;
        this.disposeFloor(this.floorMesh);
        this.disposeFloor(this.oldFloorMesh);
        this.floorMesh = null;
        this.oldFloorMesh = null;
        if (this.wallMesh) this.group.remove(this.wallMesh);
        if (this.unexploredMesh) this.group.remove(this.unexploredMesh);
        this.wallGeometry?.dispose();
        this.unexploredGeometry?.dispose();
        disposeMaterial(this.wallMaterial);
        disposeMaterial(this.unexploredMaterial);
        this.wallGeometry = null;
        this.unexploredGeometry = null;
        this.wallMaterial = null;
        this.unexploredMaterial = null;
        this.wallMesh = null;
        this.unexploredMesh = null;
        this.wallAnimations.clear();
        this.unexploredAnimations.clear();
        this.scene?.remove(this.group);
        this.scene = null;
    }
}
