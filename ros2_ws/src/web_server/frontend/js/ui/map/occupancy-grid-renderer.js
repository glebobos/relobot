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

class AnimatedCellLayer {
    constructor(group, getRobotPosition, options) {
        this.group = group;
        this.getRobotPosition = getRobotPosition;
        this.height = options.height;
        this.castShadow = options.castShadow ?? false;
        this.renderOrder = options.renderOrder ?? 0;
        this.material = new THREE.MeshStandardMaterial(options.material);
        this.cells = [];
        this.ready = true;
        this.matrixObject = new THREE.Object3D();
        this.worldPosition = new THREE.Vector3();
        this.originPosition = new THREE.Vector3();
        this.originQuaternion = new THREE.Quaternion();
    }

    update(cells, width, height, resolution, origin, resetIdentity) {
        const gridSize = width * height;
        const metadataReset = this.ensureAnimationMetadata(gridSize, resetIdentity);
        const previousCells = metadataReset ? [] : this.cells;
        this.cells = cells;
        this.width = width;
        this.resolution = resolution;
        this.originPosition.set(
            origin.position.x,
            origin.position.y,
            origin.position.z,
        );
        this.originQuaternion.set(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        );

        if (cells.length === 0) {
            this.removeMesh();
            this.ready = true;
            for (const cell of previousCells) {
                this.present[cell.row * width + cell.column] = 0;
            }
            return;
        }

        this.ensureGeometry(resolution);
        this.ensureMesh(cells.length);
        const now = performance.now();
        const robot = this.getRobotPosition?.();

        for (const cell of cells) {
            const index = cell.row * width + cell.column;
            if (this.present[index] === 0) {
                const x = (cell.column + 0.5) * resolution;
                const y = (cell.row + 0.5) * resolution;
                this.worldPosition.set(x, y, 0)
                    .applyQuaternion(this.originQuaternion)
                    .add(this.originPosition);
                this.startedAt[index] = now;
                this.delays[index] = robot
                    ? Math.min(Math.hypot(
                        this.worldPosition.x - robot.x,
                        this.worldPosition.y - robot.y,
                    ) * 150, 1000)
                    : Math.random() * 300;
                this.durations[index] = 600 + Math.random() * 200;
            }
            this.present[index] = 2;
        }
        for (const cell of previousCells) {
            const index = cell.row * width + cell.column;
            if (this.present[index] === 1) this.present[index] = 0;
        }
        for (const cell of cells) {
            this.present[cell.row * width + cell.column] = 1;
        }

        this.ready = false;
        this.updateInstances(now);
    }

    ensureAnimationMetadata(gridSize, resetIdentity) {
        if (this.gridSize !== gridSize) {
            this.gridSize = gridSize;
            this.present = new Uint8Array(gridSize);
            this.startedAt = new Float64Array(gridSize);
            this.delays = new Float32Array(gridSize);
            this.durations = new Float32Array(gridSize);
            return true;
        } else if (resetIdentity) {
            this.present.fill(0);
            return true;
        }
        return false;
    }

    ensureGeometry(resolution) {
        const size = resolution * CELL_SHRINK_FACTOR;
        if (this.geometry?.parameters.width === size) return;
        const previousGeometry = this.geometry;
        this.geometry = new THREE.BoxGeometry(size, size, this.height);
        if (this.mesh) this.mesh.geometry = this.geometry;
        previousGeometry?.dispose();
    }

    ensureMesh(count) {
        if (this.mesh?.count === count) return;
        this.removeMesh();
        this.mesh = new THREE.InstancedMesh(this.geometry, this.material, count);
        this.mesh.frustumCulled = false;
        this.mesh.castShadow = this.castShadow;
        this.mesh.receiveShadow = this.castShadow;
        this.mesh.renderOrder = this.renderOrder;
        this.group.add(this.mesh);
    }

    removeMesh() {
        if (!this.mesh) return;
        this.group.remove(this.mesh);
        this.mesh.dispose?.();
        this.mesh = null;
    }

    updateInstances(now = performance.now()) {
        if (!this.mesh) return;
        let animating = false;
        for (let instanceIndex = 0; instanceIndex < this.cells.length; instanceIndex++) {
            const cell = this.cells[instanceIndex];
            const cellIndex = cell.row * this.width + cell.column;
            const elapsed = now - this.startedAt[cellIndex];
            let scaleZ = 1;
            if (elapsed < this.delays[cellIndex]) {
                scaleZ = 0.0001;
                animating = true;
            } else if (elapsed - this.delays[cellIndex] < this.durations[cellIndex]) {
                scaleZ = Math.max(0.0001, this.easeOutBack(
                    (elapsed - this.delays[cellIndex]) / this.durations[cellIndex],
                ));
                animating = true;
            }
            this.matrixObject.position.set(
                (cell.column + 0.5) * this.resolution,
                (cell.row + 0.5) * this.resolution,
                scaleZ * this.height / 2,
            );
            this.matrixObject.scale.set(1, 1, scaleZ);
            this.matrixObject.updateMatrix();
            this.mesh.setMatrixAt(instanceIndex, this.matrixObject.matrix);
        }
        this.mesh.instanceMatrix.needsUpdate = true;
        this.ready = !animating;
    }

    easeOutBack(value) {
        const c1 = 2;
        const c3 = c1 + 1;
        return 1 + c3 * Math.pow(value - 1, 3) + c1 * Math.pow(value - 1, 2);
    }

    animate(now) {
        if (!this.ready) this.updateInstances(now);
    }

    destroy() {
        this.removeMesh();
        this.geometry?.dispose();
        disposeMaterial(this.material);
        this.geometry = null;
        this.material = null;
        this.cells = [];
        this.present = null;
        this.startedAt = null;
        this.delays = null;
        this.durations = null;
    }
}

export class OccupancyGridRenderer {
    constructor(scene, getRobotPosition) {
        this.scene = scene;
        this.group = new THREE.Group();
        this.scene.add(this.group);
        this.wallLayer = new AnimatedCellLayer(this.group, getRobotPosition, {
            height: WALL_HEIGHT,
            castShadow: true,
            material: { color: WALL_COLOR, roughness: 0.5, metalness: 0.1 },
        });
        this.frontierLayer = new AnimatedCellLayer(this.group, getRobotPosition, {
            height: BOUNDARY_HEIGHT,
            renderOrder: 10,
            material: {
                color: UNEXPLORED_COLOR,
                roughness: 0.1,
                metalness: 0.9,
                transparent: true,
                opacity: 0.32,
                depthWrite: false,
            },
        });
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

        const identityChanged = this.lastWidth !== info.width
            || this.lastHeight !== info.height
            || this.lastResolution !== info.resolution
            || !occupancyOriginEqual(this.lastOrigin, info.origin);
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
        this.wallLayer.update(
            occupied,
            info.width,
            info.height,
            info.resolution,
            info.origin,
            identityChanged,
        );
        this.frontierLayer.update(
            unexploredBoundary,
            info.width,
            info.height,
            info.resolution,
            info.origin,
            identityChanged,
        );
    }

    updateFloor(width, height, resolution, data) {
        const shapeUnchanged = this.floorMesh
            && this.floorShapeMatches(this.floorMesh, width, height, resolution);
        let nextFloor = null;

        if (shapeUnchanged && this.floorShapeMatches(
            this.oldFloorMesh,
            width,
            height,
            resolution,
        )) {
            nextFloor = this.oldFloorMesh;
            this.oldFloorMesh = null;
            this.group.remove(nextFloor);
        } else if (shapeUnchanged && this.floorShapeMatches(
            this.standbyFloorMesh,
            width,
            height,
            resolution,
        )) {
            nextFloor = this.standbyFloorMesh;
            this.standbyFloorMesh = null;
        } else if (!shapeUnchanged) {
            this.disposeFloor(this.oldFloorMesh);
            this.disposeFloor(this.standbyFloorMesh);
            this.oldFloorMesh = null;
            this.standbyFloorMesh = null;
        }

        if (this.floorMesh) {
            if (this.oldFloorMesh) this.disposeFloor(this.oldFloorMesh);
            this.oldFloorMesh = this.floorMesh;
            this.oldFloorMesh.position.z = -0.002;
            this.oldFloorMesh.renderOrder = -11;
            this.oldFloorMesh.material.transparent = true;
            this.oldFloorMesh.material.opacity = 1;
            this.oldFloorMesh.material.needsUpdate = true;
        }

        this.floorMesh = nextFloor || this.createFloor(width, height, resolution);
        this.writeFloorTexture(this.floorMesh.material.map.image.data, width, height, data);
        this.floorMesh.material.map.needsUpdate = true;
        this.floorMesh.material.transparent = true;
        this.floorMesh.material.opacity = 0;
        this.floorMesh.material.needsUpdate = true;
        this.floorMesh.position.z = 0;
        this.floorMesh.renderOrder = -10;
        this.group.add(this.floorMesh);
        this.floorStartedAt = performance.now();
        this.floorDuration = 800;
    }

    createFloor(width, height, resolution) {
        const textureData = new Uint8Array(width * height * 4);
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
        const mesh = new THREE.Mesh(new THREE.PlaneGeometry(mapWidth, mapHeight), material);
        mesh.receiveShadow = true;
        mesh.position.set(mapWidth / 2, mapHeight / 2, 0);
        mesh.userData.occupancyGridShape = { width, height, resolution };
        return mesh;
    }

    floorShapeMatches(mesh, width, height, resolution) {
        const shape = mesh?.userData?.occupancyGridShape;
        return shape?.width === width
            && shape?.height === height
            && shape?.resolution === resolution;
    }

    writeFloorTexture(textureData, width, height, data) {
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
                textureData[offset] = red;
                textureData[offset + 1] = green;
                textureData[offset + 2] = blue;
                textureData[offset + 3] = 255;
            }
        }
    }

    animate(now = performance.now()) {
        if (this.floorMesh?.material?.opacity < 1) {
            const ratio = Math.min((now - this.floorStartedAt) / this.floorDuration, 1);
            this.floorMesh.material.opacity = ratio;
            if (ratio >= 1) {
                this.floorMesh.material.transparent = false;
                this.floorMesh.material.needsUpdate = true;
                if (this.oldFloorMesh) {
                    if (this.floorShapeMatches(
                        this.oldFloorMesh,
                        this.floorMesh.userData.occupancyGridShape.width,
                        this.floorMesh.userData.occupancyGridShape.height,
                        this.floorMesh.userData.occupancyGridShape.resolution,
                    )) {
                        this.group.remove(this.oldFloorMesh);
                        this.disposeFloor(this.standbyFloorMesh);
                        this.standbyFloorMesh = this.oldFloorMesh;
                    } else {
                        this.disposeFloor(this.oldFloorMesh);
                    }
                    this.oldFloorMesh = null;
                }
            }
        }
        this.wallLayer.animate(now);
        this.frontierLayer.animate(now);
    }

    disposeFloor(mesh) {
        detachAndDispose(this.group, mesh);
    }

    destroy() {
        if (this.destroyed) return;
        this.destroyed = true;
        this.disposeFloor(this.floorMesh);
        this.disposeFloor(this.oldFloorMesh);
        this.disposeFloor(this.standbyFloorMesh);
        this.floorMesh = null;
        this.oldFloorMesh = null;
        this.standbyFloorMesh = null;
        this.wallLayer.destroy();
        this.frontierLayer.destroy();
        this.scene?.remove(this.group);
        this.scene = null;
    }
}
