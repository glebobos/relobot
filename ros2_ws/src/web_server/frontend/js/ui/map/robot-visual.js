import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { disposeObjectTree } from './three-disposal.js';

const ROBOT_SCALE = 0.68;
const ROBOT_FORWARD_OFFSET = 0.20;
const PLAYBACK_DELAY_MS = 200;
const CHASE_CLOSE_TIME = 0.15;
const EMA_DECAY = 0.93;
const EMA_GROWTH = 0.07;
const ROTATION_SMOOTH_FACTOR = 3.0;
const CUTOFF_TIME_MS = 1500;

export class RobotVisual {
    constructor(scene, modelUrl = '/models/robot.glb') {
        this.scene = scene;
        this.modelUrl = modelUrl;
        this.group = new THREE.Group();
        this.scene.add(this.group);
        this.poseBuffer = [];
        this.position = null;
        this.chaseSpeed = 0;
        this.mixer = null;
        this.destroyed = false;
        this.loadModel();
    }

    loadModel() {
        const loader = new GLTFLoader();
        loader.load(
            this.modelUrl,
            (gltf) => {
                if (this.destroyed) {
                    disposeObjectTree(gltf.scene);
                    return;
                }
                const robot = gltf.scene;
                const yUpToZUp = new THREE.Quaternion().setFromAxisAngle(
                    new THREE.Vector3(1, 0, 0),
                    Math.PI / 2,
                );
                const rosForward = new THREE.Quaternion().setFromAxisAngle(
                    new THREE.Vector3(0, 0, 1),
                    Math.PI / 2,
                );
                robot.quaternion.multiplyQuaternions(rosForward, yUpToZUp);

                robot.traverse((child) => {
                    if (!child.isMesh) return;
                    child.castShadow = true;
                    child.receiveShadow = true;
                    const materials = Array.isArray(child.material) ? child.material : [child.material];
                    materials.forEach((material) => {
                        if (material?.map) material.map.colorSpace = THREE.SRGBColorSpace;
                    });
                });

                const box = new THREE.Box3().setFromObject(robot);
                const size = box.getSize(new THREE.Vector3());
                const maxDimension = Math.max(size.x, size.y, size.z, 0.001);
                robot.scale.setScalar(ROBOT_SCALE / maxDimension);

                const scaledBox = new THREE.Box3().setFromObject(robot);
                const center = scaledBox.getCenter(new THREE.Vector3());
                robot.position.set(
                    -center.x + ROBOT_FORWARD_OFFSET,
                    -center.y,
                    -scaledBox.min.z,
                );
                this.group.add(robot);

                if (gltf.animations.length > 0) {
                    this.mixer = new THREE.AnimationMixer(robot);
                    gltf.animations.forEach(clip => this.mixer.clipAction(clip).play());
                }
                console.log('[RobotVisual] GLB model loaded and normalized.');
            },
            undefined,
            (error) => {
                if (this.destroyed) return;
                console.error('[RobotVisual] Failed to load GLB model; using fallback:', error);
                this.group.add(this.createFallbackModel());
            },
        );
    }

    createFallbackModel() {
        const fallback = new THREE.Group();
        const base = new THREE.Mesh(
            new THREE.CylinderGeometry(0.26, 0.26, 0.15, 16),
            new THREE.MeshStandardMaterial({ color: 0x3f51b5, roughness: 0.4, metalness: 0.2 }),
        );
        base.rotation.x = Math.PI / 2;
        base.position.z = 0.075;
        base.castShadow = true;
        base.receiveShadow = true;
        fallback.add(base);

        const direction = new THREE.Mesh(
            new THREE.ConeGeometry(0.06, 0.18, 4),
            new THREE.MeshBasicMaterial({ color: 0xffeb3b }),
        );
        direction.rotation.z = -Math.PI / 2;
        direction.position.set(0.18, 0, 0.15);
        fallback.add(direction);
        return fallback;
    }

    pushPose(position, quaternion, timestamp = performance.now()) {
        this.poseBuffer.push({
            position: new THREE.Vector3(position.x, position.y, 0.01),
            quaternion: new THREE.Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            timestamp,
        });
        const cutoff = timestamp - CUTOFF_TIME_MS;
        while (this.poseBuffer.length > 1 && this.poseBuffer[0].timestamp < cutoff) {
            this.poseBuffer.shift();
        }
    }

    animate(dt, now = performance.now()) {
        if (this.mixer) {
            const speedFactor = 0.35 + Math.min(1.5, this.chaseSpeed * 2);
            this.mixer.update(dt * speedFactor);
        }
        if (this.poseBuffer.length === 0) return;

        const playbackTime = now - PLAYBACK_DELAY_MS;
        let targetPosition = null;
        let targetQuaternion = null;
        for (let i = 0; i < this.poseBuffer.length - 1; i++) {
            const current = this.poseBuffer[i];
            const next = this.poseBuffer[i + 1];
            if (current.timestamp <= playbackTime && next.timestamp > playbackTime) {
                const span = next.timestamp - current.timestamp;
                const ratio = span > 0 ? (playbackTime - current.timestamp) / span : 1;
                targetPosition = current.position.clone().lerp(next.position, ratio);
                targetQuaternion = current.quaternion.clone().slerp(next.quaternion, ratio);
                break;
            }
        }

        if (!targetPosition) {
            const sample = playbackTime < this.poseBuffer[0].timestamp
                ? this.poseBuffer[0]
                : this.poseBuffer[this.poseBuffer.length - 1];
            targetPosition = sample.position;
            targetQuaternion = sample.quaternion;
        }

        const dx = targetPosition.x - this.group.position.x;
        const dy = targetPosition.y - this.group.position.y;
        const distance = Math.hypot(dx, dy);
        const desiredSpeed = distance / CHASE_CLOSE_TIME;
        this.chaseSpeed = this.chaseSpeed * EMA_DECAY + desiredSpeed * EMA_GROWTH;
        const step = this.chaseSpeed * dt;

        if (distance < 0.001 || step >= distance) {
            this.group.position.x = targetPosition.x;
            this.group.position.y = targetPosition.y;
        } else {
            const ratio = step / distance;
            this.group.position.x += dx * ratio;
            this.group.position.y += dy * ratio;
        }
        this.group.position.z = 0.01;
        this.group.quaternion.slerp(targetQuaternion, Math.min(ROTATION_SMOOTH_FACTOR * dt, 1));
        this.position = { x: this.group.position.x, y: this.group.position.y };
    }

    destroy() {
        if (this.destroyed) return;
        this.destroyed = true;
        if (this.mixer) {
            this.mixer.stopAllAction();
            this.mixer = null;
        }
        this.scene?.remove(this.group);
        disposeObjectTree(this.group);
        this.poseBuffer = [];
        this.position = null;
        this.scene = null;
    }
}
