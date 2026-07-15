import * as THREE from 'three';

export class MapCameraController {
    constructor(camera, controls) {
        this.camera = camera;
        this.controls = controls;
        this.animationFrameId = null;
        this.savedState = null;
    }

    animate(from, to, duration, onDone) {
        this.cancelAnimation();
        let startedAt = null;
        const ease = value => value < 0.5
            ? 4 * value * value * value
            : 1 - Math.pow(-2 * value + 2, 3) / 2;
        const step = (timestamp) => {
            if (startedAt === null) startedAt = timestamp;
            const raw = Math.min((timestamp - startedAt) / duration, 1);
            const progress = ease(raw);
            this.camera.position.lerpVectors(from.position, to.position, progress);
            this.camera.up.lerpVectors(from.up, to.up, progress);
            this.camera.quaternion.copy(from.quaternion).slerp(to.quaternion, progress);
            this.camera.updateMatrixWorld();
            if (raw < 1) {
                this.animationFrameId = requestAnimationFrame(step);
            } else {
                this.animationFrameId = null;
                onDone?.();
            }
        };
        this.animationFrameId = requestAnimationFrame(step);
    }

    snapTopDown() {
        if (!this.camera || !this.controls || this.savedState) return;
        const target = this.controls.target;
        this.savedState = {
            position: this.camera.position.clone(),
            up: this.camera.up.clone(),
            quaternion: this.camera.quaternion.clone(),
            target: target.clone(),
        };
        this.controls.enabled = false;
        const distance = this.camera.position.distanceTo(target);
        const targetPosition = new THREE.Vector3(target.x, target.y, target.z + distance);
        const targetUp = new THREE.Vector3(0, 1, 0);
        const temporaryCamera = this.camera.clone();
        temporaryCamera.position.copy(targetPosition);
        temporaryCamera.up.copy(targetUp);
        temporaryCamera.lookAt(target);
        this.animate(
            {
                position: this.camera.position.clone(),
                up: this.camera.up.clone(),
                quaternion: this.camera.quaternion.clone(),
            },
            { position: targetPosition, up: targetUp, quaternion: temporaryCamera.quaternion.clone() },
            420,
            () => {
                this.camera.position.copy(targetPosition);
                this.camera.up.copy(targetUp);
                this.camera.lookAt(target);
            },
        );
    }

    restore() {
        if (!this.savedState || !this.camera || !this.controls) return;
        const saved = this.savedState;
        this.savedState = null;
        this.animate(
            {
                position: this.camera.position.clone(),
                up: this.camera.up.clone(),
                quaternion: this.camera.quaternion.clone(),
            },
            { position: saved.position, up: saved.up, quaternion: saved.quaternion },
            420,
            () => {
                this.camera.position.copy(saved.position);
                this.camera.up.copy(saved.up);
                this.camera.quaternion.copy(saved.quaternion);
                this.controls.target.copy(saved.target);
                this.controls.enabled = true;
                this.controls.update();
            },
        );
    }

    cancelAnimation() {
        if (this.animationFrameId !== null) cancelAnimationFrame(this.animationFrameId);
        this.animationFrameId = null;
    }

    destroy() {
        this.cancelAnimation();
        if (this.controls) this.controls.enabled = true;
        this.savedState = null;
        this.camera = null;
        this.controls = null;
    }
}
