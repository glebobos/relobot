import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export class MapScene {
    constructor(container, onResize = null) {
        if (!container) throw new Error('MapScene requires a container element.');
        this.container = container;
        this.onResize = onResize;
        this.destroyed = false;
        this.lastFrameTime = 0;
        this.initialize();
    }

    initialize() {
        this.container.innerHTML = '';
        this.canvas = document.createElement('canvas');
        this.canvas.id = 'scene';
        Object.assign(this.canvas.style, {
            width: '100%',
            height: '100%',
            display: 'block',
            touchAction: 'none',
        });
        this.container.appendChild(this.canvas);

        const width = Math.max(this.container.offsetWidth, 1);
        const height = Math.max(this.container.offsetHeight, 1);
        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvas,
            antialias: true,
            preserveDrawingBuffer: true,
        });
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.setSize(width, height, false);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFShadowMap;
        this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
        this.renderer.toneMappingExposure = 1.05;

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0c0f14);
        this.camera = new THREE.PerspectiveCamera(52, width / height, 0.1, 100);
        this.camera.position.set(-3, -5, 5.5);
        this.camera.up.set(0, 0, 1);
        this.controls = new OrbitControls(this.camera, this.canvas);
        this.controls.enableDamping = true;
        this.controls.maxPolarAngle = Math.PI * 0.46;
        this.controls.minDistance = 2;
        this.controls.maxDistance = 30;
        this.controls.target.set(0, 0, 0);

        const sun = new THREE.DirectionalLight(0xfff5d6, 3.4);
        sun.position.set(-10, -15, 20);
        sun.castShadow = true;
        sun.shadow.mapSize.set(1024, 1024);
        Object.assign(sun.shadow.camera, {
            near: 1,
            far: 40,
            left: -15,
            right: 15,
            top: 15,
            bottom: -15,
        });
        this.scene.add(sun);
        this.scene.add(new THREE.HemisphereLight(0xddefff, 0x2c3427, 1.25));

        this.contextLostHandler = (event) => {
            event.preventDefault();
            console.warn('[MapScene] WebGL context lost.');
            this.stop();
        };
        this.contextRestoredHandler = () => {
            console.info('[MapScene] WebGL context restored.');
            this.start(this.frameCallback);
        };
        this.resizeHandler = () => this.resize();
        this.canvas.addEventListener('webglcontextlost', this.contextLostHandler, false);
        this.canvas.addEventListener('webglcontextrestored', this.contextRestoredHandler, false);
        window.addEventListener('resize', this.resizeHandler);
    }

    start(frameCallback) {
        this.frameCallback = frameCallback;
        if (this.animationFrameId || this.destroyed) return;
        this.lastFrameTime = performance.now();
        const frame = (now) => {
            if (this.destroyed) return;
            const dt = Math.min((now - this.lastFrameTime) / 1000, 0.05);
            this.lastFrameTime = now;
            this.controls?.update();
            this.frameCallback?.(dt, now);
            this.renderer?.render(this.scene, this.camera);
            this.animationFrameId = requestAnimationFrame(frame);
        };
        this.animationFrameId = requestAnimationFrame(frame);
    }

    stop() {
        if (this.animationFrameId) cancelAnimationFrame(this.animationFrameId);
        this.animationFrameId = null;
    }

    resize() {
        if (this.destroyed) return;
        const width = Math.max(this.container.offsetWidth, 1);
        const height = Math.max(this.container.offsetHeight, 1);
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height, false);
        this.onResize?.();
    }

    screenToWorld(screenX, screenY) {
        const rectangle = this.container.getBoundingClientRect();
        if (!rectangle.width || !rectangle.height) return null;
        const raycaster = new THREE.Raycaster();
        raycaster.setFromCamera({
            x: ((screenX - rectangle.left) / rectangle.width) * 2 - 1,
            y: -((screenY - rectangle.top) / rectangle.height) * 2 + 1,
        }, this.camera);
        const result = new THREE.Vector3();
        return raycaster.ray.intersectPlane(
            new THREE.Plane(new THREE.Vector3(0, 0, 1), 0),
            result,
        ) ? { x: result.x, y: result.y } : null;
    }

    destroy() {
        if (this.destroyed) return;
        this.destroyed = true;
        this.stop();
        window.removeEventListener('resize', this.resizeHandler);
        this.canvas.removeEventListener('webglcontextlost', this.contextLostHandler);
        this.canvas.removeEventListener('webglcontextrestored', this.contextRestoredHandler);
        this.controls?.dispose();
        this.renderer?.dispose();
        this.renderer?.forceContextLoss();
        this.canvas.remove();
        this.scene.clear();
        this.controls = null;
        this.renderer = null;
        this.scene = null;
        this.camera = null;
    }
}
