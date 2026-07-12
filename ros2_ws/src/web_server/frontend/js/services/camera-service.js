import { TOPICS } from '../shared/constants.js';

export class CameraService {
    constructor(imgElementId) {
        this.cameraStream = document.getElementById(imgElementId);
        this.clientId = 'web-ui-' + Math.random().toString(36).substring(2, 9);
        const isHttps = window.location.protocol === 'https:';
        const protocol = isHttps ? 'https:' : 'http:';
        const port = isHttps ? '' : ':8080';
        const path = isHttps ? '/camera-stream' : '';
        this.baseUrl = `${protocol}//${window.location.hostname}${port}${path}/stream?topic=${TOPICS.CAMERA_IMAGE}&type=mjpeg&client_id=${this.clientId}`;
        this.isActive = false;
        this.init();
    }

    init() {
        if (!this.cameraStream) return;
        this.connect();

        this.visibilityHandler = () => {
            if (document.visibilityState === 'visible') {
                if (this.isActive) {
                    console.log(`[CameraService] Page visible, reconnecting active stream (${this.cameraStream.id})...`);
                    this.connect();
                }
            } else {
                console.log(`[CameraService] Page hidden, disconnecting stream (${this.cameraStream.id})...`);
                this.stopStreamOnly();
            }
        };

        this.beforeUnloadHandler = () => this.stop();

        document.addEventListener('visibilitychange', this.visibilityHandler);
        window.addEventListener('beforeunload', this.beforeUnloadHandler);
    }

    connect() {
        if (!this.cameraStream) return;
        this.isActive = true;
        console.log(`[CameraService] Connecting to camera stream (${this.cameraStream.id})...`);
        this.cameraStream.src = this.baseUrl + '&t=' + Date.now();
        this.cameraStream.onload = () => console.log(`[CameraService] Camera stream connected (${this.cameraStream.id})`);
        this.cameraStream.onerror = () => console.warn(`[CameraService] Camera stream error (${this.cameraStream.id})`);
    }

    stopStreamOnly() {
        if (!this.cameraStream) return;
        this.cameraStream.removeAttribute('src');
    }

    stop() {
        this.isActive = false;
        console.log(`[CameraService] Stopping camera stream (${this.cameraStream?.id})...`);
        this.stopStreamOnly();
    }

    destroy() {
        document.removeEventListener('visibilitychange', this.visibilityHandler);
        window.removeEventListener('beforeunload', this.beforeUnloadHandler);
        this.stop();
    }
}
