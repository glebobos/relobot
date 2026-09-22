import { TOPICS } from '../shared/constants.js';

export class CameraService {
    constructor(imgElementId, options = {}) {
        this.cameraStream = document.getElementById(imgElementId);
        this.clientId = 'web-ui-' + Math.random().toString(36).substring(2, 9);
        this.type = options.type || 'ros_compressed';
        this.topic = options.topic || (this.type === 'ros_compressed' ? (TOPICS.CAMERA_RAW || '/camera/image_raw') : (TOPICS.CAMERA_IMAGE || '/camera/image_rect'));
        this.quality = options.quality !== undefined ? options.quality : 50;
        this.width = options.width;
        this.height = options.height;

        let streamUrl = `${window.location.protocol}//${window.location.host}/camera-stream/stream?topic=${this.topic}&type=${this.type}&qos_profile=sensor_data&client_id=${this.clientId}`;
        if (this.type === 'mjpeg') {
            streamUrl += `&quality=${this.quality}`;
            if (this.width && this.height) {
                streamUrl += `&width=${this.width}&height=${this.height}`;
            }
        }
        this.baseUrl = streamUrl;
        this.isActive = false;
        this._reconnectTimer = null;

        this.init();
    }

    reconnect() {
        if (!this.cameraStream) return;
        console.log(`[CameraService] Forcing stream reconnect (${this.cameraStream.id})...`);
        this.stopStreamOnly();
        setTimeout(() => {
            if (this.isActive) this.connect();
        }, 50);
    }

    init() {
        if (!this.cameraStream) return;

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

        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }

        const url = `${this.baseUrl}&t=${Date.now()}`;
        console.log(`[CameraService] Connecting to native stream (${this.cameraStream.id}): ${url}`);

        this.cameraStream.onload = () => {
            console.log(`[CameraService] Camera stream connected (${this.cameraStream.id})`);
        };

        this.cameraStream.onerror = () => {
            console.warn(`[CameraService] Camera stream error (${this.cameraStream.id})`);
            if (this.isActive) {
                this._scheduleReconnect();
            }
        };

        this.cameraStream.src = url;
    }

    _scheduleReconnect() {
        if (this._reconnectTimer || !this.isActive) return;
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            if (this.isActive) {
                this.connect();
            }
        }, 2000);
    }

    stopStreamOnly() {
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }
        if (this.cameraStream) {
            this.cameraStream.onload = null;
            this.cameraStream.onerror = null;
            this.cameraStream.removeAttribute('src');
        }
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
