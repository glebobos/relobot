import { TOPICS } from '../shared/constants.js';

/**
 * CameraService provides low-latency video streaming for the ReloBot web interface.
 * Supports two operational modes:
 *  - 'snapshot' (default): Request-driven frame pull with strict timeout and automatic frame-dropping.
 *    Completely eliminates bufferbloat and multi-second delays over lossy outdoor Wi-Fi.
 *  - 'stream': Standard multipart MJPEG HTTP stream fallback.
 */
export class CameraService {
    constructor(imgElementId, options = {}) {
        this.cameraStream = document.getElementById(imgElementId);
        this.clientId = 'web-ui-' + Math.random().toString(36).substring(2, 9);
        this.type = options.type || 'ros_compressed';
        this.topic = options.topic || TOPICS.CAMERA_IMAGE;
        this.quality = options.quality !== undefined ? options.quality : 40;
        this.width = options.width;
        this.height = options.height;
        this.mode = options.mode || 'snapshot'; // 'snapshot' or 'stream'
        this.targetFps = options.targetFps || 15;
        this.targetInterval = 1000 / this.targetFps;

        const protocol = window.location.protocol;
        const host = window.location.host;

        let queryParams = `topic=${this.topic}&type=${this.type}&qos_profile=sensor_data&quality=${this.quality}`;
        if (this.width && this.height) {
            queryParams += `&width=${this.width}&height=${this.height}`;
        }

        this.streamUrl = `${protocol}//${host}/camera-stream/stream?${queryParams}&client_id=${this.clientId}`;
        this.snapshotUrl = `${protocol}//${host}/camera-stream/snapshot?${queryParams}`;

        this.isActive = false;
        this._reconnectTimer = null;
        this._abortController = null;
        this._lastObjectUrl = null;
        this._loopRunning = false;

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
                    console.log(`[CameraService] Page visible, resuming active stream (${this.cameraStream.id})...`);
                    this.connect();
                }
            } else {
                console.log(`[CameraService] Page hidden, suspending stream (${this.cameraStream.id})...`);
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

        if (this.mode === 'snapshot') {
            this._startSnapshotLoop();
        } else {
            this._connectContinuousStream();
        }
    }

    /**
     * Request-driven snapshot loop with strict timeout.
     * Prevents any multi-frame queue accumulation in TCP or browser buffers.
     */
    async _startSnapshotLoop() {
        if (this._loopRunning) return;
        this._loopRunning = true;
        console.log(`[CameraService] Starting zero-lag snapshot loop (${this.cameraStream.id}, target ${this.targetFps} FPS)...`);

        const timeoutMs = Math.max(350, Math.round(this.targetInterval * 2));

        while (this.isActive && this.mode === 'snapshot') {
            const start = performance.now();

            try {
                this._abortController = new AbortController();
                const timerId = setTimeout(() => {
                    if (this._abortController) this._abortController.abort();
                }, timeoutMs);

                const url = `${this.snapshotUrl}&_t=${Date.now()}`;
                const response = await fetch(url, {
                    signal: this._abortController.signal,
                    cache: 'no-store'
                });
                clearTimeout(timerId);

                if (response.ok && this.isActive) {
                    const blob = await response.blob();
                    if (this.isActive && blob.size > 0) {
                        const newUrl = URL.createObjectURL(blob);
                        const oldUrl = this._lastObjectUrl;

                        this.cameraStream.src = newUrl;
                        this._lastObjectUrl = newUrl;

                        // Revoke old blob URL to prevent browser memory leak
                        if (oldUrl) {
                            URL.revokeObjectURL(oldUrl);
                        }
                    }
                }
            } catch (err) {
                // Aborted due to network stall / timeout -> frame dropped cleanly!
                // Next iteration will immediately request the freshest live frame.
            } finally {
                this._abortController = null;
            }

            const elapsed = performance.now() - start;
            const sleepMs = Math.max(0, this.targetInterval - elapsed);

            if (this.isActive) {
                await new Promise(r => setTimeout(r, sleepMs));
            }
        }

        this._loopRunning = false;
    }

    _connectContinuousStream() {
        const url = `${this.streamUrl}&t=${Date.now()}`;
        console.log(`[CameraService] Connecting to continuous MJPEG stream (${this.cameraStream.id}): ${url}`);

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

        if (this._abortController) {
            this._abortController.abort();
            this._abortController = null;
        }

        if (this.cameraStream) {
            this.cameraStream.onload = null;
            this.cameraStream.onerror = null;
            this.cameraStream.removeAttribute('src');
        }

        if (this._lastObjectUrl) {
            URL.revokeObjectURL(this._lastObjectUrl);
            this._lastObjectUrl = null;
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
