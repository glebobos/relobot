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

        this._abortController = null;
        this._reconnectTimer = null;
        this._currentBlobUrl = null;

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

        if (this._abortController) {
            this._abortController.abort();
            this._abortController = null;
        }
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }

        this._abortController = new AbortController();
        this._startStream(this._abortController.signal);
    }

    /**
     * Streams MJPEG/ros_compressed using fetch + ReadableStream.
     * Conflates frames: if multiple frames arrive in a chunk, only renders the newest,
     * completely eliminating FIFO buffer bloat and latency accumulation.
     */
    async _startStream(signal) {
        if (!this.cameraStream) return;
        const url = `${this.baseUrl}&t=${Date.now()}`;
        console.log(`[CameraService] Connecting to zero-buffer stream (${this.cameraStream.id}): ${url}`);

        try {
            const response = await fetch(url, { signal });
            if (!response.ok || !response.body) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            console.log(`[CameraService] Camera stream connected (${this.cameraStream.id})`);
            const reader = response.body.getReader();
            let buffer = new Uint8Array(0);

            while (!signal.aborted) {
                const { done, value } = await reader.read();
                if (done) break;

                // Merge incoming chunk into accumulator
                const merged = new Uint8Array(buffer.length + value.length);
                merged.set(buffer, 0);
                merged.set(value, buffer.length);
                buffer = merged;

                // Extract complete JPEG frames (SOI: 0xFF 0xD8, EOI: 0xFF 0xD9)
                let latestFrame = null;
                let latestFrameEnd = -1;
                let searchIdx = 0;

                while (true) {
                    const soi = this._findMarker(buffer, 0xFF, 0xD8, searchIdx);
                    if (soi === -1) break;
                    const eoi = this._findMarker(buffer, 0xFF, 0xD9, soi + 2);
                    if (eoi === -1) {
                        searchIdx = soi; // Keep from SOI onward
                        break;
                    }

                    // Complete JPEG frame found
                    latestFrame = buffer.slice(soi, eoi + 2);
                    latestFrameEnd = eoi + 2;
                    searchIdx = eoi + 2;
                }

                // If frames arrived, only render the LATEST frame and discard all older ones!
                if (latestFrame) {
                    this._renderFrame(latestFrame);
                    buffer = buffer.slice(latestFrameEnd);
                } else if (buffer.length > 2 * 1024 * 1024) {
                    // Buffer runaway protection
                    buffer = buffer.slice(-65536);
                }
            }
        } catch (err) {
            if (signal.aborted) return;
            console.warn(`[CameraService] Stream connection lost (${this.cameraStream.id}):`, err.message || err);
            if (this.isActive) {
                this._scheduleReconnect();
            }
        }
    }

    _findMarker(buf, b1, b2, startIdx = 0) {
        const len = buf.length - 1;
        for (let i = startIdx; i < len; i++) {
            if (buf[i] === b1 && buf[i + 1] === b2) {
                return i;
            }
        }
        return -1;
    }

    _renderFrame(frameBytes) {
        if (!this.cameraStream) return;
        const blob = new Blob([frameBytes], { type: 'image/jpeg' });
        const newUrl = URL.createObjectURL(blob);
        const prevUrl = this._currentBlobUrl;
        this._currentBlobUrl = newUrl;
        this.cameraStream.src = newUrl;
        if (prevUrl) {
            setTimeout(() => URL.revokeObjectURL(prevUrl), 100);
        }
    }

    _scheduleReconnect() {
        if (this._reconnectTimer || !this.isActive) return;
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            if (this.isActive && (!this._abortController || this._abortController.signal.aborted)) {
                this._abortController = new AbortController();
                this._startStream(this._abortController.signal);
            }
        }, 1000);
    }

    stopStreamOnly() {
        if (this._abortController) {
            this._abortController.abort();
            this._abortController = null;
        }
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }
        if (this._currentBlobUrl) {
            URL.revokeObjectURL(this._currentBlobUrl);
            this._currentBlobUrl = null;
        }
        if (this.cameraStream) {
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
