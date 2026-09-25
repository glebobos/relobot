/**
 * @file map-stream-service.js
 * @brief High-speed binary WebSocket service for streaming full-resolution
 * compressed OccupancyGrid map frames directly to the browser with zero JSON overhead.
 */

const MAGIC_RMAP = 0x50414D52; // 'RMAP' in little-endian (0x52, 0x4D, 0x41, 0x50)
const HEADER_SIZE = 48;

class MapStreamService {
    constructor() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        this.url = `${protocol}//${window.location.host}/map-ws/`;

        this._socket = null;
        this._listeners = new Set();
        this._reconnectTimer = null;
        this._backoffMs = 500;
        this._minBackoffMs = 500;
        this._maxBackoffMs = 5000;
        this._isPaused = false;
        this._lastMsg = null;

        this._initVisibilityListener();
    }

    /**
     * Subscribes a listener to receive decompressed OccupancyGrid messages.
     * @param {(msg: { info: object, data: Int8Array }) => void} callback
     * @returns {() => void} Unsubscribe function
     */
    subscribe(callback) {
        this._listeners.add(callback);
        // Deliver latest cached message immediately if available
        if (this._lastMsg) {
            try {
                callback(this._lastMsg);
            } catch (err) {
                console.error('[MapStreamService] Error in initial subscriber callback:', err);
            }
        }

        if (!this._socket && !this._isPaused) {
            this._connect();
        }

        return () => {
            this._listeners.delete(callback);
            if (this._listeners.size === 0) {
                this._disconnect();
            }
        };
    }

    _connect() {
        if (this._socket || this._isPaused) return;

        try {
            this._socket = new WebSocket(this.url);
            this._socket.binaryType = 'arraybuffer';

            this._socket.onopen = () => {
                console.log('[MapStreamService] Connected to binary map stream at', this.url);
                this._backoffMs = this._minBackoffMs;
                if (this._reconnectTimer) {
                    clearTimeout(this._reconnectTimer);
                    this._reconnectTimer = null;
                }
            };

            this._socket.onmessage = async (event) => {
                if (!(event.data instanceof ArrayBuffer)) {
                    console.warn('[MapStreamService] Unexpected non-binary message received');
                    return;
                }
                try {
                    const msg = await this._parseBinaryFrame(event.data);
                    if (msg) {
                        this._lastMsg = msg;
                        for (const listener of this._listeners) {
                            try {
                                listener(msg);
                            } catch (err) {
                                console.error('[MapStreamService] Listener error:', err);
                            }
                        }
                    }
                } catch (err) {
                    console.error('[MapStreamService] Failed to parse binary map frame:', err);
                }
            };

            this._socket.onerror = (err) => {
                console.warn('[MapStreamService] WebSocket error:', err);
            };

            this._socket.onclose = () => {
                this._socket = null;
                if (!this._isPaused && this._listeners.size > 0) {
                    this._scheduleReconnect();
                }
            };
        } catch (err) {
            console.error('[MapStreamService] Connection initialization failed:', err);
            this._scheduleReconnect();
        }
    }

    _disconnect() {
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }
        if (this._socket) {
            try {
                this._socket.close();
            } catch (_) {}
            this._socket = null;
        }
    }

    _scheduleReconnect() {
        if (this._reconnectTimer || this._isPaused) return;
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            this._backoffMs = Math.min(Math.round(this._backoffMs * 1.5), this._maxBackoffMs);
            if (!this._isPaused && this._listeners.size > 0) {
                this._connect();
            }
        }, this._backoffMs);
    }

    _initVisibilityListener() {
        document.addEventListener('visibilitychange', () => {
            if (document.visibilityState === 'visible') {
                this._isPaused = false;
                if (this._listeners.size > 0 && !this._socket) {
                    console.log('[MapStreamService] Tab active: resuming binary map stream...');
                    this._connect();
                }
            } else {
                this._isPaused = true;
                console.log('[MapStreamService] Tab hidden: pausing binary map stream...');
                this._disconnect();
            }
        });
    }

    /**
     * Decodes the 48-byte header and decompresses the ZLIB payload using native DecompressionStream.
     * @param {ArrayBuffer} buffer
     * @returns {Promise<{ info: object, data: Int8Array } | null>}
     */
    async _parseBinaryFrame(buffer) {
        if (buffer.byteLength < HEADER_SIZE) {
            console.warn('[MapStreamService] Truncated binary frame (< 48 bytes)');
            return null;
        }

        const view = new DataView(buffer);

        // Verify 'RMAP' magic header
        if (
            view.getUint8(0) !== 0x52 || // 'R'
            view.getUint8(1) !== 0x4D || // 'M'
            view.getUint8(2) !== 0x41 || // 'A'
            view.getUint8(3) !== 0x50    // 'P'
        ) {
            console.warn('[MapStreamService] Invalid RMAP magic identifier');
            return null;
        }

        const version = view.getUint8(4);
        const compression = view.getUint8(5);
        if (version !== 1) {
            console.warn(`[MapStreamService] Unsupported protocol version: ${version}`);
            return null;
        }

        const width = view.getUint32(8, true);
        const height = view.getUint32(12, true);
        const resolution = view.getFloat32(16, true);

        const origin = {
            position: {
                x: view.getFloat32(20, true),
                y: view.getFloat32(24, true),
                z: view.getFloat32(28, true),
            },
            orientation: {
                x: view.getFloat32(32, true),
                y: view.getFloat32(36, true),
                z: view.getFloat32(40, true),
                w: view.getFloat32(44, true),
            },
        };

        const totalExpectedCells = width * height;
        let rawData;

        if (compression === 1) {
            // Decompress ZLIB payload natively off-thread via DecompressionStream
            const compressedSlice = new Uint8Array(buffer, HEADER_SIZE);
            const stream = new Response(compressedSlice).body.pipeThrough(new DecompressionStream('deflate'));
            const decompressedBuffer = await new Response(stream).arrayBuffer();
            rawData = new Int8Array(decompressedBuffer);
        } else {
            // Uncompressed raw bytes fallback
            rawData = new Int8Array(buffer, HEADER_SIZE, totalExpectedCells);
        }

        if (rawData.length !== totalExpectedCells) {
            console.warn(`[MapStreamService] Grid cell count mismatch: expected ${totalExpectedCells}, got ${rawData.length}`);
            return null;
        }

        return {
            info: {
                width,
                height,
                resolution,
                origin,
            },
            data: rawData,
        };
    }
}

export const mapStreamService = new MapStreamService();
