export class CameraService {
    constructor(imgElementId) {
        this.cameraStream = document.getElementById(imgElementId);
        this.clientId = 'web-ui-' + Math.random().toString(36).substring(2, 9);
        this.baseUrl = `http://${window.location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg&client_id=${this.clientId}`;
        this.init();
    }

    init() {
        if (!this.cameraStream) return;
        this.connect();

        this.visibilityHandler = () => {
            if (document.visibilityState === 'visible') {
                console.log('[CameraService] Page visible, reconnecting camera stream...');
                this.connect();
            } else {
                console.log('[CameraService] Page hidden, disconnecting camera stream...');
                this.stop();
            }
        };

        this.beforeUnloadHandler = () => this.stop();

        document.addEventListener('visibilitychange', this.visibilityHandler);
        window.addEventListener('beforeunload', this.beforeUnloadHandler);
    }

    connect() {
        if (!this.cameraStream) return;
        console.log('[CameraService] Connecting to camera stream...');
        this.cameraStream.src = this.baseUrl + '&t=' + Date.now();
        this.cameraStream.onload = () => console.log('[CameraService] Camera stream connected');
        this.cameraStream.onerror = () => console.warn('[CameraService] Camera stream error');
    }

    stop() {
        if (!this.cameraStream) return;
        console.log('[CameraService] Stopping camera stream...');
        this.cameraStream.removeAttribute('src');
        fetch(`http://${window.location.hostname}:8080/shutdown?topic=/camera/image_raw&client_id=${this.clientId}`)
            .catch(err => console.debug('[CameraService] Stream shutdown error (expected if server offline)', err));
    }

    destroy() {
        document.removeEventListener('visibilitychange', this.visibilityHandler);
        window.removeEventListener('beforeunload', this.beforeUnloadHandler);
        this.stop();
    }
}
