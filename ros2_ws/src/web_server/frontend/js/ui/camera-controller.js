import { CameraService } from '../services/camera-service.js';

export class CameraController {
    constructor() {
        this.mainCameraService = new CameraService('cameraStream');
        this.pipCameraService = new CameraService('pipCameraStream');
    }

    init() {
        // Stop the main camera stream initially (Screen 0 is active by default)
        this.mainCameraService.stop();

        // Add fallback for main stream
        const mainCam = document.getElementById('cameraStream');
        if (mainCam) {
            const originalError = mainCam.onerror;
            mainCam.onerror = () => {
                if (originalError) originalError();
                if (mainCam.src && mainCam.src.includes('/stream?topic=')) {
                    console.log('[CameraController] Camera offline, loading premium garden view');
                    mainCam.src = 'https://images.unsplash.com/photo-1558905619-798c06296728?auto=format&fit=crop&w=800&q=80';
                }
            };
        }

        // Handle PIP Camera Image Fallback
        const pipCamStream = document.getElementById('pipCameraStream');
        if (pipCamStream) {
            const originalError = pipCamStream.onerror;
            pipCamStream.onerror = () => {
                if (originalError) originalError();
                if (pipCamStream.src && pipCamStream.src.includes('/stream?topic=')) {
                    console.log('[CameraController] PIP Camera offline, loading mini garden view');
                    pipCamStream.src = 'https://images.unsplash.com/photo-1558905619-798c06296728?auto=format&fit=crop&w=300&q=80';
                }
            };
        }
    }
}
