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
            mainCam.addEventListener('error', () => {
                if (this.mainCameraService.isActive && mainCam.src && mainCam.src.includes('/stream?topic=')) {
                    console.log('[CameraController] Camera offline, loading local placeholder');
                    mainCam.src = '/grass_texture.png';
                }
            });
        }

        // Handle PIP Camera Image Fallback
        const pipCamStream = document.getElementById('pipCameraStream');
        if (pipCamStream) {
            pipCamStream.addEventListener('error', () => {
                if (this.pipCameraService.isActive && pipCamStream.src && pipCamStream.src.includes('/stream?topic=')) {
                    console.log('[CameraController] PIP Camera offline, loading local placeholder');
                    pipCamStream.src = '/grass_texture.png';
                }
            });
        }
    }
}
