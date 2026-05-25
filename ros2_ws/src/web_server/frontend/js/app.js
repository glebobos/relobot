import { CameraService } from './services/camera-service.js';
import { gamepadService } from './services/gamepad-service.js';
import { Telemetry } from './ui/telemetry.js';
import { MapView } from './ui/map-view.js';
import { ControlPanel } from './ui/control-panel.js';
import { VirtualJoystick } from './ui/virtual-joystick.js';

window.addEventListener('DOMContentLoaded', () => {
    // Initialize Camera MJPEG Stream
    const cameraService = new CameraService('cameraStream');

    // Initialize Virtual Joystick
    const virtualJoystick = new VirtualJoystick('.video-container');

    // Wait 1000ms for ROS bridge / ROS3D viewer libraries to fully load/stabilize
    setTimeout(() => {
        // Initialize Map View
        const mapView = new MapView('map');

        // Initialize Telemetry
        const telemetry = new Telemetry();

        // Initialize UI Control Panel (orchestrates mapView and telemetry)
        const controlPanel = new ControlPanel(mapView, telemetry);

        console.log('[App] Modular frontend initialized successfully.');
    }, 1000);
});
