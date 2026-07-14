import * as THREE from 'three';
import * as ROSLIB from 'roslib';
import { CameraController } from './ui/camera-controller.js';
import { NavigationManager } from './ui/navigation-manager.js';
import { Telemetry } from './ui/telemetry/telemetry.js';
import { MapView } from './ui/map/map-view.js';
import { ControlPanel } from './ui/control-panel/control-panel.js';
import { VirtualJoystick } from './ui/virtual-joystick.js';
import { SettingsPanel } from './ui/settings/settings-panel.js';

window.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize Camera Services and Controller
    const cameraController = new CameraController();
    cameraController.init();

    // 2. Initialize Virtual Joystick using the camera feed wrapper as interactive gesture zone
    const virtualJoystick = new VirtualJoystick('#largeCameraWrapper');

    // 3. Initialize Navigation Manager / Pagination Page Slider
    const navigationManager = new NavigationManager(
        'screensWrapper',
        '#headerNav .c-dots-nav__dot',
        cameraController.mainCameraService,
        cameraController.pipCameraService
    );
    navigationManager.init();

    // 4. Initialize interface components immediately
    console.log('[App] ROS/ThreeJS libraries loaded, initializing interface components...');
    const mapView = new MapView('map');
    const telemetry = new Telemetry();
    const controlPanel = new ControlPanel(mapView, telemetry);
    const settingsPanel = new SettingsPanel(telemetry);
});
