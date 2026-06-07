import { Clock } from './ui/clock.js';
import { CameraController } from './ui/camera-controller.js';
import { NavigationManager } from './ui/navigation-manager.js';
import { Telemetry } from './ui/telemetry.js';
import { MapView } from './ui/map-view.js';
import { ControlPanel } from './ui/control-panel.js';
import { VirtualJoystick } from './ui/virtual-joystick.js';
import { SettingsPanel } from './ui/settings-panel.js';

window.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize Clock
    const clock = new Clock('statusTime');
    clock.start();

    // 2. Initialize Camera Services and Controller
    const cameraController = new CameraController();
    cameraController.init();

    // 3. Initialize Virtual Joystick using the camera feed wrapper as interactive gesture zone
    const virtualJoystick = new VirtualJoystick('#largeCameraWrapper');

    // 4. Initialize Navigation Manager / Pagination Page Slider
    const navigationManager = new NavigationManager(
        'screensWrapper',
        '#headerNav .dot',
        cameraController.mainCameraService,
        cameraController.pipCameraService
    );
    navigationManager.init();

    // 5. Wait 1000ms for ROS/ThreeJS libraries to stabilize, then launch ROS UI components
    setTimeout(() => {
        const mapView = new MapView('map');
        const telemetry = new Telemetry();
        const controlPanel = new ControlPanel(mapView, telemetry);
        const settingsPanel = new SettingsPanel(telemetry);
    }, 1000);
});
