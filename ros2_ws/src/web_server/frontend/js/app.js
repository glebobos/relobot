import { CameraService } from './services/camera-service.js';
import { gamepadService } from './services/gamepad-service.js';
import { Telemetry } from './ui/telemetry.js';
import { MapView } from './ui/map-view.js';
import { ControlPanel } from './ui/control-panel.js';
import { VirtualJoystick } from './ui/virtual-joystick.js';
import { SettingsPanel } from './ui/settings-panel.js';
import { rosService } from './services/ros-service.js';

window.addEventListener('DOMContentLoaded', () => {
    // 1. Initialize Clock
    const statusTime = document.getElementById('statusTime');
    if (statusTime) {
        const updateClock = () => {
            const now = new Date();
            const hrs = String(now.getHours()).padStart(2, '0');
            const mins = String(now.getMinutes()).padStart(2, '0');
            statusTime.textContent = `${hrs}:${mins}`;
        };
        updateClock();
        setInterval(updateClock, 1000);
    }

    // 2. Initialize Camera Services (with Unsplash garden fallback)
    const mainCameraService = new CameraService('cameraStream');
    const pipCameraService = new CameraService('pipCameraStream');
    
    // Initially, Screen 0 (Map View) is active, so stop the main camera stream to save resources.
    mainCameraService.stop();
    
    // Add fallback for main stream
    const mainCam = document.getElementById('cameraStream');
    if (mainCam) {
        const originalError = mainCam.onerror;
        mainCam.onerror = () => {
            if (originalError) originalError();
            if (mainCam.src && mainCam.src.includes('/stream?topic=')) {
                console.log('[App] Camera offline, loading premium garden view');
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
                console.log('[App] PIP Camera offline, loading mini garden view');
                pipCamStream.src = 'https://images.unsplash.com/photo-1558905619-798c06296728?auto=format&fit=crop&w=300&q=80';
            }
        };
    }

    // 3. Initialize Virtual Joystick
    // Using the camera feed wrapper as interactive gesture zone
    const virtualJoystick = new VirtualJoystick('#largeCameraWrapper');

    // 4. Pagination / Top Header Navigation Dots logic
    const screensWrapper = document.getElementById('screensWrapper');
    const tabs = document.querySelectorAll('#headerNav .dot');
    let currentScreen = 0;

    const navigateToScreen = (index) => {
        if (index < 0 || index > 2) return;
        currentScreen = index;
        if (screensWrapper) {
            screensWrapper.style.transform = `translateX(-${index * 33.3333}%)`;
        }
        tabs.forEach((tab, idx) => {
            tab.classList.toggle('active', idx === index);
        });

        // Optimize camera streams based on active screen
        if (index === 0) {
            pipCameraService.connect();
            mainCameraService.stop();
        } else if (index === 1) {
            mainCameraService.connect();
            pipCameraService.stop();
        } else {
            mainCameraService.stop();
            pipCameraService.stop();
        }
    };

    tabs.forEach((tab) => {
        tab.addEventListener('click', () => {
            const index = parseInt(tab.getAttribute('data-index'), 10);
            navigateToScreen(index);
        });
    });

    // 5. Picture-in-Picture (PIP) Click Navigation
    const pipCamera = document.getElementById('pipCamera');
    const pipMap = document.getElementById('pipMap');

    if (pipCamera) {
        pipCamera.addEventListener('click', () => navigateToScreen(1));
    }
    if (pipMap) {
        pipMap.addEventListener('click', () => navigateToScreen(0));
    }

    // 6. Wait 1000ms for ROS/ThreeJS libraries to stabilize, then launch ROS UI components
    setTimeout(() => {
        const mapView = new MapView('map');
        const telemetry = new Telemetry();
        const controlPanel = new ControlPanel(mapView, telemetry);
        const settingsPanel = new SettingsPanel(telemetry);

        // Coordinate double Stop Button mapping (both Screens)
        const cameraStopBtn = document.getElementById('camera-stop-btn');
        if (cameraStopBtn) {
            cameraStopBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                // Trigger the main stop logic in controlPanel
                const stopBtn = document.getElementById('coverage-stop-btn');
                if (stopBtn) {
                    stopBtn.click();
                } else {
                    // Fallback direct calls if controlPanel button is missing
                    console.log('[StopButton] Triggering safety halt direct');
                    gamepadService.publishTwist(0, 0);
                    gamepadService.setKnifeRpm(0, true);
                }
            });
        }

        // Setup dynamic demo simulation if WebSocket remains disconnected after 3 seconds
        setTimeout(() => {
            const isConnected = rosService._connectedV1 || rosService._connectedV2;
            if (!isConnected) {
                console.log('[App] Connection to ROS bridge is offline. Activating Demo/Mock Mode.');
                startDemoMode(telemetry, settingsPanel);
            }
        }, 2000);

        console.log('[App] Modular UI refactoring initialized.');
    }, 1000);
});

/**
 * Simulates robot activity (telemetry, movements) when ROS is offline.
 */
function startDemoMode(telemetry, settingsPanel) {
    let mockVoltage = 25.4;
    let mockRpm = 0;
    let mockSpeed = 0.0;
    let mockDocked = true;
    let dockStatusText = 'On Dock';

    const vinDisplay = document.getElementById('vin-display');
    const vinDisplayCam = document.getElementById('vin-display-cam');
    const chargerDisplay = document.getElementById('charger-voltage-display');
    const rpmDisplay = document.getElementById('rpm-display');
    const rpmDisplayCam = document.getElementById('rpm-display-cam');
    const speedDisplay = document.getElementById('speed-display');
    const latencyDisplay = document.getElementById('latency-display');
    const batteryPctSpan = document.getElementById('status-battery-pct');
    const batteryIcon = document.getElementById('status-battery-icon');

    // Enable Settings diagnostics simulation
    if (settingsPanel) {
        settingsPanel.addMockLog('[DemoMode] WebSocket offline. Simulation active.');
        settingsPanel.addMockLog('[DemoMode] Simulation loaded successfully.');
    }

    // Telemetry Simulation loop
    setInterval(() => {
        // 1. Sim battery voltage percentage mapping (22V - 26.2V)
        const pctVal = Math.round(((mockVoltage - 22.0) / (26.2 - 22.0)) * 100);
        const clampedPct = Math.max(0, Math.min(100, pctVal));
        const formattedVolts = mockVoltage.toFixed(1) + ' V';

        if (vinDisplay) vinDisplay.textContent = formattedVolts;
        if (vinDisplayCam) vinDisplayCam.textContent = formattedVolts;
        if (batteryPctSpan) batteryPctSpan.textContent = formattedVolts;

        // Update battery icon
        if (batteryIcon) {
            batteryIcon.className = 'fas';
            if (clampedPct > 80) batteryIcon.classList.add('fa-battery-full');
            else if (clampedPct > 55) batteryIcon.classList.add('fa-battery-three-quarters');
            else if (clampedPct > 35) batteryIcon.classList.add('fa-battery-half');
            else if (clampedPct > 15) batteryIcon.classList.add('fa-battery-quarter');
            else batteryIcon.classList.add('fa-battery-empty');
        }

        // 2. Sim Dock Charging Status
        const btnMow = document.getElementById('coverage-execute-btn');
        const isMowing = btnMow && btnMow.classList.contains('active');
        
        // Sim docking state
        const btnDock = document.getElementById('dock-btn');
        const isDocking = btnDock && btnDock.classList.contains('dock-btn-active');

        if (isMowing || isDocking) {
            mockDocked = false;
        }

        const headerChargingIcon = document.getElementById('header-charging-icon');
        if (mockDocked) {
            dockStatusText = 'Charging';
            const mockChargerVolts = (mockVoltage + 1.2).toFixed(1) + ' V';
            if (chargerDisplay) {
                chargerDisplay.textContent = mockChargerVolts;
                chargerDisplay.style.display = 'inline-block';
            }
            if (headerChargingIcon) {
                headerChargingIcon.style.display = 'inline-block';
            }
            // Slowly recharge battery in dock
            if (mockVoltage < 26.2) mockVoltage += 0.01;
        } else {
            dockStatusText = 'Discharging';
            if (chargerDisplay) {
                chargerDisplay.style.display = 'none';
            }
            if (headerChargingIcon) {
                headerChargingIcon.style.display = 'none';
            }
            // Slowly discharge battery when working
            if (mockVoltage > 22.5) mockVoltage -= 0.005;
        }

        // 3. Sim Blade RPM
        const knifeSlider = document.getElementById('knifeSlider');
        let targetRpm = 0;
        if (knifeSlider) {
            targetRpm = parseInt(knifeSlider.value, 10);
        }
        if (isMowing && targetRpm === 0) {
            targetRpm = 2800; // Auto mowing default RPM
        }

        // Smooth RPM transition
        if (mockRpm < targetRpm) mockRpm += 150;
        if (mockRpm > targetRpm) mockRpm -= 200;
        mockRpm = Math.max(0, Math.min(3000, mockRpm));

        if (rpmDisplay) rpmDisplay.textContent = Math.round(mockRpm);
        if (rpmDisplayCam) rpmDisplayCam.textContent = Math.round(mockRpm);

        // Spin the fan/blade icon based on RPM
        const rpmIconHeader = document.getElementById('rpm-icon-header');
        const rpmIconCam = document.getElementById('rpm-icon-cam');
        const vSliderFanIcon = document.getElementById('vSliderFanIcon');
        [rpmIconHeader, rpmIconCam, vSliderFanIcon].forEach(icon => {
            if (icon) {
                const img = icon.querySelector('i') || icon;
                if (mockRpm > 100) {
                    img.style.animationPlayState = 'running';
                    const speed = 3000 / mockRpm;
                    img.style.animationDuration = `${speed.toFixed(2)}s`;
                } else {
                    img.style.animationPlayState = 'paused';
                }
            }
        });

        // 4. Sim Speed and Latency
        if (targetRpm > 0 || isMowing) {
            mockSpeed = 0.8 + Math.random() * 0.4;
        } else {
            mockSpeed = 0.0;
        }
        if (speedDisplay) speedDisplay.textContent = mockSpeed.toFixed(1);

        // Latency simulation (fluctuates between 20-30ms)
        const mockLatency = 20 + Math.round(Math.random() * 10);
        if (latencyDisplay) latencyDisplay.textContent = `${mockLatency}ms`;

    }, 1000);
}
