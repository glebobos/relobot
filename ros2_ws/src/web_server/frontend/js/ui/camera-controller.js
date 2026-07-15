import { CameraService } from '../services/camera-service.js';
import { rosService } from '../services/ros-service.js';
import { SERVICES, MSG_TYPES } from '../shared/constants.js';

export class CameraController {
    constructor() {
        this.mainCameraService = new CameraService('cameraStream');
        this.pipCameraService = new CameraService('pipCameraStream');
        this.tfTopic = null;
        this.hudTimeout = null;
        this.currentScreenIndex = 0;
        this._lastSyncedAprilTagState = null;
    }

    init() {
        // Stop the main camera stream initially (Screen 0 is active by default)
        this.mainCameraService.stop();

        // Listen for screen changes
        this.screenChangedHandler = (e) => {
            this.currentScreenIndex = e.detail.index;
            this.updateCalibrationViews(this.currentScreenIndex);
        };
        window.addEventListener('screenChanged', this.screenChangedHandler);

        // Listen for settings changes
        this.calibrationSettingsHandler = () => {
            this.updateCalibrationViews(this.currentScreenIndex);
        };
        window.addEventListener('calibrationSettingsChanged', this.calibrationSettingsHandler);

        // Sync ROS parameter when ROS connects
        this.rosConnectionHandler = () => {
            const apriltagEnabled = localStorage.getItem('calibration_apriltag_enabled') === 'true';
            this.syncAprilTagParameter(apriltagEnabled);
        };
        rosService.rosV2.on('connection', this.rosConnectionHandler);

        // Initial setup
        this.updateCalibrationViews(this.currentScreenIndex);
    }

    updateCalibrationViews(activeScreenIndex) {
        const crosshairEnabled = localStorage.getItem('calibration_crosshair_enabled') === 'true';
        const apriltagEnabled = localStorage.getItem('calibration_apriltag_enabled') === 'true';

        // 1. Crosshair visibility (only show when on Camera screen and enabled)
        const crosshair = document.querySelector('.c-camera-view__crosshair');
        if (crosshair) {
            crosshair.style.display = (crosshairEnabled && activeScreenIndex === 1) ? 'block' : 'none';
        }

        // 2. AprilTag HUD overlay visibility and subscription
        const hud = document.getElementById('cameraHud');
        
        // We only subscribe if on camera screen and apriltag is enabled
        const shouldSubscribe = apriltagEnabled && activeScreenIndex === 1;

        if (shouldSubscribe) {
            if (!this.tfTopic) {
                this.initAprilTagListener();
            }
        } else {
            // Unsubscribe and cleanup
            if (this.tfTopic) {
                try {
                    this.tfTopic.unsubscribe();
                } catch (e) {
                    console.warn('[CameraController] Failed to unsubscribe from TF:', e);
                }
                this.tfTopic = null;
            }
            if (this.hudTimeout) {
                clearTimeout(this.hudTimeout);
                this.hudTimeout = null;
            }
            if (hud) {
                hud.style.display = 'none';
            }
        }

        // Sync ROS parameter for AprilTag if state changed
        if (this._lastSyncedAprilTagState !== apriltagEnabled) {
            this._lastSyncedAprilTagState = apriltagEnabled;
            this.syncAprilTagParameter(apriltagEnabled);
        }
    }

    syncAprilTagParameter(enabled) {
        console.log('[CameraController] Syncing AprilTag parameter always_on =', enabled);
        const setParamsClient = rosService.createServiceV2(
            SERVICES.SET_APRILTAG_PARAMETERS,
            MSG_TYPES.SET_PARAMETERS
        );
        if (setParamsClient) {
            setParamsClient.callService(
                {
                    parameters: [
                        {
                            name: 'always_on',
                            value: {
                                type: 1, // PARAMETER_BOOL
                                bool_value: enabled
                            }
                        }
                    ]
                },
                (result) => {
                    console.log('[CameraController] AprilTag always_on set successful:', result);
                },
                (error) => {
                    console.error('[CameraController] Failed to set AprilTag always_on:', error);
                }
            );
        } else {
            console.warn('[CameraController] set_parameters service client not ready');
        }
    }

    initAprilTagListener() {
        // Create `/tf` topic listener using rosService
        const hud = document.getElementById('cameraHud');
        const elX = document.getElementById('hud-x');
        const elY = document.getElementById('hud-y');
        const elZ = document.getElementById('hud-z');
        const elYaw = document.getElementById('hud-yaw');

        this.tfTopic = rosService.subscribeV2('/tf', 'tf2_msgs/msg/TFMessage', (message) => {
            if (!message.transforms || message.transforms.length === 0) return;
            
            // Look for a transform starting with tag25h9: or tag36h11: etc
            const tagTransform = message.transforms.find(t => 
                t.child_frame_id.startsWith('tag25h9:') || t.child_frame_id.startsWith('tag36h11:')
            );

            if (tagTransform) {
                const trans = tagTransform.transform.translation;
                const rot = tagTransform.transform.rotation;

                // Show the HUD panel
                if (hud) hud.style.display = 'block';

                // Display distances with 3-decimal precision (meters)
                if (elX) elX.innerText = parseFloat(trans.x).toFixed(3);
                if (elY) elY.innerText = parseFloat(trans.y).toFixed(3);
                if (elZ) elZ.innerText = parseFloat(trans.z).toFixed(3);

                // Compute yaw around camera's local vertical axis (Y-axis pointing down in optical frames)
                const siny = 2 * (rot.w * rot.y - rot.z * rot.x);
                const cosy = 1 - 2 * (rot.x * rot.x + rot.y * rot.y);
                const yawRad = Math.atan2(siny, cosy);
                const yawDeg = (yawRad * 180.0 / Math.PI).toFixed(1);

                if (elYaw) elYaw.innerText = yawDeg;

                // Auto-hide HUD if tag disappears (1.5 seconds threshold)
                if (this.hudTimeout) clearTimeout(this.hudTimeout);
                this.hudTimeout = setTimeout(() => {
                    if (hud) hud.style.display = 'none';
                }, 1500);
            }
        }, { throttle_rate: 100 });
    }

    destroy() {
        if (this.tfTopic) {
            this.tfTopic.unsubscribe();
            this.tfTopic = null;
        }
        if (this.hudTimeout) {
            clearTimeout(this.hudTimeout);
            this.hudTimeout = null;
        }
        if (this.screenChangedHandler) {
            window.removeEventListener('screenChanged', this.screenChangedHandler);
        }
        if (this.calibrationSettingsHandler) {
            window.removeEventListener('calibrationSettingsChanged', this.calibrationSettingsHandler);
        }
        if (this.rosConnectionHandler && typeof rosService.rosV2.off === 'function') {
            rosService.rosV2.off('connection', this.rosConnectionHandler);
        }
        this.mainCameraService.destroy();
        this.pipCameraService.destroy();
    }
}
