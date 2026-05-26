import { rosService } from '../services/ros-service.js';
import { gamepadService } from '../services/gamepad-service.js';
import { BT_DIR, DOCK_ID } from '../config.js';
import { showConfirm, showAlert } from './modal.js';

export class ControlPanel {
    constructor(mapView, telemetry) {
        this.mapView = mapView;
        this.telemetry = telemetry;

        // UI elements
        this.exploreBtn = document.getElementById('explore-btn');
        this.coveragePreviewBtn = document.getElementById('coverage-preview-btn');
        this.coverageExecuteBtn = document.getElementById('coverage-execute-btn');
        this.coverageStopBtn = document.getElementById('coverage-stop-btn');
        this.coverageRefreshMapBtn = document.getElementById('coverage-refresh-map-btn');
        this.coverageDrawZoneBtn = document.getElementById('coverage-draw-zone-btn');
        this.coverageClearZoneBtn = document.getElementById('coverage-clear-zone-btn');
        this.gotoPointBtn = document.getElementById('goto-point-btn');
        this.coverageStatus = document.getElementById('coverage-status');
        this.saveMapBtn = document.getElementById('save-map-btn');
        this.dockBtn = document.getElementById('dock-btn');
        this.undockBtn = document.getElementById('undock-btn');
        this.knifeSlider = document.getElementById('knifeSlider');
        this.knifeSliderVal = document.getElementById('knifeSliderValue');

        // State variables
        this.exploringActive = false;
        this.lastCoverageState = { state: 'idle', message: 'Coverage idle.' };
        this.dockingActive = false;

        // Topics & Action Clients
        this.exploreTopic = null;
        this.coverageCommandTopic = null;
        this.dockAction = null;
        this.nav2Action = null;

        this.init();
    }

    init() {
        // Initialize ROS publishers/subscribers/action clients (using ESM v2 connection)
        this.exploreTopic = rosService.createTopicV2('/explore/resume', 'std_msgs/Bool');
        this.coverageCommandTopic = rosService.createTopicV2('/coverage/command', 'std_msgs/String');

        this.dockAction = rosService.createActionV2('/dock_robot', 'opennav_docking_msgs/action/DockRobot');
        this.nav2Action = rosService.createActionV2('/navigate_to_pose', 'nav2_msgs/action/NavigateToPose');

        // Subscribe to explore status
        const exploreStatusTopic = rosService.createTopicV2('/explore/status', 'explore_lite_msgs/ExploreStatus');
        exploreStatusTopic.subscribe((msg) => {
            const exploring = msg.status === 'exploration_started' || msg.status === 'exploration_in_progress';
            this.updateExploreButton(exploring);
        });

        // Subscribe to coverage status
        const coverageStatusTopic = rosService.createTopicV2('/coverage/status', 'std_msgs/String');
        coverageStatusTopic.subscribe((msg) => {
            let payload;
            try {
                payload = JSON.parse(msg.data);
            } catch (_error) {
                payload = { state: 'error', message: msg.data || 'Invalid coverage status payload.' };
            }
            this.updateCoverageControls(payload);
        });

        // Subscribe to dock action status
        const dockStatusTopic = rosService.createTopicV2('/dock_robot/_action/status', 'action_msgs/GoalStatusArray', { throttle_rate: 500 });
        dockStatusTopic.subscribe((msg) => {
            const active = (msg.status_list || []).some(
                (s) => s.status === 1 || s.status === 2 || s.status === 3,
            );
            this.setDockingState(active);
        });

        // Register MapView callbacks
        if (this.mapView) {
            this.mapView.onPublishCommand = (cmd) => this.sendCoverageCommand(cmd);
            this.mapView.onNavigateToPoint = (x, y, yaw) => this.navigateToPoint(x, y, yaw);
            this.mapView.onZoneDrawModeChange = (active) => this.updateZoneDrawBtnState(active);
            this.mapView.onNavPointModeChange = (active) => this.updateNavPointBtnState(active);
        }

        // Knife Slider UI Integration
        if (this.knifeSlider) {
            const syncVisuals = (val) => {
                const progress = document.getElementById('vSliderProgress');
                const thumb = document.getElementById('vSliderThumb');
                const pct = (val / 3000) * 100;
                if (progress) progress.style.height = `${pct}%`;
                if (thumb) thumb.style.bottom = `${pct}%`;
            };

            this.knifeSlider.addEventListener('input', () => {
                const rpm = parseInt(this.knifeSlider.value, 10);
                gamepadService.setKnifeRpm(rpm, true);
                syncVisuals(rpm);
                if (this.knifeSliderVal) this.knifeSliderVal.textContent = rpm;
            });
            this.knifeSlider.addEventListener('touchstart', (e) => e.stopPropagation(), { passive: false });
            this.knifeSlider.addEventListener('mousedown',  (e) => e.stopPropagation());
        }

        // Listen to gamepadService knife RPM updates (keeps UI slider in sync)
        gamepadService.onKnifeUpdateCallback = (rpm) => {
            const absRpm = Math.abs(rpm);
            if (this.knifeSlider) {
                this.knifeSlider.value = absRpm;
                const progress = document.getElementById('vSliderProgress');
                const thumb = document.getElementById('vSliderThumb');
                const pct = (absRpm / 3000) * 100;
                if (progress) progress.style.height = `${pct}%`;
                if (thumb) thumb.style.bottom = `${pct}%`;
            }
            if (this.knifeSliderVal) this.knifeSliderVal.textContent = Math.round(absRpm);
        };

        this.setupEventListeners();
        this.updateCoverageControls(this.lastCoverageState);
    }

    setupEventListeners() {
        // Explore Button
        if (this.exploreBtn) {
            this.exploreBtn.addEventListener('click', () => {
                if (!this.exploreTopic) return;
                const next = !this.exploringActive;
                this.exploreTopic.publish({ data: next });
                this.updateExploreButton(next);
            });
        }

        // Coverage Preview
        if (this.coveragePreviewBtn) {
            this.coveragePreviewBtn.addEventListener('click', () => {
                this.updateCoverageControls({
                    state: 'planning',
                    message: 'Computing coverage path.',
                });
                this.sendCoverageCommand('preview');
            });
        }

        // Coverage Execute
        if (this.coverageExecuteBtn) {
            this.coverageExecuteBtn.addEventListener('click', () => {
                const isExecuting = this.coverageExecuteBtn.classList.contains('active');
                if (isExecuting) {
                    // Toggle stop
                    const stopBtn = document.getElementById('coverage-stop-btn');
                    if (stopBtn) stopBtn.click();
                } else {
                    this.updateCoverageControls({
                        state: 'executing',
                        message: 'Executing cached coverage path.',
                    });
                    this.sendCoverageCommand('execute');
                    this.coverageExecuteBtn.classList.add('active');
                }
            });
        }

        // Coverage Stop -> Unified Emergency Stop
        if (this.coverageStopBtn) {
            this.coverageStopBtn.addEventListener('click', () => {
                console.log('[StopBtn] Unified Emergency Stop engaged.');
                
                this.updateCoverageControls({
                    state: 'cancel_requested',
                    message: 'EMERGENCY STOP ENGAGED.',
                });
                
                // 1. Cancel active coverage task
                this.sendCoverageCommand('cancel');

                // 2. Halt drive motors
                gamepadService.publishTwist(0, 0);

                // 3. Stop knife blades
                gamepadService.setKnifeRpm(0, true);

                // 4. Invalidate navigation goal
                if (this.nav2Action) {
                    try { this.nav2Action.cancelAllGoals(); } catch(e){}
                }

                // 5. Cancel docking goals
                this.cancelDockGoal();

                // 6. Invalidate exploration
                if (this.exploreTopic) {
                    this.exploreTopic.publish({ data: false });
                    this.updateExploreButton(false);
                }

                // Remove active classes
                if (this.coverageExecuteBtn) this.coverageExecuteBtn.classList.remove('active');
                if (this.exploreBtn) this.exploreBtn.classList.remove('active');
                this.setDockingState(false);
            });
        }

        // Coverage Refresh Map
        if (this.coverageRefreshMapBtn) {
            this.coverageRefreshMapBtn.addEventListener('click', () => {
                this.updateCoverageControls({ state: 'planning', message: 'Re-extracting map boundary...' });
                this.sendCoverageCommand('refresh_map');
            });
        }

        // Draw Zone Button
        if (this.coverageDrawZoneBtn) {
            this.coverageDrawZoneBtn.addEventListener('click', () => {
                if (!this.mapView) return;
                this.mapView.startZoneDraw();
            });
        }

        // Clear Zone Button
        if (this.coverageClearZoneBtn) {
            this.coverageClearZoneBtn.addEventListener('click', () => {
                if (!this.mapView) return;
                this.mapView.clearZone();
                this.updateCoverageControls({ state: 'idle', message: 'Zone cleared. SLAM boundary active.' });
            });
        }

        // Go to Point Button
        if (this.gotoPointBtn) {
            this.gotoPointBtn.addEventListener('click', () => {
                if (!this.mapView) return;
                this.mapView.startNavPoint();
            });
        }

        // Save Map Button with custom confirmation modal
        if (this.saveMapBtn) {
            this.saveMapBtn.addEventListener('click', () => {
                showConfirm('Save current map? This will overwrite the previous save.', () => {
                    const saveMapClient = rosService.createServiceV2('/slam_toolbox/serialize_map', 'slam_toolbox/srv/SerializePoseGraph');
                    if (saveMapClient) {
                        saveMapClient.callService(
                            { filename: 'map_serialized' },
                            (result) => { 
                                console.log('[SaveMap] result:', result); 
                                showAlert('Map saved!'); 
                            },
                            (error) => { 
                                console.error('[SaveMap] error:', error); 
                                showAlert('Error saving map: ' + error); 
                            },
                        );
                    } else {
                        console.error('[SaveMap] serialize_map service not ready');
                        showAlert('Error: serialize_map service unavailable.');
                    }
                });
            });
        }

        // Dock Button
        if (this.dockBtn) {
            this.dockBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                if (this.dockingActive) {
                    this.cancelDockGoal();
                } else {
                    this.sendDockGoal();
                }
            });
        }

        // Undock Button
        if (this.undockBtn) {
            this.undockBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                this.sendUndockGoal();
            });
        }
    }

    sendCoverageCommand(command) {
        if (!this.coverageCommandTopic) return;
        this.coverageCommandTopic.publish({ data: command });
    }

    updateExploreButton(exploring) {
        this.exploringActive = exploring;
        if (this.exploreBtn) {
            this.exploreBtn.classList.toggle('active', exploring);
        }
    }

    updateCoverageControls(status) {
        this.lastCoverageState = status;

        if (this.coverageStatus) {
            this.coverageStatus.textContent = status.message || 'Coverage status updated.';
            this.coverageStatus.dataset.state = status.state || 'idle';
        }

        const busy = ['planning', 'executing', 'cancel_requested'].includes(status.state);
        const previewReady = ['preview_ready', 'completed', 'executing'].includes(status.state);

        if (this.coveragePreviewBtn) this.coveragePreviewBtn.disabled = busy;
        if (this.coverageExecuteBtn) {
            this.coverageExecuteBtn.disabled = busy && status.state !== 'executing';
            this.exploreBtn.classList.toggle('active', status.state === 'executing');
        }
        if (this.coverageStopBtn) this.coverageStopBtn.disabled = !busy;
        if (this.coverageRefreshMapBtn) this.coverageRefreshMapBtn.disabled = busy;
    }

    updateZoneDrawBtnState(active) {
        if (this.coverageDrawZoneBtn) {
            this.coverageDrawZoneBtn.classList.toggle('active', active);
        }
    }

    updateNavPointBtnState(active) {
        if (this.gotoPointBtn) {
            this.gotoPointBtn.classList.toggle('active', active);
        }
    }

    navigateToPoint(worldX, worldY, yawRad) {
        if (!this.nav2Action) { console.error('[ControlPanel] nav2Action not ready'); return; }
        
        // Convert heading angle → quaternion (rotation around Z axis)
        const halfYaw = (yawRad || 0) / 2;
        const goal = {
            pose: {
                header: { frame_id: 'map' },
                pose: {
                    position:    { x: worldX, y: worldY, z: 0 },
                    orientation: { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) },
                },
            },
        };

        const id = this.nav2Action.sendGoal(
            goal,
            (result)   => console.log('[NavPoint] reached:', result),
            (feedback) => console.log('[NavPoint] dist remaining:', feedback.distance_remaining),
            (error)    => console.error('[NavPoint] failed:', error),
        );
        console.log('[NavPoint] goal sent to', worldX.toFixed(3), worldY.toFixed(3),
            'yaw=', ((yawRad||0) * 180 / Math.PI).toFixed(1) + '°', 'id:', id);
    }

    setDockingState(active) {
        this.dockingActive = active;
        if (this.telemetry) {
            this.telemetry.setDockingActive(active);
        }
        if (this.dockBtn) {
            this.dockBtn.classList.toggle('active', active);
        }
    }

    sendDockGoal() {
        if (!this.dockAction) { console.error('[ControlPanel] dockAction not ready'); return; }
        const id = this.dockAction.sendGoal(
            { use_dock_id: true, dock_id: DOCK_ID, navigate_to_staging_pose: true, max_staging_time: 60.0 },
            (result) => console.log('[Dock] result:', result),
            (feedback) => console.log('[Dock] feedback:', feedback),
            (error) => console.error('[Dock] failed:', error),
        );
        console.log('[Dock] goal sent, id:', id);
    }

    cancelDockGoal() {
        if (!this.dockingActive) {
            console.warn('[Dock] No active docking goal to cancel');
            return;
        }
        if (!this.dockAction) { console.error('[ControlPanel] dockAction not ready'); return; }
        console.log('[Dock] Cancelling all dock goals via cancel service');
        try {
            this.dockAction.cancelAllGoals();
        } catch (err) {
            console.warn('[Dock] cancelAllGoals error:', err);
        }
    }

    sendUndockGoal() {
        if (!this.nav2Action) { console.error('[ControlPanel] nav2Action not ready'); return; }
        const id = this.nav2Action.sendGoal(
            {
                behavior_tree: `${BT_DIR}/undock_and_turn.xml`,
                pose: {
                    header: { frame_id: 'map' },
                    pose: {
                        position: { x: 0, y: 0, z: 0 },
                        orientation: { x: 0, y: 0, z: 0, w: 1 },
                    },
                },
            },
            (result) => console.log('[Undock] result:', result),
            (feedback) => console.log('[Undock] feedback:', feedback),
            (error) => console.error('[Undock] failed:', error),
        );
        console.log('[Undock] goal sent, id:', id);
    }
}
