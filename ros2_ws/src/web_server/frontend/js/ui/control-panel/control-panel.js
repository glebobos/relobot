import { rosService } from '../../services/ros-service.js';
import { gamepadService } from '../../services/gamepad-service.js';
import { BT_DIR, DOCK_ID, TOPICS, ACTIONS, SERVICES, CMDS, MSG_TYPES, EXPLORE_STATUS } from '../../shared/constants.js';
import { showConfirm, showAlert } from '../modal.js';

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
        this.resetMapBtn = document.getElementById('reset-map-btn');
        this.restartSlamBtn = document.getElementById('restart-slam-btn');
        this.rebootPiBtn = document.getElementById('reboot-pi-btn');
        this.poweroffPiBtn = document.getElementById('poweroff-pi-btn');
        this.dockBtn = document.getElementById('dock-btn');
        this.undockBtn = document.getElementById('undock-btn');
        this.knifeSlider = document.getElementById('knifeSlider');
        this.knifeSliderVal = document.getElementById('knifeSliderValue');
        this.cameraStopBtn = document.getElementById('camera-stop-btn');

        // State variables
        this.exploringActive = false;
        this.lastCoverageState = { state: 'idle', message: 'Coverage idle.' };
        this.dockingActive = false;
        this._toastTimer = null;

        // Topics & Action Clients
        this.exploreTopic = null;
        this.coverageCommandTopic = null;
        this.systemCommandTopic = null;
        this.dockAction = null;
        this.nav2Action = null;

        this.init();
    }

    init() {
        // Initialize ROS publishers/subscribers/action clients (using ESM v2 connection)
        this.exploreTopic = rosService.createTopicV2(TOPICS.EXPLORE_RESUME, MSG_TYPES.BOOL);
        this.coverageCommandTopic = rosService.createTopicV2(TOPICS.COVERAGE_COMMAND, MSG_TYPES.STRING);
        this.systemCommandTopic = rosService.createTopicV2(TOPICS.SYSTEM_COMMAND, MSG_TYPES.STRING);

        this.dockAction = rosService.createActionV2(ACTIONS.DOCK_ROBOT, MSG_TYPES.DOCK_ROBOT);
        this.nav2Action = rosService.createActionV2(ACTIONS.NAVIGATE_TO_POSE, MSG_TYPES.NAVIGATE_TO_POSE);

        // Subscribe to explore status
        this.exploreStatusTopic = rosService.createTopicV2(TOPICS.EXPLORE_STATUS, MSG_TYPES.EXPLORE_STATUS);
        this.exploreStatusCallback = (msg) => {
            const exploring = msg.status === EXPLORE_STATUS.STARTED || msg.status === EXPLORE_STATUS.IN_PROGRESS;
            this.updateExploreButton(exploring);
        };
        this.exploreStatusTopic.subscribe(this.exploreStatusCallback);

        // Subscribe to coverage status
        this.coverageStatusTopic = rosService.createTopicV2(TOPICS.COVERAGE_STATUS, MSG_TYPES.STRING);
        this.coverageStatusCallback = (msg) => {
            let payload;
            try {
                payload = JSON.parse(msg.data);
            } catch (_error) {
                payload = { state: 'error', message: msg.data || 'Invalid coverage status payload.' };
            }
            this.updateCoverageControls(payload);
        };
        this.coverageStatusTopic.subscribe(this.coverageStatusCallback);

        // Subscribe to dock action status
        this.dockStatusTopic = rosService.createTopicV2(TOPICS.DOCK_ACTION_STATUS, MSG_TYPES.GOAL_STATUS_ARRAY, { throttle_rate: 500 });
        this.dockStatusCallback = (msg) => {
            const active = (msg.status_list || []).some(
                (s) => s.status === 1 || s.status === 2 || s.status === 3,
            );
            this.setDockingState(active);
        };
        this.dockStatusTopic.subscribe(this.dockStatusCallback);

        // Register MapView callbacks
        if (this.mapView) {
            this.mapView.onPublishCommand = (cmd) => this.sendCoverageCommand(cmd);
            this.mapView.onNavigateToPoint = (x, y, yaw) => this.navigateToPoint(x, y, yaw);
            this.mapView.onZoneDrawModeChange = (active) => this.updateZoneDrawBtnState(active);
            this.mapView.onNavPointModeChange = (active) => this.updateNavPointBtnState(active);
        }

        // Knife Slider UI Integration
        if (this.knifeSlider) {
            this.knifeSlider.addEventListener('input', () => {
                const rpm = parseInt(this.knifeSlider.value, 10);
                gamepadService.setKnifeRpm(rpm, true);
                this.syncKnifeSliderVisuals(rpm);
                if (this.knifeSliderVal) this.knifeSliderVal.textContent = rpm;
            });
            this.knifeSlider.addEventListener('touchstart', (e) => e.stopPropagation(), { passive: false });
            this.knifeSlider.addEventListener('mousedown', (e) => e.stopPropagation());
            this.syncKnifeSliderVisuals(parseInt(this.knifeSlider.value, 10) || 0);
        }

        // Listen to gamepadService knife RPM updates (keeps UI slider in sync)
        gamepadService.onKnifeUpdateCallback = (rpm) => {
            const absRpm = Math.abs(rpm);
            if (this.knifeSlider) {
                this.knifeSlider.value = absRpm;
            }
            this.syncKnifeSliderVisuals(absRpm);
            if (this.knifeSliderVal) this.knifeSliderVal.textContent = Math.round(absRpm);
        };

        // Battery widget dock/undock dropdown toggle
        const headerBatteryWidget = document.getElementById('headerBatteryWidget');
        const batteryDockDropdown = document.getElementById('batteryDockDropdown');
        if (headerBatteryWidget && batteryDockDropdown) {
            // Toggle open/close on widget click — but not when a button inside is clicked
            headerBatteryWidget.addEventListener('click', (e) => {
                if (batteryDockDropdown.contains(e.target)) return;
                headerBatteryWidget.classList.toggle('is-dropdown-open');
            });

            // Prevent button clicks from bubbling up to the widget toggle
            batteryDockDropdown.addEventListener('click', (e) => {
                e.stopPropagation();
                // Close after a short delay so the user sees the button activate
                setTimeout(() => headerBatteryWidget.classList.remove('is-dropdown-open'), 300);
            });
        }

        // RPM monitoring widget inline-expand interaction
        const headerRpmWidget = document.getElementById('headerRpmWidget');
        const knivesSliderInline = document.getElementById('knivesSliderDropdown');
        const knifeSliderInput = document.getElementById('knifeSlider');
        if (headerRpmWidget && knivesSliderInline) {
            // Toggle open/close on click — EXCEPT when clicking the slider input itself
            headerRpmWidget.addEventListener('click', (e) => {
                if (knifeSliderInput && knifeSliderInput.contains(e.target)) return;
                headerRpmWidget.classList.toggle('is-dropdown-open');
            });

            // Prevent slider drag events from bubbling (don't interfere with touch/mouse drag)
            if (knifeSliderInput) {
                knifeSliderInput.addEventListener('touchstart', (e) => e.stopPropagation(), { passive: true });
                knifeSliderInput.addEventListener('mousedown', (e) => e.stopPropagation());
            }
        }

        // ── Unified outside-tap/click handler (closes ALL dropdowns) ──
        // Uses 'pointerdown' which fires reliably on both mouse AND touch
        // before any click event, so it works even when map/canvas elements
        // consume the subsequent click.
        const closeAllDropdowns = (e) => {
            if (headerBatteryWidget && !headerBatteryWidget.contains(e.target)) {
                headerBatteryWidget.classList.remove('is-dropdown-open');
            }
            if (headerRpmWidget && !headerRpmWidget.contains(e.target)) {
                headerRpmWidget.classList.remove('is-dropdown-open');
            }
        };
        document.addEventListener('pointerdown', closeAllDropdowns, { passive: true });

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
                this.sendCoverageCommand(CMDS.COVERAGE_PREVIEW);
            });
        }

        // Coverage Execute
        if (this.coverageExecuteBtn) {
            this.coverageExecuteBtn.addEventListener('click', () => {
                const isExecuting = this.coverageExecuteBtn.classList.contains('is-active');
                if (isExecuting) {
                    // Toggle stop
                    const stopBtn = document.getElementById('coverage-stop-btn');
                    if (stopBtn) stopBtn.click();
                } else {
                    this.updateCoverageControls({
                        state: 'executing',
                        message: 'Executing cached coverage path.',
                    });
                    this.sendCoverageCommand(CMDS.COVERAGE_EXECUTE);
                    this.coverageExecuteBtn.classList.add('is-active');
                }
            });
        }

        // Unified Emergency Stop
        const triggerEmergencyStop = () => {
            console.log('[StopBtn] Unified Emergency Stop engaged.');

            this.updateCoverageControls({
                state: 'cancel_requested',
                message: 'EMERGENCY STOP ENGAGED.',
            });

            // 1. Cancel active coverage task
            this.sendCoverageCommand(CMDS.COVERAGE_CANCEL);

            // 2. Halt drive motors
            gamepadService.publishTwist(0, 0);

            // 3. Stop knife blades
            gamepadService.setKnifeRpm(0, true);

            // 4. Invalidate navigation goal
            if (this.nav2Action) {
                try { this.nav2Action.cancelAllGoals(); } catch (e) { }
            }
            if (this.mapView && this.mapView.clearNavTarget) {
                this.mapView.clearNavTarget();
            }

            // 5. Cancel docking goals
            this.cancelDockGoal();

            // 6. Invalidate exploration
            if (this.exploreTopic) {
                this.exploreTopic.publish({ data: false });
                this.updateExploreButton(false);
            }

            // Remove active classes
            if (this.coverageExecuteBtn) this.coverageExecuteBtn.classList.remove('is-active');
            if (this.exploreBtn) this.exploreBtn.classList.remove('is-active');
            this.setDockingState(false);
        };

        if (this.coverageStopBtn) {
            this.coverageStopBtn.addEventListener('click', triggerEmergencyStop);
        }

        if (this.cameraStopBtn) {
            this.cameraStopBtn.addEventListener('click', (e) => {
                e.stopPropagation();
                triggerEmergencyStop();
            });
        }

        // Coverage Refresh Map
        if (this.coverageRefreshMapBtn) {
            this.coverageRefreshMapBtn.addEventListener('click', () => {
                this.updateCoverageControls({ state: 'planning', message: 'Re-extracting map boundary...' });
                this.sendCoverageCommand(CMDS.COVERAGE_REFRESH_MAP);
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
                    const saveMapClient = rosService.createServiceV2(SERVICES.SERIALIZE_MAP, MSG_TYPES.SERIALIZE_MAP_SRV);
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

        // Reset Map Button with custom confirmation modal
        if (this.resetMapBtn) {
            this.resetMapBtn.addEventListener('click', () => {
                showConfirm('Reset map? This will delete the saved map and restart SLAM in mapping mode.', () => {
                    this.sendCoverageCommand(CMDS.RESET_MAP);
                    showAlert('Resetting map and restarting SLAM...');
                });
            });
        }

        // Restart SLAM Button with custom confirmation modal
        if (this.restartSlamBtn) {
            this.restartSlamBtn.addEventListener('click', () => {
                showConfirm('Restart SLAM toolbox? This will reload the saved map if it exists.', () => {
                    this.sendCoverageCommand(CMDS.RESTART_SLAM);
                    showAlert('Restarting SLAM...');
                });
            });
        }

        // Reboot Pi Button with custom confirmation modal
        if (this.rebootPiBtn) {
            this.rebootPiBtn.addEventListener('click', () => {
                showConfirm('Are you sure you want to reboot the Relobot?', () => {
                    this.sendSystemCommand(CMDS.REBOOT);
                    showAlert('Rebooting Relobot...');
                });
            });
        }

        // Power Off Pi Button with custom confirmation modal
        if (this.poweroffPiBtn) {
            this.poweroffPiBtn.addEventListener('click', () => {
                showConfirm('Are you sure you want to power off the Relobot?', () => {
                    this.sendSystemCommand(CMDS.POWEROFF);
                    showAlert('Powering off Relobot...');
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

    sendSystemCommand(command) {
        if (!this.systemCommandTopic) return;
        this.systemCommandTopic.publish({ data: command });
    }

    updateExploreButton(exploring) {
        this.exploringActive = exploring;
        if (this.exploreBtn) {
            this.exploreBtn.classList.toggle('is-active', exploring);
        }
    }

    updateCoverageControls(status) {
        this.lastCoverageState = status;

        if (this.coverageStatus) {
            // Always show the toast when a new message arrives
            this.coverageStatus.classList.remove('is-toast-hidden');
            this.coverageStatus.textContent = status.message || 'Coverage status updated.';
            this.coverageStatus.dataset.state = status.state || 'idle';
        }

        const busy = ['planning', 'executing', 'cancel_requested'].includes(status.state);
        const isError = ['failed', 'error', 'rejected', 'server_unavailable'].includes(status.state);
        const previewReady = ['preview_ready', 'completed', 'executing'].includes(status.state);

        if (this.coveragePreviewBtn) this.coveragePreviewBtn.disabled = busy;
        if (this.coverageExecuteBtn) {
            this.coverageExecuteBtn.disabled = busy && status.state !== 'executing';
            this.coverageExecuteBtn.classList.toggle('is-active', status.state === 'executing');
        }
        if (this.coverageStopBtn) this.coverageStopBtn.disabled = false;
        if (this.coverageRefreshMapBtn) this.coverageRefreshMapBtn.disabled = busy;

        // Auto-dismiss toast for informational states; keep visible for
        // active operations (busy) and errors so the user doesn't miss them.
        clearTimeout(this._toastTimer);
        if (!busy && !isError && this.coverageStatus) {
            this._toastTimer = setTimeout(() => {
                this.coverageStatus.classList.add('is-toast-hidden');
            }, 3500);
        }
    }

    updateZoneDrawBtnState(active) {
        if (this.coverageDrawZoneBtn) {
            this.coverageDrawZoneBtn.classList.toggle('is-active', active);
        }
    }

    updateNavPointBtnState(active) {
        if (this.gotoPointBtn) {
            this.gotoPointBtn.classList.toggle('is-active', active);
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
                    position: { x: worldX, y: worldY, z: 0 },
                    orientation: { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) },
                },
            },
        };

        const id = this.nav2Action.sendGoal(
            goal,
            (result) => console.log('[NavPoint] reached:', result),
            (feedback) => console.log('[NavPoint] dist remaining:', feedback.distance_remaining),
            (error) => console.error('[NavPoint] failed:', error),
        );
        console.log('[NavPoint] goal sent to', worldX.toFixed(3), worldY.toFixed(3),
            'yaw=', ((yawRad || 0) * 180 / Math.PI).toFixed(1) + '°', 'id:', id);
    }

    setDockingState(active) {
        this.dockingActive = active;
        if (this.telemetry) {
            this.telemetry.setDockingActive(active);
        }
        if (this.dockBtn) {
            this.dockBtn.classList.toggle('is-active', active);
        }
        // Show pulsing indicator on the battery widget when docking is active
        const batteryWidget = document.getElementById('headerBatteryWidget');
        if (batteryWidget) {
            batteryWidget.classList.toggle('is-docking-active', active);
        }
    }

    sendDockGoal() {
        if (!this.dockAction) { console.error('[ControlPanel] dockAction not ready'); return; }
        const id = this.dockAction.sendGoal(
            { use_dock_id: true, dock_id: DOCK_ID, navigate_to_staging_pose: true, max_staging_time: 60.0 },
            (result) => {
                console.log('[Dock] result:', result);
                this.setDockingState(false);
            },
            (feedback) => console.log('[Dock] feedback:', feedback),
            (error) => {
                console.error('[Dock] failed:', error);
                this.setDockingState(false);
            },
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
        this.setDockingState(false);
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
            (feedback) => { },
            (error) => console.error('[Undock] failed:', error),
        );
        console.log('[Undock] goal sent, id:', id);
    }

    syncKnifeSliderVisuals(rpm) {
        const pct = (rpm / 3000) * 100;
        if (this.knifeSlider) {
            this.knifeSlider.style.background = `linear-gradient(to right, var(--color-green) 0%, var(--color-green) ${pct}%, rgba(255, 255, 255, 0.15) ${pct}%, rgba(255, 255, 255, 0.15) 100%)`;
        }
    }

    destroy() {
        if (this.exploreStatusTopic && this.exploreStatusCallback) {
            this.exploreStatusTopic.unsubscribe(this.exploreStatusCallback);
            this.exploreStatusTopic = null;
        }
        if (this.coverageStatusTopic && this.coverageStatusCallback) {
            this.coverageStatusTopic.unsubscribe(this.coverageStatusCallback);
            this.coverageStatusTopic = null;
        }
        if (this.dockStatusTopic && this.dockStatusCallback) {
            this.dockStatusTopic.unsubscribe(this.dockStatusCallback);
            this.dockStatusTopic = null;
        }
        if (this._toastTimer) {
            clearTimeout(this._toastTimer);
            this._toastTimer = null;
        }
    }
}
