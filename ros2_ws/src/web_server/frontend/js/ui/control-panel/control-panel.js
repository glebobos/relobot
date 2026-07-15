import { rosService } from '../../services/ros-service.js';
import { gamepadService } from '../../services/gamepad-service.js';
import { TOPICS, CMDS, MSG_TYPES, EXPLORE_STATUS } from '../../shared/constants.js';
import { DomListenerBag } from '../../shared/dom-listener-bag.js';
import { HeaderWidgetsController } from './header-widgets-controller.js';
import { SystemOperationsController } from './system-operations-controller.js';
import { RobotActionsController } from './robot-actions-controller.js';

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
        this.cameraStopBtn = document.getElementById('camera-stop-btn');
        this.listeners = new DomListenerBag();

        // State variables
        this.exploringActive = false;
        this.lastCoverageState = { state: 'idle', message: 'Coverage idle.' };
        this._toastTimer = null;

        // Topics & Action Clients
        this.exploreTopic = null;
        this.coverageCommandTopic = null;
        this.systemCommandTopic = null;

        this.init();
    }

    init() {
        // Initialize ROS publishers/subscribers/action clients (using ESM v2 connection)
        this.exploreTopic = rosService.createTopicV2(TOPICS.EXPLORE_RESUME, MSG_TYPES.BOOL);
        this.coverageCommandTopic = rosService.createTopicV2(TOPICS.COVERAGE_COMMAND, MSG_TYPES.STRING);
        this.systemCommandTopic = rosService.createTopicV2(TOPICS.SYSTEM_COMMAND, MSG_TYPES.STRING);

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

        this.robotActions = new RobotActionsController(this.telemetry);

        // Register MapView callbacks
        if (this.mapView) {
            this.mapView.onPublishCommand = (cmd) => this.sendCoverageCommand(cmd);
            this.mapView.onNavigateToPoint = (x, y, yaw) => this.robotActions.navigateToPoint(x, y, yaw);
            this.mapView.onZoneDrawModeChange = (active) => this.updateZoneDrawBtnState(active);
            this.mapView.onNavPointModeChange = (active) => this.updateNavPointBtnState(active);
        }

        this.headerWidgets = new HeaderWidgetsController();
        this.systemOperations = new SystemOperationsController(
            command => this.sendCoverageCommand(command),
            command => this.sendSystemCommand(command),
        );
        this.setupEventListeners();
        this.updateCoverageControls(this.lastCoverageState);
    }

    setupEventListeners() {
        // Explore Button
        if (this.exploreBtn) {
            this.listeners.add(this.exploreBtn, 'click', () => {
                if (!this.exploreTopic) return;
                const next = !this.exploringActive;
                this.exploreTopic.publish({ data: next });
                this.updateExploreButton(next);
            });
        }

        // Coverage Preview
        if (this.coveragePreviewBtn) {
            this.listeners.add(this.coveragePreviewBtn, 'click', () => {
                this.updateCoverageControls({
                    state: 'planning',
                    message: 'Computing coverage path.',
                });
                this.sendCoverageCommand(CMDS.COVERAGE_PREVIEW);
            });
        }

        // Coverage Execute
        if (this.coverageExecuteBtn) {
            this.listeners.add(this.coverageExecuteBtn, 'click', () => {
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
            this.robotActions.cancelNavigationGoals();
            if (this.mapView && this.mapView.clearNavTarget) {
                this.mapView.clearNavTarget();
            }

            // 5. Cancel docking goals
            this.robotActions.cancelDockGoal();

            // 6. Invalidate exploration
            if (this.exploreTopic) {
                this.exploreTopic.publish({ data: false });
                this.updateExploreButton(false);
            }

            // Remove active classes
            if (this.coverageExecuteBtn) this.coverageExecuteBtn.classList.remove('is-active');
            if (this.exploreBtn) this.exploreBtn.classList.remove('is-active');
            this.robotActions.setDockingState(false);
        };

        if (this.coverageStopBtn) {
            this.listeners.add(this.coverageStopBtn, 'click', triggerEmergencyStop);
        }

        if (this.cameraStopBtn) {
            this.listeners.add(this.cameraStopBtn, 'click', (e) => {
                e.stopPropagation();
                triggerEmergencyStop();
            });
        }

        // Coverage Refresh Map
        if (this.coverageRefreshMapBtn) {
            this.listeners.add(this.coverageRefreshMapBtn, 'click', () => {
                this.updateCoverageControls({ state: 'planning', message: 'Re-extracting map boundary...' });
                this.sendCoverageCommand(CMDS.COVERAGE_REFRESH_MAP);
            });
        }

        // Draw Zone Button
        if (this.coverageDrawZoneBtn) {
            this.listeners.add(this.coverageDrawZoneBtn, 'click', () => {
                if (!this.mapView) return;
                this.mapView.startZoneDraw();
            });
        }

        // Clear Zone Button
        if (this.coverageClearZoneBtn) {
            this.listeners.add(this.coverageClearZoneBtn, 'click', () => {
                if (!this.mapView) return;
                this.mapView.clearZone();
                this.updateCoverageControls({ state: 'idle', message: 'Zone cleared. SLAM boundary active.' });
            });
        }

        // Go to Point Button
        if (this.gotoPointBtn) {
            this.listeners.add(this.gotoPointBtn, 'click', () => {
                if (!this.mapView) return;
                this.mapView.startNavPoint();
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

    destroy() {
        this.listeners.destroy();
        this.headerWidgets?.destroy();
        this.systemOperations?.destroy();
        this.robotActions?.destroy();
        if (this.exploreStatusTopic && this.exploreStatusCallback) {
            this.exploreStatusTopic.unsubscribe(this.exploreStatusCallback);
            this.exploreStatusTopic = null;
        }
        if (this.coverageStatusTopic && this.coverageStatusCallback) {
            this.coverageStatusTopic.unsubscribe(this.coverageStatusCallback);
            this.coverageStatusTopic = null;
        }
        if (this._toastTimer) {
            clearTimeout(this._toastTimer);
            this._toastTimer = null;
        }
        if (this.mapView) {
            this.mapView.onPublishCommand = null;
            this.mapView.onNavigateToPoint = null;
            this.mapView.onZoneDrawModeChange = null;
            this.mapView.onNavPointModeChange = null;
        }
        [this.exploreTopic, this.coverageCommandTopic, this.systemCommandTopic].forEach((topic) => {
            try { topic?.unadvertise(); } catch (_error) { }
        });
    }
}
