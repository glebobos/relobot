import { rosService } from '../../services/ros-service.js';
import { ACTIONS, BT_DIR, DOCK_ID, MSG_TYPES, TOPICS } from '../../shared/constants.js';
import { DomListenerBag } from '../../shared/dom-listener-bag.js';

export class RobotActionsController {
    constructor(telemetry) {
        this.telemetry = telemetry;
        this.listeners = new DomListenerBag();
        this.dockButton = document.getElementById('dock-btn');
        this.undockButton = document.getElementById('undock-btn');
        this.batteryWidget = document.getElementById('headerBatteryWidget');
        this.dockAction = rosService.createActionV2(ACTIONS.DOCK_ROBOT, MSG_TYPES.DOCK_ROBOT);
        this.navigationAction = rosService.createActionV2(
            ACTIONS.NAVIGATE_TO_POSE,
            MSG_TYPES.NAVIGATE_TO_POSE,
        );
        this.dockStatusSubscription = rosService.subscribeV2(
            TOPICS.DOCK_ACTION_STATUS,
            MSG_TYPES.GOAL_STATUS_ARRAY,
            message => this.setDockingState((message.status_list || []).some(
                status => status.status === 1 || status.status === 2 || status.status === 3,
            )),
            { throttle_rate: 500 },
        );
        this.listeners.add(this.dockButton, 'click', (event) => {
            event.stopPropagation();
            if (this.dockingActive) this.cancelDockGoal();
            else this.sendDockGoal();
        });
        this.listeners.add(this.undockButton, 'click', (event) => {
            event.stopPropagation();
            this.sendUndockGoal();
        });
    }

    navigateToPoint(worldX, worldY, yaw = 0) {
        if (![worldX, worldY, yaw].every(Number.isFinite)) return;
        const halfYaw = yaw / 2;
        this.navigationAction.sendGoal(
            {
                pose: {
                    header: { frame_id: 'map' },
                    pose: {
                        position: { x: worldX, y: worldY, z: 0 },
                        orientation: { x: 0, y: 0, z: Math.sin(halfYaw), w: Math.cos(halfYaw) },
                    },
                },
            },
            result => console.log('[Navigation] Goal reached:', result),
            feedback => console.debug('[Navigation] Distance remaining:', feedback.distance_remaining),
            error => console.error('[Navigation] Goal failed:', error),
        );
    }

    sendDockGoal() {
        this.dockAction.sendGoal(
            {
                use_dock_id: true,
                dock_id: DOCK_ID,
                navigate_to_staging_pose: true,
                max_staging_time: 60,
            },
            () => this.setDockingState(false),
            feedback => console.debug('[Dock] Feedback:', feedback),
            error => {
                console.error('[Dock] Goal failed:', error);
                this.setDockingState(false);
            },
        );
    }

    cancelDockGoal() {
        if (!this.dockingActive) return;
        try {
            this.dockAction.cancelAllGoals();
        } finally {
            this.setDockingState(false);
        }
    }

    cancelNavigationGoals() {
        try {
            this.navigationAction.cancelAllGoals();
        } catch (error) {
            console.warn('[Navigation] Failed to cancel goals:', error);
        }
    }

    sendUndockGoal() {
        this.navigationAction.sendGoal(
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
            result => console.log('[Undock] Result:', result),
            () => {},
            error => console.error('[Undock] Goal failed:', error),
        );
    }

    setDockingState(active) {
        this.dockingActive = Boolean(active);
        this.telemetry?.setDockingActive(this.dockingActive);
        this.dockButton?.classList.toggle('is-active', this.dockingActive);
        this.batteryWidget?.classList.toggle('is-docking-active', this.dockingActive);
    }

    destroy() {
        this.listeners.destroy();
        this.dockStatusSubscription.unsubscribe();
        this.telemetry = null;
    }
}
