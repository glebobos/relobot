import { mapStreamService } from '../../services/map-stream-service.js';
import { rosService } from '../../services/ros-service.js';
import { MSG_TYPES, TOPICS } from '../../shared/constants.js';

export class MapRosAdapter {
    constructor(handlers) {
        this.handlers = handlers;
        this.subscriptions = [];
        this.mapUnsubscribe = null;
        this.coveragePolygonSubscription = null;

        this.subscribe();
    }

    subscribe() {
        this.add(TOPICS.COVERAGE_PREVIEW_PATH, MSG_TYPES.PATH, this.handlers.previewPath);
        this.add(
            TOPICS.COVERAGE_OBSTACLES_ACTIVE,
            MSG_TYPES.STRING,
            this.handlers.obstacles,
            { durability: 'transient_local', reliability: 'reliable' },
        );
        this.add(
            TOPICS.ROBOT_POSE,
            MSG_TYPES.POSE_STAMPED,
            this.handlers.robotPose,
            { throttle_rate: 33 },
        );

        this.subscribeMap();

        // Dynamically subscribe to coverage polygon if enabled by user
        const coverageEnabled = localStorage.getItem('navigation_coverage_boundary_enabled') === 'true';
        this.subscribeCoveragePolygon(coverageEnabled);
    }

    subscribeMap() {
        if (this.mapUnsubscribe || !this.handlers?.map) return;
        this.mapUnsubscribe = mapStreamService.subscribe(this.handlers.map);
    }

    unsubscribeMap() {
        if (!this.mapUnsubscribe) return;
        try {
            this.mapUnsubscribe();
        } catch (e) {
            console.warn('[MapRosAdapter] Failed to unsubscribe from binary map:', e);
        }
        this.mapUnsubscribe = null;
    }

    subscribeCoveragePolygon(enabled) {
        if (enabled) {
            if (!this.coveragePolygonSubscription) {
                console.log('[MapRosAdapter] Subscribing to active coverage polygon...');
                this.coveragePolygonSubscription = rosService.subscribe(
                    TOPICS.COVERAGE_POLYGON_ACTIVE,
                    MSG_TYPES.POLYGON_STAMPED,
                    this.handlers.mapPolygon,
                    { durability: 'transient_local', reliability: 'reliable' }
                );
            }
        } else {
            if (this.coveragePolygonSubscription) {
                console.log('[MapRosAdapter] Unsubscribing from active coverage polygon...');
                try {
                    this.coveragePolygonSubscription.unsubscribe();
                } catch (e) {
                    console.warn('[MapRosAdapter] Failed to unsubscribe from coverage polygon:', e);
                }
                this.coveragePolygonSubscription = null;
            }
        }
    }

    add(name, messageType, handler, options = {}) {
        if (typeof handler !== 'function') return;
        this.subscriptions.push(rosService.subscribe(name, messageType, handler, options));
    }

    destroy() {
        this.unsubscribeMap();
        this.subscriptions.forEach(subscription => subscription.unsubscribe());
        this.subscriptions = [];
        if (this.coveragePolygonSubscription) {
            try {
                this.coveragePolygonSubscription.unsubscribe();
            } catch (e) {
                console.warn('[MapRosAdapter] Failed to unsubscribe from coverage polygon in destroy:', e);
            }
            this.coveragePolygonSubscription = null;
        }
        this.handlers = null;
    }
}
