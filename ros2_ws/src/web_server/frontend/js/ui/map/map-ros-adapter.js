import { rosService } from '../../services/ros-service.js';
import { MSG_TYPES, TOPICS } from '../../shared/constants.js';

const MAP_THROTTLE_MS = 2000;

export class MapRosAdapter {
    constructor(handlers) {
        this.handlers = handlers;
        this.subscriptions = [];
        this.subscribe();
    }

    subscribe() {
        this.add(TOPICS.COVERAGE_PREVIEW_PATH, MSG_TYPES.PATH, this.handlers.previewPath);
        this.add(
            TOPICS.COVERAGE_POLYGON_ACTIVE,
            MSG_TYPES.POLYGON_STAMPED,
            this.handlers.mapPolygon,
            { durability: 'transient_local', reliability: 'reliable' },
        );
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
        this.add(
            TOPICS.MAP,
            MSG_TYPES.OCCUPANCY_GRID,
            this.handlers.map,
            { throttle_rate: MAP_THROTTLE_MS },
        );
    }

    add(name, messageType, handler, options = {}) {
        if (typeof handler !== 'function') return;
        this.subscriptions.push(rosService.subscribeV2(name, messageType, handler, options));
    }

    destroy() {
        this.subscriptions.forEach(subscription => subscription.unsubscribe());
        this.subscriptions = [];
        this.handlers = null;
    }
}
