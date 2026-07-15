import { Ros, Action, Service, Topic } from 'roslib';
import { RosSubscription } from './ros-subscription.js';

class RosService {
    constructor() {
        const isHttps = window.location.protocol === 'https:';
        const protocol = isHttps ? 'wss:' : 'ws:';
        const port = isHttps ? '' : ':9090';
        const path = isHttps ? '/rosbridge/' : '';
        this.url = `${protocol}//${window.location.hostname}${port}${path}`;
        this._connectedV2 = false;
        
        // Initialize ESM Ros (v2)
        this.rosV2 = new Ros({ url: this.url });
        this.rosV2.on('connection', () => {
            this._connectedV2 = true;
            console.log('[RosService] ESM Ros (v2) Connected');
        });
        this.rosV2.on('error', (e) => console.error('[RosService] ESM Ros (v2) Error:', e));
    }

    createTopicV2(name, messageType, options = {}) {
        return new Topic({
            ros: this.rosV2,
            name,
            messageType,
            ...options
        });
    }

    subscribeV2(name, messageType, callback, options = {}) {
        const topic = this.createTopicV2(name, messageType, options);
        topic.subscribe(callback);
        return new RosSubscription(topic, callback);
    }

    createActionV2(name, actionType) {
        return new Action({
            ros: this.rosV2,
            name,
            actionType
        });
    }

    createServiceV2(name, serviceType) {
        return new Service({
            ros: this.rosV2,
            name,
            serviceType
        });
    }
}

export const rosService = new RosService();
