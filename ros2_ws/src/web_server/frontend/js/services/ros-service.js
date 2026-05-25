import { Ros, Action, Service, Topic } from 'roslib';

class RosService {
    constructor() {
        this.url = `ws://${window.location.hostname}:9090`;
        this._connectedV2 = false;
        this._connectedV1 = false;
        
        // Initialize ESM Ros (v2)
        this.rosV2 = new Ros({ url: this.url });
        this.rosV2.on('connection', () => {
            this._connectedV2 = true;
            console.log('[RosService] ESM Ros (v2) Connected');
        });
        this.rosV2.on('error', (e) => console.error('[RosService] ESM Ros (v2) Error:', e));

        // Initialize Legacy Ros (v1) for ROS3D/ThreeJS
        // Since ROSLIB v1 is loaded as a global script, we access it via window.ROSLIB
        if (window.ROSLIB) {
            this.rosV1 = new window.ROSLIB.Ros({ url: this.url });
            this.rosV1.on('connection', () => {
                this._connectedV1 = true;
                console.log('[RosService] Legacy Ros (v1) Connected');
            });
            this.rosV1.on('error', (e) => console.error('[RosService] Legacy Ros (v1) Error:', e));
        } else {
            console.error('[RosService] Legacy ROSLIB not found in global scope!');
        }
    }

    createTopicV2(name, messageType, options = {}) {
        return new Topic({
            ros: this.rosV2,
            name,
            messageType,
            ...options
        });
    }

    createTopicV1(name, messageType, options = {}) {
        if (!window.ROSLIB) return null;
        return new window.ROSLIB.Topic({
            ros: this.rosV1,
            name,
            messageType,
            ...options
        });
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
