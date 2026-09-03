import { Ros, Action, Service, Topic } from 'roslib';
import { RosSubscription } from './ros-subscription.js';

class RosService {
    constructor() {
        const isHttps = window.location.protocol === 'https:';
        const protocol = isHttps ? 'wss:' : 'ws:';
        const port = isHttps ? '' : ':9090';
        const path = isHttps ? '/rosbridge/' : '';
        this.url = `${protocol}//${window.location.hostname}${port}${path}`;
        
        this._reconnectTimer = null;
        this._stableTimer = null;
        this._backoffMs = 500;
        this._minBackoffMs = 500;
        this._maxBackoffMs = 5000;
        this._managedSubscriptions = new Set();
        
        // Initialize Ros instance
        this.ros = new Ros({ url: this.url });
        this._setupListeners();
    }

    _setupListeners() {
        this.ros.on('connection', () => {
            console.log('[RosService] ROS WebSocket Connected');
            if (this._reconnectTimer) {
                clearTimeout(this._reconnectTimer);
                this._reconnectTimer = null;
            }

            // Only reset backoff after connection remains stable for 5 seconds
            // This prevents hammering a flapping server in rapid 500ms retry loops
            if (this._stableTimer) clearTimeout(this._stableTimer);
            this._stableTimer = setTimeout(() => {
                this._backoffMs = this._minBackoffMs;
                this._stableTimer = null;
            }, 5000);

            this._resubscribeAll();
        });

        this.ros.on('error', (e) => {
            console.warn('[RosService] ROS WebSocket Error:', e);
            this._handleDisconnect();
        });

        this.ros.on('close', () => {
            console.warn('[RosService] ROS WebSocket Connection Closed');
            this._handleDisconnect();
        });
    }

    _handleDisconnect() {
        if (this._stableTimer) {
            clearTimeout(this._stableTimer);
            this._stableTimer = null;
        }

        if (this._reconnectTimer) return;

        console.log(`[RosService] Scheduling auto-reconnect in ${this._backoffMs}ms...`);
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            // Increase backoff for next failure
            this._backoffMs = Math.min(Math.round(this._backoffMs * 1.5), this._maxBackoffMs);
            console.log(`[RosService] Attempting auto-reconnect to ${this.url}...`);
            try {
                this.ros.connect(this.url);
            } catch (err) {
                console.warn('[RosService] Auto-reconnect attempt error:', err);
                this._handleDisconnect();
            }
        }, this._backoffMs);
    }

    _resubscribeAll() {
        if (!this._managedSubscriptions.size) return;
        console.log(`[RosService] Re-hydrating ${this._managedSubscriptions.size} active subscriptions...`);
        for (const entry of this._managedSubscriptions) {
            try {
                // Re-create a fresh Topic instance bound to the reconnected Ros instance
                // to avoid stale socket/subscription state in roslibjs across reconnects
                entry.topic = this.createTopic(entry.name, entry.messageType, entry.options);
                entry.topic.subscribe(entry.callback);
                console.log(`[RosService] Resubscribed: ${entry.name} [${entry.messageType}]`);
            } catch (err) {
                console.error(`[RosService] Failed to resubscribe to ${entry.name}:`, err);
            }
        }
    }

    get isConnected() {
        return Boolean(this.ros?.isConnected);
    }

    createTopic(name, messageType, options = {}) {
        if (!name || typeof name !== 'string') {
            console.error('[RosService] createTopic called with invalid name:', name);
        }
        if (!messageType || typeof messageType !== 'string') {
            console.error(`[RosService] createTopic for "${name}" called with invalid messageType:`, messageType);
        }
        return new Topic({
            ros: this.ros,
            name,
            messageType,
            ...options
        });
    }

    subscribe(name, messageType, callback, options = {}) {
        if (typeof callback !== 'function') {
            console.error(`[RosService] subscribe called on "${name}" without valid callback function.`);
        }
        const topic = this.createTopic(name, messageType, options);
        topic.subscribe(callback);
        console.log(`[RosService] Subscribed to "${name}" [${messageType}]`);

        const entry = { name, messageType, options, callback, topic };
        this._managedSubscriptions.add(entry);

        return new RosSubscription(topic, callback, () => {
            this._managedSubscriptions.delete(entry);
            console.log(`[RosService] Unsubscribed from "${name}"`);
        });
    }

    createAction(name, actionType) {
        return new Action({
            ros: this.ros,
            name,
            actionType
        });
    }

    createService(name, serviceType) {
        return new Service({
            ros: this.ros,
            name,
            serviceType
        });
    }

    disconnect() {
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }
        if (this._stableTimer) {
            clearTimeout(this._stableTimer);
            this._stableTimer = null;
        }
        try {
            this.ros?.close();
        } catch (_) {}
    }

    destroy() {
        this.disconnect();
        for (const entry of this._managedSubscriptions) {
            try {
                entry.topic?.unsubscribe(entry.callback);
            } catch (_) {}
        }
        this._managedSubscriptions.clear();
    }

    // Aliases for seamless migration
    get rosV2() {
        return this.ros;
    }
    createTopicV2(name, messageType, options = {}) {
        return this.createTopic(name, messageType, options);
    }
    subscribeV2(name, messageType, callback, options = {}) {
        return this.subscribe(name, messageType, callback, options);
    }
    createActionV2(name, actionType) {
        return this.createAction(name, actionType);
    }
    createServiceV2(name, serviceType) {
        return this.createService(name, serviceType);
    }
}

export const rosService = new RosService();
