/**
 * Owns one ROSLIB topic callback and makes cleanup idempotent.
 * ROSLIB Topic.subscribe() returns void, so callers must retain both values.
 */
export class RosSubscription {
    constructor(topic, callback, onUnsubscribe = null) {
        this.topic = topic;
        this.callback = callback;
        this.onUnsubscribe = onUnsubscribe;
        this.closed = false;
    }

    get name() {
        return this.topic?.name || '';
    }

    unsubscribe() {
        if (this.closed) return;
        this.closed = true;
        if (this.onUnsubscribe) {
            try {
                this.onUnsubscribe();
            } catch (_) {}
            this.onUnsubscribe = null;
        }
        if (this.topic && this.callback) {
            try {
                this.topic.unsubscribe(this.callback);
            } catch (_) {}
        }
        this.topic = null;
        this.callback = null;
    }
}
