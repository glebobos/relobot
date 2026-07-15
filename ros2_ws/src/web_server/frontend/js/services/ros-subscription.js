/**
 * Owns one ROSLIB topic callback and makes cleanup idempotent.
 * ROSLIB Topic.subscribe() returns void, so callers must retain both values.
 */
export class RosSubscription {
    constructor(topic, callback) {
        this.topic = topic;
        this.callback = callback;
        this.closed = false;
    }

    get name() {
        return this.topic?.name || '';
    }

    unsubscribe() {
        if (this.closed) return;
        this.closed = true;
        if (this.topic && this.callback) this.topic.unsubscribe(this.callback);
        this.topic = null;
        this.callback = null;
    }
}
