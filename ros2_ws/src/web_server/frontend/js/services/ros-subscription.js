/**
 * Owns one ROSLIB topic callback and makes cleanup idempotent.
 * ROSLIB Topic.subscribe() returns void, so callers must retain both values.
 */
export class RosSubscription {
    constructor(target, callbackOrOnUnsubscribe = null, onUnsubscribe = null) {
        if (target && typeof target === 'object' && 'callback' in target) {
            // target is a managed subscription entry: { name, messageType, options, callback, topic }
            this.entry = target;
            this.onUnsubscribe = callbackOrOnUnsubscribe;
        } else {
            // legacy signature: (topic, callback, onUnsubscribe)
            this.entry = { topic: target, callback: callbackOrOnUnsubscribe };
            this.onUnsubscribe = onUnsubscribe;
        }
        this.closed = false;
    }

    get name() {
        return this.entry?.name || this.entry?.topic?.name || '';
    }

    get topic() {
        return this.entry?.topic || null;
    }

    get callback() {
        return this.entry?.callback || null;
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
        if (this.entry?.topic && this.entry?.callback) {
            try {
                this.entry.topic.unsubscribe(this.entry.callback);
                if (typeof this.entry.topic.removeAllListeners === 'function') {
                    this.entry.topic.removeAllListeners();
                }
            } catch (_) {}
        }
        this.entry = null;
    }
}
