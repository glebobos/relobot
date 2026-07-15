import assert from 'node:assert/strict';
import test from 'node:test';
import { RosSubscription } from '../js/services/ros-subscription.js';

test('unsubscribes the retained callback exactly once', () => {
    const callbacks = [];
    const topic = {
        name: '/test',
        unsubscribe(callback) {
            callbacks.push(callback);
        },
    };
    const callback = () => {};
    const subscription = new RosSubscription(topic, callback);
    assert.equal(subscription.name, '/test');
    subscription.unsubscribe();
    subscription.unsubscribe();
    assert.deepEqual(callbacks, [callback]);
    assert.equal(subscription.closed, true);
});
