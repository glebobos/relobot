import assert from 'node:assert/strict';
import { EventEmitter } from 'node:events';
import { readFileSync } from 'node:fs';
import test from 'node:test';
import vm from 'node:vm';

const source = readFileSync(new URL(
    '../../../ros2_ws/src/web_server/frontend/js/services/gamepad-service.js', import.meta.url,
), 'utf8').replace(/^import .*;\n/gm, '').replace(
    'export const gamepadService = new GamepadService();',
    'globalThis.gamepadService = new GamepadService();',
);

function fixture(connected = true, TopicClass = null) {
    const messages = [];
    const operations = [];
    const topics = [];
    const intervals = new Map();
    const ros = new EventEmitter();
    const rosService = {
        ros,
        isConnected: connected,
        createTopic(name, messageType, options) {
            const topic = TopicClass ? new TopicClass({ ros, name, messageType, ...options }) : {
                name, messageType, options,
                publish(message) {
                    assert.ok(rosService.isConnected, 'publishing while disconnected queues stale commands');
                    messages.push({ name, data: message.data });
                },
                unadvertise() { this.closed = true; },
            };
            topics.push(topic);
            return topic;
        },
    };
    ros.callOnConnection = operation => {
        assert.ok(rosService.isConnected || operation.op !== 'publish', 'ROSLIB queued offline motion');
        operations.push({ ...operation, connected: rosService.isConnected });
        if (operation.op === 'publish') {
            messages.push({ name: operation.topic, data: operation.msg.data });
        }
    };
    function eventTarget() {
        const events = new EventEmitter();
        events.addEventListener = events.on.bind(events);
        events.removeEventListener = events.removeListener.bind(events);
        return events;
    }
    const browserWindow = eventTarget();
    const browserDocument = eventTarget();
    browserDocument.visibilityState = 'visible';
    let nextInterval = 0;
    const context = {
        rosService,
        CFG: { knifeMinRpm: 500, knifeMaxRpm: 3000, invertKnife: false },
        TOPICS: { CMD_VEL: '/cmd_vel', CMD_KNIVES: '/knives/set_rpm' },
        MSG_TYPES: { TWIST_STAMPED: 'geometry_msgs/TwistStamped', FLOAT32: 'std_msgs/Float32' },
        window: browserWindow, document: browserDocument,
        navigator: { getGamepads: () => [] },
        setInterval(callback, period) {
            const identity = ++nextInterval;
            intervals.set(identity, { callback, period });
            return identity;
        },
        clearInterval(identity) { intervals.delete(identity); },
        console: { log() {}, warn() {} },
    };
    vm.runInNewContext(source, context);
    return {
        service: context.gamepadService, rosService, ros, messages, operations, topics, intervals,
        browserWindow, browserDocument,
        tick() {
            for (const { callback, period } of intervals.values()) {
                if (period === 200) callback();
            }
        },
        knives() { return messages.filter(message => message.name === '/knives/set_rpm'); },
    };
}

test('boot requires trigger release, and first publication is zero', () => {
    const state = fixture();
    state.service.processKnife(1, 0);
    assert.equal(state.service.knifeCurrentRpm, 0);
    state.service.processKnife(0, 0);
    state.service.processKnife(1, 0);
    state.tick();
    assert.equal(state.knives().at(-1).data, 0);
    state.tick();
    assert.equal(state.knives().at(-1).data, 3000);
});

test('disconnect clears target/cruise and never publishes offline', () => {
    const state = fixture();
    state.service.setKnifeRpm(1800, true);
    state.service.cruiseActive = true;
    state.service.cruiseRpm = 1800;
    state.rosService.isConnected = false;
    state.ros.emit('close');
    state.tick();
    state.service.processKnife(1, 1);
    assert.equal(state.messages.length, 0);
    assert.equal(state.service.knifeCurrentRpm, 0);
    assert.equal(state.service.cruiseRpm, 0);
    assert.equal(state.service.cruiseActive, false);
});

test('reconnect cannot restart from held knife or cruise buttons', () => {
    const state = fixture(false);
    state.rosService.isConnected = true;
    state.ros.emit('connection');
    for (let iteration = 0; iteration < 10; ++iteration) {
        state.service.processKnife(1, 1);
        state.tick();
    }
    assert.ok(state.knives().every(message => message.data === 0));
    state.service.processKnife(0, 0);
    state.service.processKnife(1, 0);
    state.tick();
    assert.equal(state.knives().at(-1).data, 3000);
});

test('new slider action starts only after a zero has been published', () => {
    const state = fixture();
    state.service.setKnifeRpm(1600, true);
    state.tick();
    state.tick();
    assert.deepEqual(state.knives().map(message => message.data), [0, 1600]);
});

test('offline slider changes and non-finite RPM cannot queue a restart', () => {
    const state = fixture(false);
    state.service.setKnifeRpm(1600, true);
    state.tick();
    assert.equal(state.service.knifeCurrentRpm, 0);
    assert.equal(state.messages.length, 0);
    state.rosService.isConnected = true;
    state.service.setKnifeRpm(NaN, true);
    state.service.publishRpm(Infinity);
    state.tick();
    assert.ok(state.knives().every(message => message.data === 0));
});

test('visibility loss disarms and requires released controls again', () => {
    const state = fixture();
    state.service.setKnifeRpm(1600, true);
    state.browserDocument.visibilityState = 'hidden';
    state.browserDocument.emit('visibilitychange');
    state.service.processKnife(0, 0);
    state.service.processKnife(1, 0);
    state.service.setKnifeRpm(1600, true);
    state.tick();
    assert.equal(state.service.knifeCurrentRpm, 0);
    assert.ok(state.knives().every(message => message.data === 0));
});

test('error events reset a cached command even before close', () => {
    const state = fixture();
    state.service.setKnifeRpm(1600, true);
    state.ros.emit('error', new Error('transport failed'));
    state.tick();
    assert.equal(state.service.knifeCurrentRpm, 0);
    assert.ok(state.knives().every(message => message.data === 0));
});

test('a failed stop publication cannot preserve a cached knife command', () => {
    const state = fixture();
    state.service.setKnifeRpm(1600, true);
    state.service.cruiseActive = true;
    state.service.cruiseRpm = 1600;
    state.service.cmdVelTopic.publish = () => { throw new Error('socket send failed'); };
    assert.throws(() => state.ros.emit('error', new Error('transport failed')));
    assert.equal(state.service.knifeCurrentRpm, 0);
    assert.equal(state.service.cruiseRpm, 0);
    assert.equal(state.service.cruiseActive, false);
    assert.equal(state.service.knifeReleaseRequired, true);
    state.service.processKnife(1, 1);
    state.tick();
    assert.ok(state.knives().every(message => message.data === 0));
});

test('destroy removes timers and transport listeners', () => {
    const state = fixture();
    state.service.destroy();
    assert.equal(state.intervals.size, 0);
    for (const event of ['connection', 'error', 'close']) {
        assert.equal(state.ros.listenerCount(event), 0);
    }
    assert.ok(state.topics.every(topic => topic.closed));
});

test('real ROSLIB readvertises after reconnect without queued motion', {
    skip: !process.env.ROSLIB_TEST_MODULE,
}, async () => {
    const { Topic } = await import(process.env.ROSLIB_TEST_MODULE);
    const state = fixture(true, Topic);
    state.service.setKnifeRpm(1600, true);
    state.tick();
    state.tick();
    for (let cycle = 1; cycle <= 2; ++cycle) {
        const publishCount = state.operations.filter(operation => operation.op === 'publish').length;
        state.rosService.isConnected = false;
        state.ros.emit('close');
        state.tick();
        assert.equal(state.operations.filter(operation => operation.op === 'publish').length, publishCount);
        assert.ok(state.operations.filter(operation => !operation.connected).every(operation =>
            operation.topic !== '/knives/set_rpm'));
        state.rosService.isConnected = true;
        state.ros.emit('connection');
        assert.equal(state.knives().at(-1).data, 0);
        assert.equal(state.operations.filter(operation =>
            operation.op === 'advertise' && operation.topic === '/knives/set_rpm').length, cycle + 1);
        state.service.processKnife(1, 1);
        state.tick();
        assert.equal(state.knives().at(-1).data, 0);
        state.service.processKnife(0, 0);
        state.service.processKnife(1, 0);
        state.tick();
        assert.equal(state.knives().at(-1).data, 3000);
    }
    state.service.destroy();
});
