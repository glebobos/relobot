const cameraStream = document.getElementById('cameraStream');
const cameraBaseUrl = `http://${window.location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg`;

(function initCameraFeed() {
    if (!cameraStream) return;

    cameraStream.src = cameraBaseUrl;

    cameraStream.onload = function () {
        console.log('[WebVideoServer] Camera stream connected');
    };

    cameraStream.onerror = function () {
        console.warn('[WebVideoServer] Camera stream error, retrying...');
        setTimeout(() => {
            cameraStream.src = cameraBaseUrl + '&t=' + Date.now();
        }, 2000);
    };

    console.log('[WebVideoServer] Connecting to camera stream...');
})();

/* ===== Reconnect camera stream on visibility change =================== */
document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible' && cameraStream) {
        console.log('[WebVideoServer] Page visible, reconnecting camera stream...');
        cameraStream.src = cameraBaseUrl + '&t=' + Date.now();
    }
});

// Invisible touch control on video
const videoContainer = document.querySelector('.video-container');
let isActive = false;
let startX = 0;
let startY = 0;
const CONTROL_RADIUS = 80; // pixels to reach full speed

function handleControl(x, y) {
    const dx = x - startX;
    const dy = y - startY;

    // Normalize based on control radius, clamp to -1 to 1
    let normalizedX = Math.max(-1, Math.min(1, dx / CONTROL_RADIUS));
    let normalizedY = Math.max(-1, Math.min(1, -dy / CONTROL_RADIUS));

    throttleUpdateMotors(normalizedX, normalizedY);
}

function updateMotors(x, y) {
    fetch('/set_motors', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y }),
    });
}


function throttle(func, limit) {
    let lastFunc, lastRan;
    return function (...args) {
        if (!lastRan) {
            func.apply(this, args);
            lastRan = Date.now();
        } else {
            clearTimeout(lastFunc);
            lastFunc = setTimeout(() => {
                if ((Date.now() - lastRan) >= limit) {
                    func.apply(this, args);
                    lastRan = Date.now();
                }
            }, limit - (Date.now() - lastRan));
        }
    };
}

const throttleUpdateMotors = throttle(updateMotors, 100);

function handleStart(e) {
    // Only start if touch/click is on video container
    if (!videoContainer.contains(e.target) && e.target !== videoContainer) return;

    isActive = true;
    const touch = e.touches ? e.touches[0] : e;
    startX = touch.clientX;
    startY = touch.clientY;
    e.preventDefault();
}

function handleMove(e) {
    if (!isActive) return;
    const touch = e.touches ? e.touches[0] : e;
    handleControl(touch.clientX, touch.clientY);
    e.preventDefault();
}

function handleEnd() {
    if (!isActive) return;
    isActive = false;
    throttleUpdateMotors(0, 0);
}

// Event listeners on video container
videoContainer.addEventListener('mousedown', handleStart);
videoContainer.addEventListener('touchstart', handleStart, { passive: false });

document.addEventListener('mousemove', handleMove);
document.addEventListener('touchmove', handleMove, { passive: false });

document.addEventListener('mouseup', handleEnd);
document.addEventListener('touchend', handleEnd);

// Camera connection status
let cameraConnected = false;
cameraStream.onload = function () {
    if (!cameraConnected) {
        cameraConnected = true;
        console.log('Camera stream connected');
    }
};

cameraStream.onerror = function () {
    cameraConnected = false;
    console.warn('Camera stream interrupted, reconnecting...');
};

// Gamepad API
const IDLE_ZONE_CONFIG = { 0: 0.07, 3: 0.07 };
const DISABLED_AXES = new Set([1, 2]);

function applyDeadZone(axisIndex, value) {
    if (DISABLED_AXES.has(axisIndex)) return 0.0;
    const threshold = IDLE_ZONE_CONFIG[axisIndex] ?? 0.0;
    if (Math.abs(value) < threshold) return 0.0;
    const sign = value > 0 ? 1 : -1;
    return ((Math.abs(value) - threshold) / (1 - threshold)) * sign;
}

let lastGamepadSent = 0;
let lastSentNonZero = false;

function isZeroOrAllFalse(axes, buttons) {
    return axes.every(val => Math.abs(val) < 1e-5) && buttons.every(val => Math.abs(val) < 1e-5);
}

function pollGamepad() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = gamepads[0];
    if (!gp) return;

    const processedAxes = gp.axes.map((val, idx) => applyDeadZone(idx, val));
    const buttons = gp.buttons.map(b => b.value);
    const now = Date.now();

    if (now - lastGamepadSent > 100) {
        const isZeroNow = isZeroOrAllFalse(processedAxes, buttons);
        if (!isZeroNow || (isZeroNow && lastSentNonZero)) {
            fetch('/gamepad', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ axes: processedAxes, buttons: buttons }),
            });
            lastSentNonZero = !isZeroNow;
        }
        lastGamepadSent = now;
    }
}

function startGamepadPolling() {
    setInterval(pollGamepad, 20);
}

window.addEventListener("gamepadconnected", function (e) {
    console.log("Gamepad connected:", e.gamepad);
    startGamepadPolling();
});
/* ===== Video overlay helpers ========================================== */
const vinSpan = document.getElementById('vin-display');
const rpmSpan = document.getElementById('rpm-display');

const VOLTAGE_THRESHOLD = 24;

function updateVoltage(volts) {
    vinSpan.textContent = volts.toFixed(1) + ' V';
    if (volts < VOLTAGE_THRESHOLD) {
        vinSpan.classList.add('warning');
    } else {
        vinSpan.classList.remove('warning');
    }
}

function updateRPM(rpm) {
    rpmSpan.textContent = Math.round(rpm) + ' RPM';
}


/* ===== Live voltage overlay =========================================== */
(() => {
    if (!vinSpan) return;

    // first snapshot
    fetch('/api/voltage')
        .then(r => r.ok ? r.json() : null)
        .then(j => j?.success && updateVoltage(j.vin))
        .catch(() => { });

    // live updates via SSE
    const es = new EventSource('/stream/voltage');
    es.onmessage = e => {
        const v = parseFloat(e.data);
        if (isFinite(v)) updateVoltage(v);
    };
    es.onerror = () =>
        console.error('Voltage SSE error – browser will auto-retry');
})();

/* ===== Live RPM overlay =============================================== */
(() => {
    if (!rpmSpan) return;

    // first snapshot
    fetch('/api/rpm')
        .then(r => r.ok ? r.json() : null)
        .then(j => j?.success && updateRPM(j.rpm))
        .catch(() => { });

    // live updates via SSE
    const es = new EventSource('/stream/rpm');
    es.onmessage = e => {
        const r = parseFloat(e.data);
        if (isFinite(r)) updateRPM(r);
    };
    es.onerror = () =>
        console.error('RPM SSE error – browser will auto-retry');
})();

/* ===== Map Config ==================================================== */
const saveMapBtn = document.getElementById('save-map-btn');
if (saveMapBtn) {
    saveMapBtn.addEventListener('click', () => {
        if (confirm('Save current map? This will overwrite the previous save.')) {
            if (!window.rosAction) {
                alert('rosAction not ready');
                return;
            }
            const { ros, Service } = window.rosAction;
            const saveMapClient = new Service({
                ros,
                name: '/slam_toolbox/serialize_map',
                serviceType: 'slam_toolbox/srv/SerializePoseGraph'
            });

            saveMapClient.callService(
                { filename: 'map_serialized' },
                (result) => {
                    console.log('[SaveMap] result:', result);
                    alert('Map saved successfully!');
                },
                (error) => {
                    console.error('[SaveMap] error:', error);
                    alert('Error saving map: ' + error);
                }
            );
        }
    });
}

/* ===== Dock/Undock Controls (via roslib v2.1.0 Action) =============== */
const BT_DIR = '/ros2_ws/install/nav2/share/nav2/behavior_trees';

const dockBtn = document.getElementById('dock-btn');
const undockBtn = document.getElementById('undock-btn');

/**
 * roslib v2.1.0 uses the rosbridge send_action_goal protocol and correctly
 * routes action_feedback / action_result messages back by goal ID.
 * v1.4.1 (kept for ros3d) does NOT route these ops, hence the separate
 * window.rosAction connection created in index.html.
 */
function sendDockGoal() {
    if (!window.rosAction) { console.error('[Dock] rosAction not ready'); return; }
    const { ros, Action } = window.rosAction;
    const action = new Action({
        ros,
        name: '/dock_robot',
        actionType: 'opennav_docking_msgs/action/DockRobot',
    });
    const id = action.sendGoal(
        { use_dock_id: true, dock_id: 'home_dock', navigate_to_staging_pose: true, max_staging_time: 60.0 },
        (result) => console.log('[Dock] result:', result),
        (feedback) => console.log('[Dock] feedback:', feedback),
        (error) => console.error('[Dock] failed:', error),
    );
    console.log('[Dock] goal sent, id:', id);
}

function sendUndockGoal() {
    if (!window.rosAction) { console.error('[Undock] rosAction not ready'); return; }
    const { ros, Action } = window.rosAction;
    const action = new Action({
        ros,
        name: '/navigate_to_pose',
        actionType: 'nav2_msgs/action/NavigateToPose',
    });
    const id = action.sendGoal(
        {
            behavior_tree: `${BT_DIR}/undock_and_turn.xml`,
            pose: {
                header: { frame_id: 'map' },
                pose: {
                    position: { x: 0, y: 0, z: 0 },
                    orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
            },
        },
        (result) => console.log('[Undock] result:', result),
        (feedback) => console.log('[Undock] feedback:', feedback),
        (error) => console.error('[Undock] failed:', error),
    );
    console.log('[Undock] goal sent, id:', id);
}

if (dockBtn) {
    dockBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        sendDockGoal();
    });
}

if (undockBtn) {
    undockBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        sendUndockGoal();
    });
}
