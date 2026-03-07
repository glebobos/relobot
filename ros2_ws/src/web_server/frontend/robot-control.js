/* ===== Camera stream ================================================== */
const cameraStream = document.getElementById('cameraStream');
const streamClientId = 'web-ui-' + Math.random().toString(36).substring(2, 9);
const cameraBaseUrl = `http://${window.location.hostname}:8080/stream?topic=/camera/image_raw&type=mjpeg&client_id=${streamClientId}`;

function connectCamera() {
    if (!cameraStream) return;
    console.log('[WebVideoServer] Connecting to camera stream...');
    cameraStream.src = cameraBaseUrl + '&t=' + Date.now();
    cameraStream.onload = () => console.log('[WebVideoServer] Camera stream connected');
    cameraStream.onerror = () => console.warn('[WebVideoServer] Camera stream error');
}

function stopCamera() {
    if (!cameraStream) return;
    console.log('[WebVideoServer] Stopping camera stream...');
    cameraStream.removeAttribute('src');
    fetch(`http://${window.location.hostname}:8080/shutdown?topic=/camera/image_raw&client_id=${streamClientId}`)
        .catch(err => console.debug('Stream shutdown error (expected if server offline)', err));
}

connectCamera();

document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible') {
        console.log('[WebVideoServer] Page visible, reconnecting camera stream...');
        connectCamera();
    } else {
        console.log('[WebVideoServer] Page hidden, disconnecting camera stream...');
        stopCamera();
    }
});
window.addEventListener('beforeunload', stopCamera);


/* ===== Controller config (mirrors controller_config.json defaults) ==== */
const CFG = {
    turnAxis: 0,
    driveAxis: 3,
    knifeButton: 7,
    pidToggleButton: 9,
    cruiseButton: 4,
    scaleLinear: 0.3,
    scaleAngular: 1.0,
    invertTurn: false,
    invertDrive: true,
    invertKnife: false,
    knifeMinRpm: 500,
    knifeMaxRpm: 3000,
};


/* ===== Rosbridge publishers / service client ========================== */
// window.rosAction is set up in index.html (roslib v2 ESM module).
// We wait for it before constructing Topic/Service objects.

let cmdVelTopic = null;
let knifeRpmTopic = null;
let pidService = null;
let exploreTopic = null;

function initRosControl() {
    if (!window.rosAction) {
        setTimeout(initRosControl, 50);
        return;
    }
    const { ros, Topic, Service } = window.rosAction;

    cmdVelTopic = new Topic({
        ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist',
    });

    knifeRpmTopic = new Topic({
        ros,
        name: '/knives/set_rpm',
        messageType: 'std_msgs/Float32',
    });

    pidService = new Service({
        ros,
        name: '/knives/enable_pid',
        serviceType: 'std_srvs/SetBool',
    });

    exploreTopic = new Topic({
        ros,
        name: '/explore/resume',
        messageType: 'std_msgs/Bool',
    });

    const exploreStatusTopic = new Topic({
        ros,
        name: '/explore/status',
        messageType: 'explore_lite_msgs/ExploreStatus',
    });
    exploreStatusTopic.subscribe((msg) => {
        const exploring = msg.status === 'exploration_started' || msg.status === 'exploration_in_progress';
        updateExploreButton(exploring);
    });

    console.log('[RosControl] Topics and service client ready');
}
initRosControl();

const exploreBtn = document.getElementById('explore-btn');
let exploringActive = false;

function updateExploreButton(exploring) {
    exploringActive = exploring;
    if (exploring) {
        exploreBtn.textContent = 'Stop Exploration';
        exploreBtn.className = 'map-btn explore-btn-active';
    } else {
        exploreBtn.textContent = 'Start Exploration';
        exploreBtn.className = 'map-btn explore-btn-idle';
    }
}

exploreBtn.addEventListener('click', () => {
    if (!exploreTopic) return;
    const next = !exploringActive;
    exploreTopic.publish({ data: next });
    updateExploreButton(next);
});

/** Publish a Twist message – linear.x and angular.z only. */
function publishTwist(linear, angular) {
    if (!cmdVelTopic) return;
    cmdVelTopic.publish({
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
    });
}

/** Publish desired knife RPM. */
function publishRpm(rpm) {
    if (!knifeRpmTopic) return;
    knifeRpmTopic.publish({ data: rpm });
}

/** Call the PID enable/disable service. */
function callPidService(enable) {
    if (!pidService) return;
    // roslib requires Request wrapper depending on version
    pidService.callService({ data: enable },
        (res) => console.log('[PID] toggle ok, result:', res),
        (err) => console.error('[PID] toggle failed:', err)
    );
}


/* ===== Motor helpers ================================================== */
function computeTwist(turn, drive) {
    const linear = (CFG.invertDrive ? -drive : drive) * CFG.scaleLinear;
    const angular = (CFG.invertTurn ? -turn : turn) * CFG.scaleAngular;
    return { linear, angular };
}


/* ===== Touch / drag joystick on the camera view ====================== */
const videoContainer = document.querySelector('.video-container');
let isActive = false;
let startX = 0;
let startY = 0;
const CONTROL_RADIUS = 80; // pixels → full speed

function handleControl(x, y) {
    const dx = x - startX;
    const dy = y - startY;
    const normX = Math.max(-1, Math.min(1, -dx / CONTROL_RADIUS));
    const normY = Math.max(-1, Math.min(1, dy / CONTROL_RADIUS));
    throttleUpdateMotors(normX, normY);
}

function updateMotors(x, y) {
    // x = sideways turn, y = forward/back
    const { linear, angular } = computeTwist(x, y);
    publishTwist(linear, angular);
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

videoContainer.addEventListener('mousedown', handleStart);
videoContainer.addEventListener('touchstart', handleStart, { passive: false });
document.addEventListener('mousemove', handleMove);
document.addEventListener('touchmove', handleMove, { passive: false });
document.addEventListener('mouseup', handleEnd);
document.addEventListener('touchend', handleEnd);


/* ===== Gamepad API ==================================================== */
const IDLE_ZONE_CONFIG = { 0: 0.07, 3: 0.07 };
const DISABLED_AXES = new Set([1, 2]);

function applyDeadZone(axisIndex, value) {
    if (DISABLED_AXES.has(axisIndex)) return 0.0;
    const threshold = IDLE_ZONE_CONFIG[axisIndex] ?? 0.0;
    if (Math.abs(value) < threshold) return 0.0;
    const sign = value > 0 ? 1 : -1;
    return ((Math.abs(value) - threshold) / (1 - threshold)) * sign;
}

function isAllZero(axes, buttons) {
    return axes.every(v => Math.abs(v) < 1e-5) && buttons.every(v => Math.abs(v) < 1e-5);
}

// ---- knife / cruise control state ----
let knifeCurrentRpm = 0.0;
let cruiseActive = false;
let cruiseRpm = 0.0;
let cruiseBtnWasPressed = false;

// Publish knife RPM at a fixed interval (mirrors the Python 200 ms loop)
setInterval(() => publishRpm(knifeCurrentRpm), 200);

function processKnife(knifeVal, cruiseBtnVal) {
    const rawRpm = knifeVal <= 0.05
        ? 0.0
        : CFG.knifeMinRpm + knifeVal * (CFG.knifeMaxRpm - CFG.knifeMinRpm);
    const currentRpm = (CFG.invertKnife && rawRpm !== 0) ? -rawRpm : rawRpm;
    const knifeIsActive = knifeVal > 0.05;
    const cruiseBtnIsPressed = cruiseBtnVal > 0.5;
    const justPressed = cruiseBtnIsPressed && !cruiseBtnWasPressed;
    cruiseBtnWasPressed = cruiseBtnIsPressed;

    if (justPressed) {
        if (!cruiseActive) {
            cruiseRpm = currentRpm;
            cruiseActive = true;
            console.log(`[Knife] Cruise ON @ ${cruiseRpm.toFixed(0)} RPM`);
        } else {
            cruiseActive = false;
            console.log('[Knife] Cruise OFF');
        }
    }

    // Pressing the knife button while cruise is active (without cruise button) resets cruise
    if (cruiseActive && knifeIsActive && !cruiseBtnIsPressed) {
        cruiseActive = false;
        console.log('[Knife] Cruise reset – knife button pressed');
    }

    knifeCurrentRpm = cruiseActive ? cruiseRpm : currentRpm;
}

// ---- PID toggle state ----
let pidEnabled = true;
let pidLastToggle = 0;
const PID_COOLDOWN_MS = 500;
let pidBtnWasPressed = false;

function processPidToggle(btnVal) {
    const isPressed = btnVal > 0.5;
    const justPressed = isPressed && !pidBtnWasPressed;
    pidBtnWasPressed = isPressed;

    if (!justPressed) return;

    const now = Date.now();
    if (now - pidLastToggle < PID_COOLDOWN_MS) return;
    pidLastToggle = now;

    pidEnabled = !pidEnabled;
    callPidService(pidEnabled);
    console.log(`[PID] toggled → ${pidEnabled ? 'enabled' : 'disabled'}`);
}

// ---- main gamepad poll ----
let lastGamepadSent = 0;
let lastSentNonZero = false;

function pollGamepad() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = gamepads[0];
    if (!gp) return;

    const processedAxes = gp.axes.map((val, idx) => applyDeadZone(idx, val));
    const buttons = gp.buttons.map(b => b.value);
    const now = Date.now();

    // Knife and PID toggle run on every frame for accurate edge detection
    processKnife(buttons[CFG.knifeButton] ?? 0, buttons[CFG.cruiseButton] ?? 0);
    processPidToggle(buttons[CFG.pidToggleButton] ?? 0);

    // Motion is rate-limited to 10 Hz to avoid flooding rosbridge
    if (now - lastGamepadSent > 100) {
        const isZeroNow = isAllZero(processedAxes, buttons);
        if (!isZeroNow || (isZeroNow && lastSentNonZero)) {
            const turn = processedAxes[CFG.turnAxis] ?? 0;
            const drive = processedAxes[CFG.driveAxis] ?? 0;
            const { linear, angular } = computeTwist(turn, drive);
            publishTwist(linear, angular);
            lastSentNonZero = !isZeroNow;
        }
        lastGamepadSent = now;
    }
}

function startGamepadPolling() {
    setInterval(pollGamepad, 20);
}

window.addEventListener('gamepadconnected', (e) => {
    console.log('[Gamepad] Connected:', e.gamepad.id);
    startGamepadPolling();
});


/* ===== Telemetry overlay ============================================== */
const vinSpan = document.getElementById('vin-display');
const rpmSpan = document.getElementById('rpm-display');

const VOLTAGE_THRESHOLD = 24;

function updateVoltage(volts) {
    vinSpan.textContent = volts.toFixed(1) + ' V';
    vinSpan.classList.toggle('warning', volts < VOLTAGE_THRESHOLD);
}

function updateRpm(rpm) {
    rpmSpan.textContent = Math.round(rpm) + ' RPM';
}

function initTelemetry() {
    if (!window.rosAction) {
        setTimeout(initTelemetry, 50);
        return;
    }
    const { ros, Topic } = window.rosAction;

    if (vinSpan) {
        const vinTopic = new Topic({
            ros,
            name: '/knives/vin',
            messageType: 'std_msgs/Float32',
            throttle_rate: 1000,
        });
        vinTopic.subscribe(msg => {
            const v = parseFloat(msg.data);
            if (isFinite(v)) updateVoltage(v);
        });
    }

    if (rpmSpan) {
        const rpmTopic = new Topic({
            ros,
            name: '/knives/current_rpm',
            messageType: 'std_msgs/Float32',
            throttle_rate: 1000,
        });
        rpmTopic.subscribe(msg => {
            const r = parseFloat(msg.data);
            if (isFinite(r)) updateRpm(r);
        });
    }
}
initTelemetry();


/* ===== Map / Save Map ================================================= */
const saveMapBtn = document.getElementById('save-map-btn');
if (saveMapBtn) {
    saveMapBtn.addEventListener('click', () => {
        if (!confirm('Save current map? This will overwrite the previous save.')) return;
        if (!window.rosAction) { alert('rosAction not ready'); return; }
        const { ros, Service } = window.rosAction;
        const saveMapClient = new Service({
            ros,
            name: '/slam_toolbox/serialize_map',
            serviceType: 'slam_toolbox/srv/SerializePoseGraph',
        });
        saveMapClient.callService(
            { filename: 'map_serialized' },
            (result) => { console.log('[SaveMap] result:', result); alert('Map saved!'); },
            (error) => { console.error('[SaveMap] error:', error); alert('Error saving map: ' + error); },
        );
    });
}


/* ===== Dock / Undock ================================================== */
const BT_DIR = '/ros2_ws/install/nav2/share/nav2/behavior_trees';
const dockBtn = document.getElementById('dock-btn');
const undockBtn = document.getElementById('undock-btn');

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

if (dockBtn) dockBtn.addEventListener('click', (e) => { e.stopPropagation(); sendDockGoal(); });
if (undockBtn) undockBtn.addEventListener('click', (e) => { e.stopPropagation(); sendUndockGoal(); });
