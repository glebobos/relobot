const cameraStream = document.getElementById('cameraStream');

(function initCameraFeed() {
    if (!cameraStream) return;

    const waitForRoslib = (delay = 250) => {
        if (typeof ROSLIB === 'undefined') {
            console.warn('ROSLIB not loaded yet, retrying...');
            setTimeout(() => waitForRoslib(Math.min(delay * 2, 2000)), delay);
            return;
        }
        startRosCamera();
    };

    function startRosCamera() {
        const rosbridgeUrl = `ws://${window.location.hostname}:9090`;
        const ros = new ROSLIB.Ros({ url: rosbridgeUrl });

        ros.on('connection', () => console.log(`[ROSBridge] Connected ${rosbridgeUrl}`));
        ros.on('error', err => console.error('[ROSBridge] Error', err));
        ros.on('close', () => console.warn('[ROSBridge] Connection closed'));

        // Throttle camera updates to max 10 FPS to reduce load
        let lastCameraUpdate = 0;
        const CAMERA_THROTTLE_MS = 100; // 10 FPS max

        function setImageSrc(url) {
            if (cameraStream.src === url) return;
            cameraStream.src = url;
        }

        // Subscribe to only ONE compressed image topic
        const topic = new ROSLIB.Topic({
            ros,
            name: '/camera/image_raw/compressed',
            messageType: 'sensor_msgs/msg/CompressedImage',
            throttle_rate: 100  // Throttle at ROSBridge level to 10 FPS
        });

        topic.subscribe(msg => {
            const now = Date.now();
            if (now - lastCameraUpdate < CAMERA_THROTTLE_MS) return;
            lastCameraUpdate = now;
            
            const format = msg.format || 'image/jpeg';
            setImageSrc(`data:${format};base64,${msg.data}`);
        });
    }

    waitForRoslib();
})();

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

function emergencyStop() {
    fetch('/emergency_stop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    }).then(response => response.json())
      .then(data => {
          alert(data.success ? 'Emergency stop activated!' : 'Emergency stop failed: ' + data.error);
      });
}

function throttle(func, limit) {
    let lastFunc, lastRan;
    return function(...args) {
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
    // Don't activate if clicking emergency stop button
    if (e.target.closest('.emergency-stop-overlay')) return;
    
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
cameraStream.onload = function() {
    if (!cameraConnected) {
        cameraConnected = true;
        console.log('Camera stream connected');
    }
};

cameraStream.onerror = function() {
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

window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected:", e.gamepad);
    startGamepadPolling();
});
/* ===== Video overlay helpers ========================================== */
const vinSpan = document.getElementById('vin-display');
const rpmSpan = document.getElementById('rpm-display');

const VOLTAGE_THRESHOLD = 24;

function updateVoltage(volts){
  vinSpan.textContent = volts.toFixed(1) + ' V';
  if (volts < VOLTAGE_THRESHOLD) {
    vinSpan.classList.add('warning');
  } else {
    vinSpan.classList.remove('warning');
  }
}

function updateRPM(rpm){
  rpmSpan.textContent = Math.round(rpm) + ' RPM';
}


/* ===== Live voltage overlay =========================================== */
(() => {
    if (!vinSpan) return;

    // first snapshot
    fetch('/api/voltage')
        .then(r => r.ok ? r.json() : null)
        .then(j => j?.success && updateVoltage(j.vin))
        .catch(() => {});

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
        .catch(() => {});

    // live updates via SSE
    const es = new EventSource('/stream/rpm');
    es.onmessage = e => {
        const r = parseFloat(e.data);
        if (isFinite(r)) updateRPM(r);
    };
    es.onerror = () =>
        console.error('RPM SSE error – browser will auto-retry');
})();
