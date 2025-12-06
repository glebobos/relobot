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

// Joystick functionality - overlay on video
const videoContainer = document.getElementById('videoContainer');
const joystick = document.getElementById('joystick');
const stick = document.getElementById('stick');
const speedSlider = document.getElementById('speed-sensitivity');
const turnSlider = document.getElementById('turn-sensitivity');
const speedValue = document.getElementById('speed-value');
const turnValue = document.getElementById('turn-value');
let isActive = false;
let joystickOriginX = 0;
let joystickOriginY = 0;
const JOYSTICK_RADIUS = 75; // Half of 150px joystick size
const STICK_RADIUS = 25;    // Half of 50px stick size
const MAX_STICK_DISTANCE = JOYSTICK_RADIUS - STICK_RADIUS;

speedSlider.addEventListener('input', () => {
    speedValue.textContent = speedSlider.value + '%';
});

turnSlider.addEventListener('input', () => {
    turnValue.textContent = turnSlider.value + '%';
});

function showJoystickAt(x, y) {
    const containerRect = videoContainer.getBoundingClientRect();
    const relX = x - containerRect.left;
    const relY = y - containerRect.top;
    
    joystick.style.left = relX + 'px';
    joystick.style.top = relY + 'px';
    joystick.style.display = 'block';
    
    joystickOriginX = x;
    joystickOriginY = y;
    
    // Reset stick to center
    stick.style.left = '50%';
    stick.style.top = '50%';
    
    // Trigger fade-in transition
    requestAnimationFrame(() => {
        stick.classList.add('visible');
    });
}

function hideJoystick() {
    stick.classList.remove('visible');
    // Wait for transition to finish before hiding
    setTimeout(() => {
        if (!isActive) {
            joystick.style.display = 'none';
        }
    }, 200);
}

function setStickPosition(x, y) {
    let dx = x - joystickOriginX;
    let dy = y - joystickOriginY;
    
    const distance = Math.sqrt(dx * dx + dy * dy);
    if (distance > MAX_STICK_DISTANCE) {
        dx *= MAX_STICK_DISTANCE / distance;
        dy *= MAX_STICK_DISTANCE / distance;
    }
    
    // Position stick relative to joystick center
    stick.style.left = `calc(50% + ${dx}px)`;
    stick.style.top = `calc(50% + ${dy}px)`;
    
    let normalizedX = dx / MAX_STICK_DISTANCE;
    let normalizedY = -dy / MAX_STICK_DISTANCE;

    normalizedX *= turnSlider.value / 100;
    normalizedY *= speedSlider.value / 100;
    
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
    const touch = e.touches ? e.touches[0] : e;
    isActive = true;
    showJoystickAt(touch.clientX, touch.clientY);
    e.preventDefault();
}

function handleMove(e) {
    if (!isActive) return;
    const touch = e.touches ? e.touches[0] : e;
    setStickPosition(touch.clientX, touch.clientY);
    e.preventDefault();
}

function handleEnd() {
    if (!isActive) return;
    isActive = false;
    hideJoystick();
    throttleUpdateMotors(0, 0);
}

// Event listeners - on video container
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
/* ===== Status-bar helpers ============================================ */
const vinSpan = document.getElementById('vin-display');   // “26.4 V”
const rpmSpan = document.getElementById('rpm-display');   // “1250 RPM”

const vinBar  = document.getElementById('vin-bar');       // outer bar
const rpmBar  = document.getElementById('rpm-bar');

function clamp01(x){ return Math.max(0, Math.min(1, x)); }

function updateVoltage(volts, minV = 24, maxV = 28){
  const fill = clamp01((volts - minV) / (maxV - minV));
  vinBar.style.setProperty('--fill', fill.toFixed(3));
  vinSpan.textContent = volts.toFixed(2) + ' V';
}

function updateRPM(rpm, maxRPM = 3100){
  const fill = clamp01(rpm / maxRPM);
  rpmBar.style.setProperty('--fill', fill.toFixed(3));
  rpmSpan.textContent = Math.round(rpm) + ' RPM';
}


/* ===== Live battery-voltage bar ======================================= */
(() => {
    if (!vinBar) return;          // bar not on this page

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

/* ===== Live RPM bar =================================================== */
(() => {
    if (!rpmBar) return;
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
