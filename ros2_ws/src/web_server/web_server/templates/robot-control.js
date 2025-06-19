// Camera switching
const cameraRadios = document.querySelectorAll('input[name="camera"]');
const cameraStream = document.getElementById('cameraStream');

cameraRadios.forEach(radio => {
    radio.addEventListener('change', function() {
        if (this.checked) {
            cameraStream.src = `/video_feed/${this.value}`;
        }
    });
});

// Joystick functionality
const joystick = document.getElementById('joystick');
const stick = document.getElementById('stick');
const speedSlider = document.getElementById('speed-sensitivity');
const turnSlider = document.getElementById('turn-sensitivity');
const speedValue = document.getElementById('speed-value');
const turnValue = document.getElementById('turn-value');
let isActive = false;

speedSlider.addEventListener('input', () => {
    speedValue.textContent = speedSlider.value + '%';
});

turnSlider.addEventListener('input', () => {
    turnValue.textContent = turnSlider.value + '%';
});

function setStickPosition(x, y) {
    const joystickRect = joystick.getBoundingClientRect();
    const stickRect = stick.getBoundingClientRect();
    const radius = (joystickRect.width - stickRect.width) / 2;
    
    const centerX = joystickRect.left + joystickRect.width / 2;
    const centerY = joystickRect.top + joystickRect.height / 2;
    
    let dx = x - centerX;
    let dy = y - centerY;
    
    const distance = Math.sqrt(dx * dx + dy * dy);
    if (distance > radius) {
        dx *= radius / distance;
        dy *= radius / distance;
    }
    
    stick.style.left = `${dx + radius}px`;
    stick.style.top = `${dy + radius}px`;
    
    let normalizedX = dx / radius;
    let normalizedY = -dy / radius;

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
    isActive = true;
    handleMove(e);
}

function handleMove(e) {
    if (!isActive) return;
    const touch = e.touches ? e.touches[0] : e;
    setStickPosition(touch.clientX, touch.clientY);
    e.preventDefault();
}

function handleEnd() {
    isActive = false;
    // Reset to center - calculate center position dynamically
    const joystickRect = joystick.getBoundingClientRect();
    const stickRect = stick.getBoundingClientRect();
    const centerOffset = (joystickRect.width - stickRect.width) / 2;
    
    stick.style.left = `${centerOffset}px`;
    stick.style.top = `${centerOffset}px`;
    throttleUpdateMotors(0, 0);
}

// Event listeners
joystick.addEventListener('mousedown', handleStart);
joystick.addEventListener('touchstart', handleStart);

document.addEventListener('mousemove', handleMove);
document.addEventListener('touchmove', handleMove, { passive: false });

document.addEventListener('mouseup', handleEnd);
document.addEventListener('touchend', handleEnd);

// Video error handling
cameraStream.onerror = function() {
    console.log('Camera stream error, retrying...');
    setTimeout(() => {
        const selectedCamera = document.querySelector('input[name="camera"]:checked').value;
        this.src = `/video_feed/${selectedCamera}?` + new Date().getTime();
    }, 1000);
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

/* ===== Live battery-voltage bar ======================================= */
(() => {
    const vinSpan = document.getElementById('vin-display');
    const mask    = document.getElementById('battery-mask');
    if (!vinSpan || !mask) return;          // element not on page

    const MIN_V = 24.0;
    const MAX_V = 28.0;

    function updateBar(vin) {
        // clamp to [24,28] → 0-100 %
        const pct   = Math.max(0, Math.min(1, (vin - MIN_V) / (MAX_V - MIN_V)));
        mask.style.width = (100 - pct * 100) + '%';
        vinSpan.textContent = vin.toFixed(2);
    }

    // first shot (in case SSE hasn’t fired yet)
    fetch('/api/voltage')
        .then(r => r.ok ? r.json() : null)
        .then(j => j && j.success && updateBar(j.vin))
        .catch(() => {});

    // live updates
    const es = new EventSource('/stream/voltage');
    es.onmessage = e => {
        const v = parseFloat(e.data);
        if (isFinite(v)) updateBar(v);
    };
    es.onerror = () => console.warn('Voltage SSE error – browser will auto-retry');
})();
