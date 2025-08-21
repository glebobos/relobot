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
    console.error('Camera stream error, retrying...');
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

// Map controls
const saveMapButton = document.getElementById('save-map-button');
const explorerToggle = document.getElementById('explorer-toggle');

saveMapButton.addEventListener('click', () => {
    fetch('/api/map/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    }).then(response => response.json())
      .then(data => {
          alert(data.success ? 'Map saving triggered!' : 'Map saving failed: ' + data.error);
      });
});

explorerToggle.addEventListener('change', function() {
    fetch('/api/explorer', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
    });
});

// Tab switching
const tabs = document.querySelectorAll('.nav-links a');
const tabContents = document.querySelectorAll('.tab-content');

tabs.forEach(tab => {
    tab.addEventListener('click', e => {
        const href = tab.getAttribute('href');
        if (href.startsWith('#')) {
            e.preventDefault();
            const target = document.querySelector(href);
            tabContents.forEach(tc => {
                tc.classList.remove('active');
            });
            target.classList.add('active');
            tabs.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
        }
    });
});

// Set default active tab
document.querySelector('.nav-links a[href^="#"]').click();
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
