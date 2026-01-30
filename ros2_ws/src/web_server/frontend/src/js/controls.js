const IDLE_ZONE_CONFIG = { 0: 0.07, 3: 0.07 };
const DISABLED_AXES = new Set([1, 2]);

function applyDeadZone(axisIndex, value) {
    if (DISABLED_AXES.has(axisIndex)) return 0.0;
    const threshold = IDLE_ZONE_CONFIG[axisIndex] ?? 0.0;
    if (Math.abs(value) < threshold) return 0.0;
    const sign = value > 0 ? 1 : -1;
    return ((Math.abs(value) - threshold) / (1 - threshold)) * sign;
}

export function initGamepad() {
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
                }).catch(() => {});
                lastSentNonZero = !isZeroNow;
            }
            lastGamepadSent = now;
        }
    }

    window.addEventListener("gamepadconnected", function (e) {
        console.log("[Gamepad] Connected:", e.gamepad.id);
        setInterval(pollGamepad, 20);
    });
}
