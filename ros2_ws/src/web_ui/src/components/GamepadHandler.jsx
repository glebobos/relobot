import React, { useEffect } from 'react';

const GamepadHandler = () => {
    useEffect(() => {
        const IDLE_ZONE_CONFIG = { 0: 0.07, 3: 0.07 };
        const DISABLED_AXES = new Set([1, 2]);

        const applyDeadZone = (axisIndex, value) => {
            if (DISABLED_AXES.has(axisIndex)) return 0.0;
            const threshold = IDLE_ZONE_CONFIG[axisIndex] ?? 0.0;
            if (Math.abs(value) < threshold) return 0.0;
            const sign = value > 0 ? 1 : -1;
            return ((Math.abs(value) - threshold) / (1 - threshold)) * sign;
        };

        let lastGamepadSent = 0;
        let lastSentNonZero = false;
        let intervalId = null;

        const isZeroOrAllFalse = (axes, buttons) => {
            return axes.every(val => Math.abs(val) < 1e-5) && buttons.every(val => Math.abs(val) < 1e-5);
        };

        const pollGamepad = () => {
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
        };

        const onGamepadConnected = (e) => {
            console.log("Gamepad connected:", e.gamepad);
            if (!intervalId) {
                intervalId = setInterval(pollGamepad, 20);
            }
        };

        const onGamepadDisconnected = () => {
             // Optional: Stop polling if no gamepads
        }

        window.addEventListener("gamepadconnected", onGamepadConnected);
        window.addEventListener("gamepaddisconnected", onGamepadDisconnected);

        // Check if gamepad is already connected
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        if (gamepads[0]) {
             if (!intervalId) {
                intervalId = setInterval(pollGamepad, 20);
            }
        }

        return () => {
            window.removeEventListener("gamepadconnected", onGamepadConnected);
            window.removeEventListener("gamepaddisconnected", onGamepadDisconnected);
            if (intervalId) clearInterval(intervalId);
        };
    }, []);

    return null; // This component doesn't render anything
};

export default GamepadHandler;
