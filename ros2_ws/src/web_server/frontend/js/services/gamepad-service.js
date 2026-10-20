import { rosService } from './ros-service.js';
import { CFG, TOPICS, MSG_TYPES } from '../shared/constants.js';

class GamepadService {
    constructor() {
        this.cmdVelTopic = null;
        this.knifeRpmTopic = null;
        this.knifeCurrentRpm = 0.0;
        this.knifeReleaseRequired = true;
        this.knifeStopPending = true;
        this.driveReleaseRequired = true;
        this.cruiseActive = false;
        this.cruiseRpm = 0.0;
        this.cruiseBtnWasPressed = false;
        this.lastGamepadSent = 0;
        this.lastSentNonZero = false;
        this.onKnifeUpdateCallback = null;
        this.pollingInterval = null;

        this.init();
    }

    init() {
        // Create topics
        this.cmdVelTopic = rosService.createTopic(TOPICS.CMD_VEL, MSG_TYPES.TWIST_STAMPED);
        this.knifeRpmTopic = rosService.createTopic(TOPICS.CMD_KNIVES, MSG_TYPES.FLOAT32, {
            reconnect_on_close: false,
            queue_size: 1,
        });

        this.rosConnectionHandler = () => this.stopMotion();
        for (const event of ['connection', 'error', 'close']) {
            rosService.ros.on(event, this.rosConnectionHandler);
        }

        // Publish knife RPM at a fixed interval (200ms)
        this.rpmPublishInterval = setInterval(() => this.publishRpm(this.knifeCurrentRpm), 200);

        this.gamepadConnectedHandler = (e) => {
            console.log('[GamepadService] Connected:', e.gamepad.id);
            this.startGamepadPolling();
        };
        this.gamepadDisconnectedHandler = (e) => {
            console.warn('[GamepadService] Disconnected:', e.gamepad.id);
            this.stopMotion();
            if (!navigator.getGamepads?.().some(Boolean) && this.pollingInterval) {
                clearInterval(this.pollingInterval);
                this.pollingInterval = null;
            }
        };
        this.blurHandler = () => this.stopMotion();
        this.visibilityHandler = () => {
            if (document.visibilityState !== 'visible') this.stopMotion();
        };

        window.addEventListener('gamepadconnected', this.gamepadConnectedHandler);
        window.addEventListener('gamepaddisconnected', this.gamepadDisconnectedHandler);
        window.addEventListener('blur', this.blurHandler);
        document.addEventListener('visibilitychange', this.visibilityHandler);

        if (navigator.getGamepads?.().some(Boolean)) {
            this.startGamepadPolling();
        }
    }

    publishTwist(linear, angular) {
        if (!this.cmdVelTopic || !rosService.isConnected) return;
        this.cmdVelTopic.publish({
            header: {
                frame_id: 'base_link',
                stamp: { sec: 0, nanosec: 0 },
            },
            twist: {
                linear: { x: linear, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angular },
            },
        });
    }

    publishRpm(rpm) {
        if (!this.knifeRpmTopic || !rosService.isConnected || !Number.isFinite(rpm)) return;
        if (rpm !== 0 && document.visibilityState !== 'visible') return;
        this.knifeRpmTopic.publish({ data: this.knifeStopPending ? 0 : rpm });
        this.knifeStopPending = false;
    }

    computeTwist(turn, drive) {
        const linear = (CFG.invertDrive ? -drive : drive) * CFG.scaleLinear;
        const angular = (CFG.invertTurn ? -turn : turn) * CFG.scaleAngular;
        return { linear, angular };
    }

    applyDeadZone(axisIndex, value) {
        const disabledAxes = new Set([1, 2]);
        if (disabledAxes.has(axisIndex)) return 0.0;
        const idleZoneConfig = { 0: 0.07, 3: 0.07 };
        const threshold = idleZoneConfig[axisIndex] ?? 0.0;
        if (Math.abs(value) < threshold) return 0.0;
        const sign = value > 0 ? 1 : -1;
        return ((Math.abs(value) - threshold) / (1 - threshold)) * sign;
    }

    isAllZero(axes, buttons) {
        return axes.every(v => Math.abs(v) < 1e-5) && buttons.every(v => Math.abs(v) < 1e-5);
    }

    setKnifeRpm(rpm, overrideCruise = false) {
        const canOperate = rosService.isConnected && document.visibilityState === 'visible';
        if (!Number.isFinite(rpm) || !canOperate) rpm = 0;
        this.knifeCurrentRpm = rpm;
        if (overrideCruise) {
            this.cruiseActive = false;
            this.cruiseRpm = 0;
            if (canOperate) this.knifeReleaseRequired = false;
        }
        if (this.onKnifeUpdateCallback) {
            this.onKnifeUpdateCallback(this.knifeCurrentRpm);
        }
    }

    processKnife(knifeVal, cruiseBtnVal) {
        if (!Number.isFinite(knifeVal)) knifeVal = 0;
        if (!Number.isFinite(cruiseBtnVal)) cruiseBtnVal = 0;
        const rawRpm = knifeVal <= 0.05
            ? 0.0
            : CFG.knifeMinRpm + knifeVal * (CFG.knifeMaxRpm - CFG.knifeMinRpm);
        const currentRpm = (CFG.invertKnife && rawRpm !== 0) ? -rawRpm : rawRpm;
        const knifeIsActive = knifeVal > 0.05;
        const cruiseBtnIsPressed = cruiseBtnVal > 0.5;
        if (!rosService.isConnected || document.visibilityState !== 'visible') return;
        if (this.knifeReleaseRequired) {
            if (!knifeIsActive && !cruiseBtnIsPressed) {
                this.knifeReleaseRequired = false;
                this.cruiseBtnWasPressed = false;
            }
            return;
        }
        const justPressed = cruiseBtnIsPressed && !this.cruiseBtnWasPressed;
        this.cruiseBtnWasPressed = cruiseBtnIsPressed;

        if (justPressed) {
            if (!this.cruiseActive) {
                this.cruiseRpm = currentRpm;
                this.cruiseActive = true;
                console.log(`[GamepadService] Cruise ON @ ${this.cruiseRpm.toFixed(0)} RPM`);
            } else {
                this.cruiseActive = false;
                console.log('[GamepadService] Cruise OFF');
            }
        }

        if (this.cruiseActive && knifeIsActive && !cruiseBtnIsPressed) {
            this.cruiseActive = false;
            console.log('[GamepadService] Cruise reset – knife button pressed');
        }

        const newRpm = this.cruiseActive ? this.cruiseRpm : currentRpm;
        if (Math.abs(this.knifeCurrentRpm - newRpm) > 1e-3) {
            this.setKnifeRpm(newRpm);
        }
    }

    pollGamepad() {
        const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
        const gp = gamepads[0];
        if (!gp) return;

        const processedAxes = gp.axes.map((val, idx) => this.applyDeadZone(idx, val));
        const buttons = gp.buttons.map(b => b.value);
        const now = Date.now();

        this.processKnife(buttons[CFG.knifeButton] ?? 0, buttons[CFG.cruiseButton] ?? 0);

        const turn = processedAxes[CFG.turnAxis] ?? 0;
        const drive = processedAxes[CFG.driveAxis] ?? 0;

        if (this.driveReleaseRequired) {
            if (Math.abs(turn) < 1e-5 && Math.abs(drive) < 1e-5) {
                this.driveReleaseRequired = false;
            }
            return;
        }

        if (now - this.lastGamepadSent > 100) {
            const isZeroNow = this.isAllZero(processedAxes, buttons);
            if (!isZeroNow || (isZeroNow && this.lastSentNonZero)) {
                const { linear, angular } = this.computeTwist(turn, drive);
                this.publishTwist(linear, angular);
                this.lastSentNonZero = !isZeroNow;
            }
            this.lastGamepadSent = now;
        }
    }

    startGamepadPolling() {
        if (this.pollingInterval) return;
        this.pollingInterval = setInterval(() => this.pollGamepad(), 20);
    }

    stopMotion() {
        this.lastSentNonZero = false;
        this.cruiseActive = false;
        this.cruiseRpm = 0;
        this.knifeReleaseRequired = true;
        this.knifeStopPending = true;
        this.driveReleaseRequired = true;
        this.cruiseBtnWasPressed = false;
        this.setKnifeRpm(0);
        this.publishTwist(0, 0);
        this.publishRpm(0);
    }

    destroy() {
        this.stopMotion();
        for (const event of ['connection', 'error', 'close']) {
            rosService.ros.removeListener(event, this.rosConnectionHandler);
        }
        window.removeEventListener('gamepadconnected', this.gamepadConnectedHandler);
        window.removeEventListener('gamepaddisconnected', this.gamepadDisconnectedHandler);
        window.removeEventListener('blur', this.blurHandler);
        document.removeEventListener('visibilitychange', this.visibilityHandler);
        if (this.pollingInterval) {
            clearInterval(this.pollingInterval);
            this.pollingInterval = null;
        }
        if (this.rpmPublishInterval) {
            clearInterval(this.rpmPublishInterval);
            this.rpmPublishInterval = null;
        }
        [this.cmdVelTopic, this.knifeRpmTopic].forEach((topic) => {
            try { topic?.unadvertise(); } catch (_error) { }
        });
        this.cmdVelTopic = null;
        this.knifeRpmTopic = null;
    }
}

export const gamepadService = new GamepadService();
