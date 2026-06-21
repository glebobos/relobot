import { rosService } from './ros-service.js';
import { CFG, TOPICS, MSG_TYPES } from '../shared/constants.js';

class GamepadService {
    constructor() {
        this.cmdVelTopic = null;
        this.knifeRpmTopic = null;
        this.knifeCurrentRpm = 0.0;
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
        this.cmdVelTopic = rosService.createTopicV2(TOPICS.CMD_VEL, MSG_TYPES.TWIST);
        this.knifeRpmTopic = rosService.createTopicV2(TOPICS.CMD_KNIVES, MSG_TYPES.FLOAT32);

        // Publish knife RPM at a fixed interval (200ms)
        this.rpmPublishInterval = setInterval(() => this.publishRpm(this.knifeCurrentRpm), 200);

        this.gamepadConnectedHandler = (e) => {
            console.log('[GamepadService] Connected:', e.gamepad.id);
            this.startGamepadPolling();
        };

        window.addEventListener('gamepadconnected', this.gamepadConnectedHandler);
    }

    publishTwist(linear, angular) {
        if (!this.cmdVelTopic) return;
        this.cmdVelTopic.publish({
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
        });
    }

    publishRpm(rpm) {
        if (!this.knifeRpmTopic) return;
        this.knifeRpmTopic.publish({ data: rpm });
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
        this.knifeCurrentRpm = rpm;
        if (overrideCruise) {
            this.cruiseActive = false;
        }
        if (this.onKnifeUpdateCallback) {
            this.onKnifeUpdateCallback(this.knifeCurrentRpm);
        }
    }

    processKnife(knifeVal, cruiseBtnVal) {
        const rawRpm = knifeVal <= 0.05
            ? 0.0
            : CFG.knifeMinRpm + knifeVal * (CFG.knifeMaxRpm - CFG.knifeMinRpm);
        const currentRpm = (CFG.invertKnife && rawRpm !== 0) ? -rawRpm : rawRpm;
        const knifeIsActive = knifeVal > 0.05;
        const cruiseBtnIsPressed = cruiseBtnVal > 0.5;
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

        if (now - this.lastGamepadSent > 100) {
            const isZeroNow = this.isAllZero(processedAxes, buttons);
            if (!isZeroNow || (isZeroNow && this.lastSentNonZero)) {
                const turn = processedAxes[CFG.turnAxis] ?? 0;
                const drive = processedAxes[CFG.driveAxis] ?? 0;
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

    destroy() {
        window.removeEventListener('gamepadconnected', this.gamepadConnectedHandler);
        if (this.pollingInterval) {
            clearInterval(this.pollingInterval);
            this.pollingInterval = null;
        }
        if (this.rpmPublishInterval) {
            clearInterval(this.rpmPublishInterval);
            this.rpmPublishInterval = null;
        }
    }
}

export const gamepadService = new GamepadService();
