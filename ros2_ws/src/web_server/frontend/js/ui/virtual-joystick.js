import nipplejs from 'nipplejs';
import { gamepadService } from '../services/gamepad-service.js';

export class VirtualJoystick {
    constructor(containerSelector) {
        this.videoContainer = document.querySelector(containerSelector);
        this.manager = null;
        this.isHeld = false;
        this.currentX = 0;
        this.currentY = 0;
        this.publishInterval = null;
        this.lastPublishTime = 0;
        this.publishRateMs = 100; // 10 Hz continuous heartbeat matches ROS watchdog expectations
        this.guardHandlers = [];
        this.blurHandler = null;
        this.visibilityHandler = null;
        this.init();
    }

    init() {
        if (!this.videoContainer) return;
        this.guardHandlers = [];

        // Install event stop-propagation guards on camera HUD overlays to prevent joystick activation
        const guards = [
            '#knifeControlOverlay',
            '#knifeSlider',
            '.c-rpm-widget__slider-dropdown',
            '.c-camera-view__slider-container',
            '#camera-stop-btn',
            '#pipMap',
            '#camera-chat-toggle-btn',
            '#floatingChatWidget'
        ];
        guards.forEach(selector => {
            const el = document.querySelector(selector);
            if (el) {
                const mouseHandler = (e) => e.stopPropagation();
                const touchHandler = (e) => e.stopPropagation();
                el.addEventListener('mousedown', mouseHandler);
                el.addEventListener('touchstart', touchHandler, { passive: true });
                this.guardHandlers.push({ el, mouseHandler, touchHandler });
            }
        });

        // Window blur and visibility change guards to guarantee immediate stop on tab switch / phone call
        this.blurHandler = () => this.stop();
        this.visibilityHandler = () => {
            if (document.visibilityState !== 'visible') this.stop();
        };
        window.addEventListener('blur', this.blurHandler);
        document.addEventListener('visibilitychange', this.visibilityHandler);

        this.manager = nipplejs.create({
            zone: this.videoContainer,
            mode: 'dynamic',
            size: 120, // 120px diameter (radius = 60px)
            color: 'white',
            catchDistance: 80
        });

        this.manager.on('start', () => {
            this.isHeld = true;
            this.currentX = 0;
            this.currentY = 0;
            this.startPublishLoop();
        });

        this.manager.on('move', (evt, data) => {
            if (data && data.position && data.instance) {
                const dx = data.position.x - data.instance.position.x;
                const dy = data.position.y - data.instance.position.y;
                const radius = 60; // size/2
                this.currentX = Math.max(-1, Math.min(1, dx / radius));
                this.currentY = Math.max(-1, Math.min(1, dy / radius));
                this.isHeld = true;

                // Ensure heartbeat publish loop is active
                this.startPublishLoop();

                // Responsive immediate publish on finger move if at least 50ms elapsed
                const now = performance.now();
                if (now - this.lastPublishTime >= 50) {
                    this.sendCurrentTwist();
                }
            }
        });

        this.manager.on('end', () => {
            this.stop();
        });
    }

    sendCurrentTwist() {
        this.lastPublishTime = performance.now();
        const { linear, angular } = gamepadService.computeTwist(this.currentX, this.currentY);
        gamepadService.publishTwist(linear, angular);
    }

    startPublishLoop() {
        if (this.publishInterval) return;
        this.publishInterval = setInterval(() => {
            if (this.isHeld) {
                this.sendCurrentTwist();
            } else {
                this.stop();
            }
        }, this.publishRateMs);
    }

    stop() {
        this.isHeld = false;
        this.currentX = 0;
        this.currentY = 0;
        if (this.publishInterval) {
            clearInterval(this.publishInterval);
            this.publishInterval = null;
        }
        gamepadService.publishTwist(0, 0);
    }

    destroy() {
        this.stop();
        if (this.manager) {
            this.manager.destroy();
            this.manager = null;
        }
        this.guardHandlers?.forEach(({ el, mouseHandler, touchHandler }) => {
            el.removeEventListener('mousedown', mouseHandler);
            el.removeEventListener('touchstart', touchHandler);
        });
        this.guardHandlers = [];
        if (this.blurHandler) {
            window.removeEventListener('blur', this.blurHandler);
            this.blurHandler = null;
        }
        if (this.visibilityHandler) {
            document.removeEventListener('visibilitychange', this.visibilityHandler);
            this.visibilityHandler = null;
        }
    }
}
