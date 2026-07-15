import nipplejs from 'nipplejs';
import { gamepadService } from '../services/gamepad-service.js';

export class VirtualJoystick {
    constructor(containerSelector) {
        this.videoContainer = document.querySelector(containerSelector);
        this.manager = null;
        this.throttlePublish = null;
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
            '#pipMap'
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

        // Throttle joystick command publisher (100ms) with immediate stop logic
        this.throttlePublish = this.throttle((x, y) => {
            const { linear, angular } = gamepadService.computeTwist(x, y);
            gamepadService.publishTwist(linear, angular);
        }, 100);

        this.manager = nipplejs.create({
            zone: this.videoContainer,
            mode: 'dynamic',
            size: 120, // 120px diameter (radius = 60px)
            color: 'white',
            catchDistance: 80
        });

        this.manager.on('move', (evt, data) => {
            if (data && data.position && data.instance) {
                const dx = data.position.x - data.instance.position.x;
                const dy = data.position.y - data.instance.position.y;
                const radius = 60; // size/2
                const normX = Math.max(-1, Math.min(1, dx / radius));
                const normY = Math.max(-1, Math.min(1, dy / radius));
                this.throttlePublish(normX, normY);
            }
        });

        this.manager.on('end', () => {
            this.throttlePublish(0, 0);
        });
    }

    destroy() {
        if (this.throttlePublish?.cancel) {
            this.throttlePublish.cancel();
        }
        gamepadService.publishTwist(0, 0);
        if (this.manager) {
            this.manager.destroy();
            this.manager = null;
        }
        this.guardHandlers?.forEach(({ el, mouseHandler, touchHandler }) => {
            el.removeEventListener('mousedown', mouseHandler);
            el.removeEventListener('touchstart', touchHandler);
        });
        this.guardHandlers = [];
    }

    throttle(func, limit) {
        let lastRan = 0;
        let timeout = null;
        const throttled = function(...args) {
            const now = Date.now();
            const run = () => {
                lastRan = Date.now();
                func(...args);
            };
            // Publish stop commands immediately
            if (args[0] === 0 && args[1] === 0) {
                if (timeout) clearTimeout(timeout);
                run();
                return;
            }
            if (now - lastRan >= limit) {
                run();
            } else if (!timeout) {
                timeout = setTimeout(() => {
                    timeout = null;
                    run();
                }, limit - (now - lastRan));
            }
        };
        throttled.cancel = () => {
            if (timeout) clearTimeout(timeout);
            timeout = null;
        };
        return throttled;
    }
}
