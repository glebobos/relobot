import { gamepadService } from '../services/gamepad-service.js';

export class VirtualJoystick {
    constructor(containerSelector) {
        this.videoContainer = document.querySelector(containerSelector);
        this.isActive = false;
        this.startX = 0;
        this.startY = 0;
        this.controlRadius = 80; // pixels -> full speed

        // Bind event handlers
        this.startHandler = (e) => this.handleStart(e);
        this.moveHandler = (e) => this.handleMove(e);
        this.endHandler = () => this.handleEnd();

        this.init();
    }

    init() {
        if (!this.videoContainer) return;

        // Throttled function to update motors via GamepadService
        this.throttleUpdateMotors = this.throttle((x, y) => {
            const { linear, angular } = gamepadService.computeTwist(x, y);
            gamepadService.publishTwist(linear, angular);
        }, 100);

        this.videoContainer.addEventListener('mousedown', this.startHandler);
        this.videoContainer.addEventListener('touchstart', this.startHandler, { passive: false });
        document.addEventListener('mousemove', this.moveHandler);
        document.addEventListener('touchmove', this.moveHandler, { passive: false });
        document.addEventListener('mouseup', this.endHandler);
        document.addEventListener('touchend', this.endHandler);
    }

    throttle(func, limit) {
        let lastFunc, lastRan;
        return function (...args) {
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

    handleStart(e) {
        if (!this.videoContainer.contains(e.target) && e.target !== this.videoContainer) return;
        
        // Don't activate touch-joystick when interacting with overlays
        const knifeOverlay = document.getElementById('knifeControlOverlay');
        if (knifeOverlay && knifeOverlay.contains(e.target)) return;
        
        const verticalSlider = document.querySelector('.vertical-slider-container');
        if (verticalSlider && verticalSlider.contains(e.target)) return;

        const stopBtn = document.getElementById('camera-stop-btn');
        if (stopBtn && stopBtn.contains(e.target)) return;

        const pipMap = document.getElementById('pipMap');
        if (pipMap && pipMap.contains(e.target)) return;

        this.isActive = true;
        const touch = e.touches ? e.touches[0] : e;
        this.startX = touch.clientX;
        this.startY = touch.clientY;
        e.preventDefault();
    }

    handleMove(e) {
        if (!this.isActive) return;
        const touch = e.touches ? e.touches[0] : e;
        const dx = touch.clientX - this.startX;
        const dy = touch.clientY - this.startY;
        const normX = Math.max(-1, Math.min(1, dx / this.controlRadius));
        const normY = Math.max(-1, Math.min(1, dy / this.controlRadius));
        this.throttleUpdateMotors(normX, normY);
        e.preventDefault();
    }

    handleEnd() {
        if (!this.isActive) return;
        this.isActive = false;
        this.throttleUpdateMotors(0, 0);
    }

    destroy() {
        if (this.videoContainer) {
            this.videoContainer.removeEventListener('mousedown', this.startHandler);
            this.videoContainer.removeEventListener('touchstart', this.startHandler);
        }
        document.removeEventListener('mousemove', this.moveHandler);
        document.removeEventListener('touchmove', this.moveHandler);
        document.removeEventListener('mouseup', this.endHandler);
        document.removeEventListener('touchend', this.endHandler);
    }
}
