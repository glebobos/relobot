import { gamepadService } from '../services/gamepad-service.js';

export class VirtualJoystick {
    constructor(containerSelector) {
        this.videoContainer = document.querySelector(containerSelector);
        this.isActive = false;
        this.startX = 0;
        this.startY = 0;
        this.controlRadius = 80; // pixels → full speed

        // DOM elements for the joystick visual
        this.overlay = null;
        this.base = null;
        this.knob = null;

        // Bind event handlers
        this.startHandler = (e) => this.handleStart(e);
        this.moveHandler  = (e) => this.handleMove(e);
        this.endHandler   = ()  => this.handleEnd();

        this.init();
    }

    /* ── Lifecycle ─────────────────────────────────────────────────────── */

    init() {
        if (!this.videoContainer) return;

        // Throttled function to update motors via GamepadService
        this.throttleUpdateMotors = this.throttle((x, y) => {
            const { linear, angular } = gamepadService.computeTwist(x, y);
            gamepadService.publishTwist(linear, angular);
        }, 100);

        this.videoContainer.addEventListener('mousedown',  this.startHandler);
        this.videoContainer.addEventListener('touchstart', this.startHandler, { passive: false });
        document.addEventListener('mousemove',  this.moveHandler);
        document.addEventListener('touchmove',  this.moveHandler, { passive: false });
        document.addEventListener('mouseup',    this.endHandler);
        document.addEventListener('touchend',   this.endHandler);
    }

    destroy() {
        if (this.videoContainer) {
            this.videoContainer.removeEventListener('mousedown',  this.startHandler);
            this.videoContainer.removeEventListener('touchstart', this.startHandler);
        }
        document.removeEventListener('mousemove',  this.moveHandler);
        document.removeEventListener('touchmove',  this.moveHandler);
        document.removeEventListener('mouseup',    this.endHandler);
        document.removeEventListener('touchend',   this.endHandler);
        this._removeVisual();
    }

    /* ── Joystick visual ───────────────────────────────────────────────── */

    _createVisual(x, y) {
        // Outer overlay (position anchor)
        this.overlay = document.createElement('div');
        this.overlay.className = 'c-joystick-overlay';
        this.overlay.style.left = `${x}px`;
        this.overlay.style.top  = `${y}px`;

        // Base ring — stays fixed at touch origin
        this.base = document.createElement('div');
        this.base.className = 'c-joystick-base';

        // Inner knob — starts centered at (0, 0) relative to parent
        this.knob = document.createElement('div');
        this.knob.className = 'c-joystick-knob';

        this.overlay.appendChild(this.base);
        this.overlay.appendChild(this.knob);
        document.body.appendChild(this.overlay);
    }

    _updateKnob(dx, dy) {
        // Clamp knob within the control radius
        const dist  = Math.sqrt(dx * dx + dy * dy);
        const limit = this.controlRadius;
        const scale = dist > limit ? limit / dist : 1;
        
        const targetX = dx * scale;
        const targetY = dy * scale;
        
        // Translate knob relative to touch center (0, 0)
        this.knob.style.transform = `translate(calc(-50% + ${targetX}px), calc(-50% + ${targetY}px))`;
    }

    _removeVisual() {
        if (this.overlay) {
            this.overlay.remove();
            this.overlay = null;
            this.base    = null;
            this.knob    = null;
        }
    }

    /* ── Event handlers ────────────────────────────────────────────────── */

    handleStart(e) {
        if (!this.videoContainer.contains(e.target) && e.target !== this.videoContainer) return;

        // Don't activate when interacting with overlays / controls
        const guards = [
            document.getElementById('knifeControlOverlay'),
            document.getElementById('knifeSlider'),
            document.querySelector('.c-rpm-widget__slider-dropdown'),
            document.querySelector('.c-camera-view__slider-container'),
            document.getElementById('camera-stop-btn'),
            document.getElementById('pipMap'),
        ];
        if (guards.some(el => el && el.contains(e.target))) return;

        this.isActive = true;
        const touch   = e.touches ? e.touches[0] : e;
        this.startX   = touch.clientX;
        this.startY   = touch.clientY;

        this._createVisual(this.startX, this.startY);
        e.preventDefault();
    }

    handleMove(e) {
        if (!this.isActive) return;
        const touch  = e.touches ? e.touches[0] : e;
        const dx     = touch.clientX - this.startX;
        const dy     = touch.clientY - this.startY;
        const normX  = Math.max(-1, Math.min(1, dx / this.controlRadius));
        const normY  = Math.max(-1, Math.min(1, dy / this.controlRadius));

        this._updateKnob(dx, dy);
        this.throttleUpdateMotors(normX, normY);
        e.preventDefault();
    }

    handleEnd() {
        if (!this.isActive) return;
        this.isActive = false;
        this._removeVisual();
        this.throttleUpdateMotors(0, 0);
    }

    /* ── Utilities ─────────────────────────────────────────────────────── */

    throttle(func, limit) {
        let timeout = null;
        let lastRan = 0;
        let lastArgs = null;
        let lastContext = null;

        const throttled = function (...args) {
            const now = Date.now();
            lastArgs = args;
            lastContext = this;

            const run = () => {
                lastRan = Date.now();
                timeout = null;
                func.apply(lastContext, lastArgs);
            };

            // If linear and angular are both 0 (stop command), bypass throttle and execute immediately!
            if (args[0] === 0 && args[1] === 0) {
                if (timeout) {
                    clearTimeout(timeout);
                    timeout = null;
                }
                run();
                return;
            }

            const remaining = limit - (now - lastRan);
            if (remaining <= 0 || remaining > limit) {
                if (timeout) {
                    clearTimeout(timeout);
                    timeout = null;
                }
                run();
            } else if (!timeout) {
                timeout = setTimeout(run, remaining);
            }
        };
        return throttled;
    }
}
