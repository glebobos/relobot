import { gamepadService } from '../../services/gamepad-service.js';
import { DomListenerBag } from '../../shared/dom-listener-bag.js';

export class HeaderWidgetsController {
    constructor() {
        this.listeners = new DomListenerBag();
        this.knifeSlider = document.getElementById('knifeSlider');
        this.knifeSliderValue = document.getElementById('knifeSliderValue');
        this.batteryWidget = document.getElementById('headerBatteryWidget');
        this.batteryDropdown = document.getElementById('batteryDockDropdown');
        this.rpmWidget = document.getElementById('headerRpmWidget');
        this.rpmDropdown = document.getElementById('knivesSliderDropdown');
        this.initialize();
    }

    initialize() {
        if (this.knifeSlider) {
            this.listeners.add(this.knifeSlider, 'input', () => {
                const rpm = Number.parseInt(this.knifeSlider.value, 10) || 0;
                gamepadService.setKnifeRpm(rpm, true);
                this.renderRpm(rpm);
            });
            this.listeners.add(this.knifeSlider, 'touchstart', event => event.stopPropagation(), { passive: true });
            this.listeners.add(this.knifeSlider, 'mousedown', event => event.stopPropagation());
            this.renderRpm(Number.parseInt(this.knifeSlider.value, 10) || 0);
        }
        gamepadService.onKnifeUpdateCallback = rpm => this.renderRpm(Math.abs(rpm));

        if (this.batteryWidget && this.batteryDropdown) {
            this.listeners.add(this.batteryWidget, 'click', (event) => {
                if (!this.batteryDropdown.contains(event.target)) {
                    this.batteryWidget.classList.toggle('is-dropdown-open');
                }
            });
            this.listeners.add(this.batteryDropdown, 'click', (event) => {
                event.stopPropagation();
                if (this.closeTimer) clearTimeout(this.closeTimer);
                this.closeTimer = setTimeout(
                    () => this.batteryWidget.classList.remove('is-dropdown-open'),
                    300,
                );
            });
        }
        if (this.rpmWidget && this.rpmDropdown) {
            this.listeners.add(this.rpmWidget, 'click', (event) => {
                if (!this.knifeSlider?.contains(event.target)) {
                    this.rpmWidget.classList.toggle('is-dropdown-open');
                }
            });
        }
        this.listeners.add(document, 'pointerdown', (event) => {
            if (this.batteryWidget && !this.batteryWidget.contains(event.target)) {
                this.batteryWidget.classList.remove('is-dropdown-open');
            }
            if (this.rpmWidget && !this.rpmWidget.contains(event.target)) {
                this.rpmWidget.classList.remove('is-dropdown-open');
            }
        }, { passive: true });
    }

    renderRpm(rpm) {
        const normalized = Math.max(0, Math.min(3000, Number(rpm) || 0));
        if (this.knifeSlider) {
            this.knifeSlider.value = normalized;
            const percent = normalized / 30;
            this.knifeSlider.style.background = [
                'linear-gradient(to right,',
                'var(--color-green) 0%,',
                `var(--color-green) ${percent}%,`,
                `rgba(255, 255, 255, 0.15) ${percent}%,`,
                'rgba(255, 255, 255, 0.15) 100%)',
            ].join(' ');
        }
        if (this.knifeSliderValue) this.knifeSliderValue.textContent = Math.round(normalized);
    }

    destroy() {
        this.listeners.destroy();
        if (this.closeTimer) clearTimeout(this.closeTimer);
        this.closeTimer = null;
        if (gamepadService.onKnifeUpdateCallback) gamepadService.onKnifeUpdateCallback = null;
    }
}
