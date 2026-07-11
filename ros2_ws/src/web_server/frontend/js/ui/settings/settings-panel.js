import { rosService } from '../../services/ros-service.js';
import { TOPICS, MSG_TYPES } from '../../shared/constants.js';
import { safeClear, safeCreateElement } from '../../shared/dom-utils.js';
import { renderNavigationSettings } from './panels/navigation-settings.js';
import { renderSensorsSettings } from './panels/sensors-settings.js';
import { renderWifiSettings } from './panels/wifi-settings.js';
import { renderLogsSettings } from './panels/logs-settings.js';
import { renderSystemSettings } from './panels/system-settings.js';
import { renderCalibrationSettings } from './panels/calibration-settings.js';

export class SettingsPanel {
    constructor(telemetry) {
        this.telemetry = telemetry;
        this.drawer = document.getElementById('settingsDrawer');
        this.drawerTitle = document.getElementById('drawerTitle');
        this.drawerContent = document.getElementById('drawerContent');
        this.backBtn = document.getElementById('drawerBackBtn');
        this.settingsItems = document.querySelectorAll('.c-settings-item');

        // Logs buffer
        this.logBuffer = [];
        this.maxLogs = 50;

        // Subscriptions & active panel cleanups
        this.rosoutSubscription = null;
        this.activePanelCleanup = null;

        this.init();
    }

    init() {
        if (!this.drawer || !this.backBtn) return;

        // Register back button click
        this.backBtn.addEventListener('click', () => {
            this.drawer.classList.remove('is-open');
            this.unsubscribeActive();
        });

        // Register settings items clicks
        this.settingsItems.forEach((item) => {
            item.addEventListener('click', () => {
                const settingType = item.getAttribute('data-setting');
                this.openDrawer(settingType);
            });
        });

        // Initialize general rosout logs subscription
        this.subscribeRosout();

        // Listen for screen changes to close the drawer and unsubscribe from active settings (like IMU/sensors)
        window.addEventListener('screenChanged', (e) => {
            if (e.detail.index !== 2) { // 2 is the settings screen
                if (this.drawer && this.drawer.classList.contains('is-open')) {
                    this.drawer.classList.remove('is-open');
                    this.unsubscribeActive();
                }
            }
        });
    }

    openDrawer(type) {
        safeClear(this.drawerContent);
        this.drawer.classList.add('is-open');
        this.unsubscribeActive();

        const context = {
            addLog: (msg) => this.addLog(msg),
            getLogBuffer: () => this.logBuffer,
            clearLogs: () => { this.logBuffer = []; },
            telemetry: this.telemetry
        };

        switch (type) {
            case 'navigation':
                this.drawerTitle.textContent = 'Navigation';
                renderNavigationSettings(this.drawerContent, context);
                break;

            case 'sensors': {
                this.drawerTitle.textContent = 'Sensors & IMU';
                const sensorsInstance = renderSensorsSettings(this.drawerContent, context);
                if (sensorsInstance && typeof sensorsInstance.cleanup === 'function') {
                    this.activePanelCleanup = sensorsInstance.cleanup;
                }
                break;
            }

            case 'wifi':
                this.drawerTitle.textContent = 'Connectivity';
                renderWifiSettings(this.drawerContent, context);
                break;

            case 'logs':
                this.drawerTitle.textContent = 'Logs & Diagnostics';
                renderLogsSettings(this.drawerContent, context);
                break;

            case 'system':
                this.drawerTitle.textContent = 'System Operations';
                renderSystemSettings(this.drawerContent, context);
                break;

            case 'calibration': {
                this.drawerTitle.textContent = 'Camera & Calibration';
                const calibrationInstance = renderCalibrationSettings(this.drawerContent, context);
                if (calibrationInstance && typeof calibrationInstance.cleanup === 'function') {
                    this.activePanelCleanup = calibrationInstance.cleanup;
                }
                break;
            }
                
            default: {
                this.drawerTitle.textContent = 'Settings';
                const p = safeCreateElement('p', [], {}, 'Select a category to customize parameters.');
                this.drawerContent.appendChild(p);
                break;
            }
        }
    }

    unsubscribeActive() {
        if (this.activePanelCleanup) {
            try { this.activePanelCleanup(); } catch (e) { }
            this.activePanelCleanup = null;
        }
    }

    addLog(msg) {
        const time = new Date().toISOString().slice(11, 19);
        const logLine = `[${time}] ${msg}\n`;
        this.logBuffer.push(logLine);
        if (this.logBuffer.length > this.maxLogs) {
            this.logBuffer.shift();
        }

        // If the log screen is open, append immediately
        const consoleEl = document.getElementById('logs-console-box');
        if (consoleEl) {
            consoleEl.textContent += logLine;
            consoleEl.scrollTop = consoleEl.scrollHeight;
        }
    }

    subscribeRosout() {
        try {
            const topic = rosService.createTopicV2(TOPICS.ROSOUT, MSG_TYPES.LOG);
            if (topic) {
                this.rosoutSubscription = topic.subscribe((msg) => {
                    let levelStr = 'INFO';
                    if (msg.level === 10) levelStr = 'DEBUG';
                    else if (msg.level === 30) levelStr = 'WARN';
                    else if (msg.level === 40) levelStr = 'ERROR';
                    else if (msg.level === 50) levelStr = 'FATAL';

                    const formatted = `[${levelStr}] [${msg.name}]: ${msg.msg}`;
                    this.addLog(formatted);
                });
            }
        } catch (err) {
            console.warn('[SettingsPanel] Could not subscribe to /rosout:', err);
        }
    }
}
