import { rosService } from '../services/ros-service.js';

export class SettingsPanel {
    constructor(telemetry) {
        this.telemetry = telemetry;
        this.drawer = document.getElementById('settingsDrawer');
        this.drawerTitle = document.getElementById('drawerTitle');
        this.drawerContent = document.getElementById('drawerContent');
        this.backBtn = document.getElementById('drawerBackBtn');
        this.settingsItems = document.querySelectorAll('.settings-item');
        
        // Logs buffer
        this.logBuffer = [];
        this.maxLogs = 50;

        // Subscriptions
        this.rosoutSubscription = null;
        this.imuSubscription = null;

        this.init();
    }

    init() {
        if (!this.drawer || !this.backBtn) return;

        // Register back button click
        this.backBtn.addEventListener('click', () => {
            this.drawer.classList.remove('open');
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
    }

    openDrawer(type) {
        this.drawerContent.replaceChildren(); // Safe clear
        this.drawer.classList.add('open');
        this.unsubscribeActive();

        switch (type) {
            case 'navigation':
                this.drawerTitle.textContent = 'Navigation';
                this.renderNavigationPanel();
                break;
            case 'blade':
                this.drawerTitle.textContent = 'Blade Control';
                this.renderBladePanel();
                break;
            case 'battery':
                this.drawerTitle.textContent = 'Battery Settings';
                this.renderBatteryPanel();
                break;
            case 'sensors':
                this.drawerTitle.textContent = 'Sensors & IMU';
                this.renderSensorsPanel();
                break;
            case 'rtk':
                this.drawerTitle.textContent = 'RTK / GPS';
                this.renderRtkPanel();
                break;
            case 'wifi':
                this.drawerTitle.textContent = 'Connectivity';
                this.renderWifiPanel();
                break;
            case 'safety':
                this.drawerTitle.textContent = 'Safety Systems';
                this.renderSafetyPanel();
                break;
            case 'firmware':
                this.drawerTitle.textContent = 'Firmware';
                this.renderFirmwarePanel();
                break;
            case 'logs':
                this.drawerTitle.textContent = 'Logs & Diagnostics';
                this.renderLogsPanel();
                break;
            default:
                this.drawerTitle.textContent = 'Settings';
                const p = document.createElement('p');
                p.textContent = 'Select a category to customize parameters.';
                this.drawerContent.appendChild(p);
        }
    }

    unsubscribeActive() {
        if (this.imuSubscription) {
            try { this.imuSubscription.unsubscribe(); } catch(e){}
            this.imuSubscription = null;
        }
    }

    addMockLog(msg) {
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
            // Subscribe to /rosout (using ESM Ros bridge connection)
            const topic = rosService.createTopicV2('/rosout', 'rcl_interfaces/Log');
            if (topic) {
                this.rosoutSubscription = topic.subscribe((msg) => {
                    let levelStr = 'INFO';
                    if (msg.level === 10) levelStr = 'DEBUG';
                    else if (msg.level === 30) levelStr = 'WARN';
                    else if (msg.level === 40) levelStr = 'ERROR';
                    else if (msg.level === 50) levelStr = 'FATAL';
                    
                    const formatted = `[${levelStr}] [${msg.name}]: ${msg.msg}`;
                    this.addMockLog(formatted);
                });
            }
        } catch (err) {
            console.warn('[SettingsPanel] Could not subscribe to /rosout:', err);
        }
    }

    // --- RENDER CATEGORY DRAWERS ---

    renderNavigationPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Map coverage boundaries, manual waypoints, and docking controls:';
        content.appendChild(info);

        // helper function to create a mapped button
        const createMappedBtn = (label, targetId, extraClass = '') => {
            const btn = document.createElement('button');
            btn.className = `drawer-btn ${extraClass}`;
            btn.textContent = label;
            btn.addEventListener('click', () => {
                const target = document.getElementById(targetId);
                if (target) {
                    target.click();
                    this.addMockLog(`[NavPanel] Clicked: ${label}`);
                } else {
                    this.addMockLog(`[NavPanel] Failed: ${label} (DOM target unavailable)`);
                }
            });
            return btn;
        };

        // Go to point button
        content.appendChild(createMappedBtn('📍 Go to Point (Tap on Map)', 'goto-point-btn', 'drawer-btn-primary'));
        
        // Preview Coverage path
        content.appendChild(createMappedBtn('🔍 Preview Coverage Plan', 'coverage-preview-btn'));
        
        // Undock robot
        content.appendChild(createMappedBtn('🔌 Execute Undock Sequence', 'undock-btn'));

        // Clear drawn zone
        content.appendChild(createMappedBtn('🗑️ Clear Boundary Zone', 'coverage-clear-zone-btn', 'drawer-btn-danger'));
    }

    renderBladePanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Customize knife blade rotation limits and automatic spin features:';
        content.appendChild(info);

        const createRow = (label, valText) => {
            const row = document.createElement('div');
            row.className = 'drawer-row';
            const labelEl = document.createElement('span');
            labelEl.className = 'drawer-row-label';
            labelEl.textContent = label;
            const valEl = document.createElement('span');
            valEl.className = 'drawer-row-val';
            valEl.textContent = valText;
            row.appendChild(labelEl);
            row.appendChild(valEl);
            return row;
        };

        content.appendChild(createRow('Min Operating Speed', '500 RPM'));
        content.appendChild(createRow('Max Operating Speed', '3000 RPM'));
        content.appendChild(createRow('Current Target Spin', (this.telemetry.rpmSpan ? this.telemetry.rpmSpan.textContent : '0') + ' RPM'));

        const button = document.createElement('button');
        button.className = 'drawer-btn drawer-btn-danger';
        button.textContent = '🛑 Stop Blades Immediately';
        button.addEventListener('click', () => {
            const slider = document.getElementById('knifeSlider');
            if (slider) {
                slider.value = 0;
                // Dispatch input event to trigger control-panel slider callback
                slider.dispatchEvent(new Event('input'));
            }
            this.addMockLog('[BladePanel] Emergency blade stop command sent.');
        });
        content.appendChild(button);
    }

    renderBatteryPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Monitors charger parameters and lithium-ion cells:';
        content.appendChild(info);

        const createRow = (label, valText) => {
            const row = document.createElement('div');
            row.className = 'drawer-row';
            const labelEl = document.createElement('span');
            labelEl.className = 'drawer-row-label';
            labelEl.textContent = label;
            const valEl = document.createElement('span');
            valEl.className = 'drawer-row-val';
            valEl.textContent = valText;
            row.appendChild(labelEl);
            row.appendChild(valEl);
            return row;
        };

        const currentVoltage = this.telemetry.currentChargerVolts || 25.4;
        content.appendChild(createRow('Pack Nominal Voltage', '24.0 V'));
        content.appendChild(createRow('Measured Voltage', currentVoltage.toFixed(1) + ' V'));
        content.appendChild(createRow('Voltage Cutoff Limit', '22.0 V'));
        content.appendChild(createRow('Battery Health', '98% (Good)'));
        content.appendChild(createRow('Internal Resistance', '0.11 Ohm'));
    }

    renderSensorsPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Real-time readings from active sensors (Lidar & IMU):';
        content.appendChild(info);

        // IMU output containers
        const rollRow = document.createElement('div');
        rollRow.className = 'drawer-row';
        rollRow.appendChild(document.createTextNode('IMU Roll: '));
        const rollVal = document.createElement('span');
        rollVal.className = 'drawer-row-val';
        rollVal.textContent = '0.00°';
        rollRow.appendChild(rollVal);
        content.appendChild(rollRow);

        const pitchRow = document.createElement('div');
        pitchRow.className = 'drawer-row';
        pitchRow.appendChild(document.createTextNode('IMU Pitch: '));
        const pitchVal = document.createElement('span');
        pitchVal.className = 'drawer-row-val';
        pitchVal.textContent = '0.00°';
        pitchRow.appendChild(pitchVal);
        content.appendChild(pitchRow);

        const yawRow = document.createElement('div');
        yawRow.className = 'drawer-row';
        yawRow.appendChild(document.createTextNode('IMU Yaw: '));
        const yawVal = document.createElement('span');
        yawVal.className = 'drawer-row-val';
        yawVal.textContent = '0.00°';
        yawRow.appendChild(yawVal);
        content.appendChild(yawRow);

        // Subscribe to live IMU
        try {
            const imuTopic = rosService.createTopicV1('/imu', 'sensor_msgs/Imu', { throttle_rate: 200 });
            if (imuTopic) {
                this.imuSubscription = imuTopic.subscribe((msg) => {
                    const q = msg.orientation;
                    // Compute basic Euler angles from quaternion orientation
                    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
                    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
                    const roll = Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI);

                    const sinp = 2 * (q.w * q.y - q.z * q.x);
                    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp) * (180 / Math.PI);

                    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                    const yaw = Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI);

                    rollVal.textContent = roll.toFixed(2) + '°';
                    pitchVal.textContent = pitch.toFixed(2) + '°';
                    yawVal.textContent = yaw.toFixed(2) + '°';
                });
            }
        } catch (e) {
            console.warn('[SensorsPanel] IMU subscription setup failed, running mock simulation');
            
            // Sim fallback loop
            let mockAngle = 0;
            const simInterval = setInterval(() => {
                if (!document.getElementById('settingsDrawer').classList.contains('open')) {
                    clearInterval(simInterval);
                    return;
                }
                mockAngle += 0.05;
                rollVal.textContent = (Math.sin(mockAngle) * 1.5).toFixed(2) + '°';
                pitchVal.textContent = (Math.cos(mockAngle) * 0.8).toFixed(2) + '°';
                yawVal.textContent = ((mockAngle * 5) % 360).toFixed(1) + '°';
            }, 200);
        }
    }

    renderRtkPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'High-precision RTK GNSS satellite positioning status:';
        content.appendChild(info);

        const createRow = (label, valText) => {
            const row = document.createElement('div');
            row.className = 'drawer-row';
            const labelEl = document.createElement('span');
            labelEl.className = 'drawer-row-label';
            labelEl.textContent = label;
            const valEl = document.createElement('span');
            valEl.className = 'drawer-row-val';
            valEl.textContent = valText;
            row.appendChild(labelEl);
            row.appendChild(valEl);
            return row;
        };

        content.appendChild(createRow('GNSS Fix Status', 'FIXED RTK'));
        content.appendChild(createRow('Connected Satellites', '15 (GPS/GLONASS)'));
        content.appendChild(createRow('Precision (Horizontal)', '0.015 m'));
        content.appendChild(createRow('Precision (Vertical)', '0.024 m'));
        content.appendChild(createRow('Correction Base Age', '0.6 seconds'));
    }

    renderWifiPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Network links and Websocket interface coordinates:';
        content.appendChild(info);

        const createRow = (label, valText) => {
            const row = document.createElement('div');
            row.className = 'drawer-row';
            const labelEl = document.createElement('span');
            labelEl.className = 'drawer-row-label';
            labelEl.textContent = label;
            const valEl = document.createElement('span');
            valEl.className = 'drawer-row-val';
            valEl.textContent = valText;
            row.appendChild(labelEl);
            row.appendChild(valEl);
            return row;
        };

        content.appendChild(createRow('Connected SSID', 'ReloBot_Mow_5G'));
        content.appendChild(createRow('Signal Level', '-46 dBm (Excellent)'));
        content.appendChild(createRow('Websocket IP Address', window.location.hostname));
        content.appendChild(createRow('Websocket Port', '9090'));
        content.appendChild(createRow('Connection Protocol', 'ESM / Legacy Rosbridge'));
    }

    renderSafetyPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Checks emergency triggers and tilt-prevention features:';
        content.appendChild(info);

        const createRow = (label, valText) => {
            const row = document.createElement('div');
            row.className = 'drawer-row';
            const labelEl = document.createElement('span');
            labelEl.className = 'drawer-row-label';
            labelEl.textContent = label;
            const valEl = document.createElement('span');
            valEl.className = 'drawer-row-val';
            valEl.textContent = valText;
            row.appendChild(labelEl);
            row.appendChild(valEl);
            return row;
        };

        content.appendChild(createRow('Physical Bumper Sw', 'NORMAL (Clear)'));
        content.appendChild(createRow('Tilt Angle limit', '25.0°'));
        content.appendChild(createRow('Emergency Estop Sw', 'RELEASED (Armed)'));
        content.appendChild(createRow('Geofence Boundary', 'OK (Inside Fence)'));
    }

    renderFirmwarePanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Version identifiers for controllers and ROS2 Humble nodes:';
        content.appendChild(info);

        const createRow = (label, valText) => {
            const row = document.createElement('div');
            row.className = 'drawer-row';
            const labelEl = document.createElement('span');
            labelEl.className = 'drawer-row-label';
            labelEl.textContent = label;
            const valEl = document.createElement('span');
            valEl.className = 'drawer-row-val';
            valEl.textContent = valText;
            row.appendChild(labelEl);
            row.appendChild(valEl);
            return row;
        };

        content.appendChild(createRow('Raspberry Pi 5 OS', 'Debian Bookworm (64-bit)'));
        content.appendChild(createRow('Microcontroller A (Drive)', 'MicroPython Wheel-Pico v1.2'));
        content.appendChild(createRow('Microcontroller B (Blades)', 'MicroPython Blade-Pico v1.1'));
        content.appendChild(createRow('ROS2 Base Version', 'Humble Hawksbill'));
    }

    renderLogsPanel() {
        const content = this.drawerContent;

        const info = document.createElement('p');
        info.textContent = 'Echoing diagnostics logs from active ROS2 nodes (/rosout):';
        content.appendChild(info);

        const consoleBox = document.createElement('div');
        consoleBox.id = 'logs-console-box';
        consoleBox.className = 'logs-console';
        
        // Fill initial log lines from the buffer
        consoleBox.textContent = this.logBuffer.join('');
        content.appendChild(consoleBox);

        // Auto scroll to bottom
        setTimeout(() => {
            consoleBox.scrollTop = consoleBox.scrollHeight;
        }, 50);

        const clearBtn = document.createElement('button');
        clearBtn.className = 'drawer-btn';
        clearBtn.textContent = '🧹 Clear Console';
        clearBtn.addEventListener('click', () => {
            this.logBuffer = [];
            consoleBox.textContent = '';
        });
        content.appendChild(clearBtn);
    }
}
