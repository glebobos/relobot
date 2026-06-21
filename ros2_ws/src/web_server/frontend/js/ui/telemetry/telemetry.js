import { rosService } from '../../services/ros-service.js';
import { TOPICS, MSG_TYPES } from '../../shared/constants.js';

export class Telemetry {
    constructor() {
        this.vinSpan = document.getElementById('vin-display');
        this.vinSpanCam = document.getElementById('vin-display-cam');
        this.statusBatteryPct = document.getElementById('status-battery-pct');
        this.statusBatteryIcon = document.getElementById('status-battery-icon');
        
        this.chargerSpan = document.getElementById('charger-voltage-display');
        this.rpmSpan = document.getElementById('rpm-display');
        this.rpmSpanCam = document.getElementById('rpm-display-cam');
        
        this.dockBtn = document.getElementById('dock-btn');

        this.voltageThreshold = 22.8; // warning threshold
        this.currentChargerVolts = 0.0;
        this.isRobotOnDock = false;
        this.dockingActive = false;

        this.init();
    }

    init() {
        if (this.vinSpan || this.vinSpanCam) {
            const vinTopic = rosService.createTopicV2(TOPICS.BATTERY, MSG_TYPES.FLOAT32, { throttle_rate: 1000 });
            this.vinSubscription = vinTopic.subscribe(msg => {
                const v = parseFloat(msg.data);
                if (Number.isFinite(v)) this.updateVoltage(v);
            });
        }

        if (this.chargerSpan) {
            const chargerTopic = rosService.createTopicV2(TOPICS.CHARGER, MSG_TYPES.FLOAT32, { throttle_rate: 1000 });
            this.chargerSubscription = chargerTopic.subscribe(msg => {
                const v = parseFloat(msg.data);
                if (Number.isFinite(v)) this.updateChargerVoltage(v);
            });
        }

        const onDockTopic = rosService.createTopicV2(TOPICS.ON_DOCK, MSG_TYPES.BOOL, { throttle_rate: 1000 });
        this.onDockSubscription = onDockTopic.subscribe(msg => this.updateOnDock(msg.data));

        if (this.rpmSpan || this.rpmSpanCam) {
            const rpmTopic = rosService.createTopicV2(TOPICS.KNIVES, MSG_TYPES.FLOAT32, { throttle_rate: 1000 });
            this.rpmSubscription = rpmTopic.subscribe(msg => {
                const r = parseFloat(msg.data);
                if (Number.isFinite(r)) this.updateRpm(r);
            });
        }

    }

    updateVoltage(volts) {
        // Map 22.0V - 26.2V to 0% - 100% for battery icon levels
        const pctVal = Math.round(((volts - 22.0) / (26.2 - 22.0)) * 100);
        const clampedPct = Math.max(0, Math.min(100, pctVal));
        const isLow = volts < this.voltageThreshold;
        const formattedVolts = volts.toFixed(1) + ' V';

        // Update Screen 1 Card
        if (this.vinSpan) {
            this.vinSpan.textContent = formattedVolts;
            this.vinSpan.parentElement.classList.toggle('is-warning', isLow);
        }

        // Update Screen 2 Card
        if (this.vinSpanCam) {
            this.vinSpanCam.textContent = formattedVolts;
            this.vinSpanCam.parentElement.classList.toggle('is-warning', isLow);
        }

        // Update Top Status Bar Battery
        if (this.statusBatteryPct) {
            this.statusBatteryPct.textContent = formattedVolts;
        }

        // Update Top Status Bar Battery Icon
        if (this.statusBatteryIcon) {
            this.statusBatteryIcon.className = 'fas';
            if (clampedPct > 80) this.statusBatteryIcon.classList.add('fa-battery-full');
            else if (clampedPct > 55) this.statusBatteryIcon.classList.add('fa-battery-three-quarters');
            else if (clampedPct > 35) this.statusBatteryIcon.classList.add('fa-battery-half');
            else if (clampedPct > 15) this.statusBatteryIcon.classList.add('fa-battery-quarter');
            else this.statusBatteryIcon.classList.add('fa-battery-empty');
            
            this.statusBatteryIcon.style.color = isLow ? 'hsl(0, 84%, 48%)' : '';
        }
    }

    renderChargerVoltage() {
        if (!this.chargerSpan) return;
        
        const headerChargingIcon = document.getElementById('header-charging-icon');
        if (this.isRobotOnDock) {
            const voltsVal = this.currentChargerVolts > 0.5 ? this.currentChargerVolts : 27.1;
            const voltsText = voltsVal.toFixed(1) + ' V';
            this.chargerSpan.textContent = voltsText;
            this.chargerSpan.style.display = 'inline-block';
            
            const sub = document.getElementById('fuiDockSub');
            if (sub) sub.textContent = 'Charging (' + voltsText + ')';
            
            if (headerChargingIcon) {
                headerChargingIcon.style.display = 'inline-block';
            }
        } else {
            this.chargerSpan.style.display = 'none';

            const sub = document.getElementById('fuiDockSub');
            if (sub) sub.textContent = 'Disconnected (FUI 0.0)';
            
            if (headerChargingIcon) {
                headerChargingIcon.style.display = 'none';
            }
        }
    }

    updateChargerVoltage(volts) {
        this.currentChargerVolts = volts;
        this.renderChargerVoltage();
    }

    updateRpm(rpm) {
        const rounded = Math.round(rpm);
        if (this.rpmSpan) {
            this.rpmSpan.textContent = rounded;
        }
        if (this.rpmSpanCam) {
            this.rpmSpanCam.textContent = rounded;
        }

        // Spin the blade icon visually
        const rpmIconHeader = document.getElementById('rpm-icon-header');
        const rpmIconCam = document.getElementById('rpm-icon-cam');
        [rpmIconHeader, rpmIconCam].forEach(icon => {
            if (icon) {
                const img = icon.querySelector('i') || icon;
                if (rounded > 100) {
                    img.style.animationPlayState = 'running';
                    const speed = 3000 / rounded;
                    img.style.animationDuration = `${speed.toFixed(2)}s`;
                } else {
                    img.style.animationPlayState = 'paused';
                }
            }
        });
    }

    updateOnDock(isOnDock) {
        this.isRobotOnDock = isOnDock;
        if (this.dockBtn && !this.dockingActive) {
            this.dockBtn.disabled = isOnDock;
        }
        this.renderChargerVoltage();
    }

    setDockingActive(active) {
        this.dockingActive = active;
        if (this.dockBtn) {
            if (active) {
                this.dockBtn.disabled = false;
            } else {
                this.dockBtn.disabled = this.isRobotOnDock;
            }
        }
    }
    destroy() {
        if (this.vinSubscription) {
            this.vinSubscription.unsubscribe();
            this.vinSubscription = null;
        }
        if (this.chargerSubscription) {
            this.chargerSubscription.unsubscribe();
            this.chargerSubscription = null;
        }
        if (this.onDockSubscription) {
            this.onDockSubscription.unsubscribe();
            this.onDockSubscription = null;
        }
        if (this.rpmSubscription) {
            this.rpmSubscription.unsubscribe();
            this.rpmSubscription = null;
        }
    }

}
