import { rosService } from '../services/ros-service.js';

export class Telemetry {
    constructor() {
        this.vinSpan = document.getElementById('vin-display');
        this.chargerSpan = document.getElementById('charger-voltage-display');
        this.rpmSpan = document.getElementById('rpm-display');
        this.dockBtn = document.getElementById('dock-btn');

        this.voltageThreshold = 24;
        this.currentChargerVolts = 0.0;
        this.isRobotOnDock = false;
        this.dockingActive = false;

        this.init();
    }

    init() {
        if (this.vinSpan) {
            const vinTopic = rosService.createTopicV2('/battery_voltage', 'std_msgs/Float32', { throttle_rate: 1000 });
            this.vinSubscription = vinTopic.subscribe(msg => {
                const v = parseFloat(msg.data);
                if (Number.isFinite(v)) this.updateVoltage(v);
            });
        }

        if (this.chargerSpan) {
            const chargerTopic = rosService.createTopicV2('/charger_voltage', 'std_msgs/Float32', { throttle_rate: 1000 });
            this.chargerSubscription = chargerTopic.subscribe(msg => {
                const v = parseFloat(msg.data);
                if (Number.isFinite(v)) this.updateChargerVoltage(v);
            });
        }

        const onDockTopic = rosService.createTopicV2('/on_dock', 'std_msgs/Bool', { throttle_rate: 1000 });
        this.onDockSubscription = onDockTopic.subscribe(msg => this.updateOnDock(msg.data));

        if (this.rpmSpan) {
            const rpmTopic = rosService.createTopicV2('/knives/current_rpm', 'std_msgs/Float32', { throttle_rate: 1000 });
            this.rpmSubscription = rpmTopic.subscribe(msg => {
                const r = parseFloat(msg.data);
                if (Number.isFinite(r)) this.updateRpm(r);
            });
        }
    }

    updateVoltage(volts) {
        if (this.vinSpan) {
            this.vinSpan.textContent = '🔋 ' + volts.toFixed(1);
            this.vinSpan.classList.toggle('warning', volts < this.voltageThreshold);
        }
    }

    renderChargerVoltage() {
        if (!this.chargerSpan) return;
        this.chargerSpan.textContent = '⚡ ' + this.currentChargerVolts.toFixed(1);
        this.chargerSpan.classList.toggle('charging', this.isRobotOnDock);
    }

    updateChargerVoltage(volts) {
        this.currentChargerVolts = volts;
        this.renderChargerVoltage();
    }

    updateRpm(rpm) {
        if (this.rpmSpan) {
            this.rpmSpan.textContent = '🕛 ' + Math.round(rpm);
        }
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
}
