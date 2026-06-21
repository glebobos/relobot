import { rosService } from '../../../services/ros-service.js';
import { TOPICS, MSG_TYPES } from '../../../shared/constants.js';
import { safeCreateElement } from '../../../shared/dom-utils.js';

export function renderSensorsSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Real-time readings from active sensors (Lidar & IMU):');
    contentEl.appendChild(info);

    const createRow = (label) => {
        const row = safeCreateElement('div', 'c-settings-drawer__row');
        row.appendChild(document.createTextNode(label));
        const valEl = safeCreateElement('span', 'c-settings-drawer__row-val', {}, '0.00°');
        row.appendChild(valEl);
        return { row, valEl };
    };

    const rollData = createRow('IMU Roll: ');
    const pitchData = createRow('IMU Pitch: ');
    const yawData = createRow('IMU Yaw: ');

    contentEl.appendChild(rollData.row);
    contentEl.appendChild(pitchData.row);
    contentEl.appendChild(yawData.row);

    let imuSubscription = null;
    let simInterval = null;

    try {
        const imuTopic = rosService.createTopicV1(TOPICS.IMU, MSG_TYPES.IMU, { throttle_rate: 200 });
        if (!imuTopic) {
            throw new Error('ROSLIB V1 connection not ready');
        }
        imuSubscription = imuTopic.subscribe((msg) => {
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

                rollData.valEl.textContent = roll.toFixed(2) + '°';
                pitchData.valEl.textContent = pitch.toFixed(2) + '°';
                yawData.valEl.textContent = yaw.toFixed(2) + '°';
            });
    } catch (e) {
        console.warn('[SensorsSettings] IMU subscription setup failed, running mock simulation');

        // Sim fallback loop
        let mockAngle = 0;
        simInterval = setInterval(() => {
            const drawer = document.getElementById('settingsDrawer');
            if (!drawer || !drawer.classList.contains('is-open')) {
                clearInterval(simInterval);
                return;
            }
            mockAngle += 0.05;
            rollData.valEl.textContent = (Math.sin(mockAngle) * 1.5).toFixed(2) + '°';
            pitchData.valEl.textContent = (Math.cos(mockAngle) * 0.8).toFixed(2) + '°';
            yawData.valEl.textContent = ((mockAngle * 5) % 360).toFixed(1) + '°';
        }, 200);
    }

    // Return cleanup object
    return {
        cleanup: () => {
            if (imuSubscription) {
                try {
                    imuSubscription.unsubscribe();
                } catch (err) {
                    console.warn('[SensorsSettings] Failed to unsubscribe from IMU:', err);
                }
                imuSubscription = null;
            }
            if (simInterval) {
                clearInterval(simInterval);
                simInterval = null;
            }
        }
    };
}
