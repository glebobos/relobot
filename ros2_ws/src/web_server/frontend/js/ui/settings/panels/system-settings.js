import { rosService } from '../../../services/ros-service.js';
import { TOPICS, MSG_TYPES } from '../../../shared/constants.js';
import { safeCreateElement } from '../../../shared/dom-utils.js';
import { createMappedBtn } from './panel-utils.js';

export function renderSystemSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Host system & hardware power management:');
    contentEl.appendChild(info);

    // Network & Host info card
    const card = safeCreateElement('div', 'c-metric-container', { style: 'margin-bottom: 16px;' });
    
    const wifiRow = safeCreateElement('div', 'c-metric-row');
    const wifiMeta = safeCreateElement('div', 'c-metric-header');
    wifiMeta.appendChild(safeCreateElement('span', [], {}, 'Connected Wi-Fi'));
    const wifiVal = safeCreateElement('span', 'c-metric-value', {}, 'Checking...');
    wifiMeta.appendChild(wifiVal);
    wifiRow.appendChild(wifiMeta);
    card.appendChild(wifiRow);

    const signalRow = safeCreateElement('div', 'c-metric-row');
    const signalMeta = safeCreateElement('div', 'c-metric-header');
    signalMeta.appendChild(safeCreateElement('span', [], {}, 'Signal Level'));
    const signalVal = safeCreateElement('span', 'c-metric-value', {}, '-- dBm');
    signalMeta.appendChild(signalVal);
    signalRow.appendChild(signalMeta);
    card.appendChild(signalRow);

    contentEl.appendChild(card);

    let subscription = null;
    try {
        subscription = rosService.subscribe(TOPICS.SYSTEM_METRICS, MSG_TYPES.STRING, (msg) => {
            try {
                const data = JSON.parse(msg.data);
                if (data.wifi_ssid) {
                    wifiVal.textContent = data.wifi_ssid;
                } else {
                    wifiVal.textContent = 'Disconnected';
                }
                if (data.wifi_dbm !== undefined && data.wifi_dbm !== null) {
                    signalVal.textContent = `${data.wifi_dbm} dBm`;
                } else {
                    signalVal.textContent = '-- dBm';
                }
            } catch (_) {}
        });
    } catch (_) {}

    // Reboot Pi button
    contentEl.appendChild(createMappedBtn('🔄 Reboot Relobot (Restart Host)', 'reboot-pi-btn', 'c-settings-drawer__btn--warning', context));

    // Power Off Pi button
    contentEl.appendChild(createMappedBtn('🔌 Power Off Relobot', 'poweroff-pi-btn', 'c-settings-drawer__btn--danger', context));

    return {
        cleanup: () => {
            if (subscription) {
                try { subscription.unsubscribe(); } catch (_) {}
                subscription = null;
            }
        }
    };
}
