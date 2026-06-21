import { safeCreateElement } from '../../../shared/dom-utils.js';

export function renderWifiSettings(contentEl) {
    const info = safeCreateElement('p', [], {}, 'Network links and Websocket interface coordinates:');
    contentEl.appendChild(info);

    const createRow = (label, valText) => {
        const row = safeCreateElement('div', 'c-settings-drawer__row');
        const labelEl = safeCreateElement('span', 'c-settings-drawer__row-label', {}, label);
        const valEl = safeCreateElement('span', 'c-settings-drawer__row-val', {}, valText);
        row.appendChild(labelEl);
        row.appendChild(valEl);
        return row;
    };

    contentEl.appendChild(createRow('Connected SSID', 'ReloBot_Mow_5G (Placeholder)'));
    contentEl.appendChild(createRow('Signal Level', '-46 dBm (Placeholder)'));
    contentEl.appendChild(createRow('Websocket IP Address', window.location.hostname));
    contentEl.appendChild(createRow('Websocket Port', '9090'));
    contentEl.appendChild(createRow('Connection Protocol', 'ESM / Legacy Rosbridge'));
}
