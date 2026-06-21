import { safeCreateElement } from '../../../shared/dom-utils.js';
import { createMappedBtn } from './panel-utils.js';

export function renderSystemSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Hardware host control commands:');
    contentEl.appendChild(info);

    // Reboot Pi button
    contentEl.appendChild(createMappedBtn('🔄 Reboot Relobot', 'reboot-pi-btn', 'c-settings-drawer__btn--warning', context));

    // Power Off Pi button
    contentEl.appendChild(createMappedBtn('🔌 Power Off Relobot', 'poweroff-pi-btn', 'c-settings-drawer__btn--danger', context));
}
