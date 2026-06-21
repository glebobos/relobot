import { safeCreateElement } from '../../../shared/dom-utils.js';
import { createMappedBtn } from './panel-utils.js';

export function renderNavigationSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'SLAM Toolbox mapping and localization control:');
    contentEl.appendChild(info);

    // Save Current Map button
    contentEl.appendChild(createMappedBtn('💾 Save Current Map', 'save-map-btn', 'c-settings-drawer__btn--primary', context));

    // Restart SLAM Toolbox button
    contentEl.appendChild(createMappedBtn('🔄 Restart SLAM Toolbox', 'restart-slam-btn', '', context));

    // Reset Map button
    contentEl.appendChild(createMappedBtn('🗑️ Reset Map (Mapping Mode)', 'reset-map-btn', 'c-settings-drawer__btn--danger', context));
}
