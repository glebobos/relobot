import { safeCreateElement } from '../../../shared/dom-utils.js';

export function createMappedBtn(label, targetId, extraClass = '', context = null) {
    const btn = safeCreateElement('button', `c-settings-drawer__btn ${extraClass}`, {}, label);
    btn.addEventListener('click', () => {
        const target = document.getElementById(targetId);
        if (target) {
            target.click();
            if (context && typeof context.addLog === 'function') {
                context.addLog(`[SettingsPanel] Clicked: ${label}`);
            }
        } else {
            if (context && typeof context.addLog === 'function') {
                context.addLog(`[SettingsPanel] Failed: ${label} (DOM target unavailable)`);
            }
        }
    });
    return btn;
}
