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

export function createToggleRow(label, storageKey, changeEventName, contextName, context = null) {
    const row = safeCreateElement('div', 'c-settings-drawer__row');
    
    const labelEl = safeCreateElement('span', 'c-settings-drawer__row-label', {}, label);
    row.appendChild(labelEl);

    const switchLabel = safeCreateElement('label', 'c-switch');
    const checkbox = safeCreateElement('input', [], { type: 'checkbox' });
    
    const isEnabled = localStorage.getItem(storageKey) === 'true';
    checkbox.checked = isEnabled;

    checkbox.addEventListener('change', (e) => {
        const checked = e.target.checked;
        localStorage.setItem(storageKey, checked ? 'true' : 'false');
        if (context && typeof context.addLog === 'function') {
            context.addLog(`[${contextName}] ${label} toggled ${checked ? 'ON' : 'OFF'}`);
        }
        window.dispatchEvent(new CustomEvent(changeEventName));
    });

    const slider = safeCreateElement('span', 'c-switch__slider');

    switchLabel.appendChild(checkbox);
    switchLabel.appendChild(slider);
    row.appendChild(switchLabel);

    return row;
}
