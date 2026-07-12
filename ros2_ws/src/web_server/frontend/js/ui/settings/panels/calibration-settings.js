import { safeCreateElement } from '../../../shared/dom-utils.js';

export function renderCalibrationSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Toggle camera overlays and tools to assist with calibration and targeting:');
    contentEl.appendChild(info);

    // Helper to create a toggle row
    const createToggleRow = (label, storageKey) => {
        const row = safeCreateElement('div', 'c-settings-drawer__row');
        
        const labelEl = safeCreateElement('span', 'c-settings-drawer__row-label', {}, label);
        row.appendChild(labelEl);

        const switchLabel = safeCreateElement('label', 'c-switch');
        const checkbox = safeCreateElement('input', [], { type: 'checkbox' });
        
        // Read current state from localStorage, default to false (disabled)
        const isEnabled = localStorage.getItem(storageKey) === 'true';
        checkbox.checked = isEnabled;

        checkbox.addEventListener('change', (e) => {
            const checked = e.target.checked;
            localStorage.setItem(storageKey, checked ? 'true' : 'false');
            if (context && typeof context.addLog === 'function') {
                context.addLog(`[CalibrationSettings] ${label} toggled ${checked ? 'ON' : 'OFF'}`);
            }
            // Dispatch custom event to notify camera controller
            window.dispatchEvent(new CustomEvent('calibrationSettingsChanged'));
        });

        const slider = safeCreateElement('span', 'c-switch__slider');

        switchLabel.appendChild(checkbox);
        switchLabel.appendChild(slider);
        row.appendChild(switchLabel);

        return row;
    };

    const crosshairRow = createToggleRow('Center Crosshair', 'calibration_crosshair_enabled');
    const apriltagRow = createToggleRow('AprilTag HUD', 'calibration_apriltag_enabled');

    contentEl.appendChild(crosshairRow);
    contentEl.appendChild(apriltagRow);

    return {
        cleanup: () => {
            // Nothing to clean up for static UI settings
        }
    };
}
