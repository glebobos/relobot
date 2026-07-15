import { safeCreateElement } from '../../../shared/dom-utils.js';
import { createToggleRow } from './panel-utils.js';

export function renderCalibrationSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Toggle camera overlays and tools to assist with calibration and targeting:');
    contentEl.appendChild(info);

    const crosshairRow = createToggleRow('Center Crosshair', 'calibration_crosshair_enabled', 'calibrationSettingsChanged', 'CalibrationSettings', context);
    const apriltagRow = createToggleRow('AprilTag HUD', 'calibration_apriltag_enabled', 'calibrationSettingsChanged', 'CalibrationSettings', context);

    contentEl.appendChild(crosshairRow);
    contentEl.appendChild(apriltagRow);

    return {
        cleanup: () => {
            // Nothing to clean up for static UI settings
        }
    };
}
