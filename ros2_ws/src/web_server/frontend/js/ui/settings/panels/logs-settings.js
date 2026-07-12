import { safeCreateElement } from '../../../shared/dom-utils.js';

export function renderLogsSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Echoing diagnostics logs from active ROS2 nodes (/rosout):');
    contentEl.appendChild(info);

    // Toggle row for background logging
    const toggleRow = safeCreateElement('div', 'c-settings-drawer__row');
    const toggleLabel = safeCreateElement('span', 'c-settings-drawer__row-label', {}, 'Background Logging');
    toggleRow.appendChild(toggleLabel);

    const switchLabel = safeCreateElement('label', 'c-switch');
    const checkbox = safeCreateElement('input', [], { type: 'checkbox' });
    
    checkbox.checked = context && typeof context.isBackgroundEnabled === 'function' 
        ? context.isBackgroundEnabled() 
        : false;

    checkbox.addEventListener('change', (e) => {
        const checked = e.target.checked;
        if (context && typeof context.setBackgroundEnabled === 'function') {
            context.setBackgroundEnabled(checked);
        }
        if (context && typeof context.addLog === 'function') {
            context.addLog(`[LogsSettings] Background logging toggled ${checked ? 'ON' : 'OFF'}`);
        }
    });

    const slider = safeCreateElement('span', 'c-switch__slider');
    switchLabel.appendChild(checkbox);
    switchLabel.appendChild(slider);
    toggleRow.appendChild(switchLabel);
    contentEl.appendChild(toggleRow);

    const consoleElement = safeCreateElement('div', 'c-logs-console', {
        id: 'logs-console-box'
    });

    // Fill initial log lines from the buffer provided in context
    if (context && typeof context.getLogBuffer === 'function') {
        consoleElement.textContent = context.getLogBuffer().join('');
    }
    contentEl.appendChild(consoleElement);

    // Auto scroll to bottom
    setTimeout(() => {
        if (consoleElement) {
            consoleElement.scrollTop = consoleElement.scrollHeight;
        }
    }, 50);

    const clearBtn = safeCreateElement('button', 'c-settings-drawer__btn', {}, '🧹 Clear Console');
    clearBtn.addEventListener('click', () => {
        if (context && typeof context.clearLogs === 'function') {
            context.clearLogs();
        }
        if (consoleElement) {
            consoleElement.textContent = '';
        }
    });
    contentEl.appendChild(clearBtn);
}
