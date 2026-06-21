import { safeCreateElement } from '../../../shared/dom-utils.js';

export function renderLogsSettings(contentEl, context) {
    const info = safeCreateElement('p', [], {}, 'Echoing diagnostics logs from active ROS2 nodes (/rosout):');
    contentEl.appendChild(info);

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
