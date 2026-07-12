import { rosService } from '../../../services/ros-service.js';
import { TOPICS, MSG_TYPES } from '../../../shared/constants.js';
import { safeCreateElement } from '../../../shared/dom-utils.js';

export function renderLogsSettings(contentEl, context) {
    // Section Header for metrics
    const statHeader = safeCreateElement('p', [], {}, 'System Health & Metrics:');
    contentEl.appendChild(statHeader);

    // Metric Container
    const metricContainer = safeCreateElement('div', 'c-metric-container');
    
    // CPU Row
    const cpuRow = safeCreateElement('div', 'c-metric-row');
    const cpuMeta = safeCreateElement('div', 'c-metric-header');
    cpuMeta.appendChild(safeCreateElement('span', [], {}, 'CPU Usage'));
    const cpuVal = safeCreateElement('span', 'c-metric-value', {}, '0.0%');
    cpuMeta.appendChild(cpuVal);
    cpuRow.appendChild(cpuMeta);
    
    const cpuBar = safeCreateElement('div', 'c-metric-bar');
    const cpuFill = safeCreateElement('div', 'c-metric-bar__fill');
    cpuBar.appendChild(cpuFill);
    cpuRow.appendChild(cpuBar);
    
    // RAM Row
    const ramRow = safeCreateElement('div', 'c-metric-row');
    const ramMeta = safeCreateElement('div', 'c-metric-header');
    ramMeta.appendChild(safeCreateElement('span', [], {}, 'Memory Usage'));
    const ramVal = safeCreateElement('span', 'c-metric-value', {}, '0.0%');
    ramMeta.appendChild(ramVal);
    ramRow.appendChild(ramMeta);
    
    const ramBar = safeCreateElement('div', 'c-metric-bar');
    const ramFill = safeCreateElement('div', 'c-metric-bar__fill');
    ramBar.appendChild(ramFill);
    ramRow.appendChild(ramBar);
    
    metricContainer.appendChild(cpuRow);
    metricContainer.appendChild(ramRow);
    contentEl.appendChild(metricContainer);

    // Logs Section
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

    // Subscription to system/metrics
    let subscription = null;
    try {
        subscription = rosService.createTopicV2(TOPICS.SYSTEM_METRICS, MSG_TYPES.STRING);
        if (subscription) {
            subscription.subscribe((msg) => {
                try {
                    const data = JSON.parse(msg.data);
                    if (data.cpu !== undefined) {
                        cpuVal.textContent = data.cpu.toFixed(1) + '%';
                        cpuFill.style.width = data.cpu + '%';
                    }
                    if (data.ram !== undefined) {
                        ramVal.textContent = data.ram.toFixed(1) + '%';
                        ramFill.style.width = data.ram + '%';
                    }
                } catch (err) {
                    console.error('[LogsSettings] Failed to parse metrics JSON:', err);
                }
            });
        }
    } catch (err) {
        console.warn('[LogsSettings] Could not subscribe to system metrics:', err);
    }

    return {
        cleanup: () => {
            if (subscription) {
                try {
                    subscription.unsubscribe();
                    console.log('[LogsSettings] Unsubscribed from system metrics');
                } catch (err) {
                    console.warn('[LogsSettings] Failed to unsubscribe from system metrics:', err);
                }
                subscription = null;
            }
        }
    };
}
