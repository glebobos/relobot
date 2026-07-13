import { rosService } from '../../../services/ros-service.js';
import { TOPICS, MSG_TYPES } from '../../../shared/constants.js';
import { safeCreateElement, safeClear, appendLogLine } from '../../../shared/dom-utils.js';

function dbmToPercent(dbm) {
    if (dbm === null || dbm === undefined) return 0;
    if (dbm >= -30) return 100;
    if (dbm <= -100) return 0;
    return Math.round((dbm - (-100)) * (100 / 70));
}

export function renderLogsSettings(contentEl, context) {
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
    
    // Wi-Fi Signal Row
    const wifiSignalRow = safeCreateElement('div', 'c-metric-row');
    const wifiSignalMeta = safeCreateElement('div', 'c-metric-header');
    wifiSignalMeta.appendChild(safeCreateElement('span', [], {}, 'Wi-Fi Signal Strength'));
    const wifiSignalVal = safeCreateElement('span', 'c-metric-value', {}, 'N/A');
    wifiSignalMeta.appendChild(wifiSignalVal);
    wifiSignalRow.appendChild(wifiSignalMeta);

    const wifiSignalBar = safeCreateElement('div', 'c-metric-bar');
    const wifiSignalFill = safeCreateElement('div', 'c-metric-bar__fill');
    wifiSignalBar.appendChild(wifiSignalFill);
    wifiSignalRow.appendChild(wifiSignalBar);

    metricContainer.appendChild(cpuRow);
    metricContainer.appendChild(ramRow);
    metricContainer.appendChild(wifiSignalRow);
    contentEl.appendChild(metricContainer);

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
        safeClear(consoleElement);
        context.getLogBuffer().forEach(line => {
            appendLogLine(consoleElement, line);
        });
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
            safeClear(consoleElement);
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

                    if (data.wifi_dbm !== undefined && data.wifi_dbm !== null) {
                        wifiSignalVal.textContent = data.wifi_dbm + ' dBm';
                        const percent = dbmToPercent(data.wifi_dbm);
                        wifiSignalFill.style.width = percent + '%';
                    } else {
                        wifiSignalVal.textContent = 'Disconnected';
                        wifiSignalFill.style.width = '0%';
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
