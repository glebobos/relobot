export function initTelemetry() {
    const vinOverlay = document.getElementById('vin-display');
    const rpmOverlay = document.getElementById('rpm-display');
    const vinSys = document.getElementById('sys-vin');
    const rpmSys = document.getElementById('sys-rpm');

    const updateVoltage = (volts) => {
        if (volts === null || volts === undefined) return;
        const text = volts.toFixed(1) + ' V';

        if (vinOverlay) {
            vinOverlay.textContent = text;
            if (volts < 24) vinOverlay.classList.add('text-red-500');
            else vinOverlay.classList.remove('text-red-500');
        }

        if (vinSys) vinSys.textContent = volts.toFixed(2);
    };

    const updateRPM = (rpm) => {
        if (rpm === null || rpm === undefined) return;
        const text = Math.round(rpm) + ' RPM';

        if (rpmOverlay) rpmOverlay.textContent = text;
        if (rpmSys) rpmSys.textContent = Math.round(rpm);
    };

    // Initial Fetch
    fetch('/api/voltage')
        .then(r => r.ok ? r.json() : null)
        .then(j => j?.success && updateVoltage(j.vin))
        .catch(() => {});

    fetch('/api/rpm')
        .then(r => r.ok ? r.json() : null)
        .then(j => j?.success && updateRPM(j.rpm))
        .catch(() => {});

    // SSE Setup
    try {
        const esV = new EventSource('/stream/voltage');
        esV.onmessage = e => {
            const v = parseFloat(e.data);
            if (isFinite(v)) updateVoltage(v);
        };
        esV.onerror = () => console.warn('Voltage SSE disconnected');

        const esR = new EventSource('/stream/rpm');
        esR.onmessage = e => {
            const r = parseFloat(e.data);
            if (isFinite(r)) updateRPM(r);
        };
        esR.onerror = () => console.warn('RPM SSE disconnected');
    } catch (e) {
        console.error("Error setting up SSE:", e);
    }
}
