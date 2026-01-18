import React, { useState, useEffect } from 'react';
import CameraFeed from './components/CameraFeed';
import MapViewer from './components/MapViewer';
import GamepadHandler from './components/GamepadHandler';
import './index.css';

function App() {
  const [telemetry, setTelemetry] = useState({ voltage: 0, rpm: 0 });

  useEffect(() => {
    // Voltage
    fetch('/api/voltage')
      .then(r => r.ok ? r.json() : null)
      .then(j => j?.success && setTelemetry(prev => ({ ...prev, voltage: j.vin })))
      .catch(() => {});

    const esV = new EventSource('/stream/voltage');
    esV.onmessage = e => {
      const v = parseFloat(e.data);
      if (isFinite(v)) setTelemetry(prev => ({ ...prev, voltage: v }));
    };

    // RPM
    fetch('/api/rpm')
      .then(r => r.ok ? r.json() : null)
      .then(j => j?.success && setTelemetry(prev => ({ ...prev, rpm: j.rpm })))
      .catch(() => {});

    const esR = new EventSource('/stream/rpm');
    esR.onmessage = e => {
      const r = parseFloat(e.data);
      if (isFinite(r)) setTelemetry(prev => ({ ...prev, rpm: r }));
    };

    return () => {
        esV.close();
        esR.close();
    };
  }, []);

  return (
    <div className="App">
      <GamepadHandler />
      <CameraFeed telemetry={telemetry} />
      <MapViewer />
    </div>
  );
}

export default App;
