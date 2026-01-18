import React, { useEffect, useRef, useState } from 'react';
import * as ROSLIB from 'roslib';
import './CameraFeed.css';

const CONTROL_RADIUS = 80;

const CameraFeed = ({ telemetry }) => {
  const [imgSrc, setImgSrc] = useState('');
  const [cameraConnected, setCameraConnected] = useState(false);
  const videoRef = useRef(null);
  const activeTouch = useRef(false);
  const startPos = useRef({ x: 0, y: 0 });

  useEffect(() => {
    let ros = null;
    let topic = null;
    let lastCameraUpdate = 0;
    const CAMERA_THROTTLE_MS = 100;

    const connect = () => {
      const rosbridgeUrl = `ws://${window.location.hostname}:9090`;
      ros = new ROSLIB.Ros({ url: rosbridgeUrl });

      ros.on('connection', () => console.log(`[ROSBridge] Connected ${rosbridgeUrl}`));
      ros.on('error', err => console.error('[ROSBridge] Error', err));
      ros.on('close', () => console.warn('[ROSBridge] Connection closed'));

      topic = new ROSLIB.Topic({
        ros,
        name: '/camera/image_raw/compressed',
        messageType: 'sensor_msgs/msg/CompressedImage',
        throttle_rate: 100
      });

      topic.subscribe(msg => {
        const now = Date.now();
        if (now - lastCameraUpdate < CAMERA_THROTTLE_MS) return;
        lastCameraUpdate = now;

        const format = msg.format || 'image/jpeg';
        setImgSrc(`data:${format};base64,${msg.data}`);
        setCameraConnected(true);
      });
    };

    connect();

    return () => {
        if (topic) topic.unsubscribe();
        if (ros) ros.close();
    };
  }, []);

  const throttle = (func, limit) => {
    let lastFunc;
    let lastRan;
    return function (...args) {
      const context = this;
      if (!lastRan) {
        func.apply(context, args);
        lastRan = Date.now();
      } else {
        clearTimeout(lastFunc);
        lastFunc = setTimeout(function () {
          if (Date.now() - lastRan >= limit) {
            func.apply(context, args);
            lastRan = Date.now();
          }
        }, limit - (Date.now() - lastRan));
      }
    };
  };

  const updateMotors = (x, y) => {
    fetch('/set_motors', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y }),
    }).catch(err => console.error(err));
  };

  const throttledUpdateMotors = useRef(throttle(updateMotors, 100)).current;

  const handleControl = (clientX, clientY) => {
    const dx = clientX - startPos.current.x;
    const dy = clientY - startPos.current.y;

    let normalizedX = Math.max(-1, Math.min(1, dx / CONTROL_RADIUS));
    let normalizedY = Math.max(-1, Math.min(1, -dy / CONTROL_RADIUS));

    throttledUpdateMotors(normalizedX, normalizedY);
  };

  const onStart = (e) => {
    activeTouch.current = true;
    const touch = e.touches ? e.touches[0] : e;
    startPos.current = { x: touch.clientX, y: touch.clientY };
    // e.preventDefault(); // Moved to passive: false in main if needed, but React handles events
  };

  const onMove = (e) => {
    if (!activeTouch.current) return;
    const touch = e.touches ? e.touches[0] : e;
    handleControl(touch.clientX, touch.clientY);
  };

  const onEnd = () => {
    if (!activeTouch.current) return;
    activeTouch.current = false;
    throttledUpdateMotors(0, 0);
  };

  return (
    <div className="camera-section">
      <div
        className="video-container"
        ref={videoRef}
        onMouseDown={onStart}
        onTouchStart={onStart}
        onMouseMove={onMove}
        onTouchMove={onMove}
        onMouseUp={onEnd}
        onTouchEnd={onEnd}
        onMouseLeave={onEnd}
      >
        <img src={imgSrc} alt="Camera Stream" style={{ display: cameraConnected ? 'block' : 'none' }} />
        <div className="video-overlay">
           <span className={`overlay-value ${telemetry.voltage < 24 && telemetry.voltage > 0 ? 'warning' : ''}`} style={{opacity: 0.8}}>
               {telemetry.voltage ? `${telemetry.voltage.toFixed(1)} V` : '--.-- V'}
           </span>
           <span className="overlay-value" style={{opacity: 0.85}}>
               {telemetry.rpm ? `${Math.round(telemetry.rpm)} RPM` : '--- RPM'}
           </span>
        </div>
      </div>
    </div>
  );
};

export default CameraFeed;
