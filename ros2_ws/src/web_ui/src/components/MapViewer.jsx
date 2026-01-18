import React, { useEffect, useRef } from 'react';
import * as ROSLIB from 'roslib';
import * as THREE from 'three';
// ROS3D is likely not ESM compatible, use star import
import * as ROS3D from 'ros3d';
import './MapViewer.css';

const MapViewer = () => {
  const mapRef = useRef(null);

  useEffect(() => {
    // Wait for container to be ready
    if (!mapRef.current) return;

    // Connect to ROS
    const ros = new ROSLIB.Ros({
      url: "ws://" + window.location.hostname + ":9090",
    });

    ros.on("connection", () => console.log("MapViewer: Connected to websocket server."));
    ros.on("error", (error) => console.log("MapViewer: Error connecting to websocket server: ", error));

    const containerWidth = mapRef.current.offsetWidth;
    const containerHeight = mapRef.current.offsetHeight;

    // Create viewer
    const viewer = new ROS3D.Viewer({
      divID: "map-container",
      width: containerWidth,
      height: containerHeight,
      antialias: true,
    });

    // Setup map client
    new ROS3D.OccupancyGridClient({
      ros: ros,
      rootObject: viewer.scene,
    });

    // Robot marker
    const robotMarker = new ROS3D.Arrow({
      length: 0.5,
      headLength: 0.25,
      shaftDiameter: 0.4,
      headDiameter: 0.4,
      material: new THREE.MeshBasicMaterial({ color: 0xff0000 }),
    });
    viewer.scene.add(robotMarker);

    const odomSub = new ROSLIB.Topic({
      ros: ros,
      name: "/odometry/filtered",
      messageType: "nav_msgs/Odometry",
    });

    odomSub.subscribe((message) => {
      robotMarker.position.x = message.pose.pose.position.x;
      robotMarker.position.y = message.pose.pose.position.y;
      robotMarker.position.z = 0.1;

      const q = message.pose.pose.orientation;
      const quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);
      const direction = new THREE.Vector3(1, 0, 0);
      direction.applyQuaternion(quaternion);
      robotMarker.setDirection(direction);
    });

    const handleResize = () => {
        if (!mapRef.current) return;
        const newWidth = mapRef.current.offsetWidth;
        const newHeight = mapRef.current.offsetHeight;
        viewer.camera.aspect = newWidth / newHeight;
        viewer.camera.updateProjectionMatrix();
        viewer.renderer.setSize(newWidth, newHeight);
    };

    window.addEventListener("resize", handleResize);

    return () => {
        window.removeEventListener("resize", handleResize);
        odomSub.unsubscribe();
        ros.close();
        // Cleanup viewer if possible? ROS3D doesn't have a clear dispose
    };
  }, []);

  const saveMap = () => {
    if (window.confirm('Save current map? This will overwrite the previous save.')) {
        fetch('/api/save_map', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                if (data.success) alert('Map saved successfully!');
                else alert('Error: ' + data.error);
            })
            .catch(err => alert('Network error: ' + err));
    }
  };

  return (
    <div className="map-section">
      <div className="map-controls">
        <button className="map-btn" onClick={saveMap}>Save Map</button>
      </div>
      <div id="map-container" ref={mapRef}></div>
    </div>
  );
};

export default MapViewer;
