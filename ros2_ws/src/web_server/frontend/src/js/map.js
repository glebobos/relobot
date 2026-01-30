import { getROS } from './ros_connection.js';

let viewer = null;
let gridClient = null;

export function initMap() {
    // Docking Controls
    const dockBtn = document.getElementById('dock-btn');
    const undockBtn = document.getElementById('undock-btn');
    const saveMapBtn = document.getElementById('save-map-btn');

    if (dockBtn) dockBtn.addEventListener('click', (e) => { e.stopPropagation(); callApi('/api/dock'); });
    if (undockBtn) undockBtn.addEventListener('click', (e) => { e.stopPropagation(); callApi('/api/undock'); });
    if (saveMapBtn) saveMapBtn.addEventListener('click', (e) => {
        e.stopPropagation();
        if(confirm('Save current map? This will overwrite the previous save.')) callApi('/api/save_map');
    });

    // Initialize Viewer
    setTimeout(setupViewer, 1000);
}

function callApi(endpoint) {
    fetch(endpoint, { method: 'POST' })
        .then(r => r.json())
        .then(d => {
            if(!d.success) alert('Error: ' + (d.message || d.error));
            else alert('Success');
        })
        .catch(e => alert('Network Error: ' + e));
}

function setupViewer() {
    const mapContainer = document.getElementById("map-canvas");
    if (!mapContainer || !window.ROS3D) {
        if(!window.ROS3D) console.warn("ROS3D not loaded yet");
        return;
    }

    const ros = getROS();
    if (!ros) {
        console.warn("ROS connection not ready for Map");
        return;
    }

    const width = mapContainer.offsetWidth || window.innerWidth;
    const height = mapContainer.offsetHeight || 500;

    console.log(`Initializing ROS3D Viewer: ${width}x${height}`);

    try {
        viewer = new window.ROS3D.Viewer({
            divID: "map-canvas",
            width: width,
            height: height,
            antialias: true,
            background: '#1f2937' // Tailwind gray-800
        });

        gridClient = new window.ROS3D.OccupancyGridClient({
            ros: ros,
            rootObject: viewer.scene,
            continuous: true
        });

        // Robot Marker
        const robotMarker = new window.ROS3D.Arrow({
            length: 0.5,
            headLength: 0.25,
            shaftDiameter: 0.1,
            headDiameter: 0.2,
            material: new window.THREE.MeshBasicMaterial({ color: 0x3b82f6 }),
        });
        viewer.scene.add(robotMarker);

        // Odometry
        const odomSub = new window.ROSLIB.Topic({
            ros: ros,
            name: "/odometry/filtered",
            messageType: "nav_msgs/Odometry",
            throttle_rate: 100
        });

        odomSub.subscribe(function (message) {
            robotMarker.position.x = message.pose.pose.position.x;
            robotMarker.position.y = message.pose.pose.position.y;
            robotMarker.position.z = 0.1;

            const q = message.pose.pose.orientation;
            const quaternion = new window.THREE.Quaternion(q.x, q.y, q.z, q.w);
            const direction = new window.THREE.Vector3(1, 0, 0);
            direction.applyQuaternion(quaternion);
            robotMarker.setDirection(direction);
        });

    } catch (e) {
        console.error("Error setting up ROS3D:", e);
    }
}

export function resizeMap() {
    if (!viewer) {
        // If not initialized yet (maybe tab was hidden initially), try init
        const mapContainer = document.getElementById("map-canvas");
        if (mapContainer && mapContainer.offsetWidth > 0 && !viewer) {
            setupViewer();
        }
        return;
    }

    const mapContainer = document.getElementById("map-canvas");
    if (!mapContainer) return;

    const width = mapContainer.offsetWidth;
    const height = mapContainer.offsetHeight;

    if(width === 0 || height === 0) return;

    viewer.resize(width, height);
}
