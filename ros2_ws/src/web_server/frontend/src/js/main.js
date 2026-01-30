import '../css/style.css';
import { connectROS } from './ros_connection.js';
import { initCamera } from './camera.js';
import { initTelemetry } from './telemetry.js';
import { initGamepad } from './controls.js';
import { initMap, resizeMap } from './map.js';

document.addEventListener('DOMContentLoaded', () => {
    console.log("Initializing ReloBot Frontend...");

    // Connect ROS
    connectROS();

    // Initialize Modules
    initCamera();
    initTelemetry();
    initGamepad();
    initMap();

    // Tab Switching Logic
    const tabs = document.querySelectorAll('.tab-btn');
    const views = document.querySelectorAll('.view-section');

    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            // Deactivate all
            tabs.forEach(t => t.classList.remove('active'));
            views.forEach(v => v.classList.add('hidden'));

            // Activate clicked
            tab.classList.add('active');
            const targetId = tab.dataset.target;
            const targetView = document.getElementById(targetId);
            if(targetView) {
                targetView.classList.remove('hidden');

                // Special handling for Map resize
                if(targetId === 'view-map') {
                    // Slight delay to allow display:block to render layout
                    setTimeout(() => resizeMap(), 50);
                }
            }
        });
    });
});
