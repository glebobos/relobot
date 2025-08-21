window.onload = function () {
    // Give ROS bridge some time to startup
    setTimeout(() => {
        console.log('ROS3D available:', !!window.ROS3D);
        console.log('THREE available:', !!window.THREE);
        console.log('ROSLIB available:', !!window.ROSLIB);
        
        if (!window.ROS3D) {
            console.error('ROS3D is not available!');
            return;
        }
        
        // Connect to ROS.
        var ros = new ROSLIB.Ros({
            url: 'ws://' + window.location.hostname + ':9090'
        });

        ros.on('connection', function () {
            console.log('Connected to websocket server.');
        });

        ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error);
        });

        ros.on('close', function () {
            console.log('Connection to websocket server closed.');
        });

        // Get the map container dimensions
        var mapContainer = document.getElementById('map');
        var containerWidth = mapContainer.offsetWidth;
        var containerHeight = mapContainer.offsetHeight;

        // Create the main 3D viewer with responsive dimensions
        var viewer = new ROS3D.Viewer({
            divID: 'map',
            width: containerWidth,
            height: containerHeight,
            antialias: true
        });

        // Setup the map client using ROS3D.
        var gridClient = new ROS3D.OccupancyGridClient({
            ros: ros,
            rootObject: viewer.scene
        });

        // Setup robot marker for odometry
        var robotMarker = new THREE.Mesh(
            new THREE.SphereGeometry(0.1, 16, 16),
            new THREE.MeshBasicMaterial({ color: 0xff0000 })
        );
        viewer.scene.add(robotMarker);

        var odomSub = new ROSLIB.Topic({
            ros: ros,
            name: '/odometry/filtered',
            messageType: 'nav_msgs/Odometry'
        });

        odomSub.subscribe(function(message) {
            // Update robot position
            robotMarker.position.x = message.pose.pose.position.x;
            robotMarker.position.y = message.pose.pose.position.y;
            robotMarker.position.z = 0.1; // Slightly above ground

            // Update robot orientation
            var q = message.pose.pose.orientation;
            robotMarker.quaternion.set(q.x, q.y, q.z, q.w);
            
            console.log('Robot position:', message.pose.pose.position);
        });

        // Handle window resize to make the viewer responsive
        window.addEventListener('resize', function() {
            var newWidth = mapContainer.offsetWidth;
            var newHeight = mapContainer.offsetHeight;
            
            viewer.camera.aspect = newWidth / newHeight;
            viewer.camera.updateProjectionMatrix();
            viewer.renderer.setSize(newWidth, newHeight);
        });
    }, 1000);
}
