window.onload = function () {
    // Give ROS bridge some time to startup
    setTimeout(() => {
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

        // Create the main viewer.
        var viewer = new ROS2D.Viewer({
            divID: 'map',
            width: 800,
            height: 600
        });

        // Adjust viewer size to fit the container
        function resizeViewer() {
            var mapDiv = document.getElementById('map');
            if (mapDiv.clientWidth > 0 && mapDiv.clientHeight > 0) {
                viewer.width = mapDiv.clientWidth;
                viewer.height = mapDiv.clientHeight;
            }
        }
        window.addEventListener('resize', resizeViewer);
        // Initial resize
        resizeViewer();


        // Setup the map client.
        var gridClient = new ROS2D.OccupancyGridClient({
            ros: ros,
            rootObject: viewer.scene,
            // Use continuous rendering
            continuous: true
        });

        // Scale the canvas to fit the map
        gridClient.on('change', function () {
            viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
            viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
        });

        // Setup the robot marker.
        var robotMarker = new ROS2D.RobotMarker({
            size: 0.25,
            strokeSize: 0.02,
            fillColor: createjs.Graphics.getRGB(255, 0, 0, 0.8),
            pulse: true
        });
        viewer.scene.addChild(robotMarker);

        var odomSub = new ROSLIB.Topic({
            ros : ros,
            name : '/odometry/filtered',
            messageType : 'nav_msgs/Odometry'
        });

        odomSub.subscribe(function(message) {
            // position
            robotMarker.x = message.pose.pose.position.x;
            robotMarker.y = -message.pose.pose.position.y;

            // orientation
            var quat = new ROSLIB.Quaternion(message.pose.pose.orientation);
            var euler = new ROSLIB.Euler();
            euler.fromQuaternion(quat);
            robotMarker.rotation = -euler.yaw * 180.0 / Math.PI;
        });
    }, 1000);
}
