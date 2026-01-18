window.onload = function () {
  // Give ROS bridge some time to startup
  setTimeout(() => {
    if (!window.ROS3D) {
      console.error("ROS3D is not available!");
      return;
    }

    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url: "ws://" + window.location.hostname + ":9090",
    });

    ros.on("connection", function () {
      console.log("Connected to websocket server.");
    });

    ros.on("error", function (error) {
      console.log("Error connecting to websocket server: ", error);
    });

    ros.on("close", function () {
      console.log("Connection to websocket server closed.");
    });

    // Get the map container dimensions
    var mapContainer = document.getElementById("map");
    var containerWidth = mapContainer.offsetWidth;
    var containerHeight = mapContainer.offsetHeight;

    // Create the main 3D viewer with responsive dimensions
    var viewer = new ROS3D.Viewer({
      divID: "map",
      width: containerWidth,
      height: containerHeight,
      antialias: true,
    });

    // Setup the map client using ROS3D.
    var gridClient = new ROS3D.OccupancyGridClient({
      ros: ros,
      rootObject: viewer.scene,
    });

    // Setup robot marker as an arrow for odometry
    var robotMarker = new ROS3D.Arrow({
      length: 0.5,
      headLength: 0.25,
      shaftDiameter: 0.4,
      headDiameter: 0.4,
      material: new THREE.MeshBasicMaterial({ color: 0xff0000 }),
    });
    viewer.scene.add(robotMarker);

    var odomSub = new ROSLIB.Topic({
      ros: ros,
      name: "/odometry/filtered",
      messageType: "nav_msgs/Odometry",
      throttle_rate: 100  // Throttle to 10 Hz to reduce CPU usage
    });

    odomSub.subscribe(function (message) {
      // Update robot position
      robotMarker.position.x = message.pose.pose.position.x;
      robotMarker.position.y = message.pose.pose.position.y;
      robotMarker.position.z = 0.1;

      // Convert robot orientation quaternion to direction vector
      var q = message.pose.pose.orientation;
      var quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);

      // Get the forward direction: Robot forward is +X in ROS
      var direction = new THREE.Vector3(1, 0, 0);
      direction.applyQuaternion(quaternion);

      // Set arrow direction using native method
      robotMarker.setDirection(direction);
    });

    // Handle window resize to make the viewer responsive
    window.addEventListener("resize", function () {
      var newWidth = mapContainer.offsetWidth;
      var newHeight = mapContainer.offsetHeight;

      viewer.camera.aspect = newWidth / newHeight;
      viewer.camera.updateProjectionMatrix();
      viewer.renderer.setSize(newWidth, newHeight);
    });
  }, 1000);
};
