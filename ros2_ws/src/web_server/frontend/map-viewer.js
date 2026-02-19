window.onload = function () {
  // Give ROS bridge some time to startup
  setTimeout(() => {
    if (!window.ROS3D) {
      console.error("ROS3D is not available!");
      return;
    }

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

    // Setup robot marker as an arrow for odometry
    var robotMarker = new ROS3D.Arrow({
      length: 0.5,
      headLength: 0.25,
      shaftDiameter: 0.4,
      headDiameter: 0.4,
      material: new THREE.MeshBasicMaterial({ color: 0xff0000 }),
    });
    viewer.scene.add(robotMarker);

    // --- ROS connection with auto-reconnect ---
    var ros = null;
    var gridClient = null;
    var odomSub = null;

    function connectROS() {
      ros = new ROSLIB.Ros({
        url: "ws://" + window.location.hostname + ":9090",
      });

      ros.on("connection", function () {
        console.log("Connected to websocket server.");
        setupSubscriptions();
      });

      ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
      });

      ros.on("close", function () {
        console.log("Connection to websocket server closed. Reconnecting in 3s...");
        cleanupSubscriptions();
        setTimeout(connectROS, 3000);
      });
    }

    function setupSubscriptions() {
      // Setup the map client
      gridClient = new ROS3D.OccupancyGridClient({
        ros: ros,
        rootObject: viewer.scene,
      });

      // Setup odometry subscription
      odomSub = new ROSLIB.Topic({
        ros: ros,
        name: "/odometry/filtered",
        messageType: "nav_msgs/Odometry",
        throttle_rate: 100,
      });

      odomSub.subscribe(function (message) {
        robotMarker.position.x = message.pose.pose.position.x;
        robotMarker.position.y = message.pose.pose.position.y;
        robotMarker.position.z = 0.1;

        var q = message.pose.pose.orientation;
        var quaternion = new THREE.Quaternion(q.x, q.y, q.z, q.w);
        var direction = new THREE.Vector3(1, 0, 0);
        direction.applyQuaternion(quaternion);
        robotMarker.setDirection(direction);
      });
    }

    function cleanupSubscriptions() {
      if (odomSub) {
        odomSub.unsubscribe();
        odomSub = null;
      }
      if (gridClient) {
        // Remove old grid from scene
        gridClient.rootObject = null;
        gridClient = null;
      }
    }

    // Start initial connection
    connectROS();

    // Reconnect on visibility change (tab/app switch)
    document.addEventListener("visibilitychange", function () {
      if (document.visibilityState === "visible" && ros) {
        if (!ros.isConnected) {
          console.log("[MapViewer] Page visible, ROS disconnected – reconnecting...");
          ros.close(); // ensure clean state
          connectROS();
        }
      }
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
