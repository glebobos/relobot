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
    // Expose for robot-control.js action clients
    window.rosConn = ros;

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

    var previewLayer = new THREE.Group();
    var mapPolygonLayer = new THREE.Group();
    viewer.scene.add(previewLayer);
    viewer.scene.add(mapPolygonLayer);

    function clearGroup(group) {
      while (group.children.length > 0) {
        var child = group.children.pop();
        group.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
      }
    }

    function renderPreviewPath(pathMsg) {
      clearGroup(previewLayer);
      if (!pathMsg || !pathMsg.poses || pathMsg.poses.length === 0) {
        return;
      }

      var pathPoints = pathMsg.poses.map(function (poseStamped) {
        return new THREE.Vector3(
          poseStamped.pose.position.x,
          poseStamped.pose.position.y,
          0.1
        );
      });
      var pathGeometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
      var pathMaterial = new THREE.LineBasicMaterial({ color: 0xd17a00 });
      previewLayer.add(new THREE.Line(pathGeometry, pathMaterial));
    }

    function renderMapPolygon(msg) {
      clearGroup(mapPolygonLayer);
      if (!msg || !msg.polygon || !msg.polygon.points || msg.polygon.points.length < 3) {
        return;
      }
      var rawPts = msg.polygon.points.map(function (p) {
        return new THREE.Vector3(p.x, p.y, 0.05);
      });

      // Sanity-check: filter out extreme outlier vertices that indicate a
      // degenerate bounding-rectangle polygon from approxPolyDP. Compute the
      // median of x and y across all points, then reject any point whose
      // distance from the median centre exceeds 3× the IQR-based spread.
      var xs = rawPts.map(function (v) { return v.x; }).slice().sort(function (a, b) { return a - b; });
      var ys = rawPts.map(function (v) { return v.y; }).slice().sort(function (a, b) { return a - b; });
      var n = xs.length;
      var medX = xs[Math.floor(n / 2)];
      var medY = ys[Math.floor(n / 2)];
      var spreadX = xs[Math.floor(n * 0.75)] - xs[Math.floor(n * 0.25)] || 1;
      var spreadY = ys[Math.floor(n * 0.75)] - ys[Math.floor(n * 0.25)] || 1;
      var maxSpread = Math.max(spreadX, spreadY);
      var pts = rawPts.filter(function (v) {
        var dx = Math.abs(v.x - medX);
        var dy = Math.abs(v.y - medY);
        return dx < maxSpread * 5 && dy < maxSpread * 5;
      });

      if (pts.length < 3) {
        console.warn('renderMapPolygon: all points filtered as outliers, skipping draw.');
        return;
      }

      // The backend already appends the closing point (first === last).
      // Only add a synthetic close if the last point differs from the first.
      var first = pts[0], last = pts[pts.length - 1];
      if (Math.abs(first.x - last.x) > 1e-4 || Math.abs(first.y - last.y) > 1e-4) {
        pts.push(first.clone());
      }

      var geo = new THREE.BufferGeometry().setFromPoints(pts);
      var mat = new THREE.LineBasicMaterial({ color: 0x1a6fcc });
      mapPolygonLayer.add(new THREE.Line(geo, mat));
    }

    var previewPathTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/coverage/preview_path',
      messageType: 'nav_msgs/Path'
    });
    previewPathTopic.subscribe(renderPreviewPath);

    var polygonActiveTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/coverage/polygon_active',
      messageType: 'geometry_msgs/PolygonStamped'
    });
    polygonActiveTopic.subscribe(renderMapPolygon);

    window.coverageUi = {
      clearMapPolygon: function () { clearGroup(mapPolygonLayer); },
    };

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
