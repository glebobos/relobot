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

    // Setup robot marker as a polygon for footprint
    var robotFootprint = null;

    var footprintSub = new ROSLIB.Topic({
      ros: ros,
      name: "/local_costmap/published_footprint",
      messageType: "geometry_msgs/PolygonStamped",
      throttle_rate: 100  // Throttle to 10 Hz to reduce CPU usage
    });

    footprintSub.subscribe(function (message) {
      // Remove old footprint if it exists
      if (robotFootprint) {
        viewer.scene.remove(robotFootprint);
      }

      // Create polygon shape from footprint points
      var points = message.polygon.points.map(function(p) {
        return new THREE.Vector2(p.x, p.y);
      });

      // Create shape and geometry
      var shape = new THREE.Shape(points);
      var geometry = new THREE.ShapeGeometry(shape);
      var material = new THREE.MeshBasicMaterial({ 
        color: 0xff0000, 
        side: THREE.DoubleSide 
      });
      
      robotFootprint = new THREE.Mesh(geometry, material);
      robotFootprint.position.z = 0.1;
      viewer.scene.add(robotFootprint);
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