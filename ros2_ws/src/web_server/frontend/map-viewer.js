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

    // 1. Configure TF Client
    // It will listen for transforms relative to the 'map' frame
    var tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: '/map' // Important: coordinates relative to the Map
    });

    // 2. Create the arrow (robot marker)
    var robotMarker = new ROS3D.Arrow({
      length: 0.5,
      headLength: 0.25,
      shaftDiameter: 0.1, // Slightly thinner for neatness
      headDiameter: 0.2,
      material: new THREE.MeshBasicMaterial({ color: 0xff0000 }),
    });

    // Wrap marker in a container to apply offset
    // SceneNode overwrites the position of its direct child, so we need a container.
    var container = new THREE.Object3D();
    container.add(robotMarker);
    robotMarker.position.z = 1.0; // Lift above map

    // 3. Create SceneNode
    // This object will move and rotate everything inside it
    // based on the robot's TF transform.
    var robotNode = new ROS3D.SceneNode({
      tfClient: tfClient,
      frameID: 'base_link',
      object: container
    });

    // Add node to scene
    viewer.scene.add(robotNode);

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
