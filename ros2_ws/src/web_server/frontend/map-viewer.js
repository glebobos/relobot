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

    var coverageLayer = new THREE.Group();
    var previewLayer = new THREE.Group();
    viewer.scene.add(coverageLayer);
    viewer.scene.add(previewLayer);

    var polygonPoints = [];
    var clickStart = null;
    var raycaster = new THREE.Raycaster();
    var pointer = new THREE.Vector2();
    var groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

    function clearGroup(group) {
      while (group.children.length > 0) {
        var child = group.children.pop();
        group.remove(child);
        if (child.geometry) child.geometry.dispose();
        if (child.material) child.material.dispose();
      }
    }

    function renderPolygon() {
      clearGroup(coverageLayer);
      if (polygonPoints.length === 0) {
        return;
      }

      var material = new THREE.LineBasicMaterial({ color: 0x0b8457 });
      var points = polygonPoints.map(function (point) {
        return new THREE.Vector3(point.x, point.y, 0.06);
      });
      if (points.length >= 3) {
        points.push(points[0].clone());
      }
      if (points.length >= 2) {
        var lineGeometry = new THREE.BufferGeometry().setFromPoints(points);
        coverageLayer.add(new THREE.Line(lineGeometry, material));
      }

      polygonPoints.forEach(function (point, index) {
        var sphere = new THREE.Mesh(
          new THREE.SphereGeometry(index === 0 ? 0.09 : 0.07, 16, 16),
          new THREE.MeshBasicMaterial({ color: index === 0 ? 0x146c43 : 0x20a36c })
        );
        sphere.position.set(point.x, point.y, 0.08);
        coverageLayer.add(sphere);
      });
    }

    function notifyPolygonChanged() {
      window.dispatchEvent(new CustomEvent('coverage-polygon-changed', {
        detail: { pointCount: polygonPoints.length }
      }));
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

    function buildPolygonMessage() {
      return {
        header: { frame_id: 'map' },
        polygon: {
          points: polygonPoints.map(function (point) {
            return { x: point.x, y: point.y, z: 0.0 };
          })
        }
      };
    }

    var coveragePolygonTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/coverage/polygon',
      messageType: 'geometry_msgs/PolygonStamped'
    });

    var previewPathTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/coverage/preview_path',
      messageType: 'nav_msgs/Path'
    });
    previewPathTopic.subscribe(renderPreviewPath);

    function publishPolygon() {
      coveragePolygonTopic.publish(buildPolygonMessage());
    }

    function clearPolygon() {
      polygonPoints = [];
      renderPolygon();
      publishPolygon();
      renderPreviewPath(null);
      notifyPolygonChanged();
    }

    function popPolygonPoint() {
      if (polygonPoints.length === 0) {
        return;
      }
      polygonPoints.pop();
      renderPolygon();
      publishPolygon();
      notifyPolygonChanged();
    }

    function hasPolygon() {
      return polygonPoints.length >= 3;
    }

    window.coverageUi = {
      clearPolygon: clearPolygon,
      hasPolygon: hasPolygon,
      publishPolygon: publishPolygon,
    };

    function getEventClientPoint(event) {
      if (event.changedTouches && event.changedTouches.length > 0) {
        return {
          x: event.changedTouches[0].clientX,
          y: event.changedTouches[0].clientY,
        };
      }
      if (event.touches && event.touches.length > 0) {
        return {
          x: event.touches[0].clientX,
          y: event.touches[0].clientY,
        };
      }
      return {
        x: event.clientX,
        y: event.clientY,
      };
    }

    function eventToMapPoint(event) {
      var clientPoint = getEventClientPoint(event);
      var rect = viewer.renderer.domElement.getBoundingClientRect();
      pointer.x = ((clientPoint.x - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((clientPoint.y - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, viewer.camera);
      var intersection = new THREE.Vector3();
      if (!raycaster.ray.intersectPlane(groundPlane, intersection)) {
        return null;
      }
      return { x: intersection.x, y: intersection.y };
    }

    function handleMapClick(event) {
      if (!clickStart) {
        return;
      }
      var clientPoint = getEventClientPoint(event);
      var moved = Math.hypot(clientPoint.x - clickStart.x, clientPoint.y - clickStart.y);
      clickStart = null;
      if (moved > 5) {
        return;
      }
      var point = eventToMapPoint(event);
      if (!point) {
        return;
      }
      polygonPoints.push(point);
      renderPolygon();
      publishPolygon();
      notifyPolygonChanged();
    }

    viewer.renderer.domElement.addEventListener('mousedown', function (event) {
      clickStart = { x: event.clientX, y: event.clientY };
    });
    viewer.renderer.domElement.addEventListener('mouseup', handleMapClick);
    viewer.renderer.domElement.addEventListener('touchstart', function (event) {
      var clientPoint = getEventClientPoint(event);
      clickStart = { x: clientPoint.x, y: clientPoint.y };
    }, { passive: true });
    viewer.renderer.domElement.addEventListener('touchend', function (event) {
      handleMapClick(event);
    }, { passive: true });
    viewer.renderer.domElement.addEventListener('contextmenu', function (event) {
      event.preventDefault();
      popPolygonPoint();
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
