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
    var obstacleLayer = new THREE.Group();
    var zoneLayer = new THREE.Group();   // green rectangle for custom zone
    viewer.scene.add(previewLayer);
    viewer.scene.add(mapPolygonLayer);
    viewer.scene.add(obstacleLayer);
    viewer.scene.add(zoneLayer);

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

      var pathPoints = pathMsg.poses
        .filter(function (poseStamped) {
          var x = poseStamped.pose.position.x;
          var y = poseStamped.pose.position.y;
          return Number.isFinite(x) && Number.isFinite(y);
        })
        .map(function (poseStamped) {
          return new THREE.Vector3(
            poseStamped.pose.position.x,
            poseStamped.pose.position.y,
            0.1
          );
        });
      if (pathPoints.length < 2) {
        return;
      }
      var pathGeometry = new THREE.BufferGeometry().setFromPoints(pathPoints);
      var pathMaterial = new THREE.LineBasicMaterial({ color: 0xd17a00 });
      previewLayer.add(new THREE.Line(pathGeometry, pathMaterial));
    }

    function renderMapPolygon(msg) {
      clearGroup(mapPolygonLayer);
      if (!msg || !msg.polygon || !msg.polygon.points || msg.polygon.points.length < 3) {
        return;
      }
      // Reject points with non-finite coordinates (NaN/Infinity from rosbridge
      // JSON edge cases) to prevent BufferGeometry from producing a max-size bbox.
      var pts = msg.polygon.points
        .filter(function (p) { return Number.isFinite(p.x) && Number.isFinite(p.y); })
        .map(function (p) { return new THREE.Vector3(p.x, p.y, 0.05); });

      if (pts.length < 3) {
        console.warn('renderMapPolygon: no finite points, skipping draw.');
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

    function renderObstacles(msg) {
      clearGroup(obstacleLayer);
      if (!msg || !msg.data) return;
      var polygons;
      try { polygons = JSON.parse(msg.data); } catch (e) { return; }
      if (!Array.isArray(polygons)) return;
      polygons.forEach(function (poly) {
        if (!Array.isArray(poly) || poly.length < 3) return;
        var pts = poly
          .filter(function (p) { return Number.isFinite(p.x) && Number.isFinite(p.y); })
          .map(function (p) { return new THREE.Vector3(p.x, p.y, 0.06); });
        if (pts.length < 3) return;
        // close the loop
        if (Math.abs(pts[0].x - pts[pts.length - 1].x) > 1e-4 ||
            Math.abs(pts[0].y - pts[pts.length - 1].y) > 1e-4) {
          pts.push(pts[0].clone());
        }
        var geo = new THREE.BufferGeometry().setFromPoints(pts);
        var mat = new THREE.LineBasicMaterial({ color: 0xff6600 });
        obstacleLayer.add(new THREE.Line(geo, mat));
      });
    }

    // ── Zone rect rendered in Three.js scene (green) ─────────────────────────
    function renderZoneRect(corners) {
      clearGroup(zoneLayer);
      if (!corners || corners.length < 4) return;
      var pts = corners.map(function (c) {
        return new THREE.Vector3(c.x, c.y, 0.12);
      });
      pts.push(pts[0].clone()); // close the loop
      var geo = new THREE.BufferGeometry().setFromPoints(pts);
      var mat = new THREE.LineBasicMaterial({ color: 0x00e676 });
      zoneLayer.add(new THREE.Line(geo, mat));
    }

    // ── 2-D overlay canvas for live rectangle drag preview ───────────────────
    var overlayCanvas = document.getElementById('zone-draw-canvas');
    var overlayCtx = overlayCanvas && overlayCanvas.getContext('2d');

    function resizeOverlay() {
      if (!overlayCanvas) return;
      var rect = mapContainer.getBoundingClientRect();
      overlayCanvas.width  = rect.width;
      overlayCanvas.height = rect.height;
    }
    resizeOverlay();

    function clearOverlay() {
      if (!overlayCtx) return;
      overlayCtx.clearRect(0, 0, overlayCanvas.width, overlayCanvas.height);
    }

    function drawOverlayRect(x1, y1, x2, y2) {
      if (!overlayCtx) return;
      clearOverlay();
      overlayCtx.strokeStyle = '#00e676';
      overlayCtx.lineWidth   = 2;
      overlayCtx.setLineDash([6, 3]);
      overlayCtx.strokeRect(
        Math.min(x1, x2), Math.min(y1, y2),
        Math.abs(x2 - x1), Math.abs(y2 - y1)
      );
    }

    // ── Screen pixel → ROS map-frame world coordinate ───────────────────────
    function screenToWorld(screenX, screenY) {
      var rect = mapContainer.getBoundingClientRect();
      var ndcX =  ((screenX - rect.left) / rect.width)  * 2 - 1;
      var ndcY = -((screenY - rect.top)  / rect.height) * 2 + 1;
      var raycaster = new THREE.Raycaster();
      raycaster.setFromCamera({ x: ndcX, y: ndcY }, viewer.camera);
      var plane  = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
      var target = new THREE.Vector3();
      var hit    = raycaster.ray.intersectPlane(plane, target);
      return hit ? { x: target.x, y: target.y } : null;
    }

    // ── Camera state save/restore for top-down snap ─────────────────────────
    var savedCameraState = null;

    function snapCameraTopDown() {
      if (!viewer || !viewer.camera || !viewer.cameraControls) return;
      var cam      = viewer.camera;
      var controls = viewer.cameraControls;
      var tgt      = controls.target || controls.center;
      if (!tgt) { console.warn('[ZoneDraw] controls target/center is undefined'); return; }

      // Save full camera state so we can restore it exactly
      savedCameraState = {
        position:   cam.position.clone(),
        up:         cam.up.clone(),
        quaternion: cam.quaternion.clone(),
        target:     tgt.clone(),
      };
      // Keep the same zoom distance; place camera directly above target.
      var dist = cam.position.distanceTo(tgt);
      cam.position.set(tgt.x, tgt.y, tgt.z + dist);
      cam.up.set(0, -1, 0);        // Y = south-up (180 degrees rotated ROS map frame)
      cam.lookAt(tgt.x, tgt.y, tgt.z);
      // Disable OrbitControls so they cannot recalculate camera position from
      // their internal spherical coordinates on the next mouse/touch event,
      // which would override the top-down view we just set.
      controls.enabled = false;
      // Do NOT call controls.update() here — it re-derives position from
      // internal spherical angles and would reintroduce the azimuth rotation.
    }

    function restoreCamera() {
      if (!savedCameraState || !viewer || !viewer.camera || !viewer.cameraControls) return;
      var cam      = viewer.camera;
      var controls = viewer.cameraControls;
      cam.position.copy(savedCameraState.position);
      cam.up.copy(savedCameraState.up);
      cam.quaternion.copy(savedCameraState.quaternion);
      if (controls.target) {
        controls.target.copy(savedCameraState.target);
      } else if (controls.center) {
        controls.center.copy(savedCameraState.target);
      }
      controls.enabled = true;
      // update() re-derives OrbitControls' internal spherical coords from the
      // restored camera position so future orbit interactions start correctly.
      controls.update();
      savedCameraState = null;
    }

    // ── Zone-draw state ──────────────────────────────────────────────────────
    var zoneDrawMode     = false;
    var zoneDrawing      = false;
    var zoneDragStart    = null;   // screen coords of drag origin
    var activeZoneCorners = null;  // last committed world-frame rectangle corners

    function setZoneDrawMode(enabled) {
      zoneDrawMode = enabled;
      if (overlayCanvas) {
        overlayCanvas.style.pointerEvents = enabled ? 'auto' : 'none';
      }
      mapContainer.style.cursor = enabled ? 'crosshair' : '';
      if (enabled) {
        snapCameraTopDown();   // snap to top-down; preserves zoom and pan
      } else {
        restoreCamera();       // restore previous 3D angle
        clearOverlay();
        zoneDrawing   = false;
        zoneDragStart = null;
      }
    }

    function commitZone(x1Screen, y1Screen, x2Screen, y2Screen) {
      var tl = screenToWorld(Math.min(x1Screen, x2Screen), Math.min(y1Screen, y2Screen));
      var br = screenToWorld(Math.max(x1Screen, x2Screen), Math.max(y1Screen, y2Screen));
      if (!tl || !br) { console.warn('[ZoneDraw] screenToWorld returned null'); return; }

      // Build axis-aligned rectangle (4 corners, CCW in ROS map frame)
      var corners = [
        { x: tl.x, y: tl.y },
        { x: br.x, y: tl.y },
        { x: br.x, y: br.y },
        { x: tl.x, y: br.y },
      ];
      activeZoneCorners = corners;
      clearOverlay();
      renderZoneRect(corners);

      if (window.coverageUi && typeof window.coverageUi._publishCommand === 'function') {
        window.coverageUi._publishCommand('set_zone:' + JSON.stringify(corners));
      }
      console.log('[ZoneDraw] Zone committed:', corners);

      // Auto-exit draw mode and restore camera view/controls
      setZoneDrawMode(false);

      // Update button text and styling to indicate draw mode is no longer active
      var drawBtn = document.getElementById('coverage-draw-zone-btn');
      if (drawBtn) {
        drawBtn.textContent = 'Draw Zone';
        drawBtn.classList.remove('coverage-btn-zone-active');
      }
    }

    if (overlayCanvas) {
      // ---- mouse ----
      overlayCanvas.addEventListener('mousedown', function (e) {
        if (!zoneDrawMode) return;
        zoneDrawing   = true;
        zoneDragStart = { x: e.clientX, y: e.clientY };
        e.preventDefault();
      });
      overlayCanvas.addEventListener('mousemove', function (e) {
        if (!zoneDrawMode || !zoneDrawing || !zoneDragStart) return;
        var r = mapContainer.getBoundingClientRect();
        drawOverlayRect(
          zoneDragStart.x - r.left, zoneDragStart.y - r.top,
          e.clientX       - r.left, e.clientY       - r.top
        );
        e.preventDefault();
      });
      overlayCanvas.addEventListener('mouseup', function (e) {
        if (!zoneDrawMode || !zoneDrawing || !zoneDragStart) return;
        zoneDrawing = false;
        var dx = Math.abs(e.clientX - zoneDragStart.x);
        var dy = Math.abs(e.clientY - zoneDragStart.y);
        if (dx > 10 && dy > 10) {
          commitZone(zoneDragStart.x, zoneDragStart.y, e.clientX, e.clientY);
        } else {
          clearOverlay();
        }
        zoneDragStart = null;
        e.preventDefault();
      });

      // ---- touch ----
      overlayCanvas.addEventListener('touchstart', function (e) {
        if (!zoneDrawMode) return;
        var t = e.touches[0];
        zoneDrawing   = true;
        zoneDragStart = { x: t.clientX, y: t.clientY };
        e.preventDefault();
      }, { passive: false });
      overlayCanvas.addEventListener('touchmove', function (e) {
        if (!zoneDrawMode || !zoneDrawing || !zoneDragStart) return;
        var t = e.touches[0];
        var r = mapContainer.getBoundingClientRect();
        drawOverlayRect(
          zoneDragStart.x - r.left, zoneDragStart.y - r.top,
          t.clientX       - r.left, t.clientY       - r.top
        );
        e.preventDefault();
      }, { passive: false });
      overlayCanvas.addEventListener('touchend', function (e) {
        if (!zoneDrawMode || !zoneDrawing || !zoneDragStart) return;
        zoneDrawing = false;
        var t  = e.changedTouches[0];
        var dx = Math.abs(t.clientX - zoneDragStart.x);
        var dy = Math.abs(t.clientY - zoneDragStart.y);
        if (dx > 10 && dy > 10) {
          commitZone(zoneDragStart.x, zoneDragStart.y, t.clientX, t.clientY);
        } else {
          clearOverlay();
        }
        zoneDragStart = null;
        e.preventDefault();
      }, { passive: false });
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
      messageType: 'geometry_msgs/PolygonStamped',
      durability: 'transient_local',
      reliability: 'reliable'
    });
    polygonActiveTopic.subscribe(renderMapPolygon);

    var obstaclesTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/coverage/obstacles_active',
      messageType: 'std_msgs/String',
      durability: 'transient_local',
      reliability: 'reliable'
    });
    obstaclesTopic.subscribe(renderObstacles);

    window.coverageUi = {
      clearMapPolygon: function () {
        clearGroup(mapPolygonLayer);
        clearGroup(obstacleLayer);
      },

      /** Injected by robot-control.js so the map viewer can publish ROS commands */
      _publishCommand: null,

      /** Toggle rectangle-draw mode. Returns new mode state (true = drawing). */
      startZoneDraw: function () {
        setZoneDrawMode(!zoneDrawMode);
        return zoneDrawMode;
      },

      /** Clear the drawn rectangle and revert to SLAM boundary. */
      clearZone: function () {
        clearGroup(zoneLayer);
        clearOverlay();
        activeZoneCorners = null;
        setZoneDrawMode(false);
        if (window.coverageUi._publishCommand) {
          window.coverageUi._publishCommand('clear_zone');
        }
      },

      isZoneDrawMode: function () { return zoneDrawMode; },
      hasActiveZone:  function () { return activeZoneCorners !== null; },
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
      var newWidth  = mapContainer.offsetWidth;
      var newHeight = mapContainer.offsetHeight;

      viewer.camera.aspect = newWidth / newHeight;
      viewer.camera.updateProjectionMatrix();
      viewer.renderer.setSize(newWidth, newHeight);
      resizeOverlay();
    });
  }, 1000);
};
