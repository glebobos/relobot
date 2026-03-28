docker build -t rviz2 .
export DISPLAY=:0
xhost +local:docker
export HOST_IP=$(hostname -I | awk '{print $1}')
docker run -ti     -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/shm:/dev/shm -e DISPLAY=:0 -e QT_QPA_PLATFORM=xcb -e ROS_SUPER_CLIENT=True -e ROS_DISCOVERY_SERVER=$HOST_IP:11811 --network=host   --rm rviz2:latest rviz2

Coverage testing in RViz:

1. Start the robot stack and the web frontend.
2. Launch RViz with the command above or by running `./start_rviz2.sh` from the repo root.
3. Set `Fixed Frame` to `map`.
4. Add these displays:
	- `Path` on `/coverage/preview_path`
	- `Path` on `/coverage_server/coverage_plan`
	- `Polygon` on `/coverage_server/field_boundary`
	- `Polygon` on `/coverage_server/planning_field`
	- `Marker` on `/coverage_server/swaths`
	- `Polygon` on `/coverage/polygon_active`
5. In the web UI, click points on the map to create a polygon. Right-click removes the last point.
6. Click `Preview Coverage` and confirm the orange preview path and blue swaths appear in RViz.
7. Click `Execute Coverage` to send the cached preview to Nav2 `follow_path`.
8. Click `Stop Coverage` to cancel either planning or execution.

Headless preview test:

Run this inside the Nav2 container or an environment with the workspace sourced:

`ros2 run nav2 coverage_preview_test`

That publishes a simple rectangular polygon on `/coverage/polygon` and asks the coverage manager to preview it. A successful run ends when `/coverage/status` reports `preview_ready`.