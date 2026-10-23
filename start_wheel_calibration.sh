#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> 1. Stopping robot containers..."
./start_robot.sh down || true

echo "==> 2. Starting Zenoh router container..."
docker compose -f ros2_ws/docker-compose.yml up -d ros2_zenoh_router
sleep 3

echo "==> 3. Running wheel calibration routine via ROS 2 Lyrical (rmw_zenoh_cpp)..."
docker compose -f ros2_ws/docker-compose.yml run --rm ros2_diff_robot bash -c \
  "source /opt/ros/lyrical/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && python3 /ros2_ws/helpers/wheel_calibration/calibrate.py"
