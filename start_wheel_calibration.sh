#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==> 1. Stopping robot containers..."
./start_robot.sh down || true

# echo "==> 2. Flashing calibration firmware to wheels board..."
# ./run_firmware.sh flash wheels_calibration "$@"

echo "==> 3. Starting micro-ROS agent & wheel calibration routine..."
docker compose -f ros2_ws/docker-compose.yml run --rm ros2_diff_robot bash -c \
  "source /opt/ros/humble/setup.bash && source /uros_ws/install/local_setup.bash && ros2 run micro_ros_agent micro_ros_agent multiserial --devs \"/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyACM3\" -b 115200 & sleep 5 && python3 /ros2_ws/helpers/wheel_calibration/calibrate.py"
