#!/bin/bash
#
# RViz2 Docker Launcher Script
# Starts RViz2 in a Docker container with X11/WSLg forwarding and ROS2 configuration
#

set -e

RVIZ_ARGS=()
SIM_MODE=false

for arg in "$@"; do
    if [ "$arg" == "--sim" ] || [ "$arg" == "-s" ]; then
        SIM_MODE=true
    else
        RVIZ_ARGS+=("$arg")
    fi
done

echo "Starting RViz2 Docker container..."

# Configure display and permissions
if command -v xhost &>/dev/null; then
    xhost +local:docker 2>/dev/null || true
    xhost + 2>/dev/null || true
fi

DISPLAY="${DISPLAY:-:0}"
WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Prepare device / volume mounts
GPU_OPTS=()
if [ -e "/dev/dxg" ]; then
    GPU_OPTS+=(--device /dev/dxg)
fi
if [ -d "/usr/lib/wsl/lib" ]; then
    GPU_OPTS+=(-v /usr/lib/wsl/lib:/usr/lib/wsl/lib:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib)
fi
if [ -d "/mnt/wslg" ]; then
    GPU_OPTS+=(-v /mnt/wslg:/mnt/wslg:ro)
fi

XAUTH_OPTS=()
if [ -f "$HOME/.Xauthority" ]; then
    XAUTH_OPTS+=(-v "$HOME/.Xauthority:/root/.Xauthority:ro" -e XAUTHORITY=/root/.Xauthority)
fi

RVIZ_CONFIG_OPTS=()
if [ "$SIM_MODE" = true ]; then
    echo "Running RViz2 with Simulation Configuration..."
    RVIZ_CONFIG_OPTS+=(-d /ros2_ws/src/relobot_gazebo/rviz/sim.rviz)
fi

# Run RViz2 in Docker container with all necessary configurations
echo "Launching RViz2..."
docker run -ti \
    --rm \
    --network=host \
    --ipc=host \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/shm:/dev/shm \
    -v "$SCRIPT_DIR/.rviz2:/root/.rviz2" \
    -v "$SCRIPT_DIR/ros2_ws:/ros2_ws" \
    "${GPU_OPTS[@]}" \
    "${XAUTH_OPTS[@]}" \
    -e DISPLAY="$DISPLAY" \
    -e WAYLAND_DISPLAY="$WAYLAND_DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e FASTDDS_DEFAULT_PROFILES_FILE=/ros2_ws/fastdds_localhost.xml \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/fastdds_localhost.xml \
    ros2_ws-ros2_gazebo_sim:latest \
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash 2>/dev/null || true && rviz2 ${RVIZ_CONFIG_OPTS[*]} ${RVIZ_ARGS[*]}"

echo "RViz2 container stopped."