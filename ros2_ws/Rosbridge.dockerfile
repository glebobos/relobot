FROM ros:humble-ros-base

# Install rosbridge server, build dependencies for Rust node, and standard messages
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    clang \
    libclang-dev \
    ros-humble-rosbridge-server \
    ros-humble-nav2-msgs \
    ros-humble-opennav-docking-msgs \
    ros-humble-slam-toolbox && \
    rm -rf /var/lib/apt/lists/*

# Install Rust toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Workspace is volume-mounted at /ros2_ws via docker-compose
WORKDIR /ros2_ws

# Startup script to build / run both bridges
RUN echo '#!/bin/bash\n\
set -e\n\
trap "kill \$(jobs -p) 2>/dev/null; wait" SIGTERM SIGINT EXIT\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ] && [ ! -d "/ros2_ws/install/explore_lite_msgs" ]; then\n\
  colcon build --packages-select explore_lite_msgs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release\n\
fi\n\
source install/setup.bash\n\
\n\
# Build Rust bridge\n\
if [ "$DEV" = "true" ]; then\n\
  echo "Building rosbridge_rust..."\n\
  cd /ros2_ws/src/rosbridge_rust\n\
  cargo build --release\n\
  cd /ros2_ws\n\
fi\n\
\n\
# Launch Python rosbridge on port 9091 in the background\n\
ros2 launch /ros2_ws/src/rosbridge_webvideo_launch.py &\n\
PYTHON_PID=$!\n\
\n\
# Wait for Python backend to initialize\n\
for i in $(seq 1 30); do\n\
  curl -s 127.0.0.1:9091 >/dev/null && break\n\
  sleep 1\n\
done\n\
\n\
# Run the Rust WebSocket server/proxy on port 9090 in the background\n\
/ros2_ws/src/rosbridge_rust/target/release/rosbridge_rust &\n\
RUST_PID=$!\n\
\n\
# Wait for either process to exit\n\
wait -n\n\
\n\
# If one exited, kill the other and exit\n\
kill $PYTHON_PID $RUST_PID 2>/dev/null || true\n\
exit 1' > /start.sh && \
chmod +x /start.sh

CMD ["/start.sh"]
