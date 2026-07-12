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
\n\
# Wait for Python backend to initialize\n\
sleep 2\n\
\n\
# Run the Rust WebSocket server/proxy on port 9090 in foreground\n\
/ros2_ws/src/rosbridge_rust/target/release/rosbridge_rust' > /start.sh && \
chmod +x /start.sh

CMD ["/start.sh"]
