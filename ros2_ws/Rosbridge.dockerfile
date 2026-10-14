FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Install rosbridge server, build dependencies for Rust node, and standard messages
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    clang \
    libclang-dev \
    ros-lyrical-rmw-zenoh-cpp \
    ros-lyrical-rosbridge-server \
    ros-lyrical-nav2-msgs \
    ros-lyrical-slam-toolbox \
    ros-lyrical-web-video-server && \
    rm -rf /var/lib/apt/lists/*

# Install Rust toolchain
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Workspace is volume-mounted at /ros2_ws via docker-compose
WORKDIR /ros2_ws

# Startup script to build / run both bridges
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -e' \
    'trap "kill \$(jobs -p) 2>/dev/null; wait" SIGTERM SIGINT EXIT' \
    'source /opt/ros/lyrical/setup.bash' \
    'cd /ros2_ws' \
    'if [ "$DEV" = "true" ] && [ ! -d "/ros2_ws/install/explore_lite_msgs" ]; then' \
    '  colcon build --packages-select explore_lite_msgs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release' \
    'fi' \
    'if [ -f install/setup.bash ]; then' \
    '  source install/setup.bash' \
    'fi' \
    '' \
    '# Test & build Rust bridge' \
    'if [ "$DEV" = "true" ]; then' \
    '  echo "Testing & building rosbridge_rust..."' \
    '  cd /ros2_ws/src/rosbridge_rust' \
    '  cargo test' \
    '  cargo build --release' \
    '  cd /ros2_ws' \
    'fi' \
    '' \
    '# Launch Python rosbridge on port 9091 in the background' \
    'ros2 launch /ros2_ws/src/rosbridge_webvideo_launch.py &' \
    'PYTHON_PID=$!' \
    '' \
    '# Wait for Python backend to initialize' \
    'for i in $(seq 1 30); do' \
    '  curl -s 127.0.0.1:9091 >/dev/null && break' \
    '  sleep 1' \
    'done' \
    '' \
    '# Run the Rust WebSocket server/proxy on port 9090 in the background' \
    '/ros2_ws/src/rosbridge_rust/target/release/rosbridge_rust &' \
    'RUST_PID=$!' \
    '' \
    '# Wait for either process to exit' \
    'wait -n' \
    '' \
    '# If one exited, kill the other and exit' \
    'kill $PYTHON_PID $RUST_PID 2>/dev/null || true' \
    'exit 1' > /start.sh && \
    chmod +x /start.sh

CMD ["/start.sh"]
