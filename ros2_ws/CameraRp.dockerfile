FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Install build dependencies for libcamera + camera_ros
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    git \
    meson \
    ninja-build \
    pkg-config \
    libgnutls28-dev \
    openssl \
    libboost-dev \
    python3-yaml \
    python3-ply \
    python3-jinja2 \
    nlohmann-json3-dev \
    ros-lyrical-rmw-zenoh-cpp \
    ros-lyrical-camera-info-manager \
    ros-lyrical-cv-bridge \
    ros-lyrical-rclcpp-components \
    ros-lyrical-sensor-msgs \
    ros-lyrical-image-view \
    ros-lyrical-web-video-server \
    ros-lyrical-image-proc \
    ros-lyrical-compressed-image-transport \
    ros-lyrical-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

# Build libpisp (required for RPi 5 / BCM2712)
RUN git clone https://github.com/raspberrypi/libpisp.git && \
    cd libpisp && \
    meson setup build --prefix=/usr && \
    ninja -C build install && \
    cd .. && rm -rf libpisp

# Build libcamera (Raspberry Pi fork) with PiSP support explicitly
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson setup build --prefix=/usr -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled && \
    ninja -C build install && \
    cd .. && rm -rf libcamera

# Build camera_ros in a colcon workspace
WORKDIR /opt/camera_ws/src
RUN git clone https://github.com/christianrauch/camera_ros.git && \
    find camera_ros -type f \( -name "*.cpp" -o -name "*.hpp" \) -exec sed -i 's/libcamera::dynamic_extent/std::dynamic_extent/g' {} +

WORKDIR /opt/camera_ws
RUN . /opt/ros/lyrical/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro lyrical --skip-keys=libcamera && \
    colcon build --event-handlers=console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-Wno-deprecated-declarations -Wno-error=deprecated-declarations" && \
    rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/camera_ws/install/setup.bash" >> /root/.bashrc

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc

# Create development script
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -eo pipefail' \
    'source /opt/ros/lyrical/setup.bash' \
    'source /opt/camera_ws/install/setup.bash' \
    'cd /ros2_ws' \
    'if [ "$DEV" = "true" ]; then' \
    '  colcon build --packages-select camera_ros --cmake-clean-cache' \
    'fi' \
    'source install/setup.bash' \
    'exec ros2 launch camera_ros camera.launch.py' \
    > /start_dev.sh && chmod +x /start_dev.sh

CMD ["/start_dev.sh"]

