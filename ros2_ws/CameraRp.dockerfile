ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Install build dependencies
RUN apt-get update && apt-get install -y \
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
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-image-view


WORKDIR /tmp

# Build libpisp (required for RPi 5 / BCM2712)
RUN git clone https://github.com/raspberrypi/libpisp.git && \
    cd libpisp && \
    meson setup build --prefix=/usr && \
    ninja -C build install && \
    cd .. && rm -rf libpisp

# Build libcamera (Raspberry Pi fork)
RUN git clone https://github.com/raspberrypi/libcamera.git && \
    cd libcamera && \
    meson setup build --prefix=/usr -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled && \
    ninja -C build install && \
    cd .. && rm -rf libcamera

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'"${ROS_DISTRO}"'/setup.bash\n\
cd /ros2_ws\n\
# Skip libcamera key because we installed it manually\n\
# rosdep install -y --from-paths src/camera_ros --ignore-src --rosdistro '"${ROS_DISTRO}"' --skip-keys=libcamera\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select camera_ros\n\
fi\n\
source install/setup.bash\n\
ros2 launch camera_ros camera.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]
