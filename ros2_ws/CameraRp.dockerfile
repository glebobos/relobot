FROM ros:humble

# Install build dependencies
RUN apt-get update && apt-get install -y \
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
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-rclcpp-components \
    ros-humble-sensor-msgs \
    ros-humble-image-view

# Install newer meson via pip
RUN pip3 install --upgrade meson

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
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
# Skip libcamera key because we installed it manually\n\
# rosdep install -y --from-paths src/camera_ros --ignore-src --rosdistro humble --skip-keys=libcamera\n\
colcon build --packages-select camera_ros\n\
source install/setup.bash\n\
python3 /ros2_ws/wait_for_topic.py && \\\n\
ros2 launch camera_ros camera.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]