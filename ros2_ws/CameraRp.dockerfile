FROM ros:humble

# Install build dependencies for libcamera + camera_ros + apriltag_ros
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
    ros-humble-image-view \
    ros-humble-web-video-server

# Install newer meson via pip (required for libcamera build)
RUN pip3 install --upgrade meson

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

# Build camera_ros and apriltag_ros in a colcon workspace
WORKDIR /opt/camera_ws/src
RUN git clone https://github.com/christianrauch/camera_ros.git && \
    git clone https://github.com/christianrauch/apriltag_ros.git

WORKDIR /opt/camera_ws
RUN . /opt/ros/humble/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys=libcamera && \
    colcon build --event-handlers=console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source /opt/camera_ws/install/setup.bash" >> /root/.bashrc

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /opt/camera_ws/install/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select camera_ros --cmake-clean-cache\n\
fi\n\
source install/setup.bash\n\
ros2 launch camera_ros camera_with_apriltag.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]
