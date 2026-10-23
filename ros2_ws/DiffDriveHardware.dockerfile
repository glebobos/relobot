FROM ros:lyrical-ros-base-resolute@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Install build dependencies and ROS2 control / localization packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    ros-lyrical-rmw-zenoh-cpp \
    ros-lyrical-ros2launch \
    ros-lyrical-ros2-control \
    ros-lyrical-ros2-controllers \
    ros-lyrical-controller-manager \
    ros-lyrical-realtime-tools \
    ros-lyrical-xacro \
    ros-lyrical-joint-state-publisher \
    ros-lyrical-robot-state-publisher \
    ros-lyrical-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Build topic_based_ros2_control for Lyrical
WORKDIR /opt/ros2_control_ws/src
RUN git clone https://github.com/PickNikRobotics/topic_based_ros2_control.git

WORKDIR /opt/ros2_control_ws
RUN . /opt/ros/lyrical/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    echo "source /opt/ros2_control_ws/install/setup.bash" >> /root/.bashrc

# Workspace setup
WORKDIR /ros2_ws
RUN echo "source /opt/ros/lyrical/setup.bash" >> /root/.bashrc

# Development startup script
RUN printf '%s\n' \
    '#!/bin/bash' \
    'set -eo pipefail' \
    'source /opt/ros/lyrical/setup.bash' \
    'source /opt/ros2_control_ws/install/setup.bash' \
    'cd /ros2_ws' \
    'if [ "$DEV" = "true" ] || [ ! -f /ros2_ws/install/diff_drive_hardware/share/diff_drive_hardware/package.xml ]; then' \
    '  colcon build --packages-select diff_drive_hardware --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF' \
    'fi' \
    'source install/setup.bash' \
    'exec ros2 launch diff_drive_hardware diffbot.launch.py' \
    > /start_dev.sh && chmod +x /start_dev.sh

EXPOSE 5000

CMD ["/start_dev.sh"]
