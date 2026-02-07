ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros2launch \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-serial-driver \
    ros-${ROS_DISTRO}-asio-cmake-module \
    ros-${ROS_DISTRO}-io-context \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-robot-state-publisher \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'"${ROS_DISTRO}"'/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select diff_drive_hardware\n\
fi\n\
source install/setup.bash\n\
ros2 launch diff_drive_hardware diffbot.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

EXPOSE 5000

CMD ["/start_dev.sh"]