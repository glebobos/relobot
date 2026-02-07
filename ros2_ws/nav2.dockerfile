ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-opennav-docking \
    ros-${ROS_DISTRO}-opennav-docking-core \
    ros-${ROS_DISTRO}-opennav-docking-bt \
    && rm -rf /var/lib/apt/lists/*
    # ros-${ROS_DISTRO}-nav2-msgs \
    # ros-${ROS_DISTRO}-geometry-msgs \
    # ros-${ROS_DISTRO}-sensor-msgs \
    # ros-${ROS_DISTRO}-tf2-ros \
    # ros-${ROS_DISTRO}-rclcpp \
    # ros-${ROS_DISTRO}-rclcpp-action \
    # ros-${ROS_DISTRO}-nav2-costmap-2d \
    # ros-${ROS_DISTRO}-nav2-util \
# Create workspace
WORKDIR /ros2_ws

# Copy the package
COPY ./src/nav2 /ros2_ws/src/nav2

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'"${ROS_DISTRO}"'/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-up-to nav2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release\n\
fi\n\
source install/setup.bash\n\
ros2 launch nav2 navigation_launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]