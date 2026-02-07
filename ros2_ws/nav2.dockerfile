FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    git \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2-ros \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-util \
    ros-humble-opennav-docking \
    ros-humble-opennav-docking-core \
    ros-humble-opennav-docking-bt \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy the package
COPY ./src/nav2 /ros2_ws/src/nav2

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-up-to nav2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release\n\
fi\n\
source install/setup.bash\n\
ros2 launch nav2 navigation_launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]