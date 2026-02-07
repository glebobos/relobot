ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-${ROS_DISTRO}-sensor-msgs-py ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-robot-localization

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
  colcon build --packages-select slam_tool\n\
fi\n\
source /ros2_ws/install/setup.bash\n\
ros2 pkg list | grep slam_tool && ros2 launch slam_tool launch_slam.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]