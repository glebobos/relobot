ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-serial \
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
  colcon build --packages-select mower_knife_controller\n\
fi\n\
source install/setup.bash\n\
ros2 run mower_knife_controller knife_controller_node' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]