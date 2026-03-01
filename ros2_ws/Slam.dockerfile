FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select slam_tool\n\
fi\n\
source /ros2_ws/install/setup.bash\n\
 ros2 launch slam_tool launch_slam.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]