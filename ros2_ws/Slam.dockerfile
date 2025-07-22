FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-humble-sensor-msgs-py ros-humble-slam-toolbox ros-humble-robot-localization \
    ros-humble-navigation2 ros-humble-nav2-bringup

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
colcon build --packages-select slam_tool\n\
source /ros2_ws/install/setup.bash\n\
ros2 pkg list | grep slam_tool && ros2 launch slam_tool launch_slam.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]