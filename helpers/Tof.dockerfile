ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-${ROS_DISTRO}-sensor-msgs-py ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-cv-bridge python3-opencv python3-numpy &&  pip3 install ArducamDepthCamera
RUN apt install ros-${ROS_DISTRO}-laser-filters -y
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
  colcon build --packages-select arducam_rclpy_tof_pointcloud\n\
fi\n\
source install/setup.bash\n\
ros2 launch arducam_rclpy_tof_pointcloud launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]
