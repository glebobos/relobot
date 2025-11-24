FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-humble-sensor-msgs-py ros-humble-pointcloud-to-laserscan ros-humble-cv-bridge python3-opencv python3-numpy &&  pip3 install ArducamDepthCamera
RUN apt install ros-humble-laser-filters -y
# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
colcon build --packages-select arducam_rclpy_tof_pointcloud\n\
source install/setup.bash\n\
python3 /ros2_ws/wait_for_topic.py && \\\n\
ros2 launch arducam_rclpy_tof_pointcloud launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]