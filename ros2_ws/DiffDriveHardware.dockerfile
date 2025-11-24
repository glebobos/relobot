FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros2launch \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-realtime-tools \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-io-context \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    libserial-dev \
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
colcon build --packages-select diff_drive_hardware\n\
source install/setup.bash\n\
python3 /ros2_ws/wait_for_topic.py && \\\n\
ros2 launch diff_drive_hardware diffbot.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

EXPOSE 5000

CMD ["/start_dev.sh"]