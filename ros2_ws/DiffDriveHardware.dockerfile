FROM microros/micro-ros-agent:humble AS micro_ros_stage

FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-ros2launch \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-realtime-tools \
    ros-humble-topic-based-ros2-control \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# Copy micro-ros-agent from official image
COPY --from=micro_ros_stage /uros_ws /uros_ws

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /uros_ws/install/local_setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select diff_drive_hardware\n\
fi\n\
source install/setup.bash\n\
ros2 launch diff_drive_hardware diffbot.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

EXPOSE 5000

CMD ["/start_dev.sh"]