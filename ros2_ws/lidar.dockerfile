ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-base

# Install additional dependencies
RUN apt-get update && apt-get upgrade -y  && rm -rf /var/lib/apt/lists/*
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
  colcon build --packages-select ldlidar \n\
fi\n\
source install/setup.bash\n\
ros2 launch ldlidar stl27l.launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]