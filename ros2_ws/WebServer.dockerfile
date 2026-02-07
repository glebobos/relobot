ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    cmake \
    python3-flask \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Build opennav_docking_msgs from source (not available as apt package for Jazzy)
WORKDIR /opt/opennav_msgs_ws
RUN git clone --depth 1 --branch main https://github.com/open-navigation/opennav_docking.git src/opennav_docking && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select opennav_docking_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf src build log

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/'"${ROS_DISTRO}"'/setup.bash\n\
source /opt/opennav_msgs_ws/install/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select web_server\n\
fi\n\
source install/setup.bash\n\
ros2 run web_server web_server_node' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]