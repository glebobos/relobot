FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    git \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2-ros \
    ros-humble-rclcpp \
    ros-humble-rclcpp-action \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-util \
    ros-humble-opennav-docking \
    ros-humble-opennav-docking-core \
    ros-humble-opennav-docking-bt \
    && rm -rf /var/lib/apt/lists/*

# Install coverage build prerequisites in a separate layer so Docker can reuse
# the heavier ROS package layer when only coverage integration changes.
RUN apt-get update && apt-get install -y \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libeigen3-dev \
    libgdal-dev \
    libgeos-dev \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch v1.2.1 --depth 1 https://github.com/Fields2Cover/Fields2Cover.git /opt/fields2cover_src && \
    git clone --branch 0.0.1 --depth 1 https://github.com/open-navigation/opennav_coverage.git /opt/opennav_coverage_src

# Create workspace
WORKDIR /ros2_ws

# Copy the package
COPY ./src/nav2 /ros2_ws/src/nav2

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ] || [ ! -f /ros2_ws/install/opennav_coverage_msgs/share/opennav_coverage_msgs/package.xml ]; then\n\
  colcon build --base-paths /opt/fields2cover_src /opt/opennav_coverage_src /ros2_ws/src --packages-up-to nav2 explore_lite opennav_coverage opennav_coverage_msgs fields2cover --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_DOC=OFF -DBUILD_TUTORIALS=OFF\n\
fi\n\
source install/setup.bash\n\
ros2 launch nav2 navigation_launch.py' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]