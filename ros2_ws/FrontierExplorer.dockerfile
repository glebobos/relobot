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
    ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy the package
COPY ./src/frontier_explorer /ros2_ws/src/frontier_explorer

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
echo "Cloning m-explore-ros2 repository..."\n\
if [ ! -d "src/m-explore-ros2" ]; then\n\
    git clone https://github.com/robo-friends/m-explore-ros2.git src/m-explore-ros2\n\
else\n\
    echo "m-explore-ros2 already exists, updating..."\n\
    cd src/m-explore-ros2 && git pull && cd ../..\n\
fi\n\
echo "Checking available packages in m-explore-ros2..."\n\
find src/m-explore-ros2 -name "package.xml" -exec dirname {} \\;\n\
echo "Building all m-explore packages and frontier_explorer..."\n\
colcon build --packages-up-to frontier_explorer --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release\n\
if [ $? -eq 0 ]; then\n\
    echo "Build completed successfully. Starting frontier explorer..."\n\
    source install/setup.bash\n\
    echo "Available packages:"\n\
    ros2 pkg list | grep -E "(explore|frontier)"\n\
    ros2 launch frontier_explorer explore.launch.py\n\
else\n\
    echo "Build failed! Check the build output above."\n\
    exit 1\n\
fi' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]