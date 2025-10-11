FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws

# Copy the package
COPY ./src/frontier_explorer /ros2_ws/src/frontier_explorer

# Build and run
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && colcon build --packages-select frontier_explorer && source install/setup.bash && ros2 launch frontier_explorer explore.launch.py"]