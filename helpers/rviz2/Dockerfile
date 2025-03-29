FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 ros-humble-rviz-imu-plugin

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc