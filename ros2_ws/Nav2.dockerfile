FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-humble-sensor-msgs-py ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-m-explore-ros2

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# build nav2_bringup
COPY . /ros2_ws
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    colcon build --packages-select nav2_bringup

# setup entrypoint
CMD ["/ros2_ws/install/setup.bash", "&&", "ros2", "launch", "nav2_bringup", "nav2_bringup.launch.py"]
