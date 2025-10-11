FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-humble-sensor-msgs-py ros-humble-nav2-bringup ros-humble-nav2-map-server ros-humble-ros2-control ros-humble-hardware-interface

# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# build nav2_bringup and explore_lite
COPY ./src /ros2_ws/src
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws/src && \
    git clone https://github.com/robo-friends/m-explore-ros2.git && \
    cd /ros2_ws && \
    colcon build --packages-select nav2_bringup explore_lite

# setup entrypoint
CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 launch nav2_bringup nav2_bringup.launch.py"]
