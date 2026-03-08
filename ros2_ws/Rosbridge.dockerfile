FROM ros:humble-ros-base

# Install rosbridge server and web_video_server
# Also install action message packages so rosbridge can deserialise
# nav2 NavigateToPose and opennav DockRobot/UndockRobot goals/results.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server \
    ros-humble-nav2-msgs \
    ros-humble-opennav-docking-msgs \
    ros-humble-slam-toolbox && \
    rm -rf /var/lib/apt/lists/*

# Workspace is volume-mounted at /ros2_ws via docker-compose
WORKDIR /ros2_ws

# Build explore_lite_msgs from the mounted source and launch rosbridge (mirrors nav2.dockerfile pattern)
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
if [ "$DEV" = "true" ] && [ ! -d "/ros2_ws/install/explore_lite_msgs" ]; then\n\
  colcon build --packages-select explore_lite_msgs --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release\n\
fi\n\
source install/setup.bash\n\
ros2 launch /ros2_ws/src/rosbridge_webvideo_launch.py' > /start.sh && \
chmod +x /start.sh

CMD ["/start.sh"]
