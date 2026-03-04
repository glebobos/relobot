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

# Launch both rosbridge and web_video_server (launch file mounted via docker-compose)
CMD ["ros2", "launch", "/ros2_ws/src/rosbridge_webvideo_launch.py"]
