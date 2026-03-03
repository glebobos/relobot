FROM ros:humble-ros-base

# Install rosbridge server and web_video_server
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server && \
    rm -rf /var/lib/apt/lists/*

# Launch both rosbridge and web_video_server (launch file mounted via docker-compose)
CMD ["ros2", "launch", "/ros2_ws/src/rosbridge_webvideo_launch.py"]
