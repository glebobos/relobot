FROM ros:humble-ros-base

# Install rosbridge server and web_video_server
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-server \
    ros-humble-web-video-server && \
    rm -rf /var/lib/apt/lists/*

# Set up the ROS environment
ENV ROS_DOMAIN_ID=0
ENV FASTDDS_DEFAULT_PROFILES_FILE=/usr/share/fastdds/profiles/default_profiles.xml

# Copy launch file
COPY src/rosbridge_webvideo_launch.py /tmp/rosbridge_webvideo_launch.py

# Launch both rosbridge and web_video_server
CMD ["ros2", "launch", "/tmp/rosbridge_webvideo_launch.py"]
