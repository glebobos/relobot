ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core

# Install rosbridge server and web_video_server
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-web-video-server && \
    rm -rf /var/lib/apt/lists/*

# Set up the ROS environment
ENV ROS_DOMAIN_ID=0
ENV FASTDDS_DEFAULT_PROFILES_FILE=/usr/share/fastdds/profiles/default_profiles.xml

# Launch both rosbridge and web_video_server
CMD ["ros2", "launch", "/ros2_ws/src/rosbridge_webvideo_launch.py"]
