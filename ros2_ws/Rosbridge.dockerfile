FROM ros:humble-ros-base

# Install rosbridge server
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

# Set up the ROS environment
ENV ROS_DOMAIN_ID=0
ENV FASTDDS_DEFAULT_PROFILES_FILE=/usr/share/fastdds/profiles/default_profiles.xml

# Launch rosbridge
CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml"]
