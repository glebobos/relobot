#!/bin/bash

# RViz2 Docker Launcher Script
# This script starts RViz2 in a Docker container with proper X11 forwarding and ROS2 configuration

set -e  # Exit on any error

echo "Starting RViz2 Docker container..."

# Allow local Docker containers to access X11 display
echo "Configuring X11 access for Docker..."
# Use existing DISPLAY if set (for SSH X11 forwarding), otherwise default to :0
if [ -z "$DISPLAY" ]; then
    export DISPLAY=":0"
fi
xhost +local:docker
# Get the host IP address for ROS discovery
HOST_IP=$(hostname -I | awk '{print $1}')
echo "Host IP detected: $HOST_IP"

# Run RViz2 in Docker container with all necessary configurations
echo "Launching RViz2..."
docker run -ti \
    --rm \
    --network=host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/shm:/dev/shm \
    -v $PWD/.rviz2:/root/.rviz2 \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/root/.Xauthority \
    -e QT_QPA_PLATFORM=xcb \
    rviz2:latest \
    rviz2

echo "RViz2 container stopped."