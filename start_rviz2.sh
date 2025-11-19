#!/bin/bash

# RViz2 Docker Launcher Script
# This script starts RViz2 in a Docker container with proper X11 forwarding and ROS2 configuration

set -e  # Exit on any error

echo "Starting RViz2 Docker container..."

# Allow local Docker containers to access X11 display
echo "Configuring X11 access for Docker..."
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
    -e DISPLAY=:0 \
    -e QT_QPA_PLATFORM=xcb \
    -e ROS_SUPER_CLIENT=True \
    -e ROS_DISCOVERY_SERVER="$HOST_IP:11811" \
    rviz2:latest \
    rviz2

echo "RViz2 container stopped."