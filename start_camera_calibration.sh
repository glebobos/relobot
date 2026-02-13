#!/bin/bash

# Camera Calibration Tool Launcher
# Enables X11 forwarding and runs camera calibration in a Docker container

set -e

# Default parameters
CAMERA_NAME="camera"
SIZE="8x6"
SQUARE="0.02"
TOPIC="/camera/image_raw"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --size)
      SIZE="$2"
      shift 2
      ;;
    --square)
      SQUARE="$2"
      shift 2
      ;;
    --topic)
      TOPIC="$2"
      shift 2
      ;;
    --camera_name)
      CAMERA_NAME="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: ./start_camera_calibration.sh [--size 8x6] [--square 0.02] [--topic /camera/image_raw] [--camera_name camera]"
      exit 1
      ;;
  esac
done

echo "Starting Camera Calibration Tool..."
echo "Targeting topic: $TOPIC"
echo "Checkerboard size: $SIZE"
echo "Square size: $SQUARE m"

# Setup X11 forwarding
echo "Configuring X11 access..."
if [ -z "$DISPLAY" ]; then
    export DISPLAY=":0"
fi
xhost +local:docker

# Create directory for calibration data if it doesn't exist
mkdir -p "$PWD/calibration_data"
echo "Calibration data will be saved to: $PWD/calibration_data"

# Build the image if it doesn't exist
if [[ "$(docker images -q camera_calibration:latest 2> /dev/null)" == "" ]]; then
  echo "Building camera_calibration image..."
  docker build -t camera_calibration:latest helpers/camera_calibration
fi

# Run the container
echo "Launching calibration node..."
docker run -it --rm \
    --network=host \
    --privileged \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD/calibration_data:/tmp" \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    camera_calibration:latest \
    ros2 run camera_calibration cameracalibrator \
    --size "$SIZE" \
    --square "$SQUARE" \
    --ros-args -r image:="$TOPIC" -p camera:="$CAMERA_NAME"

echo "Calibration tool exited."
