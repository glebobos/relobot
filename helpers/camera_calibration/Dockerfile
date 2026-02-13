FROM ros:humble

# Install camera calibration dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-camera-calibration \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-image-pipeline \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Disable SharedMemory for QT (fixes some GUI issues in Docker)
ENV QT_X11_NO_MITSHM=1

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
