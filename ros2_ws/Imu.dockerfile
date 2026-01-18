FROM ros:humble

# Install additional dependencies
RUN apt-get update 
# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
git clone https://github.com/RoverRobotics-forks/serial-ros2.git ./src/serial || true\n\
if [ "$DEV" = "true" ]; then\n\
  colcon build --packages-select serial icm_20948\n\
fi\n\
source install/setup.bash\n\
ros2 launch icm_20948 run.launch.xml port:=${SERIAL_PORT}' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]