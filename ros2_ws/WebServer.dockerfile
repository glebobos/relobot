FROM ros:humble

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip ros-humble-cv-bridge python3-opencv &&  pip3 install flask numpy
# Create workspace
WORKDIR /ros2_ws

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Create development script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
cd /ros2_ws\n\
colcon build --packages-select web_server\n\
source install/setup.bash\n\
python3 /ros2_ws/wait_for_topic.py && \\\n\
ros2 run web_server web_server_node' > /start_dev.sh && \
chmod +x /start_dev.sh

CMD ["/start_dev.sh"]