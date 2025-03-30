docker build -t rviz2 .
export DISPLAY=:0
xhost +local:docker
export HOST_IP=$(hostname -I | awk '{print $1}')
docker run -ti     -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/shm:/dev/shm -e DISPLAY=:0 -e QT_QPA_PLATFORM=xcb -e ROS_SUPER_CLIENT=True -e ROS_DISCOVERY_SERVER=$HOST_IP:11811 --network=host   --rm rviz2:latest rviz2