# relobot
Hello world

How to run rviz2 form docker in WSL?
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg -e DISPLAY  -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER  osrf/ros:jazzy-desktop rviz2