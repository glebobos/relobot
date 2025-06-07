#!/usr/bin/env bash
cd /home/admin/Documents/relobot/ros2_ws
until ping -c 1 8.8.8.8 &> /dev/null
do
    echo "No internet connection. Retrying in 5 seconds..."
    sleep 5
done
export HOST_IP=$(hostname -I | awk '{print $1}')
docker compose up
