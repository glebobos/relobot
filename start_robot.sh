#!/usr/bin/env bash
#
# Copyright 2025 ReloBot Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Check if command is provided
if [ -z "$1" ]; then
    echo "Usage: $0 {up|down} [service]"
    echo "  up [service]   - Start all services or a specific service"
    echo "  down           - Stop all services"
    exit 1
fi

COMMAND=$1
SERVICE=$2

cd /home/admin/Documents/relobot/ros2_ws

if [ "$COMMAND" == "up" ]; then
    # Wait for internet connection before starting
    until ping -c 1 8.8.8.8 &> /dev/null
    do
        echo "No internet connection. Retrying in 5 seconds..."
        sleep 5
    done
    
    export HOST_IP=$(hostname -I | awk '{print $1}')
    echo "Starting services with HOST_IP=$HOST_IP"
    
    if [ -z "$SERVICE" ]; then
        docker compose up
    else
        docker compose up $SERVICE
    fi
    
elif [ "$COMMAND" == "down" ]; then
    echo "Stopping all services..."
    docker compose down
    
else
    echo "Error: Invalid command '$COMMAND'"
    echo "Usage: $0 {up|down} [service]"
    exit 1
fi
