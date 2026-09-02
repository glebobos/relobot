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
set -e

# Parse arguments
COMMAND=""
SERVICE=""
DEV=false

for arg in "$@"; do
    if [ "$arg" == "--dev" ]; then
        DEV=true
    elif [ -z "$COMMAND" ]; then
        COMMAND=$arg
    elif [ -z "$SERVICE" ]; then
        SERVICE=$arg
    fi
done

# Check if command is provided
if [ -z "$COMMAND" ]; then
    echo "Usage: $0 {up|down} [service] [--dev]"
    echo "  up [service]   - Start all services or a specific service"
    echo "  down           - Stop all services"
    echo "  --dev          - Enable development mode (rebuilds code)"
    exit 1
fi

cd /home/admin/projects/relobot/ros2_ws

export DEV=$DEV
echo "Starting services with DEV=$DEV"

# Function to clean up stale FastDDS shared memory and semaphore segments
cleanup_fastdds_shm() {
    if [ -d /dev/shm ]; then
        if sudo -n true 2>/dev/null; then
            sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
        else
            rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
        fi
    fi
}

if [ "$COMMAND" == "up" ]; then
    # Pre-flight check: FastDDS host socket buffer limits
    RMEM_MAX=$(sysctl -n net.core.rmem_max 2>/dev/null || echo 0)
    if [ "$RMEM_MAX" -lt 67108864 ]; then
        echo -e "\033[33m[WARN] FastDDS socket buffer limit (net.core.rmem_max = $RMEM_MAX) is below recommended 67108864 (64 MB).\033[0m"
        echo -e "\033[33mTo prevent packet drops during heavy SLAM/TF loads, apply:\033[0m"
        echo -e "\033[33m  echo -e 'net.core.rmem_max = 67108864\\nnet.core.rmem_default = 33554432\\nnet.core.wmem_max = 67108864\\nnet.core.wmem_default = 33554432' | sudo tee /etc/sysctl.d/99-ros2-fastdds.conf && sudo sysctl --system\033[0m"
    fi

    # Clean up any leftover SHM segments from crashed runs to prevent memory exhaustion
    cleanup_fastdds_shm

    if [ -z "$SERVICE" ]; then
        docker compose up --remove-orphans
    else
        docker compose up --remove-orphans $SERVICE
    fi
    
elif [ "$COMMAND" == "down" ]; then
    echo "Stopping all services..."
    docker compose down --remove-orphans
    # Clean up FastDDS shared memory segments after stopping containers
    cleanup_fastdds_shm
    echo "Cleaned up FastDDS shared memory segments."
    
else
    echo "Error: Invalid command '$COMMAND'"
    echo "Usage: $0 {up|down} [service] [--dev]"
    exit 1
fi
