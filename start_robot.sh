#!/usr/bin/env bash
#
# Copyright 2025-2026 ReloBot Contributors
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
SIM=false
SIM_GUI=true
SIM_WORLD="garden_world.sdf"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dev)
            DEV=true
            shift
            ;;
        --sim)
            SIM=true
            shift
            ;;
        --headless)
            SIM_GUI=false
            shift
            ;;
        --gui)
            SIM_GUI=true
            shift
            ;;
        --world)
            if [[ -n "$2" && ! "$2" =~ ^-- ]]; then
                SIM_WORLD="$2"
                # If user passed short name like 'garden', append '_world.sdf'
                if [[ ! "$SIM_WORLD" =~ \.sdf$ ]]; then
                    if [[ "$SIM_WORLD" =~ _world$ ]]; then
                        SIM_WORLD="${SIM_WORLD}.sdf"
                    else
                        SIM_WORLD="${SIM_WORLD}_world.sdf"
                    fi
                fi
                shift 2
            else
                echo "Error: --world requires a world name (e.g. garden, obstacles, empty)"
                exit 1
            fi
            ;;
        up|down|restart|build|logs|ps)
            COMMAND="$1"
            shift
            ;;
        *)
            if [ -z "$COMMAND" ]; then
                COMMAND="$1"
            elif [ -z "$SERVICE" ]; then
                SERVICE="$1"
            fi
            shift
            ;;
    esac
done

# Check if command is provided
if [ -z "$COMMAND" ]; then
    echo "Usage: $0 {up|down|build|logs|ps} [service] [--sim] [--dev] [--gui|--headless] [--world <name>]"
    echo ""
    echo "Commands:"
    echo "  up [service]   - Start physical robot stack (or simulation with --sim)"
    echo "  down           - Stop all robot/simulation services"
    echo "  build [service]- Rebuild container images"
    echo "  logs [service] - View service logs"
    echo ""
    echo "Options:"
    echo "  --sim          - Run in Gazebo simulation mode (swaps hardware drivers for Gazebo)"
    echo "  --gui          - Launch Gazebo 3D GUI (default in sim mode)"
    echo "  --headless     - Run Gazebo in headless mode (no GUI window)"
    echo "  --world <name> - Specify world: garden (default), obstacles, empty"
    echo "  --dev          - Enable development mode (rebuilds ROS2 packages on startup)"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="${SCRIPT_DIR}/ros2_ws"
cd "${ROS2_WS_DIR}"

export DEV="${DEV}"

if [ "$SIM" = true ]; then
    export COMPOSE_PROFILES="sim"
    export USE_SIM_TIME=true
    export NGINX_TEMPLATE="nginx.sim.conf.template"
    export SIM_WORLD="${SIM_WORLD}"
    export SIM_GUI="${SIM_GUI}"
    
    # Configure X11 / WSLg display permissions for Docker
    if [ "$SIM_GUI" = true ]; then
        if command -v xhost &>/dev/null; then
            xhost +local:docker 2>/dev/null || true
            xhost + 2>/dev/null || true
        fi
        export DISPLAY="${DISPLAY:-:0}"
        export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
    fi

    echo "=========================================================="
    echo " ReloBot: GAZEBO SIMULATION MODE"
    echo " World:    ${SIM_WORLD}"
    echo " GUI:      ${SIM_GUI}"
    echo " Dev Mode: ${DEV}"
    echo "=========================================================="
else
    export COMPOSE_PROFILES="hardware"
    export USE_SIM_TIME=false
    export NGINX_TEMPLATE="nginx.conf.template"

    echo "=========================================================="
    echo " ReloBot: PHYSICAL HARDWARE MODE"
    echo " Dev Mode: ${DEV}"
    echo "=========================================================="
fi

if [ "$COMMAND" == "up" ]; then
    if [ -z "$SERVICE" ]; then
        docker compose --profile "${COMPOSE_PROFILES}" up --remove-orphans
    else
        docker compose --profile "${COMPOSE_PROFILES}" up --remove-orphans "${SERVICE}"
    fi

elif [ "$COMMAND" == "down" ]; then
    echo "Stopping all services..."
    docker compose --profile hardware --profile sim down --remove-orphans
    echo "All ReloBot services stopped."

elif [ "$COMMAND" == "build" ]; then
    if [ -z "$SERVICE" ]; then
        docker compose --profile "${COMPOSE_PROFILES}" build
    else
        docker compose --profile "${COMPOSE_PROFILES}" build "${SERVICE}"
    fi

elif [ "$COMMAND" == "logs" ]; then
    if [ -z "$SERVICE" ]; then
        docker compose --profile "${COMPOSE_PROFILES}" logs -f
    else
        docker compose --profile "${COMPOSE_PROFILES}" logs -f "${SERVICE}"
    fi

elif [ "$COMMAND" == "ps" ]; then
    docker compose --profile "${COMPOSE_PROFILES}" ps

else
    echo "Error: Invalid command '$COMMAND'"
    echo "Usage: $0 {up|down|build|logs|ps} [service] [--sim] [--dev] [--gui|--headless] [--world <name>]"
    exit 1
fi

