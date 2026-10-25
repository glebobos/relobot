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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="${SCRIPT_DIR}/ros2_ws"

# Parse arguments
COMMAND=""
SERVICE=""
DEV=false
SIM=false
SIM_GUI=true
SIM_WORLD="garden_world.sdf"
WSLG=false
DETACH=false
PICOROS_KNIVES=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        -d|--detach)
            DETACH=true
            shift
            ;;
        --dev)
            DEV=true
            shift
            ;;
        --sim)
            SIM=true
            shift
            ;;
        --picoros-knives)
            PICOROS_KNIVES=true
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
        --wslg)
            WSLG=true
            shift
            ;;
        --world)
            if [[ -n "$2" && ! "$2" =~ ^-- ]]; then
                SIM_WORLD="$2"
                if [[ "$SIM_WORLD" == obstacles || "$SIM_WORLD" == obstacle ]]; then
                    SIM_WORLD="obstacle_world.sdf"
                elif [[ "$SIM_WORLD" =~ \.sdf$ ]]; then
                    SIM_WORLD="$SIM_WORLD"
                elif [[ "$SIM_WORLD" =~ _world$ ]]; then
                    SIM_WORLD="${SIM_WORLD}.sdf"
                else
                    SIM_WORLD="${SIM_WORLD}_world.sdf"
                fi
                if [[ ! -f "${SCRIPT_DIR}/ros2_ws/src/relobot_gazebo/worlds/${SIM_WORLD}" ]]; then
                    echo "Error: World '${SIM_WORLD}' not found in ros2_ws/src/relobot_gazebo/worlds/"
                    echo "Available worlds: garden, obstacles, empty (e.g. garden_world.sdf, obstacle_world.sdf, empty_world.sdf)"
                    exit 1
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
    echo "  --picoros-knives - Connect a physical Pico-ROS knives board in simulation (default on hardware)"
    echo "  --gui          - Launch Gazebo 3D GUI (default in sim mode)"
    echo "  --headless     - Run Gazebo in headless mode (no GUI window)"
    echo "  --wslg         - Enable WSLg devices and libraries (requires --sim --gui)"
    echo "  --world <name> - Specify world: garden (default), obstacles, empty"
    echo "  --dev          - Enable development mode (rebuilds ROS2 packages on startup and sets INFO log level)"
    exit 1
fi

cd "${ROS2_WS_DIR}"

export DEV="${DEV}"
if [ "$DEV" = true ]; then
    export ROS_LOG_LEVEL="${ROS_LOG_LEVEL:-INFO}"
    export RUST_LOG="${RUST_LOG:-info}"
    export ZENOH_ROUTER_RUST_LOG="${ZENOH_ROUTER_RUST_LOG:-info}"
    export LIBCAMERA_LOG_LEVELS="${LIBCAMERA_LOG_LEVELS:-*:INFO}"
else
    export ROS_LOG_LEVEL="${ROS_LOG_LEVEL:-WARN}"
    export RUST_LOG="${RUST_LOG:-warn}"
    export ZENOH_ROUTER_RUST_LOG="${ZENOH_ROUTER_RUST_LOG:-warn}"
    export LIBCAMERA_LOG_LEVELS="${LIBCAMERA_LOG_LEVELS:-*:WARN}"
fi
export RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL="${RCUTILS_DEFAULT_LOGGER_DEFAULT_LEVEL:-${ROS_LOG_LEVEL}}"
export RELOBOT_HOST_DIR="${SCRIPT_DIR}"
COMPOSE=(docker compose -f docker-compose.yml)
if [[ "$SIM" != true ]]; then
    export SIM_KNIVES_MOCK=false
    if [[ "$COMMAND" = up ]]; then
        python3 "${ROS2_WS_DIR}/zenoh_devices.py" validate
    fi
else
    export SIM_KNIVES_MOCK=true
fi
if [[ "$WSLG" = true ]]; then
    if [[ "$SIM" != true || "$SIM_GUI" != true ]]; then
        echo "Error: --wslg requires --sim --gui"
        exit 1
    fi
    if [[ ! -e /dev/dxg || ! -d /mnt/wslg || ! -d /usr/lib/wsl/lib ]]; then
        echo "Error: --wslg requires /dev/dxg, /mnt/wslg and /usr/lib/wsl/lib"
        exit 1
    fi
    COMPOSE+=(-f docker-compose.wslg.yml)
fi

if [ "$SIM" = true ]; then
    export COMPOSE_PROFILES="sim"
    export USE_SIM_TIME=true
    export NGINX_TEMPLATE="nginx.sim.conf.template"
    export SIM_WORLD="${SIM_WORLD}"
    export SIM_GUI="${SIM_GUI}"
    
    # Configure X11 / WSLg display permissions for Docker
    if [[ "$SIM_GUI" = true && "$COMMAND" = up ]]; then
        if command -v xhost &>/dev/null; then
            xhost +SI:localuser:root 2>/dev/null || true
        fi
        export DISPLAY="${DISPLAY:-:0}"
    fi

    echo "=========================================================="
    echo " ReloBot: GAZEBO SIMULATION MODE"
    echo " World:     ${SIM_WORLD}"
    echo " GUI:       ${SIM_GUI}"
    echo " Dev Mode:  ${DEV}"
    echo " Log Level: ${ROS_LOG_LEVEL}"
    echo "=========================================================="
else
    export COMPOSE_PROFILES="hardware"
    export USE_SIM_TIME=false
    export NGINX_TEMPLATE="nginx.conf.template"

    echo "=========================================================="
    echo " ReloBot: PHYSICAL HARDWARE MODE"
    echo " Dev Mode:  ${DEV}"
    echo " Log Level: ${ROS_LOG_LEVEL}"
    echo "=========================================================="
fi

if [ "$COMMAND" == "up" ]; then
    UP_ARGS=(--remove-orphans)
    if [ "$DETACH" = true ]; then
        UP_ARGS+=(-d)
    fi

    if [ -z "$SERVICE" ]; then
        "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" up "${UP_ARGS[@]}"
    else
        "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" up "${UP_ARGS[@]}" "${SERVICE}"
    fi

elif [ "$COMMAND" == "down" ]; then
    echo "Stopping all services..."
    "${COMPOSE[@]}" --profile hardware --profile sim down --remove-orphans
    echo "All ReloBot services stopped."

elif [ "$COMMAND" == "build" ]; then
    if [ -z "$SERVICE" ]; then
        "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" build
    else
        "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" build "${SERVICE}"
    fi

elif [ "$COMMAND" == "logs" ]; then
    if [ -z "$SERVICE" ]; then
        "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" logs -f
    else
        "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" logs -f "${SERVICE}"
    fi

elif [ "$COMMAND" == "ps" ]; then
    "${COMPOSE[@]}" --profile "${COMPOSE_PROFILES}" ps

else
    echo "Error: Invalid command '$COMMAND'"
    echo "Usage: $0 {up|down|build|logs|ps} [service] [--sim] [--dev] [--gui|--headless] [--world <name>]"
    exit 1
fi
