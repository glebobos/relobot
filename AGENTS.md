# AGENTS.md

> [!IMPORTANT]
> **Investigation Guideline**: When starting any investigation in this repository, you MUST read `AGENTS.md` first to understand the workspace structure, workflow, and commands.

## Context
ReloBot is a ROS2-based robotics platform running on Raspberry Pi 5.
**Crucial**: The host system is ONLY for orchestration. ALL ROS2 nodes and build processes run inside Docker containers. The host cannot build or run ROS2 nodes directly.

## File Structure
- `ros2_ws/`: The main ROS2 workspace.
    - `src/`: Source code for ROS2 packages.
    - `docker-compose.yml`: Orchestration for all robot services.
    - `*.dockerfile`: Docker definitions for various nodes.
- `helpers/`: Utility scripts (joystick testing, etc.).
- `pico_ware_*/`: Microcontroller C/C++ firmware using micro-ROS (e.g. wheels, knives, sensors).
- `wiki/`: Reference guides and architecture details.
- `FLASHING.md`: Detailed guides for flashing the microcontroller firmware.
- `start_robot.sh`: Core host script to start/stop the Docker containers (supports development `--dev` mode).
- `rviz2.sh`: Shell script to run RViz2 container.
- `start_camera_calibration.sh`: Script to calibrate the camera.

## Setup
1. **Repository**: `git clone ...` (already done if you are reading this).
2. **Devices**: Run `python3 finddevice.py` to auto-detect and map serial devices (TTYs).
3. **Environment**: Ensure `docker` and `docker compose` are installed.

## Workflow
### Running the Robot
To start the entire stack:
```bash
# Recommended launcher (adds --dev to rebuild nodes on startup):
./start_robot.sh up [--dev]

# Alternatively, manual compose start:
cd ros2_ws
export HOST_IP=$(hostname -I | awk '{print $1}')
docker compose up
```

### Developing a New Node (Agent)
1. **Create Package**: Create your new ROS2 package in `ros2_ws/src/`.
2. **Docker Definition**:
    - If it requires special dependencies, create a new `NewAgent.dockerfile`.
    - Otherwise, you may reuse `ros:humble` base or existing images.
3. **Orchestration**: Add a new service to `ros2_ws/docker-compose.yml`.
    - **Network**: Must use `network_mode: host` and `ipc: host`.
    - **Discovery**: Must set `ROS_DISCOVERY_SERVER=${HOST_IP}:11811` and `ROS_SUPER_CLIENT=True`.
    - **Volumes**: Mount `.:/ros2_ws/` to allow code editing from host.
4. **Build**:
    - The existing containers are configured to run `colcon build` on startup (via entrypoint scripts like `start_dev.sh` generated in Dockerfiles).
    - To apply changes: Restart the specific container (`docker compose restart <service_name>`).

### Frontend Development
The web server frontend (`ros2_frontend` service) serves static files located in `ros2_ws/src/web_server/frontend` via Nginx.
- **Serving & Access**: The container uses `network_mode: host` and Nginx is configured to listen on **Port 80**. The web interface can be accessed via a browser at `http://localhost` (or `http://<pi-ip-address>`).
- **Volume Mounting**: The frontend directory is volume-mounted directly into Nginx's HTML root inside the container as defined in `ros2_ws/docker-compose.yml`.
- **Immediate Effect**: Editing HTML, CSS, or JavaScript files on the host takes effect **immediately** inside the running container. There is **no need to restart the container or the service** to apply frontend changes. Simply refresh the browser page to see updates.

### Visualization / Debugging
- **RViz2**: Run `./rviz2.sh` (or `rviz` alias if configured) from the host. It runs RViz in a Docker container connected to the Discovery Server.
- **Logs**: `docker compose logs -f <service_name>`.
- **Shell Access**: `docker compose exec <service_name> bash` to enter a running container.

## Commands
| Action | Command |
| :--- | :--- |
| **Start All** | `./start_robot.sh up` or `cd ros2_ws && docker compose up` |
| **Stop All** | `./start_robot.sh down` or `docker compose down` |
| **Rebuild Node** | `docker compose restart <service_name>` |
| **View Logs** | `docker compose logs -f` |
| **Find Devices** | `python3 finddevice.py` |

## Inspecting ROS2 (via Docker)
All ROS2 tools must be run **inside** the containers via `docker compose exec`. The compose file is at `ros2_ws/docker-compose.yml`.

```bash
# General pattern (run from the repo root):
# NOTE: `bash -c` does not load .bashrc, so you MUST source ROS2 setup first.
docker compose -f ros2_ws/docker-compose.yml exec <service_name> bash -c "source /opt/ros/humble/setup.bash && <command>"
```

### Common checks
```bash
# List active topics
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Echo a topic
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /scan --once"

# List running nodes
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/humble/setup.bash && ros2 node list"

# Check node parameters
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/humble/setup.bash && ros2 param list /bt_navigator"

# Find an installed shared library (.so) — no sourcing needed
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 find /opt/ros/humble -name "libsome_plugin.so"

# List files from an apt package — no sourcing needed
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 dpkg -L ros-humble-some-package

# Check installed BT XMLs — no sourcing needed
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 ls /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/

# Interactive shell (sources automatically via .bashrc)
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash
```

## Style & Conventions
- **Language**: Python (preferred for logic) or C++ (for performance).
- **ROS2**: Use ROS2 Humble.
- **Communication**: Use ROS2 topics/services. Do NOT bypass ROS2 unless necessary.
- **Hardware**: Access hardware via `/dev/` mapped devices in `docker-compose.yml`.
