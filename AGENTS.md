# AGENTS.md

> [!IMPORTANT]
> **Investigation Guideline**: When starting any investigation in this repository, you MUST read `AGENTS.md` first to understand the workspace structure, workflow, and commands.

## Context
ReloBot is a ROS2-based robotics platform running on Raspberry Pi 5.
**Crucial**: The host system is ONLY for orchestration. ALL ROS2 nodes and build processes run inside Docker containers. The host cannot build or run ROS2 nodes directly.

## File Structure & Documentation Index
- `AGENTS.md`: Central workspace entry point, rules, and commands reference.
- `README.md`: High-level repository overview.
- `FLASHING.md`: Detailed guide for building, flashing, and monitoring Pico-ROS firmwares.
- `.agents/skills/rust-skills/`: Comprehensive Rust guidelines & best practices ([SKILL.md](file:///home/glebos/projects/relobot/.agents/skills/rust-skills/SKILL.md)) for writing, reviewing, and refactoring Rust components (e.g. `rosbridge_rust`).
- [pico_firmware/README.md](pico_firmware/README.md): Shared foundation for Pico-ROS firmware: SDK/FreeRTOS, USB/Zenoh, startup and board profiles.
- `pico_ware_*_picoros/`: Native Pico-ROS (Zenoh) firmwares:
    - [pico_ware_wheels_picoros/README.md](pico_ware_wheels_picoros/README.md): Differential drive wheel control native Pico-ROS node.
    - [pico_ware_knives_picoros/README.md](pico_ware_knives_picoros/README.md): Knives motor driver native Pico-ROS node.
    - [pico_ware_ina226_picoros/README.md](pico_ware_ina226_picoros/README.md): INA226 power & dock monitoring native Pico-ROS node.
    - [pico_ware_imu_picoros/README.md](pico_ware_imu_picoros/README.md): IMU sensor native Pico-ROS node.
- `helpers/`: Utility tools and calibration procedures:
    - [helpers/wheel_calibration/README.md](helpers/wheel_calibration/README.md): Wheel velocity calibration sweep & linear regression constants.
    - [helpers/camera_calibration/README.md](helpers/camera_calibration/README.md): Fisheye camera calibration guide.
    - [helpers/cpu_profiler/README.md](helpers/cpu_profiler/README.md): ROS 2 & Docker container CPU/Memory statistical profiler.
- `ros2_ws/`: The main ROS2 workspace (`src/`, `docker-compose.yml`, Dockerfiles).
    - [ros2_ws/readme.md](ros2_ws/readme.md): ROS2 nodes architecture.
    - [ros2_ws/src/web_server/README.md](ros2_ws/src/web_server/README.md): Web frontend server details.
- `wiki/`: Architecture decisions and benchmark reports:
    - [wiki/zenoh_picoros_architecture_and_decisions.md](wiki/zenoh_picoros_architecture_and_decisions.md): Architecture Decision Records (ADR 1-4), validation evidence, and performance benchmarks for ROS 2 Lyrical + Zenoh SHM + Native Pico-ROS.
    - `composition_performance_report.md`, `cpu_profiling_and_optimization_report.md`.
- Core Helper Scripts:
    - `start_robot.sh`: Core host script to start/stop the Docker containers (supports `--sim` for Gazebo simulation and `--dev` mode).
    - `start_sim.sh`: Convenience launcher for Gazebo simulation (`./start_sim.sh up [--gui|--headless] [--world garden|obstacles|empty] [--dev]`).
    - `profile_cpu.sh`: Measures CPU and RAM consumption across all nodes and containers over time.
    - `run_firmware.sh`: Firmware management CLI (build, scan, flash, info, monitor).
    - `start_wheel_calibration.sh`: One-command wheel calibration process launcher.
    - `start_camera_calibration.sh`: Camera calibration launcher.
    - `rviz2.sh`: Runs RViz2 container (supports `--sim` for simulation view).
    - `apply_optimizations.sh`: Configures host Raspberry Pi 5 kernel, power overrides, ZRAM, and services for maximum performance.

## Setup
1. **Repository**: `git clone ...` (already done if you are reading this).
2. **Devices**: Run `python3 finddevice.py` to auto-detect and map serial devices (TTYs) when running on physical hardware.
3. **Environment**: Ensure `docker` and `docker compose` are installed. For WSL2, WSLg graphics acceleration is automatically detected.
4. **Zenoh Middleware**: ReloBot runs on ROS 2 Lyrical with `rmw_zenoh_cpp` and Zenoh shared memory (`transport/shared_memory/enabled=true`). All microcontrollers communicate via native Pico-ROS USB serial connected to `ros2_zenoh_router`.

## Workflow
### Running the Robot (Physical Hardware)
To start the entire physical stack:
```bash
# Recommended launcher (adds --dev to rebuild nodes on startup):
./start_robot.sh up [--dev]
```

### Running the Gazebo Simulation (Local Testing in WSL2/Linux)
To test locally without hardware:
```bash
# Start Gazebo simulation with garden world (default GUI mode):
./start_robot.sh up --sim [--dev]
# Or using the convenience script:
./start_sim.sh up [--world garden|obstacles|empty] [--gui|--headless] [--dev]

# Launch RViz2 connected to simulation:
./rviz2.sh --sim

# Open web interface in browser:
# http://localhost/ (or http://127.0.0.1/)
```

### Developing a New Node (Agent)
1. **Create Package**: Create your new ROS2 package in `ros2_ws/src/`.
2. **Docker Definition**:
    - If it requires special dependencies, create a new `NewAgent.dockerfile`.
    - Otherwise, you may reuse `ros:humble` base or existing images.
3. **Orchestration**: Add a new service to `ros2_ws/docker-compose.yml`.
    - Use Docker Compose `profiles: ["hardware"]` for physical devices or `profiles: ["sim"]` for simulation-only services.
    - **Network**: Must use `network_mode: host` and `ipc: host`.
    - **Volumes**: Mount `.:/ros2_ws/` to allow code editing from host.
4. **Build**:
    - The existing containers are configured to run `colcon build` on startup (via entrypoint scripts like `start_dev.sh` generated in Dockerfiles).
    - To apply changes: Restart the specific container (`docker compose restart <service_name>`).

### Frontend Development
The web server frontend (`ros2_frontend` service) serves static files located in `ros2_ws/src/web_server/frontend` via Nginx.
- **Serving & Access**: The container uses `network_mode: host` and Nginx is configured to listen on **Port 80**. The web interface can be accessed via a browser at `http://raspberrypi.local/` (or `http://localhost` / `http://<pi-ip-address>`). **Note**: When using a browser subagent, you MUST navigate to `http://raspberrypi.local/` instead of localhost for it to be resolved correctly.
- **Volume Mounting**: The frontend directory is volume-mounted directly into Nginx's HTML root inside the container as defined in `ros2_ws/docker-compose.yml`.
- **Immediate Effect (No Docker Restart Required)**: Because the static content is served directly by Nginx, editing HTML, CSS, or JavaScript files on the host takes effect **immediately** inside the running container. There is **no need to restart Docker, compose, or any container services** to apply frontend changes. Simply refresh the browser page to see updates.

### Visualization / Debugging
- **RViz2**: Run `./rviz2.sh` (physical) or `./rviz2.sh --sim` (simulation) from the host.
- **Logs**: `docker compose logs -f <service_name>` (or `./start_sim.sh logs <service_name>`).
- **Shell Access**: `docker compose exec <service_name> bash` to enter a running container.

## Commands
| Action | Command |
| :--- | :--- |
| **Start All (Hardware)** | `./start_robot.sh up --dev` |
| **Start Simulation (Gazebo)** | `./start_sim.sh up --dev` or `./start_robot.sh up --sim --dev` |
| **Stop All (Hardware & Sim)** | `./start_robot.sh down` |
| **View Logs** | `docker compose logs -f` |
| **Run RViz2 (Simulation)** | `./rviz2.sh --sim` |
| **Scan Firmware Devices** | `./run_firmware.sh scan` |
| **Build All Firmwares** | `./run_firmware.sh build all` |
| **Flash Firmware** | `./run_firmware.sh flash <wheels|wheels_calibration|knives|imu|ina226>` |
| **Run Wheel Calibration** | `./start_wheel_calibration.sh` |
| **Profile CPU & Memory** | `./profile_cpu.sh [duration_sec]` |



## Inspecting ROS2 (via Docker)
All ROS2 tools must be run **inside** the containers via `docker compose exec`. The compose file is at `ros2_ws/docker-compose.yml`.

```bash
# General pattern (run from the repo root):
# NOTE: `bash -c` does not load .bashrc, so you MUST source ROS2 setup first.
docker compose -f ros2_ws/docker-compose.yml exec <service_name> bash -c "source /opt/ros/lyrical/setup.bash && <command>"
```

### Common checks
```bash
# List active topics
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/lyrical/setup.bash && ros2 topic list"

# Echo a topic
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/lyrical/setup.bash && ros2 topic echo /scan --once"

# List running nodes
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/lyrical/setup.bash && ros2 node list"

# Check node parameters
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c "source /opt/ros/lyrical/setup.bash && ros2 param list /bt_navigator"

# Find an installed shared library (.so) — no sourcing needed
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 find /opt/ros/lyrical -name "libsome_plugin.so"

# List files from an apt package — no sourcing needed
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 dpkg -L ros-lyrical-some-package

# Check installed BT XMLs — no sourcing needed
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 ls /opt/ros/lyrical/share/nav2_bt_navigator/behavior_trees/

# Interactive shell (sources automatically via .bashrc)
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash
```

## Style & Conventions
- **Language**: Python (preferred for logic), C++ (for performance), or Rust (e.g. `rosbridge_rust`).
- **Rust Guidelines**: Follow `.agents/skills/rust-skills/` rules and idioms when writing, optimizing, or refactoring Rust code.
- **ROS2**: Use ROS2 Lyrical (with `rmw_zenoh_cpp`).
- **Communication**: Use ROS2 topics/services. Do NOT bypass ROS2 unless necessary.
- **Hardware**: Access hardware via `/dev/` mapped devices in `docker-compose.yml`.
