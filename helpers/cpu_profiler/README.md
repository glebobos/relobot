# ReloBot CPU & Memory Profiler

A lightweight, zero-dependency statistical resource profiling tool designed for the ROS 2 robotics platform running inside Docker containers on Raspberry Pi 5.

---

## Capabilities

1. **Multi-Container Monitoring**: Automatically maps all Docker containers (`ros2_nav2`, `ros2_diff_robot`, `ros2_camera_rp`, `ros2_lidar`, `ros2_rosbridge`, `ros2_frontend`) to host process IDs (PIDs).
2. **Per-Node Resource Metrics**: Calculates instantaneous and aggregate statistics for every ROS 2 node and process:
   - **Mean CPU %**
   - **Standard Deviation ($\sigma$)**
   - **Min & Max CPU %**
   - **95th Percentile (P95)**
   - **Resident Memory (RSS in MB)**
3. **Thread-Level Profiling for Composed Nodes**: Samples individual lightweight processes (LWPs/TIDs) inside composed containers (`component_container_isolated` and `component_container`), identifying which specific ROS 2 components and middleware threads consume CPU.
4. **JSON Metrics Export**: Automatically dumps structured time-series and aggregate metrics to a JSON file for benchmark analysis.

---

## How to Use

### One-Command Quick Run (Recommended)

From the root of the repository:
```bash
# Run a standard 30-second profiling sweep:
./profile_cpu.sh

# Run for a custom duration (e.g. 60 seconds):
./profile_cpu.sh 60

# Run with custom sampling interval and custom output path:
./profile_cpu.sh 30 -i 0.5 -o /path/to/report.json
```

---

### Direct Python Execution

```bash
# Basic run:
python3 helpers/cpu_profiler/cpu_profiler.py 30

# With options:
python3 helpers/cpu_profiler/cpu_profiler.py --duration 45 --interval 1.0 --output /tmp/cpu_report.json
```

---

## Command Line Arguments

| Argument | Description | Default |
| :--- | :--- | :--- |
| `duration` | Total measurement period in seconds | `30` |
| `-i, --interval` | Sampling resolution in seconds | `1.0` |
| `-o, --output` | Destination path for the exported JSON metrics report | `/tmp/cpu_profile_report.json` |

---

## Sample Output

```text
=========================================================================================================
 SAMPLING COMPLETE - SUMMARY & DESCRIPTIVE STATISTICS
=========================================================================================================

1. AGGREGATE RESOURCE USAGE BY DOCKER CONTAINER
---------------------------------------------------------------------------------------------------------
CONTAINER NAME               MEAN CPU %     TOTAL RSS (MB)     PROCESS COUNT
---------------------------------------------------------------------------------------------------------
ros2_ws-ros2_nav2-1              48.69%           521.3 MB        6 processes
ros2_ws-ros2_diff_robot-1        12.78%           222.7 MB        6 processes
ros2_ws-ros2_lidar-1              7.66%            65.5 MB        3 processes
ros2_ws-ros2_camera_rp-1          7.03%           314.7 MB        5 processes
ros2_ws-ros2_rosbridge-1          2.65%           155.5 MB        4 processes
ros2_frontend                     0.00%            32.3 MB        5 processes


2. PER-NODE / PROCESS RESOURCE CONSUMPTION (Sorted by Mean CPU)
---------------------------------------------------------------------------------------------------------
NODE / PROCESS NAME                CONTAINER                MEAN %   STD    MIN %   MAX %   P95 %   RSS (MB)
---------------------------------------------------------------------------------------------------------
[Composed] nav2_container          ros2_ws-ros2_nav2-1       41.93%  7.57  35.29%  62.34%  58.85%   219.5MB
STL27L                             ros2_ws-ros2_lidar-1       7.66%  1.47   5.25%  11.43%  10.97%    30.6MB
[Composed] camera_container        ros2_ws-ros2_camera_rp-1   6.68%  1.50   5.06%  10.97%   9.35%   148.4MB
imu_micro_ros_agent                ros2_ws-ros2_diff_robot-1   5.40%  1.12   4.05%   8.31%   7.98%    72.6MB
ekf_filter_node                    ros2_ws-ros2_diff_robot-1   4.49%  0.98   3.15%   6.90%   6.40%    35.7MB
slam_toolbox                       ros2_ws-ros2_nav2-1        2.92%  0.64   2.08%   4.16%   4.12%    53.6MB
rosbridge_rust                     ros2_ws-ros2_rosbridge-1   2.54%  0.76   1.04%   4.16%   3.99%    40.8MB
coverage_manager                   ros2_ws-ros2_nav2-1        2.09%  0.64   1.01%   3.17%   3.12%   150.7MB
ros2_control_node                  ros2_ws-ros2_diff_robot-1   1.92%  0.65   1.04%   3.20%   3.16%    43.8MB
explore_node                       ros2_ws-ros2_nav2-1        1.60%  0.58   0.99%   3.12%   2.14%    35.9MB
robot_state_publisher              ros2_ws-ros2_diff_robot-1   0.98%  0.66   0.00%   2.16%   2.14%    34.1MB
system_monitor                     ros2_ws-ros2_camera_rp-1   0.14%  0.35   0.00%   1.07%   1.05%    47.5MB
apriltag_manager                   ros2_ws-ros2_camera_rp-1   0.11%  0.32   0.00%   1.07%   1.05%    60.3MB
rosbridge_websocket                ros2_ws-ros2_rosbridge-1   0.11%  0.32   0.00%   1.07%   1.06%    78.2MB
```
