# ROS 2 Node Composition Performance Report

## Overview
This report documents the performance impact of transitioning the **Nav2 (Navigation 2)** stack from a standard multi-process architecture to a **Composable Node** architecture on a Raspberry Pi-class embedded system.

Optimizing resource consumption is critical for autonomous robots where CPU and RAM are shared between navigation, localization, perception, and autonomy logic.

## Benchmarking Results
Data was averaged over 10-second intervals during steady-state operation of the Nav2 stack (including Controller, Planner, BT Navigator, etc.).

| Metric | Multi-Process (Standard) | Composition (Optimized) | **Net Improvement** |
| :--- | :--- | :--- | :--- |
| **Sum of CPU % (ps)** | ~60.1% | **~39.8%** | **33.8% Reduction** |
| **Sum of RAM (RSS)** | ~365 MB | **~140 MB** | **61.6% Reduction** |
| **Docker Container CPU** | ~45.9% | **~26.9%** | **41.4% Reduction** |
| **System PIDs/Threads** | 147 | **79** | **46.3% Reduction** |

## Key Findings

### 1. Significant RAM Recovery
The most dramatic improvement was in memory usage. By moving 8 separate Nav2 nodes into a single process container (`component_container_isolated`), we eliminated the overhead of 8 individual shared library loads and 8 separate DDS discovery participants. Reclaiming over **220MB of RAM** allows the robot to handle larger maps or additional perception nodes.

### 2. Reduced "Middleware Tax"
Every process context switch in Linux has a cost. By consolidating the high-frequency Nav2 servers into a single process, the CPU spends less time managing process switching and more time performing the actual calculus required for **MPPI Trajectory Planning** and **Global Pathfinding**.

### 3. Stability and Predictability
Monitoring showed that the CPU load with composition enabled was much "flatter" and more predictable. In multi-process mode, CPU spikes were more frequent (jumping between 43% and 50%), likely due to discovery and scheduling jitter.

## Technical Configuration
The optimized setup uses the **`component_container_isolated`** model (`component_container --executor-type single-threaded --isolated` in ROS 2 Lyrical). This provides a dedicated thread for each Nav2 server while keeping them in the same process. This "best of both worlds" approach prevents one server from blocking another while reaping all the multi-process-to-single-process benefits.

### SLAM Toolbox Composition & Intra-Process Communication (IPC)
As of late October 2026:
1. **`slam_toolbox` Composable Migration**: `slam_toolbox` has been moved from a standalone OS process (`async_slam_toolbox_node` / `localization_slam_toolbox_node`) into `nav2_container` as a native `ComposableNode` plugin (`slam_toolbox::AsynchronousSlamToolbox` or `LocalizationSlamToolbox`). This reclaims an additional ~150 MB of process RSS memory and unifies lifecycle management under `lifecycle_manager_navigation`.
2. **Selective Intra-Process Communication**:
   - `use_intra_process_comms: True` is enabled on high-frequency volatile control nodes (`controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`, `docking_server`, `coverage_server`), allowing topics like `/cmd_vel_nav` (20–30 Hz) and internal action goals to pass zero-copy pointers in memory without Zenoh SHM middleware overhead.
   - `use_intra_process_comms: False` is kept on `slam_toolbox` and `lifecycle_manager_navigation` to protect transient-local `/map` delivery to external subscribers (such as `rosbridge_rust` and RViz2) against upstream issue `ros2/rclcpp#2704` and protect lifecycle state transitions (`ros2/rclcpp#2721`).

## References
*   **Research Paper:** *"Impact of ROS 2 Node Composition in Robotic Systems"* by Steve Macenski (Samsung Research), Alberto Soragna (iRobot), et al. (2023).
*   **Implementation:** Modified `navigation_launch.py` to enable `use_composition`, load `slam_toolbox` as a composable node, configure intra-process communications, and inject parameter files globally into the container to prevent dynamically spawned costmap crashes.

## Future Recommendations
*   **Waitsets & Events-CBG Executors:** Further tuning of the rclcpp Executors (such as `events-cbg` executor) could squeeze another 2-5% efficiency out of the system if needed.
