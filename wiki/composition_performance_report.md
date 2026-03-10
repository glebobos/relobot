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
The optimized setup uses the **`component_container_isolated`** model. This provides a dedicated thread for each Nav2 server while keeping them in the same process. This "best of both worlds" approach prevents one server from blocking another while reaping all the multi-process-to-single-process benefits.

## References
*   **Research Paper:** *"Impact of ROS 2 Node Composition in Robotic Systems"* by Steve Macenski (Samsung Research), Alberto Soragna (iRobot), et al. (2023).
*   **Implementation:** Modified `navigation_launch.py` to enable `use_composition` and inject parameter files globally into the container to prevent dynamically spawned costmap crashes.

## Future Recommendations
*   **Intra-Process Communication (IPC):** Now that the nodes share a process, the robot is ready to enable zero-copy communication for high-bandwidth topics like Lidar scans and Camera images.
*   **Waitsets & Executors:** Further tuning of the rclcpp Executors could squeeze another 2-5% efficiency out of the system if needed.
