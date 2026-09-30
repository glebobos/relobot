# ros2_ws — ROS2 workspace

This workspace runs inside Docker containers (see `docker-compose.yml`).  
All services are started from the repo root via `./start_robot.sh`.

Independent tests, offline analysis, capture instructions and the findings index
are documented in the [Navigation Tuning Lab](../helpers/navigation_diagnostics/README.md).
The lab is optional development tooling; runtime coverage sources and patches
remain required by the robot.

## Gazebo navigation baseline

Start Gazebo yourself, then start the passive recorder from another host terminal:

```bash
./start_sim.sh up --world garden --dev
python3 helpers/navigation_diagnostics/record.py --label coverage-baseline --duration 300
```

The recorder requires a running `ros2_nav2` container and automatically matches
the controller's clock: simulated time in Gazebo, system time on hardware. It
does not start services, send goals, publish velocity commands, or change parameters.
It exits after the requested **wall-clock** duration, even if Gazebo is paused.
Ctrl-C ends capture and finalizes the bag; it does **not** stop robot motion.
Use the normal UI stop/cancel control to stop the robot.

Run the same small coverage zone first, then a separate exploration trial with
`--label exploration-baseline`. Start recording before sending the goal. Note the
world, starting pose, zone, and approximate times of visible jumps or close passes.
Wait for the `Navigation recording active` line before sending a goal.
The recorder itself never changes navigation behavior. Both hardware and simulation
launches enable the shared continuous-coverage profile described below.

Results are in `ros2_ws/log/navigation/<run-id>/`: `run.json`, live
`parameters.json`, topic/publisher `graph.json`, `samples.jsonl`, `events.jsonl`,
`summary.json`, `nav2.log`, code/package snapshots, and `bag/` (rosbag2).
`--no-bag` disables the bag; `--max-mib 512` sets an approximate storage cap,
checked every two seconds. Buffered writes can exceed the cap slightly.
Nav2 console logs retain at most the last 20,000 lines / 8 MiB for this run.

Metrics compare `/cmd_vel_nav`, `/cmd_vel`, filtered odometry, and available wheel
topics; path errors use a bounded forward window of `/coverage/execution_path`
during coverage, otherwise `/plan`, rather than the closest point on any
neighboring coverage row. Each sample identifies `plan_source`. Missing/stale data and TF failures are
reported, not filled with zeros. Clock resets invalidate cached path/command data.
Reported scan clearance uses the live unpadded footprint and stamped sensor TF;
it is **observed lidar clearance**, not proof of 25 cm clearance around the whole
robot, coverage completeness, or independent Gazebo ground truth. Parameter
snapshots are captured once per node; record a new run after changing tuning.
Start recording before Execute: attaching mid-route currently initializes the
passive ordered tracker at the route start and can produce misleading tracking
errors. The controller's own ordered progress is unaffected.

## Shared continuous coverage profile

`navigation_launch.py` merges `config/coverage_sim.yaml` for both hardware and
Gazebo, in composed and standalone Nav2. The filename is retained from the
Gazebo-first validation phase; this is now the shared profile. `use_sim_time`
only selects the clock and does not enable or disable coverage. Hardware can
preview and execute routes through the same validation and stop guards.
Exploration still uses the original rotation-shim/RPP controller. Shared
costmaps now include 25 cm footprint padding and 0.95 m inflation, and the smoother
runs at 20 Hz. These shared costmap/smoother changes also affect ordinary
navigation on hardware; its original controller and goal checker are retained.
Both ordinary navigation trees explicitly select `goal_checker`; coverage selects
`coverage_goal_checker`. Omitting `goal_checker_id` from a `FollowPath` BT node
fails in Humble when both goal checkers are loaded.

Coverage previews use directional rows and a rounded perimeter pass, with
Fields2Cover Dubins connections and GEOS polygon operations. Legacy
`layout_mode: fields2cover` still preserves the server's complete `nav_path`.
Execution first plans a forward-only Smac Hybrid ingress, replaces its lattice
endpoint with a validated exact Dubins tail, then sends one
`FollowPath` goal using MPPI `CoverageFollowPath`, without the coverage BT,
intermediate waypoint stops, Spin recovery, or automatic obstacle detours.
Ingress may travel outside the selected zone but must remain in mapped free space.

The profile uses 0.20 m/s maximum speed, a 0.20 m controller turning
radius and 0.25 m planned radius. Ordinary Dubins uses the upstream spelling
`DUBIN` / `DISCONTINUOUS`: position and heading are continuous, but curvature
changes at arc boundaries. The installed continuous-curvature generator failed
the real-server test with a sideways join discontinuity.

The faster-turn profile adds a soft `VelocityDeadbandCritic` preference for
0.18 m/s (weight 35, zero angular deadband). PathAlign uses positional error
without the extra heading term, and hands off final approach at 0.20 m. The
0.15 m ordered tracking corridor and 0.05 m goal tolerance remain unchanged.
No positive minimum velocity or downstream speed clamp is used: completion,
cancellation and safety checks can still stop the robot.

Directional rows use at most 0.24 m spacing and an alternating order that keeps
consecutive rows at least two planned turning radii apart. Margins include the
asymmetric front-corner sweep and three map cells of raster allowance. With the
previous 20 cm clearance, the recorded 3.65 x 5.80 m zone gave ten rows, one perimeter pass, 0.81 m side
and 1.12 m row-end margins, versus the previous uniform 1.5 m inset. The route
was 61.10 m before ingress. The current 25 cm clearance requires larger margins;
generate a fresh preview. Preview generation is cancelable and bounded.
Disconnected rows, infeasible turns, holes that cannot be handled by a single
perimeter, and unknown/occupied swept space fail closed; split such zones.
The 25 cm body clearance applies to the selected zone as well as obstacles.
**This is not edge-to-edge mowing or a measured coverage-area guarantee.**
See [the same-zone comparison](../wiki/coverage_layout_v2.png).

Coverage uses `relobot/VerifiedMPPIController`, a version-pinned Nav2 1.1.20
patch built into the image. It reapplies motion constraints after the upstream
Savitzky-Golay filter, checks the resulting filled padded-footprint sweep
against obstacles/unknown/out-of-map space, and stores the constrained command
in filter history. It does not modify published commands downstream. The
coverage goal checker also requires ordered path progress, preventing premature
completion at a closed perimeter's start/end. Ordinary navigation retains its
original checker and controller.

Next user-operated Gazebo trial:

1. Build the new image with `./start_robot.sh build ros2_nav2 --sim --headless`.
  A plain restart is insufficient to load a new image. When ready to end the
  current session, stop with `./start_robot.sh down`, then start
  `./start_sim.sh up --world garden --dev`. Startup builds the required native
  `coverage_geometry` package if it is missing, even without dev mode.
2. Map an open area, stop exploration and any navigation goal, then select a
  small mapped zone first (2.70 x 4.45 m passed the isolated geometry test).
  Preview the route and inspect its turns. Unknown cells and insufficient
  turning space deliberately reject the preview; do not bypass that failure.
  Then use a larger mapped zone with approximately six-metre straight row
  segments, allowing additional room for the body and turns.
3. Start the recorder before Execute:
  `python3 helpers/navigation_diagnostics/record.py --label coverage-fast-turns-v3 --duration 900`
4. Wait for `Navigation recording active`, then execute once. Keep manual stop
  available. Record any pauses, rotations, clipping of rows or rejected ingress.
5. Cancel partway through on a separate run; wait for terminal `canceled`, then
  explicitly resume. Do not simultaneously issue teleoperation or other goals.

Use garden for live SLAM: the empty world contains only a ground plane and
produces all-infinite laser scans and a zero-sized map, which Nav2 rejects.

Resume uses the retained ordered route cursor, revalidates the remaining path,
and plans a fresh ingress. Changing/clearing the zone or requesting another
preview discards the previous resume route. To resume from a host terminal:

```bash
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c \
  'source /opt/ros/humble/setup.bash && ros2 topic pub --once /coverage/command std_msgs/msg/String "{data: resume}"'
```

Fresh TF, scan timestamps and full local costmaps are required. A blocked swept
horizon, lost ordered corridor or competing Nav2 navigation action requests
cancellation and retains the remaining route. `cancel_requested` is not a stop
confirmation: no new coverage action is allowed until the previous action has
terminated. Clock resets require a new preview. Independent direct
`FollowPath` clients and teleoperation are not centrally arbitrated.

Isolated container tests cover actual Fields2Cover output, Smac/MPPI activation,
ordinary `NavigateToPose` and `NavigateThroughPoses` motion with both checkers,
a saved-map ingress regression, exact overshooting joins, complete generated
row/perimeter execution, closed-loop completion, rotated zones, strict raw and
smoothed command bounds, cancellation races, and recorder path selection.
The faster-turn suite passed all 71 tests. Its synthetic coverage motion applies
the firmware's 0.35 rad/s wheel deadband, using 0.0937 m wheel radius and 0.295 m
track. Two six-metre rows achieved 1.35 cm settled straight-line p95 error; median
turn speeds across coverage cases were 0.174-0.199 m/s. Settled commands had no
below-deadband wheel episodes. Gates exclude the first and last second, require
less than 5% time below deadband and no continuous episode of 0.5 s, and verify
zero commands after completion/cancellation. These are test-model results, not
loaded motor measurements; coast inertia, stiction and wheel slip are not modeled.
Tests must source `/opt/nav2_mppi_install/setup.bash` after the workspace setup
to select the patched controller. They do not validate Gazebo dynamics, all MPPI
outputs, Raspberry Pi performance, real motor deadband, localization error or
25 cm physical clearance. The accepted Gazebo v3 trial completed a 49.76 m route,
with 3.12 cm settled straight p95 error and 0.199 m/s median measured turn speed;
see [the trial report](../wiki/navigation_baseline_2026-09-30.md).
Hardware integration then passed 86 isolated tests, including both clock modes,
namespace handling, preview generation, execution preflight and recorder clock
selection. Loaded motor behavior and Raspberry Pi controller timing remain
physical-test gates, not results established by these tests.

## Physical robot rollout

The accepted v3 settings are now active by default on the next hardware launch.
No firmware changes or flashing are needed. Build on the Raspberry Pi after
updating its checkout; a workstation image is not an ARM64 deployment artifact:

```bash
./start_robot.sh build ros2_nav2
```

Building does not restart the robot. When ready, cancel navigation, confirm the
robot is stationary and the blades are disabled, then recreate the stack:

```bash
./start_robot.sh down
./start_robot.sh up --dev
```

A plain restart does not load a rebuilt image. Verify wheel device mapping,
fresh lidar/map/TF, actual footprint and wheel geometry before the first trial.
Use a clear test area, keep blades disabled, and have a tested manual/emergency
stop available. Stop exploration and other navigation, preview a small route,
then start passive capture before Execute:

```bash
python3 helpers/navigation_diagnostics/record.py --label coverage-hardware-v3 --duration 900
```

Wait for `Navigation recording active`, execute once, and check stop/cancel
before testing a longer route and explicit resume. Coverage never automatically
detours or resumes a blocked route. Record straight tracking, the slower wheel
on turns, clearance, stopping and missed controller deadlines. Do not lower
clearance or disable freshness checks to force a test to run. A soft speed
preference is not a guarantee against loaded wheel stiction; assess that with
the actual robot before enabling mowing. This rollout does not redesign the
ordinary exploration controller.

## Building locally (outside Docker)

```bash
sudo apt-get update && sudo apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-topic-based-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-robot-localization

colcon build --packages-select diff_drive_hardware
source install/setup.bash
```

## Launching the diff-drive stack

```bash
ros2 launch diff_drive_hardware diffbot.launch.py
```

Environment variables (set before launch or in docker-compose):

| Variable | Default | Description |
| :--- | :--- | :--- |
| `WHEELS_TTY` | `/dev/ttyACM0` | Wheels Pico USB port |
| `IMU_TTY` | `/dev/ttyACM4` | IMU Pico USB port |
| `INA226_TTY` | `/dev/ttyACM2` | INA226 Pico 2 USB port |
| `KNIVES_TTY` | `/dev/ttyACM3` | Knives Pico USB port |

## Sending test commands

```bash
# Move forward 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## Monitoring joint states

```bash
ros2 topic echo /robot_joint_states
ros2 topic echo /joint_states
```

## Hardware plugin

`diff_drive_hardware` no longer contains a C++ serial plugin.  
The hardware interface is `topic_based_ros2_control/TopicBasedSystem`, configured in `src/diff_drive_hardware/config/diffbot.urdf.xacro`:

```xml
<hardware>
  <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
  <param name="joint_commands_topic">/robot_joint_commands</param>
  <param name="joint_states_topic">/robot_joint_states</param>
  <param name="trigger_joint_command_threshold">-1</param>
</hardware>
```

The wheels Pico firmware subscribes to `/robot_joint_commands` and publishes `/robot_joint_states` directly.

## RViz2

```bash
# From repo root
./rviz2.sh

# Or manually (WSL / Linux with X/Wayland)
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
  -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER \
  -e ROS_DISCOVERY_SERVER=192.168.40.120:11811 -e ROS_SUPER_CLIENT=True \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --network host osrf/ros:jazzy-desktop rviz2
```