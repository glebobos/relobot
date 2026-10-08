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
Exploration and ordinary navigation use a separate stock MPPI `FollowPath`
profile with `DiffDrive`; coverage retains its existing Ackermann profile. Shared
costmaps now include 25 cm footprint padding and 0.95 m inflation, and the smoother
runs at 20 Hz. These shared costmap/smoother changes also affect ordinary
navigation on hardware; its original ordinary goal checker is retained.
Both ordinary navigation trees explicitly select `goal_checker`; coverage selects
`coverage_goal_checker`. Omitting `goal_checker_id` from a `FollowPath` BT node
fails in Humble when both goal checkers are loaded.

Ordinary MPPI is independently configured rather than restored from the
historical MPPI profile: 0.40 m/s forward ceiling, no reverse/lateral motion,
1.0 rad/s yaw ceiling and a 60 x 0.05 s prediction horizon at 20 Hz. DiffDrive
permits turn-in-place without imposing coverage's minimum turning radius.
Native `CostCritic` footprint scoring stays enabled. PathAlign has
weight 6 rather than coverage's 20, and there is no coverage velocity-deadband
preference or positive speed floor. GoalAngle is enabled only within 0.25 m,
matching the ordinary XY tolerance, so forward-only approach reaches position
before aligning final yaw. PathAngle remains available down to the same 0.25 m
handoff instead of stopping at 0.50 m; this user-approved soft tuning retains
heading guidance nearer the goal. Ordinary XY/yaw tolerances remain 0.25 m/rad.
The shared smoother permits 0.50 m/s but does not force it; both hardware and
Gazebo drive controllers already have a 0.50 m/s ceiling. Coverage remains
limited to 0.20 m/s by its unchanged controller. Acceleration/deceleration,
body clearance and coverage constraints are unchanged. The 0.50 m/s ordinary
MPPI profile has not been validated; 0.40 m/s is the initial rollout target.

Both profiles use the unmodified `ros-humble-nav2-mppi-controller` package and
the `nav2_mppi_controller::MPPIController` plugin. `GridBased` uses packaged
`SmacPlannerLattice`; no Hybrid/Reeds-Shepp patch or private Nav2 source overlay is
built. The unused exact-connector Python extension has also been removed;
`coverage_geometry` retains the ordered goal checker. Fields2Cover and opennav
coverage remain available for legacy ordered-swath planning.

### Humble Dependency Update (2026-10-08)

The shared `ros2_nav2` image uses the official `ros2-testing-apt-source`
package and upgrades installed Ubuntu/ROS packages without removing existing
packages. This remains Humble on Jammy, but testing is the pre-stable soaking
channel, not the stable ROS repository. Both `tf2` and `tf2_ros` must be at least
0.25.24; the Docker build fails if either minimum is unavailable.

Version 0.25.24 contains the upstream waitForTransform/testTransformableRequests
ABBA deadlock fix ([Humble backport](https://github.com/ros2/geometry2/pull/990),
[matching report](https://github.com/ros2/geometry2/issues/992)). Recorded fresh
external TF alongside a frozen local footprint strongly matches this issue,
but the robot's root cause is not confirmed without an upgraded manual trial.
No TF timeout, composition setting, controller tuning or safety limit changes
are part of this dependency update.

The built amd64 image was checked with `dpkg-query` and `dpkg --audit`:
tf2/tf2_ros 0.25.24, Nav2/MPPI/Smac 1.1.20 (September 15 rebuilt binaries),
rclcpp 16.0.21, rmw_fastrtps_cpp 6.2.10, Fast DDS 2.6.12,
slam_toolbox 2.6.10 and opennav docking 0.0.2. Exact package versions and image
identity are recorded in [findings](../helpers/navigation_diagnostics/findings.yaml).
Fields2Cover stays at 1.2.1 with opennav coverage 0.0.1. Fields2Cover's latest
release is 2.1.0, but adopting its changed API is a separate migration;
opennav coverage's latest published release remains 0.0.1.

Build only (does not restart services):

```bash
./start_robot.sh build ros2_nav2 --sim --headless
```

When the operator is ready to recreate Nav2 in simulation:

```bash
./start_robot.sh up ros2_nav2 --sim --dev --headless
```

Use `--dev` to rebuild workspace packages against the new libraries. Image
build/package integrity passed; workspace compilation, manual startup/undocking,
coverage recovery and Raspberry Pi/arm64 validation remain unverified. No live
services were restarted and no motion tests were run for this update.

The shared hardware/simulation planner uses Humble's packaged differential-drive
lattice at 5 cm resolution: 16 headings, 0.5 m curved-motion radius and in-place
rotation primitives. Reverse expansion is disabled. Rotation penalty is 5.0,
cost penalty is 3.0, and planning retains its 2 s budget and 0.5 m goal-search
tolerance. Post-smoothing is disabled to retain lattice poses/headings, and the
obstacle heuristic is not cached across changing SLAM maps. MPPI limits, the
work profile, footprint padding and clearance checks are unchanged.
Parameter names and the primitive format were checked against the
[Humble 1.1.20 Lattice implementation](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_smac_planner/src/smac_planner_lattice.cpp).

Lattice configuration/provenance and cruise checks passed (6 cases). The
isolated obstacle trial reached its goal in approximately 31 s, but still
failed the ordinary command gate: stock MPPI emitted linear velocity as low
as -0.00532 m/s versus the approved -0.001 m/s floor. No acceptance threshold
was widened; the planner replacement is an operator trial, not full rollout
acceptance. No live robot restart was performed.

Coordinated ordinary MPPI terminal-handoff probes were subsequently rejected:
1.2 m Goal/PathFollow thresholds with either 0.50 m or 0.25 m heading thresholds
failed the heading-case body-clearance check. The original 1.0 m Goal, 0.5 m
PathFollow and 0.25 m heading thresholds remain active. The restored baseline
reaches the heading goal but still violates the negative-command gate, and an
obstacle repeat times out; terminal looping is not resolved. See the
[recorded probes](../helpers/navigation_diagnostics/findings.yaml).

Stock Humble scores sampled trajectories but does not revalidate the final
filtered control sequence. The former custom post-filter constraints and filled
footprint guard are intentionally absent. Coverage-manager route/current-body
clearance, unknown-space, freshness and measured-stop checks remain unchanged;
they are not an equivalent final-sequence guard. Noise sampling and lifecycle
handling are upstream behavior, not patched to preserve deterministic samples.
Clean-image motion checks must pass before rollout; see the current findings.

Before the Lattice replacement, the 2026-10-07 clean-image suite finished with
169 passes and four failures:
recorded-ingress wheel coasting, work forward/radius command violations, and
a managed directional mission abort. Two native work-critic probes did not
resolve these gates and were reverted. Rollout remains blocked; no live
services have been restarted for this cleanup.

The upstream [MPPI tuning notes](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller#notes-to-users)
also apply to Humble's horizon/path-offset balance. Ordinary PathAlign uses
an offset of 8 points: 3 s x 0.40 m/s / 0.05 m path resolution / 3. This avoids
the startup rotation stall seen with the previous offset of 6 without changing
sampling spreads, safety checks or coverage tuning. `model_dt: 0.05` matches
20 Hz; the 1.6 m ordinary prune distance covers its 1.2 m projected travel.
Use the Humble parameter schema, not newer `main` motion-model plugin syntax.

Isolated ordinary-navigation tests exercise NavigateToPose, NavigateThroughPoses,
initial rotation, final yaw, static obstacle detour and return to the target,
cancellation, cruise-speed attainment and zero-command stopping. They use the existing 0.35 rad/s wheel
deadband model and check the actual body against occupied/unknown cells with
25 cm clearance. Arrival checks include global-planner cell quantization;
controller tolerances themselves are not relaxed. Raspberry Pi timing, loaded
motor behavior and full live trials remain separate rollout gates.
Tests permit user-approved stock-filter overshoots of 0.01 m/s and 0.05 rad/s
for ordinary commands, with ordinary speeds down to -0.001 m/s treated as
numerical zero, and 0.005 m/s overshoot for work speed. The work-turn median
performance floor is 0.13 m/s. These allowances do not change configured speed
limits, forward-only work, minimum radius, clearance, freshness or stop checks.
Ordinary final-yaw observations allow 0.01 rad beyond the unchanged configured
0.25 rad tolerance for action/odometry sampling; coverage endpoints are unchanged.

Coverage previews contain separately validated forward work sections: directional
rows and rounded perimeter portions, using GEOS polygon operations. Legacy
`layout_mode: fields2cover` converts the server's ordered swaths to straight work
sections instead of executing its connector-filled `nav_path`.
Execute sends ordinary `NavigateToPose` to each work section's first pose,
including its heading. The normal BT uses `GridBased` and `FollowPath`
(`DiffDrive`, up to 0.40 m/s), so coverage's radius and speed constraints do
not apply to approach. Goal UUID ownership prevents its status from being
mistaken for competing navigation; external navigation still interrupts it.
After terminal success, fresh odometry must confirm a stable actual stop.
The route is revalidated before coverage begins, with dense validation off
the ROS callback. Actual start position/heading errors are reported, not used
as an additional hard tracking gate. `/coverage/execution_path` and
`/coverage/control_path` contain only the current work section, so native ordered
goal checking cannot mistake a transfer for work progress. Work completion also
requires fresh ordered progress and an actual stop before the next transit.

Only `FollowPath` and `CoverageFollowPath` are active controllers. The specialized
connector planner, signed cusp execution, forward/reverse maneuver profiles and
their endpoint goal checkers have been removed. `CoverageIngress`,
`start_planner_id`, `allow_reverse_maneuvers`, `planner_assisted_turns` and
connector-request budgets are no longer runtime settings. Work stays forward-only
with its existing 0.20 m/s ceiling, 0.20 m controller radius and 0.05 m endpoint
tolerance. Ordinary navigation retains its existing speeds and goal tolerances.

**Transfers are ordinary replanning, not guaranteed planned reverse.** Nav2 may
rotate or use its configured recoveries. Transfers may leave the selected work
zone but must remain in mapped free space. Preview does not promise their exact
geometry, duration or reachability: an unavailable transfer blocks execution
without silently omitting the next work section. Explicit work boundaries are
retained through cancellation, recovery and Resume. Mission distance and area
estimates exclude virtual lines between work sections.

Focused native tests complete three forward passes with two ordinary transfers
and the recorded obstacle approach while retaining physical-deadband and body
clearance checks. The complete new-suite result is recorded separately from the
historical 177-pass/2-failure approach-only run. Browser verification is left to
the user. Live Gazebo, Raspberry Pi timing and loaded hardware remain unverified.
See [validation findings](../helpers/navigation_diagnostics/findings.yaml).

The coverage critic weights for work passes, speed limits and body clearance
are unchanged. Tracking deviation beyond
0.15 m is a warning, not a cancellation trigger. Path critics provide soft
attraction to the route; 0.30 m and 0.60 m deviations retain warning-only
behavior. Collision, stale-input and actual-stop checks remain hard safeguards.
Successful coverage actions refresh ordered progress from current TF before
endpoint checks, instead of relying on an older timer sample.

The profile uses 0.20 m/s maximum speed, a 0.20 m controller turning
radius and 0.25 m planned radius. Ordinary Dubins uses the upstream spelling
`DUBIN` / `DISCONTINUOUS`: position and heading are continuous, but curvature
changes at arc boundaries. The installed continuous-curvature generator failed
the real-server test with a sideways join discontinuity.

The faster-turn profile adds a soft `VelocityDeadbandCritic` preference for
0.18 m/s (weight 35, zero angular deadband). PathAlign uses positional error
without the extra heading term, and hands off final approach at 0.20 m. The
0.15 m tracking-warning threshold and 0.05 m goal tolerance remain unchanged.
Ordered projection retains a bounded 0.6 m forward window and accepts up to
0.5 m projection error consistently in Python/native trackers; a closed path
still cannot finish immediately at its shared start/end. The executor checks
the actual current footprint in the local costmap frame. It does not invent a
straight return chord from an off-path robot to the nominal route: the native
MPPI critics score candidate trajectories instead.
No positive minimum velocity or downstream speed clamp is used: completion,
cancellation and safety checks can still stop the robot.

Directional rows use at most 0.24 m spacing, retain separate segments around
obstacles and visit neighboring rows in alternating directions. Independently
validated outer and obstacle perimeter portions remain work, not special
connector geometry. Preview never publishes motion commands.

With `open_segments: true` (the default runtime profile), work is split into
open straight and arc segments, targeting at most `work_segment_length_m: 6.0`.
An entire outer/obstacle perimeter does not have to fit or close: safe edge
portions are retained independently, with 0.20 m minimum work length. Both
forward traversal directions are tested with the asymmetric footprint.
Safe work is no longer omitted merely because a forward-turn connector cannot
be generated. Unsafe work portions remain omitted and reported. Each work section
ends with a stop and the next section uses ordinary navigation; this is not one
continuous `FollowPath`. Set `open_segments: false` to retain validated complete
rows/closed perimeters as individual work sections.

Clear the custom zone to select full-map coverage. A fresh robot TF selects its
known-free map component rather than the largest contour; disconnected free
space is reported as excluded. Custom selections are intersected with known free
space. Occupied/unknown cells, including single-cell holes, remain forbidden.
Directional map extraction does not apply the legacy `map_erode_m` and
`obstacle_dilate_m` insets before applying body clearance again. Map-origin yaw
is respected. Other Fields2Cover requests keep their existing extraction mode.

Straight-side and row-end margins remain conservative; perimeter corners account
for the asymmetric body sweep. Every accepted work path is checked against the
original occupancy grid with the filled swept footprint.
The 25 cm body clearance applies to custom selections as well as obstacles.

The old 400 m2, 50 m span, 120-row and 15,000-pose whole-zone limits are replaced
by explicit resource budgets: `planning_timeout_sec: 120.0`,
`max_path_points: 250000`. Exceeding a budget rejects the preview,
never truncates an executable route. The dense canonical path is retained for
execution/resume; the browser receives a reduced display path and timestamp-matched
`/coverage/preview_sections` boundaries. It draws independent work lines without
fictitious transfer chords or silently truncating the preview. Long preview,
execute/resume and ingress checks run on workers, keeping ROS callbacks active.

Coverage scan/full-costmap subscribers use `BEST_EFFORT`, `VOLATILE`,
`KEEP_LAST(1)`: queued obsolete snapshots are not replayed after heavy work.
Identical full costmaps reuse the validator only when frame, origin, resolution,
dimensions, data and footprint/clearance match; timestamps and reception age
still update for each message. Dense canonical routes are treated as immutable
and are not copied in the Execute callback. Freshness limits are not extended.
An invalid timestamp reports both ROS age and wall reception age in the logs.

Status reports work/edge segment counts, unconnected work, omitted work length,
`partial_coverage`, short omitted sections, infeasible perimeter passes,
excluded map area and estimated uncovered area. Open-mode summaries and failed
previews are also logged by `coverage_manager`. The estimate uses a base-link-centred
strip of `swath_spacing` width, not a calibrated blade offset, measured mowing,
or proof of physical reachability. An accepted route may leave edge/corner
strips that the body and forward-only manoeuvres cannot reach. Open-mode
connector failures do not discard other reachable work; omitted segments are
not claimed as covered. If no safe segment remains, or any total resource
budget is exhausted, the preview is rejected. Strict mode still rejects an
unconnectable row. Dynamic obstacles remain in the costmap; the bounded recovery
below revalidates the retained route and replans ingress. There is no
global coverage relayout or reverse motion in normal work passes.
**This is not edge-to-edge mowing or a measured coverage-area guarantee.**
Execute revalidation retains the original partial-coverage metadata instead of
reporting a falsely complete mission. Backend status includes `can_execute`;
after a temporary block, a valid cached route can be retried manually. The
backend repeats preflight/map checks, invalid selections remain disabled, and
no status update automatically starts or resumes motion.

Coverage recovery is explicitly enabled by `recovery_enabled: true`. Only an
ABORTED FollowPath with confirmed terminal action state can initiate it. The
executor requires fresh TF/scan/full local costmap and validates the retained
route and current footprint before replanning. `recovery_backup_enabled: false`
disables coverage-managed BackUp by default; planned reverse is not blind recovery.
Ordinary approach retains the normal navigation BT's configured recoveries.
Explicitly enabling backup additionally checks a complete reverse swept
footprint, then sends the standard Nav2 `BackUp` action for at most
`recovery_backup_distance: 0.15` m at `recovery_backup_speed: 0.05` m/s. Freshness
and rear-space checks continue during backup; the action has an 8 s ROS-time
allowance and a 15 s wall-time watchdog. Work passes stay forward-only.
At most `recovery_max_attempts: 2` recovery attempts are allowed per manual execution.
Remaining poses are retained, validated on a worker, and ordinary navigation
returns to the retained start before stop handoff and following resume.
Progress then refers to the retained path.

No backup starts with stale inputs, occupied/unknown rear swept space, competing
navigation or exhausted budget. Operator Stop takes priority in every phase,
including late action acceptance and background validation. Sensor/TF faults
and current-footprint collisions require explicit retry; they are not
automatically bypassed. Planner/backup failure leaves the mission blocked with
its remaining route, rather than silently skipping work. The existing offline
analyzer rejects multiple execution paths, including recovered missions; its
single-route summary is not a complete recovery analysis.
See [the same-zone comparison](../wiki/coverage_layout_v2.png).

Coverage uses `relobot/VerifiedMPPIController`, a version-pinned Nav2 1.1.20
patch built into the image. It reapplies motion constraints after the upstream
Savitzky-Golay filter, checks the resulting filled padded-footprint sweep
against obstacles/unknown/out-of-map space, and stores the constrained command
in filter history. It does not modify published commands downstream. The
coverage goal checker also requires ordered path progress, preventing premature
completion at a closed perimeter's start/end. Ordinary navigation retains its
original checker and separate ordinary MPPI profile.
Native rejection diagnostics distinguish `phase=current_pose` from
`phase=prediction` and report predicted time/index, frame, pose, grid cell,
world position, lethal/unknown/out-of-map type and map geometry. Formatting is
performed only on rejection; collision predicates and padding are unchanged.

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

Fresh TF, scan timestamps and full local costmaps are required. A blocked actual
footprint or competing Nav2 navigation action requests cancellation and retains
the remaining route; tracking deviation alone is a warning. An aborted
controller may use the bounded guarded recovery above. `cancel_requested` is not a stop
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