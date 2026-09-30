# Navigation Tuning Lab

Reusable development and evidence workspace for coverage/navigation tuning.
This is not a second robot stack and is not launched by `start_robot.sh`.

## Layers and Ownership

| Layer | Location | Purpose |
| --- | --- | --- |
| Lab orchestration | [compose.yml](compose.yml), [Dockerfile](Dockerfile), [lab.sh](lab.sh) | Isolated tests and offline analysis |
| Passive capture | [record.py](record.py) | Explicitly attach to a running robot or Gazebo |
| Findings index | [findings.yaml](findings.yaml) | Baseline, rejected approaches, evidence IDs and open questions |
| Detailed history | [Navigation report](../../wiki/navigation_baseline_2026-09-30.md) | Chronological measurements and decisions |
| Layout illustration | [Comparison](../../wiki/coverage_layout_v2.png) | Historical layout comparison |
| Regression suite | [Nav2 tests](../../ros2_ws/src/nav2/test) | Geometry, controller motion, action races and diagnostics |
| Portable replay input | [Fixture documentation](../../ros2_ws/src/nav2/test/data/README.md) | Tracked compact ingress map/path; full bags not required for tests |
| Raw captures | `ros2_ws/log/navigation/<run-id>/` | Local immutable evidence; not included in Git |
| Runtime | [Nav2](../../ros2_ws/src/nav2), [geometry](../../ros2_ws/src/coverage_geometry), [MPPI patches](../../ros2_ws/patches) | Required by the robot; do not archive or remove |

Controller parameters, execution code, native geometry, ordered goal checker,
BT XMLs and MPPI patches are now real robot dependencies. Even
`navigation_metrics.py` contains the executor's ordered tracker. They remain in
place. Tests stay beside their implementation and reports retain their links;
this directory is the common lab entry point, not a duplicate runtime codebase.

## Build and Test

Run from the repository root. Docker/Compose are required, not host ROS:

```bash
./start_robot.sh build ros2_nav2 --sim --headless
bash helpers/navigation_diagnostics/lab.sh build
bash helpers/navigation_diagnostics/lab.sh test
```

Neither build starts, stops or recreates robot services. The lab image builds
Fields2Cover, opennav coverage/messages, geometry and Nav2 into `/lab/install`.
It does not use the host's `ros2_ws/install`, `ros2_ws/build` or historical
`relobot-navigation-fix` volume. The production image supplies pinned patched
MPPI, avoiding another implementation of the controller.

The container has no network, devices or Docker socket; a separate ROS domain;
and a read-only repository mount. Synthetic motion cannot reach live ROS
participants. Test outputs are in a disposable container, native builds in the
lab image. Tests read Python sources from the checkout.

```bash
bash helpers/navigation_diagnostics/lab.sh test -k 'recorded_ingress or directional_finish'
bash helpers/navigation_diagnostics/lab.sh test -k 'long_rows or closed_perimeter'
bash helpers/navigation_diagnostics/lab.sh test -k 'coverage_profile_is_shared or start_requires_ready'
```

Rebuild both images after native code, dependencies, installed resources or
MPPI patch changes. The default base is `ros2_ws-ros2_nav2:latest`; select a
retained local base with `NAVIGATION_BASE_IMAGE=<tag>` during the lab build.
A tag is not provenance: preserve source snapshots and image IDs per experiment.

Verified on 2026-09-30: independent five-package image build, **94 tests passed
in 329.59 s**, plus numerical v3 bag replay and clean JSON output. This does not
include an ARM build, a hardware trial or a live robot restart.

## Explicit Live Capture

Capture is deliberately separate from the isolated lab. The existing recorder
verifies the running `ros2_nav2` workspace mount and matches its hardware/system
or Gazebo/simulated clock. It never starts services or publishes motion/goals.

```bash
python3 helpers/navigation_diagnostics/record.py --label coverage-experiment --duration 900
```

Wait for `Navigation recording active` **before Execute**. Ctrl-C finalizes the
capture but does not stop the robot. Use its stop/cancel control. For hardware,
follow the [blades-disabled rollout](../../ros2_ws/readme.md#physical-robot-rollout).
Use garden for live SLAM; the empty world produces no usable map.

Keep the complete run directory: metadata, code/parameter snapshots, JSON
events/samples, logs and SQLite bag plus bag metadata. Captures may expose maps,
robot descriptions and local paths; inspect before sharing. Git-tracked reports
and small fixtures are not a backup of local recordings.

## Offline Analysis

Select an explicit capture ID; there is no implicit latest-run selection:

```bash
bash helpers/navigation_diagnostics/lab.sh analyze \
   20260930T110718Z-coverage-fast-turns-v3-dcdb7c > /tmp/coverage-v3-analysis.json
```

[analyze.py](analyze.py) runs inside the same isolated container, opens bags
read-only and emits JSON on stdout; container and ROS messages go to stderr.
Choose a durable output location outside the original capture for archiving.
The report includes capture/image identity and analyzer/tracker source hashes.
Retain the lab base image ID and Git revision alongside archived reports.

For a late-start recording, tracking is reconstructed from recorded TF at each
odometry timestamp. The shared ordered tracker is seeded **once** from the last
preceding manager progress, then remains bounded to its 0.6 m forward search.
It does not repeatedly snap to the nearest row. The original summary is retained
as explicitly uncorrected evidence, never overwritten. Straight error, turns,
slower-wheel speeds, command limits/coast duration and observed scan clearance
are reported separately. Completion is the recorded manager state, not an
independent assessment of mowed area or action-server correctness.

Supported input is one continuous-coverage execution with the standard root
topic names and `base_link`. Multiple execution paths, resume intervals,
backwards timestamps and clock resets are rejected. Older waypoint/exploration
recordings still have their original reports but are not inputs for this
coverage-specific reconstruction. Without a bag, only sampled evidence is
available. Missing TF/commands/joints/parameters are reported as unavailable or
zero samples, not zero error/speed. Durations cover only observed intervals;
the v3 recording omits startup/ingress. Long message gaps warrant inspection.

The v3 replay reproduces 6,448 tracking samples, straight p95 0.03118135 m,
maximum path distance 0.13716063 m, and 0.035 s below the command wheel deadband.
Run the analyzer's bounded-input and metric regressions with:

```bash
bash helpers/navigation_diagnostics/lab.sh test -k offline
```

## Tuning Protocol

1. Read the findings index and report before repeating an experiment. Record
   source revision, dirty diff and image ID.
2. Change one behavior at a time; reproduce it with a focused test first.
3. Run the full suite, including ordinary navigation and stopping.
4. Run a user-operated Gazebo trial, recording before Execute. Different zones
   and routes do not establish a controlled speedup.
5. Only then schedule hardware testing. Loaded wheels, slip, physical clearance
   and Raspberry Pi controller timing require physical measurements.
6. Preserve both successful and rejected approaches in the findings/report.

Acceptance retains a 0.15 m ordered tracking corridor, 0.05 m settled straight
p95 target, forward-only coverage and 0.20 m minimum controller radius. Wheel
checks exclude the first/last second, require less than 5% time below 0.35 rad/s
and no episode lasting 0.5 s. These are test gates, not loaded-hardware guarantees.

## Storage and Cleanup

Keep lab files, runtime sources, tests/fixture and wiki reports in Git. Back up
raw captures separately under their run IDs. Never rewrite a capture to replace
a misleading summary; save derived analyses separately with input and analyzer
provenance. No files have been moved, deleted or committed automatically.

`relobot-nav2-candidate` and `relobot-navigation-fix` are historical scratch
artifacts, not lab inputs. They have not been deleted. The production image is
still used by the robot. Removing only `relobot-navigation-lab:latest` removes
rebuildable lab dependencies, not source or evidence.
