# Pico-ROS: knives rollout plan

Updated **2026-10-20**. Scope: **knives only**. This plan does not authorize
flashing a working controller, restarting the robot, or energizing actuators.

## Decision and release status

Proceed forward with the separate `pico_ware_knives_picoros` application on the
[shared platform](pico_firmware/README.md). Existing micro-ROS sources remain
untouched; preparing or qualifying a rollback is not a rollout prerequisite.

```text
knives RP2040 -> USB serial -> serial-capable rmw_zenohd -> ROS/Zenoh
other RP2040 -> micro-ROS agent -> FastDDS -> uros_bridge -> ROS/Zenoh
```

The current commit candidate contains **native knives integration and motor-safety
implementation**, not production-qualified motor firmware. Hardware startup selects
Pico-ROS knives by default; ordinary simulation uses the Gazebo knives mock.
Wheels, IMU and INA226 are not migrated. Single-core FreeRTOS remains the
standard; measurements do not justify SMP, larger TX queues or a custom
reconnect supervisor.

Earlier platform/baseline findings remain in the
[readiness history](wiki/picoros_zenoh_production_readiness.md); the old
October 16 pause snapshot is not the current rollout state.

The existing XIAO hardware profile, pinout and encoder configuration are accepted
for this migration; the operator confirmed the pins on 2026-10-20. Keep forward
PWM/enable GPIO26/27, reverse PWM/enable GPIO29/6 and encoder GPIO7 unchanged.
Do not repeat board/pin/encoder confirmation as a separate work stage.

## What goes into this commit

- [x] Restore installed ROS router defaults; remove the incomplete replacement
  JSON5 and separate knives Compose override. Use one
  [router image](ros2_ws/ZenohRouter.dockerfile) and
  [startup script](ros2_ws/start_zenoh_router.sh) for hardware and simulation.
  Native startup changes only listen endpoints.
- [x] Include the pinned serial/hotplug patch in the ordinary router build.
  Require a verified USB by-id for hardware startup; default logging to `info`,
  with explicit optional debug. No host-specific image/board defaults.
  Keep `--sim --picoros-knives` only for the explicit physical-board/Gazebo test.
- [x] Exclude the native board from the micro-ROS agent by resolved identity,
  including raw tty and `TTY_*` aliases. No invented `/dev/ttyACM0`; a missing
  explicitly selected native identity fails hardware launch closed.
- [x] Keep all other controllers' channels in both bridges; omit knives
  channels in native hardware mode. Disable only the hybrid Gazebo knives mock.
- [x] Keep finite-command validation, +/-3500 RPM clamp and 5000 ms timeout
  refreshed only by valid commands. Retry initialization only for
  `PICOROS_NOT_READY`; native libraries own reconnect, not a publish-error loop.
- [x] Add zero-first command arming after boot/expiry/fault, immediate fault
  output disable, control-fed hardware watchdog and allocation/stack/assert hooks.
  Bound encoder inactivity, control gaps and normal active braking.
- [x] Reset UI target/cruise on WebSocket loss, prevent offline command queues
  and require released controls before a new gamepad command. Test actual
  ROSLIB publisher re-advertisement with a mocked transport as well as UI logic.
- [x] Separate KAmod no-actuator bench and explicit XIAO motor builds/UF2 names.
  Compile motor/encoder GPIO access out of bench; diagnostics out of motor mode.
- [x] Keep regression tests and the guarded bench acceptance script. Remove
  personal VS Code tasks containing local image/board IDs. Exclude binaries,
  generated builds, logs and JSON captures from Git; **preserve flash backups**.

## Exact settings

| Area | Integration setting |
| --- | --- |
| Firmware | Explicit `xiao_rp2040 motor`, using the existing hardware profile and confirmed pinout. Default `kamod_rp2040 bench` is not a robot image. |
| Topics | `std_msgs/msg/Float32`: `/knives/set_rpm`, `/knives/current_rpm`; retain generated type hash/CDR. Record actual QoS before powered tests. |
| Commands | Existing UI refresh 5 Hz (200 ms); bench stress 10 Hz; +/-3500 RPM; 5000 ms valid-command timeout. Five seconds is inherited compatibility, not certified stopping time. |
| Arming | First accepted command after boot, timeout or control fault must be finite zero; only subsequent nonzero is accepted. Float32 has no source timestamp or sequence. |
| Control / RPM / telemetry | 20 / 600 / 100 ms; retain existing PI/feed-forward, ramp, direction guard and PWM wrap. |
| Fault stop | Disable both enables/PWM without normal ramp/braking on timeout, control gap >100 ms or encoder inactivity >=2000 ms with controlled speed >=100 RPM and nonzero target. Encoder-loss check is motor-only; bench has no encoder. |
| Watchdog / braking | Hardware watchdog 2000 ms, fed after control/LED progress; allocation/stack/assert faults cut outputs and request reboot. Normal reverse braking bounded to 600 ms. Mechanical stopping time remains unqualified. |
| FreeRTOS | One core, 128 KiB heap; app/control stacks 3072/512 words (12/2 KiB). Measure motor/encoder and native transport-task headroom separately. |
| MCU USB | `serial/usb#baudrate=115200`, cooperative USB polling enabled, connectivity callbacks disabled; shared dependency pins/patches unchanged. |
| Router listeners | `tcp/[::]:7447` and `serial//hostdev/serial/by-id/<verified basename>#baudrate=115200`; this baud value is not a measured USB CDC throughput ceiling. |
| Router defaults | Open/accept 60000 ms; accept pending/max sessions 10000/10000; lease 60000 ms, keep-alive 2; SHM disabled; all TX queues 2. Inherit installed defaults, do not copy them into a replacement config. |
| USB access | `/dev:/hostdev:ro,rslave`, cgroup `c 166:* rwm`, persistent by-id and one owner; no fixed tty inode mount or privileged router. |
| Devices | [ros2_ws/zenoh_devices.json](ros2_ws/zenoh_devices.json) is the shared registry for serial listeners and micro-ROS exclusions. Fill `devices.knives.serial_by_id` with the exact `usb-...` basename and keep `baudrate: 115200`. Hardware/hybrid `up` validates it automatically; no manual environment setup. Add future native devices as named entries; disabled entries are ignored. |
| Runtime | One image: `ZENOH_ROUTER_IMAGE`, default `relobot-zenoh-router:local`; optional `ZENOH_ROUTER_RUST_LOG`, default `info`. Launcher sets internal `PICO_KNIVES_ENABLED` and `SIM_KNIVES_MOCK` from the selected mode. |

Build the single deployment image with `./start_robot.sh build ros2_zenoh_router`.
Then configure the real USB by-id in the JSON registry and use `./start_robot.sh up --dev` for the
approved hardware update: no knives-specific flag or Compose file is needed.
`./start_robot.sh up --sim` disables serial and does not read the device registry.
Only `--sim --picoros-knives` enables physical knives and disables
the Gazebo knives mock. `down`, `logs`, `ps` and `build` do not require a board
identity or connection. Direct Compose simulation users must set
`PICO_KNIVES_ENABLED=false`; the launcher handles this automatically.
Autostart reads the same JSON registry; no separate environment file is required
(see [autostart](README.md#raspberry-pi-5-autostart-configuration)).

## Verified versus unqualified

Only the spare **KAmod without actuators** was flashed on 2026-10-20, before the
new safety implementation. No current safety UF2 has been flashed.
The [bench runbook](pico_ware_knives_picoros/README.md#recorded-bench-evidence-2026-10-20)
records exact firmware/image IDs and local evidence locations.

- Valid/invalid/clamped/reverse/zero commands and local timeout passed.
- Gazebo, 30 s of 10 Hz commands: telemetry 9.932 Hz, largest gap 107.94 ms;
  zero target observed after 5.216 s with 1 Hz diagnostics.
- Heap free/min-ever 67640/64480 bytes; app/control remaining stack 1879/472
  words, publication errors zero. Motor/encoder and transport-task stacks untested.
- Full simulation + running bridge: 90 s passive telemetry at 9.973 Hz,
  no router restart/OOM or network fault in that capture.
- One physical unplug/replug: fresh telemetry 0.327 s after Linux by-id return,
  9.977 Hz afterward, no router restart. Serial `early eof` ERROR on removal
  was an expected disconnect, **not a daemon crash**.
- Isolated runtime/command, USB selection, router-config and Gazebo regressions;
  KAmod bench/XIAO motor cross-builds and C++ bridge compilation passed.

Not hardware-qualified: powered XIAO, watchdog/fault stop, stale-command protection,
TTY renumbering, repeated recovery/endurance, ARM64 router, actual DDS traffic
coexistence. Cross-building and starting bridge workers do not close these gates.

### Unified deployment router checks (2026-10-20)

Before the JSON-registry follow-up, the deployment Dockerfile built `relobot-zenoh-router:local` for
**amd64**, local image ID
`sha256:3a83ad49c6ae4239321d9937c9745544ca3b9c16e5f27c07f643997a2c85cf77`.
On that image, all 22 USB-selection/launcher/startup tests passed, the
packaged startup preserved installed ROS defaults in simulation and changed
only listen endpoints in native mode, and stock ROS CLI Float32 pub/echo passed
over isolated TCP. All eight PTY listener cases passed, including ten repeated
port replacements. Two Gazebo launch tests and rendered hardware/simulation/
hybrid Compose checks also passed; each mode defines the same single router.
Dockerfile checks, shell syntax and diff checks passed.

The JSON-registry follow-up rebuilt the **amd64** deployment image as
`sha256:81c2e43ab5a52844b0f6b051f908eaab8c5840146496cfd79ed404f3527fb2c7`.
All 31 JSON/USB-selection/launcher/startup tests passed in that image, including
adding a second listener and excluding both native devices from micro-ROS.
The packaged startup again preserved simulation defaults and changed only
listen endpoints in native mode. Compose rendering confirmed the read-only
JSON mount and removal of USB identity environment variables. C++ bridge
compilation and both Gazebo launch tests passed in an isolated ROS container.
The earlier eight PTY cases and TCP pub/echo were not repeated for this follow-up;
the serial library, its patch and firmware were unchanged.

These checks used isolated containers without physical USB and did not restart
the working stack or flash firmware. They do not qualify ARM64, real USB,
powered motors, DDS coexistence or mechanical stopping behavior.

## Ordered work before the real robot

### 1. Implement motor safety, then qualify on hardware

- [x] Restore 2000 ms hardware watchdog, fed from control progress rather than
  USB/idle. Enable stack-overflow detection and malloc/assert fault shutdown
  only for knives; other shared-platform applications retain their defaults.
- [x] Separate communication-loss output shutdown from normal requested stop.
  Test actual control logic with mocked time/GPIO: timeout, control stall,
  encoder loss, bounded braking, coasting telemetry and allocation/stack faults.
- [x] Require finite zero before nonzero after boot/timeout/control fault.
  Reject late nonzero before it can refresh an already expired command.
- [ ] Qualify watchdog/reset/GPIO behavior on the production controller with
  motor power disconnected. Software mocks cannot prove electrical shutdown.
- [ ] Decide strict source-freshness/session arming and test command backlog
  across recovery. Zero-first is only a local latch, not replay protection:
  a queued zero/nonzero pair or a producer's repeated zero can re-arm it.
  Float32 has no timestamp/sequence: local receive time cannot prove source
  freshness. Decide any stamped-command/producer change explicitly if strict
  source-age rejection is required. Do not rely on the pinned client's
  incomplete reconnect notifications for safety.
- [x] Update the [command producer](ros2_ws/src/web_server/frontend/js/services/gamepad-service.js):
  clear cached RPM/cruise on connection/error/close, publish zero first, never
  queue offline commands, require release of knife/cruise controls, and prevent
  hidden-page input from bypassing the stop. Clear state before sending stops,
  so a socket send failure cannot preserve the cached command.
- [ ] Qualify actual browser/WebSocket recovery separately from USB/router
  recovery. Mocked ROSLIB transport is not an end-to-end test; a healthy
  WebSocket does not prove the MCU is online.
- [ ] Verify E-stop independently of ROS/network/tasks; agree on actual safe
  stopping time. Retain 5000 ms only as a compatibility setting until that
  decision. Test NaN/Inf, lost commands/encoder, reset and reversal unpowered.

### 2. Prepare the deployment router

- [x] Integrate the pinned [serial-listener patch](helpers/rmw_zenoh_pico_spike/host_router/README.md)
  into the single [deployment router build](ros2_ws/ZenohRouter.dockerfile),
  with serial support and release/reopen on disconnect. Native knives need
  no second router or DDS/Zenoh transmitter. Reject a ROS vendor version other
  than the matching 0.10.6 rather than silently mixing library versions.
- [x] Replace the x86-only Rust builder pin with its verified multi-architecture
  index; retain identical amd64 inputs and explicit source/toolchain/lockfile pins.
  Verify the pinned ROS runtime index also contains amd64 and arm64/v8 manifests.
  Document a native ARM64 build using the same deployment Dockerfile.
- [ ] Build and qualify the **ARM64/RPi5** image. The development builder has
  no ARM64 worker; a manifest change is not a successful ARM64 build. Record
  immutable image reference and verify library/`rmw_zenohd` compatibility.
- [ ] Recheck effective defaults, initial missing-device behavior and repeated
  hotplug on the final image. The same serial-capable library is used in all
  modes; ordinary simulation does not enable a serial listener.

### 3. Close the DDS coexistence blocker

- [x] Add an independent two-process DDS smoke test using test-only topics and
  domain 211. On this WSL2 host (`6.18.33.2-microsoft-standard-WSL2`), middleware
  defaults pass three consecutive isolated-network/private-IPC runs (50/50,
  49/50, 49/50 samples) and two isolated-network/host-IPC runs (49/50, 50/50).
  Host networking fails with both private and host IPC. The differentiator is
  host networking here, not proof of a broken MCU or bridge serialization.
- [ ] Diagnose two independent FastDDS participants first: the lab sender saw
  itself, independent peers received **0**, with repository XML and middleware
  defaults, also inside Gazebo. A minimal loopback/unicast profile produces
  occasional passes but fails repeated trials; it is **not** an accepted fix.
  Keep production DDS XML unchanged. Determine the host-network discovery/data
  failure mechanism and repeat on the target Pi before changing DDS policy.
- [ ] Verify real wheel/IMU/INA226 telemetry and wheel commands through the
  existing agent + C++ bridge alongside native knives. No duplicate knives
  bridge/mock endpoints. This is validation, not migration of those controllers.
- [ ] Capture logs, exits/restarts/OOM and MCU resources under functioning
  full traffic. Only then attribute any network fault to serial congestion,
  stack limits or another cause; do not tune TX queues by guesswork.

### 4. Qualify and roll out one controller

- [ ] Production board, motor power disconnected: command reception after
  recovery, GPIO fail-safe, re-arm and watchdog; repeated USB loss/router
  restart, TTY renumbering, short/long interruptions and recorded endurance.
  Separate USB-powered MCU reboot from link-only interruption.
- [ ] After these gates and separate operator approval: controlled powered
  tests, physical E-stop available, measured RPM response/stopping behavior,
  resource headroom with encoder interrupts active.
- [ ] Deploy verified motor UF2/ARM64 image and actual robot by-id. Rebuild
  changed ROS packages in Docker (`--dev` for the update), recreate with
  the ordinary hardware launcher; restart alone does not apply image/environment/mounts.
  Subsequent production starts need no rebuild. Other controllers stay unchanged.

## Forward rollout and cleanup boundaries

The agreed rollout direction is forward to native knives, without a mandatory
return-to-micro-ROS stage or rollback qualification. This does not delete
existing firmware sources or archived backups. Set the production USB by-id
for serial ownership; never share a device between the agent and native listener.

Regression tests are source, not artifacts. Ignore generated binaries/captures;
preserve flash backups and prepared offline dependencies. Do not delete
historical experiments outside this change set. The standalone
`ros2_knives_bridge_probe` is a lab container, not ordinary Compose startup/teardown.
All builds, tests and ROS commands run inside Docker. No live-stack restart,
flash, powered test or further controller migration is implied by this plan.

Current safety builds on 2026-10-20 (generated artifacts, not flashed):

- KAmod bench SHA-256:
  `ea5171f4f7fa041538193af421d1b929463889c77f00124b0bd9b9e997cbb7e9`.
- XIAO motor SHA-256:
  `6a6e1eff29493d4418c21cf80c62a253dcd47bcc4f4cd7ad25d3a625c82714b9`.

These differ from the historical flashed image. Do not attribute the earlier
hardware measurements to the new safety code. Current software test commands
and their scope are in the [host test runbook](helpers/rmw_zenoh_pico_spike/host_tests/README.md).
Final software validation: **38 checks passed** (runtime 4, command/control 10,
serial/launcher 11, router config 1, Gazebo launch 2, UI including real ROSLIB 10).
Both cross-builds, Dockerfile checks, base/native/debug Compose merges and shell
syntax pass. DDS isolated controls pass, but host-network failures remain a
separate failed gate, not part of those 38 passes. Compose reports pre-existing
unset AWS/DNS variables; no credentials or unrelated services are changed.
No USB access, flashing or running-stack restart is performed in this preparation.
