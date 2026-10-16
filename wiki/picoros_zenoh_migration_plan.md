# Pico MCU migration to Pico-ROS over Zenoh

For tested hardware results, executable commands and outstanding production
gates, see the [Pico-ROS USB experiment handoff](./picoros_zenoh_production_readiness.md).

## Decision

Do not replace micro-ROS with raw Zenoh-Pico publications. Raw Zenoh moves
byte payloads but does not by itself provide ROS 2 CDR serialization, RIHS type
hashes, RMW attachments, or ROS graph liveliness declarations.

Use a ROS-aware Zenoh compatibility layer on each MCU and Zenoh-Pico as its
transport. Keep `rmw_zenoh_cpp` and the Zenoh router on the host. After the
RP2040 transport experiments, the selected next hardware prototype uses
**existing Pico-ROS**, not a newly written compatibility layer. It reuses
micro-ROS-generated ROS message types, hashes and CDR callbacks, but does not
link its XRCE RMW or run an agent. `rmw_zenoh_pico` stays as an isolated
compile-time comparison; do not mistake it for the chosen production path.

Two implementations were evaluated:

- Pico-ROS is usable now as a protocol/interoperability reference and its
  typed host proof passes against ReloBot's Lyrical router.
- `rmw_zenoh_pico` is the better long-term application API because it sits
  below `rcl`/`rclc`; existing firmware can retain its nodes, executors, and
  generated ROS type support. Its current upstream is not ready for ReloBot's
  RP2040 USB serial deployment, so it requires a bounded port before adoption.

Do not remove the current micro-ROS agent or `uros_bridge` until every board
passes the exit gates below. The bridge remains the rollback path during the
migration.

## Critical pushback and hidden gaps

1. **The video is not an RP2040 production recipe.** It demonstrates ESP32-S3.
   Official Zenoh-Pico supports RP2040 serial and USB CDC, but calls USB CDC
   experimental and requires its unstable API.
2. **Zenoh-Pico is not sufficient for ROS 2 interoperation.** `rmw_zenoh_cpp`
   uses CDR payloads, a key containing domain, topic, DDS type and RIHS hash,
   an RMW attachment, and `@ros2_lv` graph declarations. Pico-ROS implements
   these pieces.
3. **Pico-ROS does not pin upstream Zenoh-Pico.** Commit
   `b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2` pins
   `g4sp3r/zenoh-pico@0eb81272682a526b4bcf5a850c16ac74bbc6c73e`.
   At review time that fork was 12 commits ahead and 40 commits behind
   Eclipse main. Upgrade only after repeating the compatibility tests.
4. **Distribution compatibility is exact, not aspirational.** Pico-ROS issue
   #19 documents protocol and type-hash failures across RMW versions. The
   pinned Pico-ROS build was therefore tested against ReloBot's actual Lyrical
   router image rather than assumed compatible.
5. **QoS is not yet proven equivalent.** The reviewed Pico-ROS code emits one
   fixed ROS liveliness QoS string. ReloBot currently has reliable command
   subscriptions and best-effort telemetry publishers. Bidirectional
   compatibility must be measured for each topic.
6. **Time synchronization is missing from the proposed replacement.** Current
   firmware uses `rmw_uros_epoch_nanos()`. Pico-ROS timestamps its transport
   attachment but does not automatically populate ROS message header time.
   IMU and wheel state migration is blocked until MCU-to-host clock behavior is
   specified and tested.
7. **Serial reconnection is a safety feature.** USB reset, cable disconnect,
   router restart, malformed frames, and lease expiry must not leave stale
   motor commands active. Wheel and knife firmware fail-safes must remain
   local and independent of middleware state.
8. **`rmw_zenoh_pico` is promising but not drop-in.** The reviewed main branch
   targets ROS 2 Jazzy and Zenoh-Pico 1.4.0, has no release, and explicitly
   states that serial is not currently supported. Its CMake declares
   `WITH_RPI_PICO` but does not select an RP2040 platform implementation. The
   provided firmware workflows build Linux and Raspberry Pi OS targets, not
   Raspberry Pi Pico firmware. The isolated ReloBot spike now cross-builds an
   RP2040 UF2, but only after adding the missing serial configuration, FreeRTOS
   condition and clock paths, RP2040 logging/GID support, and QoS fixes. Graph
   caching and practical QoS are incomplete,
   and open pull requests address service sequence IDs, guard conditions, and
   publisher/subscriber matching.

## Evidence

Authoritative references:

- [Zenoh-Pico README](https://github.com/eclipse-zenoh/zenoh-pico/blob/8e204c431f7d4a47fa6d47e02f99d3f14cc01f04/README.md):
  RP2040 transport matrix, UART locators, experimental USB CDC flags, and host
  serial locator.
- [rmw_zenoh design](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md):
  CDR serialization, ROS topic key expressions, type hashes, attachments, and
  liveliness-token graph discovery.
- [Pico-ROS README](https://github.com/Pico-ROS/Pico-ROS-software/blob/b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2/readme.md):
  Pico-ROS/Zenoh-Pico/RMW integration and lease requirements.
- [Pico-ROS protocol mismatch issue](https://github.com/Pico-ROS/Pico-ROS-software/issues/19):
  evidence that protocol, distribution, type hash, liveliness, and lease
  versions must be aligned.
- [rmw_zenoh configuration](https://github.com/ros2/rmw_zenoh/blob/rolling/README.md):
  `ZENOH_ROUTER_CONFIG_URI` for a custom router configuration.
- [`rmw_zenoh_pico` README](https://github.com/esol-community/rmw_zenoh_pico/blob/54e20a460c9533d279e07a581d0e69e647d60a0e/README.md):
  micro-ROS RMW architecture, Jazzy/Zenoh-Pico 1.4.0 pins, unsupported serial,
  supported Linux/Raspberry Pi OS builds, and graph/QoS limitations.
- [`rmw_zenoh_pico` CMake](https://github.com/esol-community/rmw_zenoh_pico/blob/54e20a460c9533d279e07a581d0e69e647d60a0e/rmw_zenoh_pico/CMakeLists.txt):
  declared but unwired `WITH_RPI_PICO` option.

WSL result on 2026-10-15:

| Check | Result |
| --- | --- |
| Attached `/dev/ttyACM*` devices | None; serial hardware test blocked |
| Pico-ROS pinned source build | Pass |
| Pico-ROS serialization executable | Pass |
| ReloBot Lyrical router starts on TCP 7447 | Pass |
| ROS graph discovers `/picoros/chatter` as `std_msgs/msg/String` | Pass |
| ROS decodes `Hello from Pico-ROS!` | Pass |
| `rmw_zenoh_pico` RP2040 cross-build | Pass; ELF, BIN, HEX, and UF2 generated |
| Middleware symbol audit | Pass; Zenoh/RMW symbols present and no XRCE symbols linked |
| RP2040 static footprint | 183,292 bytes text, 138,180 bytes BSS, approximately 350 KiB UF2 |
| `rmw_zenoh_pico` typed Lyrical interoperability | Not tested; requires attached board |
| RP2040 USB CDC and reconnect | Not tested; requires attached board |

The repeatable host test is
[`helpers/picoros_poc/run_host_poc.sh`](../helpers/picoros_poc/run_host_poc.sh).

Subsequent spare-board KAmod RP2040 test (`0xE464C498C3380C35`):

| Check | Result |
| --- | --- |
| Official raw Zenoh-Pico publisher over USB | Pass; router received increasing raw samples |
| Pico-ROS + generated `std_msgs/msg/Float32` over USB | Pass; Lyrical graph found `/picoros_usb_spike/value` and decoded `39.0`, `40.0`, `41.0` |
| Type hash and publisher endpoint | Pass; `RIHS01_7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2` |
| Router restart before USB disconnect patch | Fail; Pico stayed green but ROS publisher never reappeared |
| Router restart with fork USB disconnect patch | One pass; CDC re-enumerated, WSL was manually reattached, ROS decoded `91.0`, `92.0` without MCU reset |
| Physical USB unplug/replug with fixed tty/by-id-only mount | Router stayed alive but saw a stale empty by-id bind mount after WSL reattachment; topic disappeared until router restart |
| Physical USB unplug/replug with parent `/dev` mounted | One pass; same router process retried the serial link, ROS topic and increasing `13.0`, `14.0` returned after manual WSL attachment |
| Different tty minor, repeated reconnects, soak test | Not proven; do not migrate production boards yet |

The patched RP2040 USB transport reports DTR loss on read/write so the
upstream Zenoh-Pico auto-reconnect path can act; it does not change ROS/Zenoh
wire protocol. On WSL, CDC deinitialization/re-enumeration requires manual
USBIP reattachment. On the robot, the router must regain access to the new
tty via the stable by-id link, including container device permissions. A
by-id-only directory mount became stale after WSL recreated that directory;
Zenoh's serial listener was already retrying but could not see the new path.
The main router service now mounts the stable parent `/dev:/hostdev:ro`,
grants `ttyACM*` cgroup access, and accepts an opt-in config URI. One
hotplug test recovered ROS data without a router restart, but the device
reused ttyACM0; test actual tty renumbering and repeated recovery on the
robot before moving beyond the spare board.

## First-principles target

```text
RP2040/RP2350 firmware
  application + local fail-safe
  rclc
  ROS-aware Zenoh RMW/compatibility layer
  Zenoh-Pico serial/USB CDC
             |
             v
rmw_zenohd router with one stable serial endpoint per board
             |
             v
ROS 2 Lyrical nodes on rmw_zenoh_cpp
```

There is no micro-ROS agent, Fast DDS domain, forked bridge process, socketpair,
or second serialization hop in the final control path.

The earlier preference was a repaired `rmw_zenoh_pico` to preserve `rclc`.
On the test KAmod its support initialization stopped before USB startup,
while the official raw Zenoh-Pico publisher worked over USB. The selected
upstream Pico-ROS target on RP2040 uses generated micro-ROS type support and
has now passed typed ROS discovery and CDR decoding on the spare KAmod.
Application callbacks still need migration from `rclc`. Pico-ROS supplies
the Zenoh key, liveliness and attachment protocol; these must not be
reimplemented ad hoc in individual firmwares.

Keep current ROS topic names during migration:

| Board | Direction | Topic | Type | Current rate |
| --- | --- | --- | --- | --- |
| Wheels | Host to MCU | `/robot_joint_commands` | `sensor_msgs/msg/JointState` | controller driven |
| Wheels | MCU to host | `/robot_joint_states` | `sensor_msgs/msg/JointState` | about 30 Hz |
| Knives | Host to MCU | `/knives/set_rpm` | `std_msgs/msg/Float32` | command driven |
| Knives | MCU to host | `/knives/current_rpm` | `std_msgs/msg/Float32` | 10 Hz |
| IMU | MCU to host | `/imu` | `sensor_msgs/msg/Imu` | about 30 Hz |
| INA226 | MCU to host | `/battery_voltage` | `std_msgs/msg/Float32` | 10 Hz |
| INA226 | MCU to host | `/charger_voltage` | `std_msgs/msg/Float32` | 10 Hz |
| INA226 | MCU to host | `/on_dock` | `std_msgs/msg/Bool` | 10 Hz |

Topic renaming is a separate change and is deliberately excluded.

## Trade-off matrix

| Dimension | Current dual-RMW bridge | Pico-ROS direct Zenoh |
| --- | --- | --- |
| Host processes | Agent, Fast DDS worker, Zenoh worker | Zenoh router only |
| ROS compatibility | Supplied by micro-ROS and bridge | Supplied by Pico-ROS; version-sensitive |
| Serial maturity | Existing and working | USB CDC experimental upstream |
| Copies/hops | XRCE, DDS, socketpair, Zenoh | CDR over Zenoh serial |
| Rollback | Current production path | Keep current firmware UF2 and bridge until final cutover |
| Main risks | Complexity and reconnect races | immature transport, QoS, time sync, fork drift |

## Migration phases and gates

### Phase 0: freeze and measure

1. Save known-good UF2 files for every board.
2. Capture topic types, QoS, rates, serialized sizes, reconnect time, CPU and
   memory on the robot.
3. Record stable `/dev/serial/by-id` identities.
4. Pin the Pico-ROS, Zenoh-Pico, Pico SDK, FreeRTOS, Micro-CDR, and ROS
   interface-generation revisions. Do not build from moving branches.

Exit gate: the current system can be restored by flashing one UF2 and reverting
the router/compose selection.

### Phase 1: host compatibility

Run:

```bash
./start_robot.sh up ros2_zenoh_router --detach
./helpers/picoros_poc/run_host_poc.sh
```

Exit gate: typed discovery and CDR decoding pass against the exact production
router image. This gate currently passes.

### Phase 2: spare-board transport and two-core proof

Continue using the spare KAmod for transport/reconnect validation here on
the WSL development host; do not flash any production board yet. Record
single-core memory/timing first, then build a **separate two-core test UF2**
and repeat typed ROS and reconnect tests. The second core is unused by the
current single-core FreeRTOS application, but SMP adds concurrency and
must be verified rather than assumed faster or safer.

1. The bounded `rmw_zenoh_pico` spike cross-built, but its `rclc` support
   initialization did not reach USB startup on hardware. Keep it as a
   comparison, not the selected firmware.
2. Use the existing Pico-ROS library and generated message types. The spare
   KAmod has passed typed ROS publication, but needs repeatable reconnect
   and long-running transport tests before porting actuator application code.
3. Build a separate experimental target; do not overwrite the micro-ROS target.
4. Generate only the required Lyrical message definitions and RIHS hashes.
5. Compile the selected layer with a pinned Zenoh-Pico revision,
   RP2040/RP2350 platform, FreeRTOS, serial, experimental USB serial, and
   unstable API enabled.
6. Connect the MCU to `serial/usb#baudrate=115200`.
7. Configure the router to listen on that board's stable by-id path.
8. Validate typed publishing and inert host-to-MCU messages for 30 minutes.
9. Here in WSL, unplug/replug USB and restart the router at least 20 times
   each, including an actual `ttyACM*` renumbering. Require recovery without
   reflashing or restarting the router for a physical hotplug; WSL USBIP
   reattachment may require manual host action. Repeat representative
   hotplug/renumbering on the real Pi before deploying firmware there.
10. Compare single- and two-core memory, timing, stack margins and reconnect
    behavior, including concurrent USB/Zenoh/application tasks.

Exit gates:

- correct ROS topic names and types;
- increasing test publisher values and decoded host-to-MCU messages match
  what was sent; sensor accuracy is tested later on each production board;
- no graph ghosts or duplicate publishers after reconnect;
- no lease timeout, crash, heap growth, or missed watchdog;
- recovery in at most 5 seconds;
- old UF2 rollback demonstrated.

### Phase 3: knives (first production candidate)

Start with the cutting motor supply physically disconnected and a known-good
firmware rollback available. Test host-to-MCU command type/QoS and local
timeout, emergency stop, watchdog, RPM limits, ramping and direction dead-time
without powering the blade. Confirm that USB loss, router restart, malformed
or stale commands cannot start a motor on reconnect. Only then perform
controlled powered tests with a physical emergency stop and verify zero
motion after timeout. A working spare-board publisher is not a safety gate
for an actuator.

### Phase 4: IMU

Before porting, choose and implement one timestamp rule:

- synchronize MCU time to host and populate `header.stamp`; or
- publish acquisition age separately and stamp on the host.

Do not silently use MCU uptime as ROS epoch time. Exit gates add monotonic
timestamps, bounded clock error, 30 Hz rate, correct frame ID, and no EKF
rejection during reconnect.

### Phase 5: wheels

Port the existing motor-control loop unchanged. Middleware callbacks may only
update preallocated command state; they must not block the 100 Hz control loop.
Keep the command timeout, watchdog, slew limiting, direction dead-time, and
stall handling local.

Exit gates add:

- reliable host-to-MCU command compatibility;
- best-effort state compatibility;
- command-to-actuation latency and jitter no worse than baseline;
- zero motion after command timeout, link loss, router restart, or malformed
  input;
- 60-minute loaded test with Nav2 traffic.

### Phase 6: INA226 / RP2350

Port after knives, IMU and wheels. RP2350 support and board-specific USB,
memory and pinout require separate verification; do not treat the KAmod
RP2040 result as evidence for this board. Validate all three existing
`Float32`/`Bool` topics for 30 minutes and repeat rollback/reconnect gates.

### Phase 7: removal

Only after all boards pass:

1. remove `micro_ros_agent` from the launch file;
2. remove `uros_bridge` from launch and package build;
3. remove Fast DDS and micro-ROS agent dependencies from the container;
4. update calibration helpers to use direct Zenoh;
5. remove bridge documentation and profile data;
6. run the full robot regression and one rollback drill; and
7. delete old firmware only after a tagged stable release.

## Explicit non-goals

- No topic rename during transport migration.
- No control-algorithm rewrite.
- No simultaneous cutover of all boards.
- No claim that TCP host success proves serial reliability.
- No removal of the working bridge before hardware evidence exists.
