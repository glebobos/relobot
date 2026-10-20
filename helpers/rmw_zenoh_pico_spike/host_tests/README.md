# Host-side serial isolation

These regression tests run inside Docker and do not access the spare RP2040.
The [current application](../firmware_picoros/README.md) and
[historical stage 2 runbook](../firmware_picoros/experiments/README.md) record
hardware outcomes separately. After prototype retirement, serial RX/link tests
compile the actual [shared USB support](../../../pico_firmware/src/usb_serial/),
not private copies under an experiment directory. Keep these regression sources
when deleting temporary build trees. The native client-gap test deliberately
passes only when the recorded event defect reproduces; it is not a health pass.

## Shared executor under serial silence

`serial_executor_silence` links the pinned MCU dependency's **actual
executor and serial framing code**, built for Linux. GNU linker wrapping
replaces only `_z_serial_read` with a pipe-backed model. One case blocks
until a helper thread releases it after 400 ms; the control polls for
5 ms and returns `SIZE_MAX` when no byte is available. A 40 ms timer runs
on the same executor, while the helper maintains an independent heartbeat.
The helper waits until the read future starts before timing the silence.
Both cases have an outer CTest timeout.

This tests the **blocking-read starvation mechanism**, not an actual
RP2040/TinyUSB implementation, transport lease, reconnection, frame
preservation or ROS discovery. The release byte is just a delimiter, not
a valid serial packet; no application payload is tested. The bounded
case is a model/control, **not a candidate firmware patch**. In particular,
a real nonblocking implementation must preserve partial COBS frames
across polls and separately handle EOF, disconnects and handshake retries.

First populate the pinned source with `build_picoros.sh`. Build and test
only in Docker, from the repository root:

```bash
docker run --rm -v "$PWD:/project" -w /project \
  --entrypoint bash relobot-picoros-poc-builder:latest -c '
    cmake -S helpers/rmw_zenoh_pico_spike/host_tests \
      -B helpers/rmw_zenoh_pico_spike/build/host_serial_executor &&
    cmake --build helpers/rmw_zenoh_pico_spike/build/host_serial_executor -j 4 &&
    ctest --test-dir helpers/rmw_zenoh_pico_spike/build/host_serial_executor -V
  '
```

Exit 0 means the explicitly modeled contrast reproduced. It must not be
counted as a passing hardware recovery test. A failed assertion or timeout
is a failed mechanism experiment; preserve its output rather than waiving it.

## Listener hotplug during initial handshake

[`serial_listener_hotplug.py`](./serial_listener_hotplug.py) uses the actual
router image and pseudo-terminals to test reopening a replaced serial
device. See the [runbook](../firmware_picoros/experiments/README.md#isolated-host-listener-hotplug-regression)
for its isolated Docker command. Its nonzero recovery result remains a
failure, independently of the executor mechanism experiment.

An opt-in [host router candidate](../host_router/README.md) now provides a
rebuilt unmodified control and a minimal pinned serial-accept patch. The
[PTY matrix](./serial_listener_matrix.py) adds silent/valid/bad-CRC handshakes,
startup without a device, distinct-target hotplug, partial-frame interruption,
a missing-path interval and repeated replacement. It does not bind host `/dev`.
The separate [stock ROS CLI smoke test](./host_router_ros_cli_smoke.sh) checks
typed pub/echo over isolated TCP loopback against each rebuilt library.
Neither test is a physical Pico or complete serial-session recovery result;
see the candidate report for exact results, provenance and remaining gates.
The [two-router serial test](./serial_router_ros_cli.py) separately verifies
complete Zenoh sessions with stock ROS CLI Float32 traffic in both directions
over relayed container-local PTYs. Its shutdown observations remain separate
from its typed-traffic result; no physical USB or firmware is involved.

## Incremental serial RX candidate

The same Docker build now includes three additional tests for the isolated
[`serial-poll` firmware candidate](../firmware_picoros/experiments/README.md#cooperative-serial-rx-candidate):

- `serial_rx_test`: the candidate's actual collector with the pinned serial
  serializer/deserializer. Exercises every two-part split and bytewise
  delivery, small/max-MTU payloads, consecutive frames, malformed COBS/CRC,
  short destinations, control/empty frames, close/disconnect reset, oversize
  resynchronization and a per-poll read-attempt budget.
- `serial_rx_executor`: the actual pinned executor with the collector,
  modeled empty USB input, a retained partial frame and nominal 5 ms reader
  scheduling. A separate 40 ms future completes without input release.
  This is not the full transport read/lease/reconnect state machine.
- `serial_poll_link_test`: the actual pinned link factory and candidate's
  link adapter with a modeled USB primitive. Verifies that only the async
  callback changes, fragmented frames arrive through that callback, serial
  close resets partial state, non-USB fallback works, and CLOSED/RECONNECTING
  future results remain READY/SUSPENDED. This guards the same-object linker
  wrapping pitfall found in the first candidate ELF; it does not execute
  the RP2040/TinyUSB bridge or validate its scheduler/USB synchronization.

All four CTests pass, including ten repeats, on 2026-10-16. The candidate
executor timer measured 43 ms in the recorded normal run. AddressSanitizer
and leak checks pass for the three new tests in a separate Docker build:

```bash
docker run --rm -v "$PWD:/project" -w /project \
  --entrypoint bash relobot-picoros-poc-builder:latest -c '
    cmake -S helpers/rmw_zenoh_pico_spike/host_tests \
      -B helpers/rmw_zenoh_pico_spike/build/host_serial_rx_asan \
      -DCMAKE_BUILD_TYPE=Debug \
      "-DCMAKE_C_FLAGS=-fsanitize=address -fno-omit-frame-pointer" &&
    cmake --build helpers/rmw_zenoh_pico_spike/build/host_serial_rx_asan -j 4 &&
    ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
      ctest --test-dir helpers/rmw_zenoh_pico_spike/build/host_serial_rx_asan \
      -R "serial_rx_|serial_poll_link_test" -V
  '
```

Neither these modeled tests nor a successful UF2 cross-build is a passing
hardware recovery test. Initial/session handshakes, blocking writes, USB
lifecycle and the host's demonstrated accept-phase hotplug bug are outside
this candidate's change scope. Preserve the hardware failure and flash only
the verified spare when proceeding to the A/B test.

## Built-in connectivity through Pico-ROS

After populating/applying the pinned dependencies with
`build_picoros.sh connectivity`, enable this additional test in a separate
native build. All commands remain inside Docker:

```bash
docker run --rm -v "$PWD:/project" -w /project \
  --entrypoint bash relobot-picoros-poc-builder:latest -c '
    cmake -S helpers/rmw_zenoh_pico_spike/host_tests \
      -B helpers/rmw_zenoh_pico_spike/build/host_connectivity \
      -DPICOROS_CONNECTIVITY_TESTS=ON &&
    cmake --build helpers/rmw_zenoh_pico_spike/build/host_connectivity -j 4 &&
    ctest --test-dir helpers/rmw_zenoh_pico_spike/build/host_connectivity -V
  '
```

`picoros_connectivity_test` compiles the **actual patched Pico-ROS source**
and the firmware's connectivity adapter against pinned Zenoh-Pico with
`CONNECTIVITY=1` and `UNSTABLE_API=1`. Two local TCP peer sessions in the
container exercise an initial history PUT, a subsequent open/close pair,
transport enumeration, listener undeclaration/no later callbacks, invalid
registration and the borrowed-session accessor before init/after close.
It does not mock event creation or the Pico-ROS accessor. The test binds
only container loopback port 17467 and runs under a 15-second CTest timeout.

The default test configuration keeps connectivity disabled and the original
four tests unchanged. Passing this fifth test does **not** demonstrate
RP2040 TinyUSB scheduling, serial reconnection, a live router, ROS command
delivery or actuator safety. Built-in connection state is intentionally
not used to replace the existing fresh-command/ack acceptance check.

On 2026-10-16 this configuration passes **5/5** CTests, ten repeats of all
five, and **5/5** AddressSanitizer/leak checks in a separate
`build/host_connectivity_asan/` Docker build. Use the same configure command
with `-DCMAKE_BUILD_TYPE=Debug` and
`-DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer"`, then run CTest
with `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1`. Evidence is in
`/tmp/picoros-connectivity-20261016T100506Z-aaVfaN/`. The integrated hardware
candidate's baseline is recorded separately in the experiment runbook;
the native tests are not evidence of USB/router recovery.

## Client reconnect notification gap

The spare-board host-only trial restores command traffic while the original
native listener still reports opened=1, closed=1, connected=0. The additional
[`picoros_connectivity_client_reconnect`](./picoros_connectivity_client_reconnect.c)
probe reproduces that discrepancy entirely inside Docker, with no USB mount,
ROS node, router process or firmware change. It compiles the actual Pico-ROS
interface/adapter against the same pinned Zenoh-Pico source as the other tests.

Unlike the earlier connectivity test, the Pico-ROS interface uses **client**
mode. A native TCP peer session listens on container loopback port 17468.
The probe confirms initial history PUT and bidirectional payloads 10/11,
drops/reopens the remote session, then confirms automatic client reconnection,
transport enumeration and fresh payloads 20/21 through the original client
session and subscriber. The original listener remains at opened=1, closed=1,
but a newly registered history listener immediately sees opened=1, closed=0.
Thus resumed traffic and an enumerable transport coexist with a missed
notification; changing the Docker tty mount cannot repair this result.

The pinned client's `_z_new_transport_client` creates/adds its initial peer
without calling `_z_connectivity_peer_connected`. `_z_client_reopen_task_fn`
uses that opening path again and restores declarations/tasks, also without
dispatching a connected event. Existing history synthesis masks this omission
at the first registration. The prior native test exercised **peer** acceptance,
whose separate opening path does dispatch the event; its pass did not cover
client auto-reconnection. The relevant source is the
[pinned client manager](https://github.com/g4sp3r/zenoh-pico/blob/0eb81272682a526b4bcf5a850c16ac74bbc6c73e/src/transport/manager.c)
and [reopen task](https://github.com/g4sp3r/zenoh-pico/blob/0eb81272682a526b4bcf5a850c16ac74bbc6c73e/src/net/session.c),
not a claim about a newer upstream release.

With `PICOROS_CONNECTIVITY_TESTS=ON`, CTest now includes a sixth test named
`picoros_connectivity_client_gap`, labeled **`known-limitation`**. Its explicit
`--expect-missing-reopen-event` mode passes only after the setup, initial
traffic, disconnect, fresh reconnected traffic, enumeration and history checks
all succeed, and the original opening count is still 1. **This is a passing
defect reproducer, not a passing health/reconnection-notification gate.** It
does not use CTest `WILL_FAIL` to accidentally accept unrelated failures.

Run the same build procedure in a fresh directory, then either mode inside
the isolated Docker container:

```bash
cmake -S helpers/rmw_zenoh_pico_spike/host_tests \
  -B helpers/rmw_zenoh_pico_spike/build/host_connectivity_client \
  -DPICOROS_CONNECTIVITY_TESTS=ON
cmake --build helpers/rmw_zenoh_pico_spike/build/host_connectivity_client -j 4
ctest --test-dir helpers/rmw_zenoh_pico_spike/build/host_connectivity_client -V

helpers/rmw_zenoh_pico_spike/build/host_connectivity_client/picoros_connectivity_client_reconnect \
  --require-reopen-event
```

The strict gate returns **1** with `expected opened=2 after reconnect,
observed 1`, after completing the same transport/traffic checks and cleanup.
The default connectivity-disabled suite remains unchanged at four tests.
No new polling loop, supervisor, synthetic callback or firmware dependency
patch is introduced. Native events remain diagnostics, not a replacement for
a fresh command/ack and local command timeout. Investigation evidence is
`/tmp/picoros-client-connectivity-20261016-W66GaC/`, with the pointer
`/tmp/picoros-client-connectivity-evidence-dir`.

Recorded on **2026-10-16**: the connectivity-enabled suite passes **6/6**,
including the explicitly labeled known-gap characterization; five additional
gap repeats all reproduce the same result. The strict event-correctness mode
fails as expected with exit **1**, not a timeout or setup failure. The ASan/
leak configuration passes **6/6**, and the unchanged connectivity-disabled
configuration passes **4/4**. No change was made to the pinned dependency or
the MCU image. These results explain the event-reporting defect, not USB
recovery performance; the subsequent single candidate router restart is
recorded in the [host experiment](../host_router/README.md#single-candidate-router-restart-2026-10-16).

## Knives integration regressions

The following isolated checks do not access USB hardware or restart services.
Run from the repository root; all compilation and execution happens in Docker.
Build the unified router image with `./start_robot.sh build ros2_zenoh_router`
before running its configuration regression.

```bash
docker run --rm --network none --entrypoint bash \
  -v "$PWD:/project:ro" relobot_pico_ware_wheels_microros:latest -lc '
    set -e
    cmake -S /project/pico_firmware/tests -B /tmp/runtime-tests
    cmake --build /tmp/runtime-tests -j2
    ctest --test-dir /tmp/runtime-tests --output-on-failure
    cmake -S /project/pico_ware_knives_picoros/tests -B /tmp/knives-tests
    cmake --build /tmp/knives-tests -j2
    ctest --test-dir /tmp/knives-tests --output-on-failure
    python3 -B /project/helpers/rmw_zenoh_pico_spike/host_tests/knives_serial_selection_test.py
  '

docker run --rm --network none --entrypoint bash \
  -v "$PWD:/project:ro" relobot-zenoh-router:local -lc '
    source /opt/ros/lyrical/setup.bash
    python3 -B /project/helpers/rmw_zenoh_pico_spike/host_tests/router_config_regression.py \
      --router-startup /usr/local/bin/start_zenoh_router.sh
  '
```

The command-state test covers zero-first arming after boot/disarm/timeout,
late-command rejection, timeout boundary, finite-value clamping, NaN/Inf
rejection without timeout refresh, zero/reverse commands and millisecond
wraparound. Control tests include the actual firmware control code with mocked
GPIO/PWM/time: timeout cuts outputs without braking, control gaps and encoder
loss latch off, braking is bounded, boot coasting cannot drive reverse PWM,
coasting telemetry continues, and malloc/stack fault hooks cut outputs before
requesting reset. A separate bench compile verifies that missing encoder pulses
do not trip the motor-only guard. These ten CTests do not prove electrical or
mechanical safety, real interrupt behavior or hardware watchdog timing.
Runtime tests verify retry only on
`PICOROS_NOT_READY`, not on unexpected initialization errors. Serial selection
tests execute the shared JSON/USB helper without importing ROS launch and verify
that the launch file delegates to it:
native devices cannot reenter the agent through raw tty aliases or `TTY_*`
overrides, missing native identity fails closed, and no device means no agent.
Invalid by-id paths and locator/config fragments are rejected before selection.
The hardware launcher selects native knives without an extra flag and validates
the [JSON registry](../../../ros2_ws/zenoh_devices.json) before `up`, without
requiring shell exports. JSON schema, duplicate keys/identities, baud rates,
disabled entries and multi-device listener/exclusion behavior are covered.
Ordinary simulation ignores hardware configuration;
explicit `--sim --picoros-knives` disables only the knives mock. Management
commands work without a USB identity. These mode selections, one-file Compose
invocation, and the router startup's TCP/serial/default-mode configuration are
covered without opening a physical device.

### UI disconnect and re-arm

Run the UI logic tests offline with the Node built-in runner:

```bash
docker run --rm --network none -v "$PWD:/project:ro" -w /project \
  node:22-bookworm-slim node --test \
  helpers/rmw_zenoh_pico_spike/host_tests/knives_ui_safety_test.mjs
```

Nine cases exercise the actual gamepad service with mocked browser/ROS objects:
zero-first startup, offline/error/close target and cruise clearing, held-button
release, deliberate slider restart, non-finite inputs, hidden-page input,
transport send failure and cleanup. The tenth case is skipped unless the
real ROSLIB module is provided. For this dependency-level check, install only
the frontend's pinned ROSLIB 2.1.0 in the disposable container, not the workspace:

```bash
docker run --rm --entrypoint sh -v "$PWD:/project:ro" -w /project \
  node:22-bookworm-slim -c '
    npm install --prefix /tmp/ui-test --ignore-scripts --no-audit --no-fund roslib@2.1.0 &&
    ROSLIB_TEST_MODULE=file:///tmp/ui-test/node_modules/roslib/dist/RosLib.js \
      node --test helpers/rmw_zenoh_pico_spike/host_tests/knives_ui_safety_test.mjs
  '
```

All ten pass on 2026-10-20, including two re-advertisement cycles with the real
`Topic` class. Its transport is still mocked: no browser, live WebSocket,
rosbridge, USB or MCU is contacted. No publish operation may queue offline;
the unrelated cmd_vel topic may queue advertisement metadata, not motion.
The native knives publisher uses `reconnect_on_close=false`, which resets its
advertised state on close; the first post-recovery zero advertises it again.

### Independent DDS transport diagnosis

[`dds_loopback_smoke.py`](./dds_loopback_smoke.py) creates two independent
FastDDS processes, not a sender listening to itself. It publishes only under
`/relobot_test/dds_coexistence/`, defaults to test domain 211, and verifies
delivery to a sensor-data QoS subscriber. No robot command topics are used.
Use an unused test domain; no USB access or running-service restart is needed:

```bash
docker run --rm --network host --ipc host --entrypoint bash \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/project/ros2_ws/fastdds_localhost.xml \
  -v "$PWD:/project:ro" ros2_ws-ros2_gazebo_sim:latest -c '
    source /opt/ros/lyrical/setup.bash &&
    timeout --kill-after=3s 25s python3 -B \
      /project/helpers/rmw_zenoh_pico_spike/host_tests/dds_loopback_smoke.py
  '
```

Nonzero exit means independent delivery failed; do not waive it as a bridge
pass. For a middleware-default isolated control, use `--network none`, remove
`--ipc host` and omit `FASTDDS_DEFAULT_PROFILES_FILE`. The
`--role sender|receiver --probe-id <shared_test_id>` options also
support two disposable containers on the same test domain. This is only
DDS transport diagnosis, not an agent/bridge/full-robot qualification.

On 2026-10-20, the WSL2 host's defaults pass three consecutive five-second
isolated-network/private-IPC trials (50, 49, 49 received of 50 sent) and two
isolated-network/host-IPC trials (49, 50 of 50). Host networking fails with
either IPC mode, so shared IPC alone does not explain the failure. Explicit
loopback/unicast XML has individual passing runs, but repeated trials fail.
No experimental XML, SHM/UDP buffer tuning or host firewall change is retained.
The host-network mechanism and real Pi/agent/bridge coexistence remain open;
this is not a diagnosis of serial throughput, MCU stacks or router crashes.

The router regression reads the actual Compose override, substitutes only its
test device with a container-local PTY, and compares the real router's full debug
configuration against the installed ROS baseline. Only listen endpoints may
differ. ROS timeouts/session limits, lease, TX queue sizes and SHM policy are
also checked explicitly. Forced process cleanup is reported separately; this
is not a traffic, graceful-shutdown or physical recovery qualification.

The Gazebo launch test evaluates the real ROS launch condition without starting
Gazebo or ROS nodes. Ordinary simulation retains the mock; the hybrid override
disables it:

```bash
docker run --rm --network none --entrypoint bash \
  -v "$PWD:/project:ro" ros2_ws-ros2_gazebo_sim:latest -lc '
    source /opt/ros/lyrical/setup.bash
    python3 -B /project/helpers/rmw_zenoh_pico_spike/host_tests/knives_gazebo_launch_test.py
  '
```

For the forward rollout sequence and configuration, see the
[knives runbook](../../../pico_ware_knives_picoros/README.md) and the
[real-robot preparation gates](../../../PICO_NEXT_STEPS.md).

### Physical knives bench acceptance

Only run this after explicitly approving the spare-board flash and confirming
that the installed KAmod firmware is the **no-actuator bench build**. It sends
knives RPM commands, including non-finite and out-of-range values. Never use it
with a motor-enabled controller. Start the hybrid Gazebo/router configuration
first, without other command publishers or the Gazebo knives mock.

```bash
docker compose -f ros2_ws/docker-compose.yml exec -T ros2_gazebo_sim bash -c \
  'source /opt/ros/lyrical/setup.bash; timeout --kill-after=5s 120s python3 -B - --confirm-bench --duration 30 --report /tmp/knives-hardware-acceptance.json' \
  < helpers/rmw_zenoh_pico_spike/host_tests/knives_hardware_acceptance.py
```

This checks graph ownership, 30 seconds of continuous commands, RPM telemetry
rate, local command timeout, zero-first re-arm (nonzero alone stays rejected
after expiry), negative/zero/clamped commands, NaN/Inf rejection
without timeout refresh, resource diagnostics and the publication error counter.
Clock and scan messages must also be observed so an idle simulation cannot pass
as a loaded Gazebo trial. It sends a zero target on exit and writes JSON results.
The `--confirm-bench` flag is an explicit operator assertion, not automatic proof
of firmware identity or electrical safety. It does not perform flashing,
hotplug/router recovery, measure individual command delivery rates, or qualify
the rest of the full robot stack.
