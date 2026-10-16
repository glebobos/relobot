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
