# Spare KAmod: stage 2, typed bidirectional USB

**Historical evidence after prototype retirement, 2026-10-16.** Use the
[retained application's README](../README.md) for current build/flash steps.
Old variant commands below describe earlier experiments and are no longer
supported. Their results and unresolved gates are not removed or upgraded.

This is an **inert, single-core experiment**, not a production board port.
Use only spare KAmod flash ID `0xE464C498C3380C35`. No motor pins, production
callbacks, watchdog claims, or automatic fleet migration are included.
The [handoff](../../../../wiki/picoros_zenoh_production_readiness.md) remains
the gate list; the migration plan is architectural evidence, not a command
to start replacing controllers.

## Code boundaries

- [`../main.c`](../main.c): the retained bidirectional validation application,
  moved from `bidirectional.c`; the publisher-only entry point is retired.
- [`../common/float32_codec.c`](../common/float32_codec.c): generated ROSIDL
  hash and Micro-CDR serialization/**deserialization**, including CDR
  encapsulation/endian validation. No replacement ROS wire protocol.
- [`../common/command_probe.c`](../common/command_probe.c): hardware-free
  sequence and timeout state; caller supplies time and synchronization.
- [`../tests/`](../tests/): native tests run inside Docker, independently of
  the Pico SDK. Keep transport details outside future board applications;
  do not copy experimental sequence semantics into motor control.
- [`transport_probe.c`](../diagnostics/transport_probe.c): pinned dependency
  instrumentation through linker wrappers, separate from command logic.
  The [host mechanism experiment](../../host_tests/README.md) uses the
  actual executor/framing code with a modeled serial read, not hardware.
- [`../common/picoros_connectivity.c`](../common/picoros_connectivity.c):
  opt-in adapter to the existing Zenoh transport-events listener, with
  history enabled. No new heartbeat, connection timer or reconnect loop.
- [Shared firmware foundation](../../../../pico_firmware/README.md): common
  SDK/FreeRTOS/USB/Zenoh build rules, startup-only Pico-ROS initialization,
  and the separate KAmod profile/pins/LED driver. Application ROS entities,
  codecs, state and task creation stay in the retained application. The used
  USB polling wrappers now belong to the shared platform. The cleaned build's
  UF2 matches the hardware-tested shared-foundation image byte for byte.

The network task may block during a serial reconnect. A separate, higher
priority probe task checks timeout every 10 ms without calling Zenoh.
Only that task writes the WS2812 after the scheduler starts. The ROS
callback and probe/publisher snapshots use FreeRTOS critical sections.
Publish errors are counted and retried, not turned into a permanent
`fail(6)` loop. Startup/codec failures remain fatal. This design is still
single-core: no SMP validation or measured timing bound is claimed.

## Protocol under test

All topics below use generated `std_msgs/msg/Float32`, RIHS
`7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2`.
The node is `/picoros_usb_bidirectional`; topic prefix is
`/picoros_usb_spike/` to preserve the existing serial test namespace.

| Suffix | Direction | Meaning |
| --- | --- | --- |
| `command` | Host -> Pico | Positive integer sequence, 1..16,777,215 |
| `ack` | Pico -> host | Last accepted sequence; 0 before first acceptance |
| `active` | Pico -> host | 1 only within 1,000 ms of a newly accepted sequence |
| `value` | Pico -> host | MCU uptime in seconds; resets after loss of MCU power/reset |
| `rejected` | Pico -> host | Duplicate or older sequence count |
| `invalid` | Pico -> host | Malformed CDR or invalid numeric command count |
| `timeouts` | Pico -> host | Active -> inactive timeout transitions |
| `publish_errors` | Pico -> host | Failed Pico-ROS publish calls (not a delivery metric) |

Telemetry is attempted every 100 ms; the safety-state timer does not depend
on its success. Green means recent **accepted input**, amber means inactive,
blue means initializing; red blink groups 1..6 identify codec/interface/
node/publisher/subscriber/encode failures respectively. LED state is not
proof of a live router or delivery. Repeated ack values are not new command
acknowledgements: prove a roundtrip by sending a strictly larger sequence.

Duplicates and older sequences never refresh the timeout or reactivate
the probe. A new larger sequence can reactivate it after timeout. The last
sequence survives router-only loss, but **not MCU reset**. Float32 carries
no timestamp or session nonce: an unseen delayed larger sequence is not
detectably stale. Replay protection across reboot, command-age bounds,
explicit re-arming, and no automatic actuator restart remain **open gates**
requiring a richer typed command envelope before a real motor port.
Counters exported as Float32 lose integer precision beyond 2^24; use this
only for bounded experiments, not permanent diagnostics.

Pico-ROS's pinned graph QoS encoding is unchanged. The CLI commands request
reliable/volatile/depth 1 for commands and reliable/volatile for telemetry.
Record actual `topic info -v` output: successful delivery alone does not
establish queue depth, retransmission behavior or a custom MCU QoS policy.

## Build and inspect (host orchestrates Docker only)

From the repository root, with the existing generated library/kernel:

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh bidirectional
docker run --rm \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/firmware_picoros:/src:ro" \
  relobot-picoros-poc-builder:latest bash -lc \
  'cmake -S /src/tests -B /tmp/probe-tests -DCMAKE_BUILD_TYPE=Debug &&
   cmake --build /tmp/probe-tests &&
   ctest --test-dir /tmp/probe-tests --output-on-failure'
```

Do not regenerate `build/` with `build.sh` unless prerequisites are missing.
The no-argument build still selects the publisher-only target. Each UF2
has a distinct name in `build/firmware_picoros_sdk220/`.

### Task-liveness diagnostic variant

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh diagnostics
```

This builds **`picoros_usb_diagnostics.uf2`**, reusing the same inert
bidirectional application, node/topics, command guard and generated CDR
codec. Only this target enables `PICOROS_DIAGNOSTICS`; it does not modify
USB lifecycle, serial handshake, transport lease, task priorities or
publish rate. Flash it using the identification procedure below, choosing
this UF2 explicitly instead of the bidirectional image.

The existing probe task now turns the WS2812 off for nominally 100 ms
each second. Observe **repeated blinking**, not a static color:

- Amber/green blinking: the probe executes and the application publish
  loop has advanced within the last nominal two seconds. This does not
  prove that the router received anything.
- Magenta blinking after initialization: the probe executes, but the
  application loop has not advanced for at least nominally two seconds;
  a blocked publish call or scheduling problem needs further isolation.
- Blue blinking before initialization: the probe executes while the
  application is still initializing. Existing red failure-count patterns
  take precedence over the heartbeat.
- Frozen light or darkness: there is no observed probe heartbeat. A
  stalled/starved task, scheduler failure, reset or power loss are not
  distinguished by that observation alone.

The probe remains priority 3, below Zenoh's default priority 16. This is
diagnostic evidence of actual task execution when blinking is observed,
not a watchdog, independent safety channel or hard timing guarantee.
The diagnostic image must pass the normal CLI check before deliberately
restarting the router. Record LED behavior alongside router logs; do not
replace the typed ROS check with an LED observation.

### Executor/USB-read probe variant

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh transport-probe
```

This builds **`picoros_usb_transport_probe.uf2`**, a separate target using
the same inert application, generated types, topics and command guard.
Only this target links wrappers around `_z_executor_spin` and
`_z_usb_uart_getc`. They record entry/return times and FreeRTOS task
identity under critical sections; they do not inject serial bytes, change
read timeouts, reschedule futures, or reset/reopen USB. The instrumentation
adds overhead; do not use it for timing acceptance or production control.
It relies on internal APIs of the pinned Zenoh-Pico revision, not a
portable board application API. Existing UF2 rollback artifacts are intact.

It retains the task heartbeat, startup/fatal patterns, and magenta
application-stall indication of `diagnostics`, adding these colors while
the application loop still advances:

- **Cyan blinking:** no observed executor entry/return progress for
  nominally 65 seconds, and that same FreeRTOS task entered one USB
  get-character call at least nominally 65 seconds ago with no recorded
  return. These are software markers, not a stack sample; cancellation
  or task deletion can also leave an unmatched entry marker. This
  tests the blocking-read hypothesis directly on the MCU; it is not
  itself a recovery fix or a measured scheduler deadline.
- **White blinking:** executor progress is unobserved, or stale for
  nominally 65 seconds without the matching long USB read. This does not
  identify the blocking operation or prove that the task is dead.
- **Amber/green blinking:** application progress with neither of those
  transport indicators. This still does not prove successful publication
  or delivery; even failing publishes can leave the loop advancing.

The 65-second threshold is deliberately longer than this experiment's
configured 60-second lease, not a measured keepalive/recovery bound.
Before initialization the existing blue pattern remains; application
stall magenta and fatal red override the transport colors.

**Flashed; functional baseline passed, but no typed recovery observed after the first router-only restart.**
ELF text: 144,396 B; BSS:
137,944 B. UF2 SHA-256:
`4c4d3852784de4584bec4df7e3a0084d12886c8411dda1f0952925bbd7781e09`.
The cross-build passes `-Wall -Wextra -Werror`; disassembly confirms calls
to both wrappers. A successful link is not evidence of an MCU recovery.

The spare's flash ID, UF2 hash and flash contents were verified on
2026-10-16 at 09:08 UTC. The full CLI baseline with sequence **401** passed
at **09:09:19 UTC**; the user confirmed orange blinking once per second.
The first router-only restart did not recover ack traffic in the bounded
observations through **09:14:23 UTC**, over 80 seconds after the new router
process started. The user subsequently reported **light blue/turquoise
blinking approximately once per second**, consistent with cyan. This supports
the shared-executor USB-read mechanism through software entry/return markers;
it is not a captured stack sample or a recovery pass. That failure remains
preserved while the separate polling candidate below is built/tested.

### Cooperative serial RX candidate

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh serial-poll
```

This opt-in target is **`picoros_usb_serial_poll`**, built separately under
`build/firmware_picoros_usb_poll_sdk220/`; it is **built, not flashed**.
Existing rollback UF2 files and their hashes remain unchanged. The candidate
retains the inert application, generated ROS types, sequence guard, local
timeout and transport-probe LED indication. It is not a production port.

- [`serial_rx.c`](../../../../pico_firmware/src/usb_serial/serial_rx.c) keeps a partial COBS frame across empty
  polls, uses fixed encoded/decoded buffers and the pinned Zenoh serial
  codec/CRC, and bounds each poll to at most 1,516 byte-read attempts.
  Oversize input is discarded through its delimiter before resynchronizing.
  Malformed COBS/CRC/length, control headers and empty data frames are not
  delivered as application payload. Disconnect and close clear partial state.
- [`serial_poll_link.c`](../../../../pico_firmware/src/usb_serial/serial_poll_link.c) wraps the serial-link factory
  and replaces **only the asynchronous socket-read callback**. Synchronous
  initial/session-handshake reads, writes and link lifecycle remain the
  dependency's implementation. There is one collector for this experiment's
  single USB link, owned by the existing executor; no multi-link/SMP claim.
- [`serial_poll_rp2040.c`](../../../../pico_firmware/src/usb_serial/serial_poll_rp2040.c) is the thin USB bridge.
  The guarded [dependency patch](../../patches/zenoh-pico-usb-poll.patch)
  exposes a try-get-character primitive using the existing USB mutex with
  **try-lock**, distinguishing empty/busy from disconnected without sleeping
  for another byte. Its code is enabled only by `PICOROS_USB_POLL` in the
  separate candidate build; default builds explicitly disable the option.
- A second wrapper changes a read future's RUNNING result to a nominal
  **5 ms scheduled wake-up**, preserving READY/SUSPENDED/already-SLEEPING
  results. This is necessary because a permanently runnable nonblocking
  reader at priority 16 could otherwise starve the lower-priority app/probe
  tasks. It is an explicit polling policy, not a lease/read-timeout change
  or a measured latency/CPU guarantee. Delivery rate/headroom need hardware
  measurement; no event-driven wake-up or throughput acceptance is claimed.
- At the pinned async-client call site, `SIZE_MAX` means no delivered frame
  on this poll; both empty and disconnected results permit executor progress.
  Lease/reconnect must still handle connection loss. **Synchronous handshake,
  USB deinit/reinit, blocking TX, host initial-handshake hotplug and WSL USBIP
  attachment behavior remain unchanged** and can still prevent recovery.

Docker validation: native CTest **4/4**, then ten repeats of all four tests;
AddressSanitizer/leak checks **3/3** for the collector, executor integration
and link-factory integration. Tests cover every two-part split and bytewise
delivery of small/maximum-MTU frames, partial-frame retention, consecutive
frames, disconnect/reset, corruption, control headers and bounded work.
The actual pinned executor's 40 ms timer fired at **43 ms** while the new
collector retained a partial frame; this is a Linux mechanism test, not an
RP2040 lease measurement. The factory integration test uses the actual
pinned factory and candidate adapter with a **modeled USB primitive**, not
TinyUSB or hardware. The cross-build passes `-Wall -Wextra -Werror`; the final
ELF retains the collector/USB helper and actual factory/close/probe wrappers.
Separate Docker configuration confirms that the default build excludes the
candidate target and USB-poll definition; command-guard CTest still passes
**1/1**. The first isolation configure used a read-only checkout, which the
pinned dependency's generated umbrella-header step cannot use; that failed
log is retained alongside the corrected writable-mount validation.

The initial attempt to wrap the async callback itself did not intercept
its same-translation-unit pointer assignment; ELF inspection caught this
before flashing. Wrapping the externally called factory fixes that wiring;
the new native integration test guards it. The dependency patch was also
moved outside existing patch contexts so their idempotence checks still pass.
Initial audit/build logs are retained, not counted as hardware evidence.

Final ELF text **145,060 B**, BSS **140,984 B**; these are link sizes, not
free-heap/stack measurements. UF2 SHA-256:
`76a10cdfe41c69c28b4ea28e2ebeede33c8d70c0c36e8e159545272b2e4ccee9`.
Evidence is under the transport-probe restart directory's
`serial-poll-candidate/` (`native-tests-02.log`, `asan-tests-01.log`,
`firmware-build-03.log`, `final-artifacts-and-link-audit.txt`).

For a polling-only A/B: physical BOOTSEL, verify spare flash ID **again**, explicitly load and
verify **this candidate from its separate directory**, then a clean Docker
ROS CLI baseline with sequence **501**. Confirm the physical baseline LED
before one router-only restart. Observe ack/uptime passively beyond the
nominal lease/probe threshold without new commands or USB/MCU resets. Only
after passive traffic returns should a fresh sequence **502** prove a new
roundtrip/timeout/rejection check. Any manual USBIP attachment or reset must
be recorded separately, not counted as automatic router-only recovery.

### Built-in Zenoh connectivity candidate

Use `build_picoros.sh connectivity` for the next experiment. It builds
**`picoros_usb_connectivity`** in the separate
`build/firmware_picoros_connectivity_sdk220/` directory, combining the
same USB polling candidate with Zenoh's existing connectivity API.
The polling-only image remains a separate rollback/comparison artifact.

```bash
bash helpers/rmw_zenoh_pico_spike/build_picoros.sh connectivity
```

`Z_FEATURE_CONNECTIVITY=1` and the already-required unstable API are enabled
only in this build configuration. The default configurations explicitly
keep connectivity disabled. Existing `AUTO_RECONNECT=1`, liveliness,
60-second lease and lease-expire factor are unchanged. The dependency
[`picoros-session-access.patch`](../../patches/picoros-session-access.patch)
adds a guarded, borrowed `picoros_interface_session()` accessor so the
adapter can use the public Zenoh API without replacing Pico-ROS lifecycle
or reading its private session storage. The accessor returns NULL before
initialization and after closure. The loan must not outlive the interface;
do not close the interface concurrently with listener registration.

The adapter registers `z_declare_transport_events_listener()` with
`history=true`: an already-connected transport is reported immediately,
then subsequent PUT/DELETE events are forwarded. The caller supplies an
uninitialized/dropped listener and owns its lifetime; undeclare it before
explicitly closing the interface. Firmware retains its listener for its
whole runtime. The callback only records state/counters in the existing
FreeRTOS critical section; it does not publish, reconnect or reset USB.
Listener initialization failure is startup error group **7**.

Three additional `std_msgs/msg/Float32` topics use the existing 100 ms
telemetry loop and `/picoros_usb_spike/` prefix:

| Suffix | Meaning |
| --- | --- |
| `transport_connected` | Last built-in event was PUT (1) or DELETE (0) |
| `transport_opened` | PUT events, including the initial history snapshot |
| `transport_closed` | DELETE events |

This state assumes the experiment's one-router **client** session. It is
not an aggregate health verdict, data freshness guarantee or command ack.
Counters reset on MCU reset and have the same Float32 precision limit as
the other experimental counters. **A disconnected Pico cannot deliver
its disconnected status over that connection.** Read the counters after
recovery, not as an out-of-band outage monitor. A stuck executor can also
prevent detection/event delivery; built-in events do not fix blocking RX,
handshake, TX or USB lifecycle. Existing LED/probe code stays diagnostic,
not a new production heartbeat service.

Hardware procedure: verify spare identity and this image's hash, flash and
verify this image, then run the unchanged `check_bidirectional.sh 501`
baseline via Docker ROS CLI. Record connected=1, opened=1, closed=0 for a
clean initial session. Observe counters/uptime/ack during **one** router-only
restart, without sending commands or resetting/replugging Pico. After
passive traffic returns, expect opened>=2, closed>=1, connected=1, unchanged
ack and non-reset uptime, then use `check_bidirectional.sh 502` for a fresh
roundtrip/timeout/rejection check. These expectations are acceptance
criteria, not a claim of success; the recorded RP2040 outcome is below.
On failure preserve the
evidence instead of starting a blind matrix. Manual WSL USBIP attachment
must be recorded separately and cannot count as router-only recovery.

The [native connectivity test](../../host_tests/README.md#built-in-connectivity-through-pico-ros)
uses actual Pico-ROS and Zenoh APIs with isolated loopback TCP peers. It
tests the adapter, not the RP2040 USB transport or router-only recovery.

Software validation on 2026-10-16: Docker CTest **5/5**, ten repeats of all
five tests, ASan/leak checks **5/5**, and command guard **1/1** pass. A
second invocation of the build script confirms dependency-patch idempotence.
ELF inspection retains the actual observer, session accessor, native
listener registration and USB polling symbols. Default configuration
excludes both opt-in targets, the connectivity feature and the session
accessor macro; all five pre-existing rollback UF2 hashes are unchanged.
ELF text is **152,316 B**, BSS **141,012 B**, not measured free heap/stack.
UF2 SHA-256:
`d8869f64b36b1296f9aa2d03418a903d52174dc05520aaece9089b42932c8010`.
Logs are in `/tmp/picoros-connectivity-20261016T100506Z-aaVfaN/`.

The physical spare was already in BOOTSEL when observed at **10:07:22
UTC** (bus 1/address 14); the earlier failed application state therefore
no longer existed at this observation. Flash ID and 16 MiB capacity were
checked again before loading. This exact UF2 was flashed/verified at
**10:09:01–10:09:04 UTC** and rebooted into the application. At **10:09:34
UTC**, USB was bus 1/address 15, `2e8a:000a`, serial
`E464C498C3380C35`, by-id -> `ttyACM0`. No manual USBIP attachment was
issued during this run. The full Docker CLI baseline with sequence **501**
passed at **10:10:13 UTC**. Native-event telemetry then showed connected=1,
opened=1, closed=0; uptime was **71.848 s** at **10:10:15 UTC** and ack=501.
These are initialization/functional results, not automatic-recovery proof.

#### First connectivity-image router-only restart: FAIL

Requested at **10:11:00 UTC**. The old router required SIGKILL after the
five-second SIGTERM grace (exit **137**); this is not verified graceful
shutdown. The new router started at **10:11:05.226 UTC**. A separate,
unchanged ROS CLI observer container received ack=501 before the restart
and was left running across it. The in-router filtered
`transport_closed>=1` observer received no sample in its 100-second window,
ending with bounded forced shutdown at **10:12:48 UTC** (exit **137**).
No new command **502**, MCU reset, reflash or manual USBIP operation was
issued after this restart; do not count the cycle as recovered.
The independent ack observer ended at **10:13:20 UTC** (its 140-second
timeout plus forced-stop grace, exit **137**), with no ack after the new
router started; its last ack=501 was at **10:11:05.100 UTC**, just before
that start. Its logs/state were saved and only that observer container was
removed. The router/Pico failed state remains available for inspection.

During the observation USB re-enumerated from address **15** to **16**;
the same application serial `E464C498C3380C35` and `2e8a:000a` resolved
through by-id to **`ttyACM1`**, replacing baseline `ttyACM0`. At **10:12:16
UTC**, the actual router child held
`/proc/41/fd/21 -> /hostdev/ttyACM0 (deleted)` while its current by-id
target was `ttyACM1`. Logs showed the new serial listener entering buffer
cleanup/initial acceptance, without a completed serial transport. This
is a demonstrated stale host descriptor, consistent with the separately
reproduced accept-phase hotplug regression. The observation alone does
not establish why USB re-enumerated or prove the MCU did not reset;
post-outage uptime/counters were not delivered. No disconnected-state
telemetry can be inferred from the lack of samples.
At **10:14:15 UTC**, the same router start time and deleted serial
descriptor were still present, with by-id still pointing to `ttyACM1`.

Evidence is in the `router-only-01/` subdirectory, including both ROS CLI
observers, router start/stop state, serial descriptor and USB snapshots.
The next investigation is **host serial listener reopening during initial
handshake**, preferably checking upstream fixes/configuration against the
pinned router before another local patch. Fixed device names or mount
propagation do not repair an already-open deleted descriptor. Keep this
failed state for inspection; do not start a blind 20-cycle matrix or claim
that the connectivity API fixes recovery.

### Flash only after verifying the board

Connect the spare in BOOTSEL mode and attach its BOOTSEL USB device to WSL.
No other Pico should be attached during flashing.

```bash
lsusb -d 2e8a:0003
docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb \
  relobot_pico_ware_wheels_microros:latest picotool info -a
# STOP unless flash ID is exactly 0xE464C498C3380C35.
docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/build/firmware_picoros_sdk220:/project:ro" \
  -w /project relobot_pico_ware_wheels_microros:latest \
  picotool load -f -x picoros_usb_bidirectional.uf2
```

Reattach the application CDC device to WSL if necessary. For rollback,
repeat BOOTSEL identification and load **`picoros_usb_spike.uf2`** instead.

## One test router and ROS CLI

Check TCP 7447 is unused first. Do not stop an unrelated robot router.
The existing service uses `/dev:/hostdev:ro` plus the ACM cgroup rule;
do not substitute a fixed `--device=/dev/ttyACM0` mapping.

```bash
sed 's/REPLACE_WITH_BOARD_SYMLINK/usb-Raspberry_Pi_Pico_E464C498C3380C35-if00/' \
  helpers/picoros_poc/router-serial.example.json5 > /tmp/picoros-bidirectional-router.json5
ZENOH_ROUTER_CONFIG_URI=/tmp/picoros-bidirectional-router.json5 \
  docker compose -f ros2_ws/docker-compose.yml up -d --no-deps ros2_zenoh_router

docker compose -f ros2_ws/docker-compose.yml exec -T ros2_zenoh_router bash -c \
  'source /opt/ros/lyrical/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp &&
   ros2 topic info -v /picoros_usb_spike/command --no-daemon &&
   ros2 topic pub --once --max-wait-time-secs 10 \
     --qos-reliability reliable --qos-durability volatile --qos-depth 1 \
     /picoros_usb_spike/command std_msgs/msg/Float32 "{data: 101.0}" &&
   timeout 15 ros2 topic echo /picoros_usb_spike/ack std_msgs/msg/Float32 \
     --once --filter "m.data == 101.0" --qos-reliability reliable'
```

Expect one MCU subscriber and a decoded ack of 101. Then use a **larger**
sequence for the CLI-only automated assertions (no custom ROS test node):

```bash
set -o pipefail
bash helpers/rmw_zenoh_pico_spike/check_bidirectional.sh 102 \
  2>&1 | tee /tmp/picoros-bidirectional-check-102.log
```

The checker needs exclusive command ownership: no other publisher should
send commands during it. It checks a fresh roundtrip, active state,
timeout, duplicate/older rejection and invalid zero rejection. A timeout or
nonzero exit is failure, not an excuse to silently reset the MCU. CLI
process startup is too slow for latency measurement; this is functional
evidence only. The checker changes the sequence/state; it is not a passive
monitor. Record failures too.

## Reconnect matrix, after first hardware roundtrip

Run 20 cycles **each** of router restart, physical unplug/replug and Pico
reset on WSL. The human operates USB/BOOTSEL/WSL attachment. Keep every
failed cycle in the denominator. Use increasing sequences across router
restart; after reset or unplug with power loss expect the sequence/uptime
to reset, and explicitly record this limitation.

For each cycle record: action, UTC timestamp, old/new by-id target,
enumeration timestamp, time to first live sample and first new-command ack,
publisher/subscriber counts/type/hash, duplicate behavior, timeout result,
and whether WSL attachment or an MCU reset was required. Keep CLI evidence
and `docker compose ... logs --since=2m ros2_zenoh_router`. A final passing
echo is not a precise enumeration-to-data latency measurement: leave that
field unmeasured unless an observer was running before enumeration.

Use a second harmless ACM device to test actual minor renumbering; if none
is available, mark that case **not tested**, not passed. The current 60 s
transport lease is unchanged and may fail the <=5 s recovery goal. Do not
reduce it just to hide a failed first measurement. Existing read/lease task
status and successful publish calls are not peer-liveness evidence.

After the bidirectional/reconnect gates, add measured heap/stack and
latency instrumentation, then a separate two-core build. Do not enable SMP
or begin knives/IMU/wheels migration during this stage. Stop only the test
router when finished:

```bash
docker compose -f ros2_ws/docker-compose.yml stop ros2_zenoh_router
```

## Evidence, 2026-10-16

### Opt-in host serial tracing

Keep production Compose and the firmware unchanged while investigating a
failed router restart. The diagnostic overlay enables serial handshake and
transport logs on **only** the test router:

```bash
ZENOH_ROUTER_CONFIG_URI=/tmp/picoros-bidirectional-router.json5 \
  docker compose -f ros2_ws/docker-compose.yml \
    -f helpers/rmw_zenoh_pico_spike/router-trace.compose.yml \
    up -d --no-deps ros2_zenoh_router
docker logs --timestamps -f ros2_zenoh_router
```

Applying the overlay recreates the router; count that as a loss/reconnect
event, not passive observation. Serial traces include payload bytes and can
affect scheduling; do not use them to claim latency limits. The overlay
does not change mounts, propagation, device permissions or protocol timing.
Return to normal logging by running the same `up -d --no-deps` command
with only the main Compose file and the same config URI.
The CLI checkers set their own `RUST_LOG=warn` so inherited router trace
settings do not contaminate numeric samples with informational log lines.

For a syscall capture, attach `strace` from a separate diagnostic container
with `SYS_PTRACE` and a shared PID namespace. Do not install diagnostic
packages in the router or open its serial device with another reader.
Capture UTC times, host/container by-id targets, `/proc/<router-pid>/fd`,
router logs, and bounded ROS CLI results together. Compare EOF, close,
reopen and handshake events before attributing a failure to bind mounts.

### Isolated host listener hotplug regression

The [PTY regression](../../host_tests/serial_listener_hotplug.py) runs the
actual router binary in a separate, network-isolated container. It opens
no physical USB device and creates no custom ROS test node. While the
serial listener is waiting for its **initial handshake**, the test removes
one pseudo-terminal and points its stable symlink at a replacement.
It succeeds only if the router opens the replacement within five seconds;
exit 1 records a failed recovery, not an expected-pass test waiver.

```bash
evidence=/tmp/picoros-serial-accept-$(date -u +%Y%m%dT%H%M%SZ)
mkdir -p "$evidence"
docker run --rm --network none --entrypoint bash \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/host_tests:/experiment:ro" \
  -v "$evidence:/evidence" ros2_ws-ros2_zenoh_router:latest -c \
  'source /opt/ros/lyrical/setup.bash &&
   python3 /experiment/serial_listener_hotplug.py --evidence-dir /evidence'
```

Use a fresh evidence directory and record the image ID. The PTY case
isolates host-side descriptor replacement, not RP2040 USB behavior or a
working Zenoh/ROS roundtrip. In host revision `2687c51`, the inner
`accept().await` error loop reuses the same port instead of returning to
the outer open/retry loop. A replacement by-id target alone cannot repair
that retained descriptor. Test an isolated upstream fix against this
regression before attempting to replace the robot's router dependency.

The subsequent isolated [host candidate](../../host_router/README.md), dated
2026-10-16, now passes an initial **8/8** PTY matrix, including ten distinct
replacements, followed by ten complete matrix repeats: **88 passing cases,
143 distinct-target replacements**. Both the installed image and a rebuilt
unmodified control fail
the four hotplug cases and pass the four non-hotplug cases. All three pass
stock ROS CLI Float32 pub/echo over isolated TCP loopback and, separately, typed
traffic in both directions over a complete two-router virtual serial session;
generated C headers,
export names and the unchanged router executable are audited separately.
This isolated result is a local opt-in patch, not a physical USB test.
Full serial-session shutdown exceeds the three-second SIGTERM grace on all
three listening-router images, so that gate remains open. At that point the
live router, spare firmware and failed USB state were preserved; command
502 had not been sent.

After separate approval, the **single host-only candidate replacement** on
2026-10-16 starts at **10:51:43.201 UTC** and restores telemetry without
resetting, reflashing or replugging the spare. Its first uptime sample is
2559.660888671875 s and ack remains 501; the router holds the current
`ttyACM1` descriptor, not the deleted `ttyACM0`. The unchanged full functional
check passes at **10:52:39 UTC**, sequence **502**, including active/timeout,
duplicate/older and invalid-zero checks. USB remains at bus 1/address 16.
The candidate remains the experimental running router; production Compose
and the original image tag are unchanged.

**Native-event recovery reporting remains a blocker:** resumed traffic and
the successful command occur while `transport_connected=0`, opened=1,
closed=1. Do not treat that flag alone as recovered health. This replacement
does not establish patch-specific physical hotplug recovery, another
router-restart pass or a recovery deadline; starting a fresh host process
also removes the old descriptor. See the [host trial record](../../host_router/README.md#single-spare-board-host-only-trial-2026-10-16)
for exact image IDs, evidence and opt-in rollback. The subsequent offline
investigation reproduces the event gap in native client mode: the pinned
client creation/reopen path omits connected-event dispatch, while initial
history and peer acceptance explain why the earlier native test passed.
See the [client gap probe](../../host_tests/README.md#client-reconnect-notification-gap).
No firmware or dependency patch is added for this event-reporting defect.

The next approved **single candidate router-only restart** also passes.
The baseline check succeeds at **11:05:15 UTC**, sequence **503**. Stopping
the active candidate at **11:06:18 UTC** requires SIGKILL after five seconds
(exit 137); the same container starts at **11:06:23.743 UTC**. USB automatically
renumbers `ttyACM1` to `ttyACM0`, address 16 to 17, without manual replug,
USBIP attach, MCU reset or another router restart. Listener retries survive
the unavailable/missing path; the same process accepts a serial transport
at **11:06:39.724 UTC**. Delivered uptime continues from 3434.8740234375 to
3455.554931640625 s, and retained ack is 503. The full post-restart check
passes at **11:07:06 UTC**, sequence **504**. The current serial FD matches
the new tty inode/minor. Recovered diagnostics are still connected=0,
opened=1, closed=2, independently corroborating the client notification gap.

This is one candidate router-restart/automatic-renumber recovery, **not** a
physical unplug/replug cycle, passing five-second deadline, graceful-shutdown
pass or completed 20-cycle matrix. The same experimental candidate remains
running; production Compose and the MCU image are unchanged. The
[restart record](../../host_router/README.md#single-candidate-router-restart-2026-10-16)
contains the exact timing, evidence and limitations. The separately coordinated
physical USB trial follows; do not reset/replug blindly.

The subsequent **single physical USB power-cycle trial** passes on the same
candidate router, without a router restart, firmware change or agent-issued
USBIP attach. The full baseline passes at **11:37:15 UTC**, sequence **505**.
The user unplugs only the spare; the host and container lose its by-id target,
and the router closes its old FD and retries. The same process accepts the
returning board at **11:43:58.831 UTC**. Fresh telemetry arrives that second,
uptime **2.062000036239624 s**, ack **0**: USB power loss resets MCU state.
The device keeps `ttyACM0` and minor 0 but changes inode **1870 to 1896** and
USB address **17 to 18**; the current router FD matches the new node. The
full post-replug check passes at **11:44:54 UTC**, sequence **506**. Native
diagnostics are connected=1, opened=1, closed=0 after the new boot, not a fix
for the known same-boot reconnect-event gap. This is one physical recovery,
not tty renumbering, a measured five-second physical deadline, cross-boot
replay protection, hardware A/B causality or a completed repeated-loss matrix.
See the [physical trial record](../../host_router/README.md#single-candidate-physical-usb-cycle-2026-10-16)
for raw evidence and observer-cleanup limitations. The same experimental
candidate remains running; production Compose and firmware are unchanged.

### Hardware results

Times below are UTC. Only the spare was flashed; production firmware and
controllers are unchanged. The actual router image ID was
`sha256:27d53bfe8edbbfd46b56e785452984f70f7b761d737b92acabf01af9a625322b`.

- Docker native guard tests: passed (acceptance, duplicate/out-of-order,
  timeout boundary/no refresh, invalid values, integer range, timer wrap).
- Docker RP2040 cross-build: passed with application `-Wall -Wextra -Werror`.
- ELF: text 143,732 B; BSS 137,920 B (not measured runtime headroom).
- Linked generated Float32 deserializer, RIHS getter and Pico-ROS subscriber;
  symbol inspection found no `rmw_init`, `rmw_uros*` or XRCE session symbols.
- Initial UF2 SHA-256:
  `e0bbc0d85e1d481780f982177d5a48192b4d2553c85e8063851729e194c096a2`.
- BOOTSEL flash ID was verified as `0xE464C498C3380C35`; the inert
  bidirectional UF2 was flashed and its application USB attached to WSL.
- 07:49:08-07:49:34: full CLI functional check passed with sequence 102.
  Ack became 102, active was observed at 1 and then 0, duplicate 102 and
  older 101 increased rejection counts without reactivation, and zero
  increased the invalid count. This proves functional timeout behavior,
  not a measured hard real-time deadline.
- Actual command subscriber and ack publisher graph declarations showed
  one MCU entity each, the expected Float32 RIHS, RELIABLE, VOLATILE and
  KEEP_LAST depth **42**. Command CLI depth 1 does not change the MCU's
  fixed declaration. Delivery does not prove all QoS semantics.
- First router restart: requested 07:49:51, new router started 07:50:01;
  the functional check passed again at 07:50:39 with sequence 103, without
  an MCU reset requested by the agent. USB re-enumeration was observed;
  no <=5 s recovery result is claimed.
- Second router restart: requested 07:51:46, router started 07:51:57;
  typed traffic did not return. The planned 20-cycle loop was interrupted
  during this first automated cycle; **do not report a 20-cycle pass**.
  An independent echo also received no sample. The user reported amber
  LED, but a static WS2812 color is **not proof** that the timeout task is
  still executing. Firmware versus host-link stall is not yet isolated.
- Physical unplug/replug and WSL attachment at approximately 08:08 moved
  the spare from `ttyACM0` to **`ttyACM1`**. The same running container saw
  `/hostdev/serial/by-id/usb-Raspberry_Pi_Pico_E464C498C3380C35-if00 -> ../../ttyACM1`,
  while the router process still held `/hostdev/ttyACM0 (deleted)`. A
  bounded echo from 08:08:39 to 08:09:12 received no ack and exited 137
  after forced termination. This is a failed same-router hotplug recovery
  in an already-stalled session, not an isolated clean-session hotplug test.
- Router-only restart at 08:09:37 reopened `ttyACM1`; the full CLI check
  passed at 08:10:05 with sequence 201. This is recovery **with a router
  restart**, not proof of automatic renumber recovery.
- Clean-session baseline passed at 08:20:04 with sequence 202. A separate
  diagnostic container attached syscall tracing without restarting the
  router. On physical unplug at 08:28:10, the router received EOF from
  `ttyACM1` and closed that descriptor at 08:28:10.371. While the device
  was absent it retried opening the by-id path. After normal replug and
  manual WSL attachment, it reopened that path onto **`ttyACM0`** at
  08:28:22.722; both host and container observed the new target. The same
  router (started 08:09:38) passed the full CLI check at 08:29:10 with
  sequence 203. This is **one clean-session hotplug/renumber recovery
  pass**, without router restart or mount changes. Physical power loss
  resets the MCU command state; this is not replay protection across reboot.
  Enumeration-to-live-ROS latency was not measured.
- Applying the serial-log overlay requested router recreation at 08:30:51;
  the new router started at 08:31:02.738 and opened the current `ttyACM0`.
  There was no new-command transmission or MCU reset requested. The first
  30-second ack observer failed, and a second 90-second observer from
  08:32:05 to 08:33:38 also received no sample (both forced exit 137 during
  shutdown). The device and by-id target remained visible, but the serial
  handshake did not complete. Waiting beyond the 60-second lease did not
  restore traffic in this observation; the MCU-side cause remains unknown.
- The separate PTY regression reproduced a **host-side accept-phase
  hotplug failure** on the same router image: `/dev/pts/0` was replaced by
  `/dev/pts/1`, while the router retained `/dev/pts/0 (deleted)` throughout
  the five-second observation. This needs neither an RP2040 nor Docker
  device passthrough. It explains a reproducible retained-descriptor path
  during handshake, **not** why the MCU stopped completing its handshake
  after a router-only restart. No upstream fix has been applied or verified.
- A separate task-liveness diagnostic target was cross-built with
  `-Wall -Wextra -Werror`; at that point it had not been flashed or
  hardware-tested (see the later diagnostic baseline below).
  It adds heartbeat/progress indication only, not a proposed recovery fix.
  ELF text is 143,932 B and BSS is 137,924 B; UF2 SHA-256 is
  `3e6e27b0d7a8b1bed4648d913371e9ac8dcc35aecfdcc1c1ccb4c6b6a630667e`.
  Existing publisher and bidirectional UF2 hashes remain unchanged.
- At 08:39:23 the experimental router was stopped after saving the failed
  state and logs; the diagnostic tracing container was also stopped. The
  spare still contains the original bidirectional image. Next: human
  BOOTSEL/WSL attachment, verify flash ID, flash the diagnostic UF2, then
  establish a fresh typed baseline before testing router loss again.
- Software BOOTSEL attempts at 08:43:13–08:43:53 did **not** reach the
  bootloader. With the router stopped and spare serial identity verified,
  `picotool reboot -u -f --bus 1 --address 7 --ser E464C498C3380C35`
  returned exit 250: `Unable to locate reset interface on the device`.
  Live descriptors exposed only the two CDC interfaces, not the vendor
  reset interface; the built bidirectional ELF also had no reset-handler
  or CDC line-coding callback symbols. Windows' cached `Reset` label is
  therefore not evidence that this image supports software BOOTSEL.
  A first host-side CDC open failed on permissions without sending a
  request. A subsequent Docker attempt successfully set 1200 baud and
  asserted/deasserted DTR, sending **no application bytes**. Restoring
  line coding then failed with EIO. WSL logged a USB disconnect/reattach,
  changing USB address 7 to 8, but the device returned as application
  `2e8a:000a` with the same serial and `ttyACM0`, not BOOTSEL `2e8a:0003`.
  This is not proof of an MCU reset or of its cause. No flash occurred;
  diagnostic UF2 hardware validation still requires physical BOOTSEL.
  Evidence: `/tmp/picoros-software-bootsel-20261016T0845-MGgBKA/`.
  The router remains stopped.
- After the user connected physical BOOTSEL, the device appeared as
  `2e8a:0003`, USB bus 1/address 9. At 08:45:35 the flash procedure
  rechecked **flash ID `0xE464C498C3380C35`**, 16 MiB, and the diagnostic
  UF2 SHA-256 above before writing. `picotool verify` returned **OK**;
  readback metadata identified `picoros_usb_diagnostics`. Application
  reboot completed at 08:45:38. WSL subsequently exposed `2e8a:000a`,
  the same serial/by-id and `ttyACM0` at USB address 10.
- The isolated router started at 08:45:54.923 with the existing serial
  URI and trace overlay; no other robot services were started. The full
  Docker ROS CLI check ran from 08:45:55 to **08:46:22**, sequence **301**,
  and passed typed command/ack, active state, local timeout,
  duplicate/older rejection and invalid-value checks (exit 0). This is
  the diagnostic image's first functional hardware baseline, **not** a
  recovery pass or a measured timeout bound. Evidence:
  `/tmp/picoros-diagnostics-20261016T0845-OmswFc/`, including flash/verify,
  application identity, router state/logs and `baseline-301.txt`.
  The router was left running pending baseline LED observation; the user
  subsequently confirmed **orange blinking once per second**. This is
  observed probe execution/application progress before router loss, not
  evidence of probe liveness during a later failure.
- One controlled router-only restart was requested at **08:51:13 UTC**
  after fresh passive samples showed ack **301** and uptime
  `334.77801513671875` s. No MCU reset, USB operation or new command was
  requested. Docker sent SIGTERM, then SIGKILL after the configured
  five-second grace period (old process exit 137); this was **not a
  verified graceful shutdown**. The same container started its new
  process at **08:51:18.306** with the unchanged image/configuration.
- The post-restart 30-second observer, filtered for ack 301, received no
  matching sample from 08:51:18 to 08:51:51. A separate **unfiltered**
  30-second ack observer from 08:52:15 to **08:52:48** also received no
  sample, extending the observation beyond the nominal 60-second lease.
  Both CLI processes needed the bounded forced shutdown (exit 137).
  No fresh sequence 302 was sent; the full functional check was skipped
  rather than claiming recovery. This diagnostic image therefore also
  **fails the observed router-loss recovery check**; it is instrumentation,
  not a recovery fix.
- The spare remained at USB bus 1/address 10, `ttyACM0`, with the same
  serial/by-id. The router's serial descriptor pointed to the **current,
  non-deleted** `/hostdev/ttyACM0`; logs showed listener creation/buffer
  clearing but no completed serial transport acceptance. This observation
  does not implicate device renumbering or establish the MCU-side cause.
  The router was left running in the observed failure state, awaiting the
  user's **post-restart LED color and continued/frozen blinking** report;
  continued orange heartbeat was subsequently confirmed.
  Evidence is under
  `/tmp/picoros-diagnostics-20261016T0845-OmswFc/router-only-01/`, including
  both observers, lifecycle events, device/descriptor snapshots and logs.
- The user subsequently confirmed **orange blinking once per second
  continued after the failed restart**. The probe and application loop
  therefore remain active in that observed state. This does **not** show
  that puts succeed, the USB worker runs, or the Zenoh executor services
  its lease/reconnect work: the application also advances after a failed
  publish. No new application command or hardware reset was requested.
- A separate tracing container attached to the unchanged router process
  from **08:55:06 to 08:55:26 UTC**. Its serial FD 21 pointed to current
  `/hostdev/ttyACM0`; the selected read/write/poll/open/close/ioctl trace
  contained only five pending epoll waits, **no serial reads or writes**
  during that window. The tracer was removed afterward; no second serial
  reader was opened. Evidence: `router-only-01/orange-heartbeat-trace/`
  under the diagnostic evidence directory above.
- Pinned-source inspection found a concrete starvation mechanism to test:
  `_zp_start_transport_tasks` schedules read, keepalive and lease futures
  on the same runtime. `_z_executor_spin` invokes each future inline;
  `zp_start_lease_task` is now a compatibility no-op, not another RTOS
  worker. The serial read reaches `_z_usb_uart_getc`, which sleeps/retries
  indefinitely while CDC reports connected but no byte is available.
  An RTOS sleep yields the CPU to other tasks, **not** the blocked
  executor to its other futures. This is a source-supported hypothesis
  consistent with the LED/host evidence, **not yet MCU root-cause proof**.
- The [host mechanism experiment](../../host_tests/README.md) links that
  revision's actual executor/framing code while modeling just the serial
  read with a pipe. In the corrected harness, blocking RX delayed a 40 ms
  timer to **404 ms** while an independent heartbeat ran 40 times; the
  bounded-read control fired at **51 ms**, before external release.
  Docker CTest passed 1/1 and ten repeats. These are model timings, not
  RP2040 lease measurements or ten hardware recovery passes. The initial
  harness timed out by re-entering a second blocking read after its
  release; that harness error was corrected and its failed log retained.
  Evidence: `executor-model-tests.log` (harness timeout) and
  `executor-model-tests-02.log` (corrected run/repeats).
- At that point the separate executor/USB-read probe target above was
  cross-built but not yet flashed. Original publisher, bidirectional and
  task-heartbeat UF2 hashes were unchanged; the spare still ran
  `picoros_usb_diagnostics`. The subsequent physical flash is recorded below.
- A fresh passive, unfiltered ack observation at **09:03:36–09:03:54 UTC**
  still received no sample (15-second timeout plus forced shutdown,
  exit 137). The router's start time remained 08:51:18.306; the spare
  remained at USB address 10. No further router/MCU restart, flash or new
  command was performed during tracing, source inspection or host tests.
  Evidence: `orange-confirmation-*` files in the same evidence directory.

### Transport-probe flash and clean baseline, 2026-10-16

- After the user's physical BOOTSEL operation, the spare appeared at USB
  bus 1/address 11. Docker `picotool info -a` confirmed flash ID
  **`0xE464C498C3380C35`**, 16 MiB, before flashing. The previous failure
  logs and router state were saved, then only the test router was stopped.
- The guarded flash rechecked identity and the UF2 SHA-256 above. Load and
  flash verification passed at **09:08:34–09:08:37 UTC**; metadata confirmed
  **`picoros_usb_transport_probe`** before reboot into application mode.
  USBIP reattached the application automatically at bus 1/address 12;
  its serial stayed `E464C498C3380C35`, with the by-id link to `ttyACM0`.
- The unchanged test router image/configuration started at **09:08:51 UTC**.
  The Docker ROS CLI baseline ran **09:08:51–09:09:19 UTC**, exit **0**:
  fresh sequence **401**, active/ack, local timeout, duplicate/older
  rejection and invalid-zero rejection all passed. Command graph QoS
  remained RELIABLE, VOLATILE, KEEP_LAST **42**. This is functional baseline
  evidence, not recovery, latency, command-age or actuator-safety evidence.
- The user subsequently confirmed **orange blinking once per second**
  before the controlled router-only restart recorded below. This confirms
  the baseline probe indication, not a measured transport timing bound.
- Fresh raw evidence: `/tmp/picoros-transport-probe-20261016T0909-H7X1Ux/`
  (`flash.log`, application identity, router snapshots/configuration,
  `baseline-401.log`, exit/timestamp files and unchanged rollback hashes).

### Transport-probe router-only restart, 2026-10-16

- Before the restart, passive Docker ROS CLI observations still received
  ack **401**, uptime `256.96099853515625` s and `publish_errors=0`.
  Only the test router was restarted, requested at **09:12:55 UTC**.
  Docker lifecycle events record SIGTERM, then SIGKILL after five seconds
  and old-process exit **137**; this was **not verified graceful shutdown**.
  The unchanged image/configuration started at **09:13:00.746 UTC**.
- The 30-second observer filtered for ack 401 received no matching sample
  at **09:13:00–09:13:34 UTC**. A separate unfiltered 15-second ack observer
  at **09:14:05–09:14:23 UTC** also received no sample, extending observation
  beyond both the nominal 60-second lease and 65-second probe threshold.
  Both needed the configured forced shutdown and exited **137**. These
  are failed bounded observations, not measured MCU recovery deadlines.
- USB stayed at bus 1/address **12** and by-id still resolved to `ttyACM0`.
  The actual router process retained current, non-deleted
  `/proc/40/fd/21 -> /hostdev/ttyACM0`. The initial descriptor snapshot of
  PID 1 was empty because PID 1 is the ROS CLI launcher; the all-process
  snapshot identifies the child router's descriptor. Router logs showed
  serial-listener creation and buffer clearing, not a completed new serial
  transport acceptance in the recorded window. This does not establish
  the MCU's blocking operation.
- No new command **402**, MCU reset, flash, USB detach/reattach, additional
  router restart or recovery-policy change was performed. The board and
  router remain running. The user subsequently reported **light blue or
  turquoise blinking approximately once per second**, consistent with cyan;
  the report was recorded at **09:16:31 UTC** with the same router start time
  and USB address. This is the user's color observation, not a measured RGB
  value or a stack sample. By this target's marker logic it supports an
  unmatched long USB-get-character call in the same task as stale executor
  progress while app/probe tasks still execute. Task cancellation/deletion
  remains a caveat; the precise MCU cause and a recovery fix are not proven.
- Raw evidence:
  `/tmp/picoros-transport-probe-20261016T0909-H7X1Ux/router-only-01/`,
  including both observer logs/exits/timestamps, lifecycle events, USB and
  router snapshots, `router-all-process-serial-descriptors.txt` and logs.

The failed state distinguishes two problems: device-path visibility versus
closing/reopening an existing serial connection. The current `/dev` parent
bind already made the new symlink and device visible; adding bind
propagation does not retarget an already-open descriptor. Do not attribute
this failure to a stale by-id directory mount, as in the earlier experiment.
The precise reason the router retained the old handle in the earlier
stalled session remains unproven. The clean-session hotplug pass now shows
that the same router can close and reopen a genuinely renumbered device.
Trace serial handshake/read/write/reopen paths during router-only failure
next, before changing transport parameters or claiming a fix. Neither
this single pass nor a final successful echo completes the repeated-loss
matrix.

The first CLI harness attempt used `--field data` together with an
`m.data` filter, which Lyrical evaluates on the selected float. It failed
in the host checker, not the MCU. The corrected checker filters the full
message. Failed-session CLI shutdown also hung after SIGTERM; both helper
scripts now use `timeout --kill-after=3s` for a bounded failure. The router
matrix uses explicit failure chaining, rather than relying on `set -e`
inside an `if` subshell. Raw scratch logs remain under `/tmp/`:

```text
/tmp/picoros-bidirectional-check-101.log                 (host checker error)
/tmp/picoros-bidirectional-check-102.log                 (PASS)
/tmp/picoros-bidirectional-router-cycle-01.log            (first restart PASS)
/tmp/picoros-router-matrix-20261016/                     (interrupted run)
/tmp/picoros-bidirectional-hotplug-cycle-01.log           (graph absent)
/tmp/picoros-bidirectional-renumber-with-router-restart.log (PASS)
/tmp/picoros-clean-hotplug-20261016T082047Z/              (clean renumber PASS, syscall trace)
/tmp/picoros-router-trace-20261016T083011Z/               (restart stall and isolated PTY failure)
```

For a future router matrix, use the CLI-only orchestration helper, with a
first sequence above the current ack and a fresh evidence directory:

```bash
bash helpers/rmw_zenoh_pico_spike/check_router_reconnect.sh 20 1001
```

It stops at the first failed check and leaves later cycles untested; it
does not reset/reflash a board or perform WSL USB attachment. Recovery
latency, heap/stack headroom, independent task liveness during a stall,
20-cycle matrices and separate Pico-reset tests remain **open**. The next
step is investigating serial recovery, not migrating production controllers
or enabling SMP. The earlier task-heartbeat image failed its first
controlled router-only restart despite continued orange heartbeat; that
observation and passive trace are saved. The separate `transport-probe`
image is now flashed and its full functional baseline passes, with orange
heartbeat confirmed. Its first router-only restart produced no ack in
bounded observations extending over 80 seconds after the new process
started. The user's light-blue/turquoise heartbeat report supports the
blocking-RX markers. A separate `serial-poll` candidate and native/ASan tests
now pass, but that UF2 has not been flashed or hardware-validated. Next is
physical BOOTSEL for the verified spare, not another blind router matrix.
That failed state was preserved during polling-candidate development. The
later [connectivity experiment](#built-in-zenoh-connectivity-candidate)
supersedes it: this exact spare was reflashed after another identity check,
and its latest accepted command sequence is **501**. Use a larger sequence
for the next fresh command; reflashing resets it.
