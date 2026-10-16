# Pico-ROS over USB: experiment handoff and production gates

Status: **experimental, 2026-10-16**. This is a handoff for continuing the
investigation, not an instruction to migrate a production controller.
Run commands from the repository root. All ROS 2 commands and firmware builds
run **inside Docker**, not directly on the host; see [AGENTS.md](../AGENTS.md).
For architectural motivation and the board-by-board migration sequence, see
the [migration plan](./picoros_zenoh_migration_plan.md). The older
[`rmw_zenoh_pico` validation guide](./rmw_zenoh_pico_robot_validation.md)
contains procedures for a *different*, non-selected UF2: do not flash that
image while following this guide.

**Current software stage, 2026-10-16:** the
[shared firmware foundation](../pico_firmware/README.md) separates SDK/FreeRTOS,
USB/Zenoh build configuration and initial Pico-ROS setup from the KAmod board
profile/pins/LED driver and application-owned ROS entities/tasks/state. The
retained USB roundtrip application consumes it; production firmware is not ported.
Six opt-in and four default target builds, four runtime unit tests (also under
sanitizers), the existing command-state test and two invalid-board guards pass
inside Docker without device mounts before prototype retirement. The new shared
foundation is now flashed and hardware-regression-tested as recorded below.
After user-approved cleanup, one application target remains; used USB receive
support belongs to the shared platform rather than the experiment directory.
The canonical rebuilt UF2 is byte-identical to the hardware-tested image.
At the user's request, further repeated recovery, native-health, graceful
shutdown and Pi/ARM64 validation are **deferred from this stage**, not marked
passed or removed from the evidence. The known event defect remains recorded.
No new reconnect mechanism, safety policy, memory instrumentation or SMP
change is introduced. The migration plan remains unchanged.

## Single-core baseline and resource measurement, 2026-10-16

A minimal diagnostic telemetry set was added to [main.c](../helpers/rmw_zenoh_pico_spike/firmware_picoros/main.c) (`heap_free`, `heap_min_ever`, `stack_app_words`, `stack_probe_words`), without adding a new health, reconnect supervisor, or custom ROS protocol. The instrumented firmware was verified with unit tests (4/4 startup, 6/6 host regression) and flashed to the spare KAmod RP2040.

Flashed and canonical `picoros_usb_connectivity.uf2` SHA-256:
`608a255f4671b22c16566c82803ac18866ba490fd319b3f4280a65e5e2d120c9`.

### Static footprint (ELF breakdown)
- Flash (`text`): **153,168 bytes**
- RAM (`data`): **0 bytes**
- RAM (`bss`): **141,020 bytes** (includes 128 KiB = 131,072 bytes FreeRTOS heap + 9,948 bytes static state)
- Total static RAM: **141,020 bytes** (~53.4% of RP2040 264 KiB SRAM)

### Hardware regression & load baseline (300 commands @ 10 Hz, 30 s duration)
The standard regression check passed (sequence 1):
`PASS: typed roundtrip, timeout, duplicate/older sequence and invalid-value checks.`

Under sustained 10 Hz bidirectional command traffic using [benchmark_baseline.py](../helpers/rmw_zenoh_pico_spike/benchmark_baseline.py):
| Metric | Measured value | Notes |
| --- | --- | --- |
| Commands processed | 300 sent / 301 final ack | Zero rejected or dropped valid commands; all 300 accepted. |
| Roundtrip Latency (min / median / max) | 12.26 ms / 63.56 ms / 116.65 ms | Mean 63.88 ms, p95 109.87 ms, stddev 28.86 ms (bounded by 100 ms telemetry tick). |
| FreeRTOS Heap (total / min-ever free) | 131,072 B total / 17,160 B min-ever free | Peak usage: 113,912 B (86.91% of heap); safety margin: 17,160 B (13.09%). |
| App task stack (`picoros_usb`) | 32,768 B alloc / 4,820 B peak used | Headroom: 27,948 B (peak used: 14.71%). Over-allocated by ~24 KiB. |
| Probe task stack (`command_probe`) | 2,048 B alloc / 152 B peak used | Headroom: 1,896 B (peak used: 7.42%). |
| Telemetry stability | 0 publish errors | Continuity intact across full duration. |
| Command cessation timeout | 3.06 s observed | `active` dropped to 0.0 and `timeouts` incremented after commands stopped. |

## Shared foundation hardware regression, 2026-10-16

The user authorized flashing the new shared-foundation build to the spare,
one router-restart regression and retirement of obsolete prototypes. No
production controller, motor, complete robot stack or alternative device was
used. Only stock ROS CLI commands inside Docker generated application traffic.

| Check | Observed result |
| --- | --- |
| Target identity | KAmod RP2040, flash ID `0xE464C498C3380C35`, 16 MiB; BOOTSEL identity checked immediately before writing. |
| Flash | `picotool load -v` and application reboot, **12:10:41–12:10:44 UTC**; flash verification succeeds. The user supplied BOOTSEL; no arbitrary serial reset bytes or competing tty reader. |
| Initial roundtrip | Fresh boot starts with ack 0. Full command/ack, active/timeout and duplicate/older/invalid rejection pass with sequence **601** at **12:11:31 UTC**. |
| One bounded router restart | Stop requested at **12:12:10.318 UTC**, forced after five seconds, exit **137**. Same candidate container/image starts at **12:12:15.591 UTC** and accepts the MCU serial transport at **12:12:18.578 UTC**. This is not a graceful-shutdown pass. |
| Same MCU boot | Retained ack **601**; fresh uptime advances **84.39800262451172 to 95.00399780273438 s**. No reflash, physical USB action or MCU reset during router recovery. |
| Post-recovery roundtrip | Full CLI check passes with sequence **602** at **12:12:47 UTC**. |
| Native events | `connected=0`, `opened=1`, `closed=1` despite working bidirectional traffic. The known same-boot reopen-event defect remains. |
| Cleaned source layout | Rebuilt canonical UF2 matches the actually flashed image byte for byte; source relocation/prototype retirement does not require another flash of identical bytes. |
| Regression tests after relocation | Runtime **4/4**, command state **1/1**, native serial/connectivity **6/6** inside device-free Docker. The native six include the explicitly labeled known-limitation reproducer, not a passed health gate. |
| Reusable build integration | The actual wrapper rebuilds successfully. A separate compile-only consumer builds with shared defaults (USB polling ON, native connectivity OFF); unknown-board/RP2350 guards and the one-definition symbol audit pass. This extra consumer is never flashed. |

Flashed and canonical `picoros_usb_connectivity.uf2` SHA-256:
`d5c50234f849babed39d267c6e03ae4748d78f41f40467b8e7274b7b43cd1edb`.
Canonical output is
`helpers/rmw_zenoh_pico_spike/build/firmware_picoros_connectivity_sdk220/`;
the [current application README](../helpers/rmw_zenoh_pico_spike/firmware_picoros/README.md)
is the build/validation entry point. Its historical target name is retained;
the single default wrapper selection now builds that image, not the removed
publisher-only prototype.

The router remains `relobot-zenoh-serial-accept:experimental`, image
`sha256:f5362188e9d04a3bdace2e98a2b5b08adda3fcb34f1a187c4018396f1ffab2d3`.
The production image tag, Compose configuration, by-id locator, `/dev` parent
bind and cgroup rule are unchanged by this trial. Do not ordinary-Compose
recreate the candidate: its image override remains separate from production.

One observed serial accept about three seconds after router startup is not a
measured worst-case five-second recovery guarantee or a repeated-loss matrix.
This confirms the shared RP2040 foundation has no observed regression in the
tested path; it does not qualify other board profiles, RP2350, SMP, resource
headroom, motor safety or the previously deferred production gates.

User-approved retirement removes the publisher-only entry point, unselected
Pico-ROS variant targets and one-off LED/CDC/FreeRTOS probe sources. Used polling
code moves into [shared USB support](../pico_firmware/src/usb_serial/); executor
observation remains in application-owned diagnostics. Regression-test sources,
historical evidence, required patches, dependency pins and rollback firmware
remain. Temporary refactor/test build trees are removed after the canonical UF2
identity check. Raw records, source snapshot and rollback image are local under
`/tmp/picoros-foundation-hardware-20261016-gHQpcv/`, pointer
`/tmp/picoros-foundation-hardware-evidence-dir`.
Cleanup removes **20 generated build/test trees, 728.5 MiB**; prepared dependency
checkouts, generated ROS support and the canonical current firmware remain.

## Earlier hardware trials

**Earlier hardware trial, 2026-10-16 11:44:54 UTC:** one user-coordinated
[physical USB unplug/replug](../helpers/rmw_zenoh_pico_spike/host_router/README.md#single-candidate-physical-usb-cycle-2026-10-16)
passes on the spare KAmod, without a router restart, reflash, mount/configuration
change or agent-issued USBIP attach. The full stock Docker ROS CLI baseline
passes with sequence **505** at **11:37:15 UTC**. After unplug, the device
disappears from both host and container; the router closes its old serial FD
and retries the absent by-id path. At **11:43:58.831 UTC**, the same running
router accepts the returning board. Fresh telemetry arrives that second,
uptime **2.062000036239624 s**, ack **0**: this is a new MCU boot after USB
power loss, not retained state. The full post-replug command/ack, timeout and
duplicate/older/invalid rejection check passes with sequence **506** at
**11:44:54 UTC**.

USB keeps the same serial and `ttyACM0` name/minor, but its address changes
**17 to 18** and inode **1870 to 1896**; the router FD matches the new node.
The candidate container ID, start time and process PID stay unchanged.
Native diagnostics now read connected=1, opened=1, closed=0 after a fresh
boot; that does **not** fix or retest the known same-boot reconnect-event gap.
This is **one physical power-cycle recovery**, not a tty-renumber test,
20-cycle matrix, patch-specific hardware A/B proof or measured five-second
physical reconnect deadline. Production Compose, the original image tag and
the MCU firmware remain unchanged. No further hardware disturbance is issued.

**Previous candidate router-only trial, 2026-10-16 11:07:06 UTC:** one separately approved
[candidate router-only restart](../helpers/rmw_zenoh_pico_spike/host_router/README.md#single-candidate-router-restart-2026-10-16)
passes. The full ROS CLI baseline succeeds with sequence **503**, then the
same candidate container starts at **11:06:23.743 UTC** after a five-second
forced stop (exit **137**). USB automatically renumbers `ttyACM1` to `ttyACM0`,
address 16 to 17. The same router process retries the unavailable/missing
by-id path and accepts a serial transport at **11:06:39.724 UTC**, without
MCU reset, flash, manual USB action, USBIP attach or an extra router restart.
Uptime continues across the outage; retained ack is 503. The full post-restart
command/ack, timeout and rejection check passes at **11:07:06 UTC**, sequence
**504**. The candidate remains the experimental running image, and its FD
matches the new tty. Production Compose and the original image tag are unchanged.

**Native-event diagnosis:** the [isolated client-mode probe](../helpers/rmw_zenoh_pico_spike/host_tests/README.md#client-reconnect-notification-gap)
reproduces resumed bidirectional traffic/enumeration without a new opening
notification. The pinned Zenoh-Pico client creation/reopen path omits the
connected-event dispatch; initial history synthesis masks this at first
registration, while the older native test used peer acceptance. During the
router-only trial, recovered diagnostics remain connected=0, opened=1, closed=2. The
strict event-correctness gate still fails; no firmware patch or custom
watchdog is added. The six-test native and ASan suites include an explicitly
labeled passing **known-defect reproducer**, not a passing health gate.

That was **one candidate restart/automatic CDC-renumber recovery**, not a
controlled physical unplug/replug cycle or patch-specific hardware A/B proof.
The observed return about 16 seconds after container start is not a passing
five-second bound. The physical cycle is recorded above; graceful shutdown,
correct same-boot native-event reporting, 20-cycle recovery, hardware A/B
causality, a bounded physical recovery deadline and ARM64 deployment remain open.

**Previous host-replacement trial, 2026-10-16 10:52:39 UTC:** after separate approval,
only the experimental host router is replaced by the pinned
[serial-accept candidate](../helpers/rmw_zenoh_pico_spike/host_router/README.md#single-spare-board-host-only-trial-2026-10-16).
Telemetry resumes at **10:51:43 UTC** with uptime **2559.660888671875 s**
and retained ack **501**, without MCU reset, reflash, replug or USBIP attach.
The full stock Docker ROS CLI command/ack, timeout and rejection checks pass
with sequence **502** at **10:52:39 UTC**. The candidate holds the current
`ttyACM1` descriptor; USB remains at bus 1/address 16. It remains running only
for this spare-board experiment; production Compose and the original image
tag are unchanged.

**The built-in event health gate fails:** native-event telemetry remains
connected=0, opened=1, closed=1 despite recovered traffic and the passing
command. Investigate the event/listener discrepancy before relying on this
flag. This is one recovery through host-container replacement, not a
patch-specific physical hotplug result: restarting a process also releases
its old descriptor, and no new unmodified hardware control was run. Another
candidate router restart, physical hotplug, 20-cycle recovery, shutdown and
ARM64 deployment gates remain open. No new hardware disturbance is performed.

**Isolated host result preceding this trial, 2026-10-16:** an opt-in
[serial-accept candidate](../helpers/rmw_zenoh_pico_spike/host_router/README.md)
passes the initial **8/8 container-local PTY cases**, including ten distinct
replacements, followed by **ten complete matrix repeats**: 88 passing cases
and 143 distinct-target replacements in total. The installed router and a
rebuilt unmodified control both fail
all four hotplug cases while passing the four non-hotplug cases. Stock ROS CLI
Float32 pub/echo over isolated TCP loopback **and through a complete virtual
serial link between two routers** passes on all three images; eight
C headers and 882 exported symbol names match, and the `rmw_zenohd` executable
is unchanged. This is a local dependency patch, not an upstream release or
physical recovery result. Full serial-session shutdown exceeds the three-second
SIGTERM grace in the installed, rebuilt unmodified and patched listeners;
that shutdown gate remains open. The router and Pico were unchanged during
those isolated tests; the separately approved hardware trial above follows.

**Pre-trial hardware baseline, 2026-10-16 10:10:13 UTC:** the verified spare
runs the separate **`picoros_usb_connectivity`** image, combining the
polling candidate with native Zenoh transport events. Flash/readback
verification and the full Docker ROS CLI sequence-501 check pass. Initial
native-event telemetry is connected=1, opened=1, closed=0. Native tests
pass 5/5, ten repeats and 5/5 ASan/leak checks. **The first router-only
restart fails:** after the new router started at **10:11:05.226 UTC**, a
100-second passive observation received no recovered status sample. USB
re-enumerated to `ttyACM1`, but the router child retained
`/hostdev/ttyACM0 (deleted)`, verified at **10:12:16 UTC**. The separate ack
observer survives the restart; no command 502 or manual MCU/USB operation
was issued at that point. This was a host-listener blocker, not proof of a
fixed Pico recovery path. The failed state was preserved through isolated
host accept-phase validation until the approved replacement above. See the
[current experiment](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md#built-in-zenoh-connectivity-candidate)
for the exact image hash, limits and evidence. Older failure observations
below remain historical evidence, not the current USB/application state.

**Stage 2 experiment, 2026-10-16:** a separate, inert
[`picoros_usb_bidirectional` experiment](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md)
now builds with generated Float32 deserialization, sequence acknowledgements,
duplicate/out-of-order rejection and an independent local timeout task.
Native guard tests, the Docker cross-build and spare-board typed roundtrip/
timeout/rejection checks pass. One router restart recovered, but the next
restart stalled. A subsequent physical replug actually changed `ttyACM0` to
`ttyACM1`: both host and container saw the updated by-id link, while the
router retained an open `/hostdev/ttyACM0 (deleted)` descriptor. Restarting
only the router restored typed traffic and the full functional check on
`ttyACM1`. A subsequent **clean-session** unplug/replug changed `ttyACM1`
to `ttyACM0`; syscall tracing showed EOF, close, retries and reopening the
by-id path. The same router passed the full functional check at 08:29:10
UTC, without mount changes or router restart. This is one renumber recovery
pass, **not** a repeated-loss or <=5 s recovery result. **Automatic repeated-loss
recovery and the 20-cycle matrix have not passed.** See the experiment
runbook's evidence and opt-in serial tracing procedure. The original
publisher UF2 remains the rollback. Float32 alone cannot prove command age
or reject replay across MCU reboot; those remain production blockers.

Further isolation on the same image reproduced a host serial listener
retaining a deleted descriptor when hotplug occurs **during its initial
handshake**, using only pseudo-terminals in an isolated Docker container.
The [regression procedure](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md#isolated-host-listener-hotplug-regression)
is repeatable without touching hardware. The accept-phase retry loop is a
separate demonstrated host failure path; the cause of MCU handshake
silence after router-only restart remains unresolved. No dependency patch,
firmware change, or production migration is justified as a proven fix yet.

The previous hardware experiment used the separate **`picoros_usb_diagnostics`**
UF2 to distinguish an executing probe task from a frozen LED and to expose
stalled application progress. Following physical BOOTSEL and flash-ID/hash
verification, it was flashed to the spare at 08:45 UTC; flash verification
passed. Its full Docker ROS CLI baseline passed at **08:46:22 UTC**,
sequence **301**, including timeout and rejection checks. It changes no
USB/session recovery policy. The user confirmed orange heartbeat once per
second before a single router-only restart at **08:51:13 UTC**. Docker
needed SIGKILL after a five-second SIGTERM grace period; the new process
started at 08:51:18. Both filtered and unfiltered bounded ack observers
then failed, the latter ending at **08:52:48 UTC**, beyond the nominal
60-second lease. No fresh command was sent. USB and the router's current,
non-deleted serial descriptor remained present. The user confirmed
**orange heartbeat continues after the failure**: the probe and app loop
execute, but that does not establish publish success or transport-task
progress. A 20-second passive router syscall trace showed no serial
reads/writes. That failed state was preserved until the user's subsequent
physical BOOTSEL operation; its logs and router state were saved before
the next flash. Stable by-id or per-board udev
names remain the intended Pi deployment approach, not a substitute for
recovering an existing serial descriptor or session.

The next hypothesis is **blocking USB RX starving Zenoh's shared
executor**, including its lease/reconnect futures. The pinned source uses
one executor, while the Pico USB get-character loop can wait indefinitely
for data. A Docker experiment with the actual executor/framing code and
a modeled serial read reproduced timer starvation despite an independent
heartbeat; it does **not** prove the MCU's present call stack or recovery.
A separate **`picoros_usb_transport_probe`** UF2 adds
entry/return and task-identity markers for executor/USB reads. It was
flashed at **09:08:34–09:08:37 UTC**, after rechecking the spare's flash ID,
16 MiB capacity and UF2 hash; flash verification and target metadata passed.
The application reattached automatically with the same USB serial/by-id.
The unchanged test router started at **09:08:51 UTC**, and the full Docker
ROS CLI baseline passed at **09:09:19 UTC**, sequence **401**, including
timeout and rejection checks. The user confirmed **orange blinking once
per second** before a single router-only restart at **09:12:55 UTC**.
Docker again needed SIGKILL after five seconds of SIGTERM grace (old exit
137), so this was not verified graceful shutdown. The new router process
started at **09:13:00.746 UTC**. A filtered ack-401 observer and a later
unfiltered observer both received no sample, the latter ending at
**09:14:23 UTC**, over 80 seconds after startup. Both observers needed
bounded forced shutdown, exit 137. USB stayed at address 12, by-id still
resolved to `ttyACM0`, and the child router held a current, non-deleted
serial descriptor. No fresh command, USB operation or MCU reset followed
the restart. The user subsequently reported **light blue/turquoise blinking
approximately once per second**, recorded at **09:16:31 UTC** with unchanged
router start time/USB address. This is consistent with the cyan marker:
no observed executor entry/return progress and an unmatched USB-read entry
in the same task for nominally >=65 seconds, while app/probe tasks execute.
It supports the blocking-RX mechanism, not a captured MCU stack, hard
deadline, or recovery result; task cancellation/deletion can also leave
unmatched markers. Fresh restart evidence is in
`/tmp/picoros-transport-probe-20261016T0909-H7X1Ux/router-only-01/`.
See the [probe procedure](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md#executorusb-read-probe-variant)
for marker limits. No recovery fix or production change has been established.

A separate **`picoros_usb_serial_poll`** candidate is now built, **not
flashed**, in its own `build/firmware_picoros_usb_poll_sdk220/` directory.
It changes only established async USB reception and nominal read-future
scheduling (5 ms), preserving partial COBS frames and using the pinned
codec/CRC with a bounded fixed-buffer collector and a mutex try-read.
Initial/session handshakes, TX, USB deinit/reinit and host listener recovery
are unchanged and can still block recovery. Native Docker CTest **4/4** plus
ten repeats, **3/3** ASan checks and final ELF wiring verification pass;
these are modeled software tests, not RP2040 recovery evidence. Existing
rollback UF2 hashes are unchanged. The next physical step is BOOTSEL and
flash-ID/hash/flash-content verification on the spare, fresh baseline
sequence **501**, then one router-only loss experiment before any matrix.
That MCU/router failure was preserved during candidate development; the
later connectivity flash supersedes that hardware state.
See the [candidate procedure](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md#cooperative-serial-rx-candidate)
for the UF2 hash, test limits and retained initial link-audit correction.

The next opt-in build, **`picoros_usb_connectivity`**, combines that polling
candidate with Zenoh's built-in transport-events listener rather than a
new heartbeat/reconnect protocol. It uses a separate
`build/firmware_picoros_connectivity_sdk220/` directory and adds ROS
`transport_connected`, `transport_opened`, `transport_closed` telemetry.
`AUTO_RECONNECT` and liveliness were already enabled; the new configuration
enables `CONNECTIVITY` (unstable API) only for this experiment and exposes
the borrowed Pico-ROS session through a guarded local patch. This does not
unblock the executor by itself and cannot report an outage over a dead
link; connection state does not replace a fresh command/ack check.
The polling-only UF2 and all older rollback images remain separate.
See the [built-in connectivity procedure](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md#built-in-zenoh-connectivity-candidate).
Its verified flash and sequence-501 functional baseline pass; the first
single router-only restart fails with a stale host serial descriptor, as
recorded at the top. The migration plan remains reference/evidence, not
permission to port production controllers.

Software BOOTSEL attempts on the verified spare at 08:43 UTC failed:
`picotool` found no reset interface, and a 1200-baud/DTR request did not
produce a BOOTSEL device. WSL observed USB re-enumeration back to the same
application identity, not proof of an MCU reset. No firmware was flashed;
the router was stopped during those attempts. The later diagnostic flash
used physical BOOTSEL, not software reset; see the
[experiment evidence](../helpers/rmw_zenoh_pico_spike/firmware_picoros/experiments/README.md).

## 1. Decision and evidence

The intended path is MCU application -> **upstream Pico-ROS** (ROS graph and
Zenoh publication) -> Zenoh-Pico over USB CDC -> host `rmw_zenohd` ->
Lyrical `rmw_zenoh_cpp`. Reuse generated micro-ROS ROSIDL types, RIHS hash and
Micro-CDR callbacks, **not** the XRCE RMW/agent or a custom ROS wire protocol.
The selected prototype is the non-actuating
[`firmware_picoros/`](../helpers/rmw_zenoh_pico_spike/firmware_picoros/)
`std_msgs/msg/Float32` roundtrip application; the [application](../helpers/rmw_zenoh_pico_spike/firmware_picoros/main.c)
publishes uptime, ack and diagnostic state every 100 ms and subscribes to
`/picoros_usb_spike/command`. No production Pico
firmware has been replaced. Keep the existing agent/bridge and known-good
production images as rollback until every board passes its gates.

| Result on spare KAmod RP2040 | Evidence / limit |
| --- | --- |
| Verified board | BOOTSEL flash ID `0xE464C498C3380C35`, 16 MiB flash; application USB `2e8a:000a`, stable by-id filename `usb-Raspberry_Pi_Pico_E464C498C3380C35-if00`. Never identify a board by `ttyACM0` alone. |
| Raw serial Zenoh | Official Zenoh-Pico `z_pub` reached a host subscriber; this **does not** establish ROS compatibility. |
| ROS-compatible publisher | Host [Pico-ROS proof](../helpers/picoros_poc/) passed; the pinned MCU UF2 then advertised `/picoros_usb_spike/value [std_msgs/msg/Float32]`, RIHS `7170d3d8f841f7be3172ce5f4f59f3a4d7f63b0447e8b33327601ad64d83d6e2`, and ROS decoded increasing `39.0`, `40.0`, `41.0`. No XRCE RMW symbols were linked. |
| RTOS and LED diagnostics | Bare-metal GPIO16 WS2812 probe and **one-core** FreeRTOS LED task passed. WS2812 keeps its old color across reset; use a full power-cycle. The PIO helper needs about 400 us after a frame for latch. GPIO25 is not this board's onboard LED. |
| Zenoh-Pico fork disconnect | Its USB reader originally waited forever for another byte and writer falsely reported success after CDC loss. The local [USB disconnect patch](../helpers/rmw_zenoh_pico_spike/patches/zenoh-pico-usb-disconnect.patch) reports failure to upstream Zenoh-Pico's reconnect machinery; it does not alter the ROS wire format. After router shutdown, CDC re-enumerated; after manual WSL reattachment and router start, ROS data returned without resetting the MCU (one trial). |
| Physical USB hotplug | With a bind mount of only `/dev/serial/by-id` and fixed `--device=/dev/ttyACM0`, Zenoh logged `early eof` and the topic did not return: WSL recreated the by-id directory, leaving the old container bind mount empty. With `/dev:/hostdev:ro` and `c 166:* rwm`, the **same** router retried and ROS decoded `13.0`, `14.0` after manual WSL USB attachment (one trial). The device reused `ttyACM0`; new-minor recovery is **not yet proven**. |
| Alternative `rmw_zenoh_pico` | Its isolated RP2040 cross-build succeeded, but the KAmod stopped during `rclc_support_init_with_options`, before application USB. It is a comparison, **not** the selected migration target. |

Pins: Pico-ROS
[`b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2`](https://github.com/Pico-ROS/Pico-ROS-software/tree/b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2),
its Zenoh-Pico fork `g4sp3r/zenoh-pico@0eb81272682a526b4bcf5a850c16ac74bbc6c73e`,
Pico SDK 2.2.0 `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`,
FreeRTOS 11.2.0 `88971102bf8aa34b4fcc24f176db620dd9c854f7`.
The host must use the **actual Lyrical router image**, not an assumed
compatible protocol version. See [Pico-ROS protocol mismatch issue](https://github.com/Pico-ROS/Pico-ROS-software/issues/19),
the [ROS Zenoh design](https://github.com/ros2/rmw_zenoh/blob/rolling/docs/design.md)
and [official Zenoh-Pico RP2040 USB example](https://zenoh.io/blog/2025-01-08-introducing-raspberry-pi-pico-support-in-zenoh-pico/).

## 2. Reproduce the non-actuating hardware test

Prerequisites: Docker and the spare KAmod only; do **not** start the complete
robot stack or flash wheels, knives, IMU or INA226. WSL users must attach the
board from Windows to WSL first; after every USB re-enumeration WSL attachment
can be lost. On a Pi, use its local USB port instead. The test router
listens on TCP 7447: **stop any other router on that port first**. Do not
stop an unrelated running robot's router just to perform this test.

### Build and flash *only the spare board*

The first script produces generated micro-ROS type support and a pinned
FreeRTOS checkout. It **regenerates `build/`**, removing Pico-ROS outputs;
run it only if those prerequisites are absent and always **before**
[`build_picoros.sh`](../helpers/rmw_zenoh_pico_spike/build_picoros.sh).
The second script fetches pinned Pico-ROS/SDK revisions, applies small
platform/diagnostic patches and builds via the wheels builder image.
The current firmware uses
[`configNUMBER_OF_CORES=1`](../pico_firmware/config/FreeRTOSConfig.h).

```bash
# Only if build/libmicroros/libmicroros.a or build/FreeRTOS-Kernel is missing:
./helpers/rmw_zenoh_pico_spike/build.sh
./helpers/rmw_zenoh_pico_spike/build_picoros.sh
test -s helpers/rmw_zenoh_pico_spike/build/firmware_picoros_connectivity_sdk220/picoros_usb_connectivity.uf2
docker run --rm --entrypoint bash \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/build:/build:ro" \
  relobot_pico_ware_wheels_microros:latest \
  -lc 'arm-none-eabi-size /build/firmware_picoros_connectivity_sdk220/picoros_usb_connectivity.elf'
```

Retained diagnostic build: text 152,344 B; BSS 141,012 B (includes a
128-KiB FreeRTOS heap). This is **not** measured free
heap or stack headroom. If the wheels builder image is missing, build it
first (this does not flash anything):

```bash
docker build -t relobot_pico_ware_wheels_microros:latest pico_ware_wheels_microros
```

See also the [wheels firmware instructions](../pico_ware_wheels_microros/README.md).
Confirm only the test board is connected and put it in BOOTSEL:

```bash
lsusb -d 2e8a:0003
docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb \
  relobot_pico_ware_wheels_microros:latest picotool info -a
# STOP unless flash id is exactly 0xE464C498C3380C35.
docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/build/firmware_picoros_connectivity_sdk220:/project:ro" \
  -w /project relobot_pico_ware_wheels_microros:latest \
  picotool load -v -x picoros_usb_connectivity.uf2
```

The image reboots the board; if WSL detaches it, reattach from Windows.
`picotool`'s `--privileged` is restricted to this **manual flashing command**,
not the running Zenoh router. The WS2812 diagnostic stages require a full
power-cycle to interpret reliably: yellow = Pico-ROS entry, cyan = `z_open`,
red = TinyUSB entry, magenta = TinyUSB ready, orange = waiting for CDC
connection, white = Zenoh session opened, green = publisher declared.
Repeated red blinks indicate an error. **Green is not a link-health check**
after initialization.

### Start one router, verify ROS, stop it

**Preserve a running candidate.** The startup block below describes initial
stock-router bring-up only. Do not run its `up` command against the current
experimental candidate, since ordinary Compose recreation can replace its
patched image. For the recorded recovery setup, use the
[candidate report](../helpers/rmw_zenoh_pico_spike/host_router/README.md) and
explicit candidate override; an existing candidate only needs the ROS checks.

Use the exact by-id filename printed for the *test board*. Copy the
[serial config template](../helpers/picoros_poc/router-serial.example.json5)
to `/tmp` (mounted into the router by [Compose](../ros2_ws/docker-compose.yml));
edit the single `REPLACE_WITH_BOARD_SYMLINK` placeholder if the board differs.
Do not commit a machine-specific config or put it in a production startup.

```bash
ls -l /dev/serial/by-id/
readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Pico_E464C498C3380C35-if00
cp helpers/picoros_poc/router-serial.example.json5 /tmp/relobot-picoros-test.json5
sed -i 's/REPLACE_WITH_BOARD_SYMLINK/usb-Raspberry_Pi_Pico_E464C498C3380C35-if00/' \
  /tmp/relobot-picoros-test.json5
ZENOH_ROUTER_CONFIG_URI=/tmp/relobot-picoros-test.json5 \
  ./start_robot.sh up ros2_zenoh_router --detach
docker compose -f ros2_ws/docker-compose.yml logs --tail=50 ros2_zenoh_router
docker compose -f ros2_ws/docker-compose.yml exec -T ros2_zenoh_router bash -lc \
  'source /opt/ros/lyrical/setup.bash &&
   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 topic list -t'
docker compose -f ros2_ws/docker-compose.yml exec -T ros2_zenoh_router bash -lc \
  'source /opt/ros/lyrical/setup.bash &&
   RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 topic info -v /picoros_usb_spike/value'
docker compose -f ros2_ws/docker-compose.yml exec -T ros2_zenoh_router bash -lc \
  'source /opt/ros/lyrical/setup.bash &&
   RMW_IMPLEMENTATION=rmw_zenoh_cpp timeout 15 ros2 topic echo \
     /picoros_usb_spike/value std_msgs/msg/Float32 --once'
```

Expect one publisher, exact type and RIHS above, and **increasing decoded
numbers**, not just a tty or a node name. Repeat `echo --once` to compare.
To check live recovery without restarting the router, physically unplug and
replug **the spare** board here on the WSL development host, reattach to WSL
if needed, and repeat `topic list`, `topic info -v` and `echo`. Record
`date -Is`, old/new by-id targets, router logs, number of publishers and time
from USB enumeration to fresh samples. For a real minor-change test here,
keep another harmless ACM device connected so the KAmod reappears on a
different `ttyACM*`; do not disrupt a production device to force renumbering.
The robot Pi test remains a **separate later deployment gate**, not a
prerequisite for completing this WSL investigation.

```bash
docker compose -f ros2_ws/docker-compose.yml logs --since=2m ros2_zenoh_router
# When finished, stop ONLY the test router (not ./start_robot.sh down):
docker compose -f ros2_ws/docker-compose.yml stop ros2_zenoh_router
```

The [router service](../ros2_ws/docker-compose.yml) mounts the stable parent
`/dev` as `/hostdev:ro` and allows character major 166 (`ttyACM*`) in its
device cgroup. Its Zenoh locator is
`serial//hostdev/serial/by-id/<board>#baudrate=115200`; the MCU locator is
`serial/usb#baudrate=115200`. `:ro` stops edits to the mount, **not** serial
writes through an opened port. The rule covers *all* ACM devices; review
least privilege on the Pi. Without `ZENOH_ROUTER_CONFIG_URI`, the default
router has **no serial listener**. Zenoh's [serial accept loop](https://github.com/eclipse-zenoh/zenoh/blob/2687c51/io/zenoh-links/zenoh-link-serial/src/unicast.rs)
retries reopening the path; mounting only the ephemeral by-id subdirectory
defeated it when WSL replaced that directory. See [Docker bind-mount
propagation](https://docs.docker.com/engine/storage/bind-mounts/#configure-bind-propagation):
`rshared`/`rslave` propagate new mount points, not replacement of the
already-bound by-id directory.

## 3. Critical gaps, in priority order

1. **Connection loss and safe output state.** The publisher proof is
   unidirectional. Add an **inert** typed `Float32` subscriber on the spare
   board using Pico-ROS and generated *deserialization*, prove host->MCU
   delivery, correct hash, QoS and duplicate/stale-command rejection after
   router restart and USB reset. Do not attach a motor. For real actuators,
   keep the existing *local* command timeout, watchdog, emergency stop,
   slew/direction limits and no automatic restart on reconnect. A green LED,
   declared node or `picoros_publish()` returning success cannot serve as
   proof of a live peer. Current probe calls `fail(6)` permanently on a
   publish error; repeated-loss recovery is not established.
2. **Reconnect coverage.** Test at least 20 cycles each of (a) physical
   unplug/replug, (b) router restart, (c) Pico reset, including lost power
   and actual `ttyACM0` -> `ttyACM1` renumbering. Measure time from device
   enumeration to live ROS data (target <=5 s, revisit if the current
   60-s transport lease prevents it); assert one publisher, no ghost
   entities, no MCU reset required for router-only failure. Run this
   matrix **here in WSL first**. Manual WSL USBIP reattachment is test-host
   behavior; repeat a smaller representative matrix on the actual Pi
   before deployment. Preserve service to unrelated ROS nodes while
   testing serial hotplug. If the fork repeatedly deinitializes TinyUSB on
   session reconnect, assess keeping CDC up independently of Zenoh, with
   explicit failure reporting rather than hiding transport errors.
3. **Memory and timing.** Measure FreeRTOS current/minimum-ever free heap,
   every task's stack high-water mark, overflow/malloc failure hooks,
   reset causes and CPU utilization on the target under reconnect storms
   and representative rates. Current
   [`FreeRTOSConfig.h`](../pico_firmware/config/FreeRTOSConfig.h)
   disables stack-overflow and allocation-failure hooks, so a successful
   link does not establish memory safety. Compare serial throughput,
   message latency/jitter and dropped samples to today's micro-ROS baseline
   under concurrent sensor/control traffic; run >=30 minutes steady and
   >=60 minutes loaded before judging headroom. Set acceptance margins from
   measured worst-case usage, not from UF2 size alone.
4. **Two cores: explicit next experiment.** The current
   [`configNUMBER_OF_CORES=1`](../pico_firmware/config/FreeRTOSConfig.h)
   runs FreeRTOS tasks on one RP2040 core; the other is not used for this
   application. First record the single-core baseline, then **build a
   separate two-core test UF2** with `configNUMBER_OF_CORES=2`, instrument
   actual per-core work, stack/heap, shared data and USB/PIO synchronization,
   and repeat typed ROS, bidirectional traffic and reconnect/stress gates.
   Check for priority inversion, races and timing jitter as well as possible
   gains: enabling SMP does not automatically speed up a one-task publisher.
   Keep single-core as rollback until the two-core firmware demonstrably
   passes the same safety gates; do not copy older `configNUM_CORES` examples
   into this pinned kernel. See the [FreeRTOS RP2040 SMP demo](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/Raspberry-Pi/smp-demos-for-the-raspberry-pi-pico-board).
5. **ROS semantics and other boards.** Validate correct types/RIHS,
   best-effort telemetry versus reliable command QoS, queue depth,
   sequence behavior, transient graph liveliness, larger/variable-length
   messages and service behavior where needed. Pico-ROS has fixed QoS
   declarations; it is **not** a drop-in `rclc` replacement. IMU and wheel
   `header.stamp` need an explicit host/MCU clock synchronization strategy
   and bounded clock error (current firmware uses
   `rmw_uros_epoch_nanos()`). Test the actual 30 Hz IMU/wheel and 10 Hz
   INA226/knife traffic before porting application callbacks.
6. **Platform, supply-chain and rollout.** The spare is a KAmod RP2040;
   production includes different RP2040 boards and an **RP2350 INA226**.
   Validate board pins, USB descriptors, boot/flash recovery, memory and
   RTOS/Pico SDK support separately per board. Pin generated interface and
   toolchain revisions, audit the Zenoh-Pico fork/local patches (including
   the opt-in polling candidate)
   against current upstream, replace temporary LED instrumentation with
   proper logging/test hooks, and document deterministic builds and
   known-good UF2 rollback. Observe host USB permissions and
   `/dev:/hostdev:ro` exposure on the real Pi; check router startup both
   with a disconnected board and after reconnect. Do not remove XRCE agent,
   bridge or old firmware until every sensor and actuator passes end-to-end
   gates and an actual rollback drill.

**Next execution order:** complete spare-board two-way ROS proof and
20-cycle **WSL** reconnect matrix; measure single-core RAM/latency, then
build and validate the **two-core test UF2** against that baseline. After
those gates, migrate **knives first, then IMU, then wheels**. For knives,
start with the motor power physically disconnected; demonstrate a local
timeout, E-stop, no restart from stale commands and known-good firmware
rollback before any controlled powered test. INA226/RP2350 follows as a
separate later port. Repeat USB/reconnect and safety checks on the actual
robot Pi before production deployment. Never infer actuator safety from
the current one-topic publisher.
