# Isolated host serial accept experiment

This directory retains the **laboratory comparison build and historical evidence**.
The ordinary [deployment Dockerfile](../../../ros2_ws/ZenohRouter.dockerfile)
now includes the same pinned serial/hotplug patch. Production startup uses one
image and one router service; it does not select a knives-specific Compose file.
The deployment image still needs final-image and ARM64 hardware qualification.

The original experiment was an **opt-in laboratory candidate**.
It changes only the pinned Zenoh serial listener's error path. The isolated
tests below must not replace a running hardware router. Separately approved
single spare-board host-only trials are recorded at the end of this document.
Those isolated trials did not change Pico firmware, production Compose, udev
rules, or device mounts. The historical image IDs below describe those trials,
not a newly qualified deployment image.

## Source and change

- Runtime base: the existing `ros2_ws-ros2_zenoh_router:latest` image, recorded
  before building. The first baseline image ID is
  `sha256:27d53bfe8edbbfd46b56e785452984f70f7b761d737b92acabf01af9a625322b`.
- ROS package: `rmw_zenoh_cpp` / `zenoh_cpp_vendor` 0.10.6.
- Zenoh: `2687c51352121f006e3a603ce07925a8ad0b295c`.
- Zenoh C: `05bd370343b5161ca9269649b9a914c9c2dc4170`, the ROS vendor's
  Rust >=1.88 variant. Both ROS vendor variants reference the same Zenoh commit.
- Rust: 1.93.0, explicitly selected to match the pinned Zenoh C toolchain.
- Cargo uses the committed lockfile, existing default features plus
  `shared-memory,unstable,zenoh/transport_serial`, matching the ROS vendor flags.
- Builder base is pinned to the official multi-architecture index
  `sha256:af306cfa71d987911a781c37b59d7d67d934f49684058f96cf72079c3626bfe0`.
  It selects the unchanged tested amd64 manifest
  `sha256:4727898c104ecd2e22d780925832502faee9fe4e70581b8572af081370b315a0`
  or arm64 manifest
  `sha256:8aa70d1416cf5b1cff4b95ec6c57f1c5e4e649a3b53d616a26695cda6fbb46bc`.
  This removes the x86-only build restriction, **not** the ARM64 validation gate.

The [patch](./zenoh-serial-accept.patch) returns an `accept()` error to the
existing outer retry loop when the default `release_on_close=true` applies.
Before opening a port, that loop drops any previous port, including one left
behind by a failed buffer clear. This also avoids opening a still-exclusive
port before dropping its old handle. No periodic health probe, new supervisor,
timeout, or reconnect policy is introduced. The `release_on_close=false`
branch is unchanged; it is not a proposed hotplug configuration.

The build first compiles an unmodified library. The patched stage explicitly
cleans `zenoh-link-serial` before rebuilding, so a cached Git dependency cannot
silently leave the unmodified crate in the candidate. Runtime images retain
the installed `rmw_zenohd` executable and replace only `libzenohc.so`.

## Build inside Docker

Run from the repository root. Record the baseline image ID and use the same
base for both targets; these commands do not restart the running router.

```bash
docker image inspect ros2_ws-ros2_zenoh_router:latest --format '{{.Id}}'
docker build --progress=plain --target rebuilt-baseline \
  -t relobot-zenoh-serial-baseline:experimental \
  helpers/rmw_zenoh_pico_spike/host_router
docker build --progress=plain \
  -t relobot-zenoh-serial-accept:experimental \
  helpers/rmw_zenoh_pico_spike/host_router
```

Before a hardware trial, compare the generated C headers and exported ABI
against the installed library and verify that the runtime executable is
unchanged. Passing a descriptor test alone is not an ABI or ROS traffic check.

### ARM64 deployment build

Build the deployment image natively on the Pi or on a builder with an actual
ARM64 worker:

```bash
./start_robot.sh build ros2_zenoh_router
docker image inspect relobot-zenoh-router:local --format '{{.Id}} {{.Architecture}}'
```

The ordinary Compose build uses the repository root as its context and reuses
this directory's patch without duplicating it. The
development host currently advertises only amd64 execution; no ARM64 build or
runtime qualification is recorded. Do not install privileged emulation or
silently reuse the x86 candidate image ID.

For a separate baseline-versus-patch laboratory comparison only, first retain
an unmodified ROS router on that architecture and record its
immutable local image ID (or registry digest). BuildKit does not accept a bare
local image ID in `FROM`; retain a unique local tag derived from that ID, or
pass a published `repository@sha256:...` reference:

```bash
base_id=$(docker image inspect ros2_ws-ros2_zenoh_router:latest --format '{{.Id}}')
test "$(docker image inspect "$base_id" --format '{{.Architecture}}')" = arm64
base="relobot-zenoh-base:arm64-${base_id#sha256:}"
docker tag "$base_id" "$base"
docker build --platform linux/arm64 --progress=plain --target candidate \
  --build-arg ROUTER_IMAGE="$base" \
  -t relobot-zenoh-serial-accept:arm64-candidate \
  helpers/rmw_zenoh_pico_spike/host_router
docker image inspect relobot-zenoh-serial-accept:arm64-candidate \
  --format '{{.Id}} {{.Architecture}}'
```

Keep this separate tag. Record the installed ROS vendor version, generated
`candidate.sha256`, ABI comparison and PTY/stock-ROS traffic tests on the Pi
before selecting a deployment image as `ZENOH_ROUTER_IMAGE`. The index pin, source
commits and `cargo --locked` make inputs explicit; they do not pin every Debian
package in the runtime base. Pin and retain that base image for reproducibility.

## PTY regression matrix

The [matrix](../host_tests/serial_listener_matrix.py) runs the actual router
binary on newly allocated container-local PTYs. It opens **no physical device**,
binds no host `/dev`, and requires no custom ROS node. Run it separately on the
installed image, rebuilt unmodified image, and patched image:

```bash
image=relobot-zenoh-serial-accept:experimental
evidence=$(mktemp -d /tmp/picoros-host-serial-matrix-XXXXXX)
docker image inspect "$image" > "$evidence/image.json"
docker run --rm --network none --entrypoint bash \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/host_tests:/experiment:ro" \
  -v "$evidence:/evidence" "$image" -c \
  'source /opt/ros/lyrical/setup.bash &&
   python3 -B /experiment/serial_listener_matrix.py \
     --evidence-dir /evidence/matrix --cycles 10'
```

Cases cover silent initial handshake, valid serial Init/Init+Ack, bad CRC
followed by a valid handshake on the same tty, a device absent at startup,
replacement during accept, replacement during a partial frame, a missing-path
interval, and ten consecutive replacements. Replacement PTYs are allocated
before closing the old master so their names must differ. A recovered listener
must hold exactly the new PTY, with no old/deleted serial descriptor, within
five seconds, then complete the serial handshake. A baseline failure also
checks that another process can open the replacement: visibility is not recovery.

`summary.json` and each case's `result.json` preserve observations and failures;
exit 1 is a failed test, not an expected-pass waiver for the installed image.
`cleanup_forced` records bounded process cleanup separately: an acknowledged
serial handshake is not a complete Zenoh session, and shutdown may be waiting
for the higher-level handshake. The silent case checks no unnecessary reopen.
The fixed empty-payload frame vectors use the pinned z-serial 0.3.1 wire format;
this harness is not a general serial codec or application-protocol implementation.

The original minimal [hotplug reproducer](../host_tests/serial_listener_hotplug.py)
remains unchanged for supplementary before/after comparison. It may reuse a
PTY number after the old handle is released; the new matrix explicitly prevents
that, so its distinct-target observations are the primary reopening evidence.
These are Linux/WSL
Docker observations, **not** an independent bare-metal reproduction or a Pico
USB recovery result. Native transport events and a fresh ROS command/ack on the
spare still require a separately approved hardware trial.

## ROS CLI smoke test

The [smoke test](../host_tests/host_router_ros_cli_smoke.sh) uses only stock
`ros2 topic pub` and `ros2 topic echo` with a fresh loopback-only router config.
Run it on all three images using the same isolated Docker command as the matrix,
but replace the inner command with:

```bash
bash /experiment/host_router_ros_cli_smoke.sh /evidence/ros-smoke
```

It must publish and receive the typed Float32 value 37.0. This exercises the
installed RMW/CLI against the rebuilt C library over TCP; it does **not** prove
a complete Zenoh session over serial or Pico recovery. Its router and ROS CLI
processes have bounded timeouts, and all configs/logs remain in the fresh
evidence directory.

## Complete serial-session traffic test

The [two-router test](../host_tests/serial_router_ros_cli.py) allocates two
container-local PTYs and relays their raw bytes without interpreting the
protocol. One stock `rmw_zenohd` listens on one PTY; another connects on the
other. Each exposes a different TCP loopback port for its local ROS CLI client.
Multicast and gossip discovery are disabled. There is no router-to-router TCP
connection: the serial link is their only configured inter-router path.

Using the same isolated Docker command, run:

```bash
source /opt/ros/lyrical/setup.bash
python3 -B /experiment/serial_router_ros_cli.py \
  --evidence-dir /evidence/full-serial-session
```

Only stock `ros2 topic pub` / `ros2 topic echo` create ROS nodes. Float32 value
37.0 must cross from the first router to the second, and value 38.0 in the
reverse direction. This verifies complete Zenoh sessions and typed traffic
over virtual serial, **not** RP2040/TinyUSB behavior or serial reconnection of
an already established session. The latter remains a hardware-trial gate.

## Recorded result: 2026-10-16

The installed image and the rebuilt unmodified control both pass the four
non-hotplug cases and **fail all four hotplug cases**. Each failed case leaves
the router on the old deleted PTY even though the replacement is visible and a
separate fresh open succeeds. The patched image passes **8/8**, including ten
distinct-target replacements in its repeated-hotplug case. Its bad-CRC case
also completes Init/Init+Ack on the original tty after reopening it: exclusive
access does not prevent recovery after the old port is dropped.

The subsequent **ten complete matrix repeats all pass**: 88 candidate cases
including the initial matrix, with **143 distinct-target replacements**. Each
replacement leaves exactly one serial descriptor on the new PTY and completes
Init/Init+Ack. Observed reopen times after replacement-path publication range
from 0.021 to 0.224 seconds in this unloaded virtual-PTY test; this is **not** a
physical USB recovery deadline. The unchanged minimal reproducer also passes
eleven candidate runs, but may reuse a PTY number as noted above.

Stock ROS CLI typed Float32 pub/echo over isolated TCP loopback passes on all
three images. All eight generated C headers match the installed headers byte
for byte, and the exported symbol-name set is unchanged at 882 entries. The
installed `rmw_zenohd` executable hash is unchanged. Dynamic relocation checks
report no unresolved dependencies. These are targeted compatibility checks,
not a complete ABI certification.

The separate full-serial-session test also passes on **all three images**:
stock ROS CLI Float32 values 37.0 and 38.0 cross the two-router virtual serial
link in opposite directions. Each run records nonzero relay byte counts in
both directions and no relay error. None of these tests opens physical USB.

| Image | ID |
| --- | --- |
| Installed | `sha256:27d53bfe8edbbfd46b56e785452984f70f7b761d737b92acabf01af9a625322b` |
| Rebuilt unmodified | `sha256:211c6d8833b4ca60e080a07d50cbb79b0c7492b980225a000cb25cd49c73c569` |
| Patched candidate | `sha256:f5362188e9d04a3bdace2e98a2b5b08adda3fcb34f1a187c4018396f1ffab2d3` |

Checksums:

```text
rmw_zenohd (all three):
c01e70e281fa9145ce4f2b456b0c93d0517fe70b3eb28d6f2fe6e1ce8ae01325
Installed libzenohc.so:
91077ad09ed6aa46a8b379a908bb784fb6e39b4e9d0afebda0b0b1d5443df5d3
Rebuilt unmodified libzenohc.so:
bc09f6bade246ca177f963fd2b9fb81a9b1aaa0cfd211507383a4abe9e97a549
Patched libzenohc.so:
1de3c978cb1e70c99f0ce92a5b75dc3fea4aba5862b0cd1ceea954f0d1269426
Local patch:
db722cfc94d1025d68a3c182ee4850271253baec909c1e3417359d4b4157bb7c
```

Evidence is local and temporary:
`/tmp/picoros-host-accept-validation-20261016-ICESgZ/`; the current pointer is
`/tmp/picoros-host-accept-validation-evidence-dir`. It contains image metadata,
build logs, `stock-matrix-final`, `rebuilt-baseline-matrix`, `candidate-matrix`,
the original reproducer captures, C header/export audits, and all ROS CLI logs.
`validation-summary.json` captures the aggregate checks. Full serial-session
captures are `stock-full-serial-session`, `rebuilt-baseline-full-serial-session`
and `candidate-full-serial-session`.

Initial harness runs are retained, but are **not** the results above:
`stock-matrix` incorrectly depended on z-serial trace messages not present in
the installed release, and included an unsupported `release_on_close=false`
listener case; `stock-matrix-02` predates the absent-at-start case. The first
ROS smoke attempt stopped at ROS setup under shell nounset, before any ROS
command; the corrected stock capture is `stock-ros-smoke-02`.

Some cases require the three-second SIGKILL cleanup after completing the
serial Init/Init+Ack exchange; this is recorded independently of their asserted
reopening/handshake result. Across the eleven candidate matrices, bad-CRC,
missing-path and repeated-hotplug cases each need forced cleanup eleven times;
the other five cases never do. In the **complete serial-session test**, the
listening router also exceeds the three-second SIGTERM grace on **all three
images**, including the unmodified controls; the connecting router exits within
that grace. This is a separately recorded shutdown limitation, not a passing
shutdown gate or an established new patch regression. Graceful serial-session
shutdown is **not fixed** by this patch. Silent-listener shutdown completes
without forced cleanup in all eleven candidate matrices.

At the end of the isolated validation, the running spare router was still the
original installed image; no hardware reset, reflash, replug, or fresh command
502 was part of that result. The subsequent single host-only trial follows.

## Single spare-board host-only trial: 2026-10-16

After separate user approval, only the experimental `ros2_zenoh_router`
container was replaced with the exact candidate image recorded above. The
spare KAmod's USB serial remained `E464C498C3380C35`; its existing
`picoros_usb_connectivity` firmware was not reset, reflashed or replugged.
No USBIP attach, parallel tty reader, injected serial bytes, custom ROS node,
watchdog or production controller was involved.

Before replacement, the original router still held
`/proc/41/fd/21 -> /hostdev/ttyACM0 (deleted)`, inode 1818, major/minor
166:0, while the by-id target was `ttyACM1`, inode 1844, major/minor 166:1.
An independent observer using the original stock image, explicit localhost
TCP client configuration and bounded stock ROS CLI echoes started before
replacement and received no pre-swap samples. It had no serial-device mount.

Timeline, UTC:

- **10:51:42.735–10:51:42.950:** stop of the failed original container.
  Its recorded exit code is **0**, with no SIGKILL needed in this trial.
  This does not change the earlier forced-shutdown observations.
- **10:51:43.201:** candidate container starts, selected by exact image ID
  through a temporary Compose override. Entrypoint, command, environment,
  network/IPC, mounts, cgroup rule, restart policy, security options and
  privileges match the original container.
- **10:51:43.936:** the router logs an accepted serial Zenoh transport
  from the Pico (`c3`). The first observer samples arrive at
  **10:51:43**: uptime **2559.660888671875 s**, ack **501**. The uptime is
  consistent with the pre-failure boot, rather than an MCU reboot.
- **10:51:59:** the current serial descriptor is
  `/proc/23/fd/25 -> /hostdev/ttyACM1`, inode 1844, major/minor 166:1,
  matching the by-id target. The old deleted descriptor is absent.
- **10:52:12–10:52:39:** the unchanged
  [`check_bidirectional.sh`](../check_bidirectional.sh) passes with sequence
  **502**, exit **0**: exactly one command subscriber, fresh command/ack,
  active state, local timeout, duplicate 502 / older 501 rejection, and
  invalid-zero rejection. Last accepted sequence is now **502**.

**The native-event health signal does not pass:** throughout resumed traffic,
the adapter reports `transport_connected=0`, `transport_opened=1`, and
`transport_closed=1`, including while command 502 succeeds. A new opening
event is not reflected in these application counters. The cause is not
established by this trial; do not use this flag alone as a readiness check or
claim that native events correctly describe reconnection. No firmware change
was made to hide this discrepancy.

This is **one host-replacement recovery observation**, not proof that the
patch alone caused recovery: replacement itself drops the old process's
descriptor, and no fresh unmodified hardware control was run. In particular,
the candidate has not yet been challenged with a new physical hotplug during
its own initial handshake or with another router-only restart. The isolated
distinct-target PTY cases remain the patch-specific evidence. The 20-cycle
hardware matrix, physical recovery deadline, candidate serial-session
shutdown and Raspberry Pi ARM64 deployment gates remain open.

Evidence is local and temporary:
`/tmp/picoros-host-accept-hardware-20261016-Mv5DEX/`; the pointer is
`/tmp/picoros-host-accept-hardware-evidence-dir`. It records original/stopped
and candidate container metadata, original/final router logs, USB and file
descriptor snapshots, timestamped passive telemetry, runtime checksums,
settings comparison, the sequence-502 log and a hardware-trial summary.
The earlier isolated `validation-summary.json` is an unchanged historical
snapshot, not the current hardware state.

The candidate remains running **only as the spare-board experiment**. The
base Compose file and original image tag are unchanged; an ordinary Compose
recreation without the temporary override can select the original image.
The evidence directory contains both `candidate.compose.yml` and
`rollback.compose.yml`, pinned to their respective image IDs. If a later
rollback is approved, run from the repository root:

```bash
trial="$(cat /tmp/picoros-host-accept-hardware-evidence-dir)"
docker compose -p ros2_ws \
  -f ros2_ws/docker-compose.yml \
  -f helpers/rmw_zenoh_pico_spike/router-trace.compose.yml \
  -f "$trial/rollback.compose.yml" \
  up --no-deps --no-build --pull never --force-recreate --timeout 5 \
  -d ros2_zenoh_router
```

This replaces the router again; it is not a passive inspection command and
was not executed after the successful trial. That working state was preserved
until the separately approved investigation and single restart trial below.

## Native connectivity investigation: 2026-10-16

The missing event is reproduced without USB, ROS or a host router by a new
[client reconnect probe](../host_tests/README.md#client-reconnect-notification-gap)
using the actual Pico-ROS interface/adapter and pinned Zenoh-Pico code. After
automatic client reconnection, fresh payloads cross both ways and transport
enumeration returns one, but the original listener remains at opened=1,
closed=1; a newly registered history listener immediately sees the live
transport. The pinned client's creation/reopen path does not dispatch the
connected event. Initial history synthesis hides that omission at the first
registration; the previous native test covered peer acceptance instead.

The connectivity-enabled suite and ASan/leak checks each pass 6/6, including
the explicitly labeled known-gap characterization, with five further gap
repeats. The strict `--require-reopen-event` gate correctly fails, exit 1.
The connectivity-disabled suite remains 4/4. **The event-correctness gate is
still failing, not fixed.** No new dependency patch, synthetic event,
watchdog, firmware build or flash is introduced. Evidence pointer:
`/tmp/picoros-client-connectivity-evidence-dir`.

## Single candidate router restart: 2026-10-16

After the user approved continuing, one router-only stop/start was performed
on the same candidate container and exact image, with the same spare firmware.
No MCU reset, flash, manual replug, USBIP attach or mount/configuration change
was issued. The stock-image observers used only TCP, without serial mounts.

Timeline, UTC:

- **11:05:15:** the full unchanged ROS CLI baseline passes, sequence **503**.
- **11:06:18.561–11:06:23.681:** stopping the active candidate exceeds the
  five-second SIGTERM grace; Docker records exit **137**, not graceful exit.
- **11:06:23.743:** the same candidate container starts again. The listener
  first attempts the existing by-id path, then logs repeated `No such device`
  and a brief missing-path interval. USB automatically re-enumerates from
  bus 1/address **16**, `ttyACM1`, to address **17**, `ttyACM0`, retaining USB
  serial `E464C498C3380C35`. No user USB action or extra router restart occurs.
- **11:06:39.724:** the same new router process accepts the serial Zenoh
  transport from `c3`. First post-restart observer samples arrive at
  **11:06:39**, uptime **3455.554931640625 s**, retained ack **503**. The last
  pre-outage uptime was **3434.8740234375 s**, consistent with the same MCU
  boot. The observed return is about **16 seconds after container start**,
  not a passing five-second recovery bound.
- **11:07:06:** the full stock ROS CLI command/ack, active/timeout and
  duplicate/older/invalid rejection check passes, sequence **504**, exit **0**.
  The candidate holds `/proc/23/fd/14 -> /hostdev/ttyACM0`, inode **1870**,
  major/minor **166:0**, matching the current by-id target.
- **11:07:29:** independent passive observers still receive uptime/ack and
  native diagnostics. Throughout recovered traffic, opened stays **1**,
  closed increases from **1 to 2**, and connected stays **0**, matching the
  independently reproduced client event gap. A supplementary stock-image
  one-shot ack observer also returns **504**, exit **0**.

**One candidate router-only restart with spontaneous CDC renumber recovery
passes.** This is stronger than the earlier container-replacement observation:
the same candidate process survives an initially unavailable by-id target
and accepts the replacement tty. It is still not a controlled physical
unplug/replug test, repeated-loss matrix, patch-specific hardware A/B proof,
or ARM64 deployment result. The native-event health and graceful-shutdown
gates fail; no firmware change is made to conceal either. The five continuous
CLI observers reach their finite 160-second budgets and require the bounded
three-second forced cleanup (individual exit 137); that is separate from the
delivered samples and passing functional/one-shot checks.

Evidence is local and temporary:
`/tmp/picoros-candidate-restart-20261016-3z6jOg/`; the pointer is
`/tmp/picoros-candidate-restart-evidence-dir`. It contains baseline/post-restart
logs, stop/start and container metadata, serial logs, timestamped observations,
per-observer exit codes, USB/descriptor snapshots and a restart-trial summary.
At the end of that trial, the candidate remains running with last accepted
sequence **504**. Production Compose and the original image tag remain
unchanged. The separately coordinated physical trial follows below; no
automatic 20-cycle matrix or new firmware patch is applied.

## Single candidate physical USB cycle: 2026-10-16

The user disconnects and reconnects only the spare KAmod RP2040, USB serial
`E464C498C3380C35`, with the unchanged `picoros_usb_connectivity` firmware.
The experimental router retains container
`b09bbf0ea8f456556dc04dc781ee9473d8c85c3b66cdcbf862f7fe6955229a1c`,
image `sha256:f5362188e9d04a3bdace2e98a2b5b08adda3fcb34f1a187c4018396f1ffab2d3`,
start time **11:06:23.743 UTC**, and host PID **201093** throughout. Its
`libzenohc.so` hash still matches the candidate recorded above. No router
restart, software MCU reset, reflash, USBIP attach, device opening by another
observer, or mount/configuration change is issued by the agent.

Timeline, UTC:

- **11:37:15:** the full unchanged stock Docker ROS CLI baseline passes,
  sequence **505**, exit **0**. Independent stock-image CLI observers use
  TCP only, without device mounts, and receive all five observed topics.
- **11:38:03:** last pre-unplug samples include uptime **5339.7587890625 s**
  and ack **505**. At **11:38:04.029**, the router records serial EOF, closes
  its transport and retries opening the missing by-id path. Manual snapshots
  confirm the spare is absent from both host and container, with no old
  serial FD retained and no router restart.
- **11:43:58.831:** the same process accepts a serial transport from **7b**.
  The independent observers resume that second: first uptime
  **2.062000036239624 s**, ack **0**, connected **1**, opened **1**, closed
  **0**. USB power loss caused a new MCU boot; neither uptime nor command
  state is preserved across this cycle.
- **11:44:00.192:** passive polling observes the returned by-id target and
  matching router FD. The USB serial, `ttyACM0` name and major/minor **166:0**
  are unchanged, but inode changes **1870 to 1896** and USB address changes
  **17 to 18**. This proves replacement-node reopening, not tty renumbering.
- **11:44:54:** the full post-replug command/ack, active/timeout and
  duplicate/older/invalid rejection check passes, sequence **506**, exit **0**.
  Subsequent passive samples still deliver ack **506** and increasing uptime.

**One physical USB power-cycle recovery passes without restarting the
candidate router.** The physical plug instant is not independently timestamped;
the first host presence poll occurs after traffic already resumes, so this
does not establish a five-second physical recovery deadline. There is no new
unmodified hardware control, repeated-loss matrix or ARM64 result. The fresh
boot's connected=1 is consistent with initial history synthesis, not evidence
that the known same-boot client reconnect-event defect is fixed. Rejecting
duplicate commands within this new boot does not prove replay protection
across power loss. Graceful router shutdown remains a separate failed gate.

Evidence is local and temporary:
`/tmp/picoros-candidate-physical-usb-20261016-TRXvR4/`; the pointer is
`/tmp/picoros-candidate-physical-usb-evidence-dir`. It contains baseline and
post-replug logs/exits, router logs/metadata/hashes, USB and FD snapshots,
timestamped CLI samples, device transitions and a physical-trial summary.
The initial detached host monitor ended before unplug; manual snapshots and
serial logs capture that phase. A managed passive monitor captures reappearance.
The five CLI observers require bounded cleanup after SIGINT does not finish
within 12 seconds; their timeout wrappers return **124**, recorded separately
from both successful full checks. This is not a graceful observer-exit claim.
The candidate remains running with last accepted sequence **506**; production
Compose and the original image tag remain unchanged. Further hardware cycles
require separate coordination.
