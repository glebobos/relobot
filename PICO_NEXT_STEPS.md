# Pico-ROS: continuation plan

Date fixed: **October 16, 2026**. Work paused.
This file is a continuation point, not permission for new hardware actions.

## What is already done

- [x] Extracted the [common firmware foundation](pico_firmware/README.md): Pico SDK,
  FreeRTOS, USB/Zenoh, Pico-ROS startup and general build rules.
- [x] Board description, pins and drivers are separated from application logic.
  Only the spare KAmod RP2040 is supported and hardware-verified.
- [x] One [verification application](helpers/rmw_zenoh_pico_spike/firmware_picoros/README.md)
  with bidirectional exchange via standard ROS CLI was kept. There are no actuators in it.
- [x] The new foundation was flashed onto the spare board. Checks of commands/acknowledgments,
  local timeout, repeated/stale sequence numbers, and incorrect values passed.
- [x] One restart of the experimental router was performed: the exchange recovered
  without MCU reboot, reflashing, or manual USB reconnection.
- [x] Outdated application variants, one-off probes, and temporary
  build trees were removed. Necessary patches, regression tests, and history were preserved.
- [x] After cleanup, the build passes; the canonical UF2 byte-for-byte matches
  the flashed and verified image.

The common foundation means **shared sources and build, not a single UF2 for all boards**.
The working controllers `pico_ware_*` have not been migrated and are not affected.
The new reconnect mechanism and a custom ROS protocol have not been added.

## Where to start the next session

1. Read the [repository rules](AGENTS.md), this plan, and the
   [current results](wiki/picoros_zenoh_production_readiness.md#shared-foundation-hardware-regression-2026-10-16).
2. Without restarting, check the state of the router container, its image/config,
   the presence of the required USB by-id, and whether the board matches the saved identifier.
3. If the board is connected, read the uptime, current ack, and command state
   using standard ROS commands inside Docker. The last recorded ack was **602**;
   after the break it cannot be considered current without reading it.
4. Agree on the next experiment. Proposal: measure resources and timings
   of the existing single-core firmware, without changing the architecture or reconnect right away.

## Proposed next stages

### 1. Single-core baseline

- [x] Agree on load, duration, and measurement criteria.
- [x] Add minimal diagnostics for free/min-ever heap and task stack
  high-water marks; do not create a new health/reconnect protocol.
- [x] Measure command → ack latency, telemetry stability, and local timeout
  execution under agreed load.
- [x] Separately account for ELF/BSS size and actually measured resources:
  static heap size is not equal to free memory during operation.
- [x] After changing the firmware, perform a short regression on the spare
  board and record parameters, image SHA-256, and results in the wiki.

Stage outcome: reproducible baseline established and documented in
[wiki/picoros_zenoh_production_readiness.md](wiki/picoros_zenoh_production_readiness.md#single-core-baseline-and-resource-measurement-2026-10-16).
Peak heap usage is 113.9 KiB (86.9% of 128 KiB heap; 17.1 KiB min-ever headroom).
`app_task` uses only 4.8 KiB of 32 KiB allocated stack (27.9 KiB headroom).
Mean RTT is 63.88 ms (p95 109.87 ms, 100% command acceptance, 0 telemetry errors).

### 2. A separate dual-core experiment — only if necessary

- [x] Based on the baseline results, decide whether SMP is needed at all.
  **Decision (2026-10-16)**: SMP is **not needed**. The single-core baseline demonstrated
  100% command delivery under continuous 10 Hz load, low latency (mean ~64 ms),
  minimal stack utilization (4.8 KiB of 32 KiB), and vast idle margin. Dual-core
  SMP would introduce locking and race risks in USB CDC buffers without performance gain.
- [x] If needed, separately agree on the experiment: **Omitted / Skipped** by architectural decision.
- [x] Compare with the single-core variant: Single-core is the chosen standard for RP2040 Pico-ROS nodes.

### 3. Preparing the first application controller

- [ ] Choose the first controller together with the user and describe its ROS types,
  QoS, rates, timing requirements, and behavior on lost/stale commands.
- [ ] Add only the necessary board profile, drivers, and application,
  reusing the common platform without copying SDK/USB/Zenoh settings.
- [ ] For actuators, separately define a local safe stop, E-stop,
  post-reboot behavior, and protection against stale commands; verify firmware rollback.
- [ ] Migrate controllers one at a time, not replacing all working firmware at once.

Historical proposal: knives with the motor physically powered off, then IMU and wheels;
INA226/RP2350 is a separate port. This is **not permission**
to connect motors or flash working boards; the order needs to be confirmed.

## What remains deferred, not passed

- Repeated USB/router disconnection matrix and a strict recovery time limit.
- Correctness of built-in native health events after reconnect.
- Correct graceful shutdown of the router: the verified stop ended
  forcefully after 5 seconds, exit code 137.
- Checks on the target Raspberry Pi/ARM64 and production deployment.

These checks were previously excluded from the current stage by the user's decision.
Do not bring them back automatically in the next session; before production, separately
agree on closing the corresponding risks. The old "Next execution
order" list at the bottom of the readiness document does not override this decision.

**Known defect:** after the exchange recovers, native diagnostics may
show `connected=0, opened=1, closed=1`. Traffic works, but reopening is not
reflected correctly. Do not use `connected` as the sole
condition for health or actuator permission. A test reproducer for this
defect does not mean the health gate has passed.

## Build and reference image

From the repository root, via the existing Docker wrapper:

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh
```

Target: `picoros_usb_connectivity`.

UF2: [picoros_usb_connectivity.uf2](helpers/rmw_zenoh_pico_spike/build/firmware_picoros_connectivity_sdk220/picoros_usb_connectivity.uf2).

SHA-256 of the verified image (single-core instrumented baseline):
`608a255f4671b22c16566c82803ac18866ba490fd319b3f4280a65e5e2d120c9`
(pre-instrumentation foundation image was `d5c50234f849babed39d267c6e03ae4748d78f41f40467b8e7274b7b43cd1edb`).

Board: flash ID `0xE464C498C3380C35`, USB by-id
`usb-Raspberry_Pi_Pico_E464C498C3380C35-if00`. Do not pin the `ttyACM*` number.
Before reflashing, verify identity again; BOOTSEL — with user involvement.
If the image hasn't changed, reflashing just to continue is not needed.

Do not run the old `build.sh` unnecessarily: it recreates the helper build
and removes Pico-ROS results. Details — in the application README.

## What to preserve during the pause

- As of the last check, `ros2_zenoh_router` is running with the image
  `relobot-zenoh-serial-accept:experimental`; the original production tag has not been replaced.
- The experimental run uses a temporary override and the configuration
  `/tmp/picoros-bidirectional-router.json5`. A regular Compose recreate/up may
  revert to the stock image; do not run it blindly.
- Device connection: `/dev:/hostdev:ro` and cgroup `c 166:* rwm`.
  Verify by-id resolution inside the container; a stable name does not preserve
  an open file descriptor after reconnection.
- Local evidence and rollback are located at
  `/tmp/picoros-foundation-hardware-20261016-gHQpcv/`; the pointer is
  `/tmp/picoros-foundation-hardware-evidence-dir`. This is temporary storage,
  not a durable archive. Before clearing `/tmp`/rebooting, save the needed
  images and logs to an agreed permanent location.
- During the pause, do not perform new flash/restart/USB tests and do not delete
  temporary files that the current experimental run depends on.

## Permanent constraints and links

All builds, tests, and ROS commands run inside Docker; the host only orchestrates.
For hardware checks use standard `ros2 topic pub/echo`, only the
spare KAmod, without parallel tty reads or arbitrary reset bytes.
Do not add a custom reconnect-supervisor and do not duplicate the ROS protocol.

- [Common platform](pico_firmware/README.md).
- [Saved application and its checks](helpers/rmw_zenoh_pico_spike/firmware_picoros/README.md).
- [Results and open limitations](wiki/picoros_zenoh_production_readiness.md).
- [Hardware procedures](wiki/rmw_zenoh_pico_robot_validation.md).
- [Migration plan](wiki/picoros_zenoh_migration_plan.md) — an architectural reference point,
  not automatic permission for the next migration.