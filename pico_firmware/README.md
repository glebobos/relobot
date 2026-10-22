# Shared Pico-ROS firmware foundation

This directory supplies common **source and build rules**, not one universal
firmware or a new ROS middleware. Each application links its own ELF/UF2 from
the shared platform, one board profile and its application sources. This is the
foundation for new Pico-ROS applications; do not copy its SDK/RTOS/USB setup into
each controller. The first retained consumer is the inert
[USB roundtrip application](../helpers/rmw_zenoh_pico_spike/firmware_picoros/README.md).
Production micro-ROS firmware directories are not migrated automatically.

## Boundaries

| Part | Owns | Does not own |
| --- | --- | --- |
| [CMake modules](cmake/) | SDK/FreeRTOS imports, Pico-ROS/Zenoh features, generated message library linkage, UF2 outputs | Application topics, drivers or control policy |
| [Configuration](config/) | Single-core FreeRTOS and TinyUSB settings | Per-motor limits or transport health inference |
| [Runtime](src/picoros_runtime.c) | Initial Pico-ROS client setup on USB serial | Nodes, publishers, subscribers or post-start reconnect policy |
| [USB serial support](src/usb_serial/) | Cooperative receive buffering and pinned Zenoh-Pico link/executor bindings | ROS discovery, message schemas or a reconnect supervisor |
| [KAmod profile](boards/kamod_rp2040/) | MCU/SDK board selection, GPIO16 WS2812 pin, PIO driver and board startup | Meaning of LED colors or command state |
| Application | ROS entities, generated codecs, tasks/priorities, callbacks, command state and local timeouts | USB framing, Zenoh protocol or board-specific LED registers |

The common configuration initially matches the earlier experiment's settings:
one core, 128 KiB FreeRTOS heap, unchanged stacks/hooks, CDC buffers and Zenoh
lease. Resource instrumentation, SMP and production safety changes are separate
work. The old helper configuration/LED files remain only for the independent
`rmw_zenoh_pico` comparison; its one-off hardware probes have been removed.
Pico-ROS no longer includes that comparison's configuration or LED driver.

Implemented profiles are `kamod_rp2040`, `xiao_rp2040`, `pico` (all RP2040),
and `xiao_rp2350` (Seeed Studio XIAO RP2350, `PICO_BOARD=seeed_xiao_rp2350`,
`PICO_PLATFORM=rp2350-arm-s`). The `xiao_rp2350` profile is
cross-build-verified only (see
[`pico_ware_ina226_picoros/README.md`](../pico_ware_ina226_picoros/README.md));
it has not been flashed or validated on physical RP2350 hardware. Other
unsupported profiles and conflicting `PICO_BOARD`/`PICO_PLATFORM` settings
still fail at configuration rather than silently reusing RP2040 pins or
transport code. Adding another board requires its own profile and driver
implementation.

## Application API

- [`relobot_board_init()`](boards/kamod_rp2040/board.c) performs the existing
  stdio/LED initialization before application tasks start.
- [`relobot_board_led_set()`](include/relobot/board.h) accepts RGB values;
  color meanings and task ownership remain application decisions.
- [`relobot_picoros_init()`](include/relobot/picoros_runtime.h) runs inside
  an application FreeRTOS task, initializes the native Pico-ROS client at
  `serial/usb#baudrate=115200`, and returns a native `picoros_res_t` result.

The runtime retains the existing **startup-only** `PICOROS_NOT_READY`/one-second
wait. It can wait indefinitely for initial
availability, as before. Once successful it returns; subsequent reconnection
remains Zenoh-Pico's responsibility. There is no new supervisor, heartbeat,
watchdog or retry policy, and no replacement ROS discovery/CDR implementation.
Applications continue to call native `picoros_node_init`, declaration and
publication APIs and supply the pinned diagnostic-stage callback.

`RELOBOT_PICOROS_USB_POLL=ON` is the shared default for the prepared, pinned USB
transport. `relobot_add_firmware()` links the existing cooperative receive
implementation automatically; applications must not copy its linker wrappers.
It buffers partial frames and calls the dependency's serial codec/CRC. Initial
handshake, writes, reconnect and ROS protocol handling remain in Pico-ROS/Zenoh.
It supports the tested single USB link/executor, not multiple links or SMP.
The existing dependency USB patches are still required; this is not a claim
that pristine upstream firmware passed these experiments.

`RELOBOT_PICOROS_CONNECTIVITY=OFF` is the default for new applications. The
validation app enables native transport-event diagnostics explicitly. Those
events have a known same-boot reopen-notification defect: do not use their
`connected` flag as an authoritative health or actuator-enable gate.

## Build integration

An application includes the imports **before** `project()` and the shared
platform afterwards, then registers its own source files:

```cmake
cmake_minimum_required(VERSION 3.13)
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")
project(my_controller C CXX ASM)
include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")
relobot_add_firmware(my_controller src/main.c)
```

Supply `PICOROS_SOURCE`, `RELOBOT_ROS_TYPES_PATH`, `PICO_SDK_PATH` and
`FREERTOS_KERNEL_PATH` from the prepared dependency set. The common modules
contain no path back into a particular helper/application. The existing
[Docker build wrapper](../helpers/rmw_zenoh_pico_spike/build_picoros.sh)
remains the single dependency-pin/patch preparation entry point. It now builds
only the retained USB roundtrip application, by default or with `connectivity`;
earlier publisher/diagnostic/probe variants are retired, not silently aliased.
No dependency version or upstream patch changes in this cleanup.
Do not create per-board copies of that dependency setup.

`relobot_add_firmware()` links the shared runtime, selected board and generated
ROS support and disables USB/UART stdio, preserving the transport's ownership
of USB. It generates application-named ELF/BIN/HEX/UF2 outputs. One CMake
configuration has one board/platform/transport feature set; use separate build
directories for different settings. Application source directories and topic
definitions remain outside this shared directory.

## Validation, 2026-10-16

- Before prototype retirement, all six targets cross-built with polling and
  connectivity enabled; all four default targets also built with both disabled.
- Four stubbed native runtime tests pass, also under ASan/UBSan/leak checking;
  they cover immediate readiness, startup waits and native error propagation,
  not real serial reconnect or ROS interoperability.
- The existing command-state test passes. Two negative configuration checks
  reject unknown hardware and an RP2350/platform mismatch before SDK startup.
- After retirement, the retained target rebuilds in its canonical
  `build/firmware_picoros_connectivity_sdk220/` directory. Its UF2 is byte-identical
  to the hardware-tested shared-foundation image. Runtime tests pass 4/4,
  command-state tests 1/1 and native serial/connectivity tests 6/6. One native
  test deliberately reproduces the known missing-reopen-event defect; passing
  that reproducer does not mean native health reporting is correct.
- The real build wrapper also succeeds. An independent, compile-only consumer
  builds with shared defaults (USB polling ON, native connectivity OFF), without
  importing this validation application's CMake rules. Both invalid-board
  guards and the one-definition runtime/board/transport symbol audit pass.

To rerun only the runtime tests, from the repository root:

```bash
docker run --rm --network none --entrypoint bash \
  -v "$PWD:/project" relobot_pico_ware_wheels_microros:latest -lc '
    cmake -S /project/pico_firmware/tests -B /tmp/platform-runtime-tests &&
    cmake --build /tmp/platform-runtime-tests -j2 &&
    ctest --test-dir /tmp/platform-runtime-tests --output-on-failure
  '
```

Build/unit-test containers receive no device mounts. Separately, the verified
spare KAmod was flashed with verification at **12:10:41–12:10:44 UTC**. The full
stock Docker ROS CLI roundtrip/timeout/duplicate/older/invalid check passed with
sequence **601**, then **602** after one bounded candidate-router restart.
Ack **601** survived the outage; uptime increased from **84.398 s to 95.004 s**,
so recovery did not require an MCU reboot. No new reconnect layer was added.

Hardware-tested and canonical UF2 SHA-256:
`d5c50234f849babed39d267c6e03ae4748d78f41f40467b8e7274b7b43cd1edb`.
See the [handoff](../wiki/picoros_zenoh_production_readiness.md#shared-foundation-hardware-regression-2026-10-16)
for exact scope, router image and remaining limitations. This is one regression
pass on a spare RP2040, not production qualification for every Pico model.

Obsolete publisher-only, LED/CDC/FreeRTOS probe sources and unselected Pico-ROS
build variants are retired. Regression-test sources, diagnostic evidence,
dependency patches and the rollback image are retained. Temporary refactor/test
build directories are removed only after the canonical UF2 matches the flashed
image: **20 generated trees, 728.5 MiB** were removed; dependencies and the
canonical current build remain. Raw logs and rollback files are local under
`/tmp/picoros-foundation-hardware-20261016-gHQpcv/`, with pointer
`/tmp/picoros-foundation-hardware-evidence-dir`.
