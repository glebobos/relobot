# `rmw_zenoh_pico` RP2040 spike

For the selected Pico-ROS USB test and next production-readiness gates, see
the [experiment handoff](../../wiki/picoros_zenoh_production_readiness.md).
The `rmw_zenoh_pico` firmware described first below is a comparison target,
not the selected image for flashing.

The retained [Pico-ROS USB roundtrip application](firmware_picoros/README.md) consumes the
[shared firmware foundation](../../pico_firmware/README.md): common platform
build/runtime setup, a separate KAmod board profile, and application-owned
ROS entities/control state. The comparison firmware below and production
micro-ROS firmware are not migrated. The shared-foundation UF2 is now flashed
and regression-tested on the spare, including one same-boot router recovery.

**Current build:** `./helpers/rmw_zenoh_pico_spike/build_picoros.sh` builds only
`picoros_usb_connectivity`, with the same settings and byte-identical UF2 used
in the hardware check. Earlier publisher/diagnostic/probe variants and one-off
LED/CDC/FreeRTOS probes have been retired. The chronology below is historical
evidence; its old variant commands are not current build/flash instructions.
Use the application README and handoff for the retained path.

This is an isolated compile-time feasibility spike. It does not replace or
modify any production Pico firmware.

The spike keeps the Micro-ROS `rcl`/`rclc` application API, removes
`rmw_microxrcedds`, and supplies:

- `rmw_zenoh_pico` at commit `54e20a460c9533d279e07a581d0e69e647d60a0e`;
- Zenoh-Pico 1.4.0 at commit `87ce955e651f699516b8b57b6a310f22bff527b8`;
- Pico SDK 2.2.0 at commit `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779`;
- FreeRTOS-Kernel 11.2.0 at commit `88971102bf8aa34b4fcc24f176db620dd9c854f7`;
- experimental Zenoh-Pico USB CDC serial transport.

## Build

From the repository root:

```bash
./helpers/rmw_zenoh_pico_spike/build.sh
```

The generated artifact is:

```text
helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.uf2
```

The pinned cross-build passed on 2026-10-15. The resulting RP2040 image has
183,292 bytes of text and 138,180 bytes of BSS; the UF2 is approximately 350
KiB. The ELF defines the expected Zenoh RMW entry points and contains no
`rmw_microxrcedds` or `uxr_` symbols.

The local patch completes the upstream RP2040 path by wiring USB serial client
configuration, FreeRTOS condition variables and clocks, RP2040 logging, GID
generation, release logging macros, and the actual endpoint QoS metadata. These
changes establish compile-time feasibility only; they are not yet evidence of
wire compatibility or reconnect reliability.

The current target is a non-actuating RP2040 `std_msgs/msg/Float32` publisher.
The code intentionally remains separate from the INA226 RP2350 firmware until
the RP2040 build, typed ROS interoperability, reconnect behavior, and memory
budget are proven.

## Hardware gate

If the spike firmware does not enumerate in application mode, build and flash
`build/firmware/usb_cdc_probe.uf2` to the spare test board instead. This probe
uses the same Zenoh-Pico USB CDC implementation and FreeRTOS configuration but
does not initialize ROS or open a Zenoh session. It enumerated on the spare
KAmod RP2040 in Windows and WSL as USB VID:PID `2e8a:000a`, with a stable
`/dev/serial/by-id` symlink. The first versions incorrectly drove GPIO25;
KAmod's onboard LED is a WS2812 connected to GPIO16. The corrected probe
shows blue before the scheduler, flashing red while waiting for CDC DTR, and
steady green after the port is opened.

To test the actual KAmod LED, build and flash `kamod_led_probe.uf2` to the same
spare board. This small, standalone Pico SDK/PIO firmware cycles red, green,
blue, and off every 500 ms. It does not run FreeRTOS or enumerate USB CDC;
BOOTSEL is required for the next flash. This LED test passed on the spare
KAmod RP2040. The corrected main spike uses magenta at startup, white immediately
before task creation, green after task creation but before the scheduler starts,
amber while `rcl_init_options_init()` runs, blue while
`rclc_support_init_with_options()` runs, white during node/session initialization, yellow
after node initialization, solid red if task creation fails or the scheduler
returns, and repeated red blinks (1-8) for ROS initialization errors.

`freertos_led_probe.uf2` isolates FreeRTOS from ROS, Zenoh and TinyUSB on the
KAmod board. It shows magenta briefly, blue for 500 ms, white for 500 ms, then green before
starting the scheduler. A running task flashes orange/off every 500 ms;
steady red means task creation failed or the scheduler returned. Like the
standalone LED test, it does not enumerate USB CDC.
This FreeRTOS-only probe passed on the spare KAmod after a complete
power-cycle. The 400-microsecond WS2812 latch interval is required for
reliable stage-color changes.

On hardware the official Zenoh-Pico `z_pub` example enumerated over USB CDC
and delivered increasing raw Zenoh samples through a temporary serial
router. The full RMW spike has not reached USB startup: the latest LED
diagnostic finished `rcl_init_options_init()` but stopped during
`rclc_support_init_with_options()`. The raw example is a transport proof,
not a substitute for ROS typed discovery.

## Pico-ROS + generated micro-ROS types (selected next experiment)

**Stage 2:** the separate inert `picoros_usb_bidirectional` target and its
CLI-only roundtrip/timeout/reconnect procedure are in the
[bidirectional experiment runbook](firmware_picoros/experiments/README.md).
Build it with `./helpers/rmw_zenoh_pico_spike/build_picoros.sh bidirectional`.
The default publisher-only target and production firmware are unchanged.

The alternative non-actuating test in `firmware_picoros/` uses **upstream
Pico-ROS** at `b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2` for ROS graph
declarations and Zenoh publication, with its pinned Zenoh-Pico fork
`0eb81272682a526b4bcf5a850c16ac74bbc6c73e`. It links the already
generated micro-ROS client `std_msgs/msg/Float32` C type, its RIHS type hash,
and its Micro-CDR serialization callback, but **no XRCE RMW or agent**.
Pico-ROS remains an external pinned checkout, not a copied or rewritten
protocol implementation. The existing Pico-ROS host proof in
`../picoros_poc/` validated its wire format against the Lyrical ROS router.

After the existing `./helpers/rmw_zenoh_pico_spike/build.sh` has produced
`build/libmicroros/` and `build/FreeRTOS-Kernel/`, build the new target with:

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh
```

This fetches Pico-ROS and Pico SDK 2.2.0 in ignored build directories, applies
the USB task signature fix, temporary initialization-stage diagnostics, and
RP2040 USB-disconnect error reporting to Pico-ROS's Zenoh-Pico submodule.
These patches do not change the ROS/Zenoh wire format. The cross-build uses
the existing wheels firmware builder image as the compiler source and
explicitly selects the pinned SDK checkout, not the image's floating SDK.
Do **not** rerun `build.sh` after building this target without rebuilding it:
`build.sh` regenerates `build/`.

```text
helpers/rmw_zenoh_pico_spike/build/firmware_picoros_sdk220/picoros_usb_spike.uf2
```

The cross-build and link passed; the RIHS bytes match Lyrical's `Float32`
hash, and the ELF has Pico-ROS and generated type symbols without XRCE
symbols. The patched diagnostic UF2 was flashed only to the spare KAmod
(flash ID `0xE464C498C3380C35`). Windows and WSL enumerated USB CDC as
`2e8a:000a`. An isolated router connected over USB, and Lyrical ROS 2
discovered `/picoros_usb_spike/value [std_msgs/msg/Float32]` with the expected
type hash and decoded increasing samples (`39.0`, `40.0`, `41.0`).

The first router-restart test exposed a fork USB transport defect: its read
loop waited forever after CDC disconnection and its write function always
reported success. The `zenoh-pico-usb-disconnect.patch` reports these errors
to Zenoh-Pico's existing auto-reconnect machinery. With the patched UF2,
router shutdown caused CDC to detach and re-enumerate; after manually
reattaching the device to WSL and restarting the router, ROS discovery and
increasing samples (`91.0`, `92.0`) resumed **without resetting the board**.
Windows-to-WSL USB reattachment is a test-host operation, not automatic MCU
recovery. The robot still needs automatic USB device access/rebinding,
repeated disconnects, latency, and long-running stability tests before
production migration.

One physical USB unplug/replug with only `/dev/serial/by-id` and a fixed
`--device=/dev/ttyACM0` mapped into the router left the process running,
but the typed topic did not return. WSL had recreated the by-id directory
and the container still saw the old, empty mount. In a second test, mounting
the stable parent `/dev:/hostdev:ro` with an ACM-only device cgroup rule let
Zenoh's existing serial listener retry and reopen the board after hotplug:
the same router process published increasing ROS values (`13.0`, `14.0`)
again without a Pico reset. WSL USB reattachment was manual, and that test
reused ttyACM0. Stage 2 has since passed one clean-session change from
ttyACM1 to ttyACM0, but repeated-loss recovery still fails; see its runbook
below. The main router service now exposes
this mount and accepts an optional `ZENOH_ROUTER_CONFIG_URI`; no serial
endpoint is enabled by default.

On a **full power-cycle**, yellow means Pico-ROS entry, cyan means entering
Zenoh `z_open()`, red means entering USB TinyUSB initialization, magenta
means TinyUSB initialization returned, orange means the TinyUSB task was
requested, white means the Zenoh session opened, and green means the
publisher was declared. These temporary hooks are diagnostic only.

The MCU connects with:

```text
serial/usb#baudrate=115200
```

The host router must listen on the board's stable `/dev/serial/by-id/...` path
and have access to the current tty target. For the active Pico-ROS target,
verify `/picoros_usb_spike/value [std_msgs/msg/Float32]`; repeat router
restart and USB disconnect tests before treating reconnect as reliable.

For the current typed bidirectional experiment and clean-session renumber
evidence, see the [stage 2 runbook](firmware_picoros/experiments/README.md).
Its separate `build_picoros.sh diagnostics` variant adds a genuine task
heartbeat and application-progress indication without changing the serial
protocol or recovery implementation. It was flashed to the verified spare
and its full Docker ROS CLI baseline passed on 2026-10-16 at 08:46:22 UTC
(sequence 301); the user confirmed orange heartbeat once per second.
Its first controlled router-only restart did not recover ack traffic in
the recorded windows, including a later unfiltered observer, while the
user confirmed continued orange heartbeat. The separate `transport-probe`
variant was flashed and verified on the same spare at 09:08 UTC; its full
Docker ROS CLI baseline passed at **09:09:19 UTC**, sequence **401**.
It instruments shared-executor and USB-read progress to test the
blocking-RX hypothesis. The user confirmed orange baseline heartbeat.
Its first controlled router-only restart at **09:12:55 UTC** also produced
no ack in bounded observations through **09:14:23 UTC**, over 80 seconds
after the new process started. USB and the current serial descriptor
remained present. The user subsequently reported light-blue/turquoise
blinking once per second, consistent with the cyan USB-read/executor marker.
It supports that mechanism, not a captured MCU stack or recovery result.
The separate **`build_picoros.sh serial-poll`** candidate now builds with
incremental async serial RX, a nonwaiting USB try-read and 5 ms read-future
scheduling. Native tests (4/4 plus ten repeats), three ASan checks and ELF
wiring verification pass. Its UF2 is in the separate
`build/firmware_picoros_usb_poll_sdk220/` directory, **not yet flashed**;
existing rollback artifacts are unchanged. Handshake, TX and USB lifecycle
remain unchanged, so recovery is still unproven. Physical BOOTSEL is next.
See the runbook and
[host mechanism experiment](host_tests/README.md) for evidence and limits.

The next opt-in variant is **`build_picoros.sh connectivity`**, target
`picoros_usb_connectivity`, in its own
`build/firmware_picoros_connectivity_sdk220/` directory. It retains the
polling experiment and adds the built-in Zenoh transport-events listener
through a small guarded Pico-ROS session accessor. Three ROS telemetry
topics expose last connection state and open/close counts; no new heartbeat,
lease/reconnect logic or USB reset loop is introduced. Default configurations
keep connectivity disabled and existing UF2s are not overwritten. This is
not a health/command-delivery guarantee or a hardware recovery result; see
the [connectivity procedure](firmware_picoros/experiments/README.md#built-in-zenoh-connectivity-candidate).
This variant was flashed/verified on the spare at **10:09:01–10:09:04 UTC**
on 2026-10-16, and its full sequence-501 CLI baseline passed at **10:10:13
UTC** with initial connected=1/opened=1/closed=0. Native CTest 5/5, ten
repeats, 5/5 ASan/leak checks and the command guard pass. Its first
router-only restart still **fails**: after USB re-enumerated to `ttyACM1`,
the router child retained `/hostdev/ttyACM0 (deleted)`, with no recovered
status in the 100-second observation. No command 502 or manual reset/
reattach followed. Host accept-phase serial reopening is the next blocker;
the built-in observer is not a recovery fix or production-readiness claim.
