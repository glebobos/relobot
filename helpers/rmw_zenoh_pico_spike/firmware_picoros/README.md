# Retained Pico-ROS USB validation application

This is the one retained, non-actuating application for the
[shared firmware foundation](../../../pico_firmware/README.md). It uses the
native Pico-ROS API, generated ROSIDL/Micro-CDR support and the common
KAmod/FreeRTOS/USB setup. Its historical target name remains
`picoros_usb_connectivity` so the tested UF2 and build path stay identifiable.
It is not a universal controller application or a production command protocol.

## Build and validate

Run from the repository root; compilation and ROS CLI execution stay in Docker:

```bash
./helpers/rmw_zenoh_pico_spike/build_picoros.sh
# Equivalent explicit selection:
./helpers/rmw_zenoh_pico_spike/build_picoros.sh connectivity
```

Output: `helpers/rmw_zenoh_pico_spike/build/firmware_picoros_connectivity_sdk220/picoros_usb_connectivity.uf2`.
Dependency preparation remains centralized in that wrapper. If generated
message support is absent, follow the [handoff](../../../wiki/picoros_zenoh_production_readiness.md)
before regenerating it: the legacy `build.sh` replaces the helper's build tree.

Only the spare KAmod with flash ID **`0xE464C498C3380C35`** is authorized for
this validation. Follow the handoff's identity/BOOTSEL/verify procedure; never
select a production controller by a transient `ttyACM*` name. Do not open the
serial port alongside the router. The current candidate router uses a patched
image; ordinary Compose recreation can revert it to stock.

Once the spare and isolated router are ready, choose a fresh sequence greater
than the currently published `/picoros_usb_spike/ack`:

```bash
bash helpers/rmw_zenoh_pico_spike/check_bidirectional.sh <fresh-sequence>
```

That script uses stock `ros2 topic` commands inside the router container. It
checks typed commands/ack, local timeout, duplicate/older rejection and an
invalid value. It does not prove command age, cross-boot replay protection or
actuator safety. The latest verified sequence in the recorded trial is **602**;
always re-read the live ack rather than relying on that historical number.

## Ownership

- [main.c](main.c): native ROS entities, application tasks, command timeout and
  LED meanings. No actuators are initialized or driven.
- [common/](common/): generated-type codec adapter, inert command-state probe
  and optional native connectivity observation.
- [diagnostics/](diagnostics/): executor/USB read observation used by this
  validation app, not a reconnect policy or a required controller driver.
- [tests/](tests/): retained command-state regression tests.
- [Shared USB receive](../../../pico_firmware/src/usb_serial/): the used transport
  code, no longer copied into or owned by an experimental application.

New board applications include the shared CMake modules and supply only board
profiles/drivers and their actual ROS/control logic. Do not copy the inert
Float32 sequence convention or these diagnostics into motor control.

The [historical experiments](experiments/README.md) remain evidence, not a menu
of supported build variants. Publisher-only, diagnostics-only, transport-probe
and serial-poll-only variants and the one-off LED/CDC/FreeRTOS firmware probes
have been removed. Unit/native regression tests and required USB/router patches
remain reproducible. Existing production firmwares are unchanged.
