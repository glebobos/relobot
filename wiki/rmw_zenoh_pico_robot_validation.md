#HISTORICAL DOC, DO NOT USE TARGET. Just some what we did/

# ReloBot `rmw_zenoh_pico` Robot Validation Runbook

**For the selected Pico-ROS firmware and current production-readiness
checklist, use the [Pico-ROS USB handoff](./picoros_zenoh_production_readiness.md)
instead.** This guide retains the historical `rmw_zenoh_pico` comparison
target; its build path, UF2, topic name and several procedures do **not**
apply to Pico-ROS. Do not flash its old UF2 during the Pico-ROS experiment.

This document is an execution guide for the next agent or operator. It covers
the first physical USB test of the isolated RP2040 Zenoh spike and the gates
required before any production firmware migration.

**Current selected experiment:** the separate Pico-ROS + generated micro-ROS
type-support target described in
[`helpers/rmw_zenoh_pico_spike/README.md`](../helpers/rmw_zenoh_pico_spike/README.md).
Sections 3-6 below still document the older `rmw_zenoh_pico` UF2 and topic
name; do not use its firmware path or expected topic for the new prototype.
The updated UF2 passed the typed ROS hardware gate on the verified spare
KAmod: Windows and WSL enumerated `2e8a:000a`, Lyrical ROS 2 discovered
`/picoros_usb_spike/value [std_msgs/msg/Float32]` with the expected RIHS hash
and decoded increasing values. The first router restart **did not**
recover: the pinned Zenoh-Pico fork blocked on USB read after DTR dropped
and reported successful writes despite lost CDC. The separate
`zenoh-pico-usb-disconnect.patch` now reports USB disconnect to the fork's
existing auto-reconnect path. With this patch flashed, router shutdown
caused CDC to re-enumerate; after manually reattaching USB to WSL and
restarting the router, ROS graph and values returned without a Pico reset.
This is **one recovery**, not proof of automatic WSL/robot device rebinding
or the required reconnect/soak gates. On a full power-cycle: yellow =
Pico-ROS entry; cyan = before `z_open`; red = before TinyUSB init; magenta =
after TinyUSB init; orange = after USB task creation request; white = after
`z_open`; green = publisher declared. An isolated red **steady** stage must
not be confused with the repeated red-blink error indicator.
With the patched UF2, one physical unplug/replug while the router used a
bind mount of `/dev/serial/by-id` and a fixed `/dev/ttyACM0` device produced
`early eof`; the process survived, but the topic did **not** reappear.
WSL recreated the by-id directory after reattachment; the container's old
mount remained empty. A second unplug/replug with the stable parent
`/dev:/hostdev:ro` mounted and cgroup access to ACM devices **did** restore
the typed topic and increasing `Float32` values (`13.0`, `14.0`) in the
same router process without resetting the Pico. WSL USB reattachment was
manual, and the new tty was again ttyACM0: different-minor renumbering,
repeated recovery and production fail-safes remain unproven.
Before any spare-board flash, inspect the BOOTSEL flash ID and require
`0xE464C498C3380C35`; use the same `picotool` procedure in section 4 with
`build/firmware_picoros_sdk220/picoros_usb_spike.uf2` instead. The new typed
topic to test is `/picoros_usb_spike/value [std_msgs/msg/Float32]`.

> **Safety rule:** the spike is a non-actuating `Float32` publisher. Do not
> flash it to the wheels, knives, IMU, or INA226 boards. Do not replace a
> production UF2 until the rollback copy has been saved and all gates in this
> document pass.

## 1. Current implementation and evidence

The isolated target is
[`helpers/rmw_zenoh_pico_spike/`](../helpers/rmw_zenoh_pico_spike/). It keeps
the existing `rclc` application shape and replaces the Micro-XRCE-DDS RMW
with `rmw_zenoh_pico` and Zenoh-Pico USB CDC transport.

Pinned components:

| Component | Revision |
| --- | --- |
| `rmw_zenoh_pico` | `54e20a460c9533d279e07a581d0e69e647d60a0e` |
| Zenoh-Pico | `87ce955e651f699516b8b57b6a310f22bff527b8` (1.4.0) |
| Pico SDK | `a1438dff1d38bd9c65dbd693f0e5db4b9ae91779` (2.2.0) |
| FreeRTOS-Kernel | `88971102bf8aa34b4fcc24f176db620dd9c854f7` (11.2.0) |

The cross-build passed on 2026-10-15:

```text
helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.uf2
```

The generated ELF contains the expected `rmw_*` and Zenoh symbols, has no
`rmw_microxrcedds` or `uxr_` symbols, and has no undefined symbols. Measured
static footprint:

```text
text: 183292 bytes
bss:  138180 bytes
UF2:  approximately 350 KiB
```

This is compile/link evidence only. It does not prove ROS graph compatibility,
USB reconnect behavior, timing, or safe recovery.

On the spare KAmod RP2040, the spike UF2 flashed successfully and its BOOTSEL
metadata was readable (flash ID `0xE464C498C3380C35`). After reboot, the
spike did not enumerate in application mode on the Windows host. Earlier
interpretations of the dark LED were invalid: KAmod's onboard RGB LED is a
WS2812 at GPIO16, while the spike drives GPIO25 as if it were a Pico board.
Do not infer the startup stage from that old firmware's LED. A standalone
GPIO16 WS2812 test successfully cycled red, green, blue and off on the spare
board. The newer images drive GPIO16 correctly but have not yet been flashed.

### USB-only diagnostic gate

The full build also produces
`helpers/rmw_zenoh_pico_spike/build/firmware/usb_cdc_probe.uf2`. It uses
Zenoh-Pico's USB CDC device code and the same FreeRTOS/TinyUSB configuration,
but never calls `rclc` or opens a Zenoh session. Flash it **only to the
confirmed spare board**, following the `picotool info -a` identity check and
load procedure in section 4 with `usb_cdc_probe.uf2` as the filename.

- The probe enumerated in Windows and WSL as USB VID:PID `2e8a:000a`,
  `/dev/ttyACM0`, with by-id serial `E464C498C3380C35`.
- Opening the CDC device succeeded. This proves the board, its USB wiring,
  WSL attachment and isolated Zenoh-Pico TinyUSB stack can enumerate.
- The original GPIO25 output and dark LED were not diagnostics on KAmod.
  `kamod_led_probe.uf2` proved GPIO16 works. In the rebuilt USB probe, blue
  means startup, flashing red means CDC waiting for DTR, and green means DTR.
- The first GPIO16-enabled main spike was flashed to the confirmed spare
  board and stayed blue. Its app task had not reached the amber stage;
  application-mode USB was absent. This does not yet distinguish task
  creation failure from a scheduler startup problem.
- A second GPIO16-enabled main spike remained blue even after task-creation
  success and failure indicators were added; Windows did not see a USB
  device. Before attributing this to task creation, note that the LED helper
  calls `sleep_us(80)` just after setting blue and before `xTaskCreate()`.
  Pico SDK can route that call through an alarm/sleep path. The next test
  replaces this WS2812 reset delay with `busy_wait_us(80)` and adds a white
  indicator immediately before task creation; it also uses one FreeRTOS
  core to isolate the SMP variable.
- The [official RP2040 FreeRTOS port](https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/main/portable/ThirdParty/GCC/RP2040)
  supports both one-core and SMP use. The
  [official RP2040 SMP demo](https://www.freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/Raspberry-Pi/smp-demos-for-the-raspberry-pi-pico-board)
  explicitly warns that the SMP kernel used for its examples is still being
  tested; two cores are not a requirement for the spike. The
  [smittytone RP2040 template](https://github.com/smittytone/RP2040-FreeRTOS)
  confirms a single-core Pico SDK 2.2.0 workflow and that `xTaskCreate()`
  stack depth is in words, but uses the older `portable/GCC/ARM_CM0` port
  and `heap_3`, not this spike's RP2040 SMP-capable port and `heap_4`.
  It also disables `configASSERT`; do not copy its config wholesale.
  The [AWS Pico FreeRTOS samples](https://github.com/aws-iot-builder-tools/freertos-pi-pico)
  are closer to this build in that their
  [CMake target](https://github.com/aws-iot-builder-tools/freertos-pi-pico/blob/main/app/CMakeLists.txt)
  links the RP2040 port with `FreeRTOS-Kernel-Heap4`. They demonstrate
  tasks on both cores, but target the older FreeRTOS `smp` branch, use
  `configNUM_CORES` rather than the pinned kernel's
  `configNUMBER_OF_CORES`, and their last commit was in January 2023.
  They do not diagnose our current startup failure or justify replacing
  the pinned config.
  RP2040 has no hardware FPU, but the pinned RP2040 FreeRTOS port does not
  reference `configENABLE_FPU`, so a web-search claim that this flag causes
  the observed failure is unproven.
- In the latest diagnostic main spike, magenta marks startup, white before
  task creation (or during node/session init), green means task creation
  succeeded but the app task has not begun, amber means RCL init-options
  init, blue means support init with options, and yellow means node
  initialized. Solid red means task creation failed or the scheduler
  returned; repeated red blink groups (1-8) identify ROS initialization
  errors.
- The one-core spike also stayed blue, even after its WS2812 latch delay
  changed from `sleep_us` to `busy_wait_us`. This does not prove any FreeRTOS
  defect: isolate the scheduler from ROS/Zenoh first. The next artifact,
  `freertos_led_probe.uf2`, links the same pinned FreeRTOS heap and the
  verified GPIO16 LED helper but not ROS, Zenoh, or TinyUSB. It should show
  magenta briefly, blue for 500 ms, white for 500 ms, green briefly, then orange/off blinking
  from its task. Solid red indicates creation/scheduler failure. It has no
  application-mode CDC USB device; reflash through BOOTSEL.
- The first FreeRTOS-only image also showed blue, but this is **not evidence
  of a stopped timer**: the WS2812 helper only waited 80-120 microseconds
  starting when the frame was queued to PIO, leaving less than that after
  its approximately 30-microsecond transmission. Some WS2812 variants need
  at least 280 microseconds of idle-low time to latch. WS2812 retains its
  previous color across resets while power stays connected. The corrected
  helper waits 400 microseconds after queueing each frame, and the probe
  starts with magenta. Power-cycle the board before interpreting its colors.
  Do not conclude that FreeRTOS, SMP or the hardware timer caused the old
  blue readings. If the corrected probe remains ambiguous, use SWD to halt
  the CPU and read its PC rather than repeatedly guessing from colors.
- The corrected FreeRTOS-only probe **passed on the spare KAmod after a
  power-cycle**: magenta, blue, white, green, then repeating orange/off from
  its FreeRTOS task. Single-core task creation and scheduler startup work.
  The old blue observation was not a valid localization of the ROS failure.
  After the corrected main spike was flashed, its LED became orange or yellow
  and Windows did not see USB CDC; it got past task startup but apparently
  stopped before Zenoh USB initialization. The next diagnostic separates
  `rcl_init_options_init` (orange) from
  `rclc_support_init_with_options` (blue), preserving their usual call order.
  Do not keep changing RTOS settings on speculation.
- The split RCL diagnostic reached blue: `rcl_init_options_init()` returned,
  but `rclc_support_init_with_options()` did not return, and Windows had no
  application USB device. The exact failure inside `rcl_init()`/clock setup
  has not been localized. This is not evidence that USB CDC or Zenoh-Pico
  itself is broken.
- As an independent control, the official
  [Zenoh-Pico RP2040 `z_pub` example](https://github.com/eclipse-zenoh/zenoh-pico/blob/1.4.0/examples/rpi_pico/z_pub.c)
  was built for `serial/usb#baudrate=115200` and flashed to the same verified
  spare board. Windows and WSL enumerated it as `2e8a:000a`, with serial
  `E464C498C3380C35`. A temporary `rmw_zenohd` router listened on that
  `/dev/ttyACM0` and loopback TCP. A Zenoh 1.4.0 subscriber received
  sequential values `[45]` through `[63]` on
  `demo/example/zenoh-pico-pub`. The temporary router was stopped afterward.
  This **proves real Zenoh publication over USB** but not ROS typed discovery:
  the example publishes a raw Zenoh key, without ROS CDR type/hash or RMW
  graph metadata. No production devices or router configuration changed.
- The original spike still has not enumerated; investigate its startup path,
  heap/stack usage and ROS initialization before router testing.

Check Windows Device Manager or `usbipd list` for the application-mode device
*before* trying to attach it to WSL. If the probe enumerates but the spike
does not, investigate the spike's `rclc_support_init()` path and heap/stack
usage; do not change the router yet.

## 2. Hardware inventory before touching firmware

Stop if the board inventory is ambiguous. The host currently has multiple Pico
boards connected over USB, and `/dev/ttyACM*` numbering is not a board identity.

Run from the repository root:

```bash
./run_firmware.sh scan
ls -l /dev/serial/by-id/ 2>/dev/null || true
for dev in /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  echo "=== $dev ==="
  udevadm info --query=property --name="$dev" | \
    grep -E '^(ID_VENDOR|ID_MODEL|ID_SERIAL|ID_PATH|DEVNAME)=' || true
  udevadm info --attribute-walk --name="$dev" 2>/dev/null | \
    grep -m1 -E 'ATTRS\{manufacturer\}|ATTRS\{product\}|ATTRS\{serial\}' || true
done
```

Record an inventory table before flashing:

| Role | Existing board | `/dev/serial/by-id` | `/dev/ttyACM*` | VID:PID | Product | Action |
| --- | --- | --- | --- | --- | --- | --- |
| Wheels | XIAO RP2040 | record | record | record | record | **do not flash** |
| Knives | XIAO RP2040 | record | record | record | record | **do not flash** |
| IMU | RP2040 | record | record | record | record | **do not flash** |
| INA226 | RP2350 | record | record | record | record | **do not flash** |
| Test board | spare RP2040 only | record after spike flash | record | record | `rmw_zenoh_pico_spike` | candidate |

Use all of the following to identify a board; never rely on `/dev/ttyACM0`
alone:

1. The physical USB cable and board label.
2. Product/manufacturer string.
3. VID:PID and USB path.
4. The stable `/dev/serial/by-id/...` symlink.
5. `picotool info -a` while the board is in BOOTSEL mode.

Before entering BOOTSEL mode, save the current production firmware artifacts
and note which board is connected. For a production board, use the existing
firmware workflow only for inspection; do not select it for this spike.

## 3. Build and verify the experimental UF2

Build only the isolated target:

```bash
./helpers/rmw_zenoh_pico_spike/build.sh
```

Verify the artifact and the symbol audit:

```bash
test -s helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.uf2
```

The build must finish with exit code `0`. If the source or pinned revisions
change, repeat the symbol audit in the build container:

```bash
docker run --rm --entrypoint bash \
  -v "$PWD:/project:ro" \
  microros/micro_ros_static_library_builder@sha256:f06c5d5ade3136f781404277e6e3e2ad0de8e720e0cccc0a8253bfa7ab3a80de \
  -lc '
    apt-get update >/dev/null &&
    apt-get install -y --no-install-recommends binutils-arm-none-eabi >/dev/null &&
    arm-none-eabi-size /project/helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.elf &&
    test -z "$(arm-none-eabi-nm -u /project/helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.elf)" &&
    ! arm-none-eabi-nm /project/helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.elf |
      grep -E "rmw_microxrcedds|uxr_"
  '
```

Do not proceed if the build is not reproducible or if XRCE symbols return.

## 4. Flash only a spare test RP2040

The standard [`run_firmware.sh`](../run_firmware.sh) does not know this
experimental target. Do not pass `rmw_zenoh_pico_spike` to that script: it
would reject the target or select an unrelated production image.

Use a spare RP2040 and flash with the pinned `picotool` container:

```bash
SPIKE_UF2="$PWD/helpers/rmw_zenoh_pico_spike/build/firmware/rmw_zenoh_pico_spike.uf2"
test -s "$SPIKE_UF2"

# Put only the spare test board into BOOTSEL mode.
docker run --rm --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/build/firmware:/project:ro" \
  -w /project \
  relobot_pico_ware_wheels_microros:latest \
  picotool info -a

docker run --rm --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$PWD/helpers/rmw_zenoh_pico_spike/build/firmware:/project:ro" \
  -w /project \
  relobot_pico_ware_wheels_microros:latest \
  picotool load -f -x "$(basename "$SPIKE_UF2")"
```

If the image does not exist, build the existing wheels image first:

```bash
docker build -t relobot_pico_ware_wheels_microros:latest \
  pico_ware_wheels_microros
```

After the automatic reboot, wait for USB enumeration and identify the new
device again:

```bash
sleep 3
./run_firmware.sh scan
ls -l /dev/serial/by-id/
```

The product string should identify the spike. If it does not, stop and use
`udevadm`, the USB path, and physical cable tracking before continuing.

## 5. Configure host Zenoh access

The Zenoh locator used by the RP2040 is:

```text
serial/usb#baudrate=115200
```

The router must open the host-side stable symlink, for example:

```text
serial//hostdev/serial/by-id/usb-REPLACE_WITH_SPIKE_ID#baudrate=115200
```

The example configuration is
[`helpers/picoros_poc/router-serial.example.json5`](../helpers/picoros_poc/router-serial.example.json5).
Copy it to a local file under `/tmp` and replace the placeholder; do not
commit a machine-specific path. Confirm the board identity and symlink:

```bash
SPIKE_ID="$(readlink -f /dev/serial/by-id/usb-REPLACE_WITH_SPIKE_ID)"
printf '%s\n' "$SPIKE_ID"
ls -l "$SPIKE_ID"
```

The router service in
[`ros2_ws/docker-compose.yml`](../ros2_ws/docker-compose.yml) mounts the
stable parent `/dev` at `/hostdev` read-only; its device cgroup permits
character major 166 (`ttyACM*`) across USB renumbering. Mounting just
`/dev/serial/by-id` proved insufficient in WSL: udev deleted and recreated
that directory, leaving the container mounted on the empty old directory.
The cgroup rule grants access to all host ACM devices, not just this board;
the by-id locator still selects only the verified test board. A read-only
device-node mount does not prevent serial writes once the device is opened.

Start only the router with the optional config URI after verifying the local
file at `/tmp/picoros-router.json5`:

```bash
ZENOH_ROUTER_CONFIG_URI=/tmp/picoros-router.json5 \
  ./start_robot.sh up ros2_zenoh_router --detach
docker compose -f ros2_ws/docker-compose.yml logs -f ros2_zenoh_router
```

With no `ZENOH_ROUTER_CONFIG_URI` set, the router retains its original
configuration (no serial listener). Check inside the router that
`/hostdev/serial/by-id/usb-REPLACE_WITH_SPIKE_ID` and its tty target both
resolve. WSL requires manual USBIP reattachment after hotplug; on the robot,
verify the newly assigned tty minor is accessible to the running container.
Do not start motion controllers while using the non-actuating spike.

## 6. Required functional tests

Record command output and timestamps for every test. A pass requires all
checks in the section; a partial result is not production evidence.

### 6.1 USB enumeration and startup

```bash
ls -l /dev/serial/by-id/
readlink -f /dev/serial/by-id/usb-REPLACE_WITH_SPIKE_ID
docker compose -f ros2_ws/docker-compose.yml logs --since=2m ros2_zenoh_router
```

Pass criteria:

- the stable symlink exists;
- the router opens the endpoint without permission or framing errors;
- the board remains enumerated for at least 10 minutes.

### 6.2 ROS graph discovery and typed data

Run ROS 2 commands inside an existing ROS 2 container, after sourcing its ROS
environment as required by `AGENTS.md`:

```bash
docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c \
  "source /opt/ros/lyrical/setup.bash && ros2 topic list | grep -F /rmw_zenoh_pico/value"

docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c \
  "source /opt/ros/lyrical/setup.bash && ros2 topic type /rmw_zenoh_pico/value"

docker compose -f ros2_ws/docker-compose.yml exec ros2_nav2 bash -c \
  "source /opt/ros/lyrical/setup.bash && ros2 topic echo /rmw_zenoh_pico/value --once"
```

Pass criteria:

- topic discovery completes within 30 seconds;
- type is exactly `std_msgs/msg/Float32`;
- values increase approximately once per second;
- no Micro-ROS Agent or `uros_bridge` is needed for this topic.

### 6.3 Reconnect and lease recovery

Run each case at least 20 times and record the worst recovery time:

1. Unplug and reconnect the USB cable.
2. Restart only the Zenoh router.
3. Stop and restart the ROS 2 inspection container.
4. Reset the board without changing the router.
5. Disconnect the router endpoint for 60 seconds, then restore it.

Pass criteria:

- the board re-enumerates without reflashing;
- the topic is rediscovered automatically;
- increasing values resume;
- no stale command path is involved;
- recovery is bounded and documented;
- no process requires manual deletion of stale sessions.

### 6.4 Negative and fault tests

Verify that the system fails explicitly:

- wrong serial path;
- missing ACM device;
- router permission denied;
- malformed or unavailable router configuration;
- board reset during an active session;
- router unavailable during board startup.

Pass criteria:

- router logs a clear error;
- no success-shaped empty ROS graph is reported as healthy;
- retry/recovery behavior is documented;
- the board remains safe and non-actuating.

### 6.5 Resource and timing measurements

Measure at minimum:

- free heap after startup;
- minimum free heap during 30-minute operation;
- task stack high-water marks;
- publish period and jitter;
- USB reconnect recovery time;
- CPU load if a reliable board-side measurement is available;
- flash and RAM usage.

The current BSS already uses approximately 138 KiB of the RP2040's 264 KiB
RAM. Do not migrate a motor control loop until runtime headroom, stack
high-water marks, and watchdog behavior are measured on hardware.

## 7. Production-readiness gates

The spike is **not production-ready** until every item below is signed off:

- [ ] Spare-board USB enumeration works repeatedly.
- [ ] Stable `/dev/serial/by-id` mapping is recorded for every intended board.
- [ ] Router container has explicit, least-privileged device access.
- [ ] Lyrical ROS graph discovers the typed topic.
- [ ] CDR payloads and type hash are accepted by the actual robot stack.
- [ ] Twenty USB disconnect/reconnect cycles pass.
- [ ] Twenty router restart cycles pass.
- [ ] Startup with router unavailable recovers without reflashing.
- [ ] Runtime heap and task-stack margins are measured and documented.
- [ ] Watchdog and communication timeout behavior are implemented locally.
- [ ] No actuator can remain enabled after communication loss.
- [ ] RP2350/INA226 has a separate cross-build and memory validation.
- [ ] Production topic inventory, QoS, rates, and timestamps are validated.
- [ ] Existing production UF2 files and rollback procedure are archived.
- [ ] A second board passes the same test independently.
- [ ] No production board is migrated based on compile-only evidence.

## 8. Migration order after the spike

Use this order and stop at the first failing gate:

1. Non-actuating spare RP2040 test board.
2. INA226 RP2350 as a read-only power telemetry pilot.
3. IMU RP2040 after timestamp and reconnect behavior are accepted.
4. Wheels only after local watchdog, command timeout, direction dead-time,
   speed limits, and emergency stop remain independent of Zenoh.
5. Knives last, with blade-enable interlock and fail-safe shutdown verified.

For each board, retain the current Micro-ROS UF2 and bridge path until the
Zenoh replacement has passed the full soak test. Never perform a simultaneous
cutover of wheels and knives.

## 9. Rollback

Immediately rollback if any test causes stale data, unbounded reconnect, memory
exhaustion, actuator enablement after communication loss, or unexplained ROS
graph state.

1. Stop the robot:
   ```bash
   ./start_robot.sh down
   ```
2. Disconnect the experimental board from the robot harness.
3. Restore the saved production UF2 using the existing board-specific workflow.
4. Restore the previous router configuration and compose device mappings.
5. Start the known-good stack:
   ```bash
   ./start_robot.sh up --dev
   ```
6. Verify the original topic and actuator safety behavior before resuming.

Do not erase or overwrite the saved production UF2 while diagnosing the
experimental path.
