# Knives Pico-ROS integration

Native Pico-ROS knives firmware: RP2040 USB serial -> `rmw_zenohd` -> ROS/Zenoh.
All four microcontroller subsystems (wheels, knives, IMU, and INA226) run native Pico-ROS communicating directly with the Zenoh router. Hardware startup selects native Pico-ROS by default. Ordinary simulation keeps the Gazebo knives mock.

For architecture decisions and validation benchmarks, see [wiki/zenoh_picoros_architecture_and_decisions.md](../wiki/zenoh_picoros_architecture_and_decisions.md).

## Firmware builds

Reuse the [shared firmware platform](../pico_firmware/README.md) and its pinned
dependencies prepared under `pico_firmware/build`. The builder
image `relobot_pico_firmware_builder:latest` must already be available. Builds run offline inside Docker; source and
dependencies are read-only, with generated dependency headers in a local copy.

```bash
./pico_ware_knives_picoros/build.sh kamod_rp2040 bench
./pico_ware_knives_picoros/build.sh xiao_rp2040 motor
```

The default is **KAmod bench**: no motor/enable/encoder GPIO access or IRQs;
`/knives/current_rpm` is zero, not a simulated command echo. KAmod motor mode
is rejected. XIAO motor is an explicit cross-build, **not hardware approval**.
The outputs are `picoros_knives_<board>_<mode>.uf2` and separate
`build/<board>-<mode>/` directories. Neither command flashes a board.
`run_firmware.sh build/flash knives` still selects the existing micro-ROS firmware.

The application uses standard Float32 command/RPM topics, validates finite
commands, clamps to +/-3500 RPM, and expires the target after 5000 ms without
an accepted command. Control runs every 20 ms, RPM sampling every 600 ms and
telemetry every 100 ms. Native Pico-ROS/Zenoh-Pico owns reconnect; entity
declarations, LED state or successful publishes do not prove a live link.

### Implemented safety, pending hardware qualification

- Boot, command timeout and control faults require an accepted finite zero
  before accepting nonzero RPM. Late nonzero cannot refresh an expired command.
- Timeout disables both enables and PWM in the next control iteration, without
  ramp or active reverse braking. A control-progress gap over 100 ms and encoder
  inactivity of 2000 ms at controlled speed >=100 RPM with a nonzero target
  also disable outputs/disarm. Encoder-loss
  detection is motor-only; the no-encoder bench is exempt.
- A 2000 ms hardware watchdog is fed after control/LED progress, not USB/idle.
  FreeRTOS allocation failure, stack-overflow detection and asserts cut outputs
  and request reboot, including Release builds. Normal active braking is bounded
  to 600 ms; ordinary ramp/PI/feed-forward and confirmed pins remain unchanged.
- The UI publishes at 5 Hz, clears RPM/cruise on WebSocket loss/reconnect, sends
  zero first and does not publish while disconnected. Held knife/cruise controls
  must be released before gamepad restart; deliberate slider input can restart
  after zero. Hidden-page input cannot bypass the stop. The 10 Hz rate below
  remains bench stress, not a required production rate.

**This is not strict stale-command or session protection.** Float32 has no source
timestamp/sequence; an old zero/nonzero backlog can still satisfy the local
latch. A healthy WebSocket does not prove the MCU is connected, and USB recovery
shorter than the timeout cannot be identified reliably from the pinned client's
notifications. Decide a stamped/session-aware protocol explicitly if required.
Mocked GPIO/controller and ROSLIB transport tests do not qualify electrical
shutdown, mechanical stopping time, physical E-stop or real browser recovery.

The safety builds were cross-built, **not flashed**. Their SHA-256 values are in
the [current plan](../PICO_NEXT_STEPS.md#forward-rollout-and-cleanup-boundaries).
Historical bench evidence below applies to the earlier flashed firmware only.

## Router and startup settings

The [ordinary Compose](../ros2_ws/docker-compose.yml) runs one router from the
same [serial-capable image](../ros2_ws/ZenohRouter.dockerfile) in all modes.
The previous partial JSON5 and separate knives Compose override have been
removed. The [startup script](../ros2_ws/start_zenoh_router.sh) uses
`ZENOH_CONFIG_OVERRIDE` to add serial **without changing other settings**:

| Setting | Preserved ROS default |
| --- | --- |
| Open / accept timeout | 60000 ms |
| Accept pending / maximum sessions | 10000 / 10000 |
| Lease / keep-alive count | 60000 ms / 2 |
| Shared memory | Disabled |
| TX queue sizes | 2 in all classes |

These values are not another replacement config. The regression test compares
the real effective router configs. TX queues count batches; changing them
does not resize MCU stacks or increase sustained USB bandwidth. The locator's
`baudrate=115200` is not a measured USB CDC throughput ceiling.

Operator settings:

- [../ros2_ws/zenoh_devices.json](../ros2_ws/zenoh_devices.json): the USB registry,
  read automatically by the launcher, router and micro-ROS device selector.
  Set `devices.knives.serial_by_id` to the exact `usb-...` basename from
  `ls -1 /dev/serial/by-id/`, not a full path, raw tty number or locator fragment.
  The checked-in value is empty deliberately: there is no implicit spare-board
  identity. No `export`, `.env` or systemd environment file is needed.
- `ZENOH_ROUTER_IMAGE`: optional deployment image reference, default
  `relobot-zenoh-router:local`, built by `./start_robot.sh build ros2_zenoh_router`.
  The ordinary Dockerfile includes the pinned serial/hotplug patch and checks
  the matching ROS vendor version (0.10.6). The tested lab image is **x86-64**;
  an ARM64/RPi5 build and qualification are still required. Record the qualified
  image ID/digest before deployment; a stock router is not equivalent.
- `ZENOH_ROUTER_RUST_LOG`: optional, default `info`; for an approved diagnosis
  use `info,zenoh_link_serial=debug,zenoh_transport=debug` and recreate the router.

The registry has this shape (replace the identity with the actual board name):

```json
{
  "devices": {
    "knives": {
      "serial_by_id": "<exact usb-... basename>",
      "baudrate": 115200,
      "enabled": true
    }
  }
}
```

Add future native Pico-ROS devices as other named entries under `devices`.
Every enabled entry gets a serial listener on the **same router** and is
excluded from the micro-ROS agent. Disabled entries are ignored; native
hardware/hybrid mode requires an enabled, configured `knives` entry. Do not add
the existing micro-ROS wheels/IMU/INA226 to this native-device registry.
Duplicate identities, invalid basenames, baud rates and malformed JSON are
rejected before hardware/hybrid `up`. Apply registry changes by stopping and
starting the affected stack with the launcher; editing JSON does not live-reload
a running router or agent. Configuration changes do not require rebuilding
the router image.

Hardware/hybrid mode retains TCP `[::]:7447` and adds the enabled serial by-id listeners. Compose
mounts the stable `/dev` parent at `/hostdev` read-only with `rslave`, and grants
ACM access through `c 166:* rwm`. A new tty inode after hotplug is therefore
visible; no fixed `/dev/ttyACM0` binding or privileged router is needed.
The agent excludes the selected board even via aliases/`TTY_*`; missing native
identity fails hardware launch closed. Both bridges exclude only knives topics.
Gazebo's knives mock is disabled only for explicit hybrid tests and retained
in ordinary simulation. Ordinary simulation does not read the device registry
or open serial listeners. Management commands (`down`, `logs`, `ps`, `build`) do not
require a board identity or connection. Direct Compose users must set
`PICO_KNIVES_ENABLED=false` for simulation; the launcher selects it automatically.

Build the single image without starting services:

```bash
./start_robot.sh build ros2_zenoh_router
```

For the approved physical-robot update, configure its USB identity and run
`./start_robot.sh up --dev`; no `--picoros-knives` flag is needed on hardware.
For the spare-board bench, select its USB by-id, with explicit permission to
start or recreate services:

```bash
./start_robot.sh up --sim --headless --dev --picoros-knives -d
```

**Simulation does not make a motor-enabled USB board safe.** Only use the
no-actuator bench firmware until production gates are met. Do not run this
command blindly against a working stack. `--dev` applies changed ROS launch
and bridge sources by rebuilding inside Docker; a plain restart is insufficient.
Keep `--sim --picoros-knives` on later hybrid operations; normal hardware
operations need no extra flag. A plain `./start_robot.sh up --sim --headless`
uses the same image without a serial listener and requires no USB board.

Inspect topic ownership and actual QoS with ROS tools **inside Docker**. There
must be one `/knives_node` RPM publisher and command subscriber, no knives mock
and no knives bridge endpoints. Do not open a parallel serial monitor on the
transport. Bench diagnostics at 1 Hz are under `/knives/bench/`:
`target_rpm`, `heap_free_bytes`, `heap_min_bytes`, `app_stack_free_words`,
`control_stack_free_words`, `publish_errors`. Heap values are bytes; stack
values are minimum remaining 32-bit words. Diagnostics are absent in motor mode.

## Tests and cleanup

Unit tests are run via `./pico_ware_knives_picoros/tests/run_tests.sh`. Architecture decisions and validation evidence are documented in [wiki/zenoh_picoros_architecture_and_decisions.md](../wiki/zenoh_picoros_architecture_and_decisions.md).
They cover initialization, command expiry/invalid values, USB ownership, router
config preservation and Gazebo mock selection. The command acceptance script
is for the **no-actuator bench only**, never a powered motor. No regression
command authorizes flashing, resets, USB actions or a live-stack restart.

The agreed rollout proceeds forward to native knives; rollback preparation is
not a prerequisite. Never give the serial device to both an agent and the
native router. Existing flash backups remain archived; generated UF2/ELF,
build trees and log/JSON captures stay outside Git. The standalone lab bridge
container is not managed by the ordinary Compose `down` command.

## Recorded bench evidence, 2026-10-20

These results apply to the spare KAmod RP2040 B2, 16 MiB flash,
ID `0xE464C498C3380C35`, by-id `usb-Raspberry_Pi_Pico_E464C498C3380C35-if00`.
They are historical evidence, **not defaults for the robot**. A verified full
flash backup and rollback UF2 were saved before the separately approved flash.

- Tested bench UF2 SHA-256:
  `ad7ec5ab815874a6415df2bf52e8d057a0026cf3db42a2b7d5cf2201c52c4744`.
- Tested x86 router image:
  `sha256:f5362188e9d04a3bdace2e98a2b5b08adda3fcb34f1a187c4018396f1ffab2d3`.
- 30 s of 1200-RPM targets at 10 Hz: 298 zero-RPM bench telemetry samples,
  9.932 Hz, maximum gap 107.94 ms. Zero target observed 5.216 s after the last
  command; negative/zero/clamped and NaN/Inf rejection/expiry checks passed.
  These host timeout observations are quantized by 1 Hz diagnostics.
- Heap free/min-ever 67640/64480 bytes; app/control minimum remaining stack
  1879/472 words; publish errors zero. Native transport-task stacks were not
  measured, and the encoder/motor path was disabled.
- Full headless simulation + a separately started C++ bridge: 90 s passive
  telemetry at 9.973 Hz, maximum gap 120.47 ms, no restart/OOM or router fault.
  **Actual DDS forwarding failed**: independent DDS and Zenoh observers
  received none of 78 telemetry probe samples. Independent DDS pairs also
  failed with custom XML (0/58), defaults (0/59) and defaults inside Gazebo
  (0/59). The later independent test narrows this to the current host-network
  environment: isolated-network controls pass, even with host IPC; host-network
  peers fail, even with private IPC. A minimal loopback profile does not pass
  repeated trials, so production DDS XML is unchanged and root cause is not
  declared fixed. This proves passive coexistence only,
  not full working micro-ROS/DDS traffic or the cause of the original error.
- One operator-performed USB unplug/replug with simulation/bridge running:
  Linux node absent 46.553 s; first fresh RPM 0.327 s after its return;
  610 subsequent messages over 61.039 s, 9.977 Hz, maximum gap 141.32 ms.
  Router/Gazebo/bridge IDs and start times unchanged, zero automatic restarts.
  One serial `early eof` ERROR on removal and 254 DEBUG missing-device retries
  were expected disconnect evidence, **not a router crash**.
  Same ttyACM0/minor returned; renumbering, command reception after reconnect
  and stale-command safety were not exercised. USB loss may reboot the board;
  counters do not establish uninterrupted MCU uptime.

Local ignored evidence/rollback directories (do not delete as test trash):

- `build/hardware-20261020T080408Z/`: acceptance JSON, metadata, logs,
  `flash-before.bin`, `firmware-before.uf2` and checksums.
- `build/fullstack-bridge-20261020T083234Z/`: passive telemetry, DDS failure
  captures, resource and lifecycle records.
- `build/usb-recovery-20261020T084517Z/`: USB/ROS timelines, lifecycle/logs,
  `usb-recovery-summary.json`.

No powered motor test, repeated recovery/endurance, working full DDS traffic
or ARM64 router qualification is claimed. The next work is the ordered
[real-robot preparation plan](../PICO_NEXT_STEPS.md#ordered-work-before-the-real-robot).
