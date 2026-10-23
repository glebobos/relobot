# Differential Drive Wheels Pico-ROS integration

Differential drive wheel control firmware migrated to native Pico-ROS: RP2040 USB serial -> `rmw_zenohd` -> ROS/Zenoh.
Subscribes to `/robot_joint_commands` and publishes `/robot_joint_states` directly into the Zenoh ROS 2 graph without micro-ROS agent or `uros_bridge`.

## Topics & Contracts

| Direction | Topic | ROS 2 Type | Rate | Description |
| :--- | :--- | :--- | :--- | :--- |
| Subscribe | `/robot_joint_commands` | `sensor_msgs/msg/JointState` | Up to 100 Hz | Commanded wheel velocities in rad/s (`left_wheel_joint`, `right_wheel_joint`). |
| Publish   | `/robot_joint_states`   | `sensor_msgs/msg/JointState` | ~30 Hz (33 ms) | Measured wheel positions (rad) and velocities (rad/s) from encoder ticks. |

Joint names: `left_wheel_joint`, `right_wheel_joint`.

## Architecture & Hardware

- **Transport**: Native Zenoh-Pico client over USB serial CDC (`serial/usb#baudrate=115200`).
- **Operating System**: Single-core FreeRTOS with cooperative USB polling.
- **Motor Driver**: Dual Cytron MD13S PWM/DIR drivers.
- **Control**: Per-motor feedforward `pwm = K * |omega| + C` (direction-aware, calibrated for forward and reverse on each motor) plus throttled PI feedback correction (50 ms interval) and kick-start stall handling.
- **Velocity measurement**: Period-based (ISR timestamps between encoder ticks) combined with moving average filtering and age-based velocity decay.
- **Encoders**: Single-channel rising-edge interrupt, 60 CPR, 10 ms debounce filter.

## Pin Assignments (XIAO RP2040 -> GPIO)

| Signal | Function | GPIO | Board Label |
| :--- | :--- | :--- | :--- |
| `PIN_LEFT_PWM` | Left Motor Speed | 27 | D1 (Slice 5 Ch B) |
| `PIN_LEFT_DIR` | Left Motor Direction | 26 | D0 |
| `PIN_RIGHT_PWM` | Right Motor Speed | 28 | D2 (Slice 6 Ch A) |
| `PIN_RIGHT_DIR` | Right Motor Direction | 7 | D5 |
| `PIN_ENCODER_LEFT` | Left Encoder Pulse | 6 | D4 |
| `PIN_ENCODER_RIGHT` | Right Encoder Pulse | 3 | D10 |

## Key Parameters

| Parameter | Value | Description |
| :--- | :--- | :--- |
| `COUNTS_PER_REVOLUTION` | 60.0 | Encoder CPR |
| `PWM_WRAP` | 9614 | ~13 kHz at 125 MHz sys_clk |
| `CONTROL_PERIOD_MS` | 10 | 100 Hz control loop |
| `PUBLISH_PERIOD_MS` | 33 | ~30 Hz state publish |
| `COMMAND_TIMEOUT_MS` | 2000 | Command expiration timeout (motors ramp/stop) |
| `MOTOR_DEADBAND_RAD_S` | 0.35 | Below this speed: coast |
| `MAX_VEL_RAD_S` | 7.5 | Absolute velocity clamp |
| `SLEW_LIMIT_RAD_S_PER_CYCLE` | 0.5 | Max velocity delta per 10 ms step |
| `FF_K_LEFT_FWD / REV` | 0.135435 / 0.160302 | Left motor feedforward gain |
| `FF_C_LEFT_FWD / REV` | 0.041088 / 0.026934 | Left motor stiction offset |
| `FF_K_RIGHT_FWD / REV` | 0.142810 / 0.153041 | Right motor feedforward gain |
| `FF_C_RIGHT_FWD / REV` | 0.046127 / 0.026962 | Right motor stiction offset |
| `PI_KP / PI_KI` | 0.10 / 0.30 | PI gains in PWM fraction units |
| `DIRECTION_DEADTIME_MS` | 2 | Minimum coast time during direction reversal |

## Firmware Builds

Reuse the [shared firmware platform](../pico_firmware/README.md) and its prepared dependencies. Builds run offline inside Docker; source and dependencies are read-only.

```bash
# Build bench test artifact (no motor hardware needed; mock telemetry & bench diagnostics)
./pico_ware_wheels_picoros/build.sh kamod_rp2040 bench

# Build bench firmware for Seeed Studio XIAO RP2040
./pico_ware_wheels_picoros/build.sh xiao_rp2040 bench

# Build production motor control firmware for Seeed Studio XIAO RP2040
./pico_ware_wheels_picoros/build.sh xiao_rp2040 motor
```

Outputs are `picoros_wheels_<board>_<mode>.uf2` in the firmware directory.

### Bench vs Motor Modes

- **`bench` mode (`RELOBOT_WHEELS_BENCH=ON`)**: Real motor GPIO and PWM hardware calls are compiled out. Commanded speeds, control slew calculations, and deadtime logic run cleanly. Additional FreeRTOS heap, stack high-water marks, target speeds, and publish error count diagnostics are published under `/wheels/bench/*` at 1 Hz.
- **`motor` mode (`RELOBOT_WHEELS_BENCH=OFF`)**: Full hardware PWM and GPIO directional outputs on pins 26, 27, 28, 7, and rising-edge encoder interrupts on pins 6 and 3.

## Safety & Diagnostics

- **Hardware Watchdog**: 2000 ms hardware watchdog enabled at startup and fed strictly in `control_task` upon successful control progress.
- **Max Control Gap**: If the control loop stalls for >100 ms, an emergency stop is triggered.
- **Command Timeout**: If `/robot_joint_commands` stops publishing for >2000 ms, targets safely reset to 0 rad/s.
- **Fault Hooks**: Stack overflow (`vApplicationStackOverflowHook`) and heap allocation failure (`vApplicationMallocFailedHook`) immediately cut interrupts, stop motor outputs, and trigger a hardware watchdog reset.
- **1200 Baud Touch**: CDC line coding touch at 1200 baud triggers clean reboot into BOOTSEL mode via `reset_usb_boot()` for flashing convenience.
- **Status LED**:
  - Heartbeat: 100 ms off phase every 1000 ms confirms FreeRTOS scheduler liveliness.
  - Solid Blue: Codec initialization failure.
  - Red: Waiting/attempting Zenoh session connection.
  - Yellow/Amber: Zenoh session connected, declaring ROS entities.
  - Green: Fully operational and active (motors commanded).
  - Dim Amber: Ready/idle in standby.

## Unit Testing

Run the isolated host/container test suite:

```bash
./pico_ware_wheels_picoros/tests/run_tests.sh
```
