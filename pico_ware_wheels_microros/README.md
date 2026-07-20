# Wheels RP2040 micro-ROS firmware

This package contains the RP2040 (XIAO) firmware for differential-drive wheel control using micro-ROS over USB CDC serial transport.

## Topics

| Direction | Topic | Type |
| :--- | :--- | :--- |
| Subscribe | `/robot_joint_commands` | `sensor_msgs/msg/JointState` |
| Publish   | `/robot_joint_states`   | `sensor_msgs/msg/JointState` |

Joint names: `left_wheel_joint`, `right_wheel_joint`.  
Publish rate: ~30 Hz.  
Control loop rate: 100 Hz.

## Architecture & Hardware

- **Transport**: USB CDC (stdio USB). UART stdio is disabled.
- **Motor Driver**: Dual Cytron MD13S PWM/DIR drivers.
- **Hardware plugin (ROS2 side)**: `topic_based_ros2_control/TopicBasedSystem` — reads `/robot_joint_states` and writes `/robot_joint_commands`.
- **Velocity measurement**: Period-based (ISR timestamps between encoder ticks) combined with position-delta velocity fallback and moving average filtering.
- **Control**: Per-motor feedforward `pwm = K·|ω| + C` (direction-aware, calibrated for forward and reverse on each motor) plus throttled PI feedback correction (50ms interval) and kick-start stall handling.
- **Encoders**: Single-channel, rising-edge interrupt, 60 CPR.

## Pin Assignments (XIAO RP2040 -> GPIO)

| Signal | Function | GPIO | Pin Label |
| :--- | :--- | :--- | :--- |
| `PIN_LEFT_PWM` | Left Motor Speed | 27 | D1 |
| `PIN_LEFT_DIR` | Left Motor Direction | 26 | D0 |
| `PIN_RIGHT_PWM` | Right Motor Speed | 28 | D2 |
| `PIN_RIGHT_DIR` | Right Motor Direction | 7 | D5 |
| `PIN_ENCODER_LEFT` | Left Encoder Pulse | 6 | D4 |
| `PIN_ENCODER_RIGHT` | Right Encoder Pulse | 3 | D10 |

## Key Parameters (in `src/main.cpp`)

| Define | Value | Description |
| :--- | :--- | :--- |
| `COUNTS_PER_REVOLUTION` | 60 | Encoder CPR |
| `PWM_WRAP` | 9614 | ~13 kHz at 125 MHz sys_clk |
| `CONTROL_PERIOD_MS` | 10 | 100 Hz control loop |
| `PUBLISH_PERIOD_MS` | 33 | ~30 Hz state publish |
| `MOTOR_DEADBAND_RAD_S` | 0.35 | Below this: coast |
| `FF_K_LEFT_FWD / REV` | 0.135435 / 0.160302 | Left motor feedforward gain (rad/s to PWM fraction) |
| `FF_C_LEFT_FWD / REV` | 0.041088 / 0.026934 | Left motor stiction offset |
| `FF_K_RIGHT_FWD / REV` | 0.142810 / 0.153041 | Right motor feedforward gain |
| `FF_C_RIGHT_FWD / REV` | 0.046127 / 0.026962 | Right motor stiction offset |
| `PI_KP / PI_KI` | 0.10 / 0.30 | PI gains in PWM-fraction units |

## Build & Flashing

Use the unified firmware helper script from the repository root:

```bash
# Build wheels firmware
./run_firmware.sh build wheels

# Flash wheels firmware (auto-resets board and flashes via picotool)
./run_firmware.sh flash wheels
```

Alternatively, run `./run.sh` inside this directory:
```bash
./run.sh build
./run.sh flash
```

For full flashing details see [FLASHING.md](../FLASHING.md).

