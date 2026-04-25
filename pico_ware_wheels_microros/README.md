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

## Architecture

- **Transport**: USB CDC (stdio USB). UART stdio is disabled.
- **Hardware plugin (ROS2 side)**: `topic_based_ros2_control/TopicBasedSystem` — reads `/robot_joint_states` and writes `/robot_joint_commands`.
- **Velocity measurement**: Period-based (ISR timestamps between encoder ticks), not delta/dt. Avoids ×10 over-reading at low speeds with 60 CPR encoders.
- **Control**: Per-motor feedforward `pwm = K·|ω| + C` (direction-aware — right motor uses different K in reverse) plus PI correction in PWM-fraction units.
- **Encoders**: Single-channel, rising-edge interrupt, 60 CPR.

## Key Parameters (in `src/main.cpp`)

| Define | Value | Description |
| :--- | :--- | :--- |
| `COUNTS_PER_REVOLUTION` | 60 | Encoder CPR |
| `PWM_WRAP` | 12499 | ~10 kHz at 125 MHz |
| `CONTROL_PERIOD_MS` | 10 | 100 Hz control loop |
| `PUBLISH_PERIOD_MS` | 33 | ~30 Hz state publish |
| `MOTOR_DEADBAND_RAD_S` | 0.35 | Below this: coast |
| `FF_K_RIGHT_FWD / REV` | 0.12681 / 0.11793 | Right motor asymmetry from calibration |
| `PI_KP / PI_KI` | 0.018 / 0.005 | PI gains in PWM-fraction units |

## Build

```bash
docker compose run --rm builder
```

The generated `pico_wheels_microros.uf2` is placed in `src/`.

## Flashing

```bash
# 1. Trigger BOOTSEL mode (Pico is normally on /dev/ttyACM0)
sudo stty -F /dev/ttyACM0 1200

# 2. Wait ~3 s for the drive to mount, then copy
cp src/pico_wheels_microros.uf2 /media/admin/RPI-RP2/
```

The board reboots automatically. For full flashing details see [FLASHING.md](../FLASHING.md).
