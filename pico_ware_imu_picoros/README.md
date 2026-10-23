# IMU Pico-ROS integration

ICM20948 IMU sensor firmware migrated to native Pico-ROS: RP2040 USB serial -> `rmw_zenohd` -> ROS/Zenoh.
Publishes 6-axis DMP orientation quaternion, angular velocity, and linear acceleration directly into the Zenoh ROS 2 graph without micro-ROS agent or `uros_bridge`.

## Topics & Contracts

| Topic | ROS 2 Type | Rate | Description |
| --- | --- | --- | --- |
| `/imu` | `sensor_msgs/msg/Imu` | 30 Hz | 6-axis fused DMP orientation quaternion, gyro in rad/s, and linear acceleration in m/s². Frame ID `imu_link`. |
| `/imu/bench/*` | `std_msgs/msg/Float32` | 1 Hz | Bench mode diagnostics (`heap_free_bytes`, `heap_min_bytes`, `app_stack_free_words`, `sensor_stack_free_words`, `publish_errors`). |

### Covariances

| Measurement | Diagonal Value | Notes |
| --- | --- | --- |
| Orientation | `0.001` | Indices [0, 4, 8] |
| Angular Velocity | `0.0001` | Indices [0, 4, 8] |
| Linear Acceleration | `0.01` | Indices [0, 4, 8] |

## Hardware & Pinout

| Parameter | Value | Notes |
| --- | --- | --- |
| Controller | RP2040 (`pico`, `kamod_rp2040`, `xiao_rp2040`) | Single-core FreeRTOS |
| Sensor | ICM20948 | 9-axis motion sensor with on-chip DMP (`shared/drivers/ICM20948`) |
| I2C Peripheral | `i2c1` | 400 kHz Fast-mode |
| SDA Pin | GPIO 6 | Pull-up enabled |
| SCL Pin | GPIO 7 | Pull-up enabled |
| I2C Address | `0x68` | Configured via `IMU_I2C_ADDR` |
| Output Rate | 30 Hz | ~33 ms period matching DMP FIFO |
| Watchdog | 2000 ms | Hardware watchdog fed on sensor progress |

## Firmware Builds

Reuse the [shared firmware platform](../pico_firmware/README.md) and its prepared dependencies. Builds run offline inside Docker; source and dependencies are read-only.

```bash
# Build bench test artifact (no I2C hardware needed; mock telemetry & bench diagnostics)
./pico_ware_imu_picoros/build.sh kamod_rp2040 bench

# Build production sensor firmware for standard Raspberry Pi Pico
./pico_ware_imu_picoros/build.sh pico sensor

# Build sensor firmware for Seeed Studio XIAO RP2040
./pico_ware_imu_picoros/build.sh xiao_rp2040 sensor
```

Outputs are `picoros_imu_<board>_<mode>.uf2` in the firmware directory. Neither command flashes the board automatically.

### Bench vs Sensor Modes

- **`bench` mode (`RELOBOT_IMU_BENCH=ON`)**: Real I2C calls and vendor DMP loading are compiled out. Mock telemetry (identity quaternion [1, 0, 0, 0], zero angular velocity, and 1g vertical acceleration 9.80665 m/s²) is generated. Additional FreeRTOS heap, stack high-water marks, and publish error count diagnostics are published under `/imu/bench/*` at 1 Hz.
- **`sensor` mode (`RELOBOT_IMU_BENCH=OFF`)**: Full hardware I2C communication on pins 6 and 7 with ICM20948 DMP at 30 Hz.

## Safety & Diagnostics

- **Hardware Watchdog**: 2000 ms hardware watchdog enabled after sensor init and fed strictly in `sensor_task` upon successful progress. If the scheduler or I2C bus stalls, the hardware resets.
- **Data Staleness**: Each field group (orientation, angular velocity, linear acceleration) tracks the time since its last valid reading independently. If a group goes without a fresh reading for 500 ms (~15 missed 30 Hz samples, e.g. a dead/disconnected sensor that still lets the scheduler run), its `covariance[0]` is set to `-1` per the `sensor_msgs/Imu` convention instead of silently republishing the last-known value as if it were live.
- **Fault Hooks**: Stack overflow (`vApplicationStackOverflowHook`) and heap allocation failure (`vApplicationMallocFailedHook`) immediately disable interrupts and trigger a hardware watchdog reset.
- **1200 Baud Touch**: CDC line coding touch at 1200 baud triggers clean reboot into BOOTSEL mode via `reset_usb_boot()` for flashing convenience.
- **Status LED**:
  - Heartbeat: 100 ms off phase every 1000 ms confirms FreeRTOS scheduler liveliness.
  - Solid Blue: Codec initialization failure.
  - Red: Waiting/attempting Zenoh session connection.
  - Yellow/Amber: Zenoh session connected, declaring ROS entities.
  - Green: Fully operational and publishing telemetry.
  - Amber (solid, post-connection): Operational but one or more field groups are stale.

## Router & USB Configuration

Add the device to [ros2_ws/zenoh_devices.json](../ros2_ws/zenoh_devices.json) when ready for hardware deployment:

```json
{
  "devices": {
    "imu": {
      "serial_by_id": "usb-Raspberry_Pi_Pico_...-if00",
      "baudrate": 115200,
      "enabled": true
    }
  }
}
```

When enabled, `start_zenoh_router.sh` automatically adds the serial endpoint to `rmw_zenohd` and excludes the port from the micro-ROS agent.
