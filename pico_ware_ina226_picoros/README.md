# INA226 Pico-ROS integration

INA226 power monitoring firmware migrated to native Pico-ROS: RP2040 USB serial -> `rmw_zenohd` -> ROS/Zenoh.
Publishes battery voltage, charger voltage, and docking detection status directly into the Zenoh ROS 2 graph without micro-ROS agent or `uros_bridge`.

## Topics & Contracts

| Topic | ROS 2 Type | Rate | Description |
| --- | --- | --- | --- |
| `/battery_voltage` | `std_msgs/msg/Float32` | 10 Hz | Battery pack bus voltage in Volts (1.25 mV LSB). Omitted on I2C error. |
| `/charger_voltage` | `std_msgs/msg/Float32` | 10 Hz | Docking charger bus voltage in Volts. Omitted on I2C error. |
| `/on_dock` | `std_msgs/msg/Bool` | 10 Hz | Docking state (`true` if charger voltage > 12.5V, guarded against `NAN`). Always published. |

## Hardware & Pinout

| Parameter | Value | Notes |
| --- | --- | --- |
| Controller | RP2040 (`pico`, `kamod_rp2040`, `xiao_rp2040`) | Single-core FreeRTOS |
| I2C Peripheral | `i2c1` | 400 kHz Fast-mode |
| SDA Pin | GPIO 6 | Pull-up enabled |
| SCL Pin | GPIO 7 | Pull-up enabled |
| I2C Timeout | 5 ms | Non-blocking timeout preventing bus lockup |
| Battery Address | `0x40` | Bus voltage register `0x02` |
| Charger Address | `0x41` | Bus voltage register `0x02` |
| Dock Threshold | 12.5V | Configured via `DOCK_VOLTAGE_THRESHOLD` |
| Watchdog | 2000 ms | Hardware watchdog fed on sensor progress |

## Firmware Builds

Reuse the [shared firmware platform](../pico_firmware/README.md) and its prepared dependencies. Builds run offline inside Docker; source and dependencies are read-only.

```bash
# Build bench test artifact (no I2C hardware needed; mock telemetry & bench diagnostics)
./pico_ware_ina226_picoros/build.sh kamod_rp2040 bench

# Build production sensor firmware for standard Raspberry Pi Pico
./pico_ware_ina226_picoros/build.sh pico sensor

# Build sensor firmware for Seeed Studio XIAO RP2040
./pico_ware_ina226_picoros/build.sh xiao_rp2040 sensor
```

Outputs are `picoros_ina226_<board>_<mode>.uf2` in the firmware directory. Neither command flashes the board automatically.

### Bench vs Sensor Modes

- **`bench` mode (`RELOBOT_INA226_BENCH=ON`)**: Real I2C calls are compiled out. Nominal battery (14.8V) and charger (0.0V, off-dock) mock telemetry are produced. Additional FreeRTOS heap, stack high-water marks, and publish error count diagnostics are published under `/ina226/bench/*` at 1 Hz.
- **`sensor` mode (`RELOBOT_INA226_BENCH=OFF`)**: Full hardware I2C communication on pins 6 and 7. Valid readings are published at 10 Hz; I2C error values (`NAN`) are suppressed from battery/charger topics to prevent invalid sensor data from polluting consumers. `/on_dock` defaults safely to `false` on I2C failure.

## Safety & Diagnostics

- **Hardware Watchdog**: 2000 ms hardware watchdog enabled at startup and fed strictly in `sensor_task` upon successful progress. If the scheduler or I2C bus stalls, the hardware resets.
- **Fault Hooks**: Stack overflow (`vApplicationStackOverflowHook`) and heap allocation failure (`vApplicationMallocFailedHook`) immediately cut interrupts, record fault state, and trigger a hardware watchdog reset.
- **1200 Baud Touch**: CDC line coding touch at 1200 baud triggers clean reboot into BOOTSEL mode via `reset_usb_boot()` for flashing convenience.
- **Status LED**:
  - Heartbeat: 100 ms off phase every 1000 ms confirms FreeRTOS scheduler liveliness.
  - Solid Blue: Codec initialization failure.
  - Red: Waiting/attempting Zenoh session connection.
  - Yellow/Amber: Zenoh session connected, declaring ROS entities.
  - Green: Fully operational and publishing telemetry.

## Router & USB Configuration

Add the device to [ros2_ws/zenoh_devices.json](../ros2_ws/zenoh_devices.json) when ready for hardware deployment:

```json
{
  "devices": {
    "ina226": {
      "serial_by_id": "usb-Raspberry_Pi_Pico_...-if00",
      "baudrate": 115200,
      "enabled": true
    }
  }
}
```

When enabled, `start_zenoh_router.sh` automatically adds the serial endpoint to `rmw_zenohd` and excludes the port from the micro-ROS agent.
