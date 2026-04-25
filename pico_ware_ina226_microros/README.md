# INA226 RP2350 micro-ROS firmware

This package contains the RP2350 (Pico 2) firmware for power monitoring using the INA226 sensor, publishing over micro-ROS.

## Build
Use the same workflow as the other pico micro-ROS targets:

```bash
docker compose run --rm builder
```

The generated UF2 is copied into `src/`.

## Flashing
To upload the generated `.uf2` file to the board:

1. **Put the board in BOOTSEL mode**:
   ```bash
   stty -F /dev/ttyACM2 1200
   ```
2. **Copy the firmware**:
   ```bash
   cp src/pico_micro_ros_ina226.uf2 /media/admin/RPI-RP2/
   ```
   *Note: The board will reboot automatically after the copy completes.*

For more details, see [FLASHING.md](../../FLASHING.md).
