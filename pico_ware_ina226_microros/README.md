# INA226 RP2350 micro-ROS firmware

This package contains the RP2350 (Pico 2) firmware for power monitoring using the INA226 sensor, publishing over micro-ROS.

## Build & Flashing

Use the unified firmware management script from the repository root:

```bash
# Build firmware
./run_firmware.sh build ina226

# Flash firmware (auto-resets board and uploads via picotool)
./run_firmware.sh flash ina226
```

Alternatively, run `./run.sh` inside this directory:
```bash
./run.sh build
./run.sh flash
```

For more details, see [FLASHING.md](../../FLASHING.md).
