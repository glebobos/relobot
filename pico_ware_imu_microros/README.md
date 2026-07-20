# IMU RP2040 micro-ROS firmware

This package contains the RP2040 firmware for IMU sensor data publishing using micro-ROS over USB serial transport.

## Build & Flashing

Use the unified firmware management script from the repository root:

```bash
# Build firmware
./run_firmware.sh build imu

# Flash firmware (auto-resets board and uploads via picotool)
./run_firmware.sh flash imu
```

Alternatively, run `./run.sh` inside this directory:
```bash
./run.sh build
./run.sh flash
```

For more details, see [FLASHING.md](../../FLASHING.md).
