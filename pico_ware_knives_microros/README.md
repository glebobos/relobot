# Knife RP2040 micro-ROS firmware

This package contains the RP2040 firmware for knife spindle control using micro-ROS over USB serial transport.

## Topics
- Subscribes: `knives/set_rpm` (`std_msgs/msg/Float32`)
- Publishes: `knives/current_rpm` (`std_msgs/msg/Float32`)

## Notes
- PID control is internal only (not exposed as ROS service/status topic).
- Voltage telemetry is intentionally removed from this firmware.
- Legacy line-based serial commands (such as `whoyouare`) are not used.

## Build & Flashing

Use the unified firmware management script from the repository root:

```bash
# Build firmware
./run_firmware.sh build knives

# Flash firmware (auto-resets board and uploads via picotool)
./run_firmware.sh flash knives
```

Alternatively, run `./run.sh` inside this directory:
```bash
./run.sh build
./run.sh flash
```

For detailed flashing info, see [FLASHING.md](../../FLASHING.md).
