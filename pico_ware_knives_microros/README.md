# Knife RP2040 micro-ROS firmware

This package contains the RP2040 firmware for knife spindle control using micro-ROS over USB serial transport.

## Topics
- Subscribes: `knives/set_rpm` (`std_msgs/msg/Float32`)
- Publishes: `knives/current_rpm` (`std_msgs/msg/Float32`)

## Notes
- PID control is internal only (not exposed as ROS service/status topic).
- Voltage telemetry is intentionally removed from this firmware.
- Legacy line-based serial commands (such as `whoyouare`) are not used.

## Build
Use the same workflow as the other pico micro-ROS targets:

```bash
docker compose run --rm builder
```

The generated UF2 is copied into `src/`.
