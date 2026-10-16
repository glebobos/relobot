# Pico-ROS host proof

This helper validates the part of the migration that does not require an MCU:

1. checks out Pico-ROS commit `b739fc713da2ee1caa3d1c278c5ed9cc1702eaa2`;
2. initializes its exact submodule revisions;
3. builds it in a pinned Ubuntu container;
4. runs the Pico-ROS serialization test executable;
5. publishes a CDR-encoded `std_msgs/msg/String` through Zenoh-Pico; and
6. verifies that ReloBot's Lyrical `rmw_zenoh_cpp` discovers and decodes it.

It does not claim to validate RP2040 USB CDC, reconnect behavior, clock
synchronization, or hardware safety.

## Run

From the repository root:

```bash
./start_robot.sh up ros2_zenoh_router --detach
./helpers/picoros_poc/run_host_poc.sh
```

The success marker is:

```text
PASS: pinned Pico-ROS -> project rmw_zenoh_cpp typed interoperability
```

The source checkout defaults to `/tmp/relobot-picoros-poc`. Override it with
`PICOROS_POC_WORK_DIR`.

## Serial router configuration

[`router-serial.example.json5`](./router-serial.example.json5) is a minimal
configuration template for the first attached-board experiment. Replace the
placeholder with the filename from the board's stable `/dev/serial/by-id/`
symlink. Do not use a `/dev/ttyACM<N>` index in the robot configuration.

The router service mounts the host `/dev` at `/hostdev` read-only and grants
access to `ttyACM*` devices (character major 166) across USB re-enumeration.
Copy the example to a local file under `/tmp`, replace the placeholder, and
start only the router with:

```bash
ZENOH_ROUTER_CONFIG_URI=/tmp/picoros-router.json5 \
  ./start_robot.sh up ros2_zenoh_router --detach
```

The URI is optional; without it the router uses its original default
configuration. The official Zenoh-Pico locator syntax for an RP2040 USB CDC
client is `serial/usb#baudrate=115200`; inside the container the router uses
`serial//hostdev/serial/by-id/<board>#baudrate=115200`.
On WSL, manually reattach USB to WSL after physical unplug/replug. The
router's serial listener can then reopen the new device without restarting.
The read-only bind prevents modifications to device nodes, not writes through
an opened serial device. Do not use this configuration for actuating boards
until all reconnect and safety gates pass.

USB CDC support is currently marked experimental by Zenoh-Pico and requires
both `Z_FEATURE_LINK_SERIAL_USB=1` and `Z_FEATURE_UNSTABLE_API=1`. Treat a
successful TCP proof as necessary but not sufficient evidence for serial.
