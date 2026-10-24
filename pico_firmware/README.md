# Shared Pico-ROS Firmware Foundation

This directory supplies common **source and build rules**, board profiles, runtime initialization, and toolchain infrastructure for all Pico-ROS firmwares in ReloBot. Each microcontroller application links its own ELF/UF2 binary from this shared foundation, its selected board profile, and its application sources.

For architectural decisions, performance benchmarks, and hardware validation results, see [wiki/zenoh_picoros_architecture_and_decisions.md](../wiki/zenoh_picoros_architecture_and_decisions.md).

## Boundaries

| Part | Owns | Does not own |
| :--- | :--- | :--- |
| [CMake modules](cmake/) | SDK/FreeRTOS imports, Pico-ROS/Zenoh features, generated message library linkage, UF2 outputs | Application topics, drivers or control policy |
| [Configuration](config/) | Single-core FreeRTOS and TinyUSB settings | Per-motor limits or transport health inference |
| [Runtime](src/picoros_runtime.c) | Initial Pico-ROS client setup on USB serial | Nodes, publishers, subscribers or post-start reconnect policy |
| [USB serial support](src/usb_serial/) | Cooperative receive buffering and pinned Zenoh-Pico link/executor bindings | ROS discovery, message schemas or a reconnect supervisor |
| [Board profiles](boards/) | MCU/SDK board selection (`xiao_rp2040`, `xiao_rp2350`, `kamod_rp2040`, `pico`), WS2812 pin, PIO driver and board startup | Meaning of LED colors or command state |
| Application | ROS entities, generated codecs, tasks/priorities, callbacks, command state and local timeouts | USB framing, Zenoh protocol or board-specific LED registers |

Implemented profiles are:
- `xiao_rp2040` (Seeed Studio XIAO RP2040, used by Wheels and Knives)
- `xiao_rp2350` (Seeed Studio XIAO RP2350, used by INA226 Power Monitor)
- `pico` (Raspberry Pi Pico RP2040, used by IMU)
- `kamod_rp2040` (KAmod RP2040 development/bench test board)

## Application API

- [`relobot_board_init()`](include/relobot/board.h) performs stdio/LED initialization before application tasks start.
- [`relobot_board_led_set()`](include/relobot/board.h) accepts RGB values; color meanings and task ownership remain application decisions.
- [`relobot_picoros_init()`](include/relobot/picoros_runtime.h) runs inside an application FreeRTOS task, initializes the native Pico-ROS client at `serial/usb#baudrate=115200`, and returns a native `picoros_res_t` result.

`RELOBOT_PICOROS_USB_POLL=ON` is the shared default for cooperative USB serial transport with the pinned Zenoh-Pico library. `relobot_add_firmware()` links the cooperative receive implementation automatically.

## Build Integration

An application includes the imports **before** `project()` and the shared platform afterwards:

```cmake
cmake_minimum_required(VERSION 3.13)
include("${CMAKE_CURRENT_LIST_DIR}/../pico_firmware/cmake/import.cmake")
project(my_controller C CXX ASM)
include("${RELOBOT_PICO_ROOT}/cmake/picoros.cmake")
relobot_add_firmware(my_controller src/main.c)
```

The toolchain image is `relobot_pico_firmware_builder:latest`, defined in [Dockerfile](Dockerfile).
Dependencies (`pico-sdk` v2.2.0, `FreeRTOS-Kernel` v11.2.0, `picoros`, and `libmicroros` type headers) are initialized via `./pico_firmware/setup_dependencies.sh`.
