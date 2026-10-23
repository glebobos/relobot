# ReloBot Architecture Decisions & Validation Evidence: Zenoh & Pico-ROS Migration

## 1. Executive Summary & Status

As of October 2026, ReloBot has fully migrated from **micro-ROS + Micro-XRCE-DDS + FastDDS (ROS 2 Humble)** to **native Pico-ROS over Zenoh-Pico USB CDC + rmw_zenoh_cpp (ROS 2 Lyrical)**.

This document consolidates the architectural rationale, hardware trial logs, resource benchmarks, and production decisions established during the migration spikes. It supersedes:
- `wiki/picoros_zenoh_migration_plan.md` (initial 7-phase migration roadmap)
- `wiki/picoros_zenoh_production_readiness.md` (experimental trial logs from October 16, 2026)
- `wiki/rmw_zenoh_pico_robot_validation.md` (historical `rmw_zenoh_pico` evaluation runbook)
- `PICO_NEXT_STEPS.md` (transitional knives-only rollout plan from October 20, 2026)

---

## 2. Architecture Decision Records (ADRs)

### ADR-1: Stack Migration to ROS 2 Lyrical with `rmw_zenoh_cpp` and Shared Memory

- **Context**: Under ROS 2 Humble with default FastDDS UDP transport, heavy SLAM costmap generation, high-rate TF broadcasting, and multi-process communication caused frequent UDP socket buffer overflows (`net.core.rmem_max` drops), cascading NACK storms, high kernel Software Interrupt (`si`) CPU utilization, and cross-process latency spikes.
- **Decision**: Upgrade host container stack to ROS 2 Lyrical, adopt `rmw_zenoh_cpp` as the primary middleware, and enable Zenoh shared memory (`transport/shared_memory/enabled=true`, lazy pool size 16 MB, threshold 512 B).
- **Outcome**: Process-to-process discovery jitter was eliminated. Zero-copy / shared memory transfers drastically reduced CPU overhead on high-frequency topics (`/scan`, `/tf`, `/map`).

### ADR-2: MCU Middleware Selection — Native Pico-ROS vs. `rmw_zenoh_pico` & micro-ROS

- **Evaluation of Alternatives**:
  1. **micro-ROS (`rmw_microxrcedds`)**: Required running separate `micro_ros_agent` containers on the host, bridging through FastDDS, and maintaining custom serial reconnection logic. UDP/serial drops caused agent restarts.
  2. **`rmw_zenoh_pico`**: An attempt was made to keep the Micro-ROS `rcl`/`rclc` API layer while replacing the RMW with `rmw_zenoh_pico`. While cross-compilation succeeded (text: 183 kB, bss: 138 kB), hardware tests on the RP2040 halted during `rclc_support_init_with_options()` prior to USB enumeration due to clock/allocator deadlocks in the early experimental RMW layer.
  3. **Native Pico-ROS (`Pico-ROS-software` + `zenoh-pico`)**: Reuses standard ROS 2 message types, Micro-CDR serialization/deserialization, and RIHS type hashes directly on top of Zenoh-Pico client over USB serial CDC. Messages publish and subscribe directly into the host `rmw_zenohd` router without any bridge node or agent.
- **Decision**: Adopt native Pico-ROS with the shared platform foundation (`pico_firmware/`) across all microcontroller boards (`wheels`, `knives`, `imu`, `ina226`).

### ADR-3: USB CDC Cooperative Polling & Host `zenoh-link-serial` Hotplug Patch

- **Issues Identified**:
  1. **Blocking USB RX Starvation**: The original upstream Zenoh-Pico USB transport used blocking `getc` calls on the single shared executor, starving the executor's reconnect and lease-check timers during serial silence.
  2. **Host Port Reopening after Hotplug**: When microcontrollers rebooted or disconnected, Linux assigned a new USB inode/minor. The host router retained a deleted `/hostdev/ttyACM* (deleted)` file descriptor and failed to reconnect.
- **Implemented Fixes**:
  1. **Cooperative USB Polling (`RELOBOT_PICOROS_USB_POLL=ON`)**: Replaced blocking USB reads with a non-blocking cooperative ring buffer poll (`serial_rx.c`, `serial_poll_link.c`), preserving partial COBS frames and allowing timer futures to execute.
  2. **Zenoh Serial Accept Patch (`zenoh-serial-accept.patch`)**: Applied to `zenoh-c` / `zenoh-link-serial` in [`ros2_ws/ZenohRouter.dockerfile`](../ros2_ws/ZenohRouter.dockerfile). It catches EOF/disconnect errors on serial streams and loops to reopen the canonical `/hostdev/serial/by-id/<device>` path automatically upon re-enumeration.

### ADR-4: Single-Core FreeRTOS vs. SMP on RP2040

- **Context**: The RP2040 has dual Cortex-M0+ cores. An evaluation was conducted to determine whether FreeRTOS SMP (`configNUMBER_OF_CORES=2`) should be used.
- **Findings**: Single-core FreeRTOS easily sustained 100 Hz control loops and 30 Hz telemetry with substantial idle margin. SMP introduced locking complexity, potential priority inversion on the USB peripheral, and jitter on encoder ISRs without offering measurable latency reductions.
- **Decision**: Retain `configNUMBER_OF_CORES=1` as the production standard across all firmware.

---

## 3. Hardware Validation & Empirical Benchmarks

### 3.1 Static & Dynamic Resource Baseline (RP2040)
- **Static Footprint**:
  - Flash (`text`): ~153 KiB (well within 2 MiB / 16 MiB flash).
  - SRAM (`bss` + static): ~141 KiB (includes 128 KiB FreeRTOS heap + 10 KiB static state, leaving ample stack headroom in 264 KiB SRAM).
- **Dynamic Heap & Stack Measurements (10 Hz Bidirectional Load, 300 Commands)**:
  - Total FreeRTOS Heap: 131,072 bytes.
  - Peak Heap Usage: 113,912 bytes (86.9%).
  - Minimum Ever Free Heap: 17,160 bytes (13.1% safety margin).
  - Main App Task Stack: 32 KiB allocated, peak used 4,820 bytes (27.9 KiB headroom).
  - Telemetry Continuity: 0 dropped packets / 0 publish errors over 30 s continuous trials.
  - Roundtrip Latency: median 63.5 ms, p95 109.8 ms (bounded by the 100 ms telemetry tick).

### 3.2 Physical USB Hotplug & Recovery Trials
- **Single Power-Cycle Recovery (2026-10-16)**:
  - Spare KAmod board physically unplugged while bidirectional traffic was active.
  - Router cleanly logged `early eof`, closed the dead descriptor, and polled the stable `/dev/serial/by-id` path.
  - Upon reconnection, fresh telemetry resumed in 0.327 s without router crash or manual intervention.
  - 610 subsequent messages arrived over 61 s (9.977 Hz, max gap 141 ms).
- **Router Restart Recovery**:
  - Router container restarted with MCU running.
  - MCU re-established Zenoh session upon router restart; ack counter and uptime survived without MCU reset.

---

## 4. Production Safety Controls Implemented in Firmware

All native Pico-ROS firmwares implement unified safety guards:
1. **2000 ms Hardware Watchdog**: Enabled at startup and serviced exclusively in the control/sensor task upon proven forward progress (never fed in the USB/idle loop).
2. **Command Expiration & Zero-First Latch**: If motor commands cease for >2000 ms (wheels) or >5000 ms (knives), outputs ramp/disarm immediately. Nonzero targets cannot refresh an expired state without first receiving an explicit finite zero command.
3. **Execution Gap & Stall Detection**: If the control loop execution stalls for >100 ms, or if encoder pulses stop while commanded speed is active, outputs are cut immediately.
4. **Memory Fault Hooks**: Stack overflow (`vApplicationStackOverflowHook`) and malloc failure (`vApplicationMallocFailedHook`) immediately cut motor PWM/enables and trigger a watchdog hardware reset.
5. **1200-Baud BOOTSEL Touch**: Setting the USB CDC port to 1200 baud triggers an automatic software reset into BOOTSEL mode for seamless Docker-based reflashing via `picotool`.
