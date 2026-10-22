# ReloBot Skills Suite

This directory contains specialized AI agent skills designed to enhance robotics, firmware, systems engineering, profiling, and debugging across the ReloBot repository.

> [!NOTE]
> Each skill folder contains a `SKILL.md` file (with YAML frontmatter and triggers) loaded dynamically on-demand by Antigravity agents when working on matching tasks.

## Skills Index by Domain

### Microcontroller & Embedded Firmware (RP2040 / Pico-ROS / FreeRTOS)

| Skill | Summary |
| :--- | :--- |
| [`freertos`](freertos/SKILL.md) | FreeRTOS tasks, priorities, queues, mutexes, stack overflow detection, and FreeRTOSConfig.h. |
| [`timers-pwm-baremetal`](timers-pwm-baremetal/SKILL.md) | Timer prescaler/ARR, PWM generation for Cytron MD13S motor drivers, and encoder input capture. |
| [`gpio-baremetal`](gpio-baremetal/SKILL.md) | Pin mode configuration, pull-ups/pull-downs, drive strength, debouncing, and GPIO interrupts. |
| [`spi-i2c-baremetal`](spi-i2c-baremetal/SKILL.md) | I2C and SPI master transfers, sensor register reads/writes (INA226, IMUs), bus lockup recovery. |
| [`uart-serial-baremetal`](uart-serial-baremetal/SKILL.md) | UART/CDC serial communications, baud rates, circular ring buffers, and micro-ROS/Zenoh transports. |
| [`dma-baremetal`](dma-baremetal/SKILL.md) | Direct Memory Access configuration for high-throughput sensor and serial streaming without CPU stalls. |
| [`interrupts-and-exceptions-baremetal`](interrupts-and-exceptions-baremetal/SKILL.md) | Cortex-M NVIC, ISR priorities, volatile variables, atomic flags, and interrupt latency. |
| [`mmio-and-bit-manipulation`](mmio-and-bit-manipulation/SKILL.md) | Direct register access, bitmasks, bitfield manipulation, and volatile MMIO semantics. |
| [`linker-scripts`](linker-scripts/SKILL.md) | GNU ld memory layouts, flash/RAM section definitions, vector tables, and stack/heap sizing. |
| [`openocd-jtag`](openocd-jtag/SKILL.md) | Hardware SWD/JTAG debugging, OpenOCD configuration, flashing, and GDB breakpointing on RP2040. |
| [`low-power-embedded`](low-power-embedded/SKILL.md) | MCU power optimization, sleep/dormant modes, clock gating, and power saving while docked. |
| [`resource-optimization-lowend`](resource-optimization-lowend/SKILL.md) | Flash & RAM optimization for constrained microcontrollers, analyzing .bss, .data, and stack size. |
| [`protocol-analysis`](protocol-analysis/SKILL.md) | Logic analyzer decoding and timing analysis for I2C, SPI, and UART bus troubleshooting. |
| [`peripherals-from-datasheet`](peripherals-from-datasheet/SKILL.md) | Methodology for developing peripheral drivers directly from hardware reference manuals. |
| [`datasheet-and-refmanual-reading`](datasheet-and-refmanual-reading/SKILL.md) | Systematic reading of MCU datasheets, register maps, electrical ratings, and errata sheets. |

### Performance Profiling, Concurrency & CPU Optimization (Raspberry Pi 5 host & ROS2)

| Skill | Summary |
| :--- | :--- |
| [`linux-perf`](linux-perf/SKILL.md) | CPU profiling with `perf record`/`report`, sampling hardware PMU counters, and hotspot analysis. |
| [`flamegraphs`](flamegraphs/SKILL.md) | Generating, reading, and interpreting interactive SVG flamegraphs from perf and profiling data. |
| [`strace-ltrace`](strace-ltrace/SKILL.md) | Tracing system calls, signals, futex locks, and library calls to diagnose latency or blocking bugs. |
| [`valgrind`](valgrind/SKILL.md) | Memory leak detection with Memcheck, thread synchronization with Helgrind/DRD, and cache simulation with Callgrind. |
| [`heaptrack`](heaptrack/SKILL.md) | Fast Linux heap memory allocation profiler for finding leaks and memory allocation churn. |
| [`hardware-counters`](hardware-counters/SKILL.md) | Low-level CPU performance counter analysis (IPC, branch mispredictions, cache misses). |
| [`cpu-cache-opt`](cpu-cache-opt/SKILL.md) | Cache-aware design, false sharing prevention, AoS vs SoA data layouts, and prefetching. |
| [`concurrency-debugging`](concurrency-debugging/SKILL.md) | Diagnosing race conditions, deadlocks, lock ordering, thread inspection, and TSan reports. |
| [`memory-model`](memory-model/SKILL.md) | C++ and Rust memory model, atomic operations, acquire-release ordering, and lock-free concurrency. |
| [`assembly-arm`](assembly-arm/SKILL.md) | AArch64 (Pi 5) and ARM Thumb (RP2040) assembly, calling conventions, inline asm, and SIMD registers. |

### Compilers, Build Systems, Sanitizers & Runtime Correctness

| Skill | Summary |
| :--- | :--- |
| [`cmake`](cmake/SKILL.md) | Modern target-based CMake, toolchains, build presets, compiler options, and ROS 2 package builds. |
| [`cross-gcc`](cross-gcc/SKILL.md) | ARM cross-compilation toolchains (arm-none-eabi-gcc for RP2040, aarch64-linux-gnu for Pi 5). |
| [`gdb`](gdb/SKILL.md) | Advanced GDB debugging, remote GDB, thread debugging, watchpoints, and Python scripting. |
| [`sanitizers`](sanitizers/SKILL.md) | Compiler sanitizers (AddressSanitizer, ThreadSanitizer, UndefinedBehaviorSanitizer) for C/C++. |
| [`static-analysis`](static-analysis/SKILL.md) | clang-tidy, cppcheck, scan-build, compile_commands.json setup, and compiler warning hardening. |
| [`debug-optimized-builds`](debug-optimized-builds/SKILL.md) | Debugging release and RelWithDebInfo builds without losing symbols or frame pointers. |
| [`core-dumps`](core-dumps/SKILL.md) | Configuring Linux core dump generation and post-mortem crash triage with GDB in Docker/host. |
| [`elf-inspection`](elf-inspection/SKILL.md) | Inspecting ELF binaries with readelf, objdump, nm, and ldd for symbol visibility and dependencies. |
| [`linkers-lto`](linkers-lto/SKILL.md) | Linker flags, Link-Time Optimization (LTO), --gc-sections, and binary size reduction. |
| [`ninja`](ninja/SKILL.md) | High-speed Ninja build system diagnosis, build graphs, and parallel compilation. |
| [`gcc`](gcc/SKILL.md) | GCC compiler optimization levels, warning flags, diagnostic flags, and codegen tuning. |
| [`clang`](clang/SKILL.md) | Clang diagnostics, AST dumps, sanitize options, and Clang-based tooling. |

### Linux Systems, Virtualization & Hardware Buses

| Skill | Summary |
| :--- | :--- |
| [`containers-internals`](containers-internals/SKILL.md) | Linux namespaces, cgroups v2 resource limits, Docker host networking, IPC, and /dev/ device passthrough. |
| [`device-tree`](device-tree/SKILL.md) | Raspberry Pi Device Tree overlays (DTS/DTSI) for pinmuxing, UART, SPI, and I2C buses. |
| [`bus-drivers-i2c-spi`](bus-drivers-i2c-spi/SKILL.md) | Linux kernel I2C and SPI client drivers, regmap abstraction, and bus transaction debugging. |
| [`ebpf`](ebpf/SKILL.md) | eBPF and bpftrace scripting for non-invasive kernel and socket buffer latency tracing. |

### Rust Systems Development & Profiling (rosbridge_rust)

| Skill | Summary |
| :--- | :--- |
| [`rust-skills`](rust-skills/SKILL.md) | Comprehensive 265-rule guide across 26 categories for writing high-performance, idiomatic Rust. |
| [`rust-profiling`](rust-profiling/SKILL.md) | cargo-flamegraph, cargo-bloat, cargo-llvm-lines, and Criterion microbenchmarking for Rust. |

### Architecture & Review

| Skill | Summary |
| :--- | :--- |
| [`design-challenger`](design-challenger/SKILL.md) | Principal Robotics & Systems Architect reviewer for critical code/design review and first-principles pushback. |

*Total active skills: 44*
