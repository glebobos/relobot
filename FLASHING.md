# Flashing Firmware to ReloBot Boards

This guide explains how to identify, reset, and upload firmware to the RP2040 and RP2350 boards used in ReloBot.

## 1. Identify Your Boards

Use the following tools to see which boards are connected and which serial ports they are using:

### Using `lsusb`
List all USB devices:
```bash
lsusb
```
- **RP2040 (XIAO)**: `2886:0042`
- **RP2040 (Pico)**: `2e8a:000a` (Standard)
- **RP2350 (Pico 2)**: `2e8a:0009` (Standard)

### Using `picotool`
`picotool` is the most powerful way to inspect boards:
```bash
picotool info -a
```
*Note: Boards running CircuitPython (like WHEELS and KNIVES) may not show up in `picotool` until they are put into BOOTSEL mode.*

---

## 2. Put Boards into BOOTSEL Mode

To upload new firmware (UF2), the board must be in **BOOTSEL mode**.

### The Programmatic Way (1200 Baud Trick)
You can force a board into BOOTSEL mode by setting its serial port to 1200 baud:

- **WHEELS**: `stty -F /dev/ttyACM0 1200`
- **KNIVES**: `stty -F /dev/ttyACM3 1200`
- **IMU**: `stty -F /dev/ttyACM5 1200`
- **INA226**: `stty -F /dev/ttyACM2 1200`

### The Hardware Way
If the serial port is not responding:
1. Unplug the USB cable.
2. Hold down the **BOOT** button on the board.
3. Plug the USB cable back in.
4. Release the button.

---

## 3. Uploading Firmware (UF2)

Once in BOOTSEL mode, the board will appear as a USB drive named `RPI-RP2`.

### Method A: Manual Copy (Recommended)
Simply copy your `.uf2` file to the mounted drive:
```bash
cp your_firmware.uf2 /media/admin/RPI-RP2/
```
The board will automatically disconnect, reboot, and start running the new firmware.

### Method B: Using `picotool`
If you have `picotool` installed, you can flash without manually mounting:
```bash
picotool load -x your_firmware.uf2
```

---

## 4. Current Device Mapping Reference

| Function | Board Type | Chip | Default Serial Port |
| :--- | :--- | :--- | :--- |
| **WHEELS** | Seeeduino XIAO | RP2040 | `/dev/ttyACM0` |
| **KNIVES** | Seeeduino XIAO | RP2040 | `/dev/ttyACM3` |
| **IMU** | RPi Pico | RP2040 | `/dev/ttyACM4` |
| **INA226** | RPi Pico 2 | **RP2350** | `/dev/ttyACM2` |
