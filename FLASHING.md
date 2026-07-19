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

#### Note for WSL2 Users (Mounting USB Storage):
If you are using WSL2, after attaching the BOOTSEL device using `usbipd`, the storage drive will appear as a block device (e.g., `/dev/sdb` or `/dev/sdc`) inside WSL, but it will not mount automatically. To mount it manually:

1. **Identify the block device and verify it is `vfat`**:
   Run `lsblk -f` to list devices along with their filesystem types (`FSTYPE` column):
   ```bash
   lsblk -f
   ```
   Alternatively, inspect a specific partition with `blkid`:
   ```bash
   blkid /dev/sde1
   ```
   Look for a ~128MB partition with `FSTYPE` or `TYPE` equal to `vfat` (which based on your `lsblk` output is `/dev/sde1`).

2. **Create the mountpoint and mount**:
   Run the following commands (replace `/dev/sdb1` with your actual partition name):
   ```bash
   sudo mkdir -p /media/RPI-RP2
   sudo mount -t vfat /dev/sdf1 /media/RPI-RP2 -o uid=$(id -u),gid=$(id -g),utf8
   ```
3. **Copy the firmware**:
   ```bash
   cp your_firmware.uf2 /media/RPI-RP2/
   ```


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

---

## 5. WSL2 USB Passthrough (usbipd-win)

If you are developing inside Windows Subsystem for Linux (WSL2), USB devices connected to Windows are not attached to WSL by default. Use `usbipd-win` to forward the RP2040 boards to WSL.

### 1. Install usbipd-win
On Windows (Host), download and install the latest release from the [usbipd-win GitHub repository](https://github.com/dorssel/usbipd-win/releases).

### 2. Connect the Device to WSL
> [!IMPORTANT]
> The following commands **must be run on Windows (Host)** (in a Windows PowerShell or Command Prompt window), NOT inside the WSL terminal.

Open PowerShell or Command Prompt on Windows:


1. **List connected USB devices** to find your RP2040 board:
   ```cmd
   usbipd list
   ```
   Look for the `BUSID` (e.g., `1-4`) corresponding to a device like `USB Serial Device (COMx)` or `Raspberry Pi Pico`.

2. **Bind the device** (requires Administrator privileges, only needed once per port/device):
   ```cmd
   usbipd bind --busid <BUSID>
   ```

3. **Attach the device to WSL**:
   ```cmd
   usbipd attach --wsl --busid <BUSID>
   ```
   Now the device should appear inside WSL under `/dev/ttyACM*`. You can verify this by running `lsusb` or `ls /dev/ttyACM*` inside the WSL terminal.

4. **Detach the device**:
   When done, detach the device using:
   ```cmd
   usbipd detach --busid <BUSID>
   ```

