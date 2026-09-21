#!/bin/bash
# ==============================================================================
# ReloBot: Raspberry Pi 5 Host Performance & Power Optimization Script
# ==============================================================================
# Safe, non-destructive host optimizations for Raspberry Pi 5 / ROS2:
#  1. Unlocks 1.6A USB port current & GPU memory in /boot/firmware/config.txt
#  2. Enables Docker cgroup memory & swap accounting in /boot/firmware/cmdline.txt
#  3. Disables GUI / Desktop stack (multi-user.target, lightdm) for headless operation
#  4. Disables slow MicroSD swap and enables compressed ZRAM swap in RAM (zstd)
#  5. Pins CPU governor to "performance" (2.4 GHz constant Cortex-A76 clock)
#  6. Disables unused background daemons (cups, cups-browsed, ModemManager)
#  7. Tunes virtual memory & FastDDS network buffer sysctl parameters
#  8. Sets up systemd journal limits to protect storage from log thrashing
#  9. Disables USB autosuspend and locks hub power to active 'on'
# ==============================================================================

set -e

# Ensure running as root or with sudo
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or with sudo: sudo $0"
    exit 1
fi

echo "============================================================"
echo " Starting ReloBot Raspberry Pi 5 Performance Optimization "
echo "============================================================"

BOOT_CONFIG="/boot/firmware/config.txt"
BOOT_CMDLINE="/boot/firmware/cmdline.txt"

# Fallback for older boot partition paths if needed
if [ ! -f "$BOOT_CONFIG" ] && [ -f "/boot/config.txt" ]; then
    BOOT_CONFIG="/boot/config.txt"
fi
if [ ! -f "$BOOT_CMDLINE" ] && [ -f "/boot/cmdline.txt" ]; then
    BOOT_CMDLINE="/boot/cmdline.txt"
fi

# ------------------------------------------------------------------------------
# 1. Power & USB Supply Overrides in config.txt
# ------------------------------------------------------------------------------
echo "[1/8] Updating $BOOT_CONFIG (USB current limit & GPU memory)..."

if ! grep -q "^usb_max_current_enable=" "$BOOT_CONFIG"; then
    echo "usb_max_current_enable=1" >> "$BOOT_CONFIG"
    echo "  -> Added usb_max_current_enable=1 (unlocked 1.6A USB current)"
else
    echo "  -> usb_max_current_enable already set."
fi

if ! grep -q "^gpu_mem=" "$BOOT_CONFIG"; then
    echo "gpu_mem=16" >> "$BOOT_CONFIG"
    echo "  -> Added gpu_mem=16 (reclaimed GPU RAM for headless operation)"
else
    echo "  -> gpu_mem already set."
fi

# Ensure Raspberry Pi 5 EEPROM unlocks 1.6A USB current regardless of USB-PD handshake
if command -v rpi-eeprom-config >/dev/null 2>&1; then
    TEMP_EEPROM=$(mktemp)
    rpi-eeprom-config > "$TEMP_EEPROM" 2>/dev/null || true
    if [ -s "$TEMP_EEPROM" ] && ! grep -q "PSU_MAX_CURRENT=5000" "$TEMP_EEPROM"; then
        echo "PSU_MAX_CURRENT=5000" >> "$TEMP_EEPROM"
        rpi-eeprom-config --apply "$TEMP_EEPROM" >/dev/null 2>&1 || true
        echo "  -> Added PSU_MAX_CURRENT=5000 to EEPROM bootloader."
    else
        echo "  -> EEPROM PSU_MAX_CURRENT already configured."
    fi
    rm -f "$TEMP_EEPROM"
fi

# Configure Realtek RTL8822BU Wi-Fi: lock to USB 2 mode (no RF interference) and disable deep sleep
cat << "EOF" > /etc/modprobe.d/rtw88.conf
# ReloBot: Prevent Realtek RTL8822BU Wi-Fi firmware crashes and RF interference
options rtw88_core disable_lps_deep=y
options rtw88_usb switch_usb_mode=n
EOF
echo "  -> Configured /etc/modprobe.d/rtw88.conf (switch_usb_mode=n, disable_lps_deep=y)."

# ------------------------------------------------------------------------------
# 2. Docker Cgroup Memory Accounting in cmdline.txt
# ------------------------------------------------------------------------------
echo "[2/7] Updating $BOOT_CMDLINE for Docker cgroups..."

if ! grep -q "cgroup_enable=memory" "$BOOT_CMDLINE"; then
    sed -i "s/$/ cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1/" "$BOOT_CMDLINE"
    echo "  -> Added Docker cgroup memory/swap accounting."
else
    echo "  -> Docker cgroups already configured."
fi

# ------------------------------------------------------------------------------
# 3. Headless Multi-User Target & Disable Unused Services
# ------------------------------------------------------------------------------
echo "[3/7] Configuring headless target and disabling unused daemons..."

systemctl set-default multi-user.target
systemctl stop lightdm cups cups-browsed ModemManager 2>/dev/null || true
systemctl disable lightdm cups cups-browsed ModemManager 2>/dev/null || true
echo "  -> Switched default target to multi-user.target."
echo "  -> Disabled lightdm (desktop), cups (printing), and ModemManager."

# ------------------------------------------------------------------------------
# 4. Disable MicroSD Swap & Configure ZRAM in RAM
# ------------------------------------------------------------------------------
echo "[4/7] Configuring ZRAM swap (in-memory) & disabling MicroSD swapfile..."

systemctl stop dphys-swapfile 2>/dev/null || true
systemctl disable dphys-swapfile 2>/dev/null || true
dphys-swapfile swapoff 2>/dev/null || true

# Install zram-tools if not installed
if ! dpkg -s zram-tools >/dev/null 2>&1; then
    echo "  -> Installing zram-tools..."
    apt-get update -qq
    DEBIAN_FRONTEND=noninteractive apt-get install -y -qq zram-tools
fi

# Configure zramswap config
cat << "EOF" > /etc/default/zramswap
ALGO=zstd
PERCENT=50
PRIORITY=100
EOF

systemctl restart zramswap.service 2>/dev/null || zramswap start 2>/dev/null || true
echo "  -> ZRAM swap configured (50% RAM with zstd compression)."

# ------------------------------------------------------------------------------
# 5. CPU Performance Governor Service
# ------------------------------------------------------------------------------
echo "[5/7] Setting CPU governor to 'performance'..."

for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -f "$g" ] && echo performance > "$g"
done

cat << "EOF" > /etc/systemd/system/cpu-governor-performance.service
[Unit]
Description=Set CPU scaling governor to performance
After=sysinit.target local-fs.target
DefaultDependencies=no

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do [ -f "$g" ] && echo performance > "$g"; done'
RemainAfterExit=yes

[Install]
WantedBy=sysinit.target
EOF

systemctl daemon-reload
systemctl enable cpu-governor-performance.service
echo "  -> CPU performance governor service enabled on boot."

# ------------------------------------------------------------------------------
# 6. Virtual Memory & FastDDS Sysctl Optimization
# ------------------------------------------------------------------------------
echo "[6/7] Applying kernel sysctl parameters..."

cat << "EOF" > /etc/sysctl.d/98-rpi-perf.conf
# Virtual Memory Tuning (low swappiness, cache pressure)
vm.swappiness = 15
vm.vfs_cache_pressure = 50
vm.dirty_ratio = 10
vm.dirty_background_ratio = 5
EOF

cat << "EOF" > /etc/sysctl.d/99-ros2-fastdds.conf
# FastDDS Network Sockets & UDP Buffer Optimization
net.core.rmem_max = 67108864
net.core.wmem_max = 67108864
net.core.rmem_default = 33554432
net.core.wmem_default = 33554432
net.core.netdev_max_backlog = 10000
net.ipv4.udp_rmem_min = 16384
net.ipv4.udp_wmem_min = 16384
EOF

sysctl --system >/dev/null
echo "  -> Sysctl configuration applied."

# ------------------------------------------------------------------------------
# 7. Systemd Journal Limits
# ------------------------------------------------------------------------------
echo "[7/8] Setting journald limits to protect flash storage..."

mkdir -p /etc/systemd/journald.conf.d
cat << "EOF" > /etc/systemd/journald.conf.d/00-size-limit.conf
[Journal]
SystemMaxUse=100M
RuntimeMaxUse=50M
MaxRetentionSec=7day
EOF
systemctl restart systemd-journald 2>/dev/null || true

# ------------------------------------------------------------------------------
# 8. Disable USB Autosuspend & Hub Low-Power Sleep
# ------------------------------------------------------------------------------
echo "[8/8] Disabling USB autosuspend and configuring hub power control to 'on'..."

if ! grep -q "usbcore.autosuspend=-1" "$BOOT_CMDLINE"; then
    sed -i "s/$/ usbcore.autosuspend=-1/" "$BOOT_CMDLINE"
    echo "  -> Added usbcore.autosuspend=-1 to cmdline.txt."
else
    echo "  -> usbcore.autosuspend=-1 already configured in cmdline.txt."
fi

# Set runtime kernel module parameter if writable
if [ -w /sys/module/usbcore/parameters/autosuspend ]; then
    echo -1 > /sys/module/usbcore/parameters/autosuspend
fi

# Create persistent udev rule to keep all USB devices awake (power/control = on)
cat << "EOF" > /etc/udev/rules.d/50-usb-power.rules
# ReloBot: Prevent USB Hub & Serial devices from entering low-power autosuspend
ACTION=="add", SUBSYSTEM=="usb", TEST=="power/control", ATTR{power/control}="on"
EOF
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger --subsystem-match=usb 2>/dev/null || true

# Apply immediately to all currently enumerated USB devices
for d in /sys/bus/usb/devices/*/power/control; do
    [ -f "$d" ] && echo on > "$d" 2>/dev/null || true
done
echo "  -> USB autosuspend disabled globally and udev rule 50-usb-power.rules installed."

echo "============================================================"
echo " ReloBot Raspberry Pi 5 Optimization Complete! "
echo "============================================================"
echo "Note: A reboot (sudo reboot) is recommended to activate all"
echo "config.txt and cmdline.txt parameters."
