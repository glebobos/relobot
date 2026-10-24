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
#  6. Unlocks POSIX shared memory (memlock unlimited) for Zenoh SHM in PAM, systemd & Docker daemon
#  7. Tunes virtual memory & kernel network sysctl (TCP Westwood+, FQ-CoDel AQM, anti-bufferbloat)
#  8. Disables Wi-Fi power saving globally (802.11 PSM) for low-latency wireless
#  9. Sets up systemd journal limits to protect storage from log thrashing
# 10. Disables USB autosuspend and locks hub power to active 'on'
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
echo "[1/10] Updating $BOOT_CONFIG (USB current limit & GPU memory)..."

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

# Configure Realtek RTL8822BU Wi-Fi: lock to USB 2 mode (no RF interference), disable deep sleep and beamformee stalls
cat << "EOF" > /etc/modprobe.d/rtw88.conf
# ReloBot: Prevent Realtek RTL8822BU Wi-Fi firmware crashes, TX report stalls, and RF interference
options rtw88_core disable_lps_deep=y support_bf=n
options rtw88_usb switch_usb_mode=n
EOF
echo "  -> Configured /etc/modprobe.d/rtw88.conf (switch_usb_mode=n, disable_lps_deep=y, support_bf=n)."

# ------------------------------------------------------------------------------
# 2. Docker Cgroup Memory Accounting in cmdline.txt
# ------------------------------------------------------------------------------
echo "[2/10] Updating $BOOT_CMDLINE for Docker cgroups..."

if ! grep -q "cgroup_enable=memory" "$BOOT_CMDLINE"; then
    sed -i "s/$/ cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1/" "$BOOT_CMDLINE"
    echo "  -> Added Docker cgroup memory/swap accounting."
else
    echo "  -> Docker cgroups already configured."
fi

# ------------------------------------------------------------------------------
# 3. Headless Multi-User Target & Disable Unused Services
# ------------------------------------------------------------------------------
echo "[3/10] Configuring headless target and disabling unused daemons..."

systemctl set-default multi-user.target
systemctl stop lightdm cups cups-browsed ModemManager 2>/dev/null || true
systemctl disable lightdm cups cups-browsed ModemManager 2>/dev/null || true
echo "  -> Switched default target to multi-user.target."
echo "  -> Disabled lightdm (desktop), cups (printing), and ModemManager."

# ------------------------------------------------------------------------------
# 4. Disable MicroSD Swap & Configure ZRAM in RAM
# ------------------------------------------------------------------------------
echo "[4/10] Configuring ZRAM swap (in-memory) & disabling MicroSD swapfile..."

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
echo "[5/10] Setting CPU governor to 'performance'..."

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
# 6. Unlock POSIX Shared Memory (memlock) for Zenoh SHM & Docker Containers
# ------------------------------------------------------------------------------
echo "[6/10] Configuring unlimited locked memory (memlock) for Zenoh SHM & Docker..."

# Configure PAM limits for user and root
cat << "EOF" > /etc/security/limits.d/99-ros2-memlock.conf
# ReloBot: Unlimited locked memory for ROS 2 Zenoh POSIX Shared Memory
* soft memlock unlimited
* hard memlock unlimited
root soft memlock unlimited
root hard memlock unlimited
EOF

# Configure systemd default service manager limits
mkdir -p /etc/systemd/system.conf.d
cat << "EOF" > /etc/systemd/system.conf.d/10-memlock.conf
[Manager]
DefaultLimitMEMLOCK=infinity
EOF
systemctl daemon-reload

# Configure Docker daemon default-ulimits for memlock
DOCKER_DAEMON_JSON="/etc/docker/daemon.json"
mkdir -p /etc/docker
/usr/bin/python3 -c "
import json, os

path = '$DOCKER_DAEMON_JSON'
cfg = {}
if os.path.exists(path):
    try:
        with open(path, 'r') as f:
            cfg = json.load(f)
    except Exception:
        cfg = {}

ulimits = cfg.setdefault('default-ulimits', {})
ulimits['memlock'] = {'Name': 'memlock', 'Hard': -1, 'Soft': -1}

with open(path, 'w') as f:
    json.dump(cfg, f, indent=2)
"
# Reload Docker daemon to apply default ulimits if running
systemctl reload docker 2>/dev/null || true
echo "  -> Configured /etc/security/limits.d/99-ros2-memlock.conf."
echo "  -> Configured /etc/systemd/system.conf.d/10-memlock.conf."
echo "  -> Updated /etc/docker/daemon.json with default-ulimits.memlock."

# ------------------------------------------------------------------------------
# 7. Virtual Memory & Network Sysctl Optimization (Westwood+ & FQ-CoDel AQM)
# ------------------------------------------------------------------------------
echo "[7/10] Applying kernel sysctl parameters (TCP Westwood+ & FQ-CoDel AQM)..."

cat << "EOF" > /etc/sysctl.d/98-rpi-perf.conf
# Virtual Memory Tuning (low swappiness, cache pressure)
vm.swappiness = 15
vm.vfs_cache_pressure = 50
vm.dirty_ratio = 10
vm.dirty_background_ratio = 5
EOF

# Ensure TCP Westwood+ and FQ-CoDel kernel modules are loaded on boot
modprobe tcp_westwood 2>/dev/null || true
modprobe sch_fq_codel 2>/dev/null || true
cat << "EOF" > /etc/modules-load.d/ros2-network.conf
tcp_westwood
sch_fq_codel
EOF
rm -f /etc/modules-load.d/bbr.conf 2>/dev/null || true

# Remove legacy/conflicting FastDDS network configuration if present
rm -f /etc/sysctl.d/99-ros2-fastdds.conf 2>/dev/null || true

cat << "EOF" > /etc/sysctl.d/99-ros2-network.conf
# ReloBot: Low-Latency Wireless Network Tuning (TCP Westwood+ & FQ-CoDel AQM)
net.core.default_qdisc = fq_codel
net.ipv4.tcp_congestion_control = westwood
net.core.rmem_max = 33554432
net.core.wmem_max = 16777216
net.core.rmem_default = 1048576
net.core.wmem_default = 262144
net.ipv4.tcp_rmem = 4096 87380 16777216
net.ipv4.tcp_wmem = 4096 65536 4194304
net.core.netdev_max_backlog = 10000
net.ipv4.udp_rmem_min = 16384
net.ipv4.udp_wmem_min = 16384
EOF

sysctl --system >/dev/null
echo "  -> Sysctl configuration applied (TCP Westwood+, FQ-CoDel AQM, bufferbloat mitigation)."

# ------------------------------------------------------------------------------
# 8. Disable Wi-Fi Power Saving (802.11 PSM) for Deterministic Wireless
# ------------------------------------------------------------------------------
echo "[8/10] Disabling Wi-Fi power saving globally..."

# NetworkManager power save override (2 = disable)
mkdir -p /etc/NetworkManager/conf.d
cat << "EOF" > /etc/NetworkManager/conf.d/99-wifi-powersave.conf
[connection]
wifi.powersave = 2
EOF

# Persistent udev rule for all current and future wireless interfaces
cat << "EOF" > /etc/udev/rules.d/70-wifi-powersave.rules
# ReloBot: Disable 802.11 power saving on all wireless interfaces for low latency
ACTION=="add", SUBSYSTEM=="net", KERNEL=="wlan*", RUN+="/usr/sbin/iw dev %k set power_save off"
EOF
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger --subsystem-match=net 2>/dev/null || true

# Apply immediately to all enumerated wireless interfaces
for iface in /sys/class/net/wlan*; do
    if [ -e "$iface" ]; then
        dev_name=$(basename "$iface")
        iw dev "$dev_name" set power_save off 2>/dev/null || true
    fi
done
echo "  -> Wi-Fi power saving disabled (NetworkManager, udev, and runtime interfaces)."

# ------------------------------------------------------------------------------
# 9. Systemd Journal Limits
# ------------------------------------------------------------------------------
echo "[9/10] Setting journald limits to protect flash storage..."

mkdir -p /etc/systemd/journald.conf.d
cat << "EOF" > /etc/systemd/journald.conf.d/00-size-limit.conf
[Journal]
SystemMaxUse=100M
RuntimeMaxUse=50M
MaxRetentionSec=7day
EOF
systemctl restart systemd-journald 2>/dev/null || true

# ------------------------------------------------------------------------------
# 10. Disable USB Autosuspend & Hub Low-Power Sleep
# ------------------------------------------------------------------------------
echo "[10/10] Disabling USB autosuspend and configuring hub power control to 'on'..."

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
