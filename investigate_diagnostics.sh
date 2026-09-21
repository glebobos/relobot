#!/usr/bin/env bash
set -u

OUT="/home/admin/projects/relobot/diag_output.txt"
exec > "$OUT" 2>&1

echo "==================== 1. SYSTEM UPTIME & THROTTLING ===================="
uptime
if command -v vcgencmd >/dev/null 2>&1; then
    echo "--- vcgencmd get_throttled ---"
    vcgencmd get_throttled || true
    echo "--- vcgencmd measure_temp ---"
    vcgencmd measure_temp || true
    echo "--- vcgencmd measure_volts ---"
    vcgencmd measure_volts || true
    echo "--- vcgencmd get_config total_mem ---"
    vcgencmd get_config total_mem || true
else
    echo "vcgencmd not found or not permitted"
fi

echo "==================== 2. KERNEL DMESG & USB ISSUES ===================="
echo "--- Last 50 dmesg lines matching usb/error/tty/reset/fail/warn ---"
dmesg -T | grep -E -i "usb|disconnect|reset|error|ttyACM|ttyUSB|under-voltage|throttled|timeout|overcurrent" | tail -n 50 || true

echo "--- Recent 30 dmesg lines (raw tail) ---"
dmesg -T | tail -n 30 || true

echo "==================== 3. CONNECTED USB DEVICES & TTYS ===================="
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
lsusb || true
lsusb -t || true

echo "==================== 4. NETWORK & UDP SOCKET BUFFER STATS ===================="
echo "--- Wireless stats (iwconfig / nmcli) ---"
iwconfig 2>/dev/null || true
echo "--- IP & Routes ---"
ip -s link || true
echo "--- UDP Statistics (packet drops, buffer overflows) ---"
netstat -su 2>/dev/null || ss -u -a 2>/dev/null || true
echo "--- FastDDS Sysctl settings ---"
sysctl net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default net.ipv4.udp_mem || true

echo "==================== 5. DOCKER CONTAINERS STATUS ===================="
docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.RunningFor}}\t{{.Ports}}"

echo "==================== 6. DOCKER LOGS: ROS2_DIFF_ROBOT ===================="
docker logs --tail 40 ros2_diff_robot 2>&1 || true

echo "==================== 7. DOCKER LOGS: ROS2_CAMERA_RP ===================="
docker logs --tail 40 ros2_camera_rp 2>&1 || true

echo "==================== 8. DOCKER LOGS: ROS2_ROSBRIDGE ===================="
docker logs --tail 40 ros2_rosbridge 2>&1 || true

echo "==================== 9. DOCKER LOGS: ROS2_NAV2 ===================="
docker logs --tail 40 ros2_nav2 2>&1 || true

echo "==================== 10. DOCKER LOGS: ROS2_LIDAR ===================="
docker logs --tail 30 ros2_lidar 2>&1 || true

echo "==================== DIAGNOSTICS COMPLETE ===================="
