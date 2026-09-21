#!/usr/bin/env bash
set -u

OUT="/home/admin/projects/relobot/deep_output.txt"
exec > "$OUT" 2>&1

echo "==================== 1. PREVIOUS BOOT LOGS (IF AVAILABLE) ===================="
journalctl -b -1 -n 60 --no-pager 2>/dev/null || echo "No previous journal available"

echo "==================== 2. SYSTEMD CRASHES / KERNEL OOPS ===================="
journalctl -b 0 -p 3 --no-pager 2>/dev/null || true

echo "==================== 3. FULL MICRO-ROS & DIFF_ROBOT LOGS ===================="
docker logs ros2_diff_robot 2>&1 | grep -E -i "error|warn|disconnect|timeout|fail|drop|abort|stuck|reset" | tail -n 50 || true

echo "==================== 4. FULL NAV2 LOGS (WARNINGS & ERRORS) ===================="
docker logs ros2_nav2 2>&1 | grep -E -i "error|warn|abort|fail|recover|stuck|timeout|clearing|costmap" | tail -n 50 || true

echo "==================== 5. CAMERA CONTAINER LOGS (WARNINGS & ERRORS) ===================="
docker logs ros2_camera_rp 2>&1 | grep -E -i "error|warn|drop|slow|timeout|fail" | tail -n 50 || true

echo "==================== 6. ROSBRIDGE LOGS (WARNINGS & ERRORS) ===================="
docker logs ros2_rosbridge 2>&1 | grep -E -i "error|warn|disconnect|closed|fail" | tail -n 50 || true

echo "==================== 7. FASTDDS CONFIGURATION & ENV ===================="
for c in ros2_diff_robot ros2_nav2 ros2_camera_rp ros2_lidar ros2_rosbridge; do
    echo "--- Env in $c ---"
    docker exec "$c" env 2>/dev/null | grep -E "FASTRTPS|ROS_DOMAIN_ID|RMW|CYCLONE" || true
done

echo "==================== 8. ACTIVE ROS2 TOPIC RATES ===================="
docker exec ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /camera/image_raw" 2>&1 || true
docker exec ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /camera/image_rect" 2>&1 || true
docker exec ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /scan" 2>&1 || true
docker exec ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /diff_drive_controller/cmd_vel_unstamped" 2>&1 || true
docker exec ros2_diff_robot bash -c "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic hz /diff_drive_controller/odom" 2>&1 || true

echo "==================== 9. NETWORK SOCKETS & TRAFFIC BY PORT ===================="
ss -tulpn || true

echo "==================== 10. WIREGUARD STATUS ===================="
sudo wg show 2>/dev/null || wg show 2>/dev/null || true
ip route show || true

echo "==================== DEEP CHECK COMPLETE ===================="
