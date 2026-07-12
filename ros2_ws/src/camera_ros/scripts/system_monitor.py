#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def get_wifi_signal():
    try:
        with open('/proc/net/wireless', 'r') as f:
            lines = f.readlines()
        for line in lines:
            if ':' in line:
                parts = line.split(':')
                iface = parts[0].strip()
                if iface.startswith('wl'):
                    data_parts = parts[1].split()
                    if len(data_parts) >= 3:
                        try:
                            return int(data_parts[2].rstrip('.'))
                        except ValueError:
                            pass
                    break
    except Exception:
        pass
    return None

def get_memory_usage():
    try:
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()
        mem_info = {}
        for line in lines:
            parts = line.split()
            if len(parts) >= 2:
                key = parts[0].rstrip(':')
                val = int(parts[1])
                mem_info[key] = val
        
        total = mem_info.get('MemTotal', 0)
        free = mem_info.get('MemFree', 0)
        buffers = mem_info.get('Buffers', 0)
        cached = mem_info.get('Cached', 0)
        
        if total > 0:
            # MemUsed = MemTotal - MemFree - Buffers - Cached
            used = total - free - buffers - cached
            percent = (used / total) * 100.0
            return round(percent, 1)
    except Exception as e:
        print(f"Error reading meminfo: {e}")
    return 0.0

class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher = self.create_publisher(String, '/system/metrics', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.prev_idle = 0
        self.prev_total = 0
        self.has_prev = False
        self.get_logger().info("System Monitor Node initialized.")

    def timer_callback(self):
        subs = self.publisher.get_subscription_count()
        if subs == 0:
            self.has_prev = False
            return

        # Calculate memory
        ram_percent = get_memory_usage()

        # Calculate WiFi info
        wifi_dbm = get_wifi_signal()

        # Calculate CPU
        cpu_percent = 0.0
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
            parts = line.split()
            if len(parts) >= 5:
                user = int(parts[1])
                nice = int(parts[2])
                system = int(parts[3])
                idle = int(parts[4])
                iowait = int(parts[5]) if len(parts) > 5 else 0
                irq = int(parts[6]) if len(parts) > 6 else 0
                softirq = int(parts[7]) if len(parts) > 7 else 0
                steal = int(parts[8]) if len(parts) > 8 else 0

                idle_time = idle + iowait
                non_idle_time = user + nice + system + irq + softirq + steal
                total_time = idle_time + non_idle_time

                if self.has_prev:
                    diff_idle = idle_time - self.prev_idle
                    diff_total = total_time - self.prev_total
                    if diff_total > 0:
                        cpu_percent = round((1.0 - (diff_idle / diff_total)) * 100.0, 1)
                
                self.prev_idle = idle_time
                self.prev_total = total_time
                self.has_prev = True
        except Exception as e:
            self.get_logger().error(f"Error reading CPU stat: {e}")

        # Publish metrics
        msg = String()
        msg.data = json.dumps({
            "cpu": cpu_percent,
            "ram": ram_percent,
            "wifi_dbm": wifi_dbm
        })
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
