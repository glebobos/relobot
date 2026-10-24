#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import dbus
    HAVE_DBUS = True
except ImportError:
    HAVE_DBUS = False


def get_wifi_ssid():
    """
    Retrieve currently connected Wi-Fi SSID from NetworkManager via D-Bus system bus.
    """
    if not HAVE_DBUS:
        return None
    try:
        bus = dbus.SystemBus()
        nm = bus.get_object('org.freedesktop.NetworkManager', '/org/freedesktop/NetworkManager')
        props = dbus.Interface(nm, 'org.freedesktop.DBus.Properties')
        active_paths = props.Get('org.freedesktop.NetworkManager', 'ActiveConnections')
        for path in active_paths:
            conn = bus.get_object('org.freedesktop.NetworkManager', path)
            c_props = dbus.Interface(conn, 'org.freedesktop.DBus.Properties')
            c_type = c_props.Get('org.freedesktop.NetworkManager.Connection.Active', 'Type')
            if str(c_type) == '802-11-wireless':
                ssid = c_props.Get('org.freedesktop.NetworkManager.Connection.Active', 'Id')
                return str(ssid)
    except Exception:
        pass
    return None


def get_wifi_signal():
    """
    Parse /proc/net/wireless to extract wireless link signal strength (dBm).
    """
    valid_signals = []
    try:
        with open('/proc/net/wireless', 'r') as f:
            lines = f.readlines()
        for line in lines:
            if ':' in line:
                parts = line.split(':')
                iface = parts[0].strip()
                if iface.startswith('wl'):
                    data_parts = parts[1].split()
                    if len(data_parts) >= 2:
                        for idx in (2, 1):
                            if idx < len(data_parts):
                                try:
                                    val = int(data_parts[idx].rstrip('.'))
                                    if -100 <= val <= -10:
                                        valid_signals.append(val)
                                        break
                                except ValueError:
                                    pass
        if valid_signals:
            return max(valid_signals)
    except Exception:
        pass
    return None


def get_memory_usage():
    """
    Calculate RAM percentage usage from /proc/meminfo.
    """
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
    except Exception:
        pass
    return 0.0


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher = self.create_publisher(String, '/system/metrics', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.command_sub = self.create_subscription(
            String,
            '/system/command',
            self.command_callback,
            10
        )
        self.prev_idle = 0
        self.prev_total = 0
        self.has_prev = False

        # Cache Wi-Fi SSID to avoid blocking D-Bus calls every second
        self.cached_wifi_ssid = None
        self.wifi_ssid_poll_counter = 0

        self.get_logger().info("System Monitor Node initialized with D-Bus host command listener.")

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == 'reboot':
            self._reboot_pi()
        elif command == 'poweroff':
            self._poweroff_pi()
        else:
            self.get_logger().warn(f"Unknown system command: {command}")

    def _reboot_pi(self):
        self.get_logger().info('Rebooting Raspberry Pi via D-Bus systemd-logind...')
        if not HAVE_DBUS:
            self.get_logger().error('dbus module not available, cannot reboot')
            return
        try:
            bus = dbus.SystemBus()
            login1 = bus.get_object('org.freedesktop.login1', '/org/freedesktop/login1')
            manager = dbus.Interface(login1, 'org.freedesktop.login1.Manager')
            manager.Reboot(True)
        except Exception as e:
            self.get_logger().error(f'Failed to send D-Bus reboot command: {e}')

    def _poweroff_pi(self):
        self.get_logger().info('Powering off Raspberry Pi via D-Bus systemd-logind...')
        if not HAVE_DBUS:
            self.get_logger().error('dbus module not available, cannot power off')
            return
        try:
            bus = dbus.SystemBus()
            login1 = bus.get_object('org.freedesktop.login1', '/org/freedesktop/login1')
            manager = dbus.Interface(login1, 'org.freedesktop.login1.Manager')
            manager.PowerOff(True)
        except Exception as e:
            self.get_logger().error(f'Failed to send D-Bus poweroff command: {e}')

    def timer_callback(self):
        subs = self.publisher.get_subscription_count()
        if subs == 0:
            self.has_prev = False
            return

        # RAM & Wi-Fi
        ram_percent = get_memory_usage()
        wifi_dbm = get_wifi_signal()

        # Query Wi-Fi SSID every 10 seconds or immediately on first run
        if self.cached_wifi_ssid is None or self.wifi_ssid_poll_counter % 10 == 0:
            self.cached_wifi_ssid = get_wifi_ssid()
        self.wifi_ssid_poll_counter += 1
        wifi_ssid = self.cached_wifi_ssid

        # CPU calculation
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
            "wifi_dbm": wifi_dbm,
            "wifi_ssid": wifi_ssid
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
