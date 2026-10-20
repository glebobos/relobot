#!/usr/bin/env python3
# Copyright 2026 ReloBot Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Micro-ROS FastDDS <-> Zenoh bidirectional relay bridge.

Bridges raw Micro-ROS serial topics (running on FastDDS) with the
native ROS 2 Zenoh middleware (rmw_zenoh_cpp) using zero-copy / low-latency
IPC over a local Unix domain socket pair.
"""

import os
import sys
import struct
import socket
import signal
import queue
import threading
import subprocess

# Topic definitions: (id, topic_name, msg_module, msg_class_name, direction)
# direction:
#   'pico_to_zenoh': Micro-ROS FastDDS publisher -> Zenoh subscriber
#   'zenoh_to_pico': Zenoh publisher -> Micro-ROS FastDDS subscriber
TOPIC_SPECS = [
    (0, "/robot_joint_states", "sensor_msgs.msg", "JointState", "pico_to_zenoh"),
    (1, "/imu", "sensor_msgs.msg", "Imu", "pico_to_zenoh"),
    (2, "/battery_voltage", "std_msgs.msg", "Float32", "pico_to_zenoh"),
    (3, "/charger_voltage", "std_msgs.msg", "Float32", "pico_to_zenoh"),
    (4, "/on_dock", "std_msgs.msg", "Bool", "pico_to_zenoh"),
    (6, "/robot_joint_commands", "sensor_msgs.msg", "JointState", "zenoh_to_pico"),
]


def load_msg_class(module_name, class_name):
    import importlib
    mod = importlib.import_module(module_name)
    return getattr(mod, class_name)


def send_exact(sock, data):
    total = 0
    while total < len(data):
        sent = sock.send(data[total:])
        if sent == 0:
            raise BrokenPipeError("Socket connection closed")
        total += sent


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def run_worker(mode, sock):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.serialization import serialize_message, deserialize_message

    node_name = f"uros_bridge_{mode}"
    rclpy.init()
    node = Node(node_name)
    node.get_logger().info(f"Starting uros_bridge worker [{mode}] on RMW: {os.environ.get('RMW_IMPLEMENTATION', 'default')}")

    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    reliable_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    send_queue = queue.Queue(maxsize=100)
    publishers = {}

    for topic_id, topic_name, mod_name, cls_name, direction in TOPIC_SPECS:
        msg_cls = load_msg_class(mod_name, cls_name)
        if mode == "dds":
            if direction == "pico_to_zenoh":
                # Subscribe on FastDDS from Pico, relay to Zenoh
                def make_cb(tid):
                    return lambda msg: send_queue.put((tid, serialize_message(msg)), block=False)
                node.create_subscription(msg_cls, topic_name, make_cb(topic_id), sensor_qos)
            else:
                # Publish on FastDDS to Pico from Zenoh
                publishers[topic_id] = (node.create_publisher(msg_cls, topic_name, reliable_qos), msg_cls)
        else:  # mode == "zenoh"
            if direction == "pico_to_zenoh":
                # Publish on Zenoh to robot stack from Pico
                publishers[topic_id] = (node.create_publisher(msg_cls, topic_name, reliable_qos), msg_cls)
            else:
                # Subscribe on Zenoh from robot stack, relay to Pico
                def make_cb(tid):
                    return lambda msg: send_queue.put((tid, serialize_message(msg)), block=False)
                node.create_subscription(msg_cls, topic_name, make_cb(topic_id), reliable_qos)

    # Socket sender thread
    def socket_writer():
        while rclpy.ok():
            try:
                tid, payload = send_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if tid is None:
                break
            try:
                header = struct.pack("!BI", tid, len(payload))
                send_exact(sock, header + payload)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    # Socket receiver thread
    def socket_reader():
        while rclpy.ok():
            try:
                header = recv_exact(sock, 5)
                if not header:
                    break
                tid, length = struct.unpack("!BI", header)
                payload = recv_exact(sock, length)
                if not payload:
                    break
                if tid in publishers:
                    pub, msg_cls = publishers[tid]
                    msg = deserialize_message(payload, msg_cls)
                    pub.publish(msg)
            except (BrokenPipeError, ConnectionResetError, OSError):
                break

    writer_t = threading.Thread(target=socket_writer, daemon=True)
    reader_t = threading.Thread(target=socket_reader, daemon=True)
    writer_t.start()
    reader_t.start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        send_queue.put((None, b""))
        try:
            sock.shutdown(socket.SHUT_RDWR)
            sock.close()
        except OSError:
            pass
        node.destroy_node()
        rclpy.shutdown()


def main():
    if "--worker-dds" in sys.argv:
        fd = int(sys.argv[sys.argv.index("--worker-dds") + 1])
        sock = socket.fromfd(fd, socket.AF_UNIX, socket.SOCK_STREAM)
        os.close(fd)
        run_worker("dds", sock)
        return

    # Parent process: runs the Zenoh worker and spawns the FastDDS child worker
    sock_parent, sock_child = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)

    child_env = os.environ.copy()
    child_env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    child_env["FASTRTPS_DEFAULT_PROFILES_FILE"] = "/ros2_ws/fastdds_localhost.xml"

    child_proc = subprocess.Popen(
        [sys.executable, os.path.abspath(__file__), "--worker-dds", str(sock_child.fileno())],
        env=child_env,
        close_fds=False,
        pass_fds=[sock_child.fileno()],
    )
    sock_child.close()

    def handle_signal(sig, frame):
        if child_proc.poll() is None:
            child_proc.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        # Parent runs Zenoh worker
        os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"
        run_worker("zenoh", sock_parent)
    finally:
        if child_proc.poll() is None:
            child_proc.terminate()
            child_proc.wait(timeout=2.0)


if __name__ == "__main__":
    main()
