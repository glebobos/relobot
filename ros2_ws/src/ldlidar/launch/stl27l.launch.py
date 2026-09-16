#!/usr/bin/env python3
import os
import glob
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def find_lidar_port():
    # 1. Environment variable override
    env_port = os.environ.get('LIDAR_PORT')
    if env_port and os.path.exists(env_port):
        return env_port

    # 2. Match by-id CP210x or Silicon Labs UART converter
    by_id_matches = glob.glob('/dev/serial/by-id/*CP210*') + glob.glob('/dev/serial/by-id/*Silicon_Labs*')
    for dev in by_id_matches:
        if os.path.exists(dev):
            return os.path.realpath(dev)

    # 3. Match any existing /dev/ttyUSB* device
    tty_usb_devices = sorted(glob.glob('/dev/ttyUSB*'))
    if tty_usb_devices:
        return tty_usb_devices[0]

    # 4. Fallback default
    return '/dev/ttyUSB0'

def generate_launch_description():
  port_name = find_lidar_port()
  print(f"[STL27L Launch] Using LiDAR serial port: {port_name}")

  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar_node',
      name='STL27L',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_STL27L'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_laser'},
        {'port_name': port_name},
        {'port_baudrate': 921600},
        {'laser_scan_dir': False},
        {'enable_angle_crop_func': False},
        {'angle_crop_min': 0.0},
        {'angle_crop_max': 0.0}
      ]
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  return ld
