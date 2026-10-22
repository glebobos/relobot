import argparse
import glob
import json
import os
from pathlib import Path
import re
import sys


DEFAULT_CONFIG = Path(__file__).with_name('zenoh_devices.json')


def unique_keys(pairs):
    result = {}
    for name, value in pairs:
        if name in result:
            raise ValueError(f'duplicate JSON key: {name}')
        result[name] = value
    return result


def load_devices(config=DEFAULT_CONFIG):
    with Path(config).open() as source:
        settings = json.load(source, object_pairs_hook=unique_keys)
    if not isinstance(settings, dict) or set(settings) != {'devices'}:
        raise ValueError('expected a JSON object containing only "devices"')
    entries = settings['devices']
    if not isinstance(entries, dict):
        raise ValueError('"devices" must be an object keyed by device name')
    devices = {}
    identities = set()
    for name, entry in entries.items():
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', name):
            raise ValueError(f'invalid device name: {name!r}')
        if not isinstance(entry, dict) or set(entry) - {'serial_by_id', 'baudrate', 'enabled'}:
            raise ValueError(f'{name}: expected serial_by_id, baudrate and optional enabled')
        enabled = entry.get('enabled', True)
        if not isinstance(enabled, bool):
            raise ValueError(f'{name}.enabled must be true or false')
        if not enabled:
            continue
        identity = entry.get('serial_by_id')
        if not isinstance(identity, str) or not re.fullmatch(r'usb-[A-Za-z0-9_.:+-]+', identity):
            raise ValueError(f'{name}.serial_by_id must be a USB by-id basename (usb-...), '
                             'not a path; copy its exact name from /dev/serial/by-id/')
        baudrate = entry.get('baudrate')
        if type(baudrate) is not int or not 0 < baudrate <= 4000000:
            raise ValueError(f'{name}.baudrate must be an integer between 1 and 4000000')
        if identity in identities:
            raise ValueError(f'{name}: duplicate serial_by_id: {identity}')
        identities.add(identity)
        devices[name] = {'serial_by_id': identity, 'baudrate': baudrate}
    if not devices:
        raise ValueError('native Pico-ROS mode requires at least one enabled device')
    return devices


def router_override(config=DEFAULT_CONFIG):
    endpoints = ['tcp/[::]:7447']
    for device in load_devices(config).values():
        endpoints.append(f'serial//hostdev/serial/by-id/{device["serial_by_id"]}'
                         f'#baudrate={device["baudrate"]}')
    return 'listen/endpoints=' + json.dumps(endpoints, separators=(',', ':'))


def micro_ros_serial_devices(config=DEFAULT_CONFIG, native_enabled=True):
    native_devices = load_devices(config) if native_enabled else {}
    excluded = set()
    for device in native_devices.values():
        native_path = f'/dev/serial/by-id/{device["serial_by_id"]}'
        if not os.path.exists(native_path):
            raise RuntimeError(f'Cannot exclude missing native Pico-ROS device: {native_path}')
        excluded.add(os.path.realpath(native_path))
    by_id = sorted(set(
        glob.glob('/dev/serial/by-id/usb-*Pico*') +
        glob.glob('/dev/serial/by-id/usb-*ReloBot*')
    ))
    eligible_by_id = [path for path in by_id
                      if os.path.exists(path) and os.path.realpath(path) not in excluded]
    candidates = eligible_by_id or sorted(glob.glob('/dev/ttyACM*'))
    candidates += [value for key, value in sorted(os.environ.items())
                   if key.startswith('TTY_') and value]
    devices = []
    identities = set()
    for candidate in candidates:
        identity = os.path.realpath(candidate)
        if os.path.exists(candidate) and identity not in excluded and identity not in identities:
            devices.append(candidate)
            identities.add(identity)
    return devices


def main():
    parser = argparse.ArgumentParser(description='Load native Pico-ROS USB devices from JSON.')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('validate')
    commands.add_parser('router-override')
    selection = commands.add_parser('micro-ros-devices')
    selection.add_argument('--native-enabled', choices=('true', 'false'), default='true')
    arguments = parser.parse_args()
    try:
        if arguments.command == 'validate':
            devices = load_devices(arguments.config)
            print('Pico-ROS USB: ' + ', '.join(
                f'{name}={device["serial_by_id"]}' for name, device in devices.items()))
        elif arguments.command == 'router-override':
            print(router_override(arguments.config))
        else:
            print(json.dumps(micro_ros_serial_devices(
                arguments.config, native_enabled=arguments.native_enabled == 'true')))
    except (OSError, ValueError, RuntimeError) as error:
        print(f'Error: {arguments.config}: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
