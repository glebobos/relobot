#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile
import time

from serial_listener_hotplug import serial_descriptors


def router_config(directory, name, override=None, serial_target=None, startup=None):
    environment = os.environ.copy()
    environment.pop('ZENOH_ROUTER_CONFIG_URI', None)
    environment.pop('ZENOH_CONFIG_OVERRIDE', None)
    environment['RUST_LOG'] = 'debug'
    environment['NO_COLOR'] = '1'
    environment['PICO_KNIVES_ENABLED'] = 'false'
    if override is not None:
        environment['ZENOH_CONFIG_OVERRIDE'] = override
    log_path = directory / f'{name}.log'
    forced_cleanup = False
    with log_path.open('w') as output:
        command = (['bash', str(startup)] if startup else
                   ['/opt/ros/lyrical/lib/rmw_zenoh_cpp/rmw_zenohd'])
        process = subprocess.Popen(
            command,
            env=environment, stdout=output, stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                text = log_path.read_text()
                ready = ('Started Zenoh router' in text if serial_target is None else
                         serial_target in serial_descriptors(process.pid).values())
                if 'Config: Config(' in text and ready:
                    break
                if process.poll() is not None:
                    raise AssertionError(f'{name} exited with {process.returncode}: {text}')
                time.sleep(0.02)
            else:
                raise AssertionError(f'{name} did not initialize: {log_path.read_text()}')
        finally:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                forced_cleanup = True
                process.kill()
                process.wait(timeout=2)
    config = next(line.split('Config: ', 1)[1] for line in log_path.read_text().splitlines()
                  if 'Config: Config(' in line)
    return config, forced_cleanup


def without_listen_endpoints(config):
    prefix, listen = config.split('listen: ListenConfig {', 1)
    normalized, count = re.subn(r'endpoints: Unique\(\[.*?\]\), exit_on_failure:',
                                'endpoints: <test-listeners>, exit_on_failure:', listen, count=1)
    assert count == 1
    return prefix + 'listen: ListenConfig {' + normalized


def startup_override(script, enabled):
    environment = os.environ.copy()
    environment.update({
        'PICO_KNIVES_ENABLED': enabled,
        'ZENOH_CONFIG_OVERRIDE': 'listen/endpoints=["serial/stale"]',
    })
    with tempfile.TemporaryDirectory(prefix='router-device-config-') as temporary:
        config = Path(temporary) / 'zenoh_devices.json'
        config.write_text(json.dumps({'devices': {'knives': {
            'serial_by_id': 'usb-test-knives', 'baudrate': 115200,
        }}}))
        result = subprocess.run(
            ['bash', '-c', 'source() { :; }; exec() { env -0; }; '
             'export -f source exec; bash "$1" "$2"', 'router-config-test', str(script), str(config)],
            env=environment, capture_output=True, text=True, check=True, timeout=5,
        )
    configured = dict(entry.split('=', 1) for entry in result.stdout.split('\0') if entry)
    return configured.get('ZENOH_CONFIG_OVERRIDE')


def main():
    parser = argparse.ArgumentParser(description='Compare actual ROS router defaults with unified startup.')
    parser.add_argument('--router-startup', type=Path, required=True)
    arguments = parser.parse_args()
    if not Path('/.dockerenv').is_file():
        parser.error('run inside a network-isolated Docker container, without host devices')
    assert startup_override(arguments.router_startup, 'false') is None
    override = startup_override(arguments.router_startup, 'true')
    assert override is not None, 'missing native listeners'
    master, slave = os.openpty()
    try:
        target = os.ttyname(slave)
        with tempfile.TemporaryDirectory(prefix='knives-router-config-') as temporary:
            directory = Path(temporary)
            serial_path = directory / 'usb-test-knives'
            serial_path.symlink_to(target)
            override = override.replace('/hostdev/serial/by-id/usb-test-knives', str(serial_path))
            baseline, baseline_forced = router_config(directory, 'baseline')
            simulation, simulation_forced = router_config(
                directory, 'simulation', startup=arguments.router_startup)
            candidate, serial_forced = router_config(directory, 'serial', override, target)
            assert baseline == simulation, 'simulation startup changed installed ROS defaults'
            assert 'tcp/[::]:7447' in candidate
            assert f'serial/{serial_path}#baudrate=115200' in candidate
            for expected in ('open_timeout: 60000', 'accept_timeout: 60000',
                             'accept_pending: 10000', 'max_sessions: 10000',
                             'lease: 60000, keep_alive: 2', 'ShmConf { enabled: false',
                             'control: 2, real_time: 2, interactive_high: 2, interactive_low: 2, '
                             'data_high: 2, data: 2, data_low: 2, background: 2'):
                assert expected in baseline, f'ROS baseline changed: {expected}'
            assert without_listen_endpoints(baseline) == without_listen_endpoints(candidate), (
                'the serial override changed settings other than listen/endpoints')
            print(json.dumps({'passed': True, 'only_listen_endpoints_changed': True,
                              'simulation_matches_defaults': True,
                              'forced_cleanup': {'baseline': baseline_forced,
                                                 'simulation': simulation_forced,
                                                 'serial': serial_forced},
                              'hardware_or_shutdown_qualification': False}, indent=2))
    finally:
        os.close(slave)
        os.close(master)


if __name__ == '__main__':
    main()
