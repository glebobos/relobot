#!/usr/bin/env python3
import ast
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
HELPER = ROOT / 'ros2_ws/zenoh_devices.py'
SPEC = importlib.util.spec_from_file_location('zenoh_devices', HELPER)
DEVICES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DEVICES)
NATIVE = '/dev/serial/by-id/usb-Raspberry_Pi_Pico_native-if00'
WHEELS = '/dev/serial/by-id/usb-Raspberry_Pi_Pico_wheels-if00'
SECOND = '/dev/serial/by-id/usb-ReloBot_second-if00'


def device_entry(identity=NATIVE, **overrides):
    return {'serial_by_id': os.path.basename(identity), 'baudrate': 115200,
            'enabled': True, **overrides}


class ConfigFixture(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='zenoh-devices-test-')
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.config = self.directory / 'zenoh_devices.json'
        self.write_devices()

    def write_devices(self, entries=None):
        self.config.write_text(json.dumps({'devices': entries if entries is not None else {
            'knives': device_entry(),
        }}))


class DeviceConfigTest(ConfigFixture):
    def test_valid_knives(self):
        self.assertEqual(DEVICES.load_devices(self.config), {
            'knives': {'serial_by_id': os.path.basename(NATIVE), 'baudrate': 115200},
        })

    def test_invalid_identity(self):
        for identity in ('', '/dev/ttyACM0', '../usb-native', 'usb-native/extra',
                         'usb-native#baudrate=1200', 'usb-native"]', 'usb-'):
            with self.subTest(identity=identity), self.assertRaisesRegex(ValueError, 'serial_by_id'):
                self.write_devices({'knives': device_entry(serial_by_id=identity)})
                DEVICES.load_devices(self.config)

    def test_invalid_baudrate(self):
        for baudrate in (None, '115200', True, 0, -1, 115200.5, 4000001):
            with self.subTest(baudrate=baudrate), self.assertRaisesRegex(ValueError, 'baudrate'):
                self.write_devices({'knives': device_entry(baudrate=baudrate)})
                DEVICES.load_devices(self.config)

    def test_invalid_structure(self):
        for value in ([], None, {}, {'devices': []}, {'devices': {}, 'typo': 1},
                      {'devices': {'knives': []}},
                      {'devices': {'knives': device_entry(typo='ignored')}},
                      {'devices': {'knives': device_entry(enabled='true')}}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config.write_text(json.dumps(value))
                DEVICES.load_devices(self.config)

    def test_empty_device_set_is_rejected(self):
        for entries in ({}, {'knives': device_entry(enabled=False)}):
            with self.subTest(entries=entries), \
                    self.assertRaisesRegex(ValueError, 'at least one enabled device'):
                self.write_devices(entries)
                DEVICES.load_devices(self.config)

    def test_non_knives_device_is_accepted_alone(self):
        self.write_devices({'ina226': device_entry(SECOND)})
        self.assertEqual(DEVICES.load_devices(self.config), {
            'ina226': {'serial_by_id': os.path.basename(SECOND), 'baudrate': 115200},
        })

    def test_duplicate_device_identity_is_rejected(self):
        self.write_devices({'knives': device_entry(), 'second': device_entry()})
        with self.assertRaisesRegex(ValueError, 'duplicate serial_by_id'):
            DEVICES.load_devices(self.config)

    def test_duplicate_json_keys_are_rejected(self):
        self.config.write_text('{"devices": {}, "devices": {}}')
        with self.assertRaisesRegex(ValueError, 'duplicate JSON key'):
            DEVICES.load_devices(self.config)

    def test_disabled_future_device_is_ignored(self):
        self.write_devices({'knives': device_entry(), 'future': device_entry('', enabled=False)})
        self.assertEqual(list(DEVICES.load_devices(self.config)), ['knives'])

    def test_cli_reports_config_filename(self):
        self.config.write_text('{broken JSON')
        result = subprocess.run(['python3', str(HELPER), '--config', str(self.config), 'validate'],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 1)
        self.assertIn(str(self.config), result.stderr)


class SerialSelectionTest(ConfigFixture):
    def select(self, by_id, raw, overrides=None, native=False, missing_native=False):
        identities = {NATIVE: '/dev/ttyACM0', WHEELS: '/dev/ttyACM1', SECOND: '/dev/ttyACM2'}
        existing = set(by_id + raw + list(identities))
        if missing_native:
            existing.discard(NATIVE)

        def discover(pattern):
            if pattern == '/dev/ttyACM*':
                return raw
            return [path for path in by_id if ('Pico' in path if 'Pico' in pattern else 'ReloBot' in path)]

        with patch.dict(os.environ, overrides or {}, clear=True), \
                patch.object(DEVICES.glob, 'glob', side_effect=discover), \
                patch('os.path.realpath', side_effect=lambda path: identities.get(path, path)), \
                patch('os.path.exists', side_effect=lambda path: path in existing):
            return DEVICES.micro_ros_serial_devices(self.config, native_enabled=native)

    def test_non_native_mode_keeps_micro_ros_knives(self):
        self.config.unlink()
        self.assertEqual(self.select([NATIVE, WHEELS], []), [NATIVE, WHEELS])

    def test_native_is_excluded(self):
        self.assertEqual(self.select([NATIVE, WHEELS], [], native=True), [WHEELS])

    def test_raw_fallback_cannot_reopen_native(self):
        self.assertEqual(self.select([], ['/dev/ttyACM0', '/dev/ttyACM1'], native=True), ['/dev/ttyACM1'])

    def test_native_by_id_does_not_hide_raw_micro_ros(self):
        self.assertEqual(self.select([NATIVE], ['/dev/ttyACM0', '/dev/ttyACM1'], native=True), ['/dev/ttyACM1'])

    def test_environment_override_cannot_reopen_native(self):
        self.assertEqual(self.select([WHEELS], ['/dev/ttyACM0'],
                                     {'TTY_KNIVES': '/dev/ttyACM0'}, native=True), [WHEELS])

    def test_aliases_are_deduplicated(self):
        self.assertEqual(self.select([WHEELS], ['/dev/ttyACM1'],
                                     {'TTY_WHEELS': '/dev/ttyACM1'}), [WHEELS])

    def test_native_only_does_not_invent_fallback(self):
        self.assertEqual(self.select([NATIVE], ['/dev/ttyACM0'], native=True), [])

    def test_missing_native_identity_fails_closed(self):
        with self.assertRaises(RuntimeError):
            self.select([], ['/dev/ttyACM0'], native=True, missing_native=True)

    def test_no_devices(self):
        self.assertEqual(self.select([], []), [])

    def test_all_configured_native_devices_are_excluded(self):
        self.write_devices({'knives': device_entry(), 'second': device_entry(SECOND)})
        self.assertEqual(self.select([NATIVE, SECOND, WHEELS], [],
                                     {'TTY_SECOND': '/dev/ttyACM2'}, native=True), [WHEELS])


class LaunchDelegationTest(unittest.TestCase):
    def test_launch_uses_shared_json_helper(self):
        launch = ROOT / 'ros2_ws/src/diff_drive_hardware/launch/diffbot.launch.py'
        function = next(node for node in ast.parse(launch.read_text()).body
                        if isinstance(node, ast.FunctionDef) and node.name == 'micro_ros_serial_devices')
        namespace = {'os': os, 'json': json, 'subprocess': subprocess}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(launch), 'exec'), namespace)
        for enabled in ('true', 'false'):
            with self.subTest(enabled=enabled), \
                    patch.dict(os.environ, {'PICO_KNIVES_ENABLED': enabled}, clear=True), \
                    patch('subprocess.check_output', return_value='["/dev/ttyACM1"]') as check_output:
                self.assertEqual(namespace['micro_ros_serial_devices'](), ['/dev/ttyACM1'])
                check_output.assert_called_once_with([
                    'python3', '/ros2_ws/zenoh_devices.py', 'micro-ros-devices',
                    '--native-enabled', enabled,
                ], text=True)


class LauncherTest(ConfigFixture):
    def launch(self, arguments, config_text=None):
        workspace = self.directory / 'ros2_ws'
        workspace.mkdir()
        shutil.copy2(ROOT / 'start_robot.sh', self.directory)
        shutil.copy2(HELPER, workspace)
        if config_text is not None:
            (workspace / 'zenoh_devices.json').write_text(config_text)
        else:
            shutil.copy2(self.config, workspace)
        docker = self.directory / 'docker'
        docker.write_text('''#!/usr/bin/env python3
import json
import os
import sys
print('COMPOSE_CAPTURE:' + json.dumps({
    'arguments': sys.argv[1:],
    'environment': {key: os.environ.get(key) for key in (
        'PICO_KNIVES_ENABLED', 'SIM_KNIVES_MOCK')},
}))
''')
        docker.chmod(0o755)
        environment = os.environ.copy()
        environment.update({
            'PATH': str(self.directory) + os.pathsep + environment['PATH'],
            'PICO_KNIVES_ENABLED': 'inherited',
            'SIM_KNIVES_MOCK': 'inherited',
        })
        return subprocess.run(['bash', str(self.directory / 'start_robot.sh'), *arguments],
                              env=environment, capture_output=True, text=True, timeout=5)

    def capture(self, result):
        self.assertEqual(result.returncode, 0, result.stderr)
        capture = next(line.removeprefix('COMPOSE_CAPTURE:')
                       for line in result.stdout.splitlines() if line.startswith('COMPOSE_CAPTURE:'))
        capture = json.loads(capture)
        self.assertEqual(capture['arguments'][:3], ['compose', '-f', 'docker-compose.yml'])
        compose_options = capture['arguments'][:capture['arguments'].index('--profile')]
        self.assertEqual(compose_options.count('-f'), 1)
        return capture

    def test_hardware_uses_json_without_exports(self):
        result = self.launch(['up'])
        capture = self.capture(result)
        self.assertIn(os.path.basename(NATIVE), result.stdout)
        self.assertEqual(capture['environment'], {
            'PICO_KNIVES_ENABLED': 'true', 'SIM_KNIVES_MOCK': 'false',
        })
        self.assertEqual(capture['arguments'][3:5], ['--profile', 'hardware'])

    def test_simulation_ignores_broken_hardware_config(self):
        capture = self.capture(self.launch(['up', '--sim', '--headless'], '{broken JSON'))
        self.assertEqual(capture['environment'], {
            'PICO_KNIVES_ENABLED': 'false', 'SIM_KNIVES_MOCK': 'true',
        })
        self.assertEqual(capture['arguments'][3:5], ['--profile', 'sim'])

    def test_hybrid_uses_json_and_disables_mock(self):
        capture = self.capture(self.launch(['up', '--sim', '--headless', '--picoros-knives']))
        self.assertEqual(capture['environment']['PICO_KNIVES_ENABLED'], 'true')
        self.assertEqual(capture['environment']['SIM_KNIVES_MOCK'], 'false')

    def test_hardware_and_hybrid_reject_bad_config_before_compose(self):
        for arguments in (['up'], ['up', '--sim', '--headless', '--picoros-knives']):
            for text in ('{broken JSON', '{"devices":{"knives":{"serial_by_id":""}}}'):
                with self.subTest(arguments=arguments, text=text):
                    with tempfile.TemporaryDirectory(prefix='launcher-invalid-') as temporary:
                        original = self.directory
                        self.directory = Path(temporary)
                        try:
                            result = self.launch(arguments, text)
                        finally:
                            self.directory = original
                    self.assertEqual(result.returncode, 1)
                    self.assertIn('zenoh_devices.json', result.stderr)
                    self.assertNotIn('COMPOSE_CAPTURE:', result.stdout)

    def test_management_does_not_require_valid_device_config(self):
        for command in ('down', 'ps', 'logs', 'build'):
            with self.subTest(command=command):
                with tempfile.TemporaryDirectory(prefix='launcher-management-') as temporary:
                    original = self.directory
                    self.directory = Path(temporary)
                    try:
                        self.capture(self.launch([command], '{broken JSON'))
                    finally:
                        self.directory = original


class RouterStartupTest(ConfigFixture):
    def start(self, settings=None):
        environment = os.environ.copy()
        for key in ('PICO_KNIVES_ENABLED', 'ZENOH_CONFIG_OVERRIDE'):
            environment.pop(key, None)
        environment.update(settings or {})
        return subprocess.run(
            ['bash', '-c', 'source() { :; }; exec() { env -0; }; '
             'export -f source exec; bash "$1" "$2"', 'router-startup-test',
             str(ROOT / 'ros2_ws/start_zenoh_router.sh'), str(self.config)],
            env=environment, capture_output=True, text=True, timeout=5,
        )

    def test_native_startup_requires_configured_identity_by_default(self):
        self.write_devices({'knives': device_entry('')})
        result = self.start()
        self.assertEqual(result.returncode, 1)
        self.assertIn('knives.serial_by_id', result.stderr)

    def test_native_startup_rejects_missing_file(self):
        self.config.unlink()
        result = self.start()
        self.assertEqual(result.returncode, 1)
        self.assertIn(str(self.config), result.stderr)

    def test_one_router_gets_tcp_and_serial_listeners(self):
        result = self.start()
        self.assertEqual(result.returncode, 0, result.stderr)
        environment = dict(entry.split('=', 1) for entry in result.stdout.split('\0') if entry)
        self.assertEqual(environment['ZENOH_CONFIG_OVERRIDE'],
                         'listen/endpoints=["tcp/[::]:7447",'
                         f'"serial//hostdev/serial/by-id/{os.path.basename(NATIVE)}#baudrate=115200"]')

    def test_added_device_gets_its_own_listener_on_same_router(self):
        self.write_devices({'knives': device_entry(), 'second': device_entry(SECOND, baudrate=230400)})
        result = self.start()
        self.assertEqual(result.returncode, 0, result.stderr)
        environment = dict(entry.split('=', 1) for entry in result.stdout.split('\0') if entry)
        endpoints = json.loads(environment['ZENOH_CONFIG_OVERRIDE'].split('=', 1)[1])
        self.assertEqual(len(endpoints), 3)
        self.assertEqual(endpoints[-1], f'serial//hostdev/serial/by-id/{os.path.basename(SECOND)}#baudrate=230400')

    def test_simulation_removes_inherited_serial_override_without_config(self):
        self.config.unlink()
        result = self.start({'PICO_KNIVES_ENABLED': 'false',
                             'ZENOH_CONFIG_OVERRIDE': 'listen/endpoints=["serial/stale"]'})
        self.assertEqual(result.returncode, 0, result.stderr)
        environment = dict(entry.split('=', 1) for entry in result.stdout.split('\0') if entry)
        self.assertNotIn('ZENOH_CONFIG_OVERRIDE', environment)

    def test_invalid_mode_is_rejected(self):
        result = self.start({'PICO_KNIVES_ENABLED': 'unexpected'})
        self.assertEqual(result.returncode, 1)
        self.assertIn('must be true or false', result.stderr)


if __name__ == '__main__':
    unittest.main()
