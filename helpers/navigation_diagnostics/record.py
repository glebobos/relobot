#!/usr/bin/env python3
"""Capture navigation on hardware or Gazebo using the already-running Nav2 container."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys
import threading
import uuid


ROOT = Path(__file__).resolve().parents[2]
ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && '


def command_output(command, timeout=20):
    return subprocess.run(command, check=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, text=True, timeout=timeout).stdout


def forward_output(stream, log):
    for line in stream:
        log.write(line)
        log.flush()
        print(line, end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', default='baseline')
    parser.add_argument('--duration', type=float, default=300.0, help='Wall seconds, maximum 3600')
    parser.add_argument('--max-mib', type=int, default=512, help='Approximate capture size limit')
    parser.add_argument('--no-bag', action='store_true', help='Only JSON metrics and snapshots')
    options = parser.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}', options.label):
        parser.error('label must contain 1-64 letters, numbers, underscores or hyphens')
    if not 0.0 < options.duration <= 3600.0 or options.max_mib < 16:
        parser.error('duration must be (0, 3600] and max-mib >= 16')

    try:
        info = json.loads(command_output([
            'docker', 'inspect', '--format',
            '{"id":{{json .Id}},"image":{{json .Image}},"running":{{json .State.Running}},'
            '"mounts":{{json .Mounts}}}', 'ros2_nav2',
        ]))
        if not info['running']:
            raise RuntimeError('Start the robot with ./start_robot.sh up or Gazebo with ./start_sim.sh up --world garden')
        workspace = ROOT / 'ros2_ws'
        if not any(mount['Destination'] == '/ros2_ws' and
                   Path(mount['Source']).resolve() == workspace for mount in info['mounts']):
            raise RuntimeError('ros2_nav2 is not mounted to this workspace')
        clock_mode = command_output([
            'docker', 'exec', 'ros2_nav2', 'bash', '-c', ROS_SETUP +
            'ros2 param get /controller_server use_sim_time --hide-type',
        ])
        use_sim_time = json.loads(clock_mode.strip().lower())
        if not isinstance(use_sim_time, bool):
            raise RuntimeError('Controller use_sim_time must be a boolean')

        run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + options.label + '-' + uuid.uuid4().hex[:6]
        output = workspace / 'log' / 'navigation' / run_id
        output.mkdir(parents=True)
        container_output = '/ros2_ws/log/navigation/' + run_id
        started = datetime.now(timezone.utc).isoformat()
        metadata = {
            'run_id': run_id, 'started_utc': started, 'duration_wall_s': options.duration,
            'max_mib': options.max_mib, 'bag_enabled': not options.no_bag,
            'container_id': info['id'], 'image_id': info['image'],
            'controller_sim_time': use_sim_time,
        }
        (output / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
        snapshots = {
            'git-revision.txt': ['git', '-C', str(ROOT), 'rev-parse', 'HEAD'],
            'git-status.txt': ['git', '-C', str(ROOT), 'status', '--short'],
            'git-diff.patch': ['git', '-C', str(ROOT), 'diff', 'HEAD', '--'],
            'packages.txt': ['docker', 'exec', 'ros2_nav2', 'dpkg-query', '-W',
                             'ros-humble-nav2-*', 'ros-humble-rosbag2'],
            'coverage-revision.txt': ['docker', 'exec', 'ros2_nav2', 'git', '-C',
                                      '/opt/opennav_coverage_src', 'rev-parse', 'HEAD'],
            'fields2cover-revision.txt': ['docker', 'exec', 'ros2_nav2', 'git', '-C',
                                          '/opt/fields2cover_src', 'rev-parse', 'HEAD'],
        }
        for filename, command in snapshots.items():
            try:
                text = command_output(command)
            except (subprocess.SubprocessError, OSError) as exc:
                text = f'Snapshot unavailable: {exc}\n'
            (output / filename).write_text(text)

        recorder = ['python3', '-u', '/ros2_ws/src/nav2/frontier_explorer/navigation_diagnostics.py',
                    '--output', container_output, '--duration', str(options.duration),
                    '--max-mib', str(options.max_mib), '--stop-on-stdin']
        if options.no_bag:
            recorder.append('--no-bag')
        recorder.extend(['--ros-args', '-p', f'use_sim_time:={str(use_sim_time).lower()}'])
        shell = ROS_SETUP + 'export PYTHONPATH=/ros2_ws/src/nav2:$PYTHONPATH && exec ' + shlex.join(recorder)
        print(f'Run: {run_id}\nOutput: {output}\nCtrl-C ends recording; it does not stop the robot.', flush=True)
        with (output / 'recorder.log').open('w') as log:
            process = subprocess.Popen(['docker', 'exec', '-i', 'ros2_nav2', 'bash', '-c', shell],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, start_new_session=True)
            reader = threading.Thread(target=forward_output, args=(process.stdout, log), daemon=True)
            reader.start()
            try:
                return_code = process.wait(timeout=options.duration + 45)
            except (KeyboardInterrupt, subprocess.TimeoutExpired):
                try:
                    process.stdin.write('stop\n')
                    process.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
                return_code = process.wait(timeout=30)
            finally:
                process.stdin.close()
                reader.join(timeout=5)
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)

        logs = subprocess.run(['docker', 'logs', '--since', started, '--tail', '20000', 'ros2_nav2'],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
        (output / 'nav2.log').write_bytes(logs.stdout[-8 * 1024 ** 2:])
        summary_path = output / 'summary.json'
        if summary_path.exists():
            summary = json.loads(summary_path.read_text())
            print(json.dumps(summary, indent=2))
        else:
            print(f'No summary was produced. Check {output / "recorder.log"}', file=sys.stderr)
            return_code = return_code or 1
        print(f'Recording saved: {output}')
        return return_code
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        print(f'Cannot record: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())