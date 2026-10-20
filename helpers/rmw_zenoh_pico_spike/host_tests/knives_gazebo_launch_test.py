#!/usr/bin/env python3
import importlib.util
import os
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument


ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / 'ros2_ws/src/relobot_gazebo/launch/gazebo_sim.launch.py'
SPEC = importlib.util.spec_from_file_location('knives_gazebo_launch', LAUNCH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class KnivesGazeboLaunchTest(unittest.TestCase):
    def mock_enabled(self, environment):
        nodes = []
        real_node = MODULE.Node
        real_share = MODULE.get_package_share_directory
        launch_environment = dict(os.environ)
        launch_environment.pop('SIM_KNIVES_MOCK', None)
        launch_environment.update(environment)

        def capture_node(*arguments, **keywords):
            node = real_node(*arguments, **keywords)
            if keywords.get('executable') == 'knives_mock.py':
                nodes.append(node)
            return node

        def package_share(package):
            if package == 'relobot_gazebo':
                return str(ROOT / 'ros2_ws/src/relobot_gazebo')
            return real_share(package)

        with patch.dict(os.environ, launch_environment, clear=True), \
                patch.object(MODULE, 'Node', side_effect=capture_node), \
                patch.object(MODULE, 'get_package_share_directory', side_effect=package_share):
            description = MODULE.generate_launch_description()
            context = LaunchContext()
            argument = next(action for action in description.entities
                            if isinstance(action, DeclareLaunchArgument) and action.name == 'knives_mock')
            argument.execute(context)
            self.assertEqual(len(nodes), 1)
            self.assertIsNotNone(nodes[0].condition)
            return nodes[0].condition.evaluate(context)

    def test_ordinary_gazebo_keeps_mock(self):
        self.assertTrue(self.mock_enabled({}))

    def test_native_knives_disables_mock(self):
        self.assertFalse(self.mock_enabled({'SIM_KNIVES_MOCK': 'false'}))


if __name__ == '__main__':
    unittest.main()
