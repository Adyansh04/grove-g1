"""Offline CLI fixtures: no ROS installation, graph, or simulator required."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'verify_week1.sh'
ROS = r'''#!/usr/bin/env python3
import os, sys
args = sys.argv[1:]
with open(os.environ['CALL_LOG'], 'a') as log:
    log.write(' '.join(args) + '\n')
mode = os.environ.get('FIXTURE', 'healthy')
if args[:2] == ['pkg', 'prefix']:
    print('/fixture/install/' + args[2])
elif args[:2] == ['node', 'list']:
    print('/bt_navigator')
elif args[:2] == ['topic', 'list']:
    print('/g1_odometry_publisher/odom [nav_msgs/msg/Odometry]')
elif args[:2] == ['action', 'list']:
    print('/navigate_to_pose [nav2_msgs/action/NavigateToPose]')
elif args[:2] == ['action', 'info']:
    print('Action servers: ' + ('0' if mode == 'no_server' else '1'))
elif args[:2] == ['control', 'list_controllers']:
    if mode == 'no_plugin':
        print("invalid choice: 'control'"); sys.exit(2)
    print('fixture_controller fixture/Controller active')
elif args[:2] == ['param', 'get']:
    print('String value is: fast_lio')
elif args[:2] == ['lifecycle', 'get']:
    print('inactive [2]' if mode == 'inactive' else 'active [3]')
elif args[:2] == ['topic', 'type']:
    print('nav_msgs/msg/Odometry' if args[2].endswith('/odom') else
          'sensor_msgs/msg/CameraInfo' if args[2].endswith('/camera_info') else 'sensor_msgs/msg/Image')
elif args[:2] == ['topic', 'info']:
    print('Publisher count: 1')
elif args[:2] == ['topic', 'echo']:
    if mode == 'no_data': sys.exit(124)
    topic = args[2]
    frame = 'odom' if topic.endswith('/odom') else (
        ('chest_camera' if topic.startswith('/chest_camera/') else 'camera') +
        ('_depth_optical_frame' if '/aligned_depth_to_color/' in topic else '_color_optical_frame'))
    if mode == 'wrong_frame': frame += '_wrong'
    print('stamp:\n  sec: 1\n  nanosec: 0\nframe_id: ' + frame)
elif args[:2] == ['topic', 'hz']:
    if mode != 'no_data': print('average rate: 30.000')
    sys.exit(124)
elif args[:3] == ['run', 'tf2_ros', 'tf2_echo']:
    if mode != 'no_tf': print('Translation: [0, 0, 0]\nRotation: in Quaternion [0, 0, 0, 1]')
    sys.exit(124)
else:
    raise SystemExit('Unexpected command: ' + repr(args))
'''


class VerificationTests(unittest.TestCase):
    def run_fixture(self, mode='healthy', **overrides):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, content in {
                'ros2': ROS,
                # Preserve command arguments but avoid real waits in fixture tests.
                'timeout': '#!/bin/bash\nshift 3\nexec "$@"\n',
            }.items():
                path = root / name
                path.write_text(content)
                path.chmod(0o755)
            log = root / 'calls'
            env = dict(os.environ, PATH=f'{root}:/usr/bin:/bin', ROS_DISTRO='jazzy',
                       CALL_LOG=str(log), FIXTURE=mode)
            env.update(overrides)
            result = subprocess.run(['bash', str(SCRIPT)], env=env, text=True,
                                    capture_output=True, timeout=30)
            calls = log.read_text().splitlines() if log.exists() else []
            allowed = {('pkg', 'prefix'), ('node', 'list'), ('topic', 'list'),
                       ('action', 'list'), ('action', 'info'), ('control', 'list_controllers'),
                       ('param', 'get'), ('lifecycle', 'get'), ('topic', 'type'),
                       ('topic', 'info'), ('topic', 'echo'), ('topic', 'hz'), ('run', 'tf2_ros')}
            for call in calls:
                self.assertIn(tuple(call.split()[:2]), allowed)
            return result

    def test_observed_interfaces_pass_even_when_stream_probes_timeout(self):
        result = self.run_fixture()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('FAIL=0 SKIP=0', result.stdout)

    def test_names_without_data_do_not_pass(self):
        result = self.run_fixture('no_data')
        self.assertEqual(result.returncode, 1)
        self.assertIn('no positive rate measured', result.stdout)
        self.assertIn('no received header', result.stdout)

    def test_frame_prefix_is_not_a_frame_match(self):
        result = self.run_fixture('wrong_frame')
        self.assertEqual(result.returncode, 1)
        self.assertIn('no received header', result.stdout)

    def test_missing_action_server_fails(self):
        self.assertEqual(self.run_fixture('no_server').returncode, 1)

    def test_inactive_lifecycle_fails(self):
        self.assertEqual(self.run_fixture('inactive').returncode, 1)

    def test_no_transform_fails(self):
        self.assertEqual(self.run_fixture('no_tf').returncode, 1)

    def test_unavailable_plugin_is_incomplete(self):
        result = self.run_fixture('no_plugin')
        self.assertEqual(result.returncode, 2)
        self.assertIn('SKIP: ros2_control controllers', result.stdout)

    def test_invalid_timeout_rejected_before_ros(self):
        self.assertEqual(self.run_fixture(BODYGUARD_CHECK_SECONDS='0').returncode, 2)


if __name__ == '__main__':
    unittest.main()
