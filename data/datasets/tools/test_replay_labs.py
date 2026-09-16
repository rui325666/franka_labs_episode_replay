"""Pure checks plus a real ROS graph test, restricted to an isolated ROS domain."""
import importlib.util
import json
import os
import tempfile
import time
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from replay_labs import Robot, SIDES, CONTROLLER, ARM_COLUMNS, MAX_STATE_AGE
from replay_labs import sample_action, ordered_joints, assert_tracking, move_home, replay
from replay_labs import GripperMotionGuard, load_gripper_reference, recorded_tracking_limits


class DataTests(unittest.TestCase):
    def test_slow_trial_keeps_absolute_tracking_cap_for_fast_recorded_wrist_turn(self):
        target, recorded = np.zeros(14), np.zeros(14)
        recorded[6] = .627318
        for speed in (1, .75):
            with self.assertRaisesRegex(ValueError, 'above 0.50'):
                recorded_tracking_limits(target, recorded, speed)
        limits = recorded_tracking_limits(target, recorded, .5)
        self.assertEqual(limits[6], .5)
        np.testing.assert_array_equal(np.delete(limits, 6), np.full(13, .35))
        actual = np.zeros(14)
        actual[6] = .49
        assert_tracking(target, actual, limits)
        actual[6] = .501
        with self.assertRaisesRegex(RuntimeError, 'left j7'):
            assert_tracking(target, actual, limits)
        with self.assertRaises(RuntimeError):
            assert_tracking(target, recorded, limits)  # Fast recorded lag is never exempted.

    def test_preflight_waits_for_disappearing_publishers_and_still_refuses_persistent_ones(self):
        class FakeClock:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, seconds):
                self.now += seconds

        robot = object.__new__(Robot)
        clock = FakeClock()
        def transient_publishers():
            if clock.now < 2 or 2.4 < clock.now < 2.8:
                raise RuntimeError('Other command publishers: UNKNOWN')
        robot.foreign_publishers = transient_publishers
        with patch('replay_labs.time', clock), patch('builtins.print'):
            robot.wait_for_command_topics(timeout=5, quiet_period=1)
        self.assertGreaterEqual(clock.now, 3.8)  # One uninterrupted second clear.

        for name in ('UNKNOWN', 'actual_teleop_node'):
            def persistent_publisher():
                raise RuntimeError('Other command publishers: ' + name)
            robot.foreign_publishers = persistent_publisher
            clock.now = 0
            with patch('replay_labs.time', clock), patch('builtins.print'):
                with self.assertRaisesRegex(RuntimeError, name):
                    robot.wait_for_command_topics(timeout=3)
            self.assertGreaterEqual(clock.now, 3)

    def test_recorded_tracking_allowance_only_for_relevant_joint_and_instant(self):
        target = np.zeros(14)
        recorded = np.zeros(14)
        recorded[13] = -.403744
        limits = recorded_tracking_limits(target, recorded)
        np.testing.assert_array_equal(limits[:13], np.full(13, .35))
        self.assertAlmostEqual(limits[13], .453744)
        assert_tracking(target, recorded, limits)
        with self.assertRaises(RuntimeError):
            assert_tracking(target, recorded)  # Previous fixed limit misclassified this sample.
        actual = recorded.copy()
        actual[13] -= .051
        with self.assertRaisesRegex(RuntimeError, 'right j7'):
            assert_tracking(target, actual, limits)
        actual = recorded.copy()
        actual[0] = .351
        with self.assertRaisesRegex(RuntimeError, 'left j1'):
            assert_tracking(target, actual, limits)
        np.testing.assert_array_equal(recorded_tracking_limits(target, target), np.full(14, .35))
        recorded[13] = -.451
        with self.assertRaisesRegex(ValueError, 'above 0.50'):
            recorded_tracking_limits(target, recorded)

    def test_homing_corrects_direction_dependent_static_friction_then_restores_action(self):
        class FakeClock:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, seconds):
                self.now += seconds

        class FrictionRobot:
            command = np.full(14, -.2)
            position = np.full(14, -.24)
            history = []
            def measured(self):
                return self.position.copy()
            def check_health(self):
                return self.measured()
            def foreign_publishers(self):
                pass
            def set_command(self, target):
                self.command = target.copy()
                # Direction-dependent +/- .04 rad deadband, not a fixed bias.
                self.position = np.clip(self.position, target - .04, target + .04)
                self.history.append(target.copy())

        expected = np.zeros(14)
        action = expected.copy()
        action[6], action[13] = -.0264328, .03172946
        robot = FrictionRobot()
        with patch('replay_labs.time', FakeClock()), patch('builtins.print'):
            move_home(robot, expected, command_target=action)
        np.testing.assert_allclose(robot.command, action, atol=1e-12)
        self.assertLess(np.max(np.abs(robot.measured() - expected)), .05)
        self.assertLessEqual(max(q[6] for q in robot.history), action[6] + .06)

    def test_gripper_guard_rejects_frozen_open_but_allows_grasp_contact(self):
        guard = GripperMotionGuard()
        guard.check(0, [.3, 1], [0, .028])
        guard.check(1, [.3, 1], [0, .028])
        with self.assertRaisesRegex(RuntimeError, 'left gripper did not move'):
            guard.check(1.51, [.3, 1], [0, .028])
        # A grasped object can stop closure well before the requested .7 rad.
        guard.check(2, [.3, 1], [.2, .028])
        guard.check(20, [.3, 1], [.2, .028])
        guard.check(21, [1, 1], [.2, .028])
        with self.assertRaisesRegex(RuntimeError, 'left gripper did not move'):
            guard.check(23, [1, 1], [.2, .028])

    def test_missing_gripper_reference_requires_prepare(self):
        with tempfile.TemporaryDirectory() as directory:
            np.savez(Path(directory) / 'episode.npz', measured_joints=np.zeros((3, 14)))
            with self.assertRaisesRegex(ValueError, 'prepare'):
                load_gripper_reference(Path(directory), 3)

    def test_compliant_follower_homes_with_recorded_command(self):
        # Physical failure: right j7 remains 0.0504 rad below the command.
        # Recorded action[0] is 0.031729 rad above recorded state[0].
        bias = np.zeros(14)
        bias[6], bias[13] = .0322, -.0504
        expected = np.zeros(14)
        recorded_command = expected.copy()
        recorded_command[6], recorded_command[13] = -.0264328, .03172946

        class FakeClock:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, seconds):
                self.now += seconds

        class CompliantRobot:
            command = np.zeros(14)
            def measured(self):
                return self.command + bias
            def check_health(self):
                return self.measured()
            def foreign_publishers(self):
                pass
            def set_command(self, target):
                self.command = target.copy()

        clock = FakeClock()
        with patch('replay_labs.time', clock), patch('builtins.print'):
            with self.assertRaisesRegex(RuntimeError, 'right j7'):
                move_home(CompliantRobot(), expected)
            robot = CompliantRobot()
            move_home(robot, expected, command_target=recorded_command)
        np.testing.assert_allclose(robot.command, recorded_command, atol=1e-12)
        self.assertLess(np.max(np.abs(robot.measured() - expected)), .05)

    def test_joint_interpolation_preserves_gripper_event_time_and_sides(self):
        actions = np.arange(48, dtype=float).reshape(3, 16) / 100
        times = np.array([0., .05, .1])
        value = sample_action(times, actions, .075)
        np.testing.assert_allclose(value[ARM_COLUMNS], (actions[1, ARM_COLUMNS] + actions[2, ARM_COLUMNS]) / 2)
        np.testing.assert_array_equal(value[[7, 15]], actions[1, [7, 15]])
        np.testing.assert_array_equal(sample_action(times, actions, .1), actions[2])
        np.testing.assert_array_equal(sample_action(times, actions, 100), actions[2])

    def test_joint_names_reordered_and_invalid_rejected(self):
        names, values = ordered_joints([f'left_fr3_joint{i}' for i in range(7, 0, -1)], list(range(7, 0, -1)))
        self.assertEqual(names[0], 'left_fr3_joint1')
        np.testing.assert_array_equal(values, np.arange(1, 8))
        for names, positions in [(['joint1'] * 7, [0] * 7), (['unknown'] * 7, [0] * 7)]:
            with self.assertRaises(ValueError):
                ordered_joints(names, positions)

    def test_stale_feedback_and_tracking_refuse(self):
        import threading
        robot = object.__new__(Robot)
        robot.lock = threading.Lock()
        robot.states = {s: ([], np.zeros(7), time.monotonic() - MAX_STATE_AGE - .1) for s in SIDES}
        with self.assertRaises(RuntimeError):
            robot.measured()
        before = robot.states['left'][2]
        robot.on_joints('left', SimpleNamespace(name=['broken'], position=[0]))
        self.assertEqual(robot.states['left'][2], before)
        with self.assertRaises(RuntimeError):
            assert_tracking(np.ones(14), np.zeros(14))
        with self.assertRaises(RuntimeError):
            assert_tracking(np.full(14, np.nan), np.zeros(14))


@unittest.skipUnless(importlib.util.find_spec('rclpy'), 'ROS test runs in the isolated Humble container')
class RosIntegrationTests(unittest.TestCase):
    def test_check_activate_home_replay_stop_and_conflict(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '93', 'Never run mock controllers on the robot domain')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32, String
        from std_srvs.srv import Trigger
        from controller_manager_msgs.srv import ListControllers
        from controller_manager_msgs.msg import ControllerState

        robot = Robot()
        mock = robot.rclpy.create_node('isolated_fake_lab')
        state = {s: 'IDLE' for s in SIDES}
        controllers = {s: {CONTROLLER: 'inactive', 'gravity_compensation_controller': 'inactive'} for s in SIDES}
        q = {s: np.zeros(7) for s in SIDES}
        grip = {s: 0.0 for s in SIDES}
        frozen_grippers = set()
        arm_messages, grip_messages, events = [], [], []
        publishers, handles = {}, []
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def control(side, operation, request, response):
            events.append((side, operation))
            if operation == 'get_ready':
                state[side] = 'READY'
                controllers[side]['gravity_compensation_controller'] = 'active'
            elif operation == 'start_operating':
                state[side] = 'FOLLOWING'
                controllers[side]['gravity_compensation_controller'] = 'inactive'
                controllers[side][CONTROLLER] = 'active'
            else:
                state[side] = 'IDLE'
                controllers[side] = {k: 'inactive' for k in controllers[side]}
            response.success = True
            return response

        def list_controllers(side, request, response):
            response.controller = [ControllerState(name=k, state=v) for k, v in controllers[side].items()]
            return response

        def command(side, message):
            arm_messages.append((side, message.position))
            if state[side] == 'FOLLOWING':
                q[side] = np.array(message.position)

        def gripper_command(side, message):
            grip_messages.append((side, message.data))
            if side not in frozen_grippers:
                grip[side] = 1 - message.data

        def reactivate(side, request, response):
            events.append((side, 'gripper_reactivate'))
            grip[side] = 0.0
            response.success = True
            return response

        def list_gripper_controllers(request, response):
            response.controller = [ControllerState(name=name, state='active') for name in
                                   ('robotiq_gripper_controller', 'robotiq_activation_controller')]
            return response

        def publish_feedback():
            for side in SIDES:
                msg = JointState()
                msg.header.stamp = mock.get_clock().now().to_msg()
                msg.name = [f'{side}_fr3_joint{i}' for i in range(1, 8)]
                msg.position = q[side].tolist()
                publishers[side, 'joints'].publish(msg)
                publishers[side, 'state'].publish(String(data=state[side]))
                publishers[side, 'gripper'].publish(JointState(position=[grip[side]]))

        for side in SIDES:
            publishers[side, 'joints'] = mock.create_publisher(JointState, f'/{side}/franka_robot_state_broadcaster/measured_joint_states', 1)
            publishers[side, 'state'] = mock.create_publisher(String, f'/{side}/controller_coordinator/state', durable)
            publishers[side, 'gripper'] = mock.create_publisher(JointState, f'/{side}/follower/gripper/joint_states', 1)
            handles.append(mock.create_subscription(JointState, f'/{side}/follower/gello/joint_states', lambda msg, s=side: command(s, msg), 1))
            handles.append(mock.create_subscription(Float32, f'/{side}/follower/gripper/gripper_client/target_gripper_width_percent', lambda msg, s=side: gripper_command(s, msg), 1))
            for operation in ('get_ready', 'start_operating', 'stop'):
                handles.append(mock.create_service(Trigger, f'/{side}/controller_coordinator/{operation}', lambda req, res, s=side, op=operation: control(s, op, req, res)))
            handles.append(mock.create_service(ListControllers, f'/{side}/controller_manager/list_controllers', lambda req, res, s=side: list_controllers(s, req, res)))
            handles.append(mock.create_service(ListControllers, f'/{side}/follower/gripper/controller_manager/list_controllers', list_gripper_controllers))
            handles.append(mock.create_service(Trigger, f'/{side}/follower/gripper/robotiq_activation_controller/reactivate_gripper', lambda req, res, s=side: reactivate(s, req, res)))
        timer = mock.create_timer(.005, publish_feedback)
        robot.executor.add_node(mock)
        try:
            self.assertEqual(robot.preflight(np.zeros(14)), 0)
            self.assertEqual(arm_messages, [])
            self.assertEqual(grip_messages, [])
            self.assertEqual(events, [])  # The check did not request a control transition.
            with tempfile.TemporaryDirectory() as directory:
                robot.verify_grippers(Path(directory))
                self.assertEqual(len(list(Path(directory).glob('gripper_check_*.json'))), 1)
            self.assertEqual(arm_messages, [])
            self.assertTrue(all(value == 'IDLE' for value in state.values()))
            self.assertTrue(all((side, 'gripper_reactivate') in events for side in SIDES))
            time.sleep(.1)
            gripper_count = len(grip_messages)
            robot.start()
            target = np.full(14, .015)
            move_home(robot, target)
            self.assertEqual(len(grip_messages), gripper_count)  # Homing never commands either gripper.
            np.testing.assert_allclose(robot.measured(), target, atol=.001)
            times = np.arange(8) / 20
            actions = np.full((8, 16), .015)
            actions[:, 7] = np.linspace(1, .5, 8)
            actions[:, 15] = 1
            with tempfile.TemporaryDirectory() as directory:
                replay(robot, times, actions, 20, 1, Path(directory),
                       recorded_grippers=1 - actions[:, [7, 15]], recorded_joints=actions[:, ARM_COLUMNS])
                self.assertEqual(len(list(Path(directory).glob('trace_*.csv'))), 1)
            self.assertTrue(grip_messages)
            robot.stop()
            self.assertEqual([op for s, op in events if s == 'left'], ['gripper_reactivate', 'get_ready', 'start_operating', 'stop'])
            self.assertTrue(all(value == 'inactive' for side in SIDES for value in controllers[side].values()))
            # Reproduce the real failure: feedback is fresh but the hardware stays open.
            frozen_grippers.add('left')
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaisesRegex(RuntimeError, 'left gripper motion test failed'):
                    robot.verify_grippers(Path(directory))
                report = json.loads(next(Path(directory).glob('gripper_check_*.json')).read_text())
                self.assertFalse(report['success'])
                self.assertEqual(report['sides']['left'][-1]['command_open'], .8)
                self.assertFalse(report['sides']['left'][-1]['verified'])
            self.assertTrue(all(value == 'IDLE' for value in state.values()))
            # A competing sender must be rejected, even if its values are harmless.
            conflict = mock.create_publisher(JointState, '/left/follower/gello/joint_states', 1)
            time.sleep(.3)
            with self.assertRaises(RuntimeError):
                robot.foreign_publishers()
            mock.destroy_publisher(conflict)
            timer.cancel()
            time.sleep(MAX_STATE_AGE + .1)
            with self.assertRaises(RuntimeError):
                robot.check_health()
        finally:
            # Simulation services are alive during teardown even on assertion failures.
            robot.stop()
            robot.executor.remove_node(mock)
            mock.destroy_node()
            robot.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
