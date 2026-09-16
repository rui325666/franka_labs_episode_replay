#!/usr/bin/env python3
"""Replay LABS arm/gripper commands. check is read-only; run moves arms and grippers."""
import argparse
import csv
import hashlib
import json
import re
import signal
import threading
import time
from pathlib import Path

import numpy as np

SIDES = ('left', 'right')
ARM_COLUMNS = np.r_[0:7, 8:15]
CONTROLLER = 'joint_follower_controller'
MAX_STATE_AGE = 0.20
MAX_TRACKING_ERROR = 0.35
RECORDED_TRACKING_MARGIN = 0.05
MAX_REPLAY_TRACKING_LIMIT = 0.50
START_TOL = 0.05


def load_bundle(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    path = directory / 'episode.npz'
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest['bundle_sha256']:
        raise ValueError('Replay bundle hash does not match manifest; prepare again')
    with np.load(path, allow_pickle=False) as data:
        times = data['timestamps'].copy()
        actions = data['actions'].copy()
        measured = data['measured_joints'].copy()
        fps = float(data['fps'])
    n = len(times)
    if n < 2 or actions.shape != (n, 16) or measured.shape != (n, 14):
        raise ValueError('Invalid replay dimensions')
    if not all(np.isfinite(v).all() for v in (times, actions, measured)):
        raise ValueError('Nonfinite replay values')
    if fps <= 0 or times[0] != 0 or not np.allclose(np.diff(times), 1 / fps, atol=1e-5):
        raise ValueError('Invalid replay clock')
    if np.any((actions[:, [7, 15]] < 0) | (actions[:, [7, 15]] > 1)):
        raise ValueError('Invalid gripper fraction')
    return times, actions, measured, fps, manifest


def load_gripper_reference(directory, frames):
    with np.load(directory / 'episode.npz', allow_pickle=False) as data:
        if 'measured_grippers' not in data:
            raise ValueError('Gripper reference missing; run replay_tower.sh prepare first')
        values = data['measured_grippers'].copy()
    if values.shape != (frames, 2) or not np.isfinite(values).all():
        raise ValueError('Invalid recorded gripper feedback')
    return values


class GripperMotionGuard:
    """Detect a fully open, unmoving gripper under sustained close commands.

    Contact during grasping can legitimately prevent reaching the requested
    angle. Do not treat ordinary contact away from full-open as a fault.
    """
    def __init__(self):
        self.since = [None, None]

    def check(self, now, opening, measured):
        for i, side in enumerate(SIDES):
            failed_to_move = ((opening[i] < 0.8 and measured[i] < 0.02)
                              or (opening[i] >= 0.98 and measured[i] > 0.10))
            if not failed_to_move:
                self.since[i] = None
            elif self.since[i] is None:
                self.since[i] = now
            elif now - self.since[i] > 1.5:
                raise RuntimeError(f'{side} gripper did not move as requested for 1.5s: '
                                   f'open command={opening[i]:.3f}, knuckle={measured[i]:.3f} rad')


def sample_action(times, actions, elapsed):
    """Interpolate joints; hold grippers until the recorded transition time."""
    i = int(np.clip(np.searchsorted(times, elapsed, side='right') - 1, 0, len(times) - 1))
    result = actions[i].copy()
    if i < len(times) - 1:
        fraction = float(np.clip((elapsed - times[i]) / (times[i + 1] - times[i]), 0, 1))
        result[ARM_COLUMNS] += fraction * (actions[i + 1, ARM_COLUMNS] - result[ARM_COLUMNS])
    return result


def ordered_joints(names, positions):
    result = {}
    for name, value in zip(names, positions):
        match = re.search(r'(?:^|_)joint([1-7])$', name)
        if match:
            index = int(match.group(1))
            if index in result:
                raise ValueError('Duplicate arm joint name')
            result[index] = (name, float(value))
    if set(result) != set(range(1, 8)):
        raise ValueError(f'Expected seven named arm joints, got {names}')
    values = np.array([result[i][1] for i in range(1, 8)])
    if not np.isfinite(values).all():
        raise ValueError('Nonfinite measured joint state')
    return [result[i][0] for i in range(1, 8)], values


def recorded_tracking_limits(target, recorded_measured, speed=1.0):
    """Allow the demonstrated tracking lag at this instant, plus a small margin.

    Limits apply separately to each joint and time sample. Homing still uses
    the fixed limit, and no replay limit may exceed the absolute ceiling.
    For recordings exceeding that ceiling, only half speed or slower is
    accepted. Their reference allowance is capped, never extrapolated into a
    claim about slow-speed tracking: live error must still stay below the cap.
    """
    target, recorded_measured = np.asarray(target), np.asarray(recorded_measured)
    if target.shape != recorded_measured.shape or target.shape[-1:] != (14,):
        raise ValueError('Invalid recorded tracking reference shape')
    if not np.isfinite(speed) or not 0 < speed <= 1:
        raise ValueError('Invalid replay speed')
    limits = np.maximum(MAX_TRACKING_ERROR, np.abs(target - recorded_measured) + RECORDED_TRACKING_MARGIN)
    if not np.isfinite(limits).all():
        raise ValueError('Nonfinite recorded tracking reference')
    if np.max(limits) > MAX_REPLAY_TRACKING_LIMIT and speed > .5:
        raise ValueError('Recorded tracking lag requires a limit above 0.50 rad; '
                         'use --speed 0.5 or slower for a monitored trial. '
                         'Live tracking error remains limited to 0.50 rad.')
    return np.minimum(limits, MAX_REPLAY_TRACKING_LIMIT)


def assert_tracking(target, measured, limits=None):
    errors = np.abs(target - measured)
    error = float(np.max(errors))
    limits = MAX_TRACKING_ERROR if limits is None else np.asarray(limits)
    if not np.isfinite(error) or not np.isfinite(limits).all() or np.any(errors > limits):
        if np.isfinite(errors).all():
            index = int(np.argmax(errors - limits))
            bound = float(np.broadcast_to(limits, errors.shape)[index])
            detail = f'{SIDES[index // 7]} j{index % 7 + 1}: {errors[index]:.3f} rad exceeds {bound:.3f}'
        else:
            detail = 'nonfinite measured or target values'
        raise RuntimeError(f'Joint tracking error {detail}; stopping')
    return error


def pose_error_description(measured, expected, section=slice(None)):
    indices = np.arange(14)[section]
    errors = np.asarray(measured)[section] - np.asarray(expected)[section]
    index = int(indices[np.argmax(np.abs(errors))])
    side, joint = SIDES[index // 7], index % 7 + 1
    signed = float(measured[index] - expected[index])
    return (f'{side} j{joint}: measured={measured[index]:.6f}, '
            f'recorded_measured={expected[index]:.6f}, error={signed:+.6f} rad; '
            f'per-joint errors={np.array2string(errors, precision=5)}')


class Robot:
    def __init__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float32, String
        from std_srvs.srv import Trigger
        from controller_manager_msgs.srv import ListControllers, SwitchController

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy, self.JointState, self.Float32 = rclpy, JointState, Float32
        self.Trigger, self.ListControllers, self.SwitchController = Trigger, ListControllers, SwitchController
        self.node = rclpy.create_node('labs_dataset_replay')
        self.lock = threading.Lock()
        self.states, self.gripper_states, self.coordinators, self.clients = {}, {}, {}, {}
        self.subscriptions = []
        self.publishers = {}
        self.command = None
        self.grippers = None
        self.stream_error = None
        self.last_publish = None
        self.stop_stream = threading.Event()
        self.stream_thread = None
        self.touched = []
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        durable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for side in SIDES:
            topic = f'/{side}/franka_robot_state_broadcaster/measured_joint_states'
            self.subscriptions.append(self.node.create_subscription(
                JointState, topic, lambda msg, s=side: self.on_joints(s, msg), state_qos))
            self.subscriptions.append(self.node.create_subscription(
                JointState, f'/{side}/follower/gripper/joint_states',
                lambda msg, s=side: self.on_gripper(s, msg), state_qos))
            self.subscriptions.append(self.node.create_subscription(
                String, f'/{side}/controller_coordinator/state',
                lambda msg, s=side: self.on_coordinator(s, msg), durable_qos))
            for operation in ('get_ready', 'start_operating', 'stop'):
                self.clients[side, operation] = self.node.create_client(
                    Trigger, f'/{side}/controller_coordinator/{operation}')
            self.clients[side, 'list'] = self.node.create_client(
                ListControllers, f'/{side}/controller_manager/list_controllers')
            self.clients[side, 'switch'] = self.node.create_client(
                SwitchController, f'/{side}/controller_manager/switch_controller')
            self.clients[side, 'gripper_reactivate'] = self.node.create_client(
                Trigger, f'/{side}/follower/gripper/robotiq_activation_controller/reactivate_gripper')
            self.clients[side, 'gripper_list'] = self.node.create_client(
                ListControllers, f'/{side}/follower/gripper/controller_manager/list_controllers')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def on_joints(self, side, message):
        try:
            names, values = ordered_joints(message.name, message.position)
        except ValueError:
            return  # Invalid samples must never refresh the freshness timestamp.
        with self.lock:
            self.states[side] = (names, values, time.monotonic())

    def on_coordinator(self, side, message):
        with self.lock:
            self.coordinators[side] = (message.data, time.monotonic())

    def on_gripper(self, side, message):
        if len(message.position) == 1 and np.isfinite(message.position[0]):
            with self.lock:
                self.gripper_states[side] = (float(message.position[0]), time.monotonic())

    def fresh_grippers(self):
        with self.lock:
            current = dict(self.gripper_states)
        for side in SIDES:
            if side not in current or time.monotonic() - current[side][1] > MAX_STATE_AGE:
                raise RuntimeError(f'{side}: no fresh gripper feedback')
        return [current[s][0] for s in SIDES]

    def measured(self):
        with self.lock:
            current = dict(self.states)
        now = time.monotonic()
        for side in SIDES:
            if side not in current or now - current[side][2] > MAX_STATE_AGE:
                raise RuntimeError(f'{side}: no fresh joint feedback (limit {MAX_STATE_AGE}s); check FCI and franka-robot')
        return np.concatenate([current[s][1] for s in SIDES])

    def call(self, side, operation, request=None, timeout=7):
        client = self.clients[side, operation]
        if not client.wait_for_service(timeout_sec=1):
            raise RuntimeError(f'{side}/{operation}: service unavailable')
        if request is None:
            request = self.ListControllers.Request() if operation in ('list', 'gripper_list') else self.Trigger.Request()
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f'{side}/{operation}: response timeout')
        result = future.result()
        if result is None or (hasattr(result, 'success') and not result.success):
            raise RuntimeError(f'{side}/{operation}: {getattr(result, "message", "failed")}')
        return result

    def foreign_publishers(self):
        conflicts = []
        for side in SIDES:
            for topic in (f'/{side}/follower/gello/joint_states',
                          f'/{side}/follower/gripper/gripper_client/target_gripper_width_percent'):
                for endpoint in self.node.get_publishers_info_by_topic(topic):
                    if (endpoint.node_name, endpoint.node_namespace) != (self.node.get_name(), self.node.get_namespace()):
                        conflicts.append(f'{topic}: {endpoint.node_namespace}/{endpoint.node_name}')
        if conflicts:
            raise RuntimeError('Other command publishers exist. Stop LABS teleoperation first: ' + '; '.join(conflicts))

    def wait_for_command_topics(self, timeout=10, quiet_period=1):
        """Wait for teleop teardown/DDS discovery before creating publishers.

        Unknown endpoints are still conflicts. Only an actually empty graph
        for a continuous quiet period permits preflight to proceed. During
        motion, foreign_publishers() continues to reject conflicts immediately.
        """
        deadline = time.monotonic() + timeout
        quiet_since = None
        reported = False
        last_conflict = None
        while True:
            try:
                self.foreign_publishers()
            except RuntimeError as error:
                last_conflict = error
                quiet_since = None
                if not reported:
                    print(f'Waiting up to {timeout:g}s for teleoperation publishers / ROS discovery '
                          'to clear; arm controllers remain inactive.', flush=True)
                    reported = True
            else:
                now = time.monotonic()
                if quiet_since is None:
                    quiet_since = now
                if now - quiet_since >= quiet_period:
                    if reported:
                        print('Command topics are clear; continuing preflight.', flush=True)
                    return
            if time.monotonic() >= deadline:
                raise RuntimeError(f'Command topics did not remain clear for {quiet_period:g}s '
                                   f'within {timeout:g}s. {last_conflict or "ROS graph kept changing"}')
            time.sleep(.1)

    def preflight(self, home):
        time.sleep(2)  # DDS discovery and fresh measurements, no command publishers yet.
        problems = []
        for check in (self.wait_for_command_topics, self.measured, self.fresh_grippers):
            try:
                check()
            except RuntimeError as error:
                problems.append(str(error))
        for side in SIDES:
            try:
                controllers = {c.name: c.state for c in self.call(side, 'list', timeout=3).controller}
                print(f'{side} controllers: {controllers}', flush=True)
                if controllers.get(CONTROLLER) != 'inactive':
                    problems.append(f'{side}: {CONTROLLER} must be loaded and inactive')
                if controllers.get('gravity_compensation_controller') != 'inactive':
                    problems.append(f'{side}: gravity_compensation_controller must be inactive')
                with self.lock:
                    state, stamp = self.coordinators.get(side, ('UNKNOWN', 0))
                if state != 'IDLE' or time.monotonic() - stamp > 1:
                    problems.append(f'{side}: coordinator must be live and IDLE, got {state}')
                gripper_controllers = {c.name: c.state for c in self.call(side, 'gripper_list', timeout=3).controller}
                for controller in ('robotiq_gripper_controller', 'robotiq_activation_controller'):
                    if gripper_controllers.get(controller) != 'active':
                        problems.append(f'{side}: {controller} must be active')
            except RuntimeError as error:
                problems.append(str(error))
        if problems:
            raise RuntimeError('\n'.join(problems))
        error = np.abs(self.measured() - home)
        print(f'Gripper feedback (knuckle rad): {self.fresh_grippers()}', flush=True)
        print(f'Initial-pose error: left {max(error[:7]):.4f}, right {max(error[7:]):.4f} rad', flush=True)
        print(pose_error_description(self.measured(), home), flush=True)
        return float(max(error))

    def set_command(self, values, grippers=None):
        values = np.asarray(values, dtype=float)
        if values.shape != (14,) or not np.isfinite(values).all():
            raise ValueError('Invalid joint command')
        with self.lock:
            self.command = values.copy()
            self.grippers = None if grippers is None else np.asarray(grippers, dtype=float).copy()

    def publish_loop(self):
        deadline = time.monotonic()
        try:
            while not self.stop_stream.is_set():
                now = time.monotonic()
                if self.last_publish is not None and now - self.last_publish > 0.15:
                    self.stream_error = 'Command publication gap exceeded 0.15 s'
                with self.lock:
                    command, grippers, states = self.command.copy(), self.grippers, dict(self.states)
                for i, side in enumerate(SIDES):
                    msg = self.JointState()
                    msg.header.stamp = self.node.get_clock().now().to_msg()
                    msg.name = states[side][0]
                    msg.position = command[i * 7:(i + 1) * 7].tolist()
                    self.publishers[side, 'arm'].publish(msg)
                    if grippers is not None:
                        self.publishers[side, 'gripper'].publish(self.Float32(data=float(grippers[i])))
                self.last_publish = time.monotonic()
                deadline = max(deadline + 0.01, time.monotonic())
                self.stop_stream.wait(max(0, deadline - time.monotonic()))
        except Exception as error:
            self.stream_error = str(error)

    def start(self):
        self.foreign_publishers()
        self.set_command(self.measured())
        for side in SIDES:
            self.publishers[side, 'arm'] = self.node.create_publisher(
                self.JointState, f'/{side}/follower/gello/joint_states', 1)
            self.ensure_gripper_publisher(side)
        self.stream_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.stream_thread.start()
        time.sleep(0.5)
        for side in SIDES:
            self.check_health(following=False)
            self.touched.append(side)  # Include even a timed-out activation in cleanup.
            self.call(side, 'get_ready')
            self.call(side, 'start_operating')
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            self.check_health(following=False)
            with self.lock:
                ready = all(self.coordinators.get(s, ('', 0))[0] == 'FOLLOWING' for s in SIDES)
            if ready:
                print('Both arms FOLLOWING; initial target is the current measured pose.', flush=True)
                return
            time.sleep(0.02)
        raise RuntimeError('Controllers did not enter FOLLOWING')

    def ensure_gripper_publisher(self, side):
        if (side, 'gripper') not in self.publishers:
            self.publishers[side, 'gripper'] = self.node.create_publisher(
                self.Float32, f'/{side}/follower/gripper/gripper_client/target_gripper_width_percent', 1)

    def verify_grippers(self, output):
        """Re-activate, open, close slightly, and reopen; arm controllers stay inactive."""
        self.foreign_publishers()
        report = {'started': time.strftime('%Y-%m-%dT%H:%M:%S'), 'sides': {}, 'success': False}
        path = output / f'gripper_check_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()}.json'
        try:
            for side in SIDES:
                print(f'{side} gripper: reactivating hardware (automatic calibration).', flush=True)
                self.call(side, 'gripper_reactivate', timeout=20)
                self.ensure_gripper_publisher(side)
                time.sleep(.3)
                observations = []
                report['sides'][side] = observations
                for opening in (1.0, 0.8, 1.0):
                    print(f'{side} gripper: verifying open fraction {opening:.1f}', flush=True)
                    deadline = time.monotonic() + 5
                    matched_since = None
                    observation = {'command_open': opening, 'verified': False}
                    observations.append(observation)
                    while True:
                        self.foreign_publishers()
                        self.publishers[side, 'gripper'].publish(self.Float32(data=opening))
                        measured = self.fresh_grippers()[SIDES.index(side)]
                        observation['measured_rad'] = measured
                        now = time.monotonic()
                        # Current LABS client sends GripperCommand.position = 1 - opening.
                        if abs(measured - (1 - opening)) <= 0.04:
                            if matched_since is None:
                                matched_since = now
                            if now - matched_since >= .3:
                                observation['verified'] = True
                                break
                        else:
                            matched_since = None
                        if now > deadline:
                            raise RuntimeError(f'{side} gripper motion test failed: open command {opening:.2f}, '
                                               f'actual knuckle {measured:.4f} rad (expected {1-opening:.2f}). '
                                               'Check gripper power, activation and serial communication.')
                        time.sleep(.05)
                print(f'{side} gripper motion verified: {observations}', flush=True)
            report['success'] = True
        except Exception as error:
            report['error'] = str(error)
            # This check starts with empty/open grippers. End the probe by reopening.
            try:
                self.foreign_publishers()
                for side in SIDES:
                    if (side, 'gripper') in self.publishers:
                        self.publishers[side, 'gripper'].publish(self.Float32(data=1.0))
            except RuntimeError:
                pass  # Do not override a sender that appeared during the probe.
            raise
        finally:
            path.write_text(json.dumps(report, indent=2) + '\n')
            print(f'Gripper verification report: {path}', flush=True)

    def check_health(self, following=True):
        q = self.measured()
        self.fresh_grippers()
        if self.stream_error:
            raise RuntimeError(self.stream_error)
        if following:
            with self.lock:
                status = dict(self.coordinators)
            for side in SIDES:
                state, stamp = status.get(side, ('UNKNOWN', 0))
                if state != 'FOLLOWING' or time.monotonic() - stamp > 0.5:
                    raise RuntimeError(f'{side}: coordinator is {state} or stopped updating')
        return q

    def stop(self):
        if not self.touched:
            return
        print('Stopping: hold target while deactivating controllers.', flush=True)
        try:
            self.set_command(self.measured())
        except RuntimeError:
            pass  # Hold last sent target if measurements have stopped.
        for side in self.touched:
            try:
                self.call(side, 'stop')
            except Exception as error:
                print(f'Stop response: {error}', flush=True)
        pending = set(self.touched)
        while pending:
            for side in list(pending):
                try:
                    controllers = {c.name: c.state for c in self.call(side, 'list', timeout=2).controller}
                    if CONTROLLER not in controllers or 'gravity_compensation_controller' not in controllers:
                        raise RuntimeError('Cannot verify unloaded controllers')
                    active = [name for name in (CONTROLLER, 'gravity_compensation_controller')
                              if controllers.get(name) == 'active']
                    if not active:
                        pending.remove(side)
                        continue
                    request = self.SwitchController.Request()
                    request.deactivate_controllers = active
                    request.strictness = request.BEST_EFFORT
                    request.timeout.sec = 2
                    self.call(side, 'switch', request, timeout=3)
                except Exception as error:
                    print(f'{side}: stop NOT VERIFIED ({error}). Holding; use physical stop if needed.', flush=True)
            if pending:
                time.sleep(1)
        self.touched.clear()
        print('Both arm controllers verified inactive.', flush=True)

    def close(self):
        self.stop_stream.set()
        if self.stream_thread:
            self.stream_thread.join(timeout=2)
        self.executor.shutdown(timeout_sec=2)
        self.spin_thread.join(timeout=2)
        self.node.destroy_node()
        self.rclpy.shutdown()


def move_home(robot, target, *, command_target=None, output=None):
    """Restore the recorded command, verifying the recorded measured pose.

    A torque/impedance follower has a steady-state command/measurement gap.
    Sending the measured pose as the command discards that gap and can make
    an otherwise reproducible pose unreachable under the existing gains.
    """
    command_target = target if command_target is None else np.asarray(command_target)
    if command_target.shape != (14,) or not np.isfinite(command_target).all():
        raise ValueError('Invalid initial command target')
    handle = None
    writer = None
    if output is not None:
        path = output / f'home_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()}.csv'
        handle = path.open('x', newline='')
        writer = csv.writer(handle)
        writer.writerow(['wall_s', 'phase']
                        + [f'cmd_{s}_j{j}' for s in SIDES for j in range(1, 8)]
                        + [f'meas_{s}_j{j}' for s in SIDES for j in range(1, 8)]
                        + [f'recorded_meas_{s}_j{j}' for s in SIDES for j in range(1, 8)])
        print(f'Homing trace: {path}', flush=True)
    try:
        _move_home(robot, np.asarray(target), command_target, writer)
    finally:
        if handle is not None:
            handle.close()


def _move_home(robot, target, command_target, writer):
    current = robot.measured()
    origin = time.monotonic()

    def log_sample(phase, measured):
        if writer is not None:
            writer.writerow([time.monotonic() - origin, phase]
                            + current.tolist() + measured.tolist() + target.tolist())

    def correct_residual(side, section):
        # Static friction can leave a different offset depending on approach
        # direction. Briefly trim only joints outside tolerance, then restore
        # the exact recorded command before accepting the initial pose.
        baseline = current.copy()
        before = robot.check_health()[section]
        selected = np.abs(before - target[section]) > START_TOL
        trim = np.zeros(7)
        started = previous = time.monotonic()
        print(f'Homing {side}: bounded residual correction, at most 0.06 rad at 0.01 rad/s; '
              'recorded command will be restored before verification.', flush=True)
        while True:
            q = robot.check_health()
            robot.foreign_publishers()
            error = target[section] - q[section]
            now = time.monotonic()
            if np.all(np.abs(error[selected]) <= .035):
                break
            if now - started > 8:
                raise RuntimeError(f'{side}: bounded initial-pose correction failed: '
                                   + pose_error_description(q, target, section))
            if now - started > 2 and np.any(np.abs(q[section][selected] - before[selected]) < .002):
                raise RuntimeError(f'{side}: initial-pose correction produced no joint response')
            trim[selected] += np.sign(error[selected]) * .01 * min(now - previous, .05)
            trim = np.clip(trim, -.06, .06)
            current[section] = baseline[section] + trim
            assert_tracking(current, q)
            robot.set_command(current)
            log_sample(f'{side}_residual_correction', q)
            previous = now
            time.sleep(.01)
        corrected = current.copy()
        started = time.monotonic()
        duration = max(1, float(np.max(np.abs(trim))) * 1.875 / .03)
        while True:
            q = robot.check_health()
            robot.foreign_publishers()
            u = min(1, (time.monotonic() - started) / duration)
            blend = 10 * u**3 - 15 * u**4 + 6 * u**5
            current[section] = corrected[section] + (baseline[section] - corrected[section]) * blend
            assert_tracking(current, q)
            robot.set_command(current)
            log_sample(f'{side}_restore_recorded_command', q)
            if u >= 1:
                break
            time.sleep(.01)

    for i, side in enumerate(SIDES):
        section = slice(i * 7, (i + 1) * 7)
        start = current.copy()
        delta = command_target[section] - start[section]
        duration = max(1, float(np.max(np.abs(delta))) * 1.875 / 0.15)
        print(f'Homing {side}, ramp {duration:.1f}s, maximum target speed 0.15 rad/s', flush=True)
        started = time.monotonic()
        while True:
            q = robot.check_health()
            robot.foreign_publishers()
            u = min(1, (time.monotonic() - started) / duration)
            blend = 10 * u**3 - 15 * u**4 + 6 * u**5
            current[section] = start[section] + delta * blend
            assert_tracking(current, q)
            robot.set_command(current)
            log_sample(f'{side}_ramp', q)
            if u >= 1:
                break
            time.sleep(0.01)
        deadline = time.monotonic() + 10
        settled_since = None
        report_at = 0
        corrected_once = False
        while True:
            q = robot.check_health()
            robot.foreign_publishers()
            assert_tracking(current, q)
            log_sample(f'{side}_settle', q)
            now = time.monotonic()
            if now >= report_at:
                print(f'Homing {side}: {pose_error_description(q, target, section)}', flush=True)
                report_at = now + 1
            if np.max(np.abs(q[section] - target[section])) <= START_TOL:
                if settled_since is None:
                    settled_since = now
                if now - settled_since >= 0.5:
                    break
            else:
                settled_since = None
            if now > deadline:
                if not corrected_once:
                    correct_residual(side, section)
                    corrected_once = True
                    deadline = time.monotonic() + 5
                    settled_since = None
                    continue
                raise RuntimeError(f'{side} did not reach recorded initial pose within {START_TOL:.3f} rad: '
                                   + pose_error_description(q, target, section))
            time.sleep(0.02)
    final_error = float(np.max(np.abs(robot.check_health() - target)))
    if final_error > START_TOL:
        raise RuntimeError('Initial pose verification failed after homing: '
                           + pose_error_description(robot.check_health(), target))
    print(f'Both arms reached recorded initial pose (max error {final_error:.4f} rad); grippers were not commanded.', flush=True)


def replay(robot, times, actions, fps, speed, output, recorded_grippers=None, recorded_joints=None):
    duration = len(times) / fps / speed
    path = output / f'trace_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns()}.csv'
    print(f'Replaying at {speed:g}x for {duration:.2f}s; trace: {path}', flush=True)
    # Homing already holds action[0]. Keep the same arm command while setting
    # initial gripper widths; re-anchoring on measured joints would undo homing.
    for _ in range(101):
        q = robot.check_health()
        robot.foreign_publishers()
        target = actions[0, ARM_COLUMNS]
        assert_tracking(target, q)
        robot.set_command(target, actions[0, [7, 15]])
        time.sleep(0.01)
    grip_guard = GripperMotionGuard()
    with path.open('x', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['wall_s', 'recorded_s', 'max_tracking_error_rad']
                        + [f'cmd_{s}_j{j}' for s in SIDES for j in range(1, 8)]
                        + [f'meas_{s}_j{j}' for s in SIDES for j in range(1, 8)]
                        + ['left_gripper_open', 'right_gripper_open',
                           'left_gripper_measured_rad', 'right_gripper_measured_rad',
                           'left_gripper_recorded_rad', 'right_gripper_recorded_rad']
                        + [f'tracking_limit_{s}_j{j}' for s in SIDES for j in range(1, 8)])
        started = time.monotonic()
        next_graph_check = next_report = started
        previous_tick = started
        while True:
            now = time.monotonic()
            if now - previous_tick > 0.15:
                raise RuntimeError('Replay scheduler paused for over 0.15 s; stopping')
            previous_tick = now
            elapsed = min((now - started) * speed, len(times) / fps)
            action = sample_action(times, actions, elapsed)
            measured = robot.check_health()
            gripper_measured = robot.fresh_grippers()
            grip_guard.check(now, action[[7, 15]], gripper_measured)
            if recorded_joints is None:
                tracking_limits = np.full(14, MAX_TRACKING_ERROR)
            else:
                reference_q = np.array([np.interp(elapsed, times, recorded_joints[:, i]) for i in range(14)])
                tracking_limits = recorded_tracking_limits(action[ARM_COLUMNS], reference_q, speed)
            error = assert_tracking(action[ARM_COLUMNS], measured, tracking_limits)
            if now >= next_graph_check:
                robot.foreign_publishers()
                next_graph_check = now + 0.5
            robot.set_command(action[ARM_COLUMNS], action[[7, 15]])
            recorded_grip = ([float(np.interp(elapsed, times, recorded_grippers[:, i])) for i in range(2)]
                             if recorded_grippers is not None else ['', ''])
            writer.writerow([now - started, elapsed, error]
                            + action[ARM_COLUMNS].tolist() + measured.tolist() + action[[7, 15]].tolist()
                            + list(gripper_measured) + recorded_grip + tracking_limits.tolist())
            if now >= next_report:
                handle.flush()
                print(f'{elapsed:.1f}/{len(times)/fps:.1f} recorded seconds; arm error {error:.3f} rad; '
                      f'gripper open cmd L/R={action[7]:.3f}/{action[15]:.3f}, '
                      f'measured rad={gripper_measured[0]:.3f}/{gripper_measured[1]:.3f}', flush=True)
                next_report = now + 5
            if now - started >= duration:
                break
            time.sleep(0.01)
    print('Replay timeline completed. Confirm task outcome from the scene.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['inspect', 'check', 'gripper-check', 'home', 'run'])
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=1.0)
    args = parser.parse_args()
    if not np.isfinite(args.speed) or not 0 < args.speed <= 1:
        parser.error('--speed must be between 0 (exclusive) and 1')
    times, actions, measured, fps, manifest = load_bundle(args.bundle)
    print(f'Episode {manifest["episode"]}: {len(times)} frames, {fps:g} Hz, both arms', flush=True)
    if args.mode == 'inspect':
        print(json.dumps(manifest, indent=2))
        return 0
    if args.mode == 'run':
        try:
            tracking_limits = recorded_tracking_limits(actions[:, ARM_COLUMNS], measured, args.speed)
        except ValueError as error:
            print(f'REPLAY REFUSED: {error}', flush=True)
            return 1
        if np.max(tracking_limits) > MAX_TRACKING_ERROR:
            print(f'Replay tracking limits follow recorded lag + {RECORDED_TRACKING_MARGIN:.2f} rad per joint/time; '
                  f'maximum for this episode {np.max(tracking_limits):.3f} rad (ceiling {MAX_REPLAY_TRACKING_LIMIT:.2f}).', flush=True)
        if np.max(np.abs(actions[:, ARM_COLUMNS] - measured)) + RECORDED_TRACKING_MARGIN > MAX_REPLAY_TRACKING_LIMIT:
            print(f'Recorded lag exceeds the ceiling: {args.speed:g}x monitored replay; '
                  'allowances capped at 0.50 rad. Tracking at this speed requires physical verification.', flush=True)
    gripper_reference = load_gripper_reference(args.bundle, len(times)) if args.mode == 'run' else None
    robot = Robot()
    code = 0
    try:
        error = robot.preflight(measured[0])
        if args.mode == 'check':
            print(f'Controller check passed; ready at starting pose: {error <= START_TOL}', flush=True)
            return 0
        if args.mode in ('gripper-check', 'run'):
            robot.verify_grippers(args.bundle)
        if args.mode == 'gripper-check':
            return 0
        robot.start()
        move_home(robot, measured[0], command_target=actions[0, ARM_COLUMNS], output=args.bundle)
        if args.mode == 'run':
            replay(robot, times, actions, fps, args.speed, args.bundle, gripper_reference, measured)
    except KeyboardInterrupt:
        print('Stop requested.', flush=True)
        code = 130
    except Exception as error:
        print(f'REPLAY REFUSED/STOPPED: {error}', flush=True)
        code = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        robot.stop()
        robot.close()
    return code


if __name__ == '__main__':
    def request_stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    raise SystemExit(main())
