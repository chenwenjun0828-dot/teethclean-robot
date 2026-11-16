"""Small ROS 1 MoveIt replay client. Trajectory data comes only from JSON."""
import argparse
import copy
from contextlib import nullcontext
import json
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

from .trajectory_contract import sha256, write_json, unique_json_object, quintic, bernstein_bounds
from .ros1_trajectory import fill_joint_trajectory
from .standalone_keyboard import KeyboardControl


MOVEIT_OPTIONS = dict(group='manipulator', base_frame='base_link', tip_frame='tool0',
    joint_states='/joint_states', execute_action='/execute_trajectory',
    robot_description='robot_description', joint_limits_param='robot_description_planning/joint_limits',
    start_tolerance_rad=.01, stopped_rad_s=.01, state_timeout_s=1., stop_timeout_s=5.,
    settle_s=.2, poll_s=.02, joint_velocity_limits=None, joint_acceleration_limits=None)
MOVEIT_POINT_FIELDS = {'positions', 'velocities', 'accelerations', 'time_from_start'}
MOVEIT_TERMINAL_STATES = (2, 3, 4, 5, 8, 9)


class MoveItPoints:
    def __init__(self, payload):
        if not isinstance(payload, dict) or set(payload) != {'joint_names', 'points'}:
            raise ValueError('Expected plain joint_names/points JSON')
        self.record = payload
        self.names = payload['joint_names']
        if (not isinstance(self.names, list) or len(self.names) != 6
                or any(not isinstance(n, str) or not n.strip() for n in self.names)
                or len(set(self.names)) != 6):
            raise ValueError('Expected six unique joint names')
        points = payload['points']
        if (not isinstance(points, list) or len(points) < 2
                or any(not isinstance(p, dict) or set(p) != MOVEIT_POINT_FIELDS for p in points)):
            raise ValueError('Expected at least two points with exactly four joint/time fields')
        self.times = np.asarray([p['time_from_start'] for p in points], float)
        self.q, self.v, self.a = [np.asarray([p[k] for p in points], float)
                                 for k in ('positions', 'velocities', 'accelerations')]
        if (self.times.shape != (len(points),) or not np.isfinite(self.times).all()
                or self.times[0] != 0 or np.any(self.times < 0) or np.any(self.times >= 2147483648)
                or np.any(np.diff(np.rint(self.times*1e9).astype(np.int64)) < 1)
                or any(x.shape != (len(points), 6) or not np.isfinite(x).all()
                       for x in (self.q, self.v, self.a))
                or not np.allclose(self.v[[0, -1]], 0, atol=1e-8)
                or not np.allclose(self.a[[0, -1]], 0, atol=1e-8)):
            raise ValueError('Invalid points, nanosecond clock or stopped endpoints')
        self.duration = float(self.times[-1])

    def retimed(self, factor):
        if isinstance(factor, bool) or not np.isfinite(factor) or factor <= 0:
            raise ValueError('speed_factor must be finite and positive')
        if factor == 1.:
            return self
        payload = copy.deepcopy(self.record)
        with np.errstate(over='raise', invalid='raise'):
            try:
                for p, t, v, a in zip(payload['points'], self.times, self.v, self.a):
                    scaled = t/factor
                    if not np.isfinite(scaled) or scaled >= 2147483648:
                        raise ValueError('Scaled clock exceeds ROS Duration range')
                    p.update(time_from_start=round(scaled*1e9)/1e9,
                             velocities=(v*factor).tolist(), accelerations=(a*factor*factor).tolist())
            except FloatingPointError as error:
                raise ValueError('Scaled derivatives are not finite') from error
        return MoveItPoints(payload)

    def speed_audit(self, limits):
        audit, blockers = {}, []
        for key, derivative in (('velocity', 1), ('acceleration', 2)):
            value = limits.get(key)
            if (not isinstance(value, list) or len(value) != 6 or any(
                    isinstance(x, bool) or not isinstance(x, (int, float)) or not np.isfinite(x) or x <= 0
                    for x in value)):
                blockers.append('missing_joint_'+key+'_limits')
        if blockers:
            return {'passed': False, 'blockers': blockers}
        bounds = {key: np.zeros(6) for key in ('velocity', 'acceleration')}
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            for i, dt in enumerate(np.diff(self.times)):
                c = quintic(self.q[i], self.q[i+1], self.v[i], self.v[i+1], self.a[i], self.a[i+1], dt)
                for key, derivative in (('velocity', 1), ('acceleration', 2)):
                    low, high = bernstein_bounds(c, derivative, dt)
                    bounds[key] = np.maximum(bounds[key], np.maximum(abs(low), abs(high)))
        for key, bound in bounds.items():
            if not np.isfinite(bound).all():
                raise ValueError('Nonfinite continuous speed bounds')
            audit[key+'_bound'] = bound.tolist()
            blockers += ['joint_'+key+'_exceeded.'+self.names[i]
                         for i in range(6) if bound[i] > limits[key][i]]
        return dict(audit, passed=not blockers, blockers=blockers, limits=limits,
                    method='quintic_bernstein_conservative_bounds')


def moveit_limits(names, settings, urdf):
    """Acceleration is not inferred from URDF velocity or planning scale factors."""
    result = {'velocity': [None]*6, 'acceleration': [None]*6}
    joints = {j.get('name'): j for j in ET.fromstring(urdf).findall('joint')}
    for i, name in enumerate(names):
        entry = settings.get(name, {})
        for key in result:
            candidates = []
            if entry.get('has_'+key+'_limits') is True:
                value = entry.get('max_'+key)
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                    raise ValueError('Invalid MoveIt '+key+' limit for '+name)
                candidates.append(float(value))
            joint = joints.get(name)
            limit = joint.find('limit') if joint is not None else None
            if key == 'velocity' and limit is not None and limit.get('velocity') is not None:
                value = float(limit.get('velocity'))
                if not np.isfinite(value) or value <= 0:
                    raise ValueError('Invalid URDF velocity limit for '+name)
                candidates.append(value)
            result[key][i] = min(candidates) if candidates else None
    return result


def effective_moveit_limits(options, model=None):
    result = {}
    for key in ('velocity', 'acceleration'):
        supplied = options['joint_'+key+'_limits']
        actual = (model or {}).get(key, [None]*6)
        if supplied is not None:
            if len(supplied) != 6 or any(isinstance(x, bool) or not np.isfinite(x) or x <= 0 for x in supplied):
                raise ValueError('Expected six finite positive joint '+key+' limits')
            actual = [min(a, b) if a is not None else b for a, b in zip(actual, supplied)]
        result[key] = actual
    return result


class MoveItBackend:
    def __init__(self, options, curve):
        import actionlib
        import moveit_commander
        import rospy
        from moveit_msgs.msg import ExecuteTrajectoryAction, ExecuteTrajectoryGoal
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectoryPoint
        self.subscriber = None
        self.rospy, self.lock = rospy, threading.RLock()
        self.state, self.received, self.sequence = None, None, 0
        self.error, self.sent = None, False
        self.curve, self.options = curve, options
        moveit_commander.roscpp_initialize([])
        if not rospy.core.is_initialized():
            rospy.init_node('saved_trajectory_moveit', anonymous=True, disable_signals=True)
        if rospy.get_param('/use_sim_time', False):
            raise ValueError('Real execution requires /use_sim_time=false')
        self.group = moveit_commander.MoveGroupCommander(options['group'],
            robot_description=options['robot_description'], wait_for_servers=options['state_timeout_s'])
        self.group.set_end_effector_link(options['tip_frame'])
        self.group.set_pose_reference_frame(options['base_frame'])
        self.names = list(self.group.get_active_joints())
        if len(self.names) != 6 or set(self.names) != set(curve.names):
            raise ValueError('MoveIt group does not match trajectory joints')
        self.client = actionlib.SimpleActionClient(options['execute_action'], ExecuteTrajectoryAction)
        if not self.client.wait_for_server(rospy.Duration(options['state_timeout_s'])) or not callable(self.group.stop):
            raise ValueError('MoveIt execution/cancellation interface unavailable')
        self.goal = ExecuteTrajectoryGoal()
        fill_joint_trajectory(curve, self.goal.trajectory.joint_trajectory, JointTrajectoryPoint, rospy.Duration)
        self.subscriber = rospy.Subscriber(options['joint_states'], JointState, self._joints, queue_size=1)

    def limits(self):
        return moveit_limits(self.curve.names, self.rospy.get_param(self.options['joint_limits_param'], {}),
                             self.rospy.get_param(self.options['robot_description']))

    def _joints(self, msg):
        with self.lock:
            try:
                if len(set(msg.name)) != len(msg.name):
                    raise ValueError('Duplicate joint state names')
                order = [msg.name.index(name) for name in self.curve.names]
                q, dq = ([float(msg.position[i]) for i in order], [float(msg.velocity[i]) for i in order])
                if not np.isfinite(q+dq).all():
                    raise ValueError('Nonfinite joint state')
                self.sequence += 1
                self.state = dict(q=q, dq=dq, stamp=float(msg.header.stamp.to_sec()), sequence=self.sequence)
                self.received = time.monotonic()
            except Exception as error:
                self.error = str(error)

    def snapshot(self):
        with self.lock:
            state = dict(self.state or {})
            stamp_age = self.rospy.Time.now().to_sec()-state.get('stamp', 0.)
            state['age'] = (max(time.monotonic()-self.received, stamp_age)
                            if self.received is not None and state['stamp'] > 0 and stamp_age >= -.01 else float('inf'))
            state['error'] = self.error
        state['action_state'] = self.client.get_state() if self.sent else None
        result = self.client.get_result() if state['action_state'] in MOVEIT_TERMINAL_STATES else None
        state['result_code'] = result.error_code.val if result is not None else None
        return state

    def send(self, curve):
        if curve is not self.curve or self.sent:
            raise ValueError('Only the checked trajectory may be sent once')
        self.sent = True
        # MoveGroupCommander.execute uses this same ExecuteTrajectory action.
        self.client.send_goal(self.goal)

    def stop(self):
        errors = []
        for method in (self.client.cancel_goal, self.group.stop):
            try:
                method()
            except Exception as error:
                errors.append(str(error))
        return errors

    def close(self):
        if self.subscriber is not None:
            self.subscriber.unregister()


def moveit_state_fault(state, options, target=None):
    if state.get('error'):
        return 'invalid_joint_state: '+state['error']
    age = state.get('age', float('inf'))
    if not np.isfinite(age) or not 0 <= age <= options['state_timeout_s']:
        return 'stale_joint_state'
    q, dq = np.asarray(state.get('q', [])), np.asarray(state.get('dq', []))
    if q.shape != (6,) or dq.shape != (6,) or not np.isfinite(np.r_[q, dq]).all():
        return 'invalid_joint_state'
    if target is not None:
        if np.max(abs(q-target)) > options['start_tolerance_rad']:
            return 'joint_position_mismatch'
        if np.max(abs(dq)) > options['stopped_rad_s']:
            return 'robot_not_stopped'
    return None


def moveit_confirm_stop(backend, options, now, sleep, log):
    errors = []
    try:
        baseline = backend.snapshot().get('stamp', 0.)
    except (Exception, KeyboardInterrupt) as error:
        baseline = 0.
        errors.append(str(error) or type(error).__name__)
    try:
        errors.extend(backend.stop() or [])
    except Exception as error:
        errors.append(str(error))
    start, stable, first_stamp = now(), None, None
    while now()-start < options['stop_timeout_s']:
        try:
            state = backend.snapshot()
            log.write(json.dumps(dict(stage='stopping', **state), allow_nan=False)+'\n')
            stopped = (moveit_state_fault(state, options) is None
                and state.get('action_state') in (2, 3, 4, 5, 8)
                and state.get('stamp', 0.) > baseline
                and max(abs(x) for x in state['dq']) <= options['stopped_rad_s'])
            if stopped:
                if stable is None:
                    stable, first_stamp = now(), state['stamp']
                if now()-stable >= options['settle_s'] and state['stamp'] > first_stamp:
                    return dict(confirmed=True, elapsed_s=now()-start, errors=errors)
            else:
                stable = None
        except Exception as error:
            stable = None
            errors.append(str(error))
        sleep(options['poll_s'])
    return dict(confirmed=False, elapsed_s=now()-start, errors=errors)


class MoveItOperatorStop(RuntimeError):
    pass


def moveit_operation(command, trajectory, output=None, *, speed_factor=1., operator=None,
                     options=None, expected_sha256=None, allow_retiming=None, backend_factory=None,
                     now=time.monotonic, sleep=time.sleep):
    if command not in ('inspect', 'check', 'execute') or (operator is not None and command != 'execute'):
        raise ValueError('Invalid operation or keyboard mode')
    opts = dict(MOVEIT_OPTIONS, **(options or {}))
    for key, value in opts.items():
        if isinstance(MOVEIT_OPTIONS[key], float):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError('Expected finite positive '+key)
    path = Path(trajectory).resolve()
    digest = sha256(path)
    expected = expected_sha256 if expected_sha256 is not None else globals().get('EXPECTED_TRAJECTORY_SHA256')
    if not expected or digest != expected:
        raise ValueError('Trajectory does not match this script SHA256')
    original = MoveItPoints(json.loads(path.read_bytes().decode('utf-8'), object_pairs_hook=unique_json_object))
    curve = original.retimed(speed_factor)
    retiming = allow_retiming if allow_retiming is not None else globals().get('ALLOW_RETIMING', False)
    if speed_factor != 1. and not retiming:
        raise ValueError('Full-policy retiming requires a new audit/export')
    script_path = globals().get('MOVEIT_SCRIPT_PATH')
    script_digest = sha256(script_path) if script_path else None

    def unchanged():
        if sha256(path) != digest or (script_path and sha256(script_path) != script_digest):
            raise ValueError('input_or_script_changed_before_send')

    unchanged()
    report = dict(status='blocked', backend='moveit', points=len(curve.times), joint_names=curve.names,
        trajectory_sha256=digest, original_duration_s=original.duration, commanded_duration_s=curve.duration,
        speed_factor=speed_factor, motion_sent=False, motion_send_attempted=False, blockers=[],
        collision_dynamics_verified=False, hardware_validated=False, keyboard_control=operator is not None,
        source_validation='original_bundle_verified_at_build_time',
        runtime_validation='plain_json_sha256_and_joint_state',
        monitoring='joint_state_and_moveit_action_only')
    audit = curve.speed_audit(effective_moveit_limits(opts)) if speed_factor > 1 else None
    if command == 'inspect':
        if audit and not audit['passed']:
            report['blockers'] = audit['blockers']
        report.update(status='blocked' if report['blockers'] else 'offline_checks_passed', speed_limit_audit=audit)
        return report
    if output is None:
        raise ValueError('check/execute requires --out')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    backend, attempted = None, False

    def check_operator():
        reason = operator.stop_reason() if operator is not None else None
        if reason is not None:
            raise MoveItOperatorStop(reason)

    with (output/'states.jsonl').open('w', encoding='utf-8', buffering=1) as log:
        def snapshot(stage):
            state = backend.snapshot()
            # Stale/missing observations may contain infinity; log it as null.
            clean = {k: None if isinstance(v, float) and not np.isfinite(v) else v for k, v in state.items()}
            log.write(json.dumps(dict(stage=stage, **clean), allow_nan=False)+'\n')
            return state
        try:
            check_operator()
            if audit and any(not b.startswith('missing_joint_') for b in audit['blockers']):
                report['blockers'] = audit['blockers']
                return report
            backend = (MoveItBackend if backend_factory is None else backend_factory)(opts, curve)
            if len(backend.names) != 6 or set(backend.names) != set(curve.names):
                raise ValueError('MoveIt group does not match trajectory joints')
            if not callable(getattr(backend, 'stop', None)):
                raise ValueError('MoveIt cancellation interface unavailable')
            if speed_factor > 1:
                audit = curve.speed_audit(effective_moveit_limits(opts, backend.limits()))
                report['speed_limit_audit'] = audit
                if not audit['passed']:
                    report['blockers'] = audit['blockers']
                    return report
            start, stable = now(), None
            while True:
                check_operator()
                state = snapshot('preflight')
                fault = moveit_state_fault(state, opts, curve.q[0])
                if fault is None:
                    stable = now() if stable is None else stable
                    if now()-stable >= opts['settle_s']:
                        break
                elif fault != 'stale_joint_state':
                    raise RuntimeError(fault)
                else:
                    stable = None
                if now()-start > opts['state_timeout_s']+opts['settle_s']:
                    raise RuntimeError(fault or 'preflight_timeout')
                sleep(opts['poll_s'])
            if command == 'check':
                report['status'] = 'read_only_checked'
                return report
            if operator is not None:
                operator.arm()
                wait_start = now()
                while True:
                    check_operator()
                    fault = moveit_state_fault(snapshot('waiting_start'), opts, curve.q[0])
                    if fault:
                        raise RuntimeError(fault)
                    if operator.start_requested():
                        break
                    sleep(opts['poll_s'])
                report['operator_wait_s'] = now()-wait_start
            unchanged()
            fault = moveit_state_fault(snapshot('before_send'), opts, curve.q[0])
            if fault:
                raise RuntimeError(fault)
            check_operator()
            unchanged()
            attempted = True
            report.update(motion_sent=None, motion_send_attempted=True)
            backend.send(curve)
            report['motion_sent'] = True
            stable, first_stamp = None, None
            while True:
                check_operator()
                state = snapshot('executing')
                check_operator()
                fault = moveit_state_fault(state, opts)
                if fault:
                    raise RuntimeError(fault)
                status = state.get('action_state')
                if status in MOVEIT_TERMINAL_STATES:
                    if status != 3 or state.get('result_code') != 1:
                        raise RuntimeError('moveit_execution_failed: '+str(status))
                    fault = moveit_state_fault(state, opts, curve.q[-1])
                    if fault:
                        raise RuntimeError('final_'+fault)
                    if stable is None:
                        stable, first_stamp = now(), state.get('stamp', 0.)
                    if now()-stable >= opts['settle_s'] and state.get('stamp', 0.) > first_stamp:
                        report['status'] = 'completed'
                        return report
                elif status not in (0, 1, 6, 7):
                    raise RuntimeError('invalid_moveit_action_state')
                sleep(opts['poll_s'])
        except MoveItOperatorStop as error:
            report['operator_stop_reason'] = str(error)
            report['status'] = ('operator_stopped' if attempted else 'cancelled_before_start') if str(error) == 'operator_space' else 'fault_latched'
            if str(error) != 'operator_space':
                report['fault'] = str(error)
        except (Exception, KeyboardInterrupt) as error:
            report.update(status='fault_latched', fault=str(error) or type(error).__name__)
        finally:
            if attempted and report['status'] != 'completed':
                try:
                    report['stop'] = moveit_confirm_stop(backend, opts, now, sleep, log)
                except (Exception, KeyboardInterrupt) as error:
                    report['stop'] = dict(confirmed=False, errors=[str(error) or type(error).__name__])
                if not report['stop']['confirmed']:
                    report.update(status='fault_latched', stop_fault='stop_not_confirmed')
            if backend is not None:
                try:
                    backend.close()
                except Exception as error:
                    report['close_error'] = str(error)
            write_json(output/'run_report.json', report)
    return report


def moveit_main(argv=None):
    parser = argparse.ArgumentParser(description='Standalone ROS 1 MoveIt joint trajectory replay')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('inspect', 'check', 'execute'):
        command = commands.add_parser(name)
        command.add_argument('--trajectory', required=True)
        command.add_argument('--speed-factor', type=float, default=1.)
        for key, default in MOVEIT_OPTIONS.items():
            command.add_argument('--'+key.replace('_', '-'), default=default,
                **({'nargs': 6, 'type': float} if key.endswith('_limits')
                   else {'type': float} if isinstance(default, float) else {}))
        if name != 'inspect':
            command.add_argument('--out', required=True)
        if name == 'execute':
            command.add_argument('--keyboard', action='store_true')
    args = parser.parse_args(argv)
    try:
        context = KeyboardControl() if getattr(args, 'keyboard', False) else nullcontext(None)
        with context as operator:
            result = moveit_operation(args.command, args.trajectory, getattr(args, 'out', None),
                speed_factor=args.speed_factor, operator=operator,
                options={k: getattr(args, k) for k in MOVEIT_OPTIONS})
        print(json.dumps(result, indent=2, allow_nan=False))
        return 0 if result['status'] in ('offline_checks_passed', 'read_only_checked', 'completed',
                                         'cancelled_before_start', 'operator_stopped') else 2
    except (ValueError, RuntimeError, OSError, KeyError, TypeError, ImportError, ET.ParseError) as error:
        print('MoveIt executor refused: '+str(error))
        return 2
