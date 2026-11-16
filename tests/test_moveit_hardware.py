"""Minimal MoveIt delivery, transport messages and stop lifecycle without devices."""
import ast
import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from control.hardware.moveit_runtime import (MOVEIT_OPTIONS, MoveItBackend, MoveItPoints,
    effective_moveit_limits, moveit_limits, moveit_operation)
from control.hardware.standalone_pack import pack_standalone
from control.hardware.trajectory_contract import sha256, write_json, read_json
from tests.test_keyboard_hardware import ScriptedKeyboard
from tests.test_ros1_hardware import Clock, make_bundle


class MockMoveItBackend:
    def __init__(self, options, curve, clock, *, stop_ok=True, cancel_ack=True,
                 frozen_stamp=False, fault=None, fail_send=False):
        self.curve, self.clock = curve, clock
        self.names = list(reversed(curve.names))
        self.sent = self.cancelled = self.steps = 0
        self.closed = False
        self.stop_ok, self.cancel_ack, self.frozen_stamp = stop_ok, cancel_ack, frozen_stamp
        self.fault, self.fail_send = fault, fail_send

    def limits(self):
        return {'velocity': [100.]*6, 'acceleration': [100.]*6}

    def snapshot(self):
        if self.sent and not self.cancelled:
            self.steps += 1
        final = self.sent and self.steps >= 3
        q = self.curve.q[-1] if final else self.curve.q[0]
        dq = [1. if self.cancelled and not self.stop_ok else 0.]*6
        status = (2 if self.cancel_ack else 1) if self.cancelled else (3 if final else 1)
        state = dict(q=q.tolist(), dq=dq, age=0., stamp=10.+(0 if self.frozen_stamp else self.clock.now()),
                     sequence=self.steps, action_state=status if self.sent else None, result_code=1)
        if self.fault:
            self.fault(self, state)
        return state

    def send(self, curve):
        if curve is not self.curve or self.sent:
            raise AssertionError('Unexpected replay/retiming')
        self.sent += 1
        if self.fail_send:
            raise RuntimeError('ambiguous_send_failure')

    def stop(self):
        self.cancelled += 1
        return []

    def close(self):
        self.closed = True


class MoveItHardwareTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='moveit_replay_test_')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root/'source'
        self.source.mkdir()
        make_bundle(self.source, ready=False)
        pack_standalone(self.source/'trajectory.json', None, self.root/'delivery', format='plain',
                        backend='moveit', allow_geometry_only=True)
        self.path = self.root/'delivery'/'trajectory.json'
        self.script = self.path.parent/'ur7e_saved_trajectory_executor.py'
        self.payload = read_json(self.path)
        self.payload['points'][-1]['positions'] = [.05]*6
        write_json(self.path, self.payload)
        self.digest = sha256(self.path)
        self.backends = []

    def invoke(self, *, command='execute', factor=1., operator=None, options=None,
               on_sleep=None, operation=moveit_operation, **backend_kwargs):
        clock = Clock()
        def factory(opts, curve):
            backend = MockMoveItBackend(opts, curve, clock, **backend_kwargs)
            self.backends.append(backend)
            return backend
        def sleep(duration):
            clock.sleep(duration)
            if on_sleep:
                on_sleep(clock)
        return operation(command, self.path, self.root/'run', speed_factor=factor,
            operator=operator, options=options, expected_sha256=self.digest, allow_retiming=True,
            backend_factory=factory, now=clock.now, sleep=sleep)

    def test_legacy_defaults_and_no_extra_motion(self):
        self.assertEqual([MOVEIT_OPTIONS[k] for k in ('group', 'base_frame', 'tip_frame')],
                         ['manipulator', 'base_link', 'tool0'])
        result = self.invoke()
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(self.backends[0].sent, 1)
        self.assertEqual(self.backends[0].cancelled, 0)
        self.assertTrue(self.backends[0].closed)
        self.assertFalse(result['collision_dynamics_verified'])

    def test_read_only_never_sends_or_stops(self):
        result = self.invoke(command='check')
        self.assertEqual(result['status'], 'read_only_checked')
        self.assertEqual((self.backends[0].sent, self.backends[0].cancelled), (0, 0))

    def test_inspect_without_ros_and_speedup_needs_limits_offline(self):
        result = self.invoke(command='inspect', factor=2.)
        self.assertEqual(result['status'], 'blocked')
        self.assertIn('missing_joint_acceleration_limits', result['blockers'])
        self.assertEqual(self.backends, [])

    def test_speedup_uses_model_limits_and_preserves_input(self):
        result = self.invoke(factor=2.)
        self.assertEqual(result['status'], 'completed')
        curve = self.backends[0].curve
        np.testing.assert_array_equal(curve.q, [p['positions'] for p in self.payload['points']])
        self.assertEqual(curve.duration, .5)
        self.assertEqual(sha256(self.path), self.digest)
        self.assertTrue(result['speed_limit_audit']['passed'])

    def test_speedup_without_model_acceleration_refuses(self):
        with patch.object(MockMoveItBackend, 'limits', return_value={'velocity': [1.]*6}):
            result = self.invoke(factor=2.)
        self.assertEqual(result['status'], 'blocked')
        self.assertIn('missing_joint_acceleration_limits', result['blockers'])
        self.assertEqual(self.backends[0].sent, 0)

    def test_explicit_limits_cannot_exceed_lower_model_limits(self):
        with patch.object(MockMoveItBackend, 'limits', return_value={
                'velocity': [.001]*6, 'acceleration': [.001]*6}):
            result = self.invoke(factor=2., options={'joint_velocity_limits': [100.]*6,
                                                   'joint_acceleration_limits': [100.]*6})
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(self.backends[0].sent, 0)

    def test_space_before_start_does_not_send_or_stop(self):
        control = ScriptedKeyboard(lambda c: c.handle_key(' '))
        result = self.invoke(operator=control)
        self.assertEqual(result['status'], 'cancelled_before_start')
        self.assertEqual((self.backends[0].sent, self.backends[0].cancelled), (0, 0))

    def test_s_waits_then_sends_once(self):
        control = ScriptedKeyboard()
        control.handle_key('S')
        def start(clock):
            if control.armed and not control.start_requested() and clock.now() > .7:
                self.assertEqual(self.backends[0].sent, 0)
                for key in 'SsSS':
                    control.handle_key(key)
        result = self.invoke(operator=control, on_sleep=start)
        self.assertEqual(result['status'], 'completed')
        self.assertGreater(result['operator_wait_s'], .4)
        self.assertEqual(self.backends[0].sent, 1)

    def test_space_stops_and_confirms_with_new_joint_samples(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        def stop(backend, state):
            if backend.sent:
                control.handle_key(' ')
        result = self.invoke(operator=control, fault=stop)
        self.assertEqual(result['status'], 'operator_stopped')
        self.assertTrue(result['stop']['confirmed'])
        self.assertEqual((self.backends[0].sent, self.backends[0].cancelled), (1, 1))

    def test_stopped_velocity_without_cancel_ack_or_new_samples_is_not_confirmation(self):
        for kwargs in ({'cancel_ack': False}, {'stop_ok': False}, {'frozen_stamp': True}):
            with self.subTest(kwargs=kwargs):
                control = ScriptedKeyboard(lambda c: c.handle_key('S'))
                result = self.invoke(operator=control, fault=lambda b, s: control.handle_key(' ') if b.sent else None,
                                     **kwargs)
                self.assertEqual(result['status'], 'fault_latched')
                self.assertFalse(result['stop']['confirmed'])
                (self.root/'run').rename(self.root/next(iter(kwargs)))

    def test_input_failure_stops_active_execution(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        result = self.invoke(operator=control,
            fault=lambda b, s: control.fail('keyboard_input_eof') if b.sent else None)
        self.assertEqual(result['status'], 'fault_latched')
        self.assertTrue(result['stop']['confirmed'])

    def test_ctrl_c_stops_active_execution(self):
        def interrupt(backend, state):
            if backend.sent and not backend.cancelled:
                raise KeyboardInterrupt()
        result = self.invoke(fault=interrupt)
        self.assertEqual(result['fault'], 'KeyboardInterrupt')
        self.assertTrue(result['stop']['confirmed'])

    def test_ambiguous_send_failure_still_cancels(self):
        result = self.invoke(fail_send=True)
        self.assertTrue(result['motion_send_attempted'])
        self.assertIsNone(result['motion_sent'])
        self.assertEqual(result['fault'], 'ambiguous_send_failure')
        self.assertTrue(result['stop']['confirmed'])

    def test_start_mismatch_refuses_without_moving_to_start(self):
        result = self.invoke(fault=lambda b, s: s.update(q=[2.]*6))
        self.assertEqual(result['fault'], 'joint_position_mismatch')
        self.assertEqual((self.backends[0].sent, self.backends[0].cancelled), (0, 0))

    def test_stale_joint_stream_stops_active_execution(self):
        result = self.invoke(fault=lambda b, s: s.update(age=10.) if b.sent and not b.cancelled else None)
        self.assertEqual(result['fault'], 'stale_joint_state')
        self.assertTrue(result['stop']['confirmed'])

    def test_moveit_failure_or_wrong_final_position_does_not_complete(self):
        for kind in ('action', 'final'):
            def fail(backend, state):
                if backend.sent and not backend.cancelled and state['action_state'] == 3:
                    state.update(result_code=-1) if kind == 'action' else state.update(q=[1.]*6)
            result = self.invoke(fault=fail)
            self.assertEqual(result['status'], 'fault_latched')
            self.assertTrue(result['stop']['confirmed'])
            (self.root/'run').rename(self.root/kind)

    def test_json_change_during_s_wait_refuses(self):
        def change(control):
            self.path.write_bytes(self.path.read_bytes()+b' ')
            control.handle_key('S')
        result = self.invoke(operator=ScriptedKeyboard(change))
        self.assertEqual(result['fault'], 'input_or_script_changed_before_send')
        self.assertEqual(self.backends[0].sent, 0)

    def test_modified_script_during_s_wait_refuses(self):
        spec = importlib.util.spec_from_file_location('modified_moveit_replay', self.script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        def change(control):
            self.script.write_bytes(self.script.read_bytes()+b'\n')
            control.handle_key('S')
        result = self.invoke(operator=ScriptedKeyboard(change), operation=module.moveit_operation)
        self.assertEqual(result['fault'], 'input_or_script_changed_before_send')
        self.assertEqual(self.backends[0].sent, 0)

    def test_no_matching_hash_duplicate_fields_or_invalid_endpoints(self):
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            moveit_operation('inspect', self.path, expected_sha256='0'*64)
        with self.assertRaisesRegex(ValueError, 'SHA256'):
            moveit_operation('inspect', self.path)
        for change in (lambda p: p['joint_names'].append('extra'),
                       lambda p: p['points'][0].update(velocities=[1.]*6),
                       lambda p: p['points'][0].update(accelerations=[1.]*6),
                       lambda p: p['points'][1].update(positions=[float('nan')]*6)):
            payload = copy.deepcopy(self.payload)
            change(payload)
            with self.assertRaises(ValueError):
                MoveItPoints(payload)
        self.path.write_text('{"joint_names": [], "joint_names": [], "points": []}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
            moveit_operation('inspect', self.path, expected_sha256=sha256(self.path))

    def test_full_source_retiming_still_refuses(self):
        with self.assertRaisesRegex(ValueError, 'Full-policy retiming'):
            moveit_operation('inspect', self.path, speed_factor=.5,
                             expected_sha256=self.digest, allow_retiming=False)

    def test_speedup_checks_between_points_not_just_knot_derivatives(self):
        # Both endpoint velocities are zero, but moving between them needs positive speed.
        result = self.invoke(factor=2., options={'joint_velocity_limits': [.001]*6,
                                               'joint_acceleration_limits': [.001]*6})
        self.assertEqual(result['status'], 'blocked')
        self.assertTrue(any(x.startswith('joint_velocity_exceeded.') for x in result['blockers']))
        self.assertEqual(self.backends, [])

    def test_generated_script_is_small_no_reference_data_and_imports_in_isolation(self):
        source = self.script.read_text(encoding='utf-8')
        self.assertLess(len(source), 45000)
        for forbidden in ('SITE_DEPLOYMENT', 'PLAIN_REFERENCE', 'tcp_poses', 'restored_portable',
                          'Ros1TrajectoryBackend', 'import planning'):
            self.assertNotIn(forbidden, source)
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.ImportFrom):
                self.assertFalse(node.level)
                self.assertNotIn((node.module or '').split('.')[0], ('control', 'planning', 'perception'))
        # Original built pair is bound to the unmodified fixture, not this test's changed q.
        payload = read_json(self.path)
        payload['points'][-1]['positions'] = [0.]*6
        write_json(self.path, payload)
        result = subprocess.run([sys.executable, '-I', '-B', str(self.script), 'inspect',
            '--trajectory', str(self.path)], cwd=self.root, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(json.loads(result.stdout)['status'], 'offline_checks_passed')

    def test_generated_runtime_execution_uses_same_mock_lifecycle(self):
        spec = importlib.util.spec_from_file_location('generated_moveit_replay', self.script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        result = self.invoke(operation=module.moveit_operation)
        self.assertEqual(result['status'], 'completed')

    def test_source_gate_not_bypassed_by_moveit_builder(self):
        with self.assertRaisesRegex(ValueError, 'source gate'):
            pack_standalone(self.source/'trajectory.json', None, self.root/'rejected',
                            format='plain', backend='moveit')
        report = read_json(self.source/'export_report.json')
        report['interpolation_audit']['passed'] = False
        write_json(self.source/'export_report.json', report)
        with self.assertRaisesRegex(ValueError, 'interpolation_audit_failed'):
            pack_standalone(self.source/'trajectory.json', None, self.root/'bad_audit',
                            format='plain', backend='moveit', allow_geometry_only=True)

    def test_clock_point_validation_and_derivative_scaling(self):
        payload = copy.deepcopy(self.payload)
        mid = copy.deepcopy(payload['points'][0])
        mid.update(positions=[.02]*6, velocities=[.03]*6, accelerations=[.04]*6, time_from_start=.4)
        payload['points'].insert(1, mid)
        curve = MoveItPoints(payload)
        self.assertIs(curve.retimed(1.), curve)
        for factor in (.5, 2.):
            scaled = curve.retimed(factor)
            np.testing.assert_array_equal(scaled.q, curve.q)
            np.testing.assert_array_equal(scaled.v, curve.v*factor)
            np.testing.assert_array_equal(scaled.a, curve.a*factor*factor)
            self.assertEqual(scaled.duration, curve.duration/factor)
        for factor in (0., -1., float('inf'), float('nan'), True, 1e20, 1e-20):
            with self.subTest(factor=factor), self.assertRaises((ValueError, FloatingPointError)):
                curve.retimed(factor)
        payload['points'][1]['time_from_start'] = 0.
        with self.assertRaises(ValueError):
            MoveItPoints(payload)

    def test_model_limit_units_and_lower_of_urdf_and_moveit(self):
        names = self.payload['joint_names']
        urdf = '<robot>'+''.join('<joint name="'+n+'"><limit velocity="2"/></joint>' for n in names)+'</robot>'
        settings = {n: dict(has_velocity_limits=True, max_velocity=1.,
                            has_acceleration_limits=True, max_acceleration=3.) for n in names}
        limits = moveit_limits(names, settings, urdf)
        self.assertEqual(limits, {'velocity': [1.]*6, 'acceleration': [3.]*6})
        self.assertEqual(moveit_limits(names, {}, urdf)['acceleration'], [None]*6)
        settings[names[0]]['max_velocity'] = float('nan')
        with self.assertRaises(ValueError):
            moveit_limits(names, settings, urdf)


class MoveItTransportTests(unittest.TestCase):
    def test_execute_trajectory_message_and_cancel_use_moveit_not_joint_action(self):
        names = ['j'+str(i) for i in range(6)]
        curve = MoveItPoints(dict(joint_names=names, points=[dict(positions=[q]*6,
            velocities=[0.]*6, accelerations=[0.]*6, time_from_start=t) for q, t in ((0., 0.), (.05, 1.))]))
        client = Mock()
        client.wait_for_server.return_value = True
        client.get_state.return_value = 1
        group = Mock()
        group.get_active_joints.return_value = names[::-1]
        subscriber = Mock()
        rospy = SimpleNamespace(core=SimpleNamespace(is_initialized=lambda: True),
            get_param=lambda key, default=None: False if key == '/use_sim_time' else default,
            Duration=lambda sec, nsec=None: sec if nsec is None else (sec, nsec),
            Subscriber=Mock(return_value=subscriber), Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 10.)))
        goal_factory = lambda: SimpleNamespace(trajectory=SimpleNamespace(
            joint_trajectory=SimpleNamespace(joint_names=[], points=[])))
        modules = {'actionlib': SimpleNamespace(SimpleActionClient=Mock(return_value=client)),
            'moveit_commander': SimpleNamespace(roscpp_initialize=Mock(), MoveGroupCommander=Mock(return_value=group)),
            'rospy': rospy, 'moveit_msgs.msg': SimpleNamespace(ExecuteTrajectoryAction=object, ExecuteTrajectoryGoal=goal_factory),
            'sensor_msgs.msg': SimpleNamespace(JointState=object),
            'trajectory_msgs.msg': SimpleNamespace(JointTrajectoryPoint=SimpleNamespace)}
        with patch.dict(sys.modules, modules):
            backend = MoveItBackend(dict(MOVEIT_OPTIONS), curve)
            self.assertEqual(client.send_goal.call_count, 0)
            backend.send(curve)
            goal = client.send_goal.call_args.args[0]
            self.assertEqual(goal.trajectory.joint_trajectory.joint_names, names)
            self.assertEqual(goal.trajectory.joint_trajectory.points[-1].positions, [.05]*6)
            self.assertEqual(goal.trajectory.joint_trajectory.points[-1].time_from_start, (1, 0))
            msg = SimpleNamespace(name=names[::-1], position=list(range(6)), velocity=[0.]*6,
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 10.)))
            backend._joints(msg)
            self.assertEqual(backend.snapshot()['q'], list(reversed(range(6))))
            self.assertEqual(backend.stop(), [])
            client.cancel_goal.assert_called_once()
            group.stop.assert_called_once()
            backend.close()
            subscriber.unregister.assert_called_once()

    def test_stop_attempts_both_cancel_and_group_stop_even_if_cancel_raises(self):
        backend = object.__new__(MoveItBackend)
        backend.client = SimpleNamespace(cancel_goal=Mock(side_effect=RuntimeError('cancel failed')))
        backend.group = SimpleNamespace(stop=Mock())
        self.assertEqual(backend.stop(), ['cancel failed'])
        backend.group.stop.assert_called_once()


if __name__ == '__main__':
    unittest.main()
