"""Independent deployment, policy separation and actual action lifecycle in Mock."""
import base64
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from control.hardware.standalone_pack import build_standalone_script, pack_standalone
from control.hardware.standalone_runtime import (portable_operation, restored_portable,
                                                 prepare_execution_curve, standalone_source_gate)
from control.hardware.trajectory_contract import (JointTrajectory, load_bundle, read_json,
                                                  sha256, write_json)
from control.hardware.ros1_trajectory import fill_goal
from control.hardware.ros_runner import run
from tests.test_ros1_hardware import Clock, FakeBackend, make_bundle, make_record, make_state


class ScaledReferenceBackend(FakeBackend):
    """Use controller trajectory time and the serialized goal, including its derivatives."""
    def __init__(self, curve, **kwargs):
        super().__init__(**kwargs)
        self.curve = curve

    def send(self, curve):
        if curve is not self.curve:
            raise AssertionError('Goal and monitor must share a curve')
        self.goal = SimpleNamespace(trajectory=SimpleNamespace(joint_names=[], points=[]))
        fill_goal(curve, self.goal, SimpleNamespace, lambda sec, nsec: (sec, nsec))
        record = copy.deepcopy(curve.record)
        for target, point in zip(record['points'], self.goal.trajectory.points):
            sec, nsec = point.time_from_start
            target.update(time_from_start=sec+nsec/1e9, positions=point.positions,
                          velocities=point.velocities, accelerations=point.accelerations)
        self.controller_curve = JointTrajectory(record)
        super().send(curve)

    def snapshot(self):
        if self.cancelled:
            return super().snapshot()
        if not self.sent:
            return make_state()
        self.step += 1
        progress = 0. if self.stalled else min(self.step*self.curve.duration/4, self.curve.duration)
        state = make_state(progress, progress == self.curve.duration)
        q, v, _, tcp, _ = self.controller_curve.sample(progress)
        state.update(q=q.tolist(), dq=v.tolist(), tcp=tcp.tolist(), desired_q=q.tolist())
        if self.fault:
            self.fault(state)
        return state


class StandaloneHardwareTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='ur7e_standalone_test_')
        self.root = Path(self.temporary.name)
        self.source = self.root/'source'
        self.source.mkdir()
        self.config = make_bundle(self.source, ready=False)
        self.delivery = self.root/'delivery'
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json',
                        self.delivery, allow_geometry_only=True)
        self.payload = self.delivery/'trajectory.json'

    def tearDown(self):
        self.temporary.cleanup()

    def invoke(self, backend, *, command='execute', operation=portable_operation):
        clock = Clock()
        return operation(command, self.payload, self.root/'run', backend_factory=lambda *_: backend,
                         now=clock.now, sleep=clock.sleep)

    def generated_module(self):
        spec = importlib.util.spec_from_file_location('isolated_ur_executor',
                    self.delivery/'ur7e_saved_trajectory_executor.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def moving_payload(self, *, full=False):
        config = make_bundle(self.source, ready=full)
        record = make_record(ready=full)
        middle = copy.deepcopy(record['points'][0])
        middle.update(time_from_start=.4, positions=[.04]*6, velocities=[.02]*6,
                      accelerations=[.01]*6)
        record['points'].insert(1, middle)
        record['points'][-1]['positions'] = [.08]*6
        record['phases'] = [{'name': 'trace', 'start_s': 0., 'end_s': .4},
                            {'name': 'retreat', 'start_s': .4, 'end_s': 1.}]
        write_json(self.source/'trajectory.json', record)
        report = read_json(self.source/'export_report.json')
        report['trajectory_sha256'] = sha256(self.source/'trajectory.json')
        write_json(self.source/'export_report.json', report)
        config['authorization'].update(trajectory_sha256=sha256(self.source/'trajectory.json'),
                                      export_report_sha256=sha256(self.source/'export_report.json'))
        config['phase_limits']['retreat'] = dict(config['phase_limits']['trace'])
        config['motion_limits'] = {'joint_velocity_rad_s': [20.]*6,
                                   'joint_acceleration_rad_s2': [20.]*6}
        write_json(self.source/'deployment.json', config)
        self.delivery = self.root/'moving'
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json',
                        self.delivery, allow_geometry_only=not full)
        self.payload = self.delivery/'trajectory.json'
        return JointTrajectory(record), read_json(self.payload)['deployment']

    def speed_invoke(self, factor, *, command='execute', fault=None, stalled=False,
                     stop_ok=True, operation=portable_operation):
        backends = []
        def factory(config, curve):
            backend = ScaledReferenceBackend(curve, fault=fault, stalled=stalled, stop_ok=stop_ok)
            backends.append(backend)
            return backend
        clock = Clock()
        result = operation(command, self.payload, self.root/('speed_'+str(factor)+'_'+command),
                           speed_factor=factor, backend_factory=factory,
                           now=clock.now, sleep=clock.sleep)
        return result, backends

    def test_geometry_replays_once_without_project_readiness(self):
        backend = FakeBackend()
        result = self.invoke(backend)
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(backend.sent, 1)
        self.assertFalse(result['source']['execution_ready'])
        self.assertIn('source.check_mode', result['source_warnings'])
        self.assertFalse(result['collision_dynamics_verified'])
        self.assertFalse(result['hardware_validated'])

    def test_project_default_still_refuses_same_source(self):
        def forbidden(*_):
            self.fail('Project gate must refuse before backend construction')
        result = run(self.source/'trajectory.json', self.source/'deployment.json',
                     self.root/'project_run', forbidden, execute=True)
        self.assertEqual(result['status'], 'blocked')
        self.assertIn('source.check_mode', result['blockers'])

    def test_generated_runtime_replays_geometry(self):
        module = self.generated_module()
        result = self.invoke(FakeBackend(), operation=module.portable_operation)
        self.assertEqual(result['status'], 'completed')

    def test_read_only_sends_no_commands(self):
        backend = FakeBackend()
        result = self.invoke(backend, command='check')
        self.assertEqual(result['status'], 'read_only_checked')
        self.assertEqual((backend.sent, backend.beats, backend.cancelled, backend.stops), (0, 0, 0, 0))

    def test_force_fault_cancels_stops_and_confirms(self):
        backend = FakeBackend(fault=lambda state: state.update(wrench=[2., 0, 0, 0, 0, 0]))
        result = self.invoke(backend)
        self.assertEqual(result['fault'], 'force_limit')
        self.assertEqual((backend.sent, backend.cancelled, backend.stops), (1, 1, 1))
        self.assertTrue(result['stop']['confirmed'])

    def test_start_mismatch_sends_nothing(self):
        class WrongStart(FakeBackend):
            def snapshot(self):
                state = super().snapshot()
                state['q'] = [1.]*6
                return state
        backend = WrongStart()
        result = self.invoke(backend)
        self.assertEqual(result['fault'], 'start_mismatch')
        self.assertEqual(backend.sent, 0)

    def test_stale_force_stops_motion(self):
        def stale(state):
            state['ages']['wrench'] = 10.
        backend = FakeBackend(fault=stale)
        result = self.invoke(backend)
        self.assertEqual(result['fault'], 'stale_wrench')
        self.assertEqual(backend.stops, 1)

    def test_stall_and_stop_failure_are_reported(self):
        backend = FakeBackend(stalled=True, stop_ok=False)
        result = self.invoke(backend)
        self.assertEqual(result['fault'], 'controller_progress_stalled')
        self.assertFalse(result['stop']['confirmed'])

    def test_interrupt_cancels_and_stops(self):
        def interrupt(state):
            raise KeyboardInterrupt()
        backend = FakeBackend(fault=interrupt)
        result = self.invoke(backend)
        self.assertEqual(result['fault'], 'KeyboardInterrupt')
        self.assertTrue(result['stop']['confirmed'])

    def test_expired_authorization_prevents_send(self):
        payload = read_json(self.payload)
        payload['deployment']['authorization']['expires_utc'] = '2020-01-01T00:00:00Z'
        write_json(self.payload, payload)
        backend = FakeBackend()
        result = self.invoke(backend)
        self.assertIn('authorization.expired', result['blockers'])
        self.assertEqual(backend.sent, 0)

    def test_full_pack_reuses_original_validation(self):
        make_bundle(self.source, ready=True)
        other = self.root/'full'
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json', other)
        result = portable_operation('inspect', other/'trajectory.json')
        self.assertEqual(result['status'], 'offline_checks_passed')
        self.assertEqual(result['project_source_restrictions_waived'], [])

    def test_unknown_site_fields_still_refuse_before_ros(self):
        payload = read_json(self.payload)
        payload['deployment']['identity']['robot_ip'] = None
        write_json(self.payload, payload)
        def forbidden(*_):
            self.fail('Missing deployment must refuse before ROS imports')
        result = portable_operation('execute', self.payload, self.root/'run', backend_factory=forbidden)
        self.assertEqual(result['status'], 'blocked')
        self.assertIn('identity.robot_ip', result['blockers'])
        self.assertNotIn('source.check_mode', result['blockers'])

    def test_default_pack_does_not_enable_exception(self):
        other = self.root/'strict'
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json', other)
        result = portable_operation('inspect', other/'trajectory.json')
        self.assertIn('source.check_mode', result['blockers'])

    def test_explicit_acknowledgment_required(self):
        payload = read_json(self.payload)
        payload['deployment']['execution_policy']['acknowledge_unverified_motion'] = False
        write_json(self.payload, payload)
        result = portable_operation('inspect', self.payload)
        self.assertIn('execution_policy.acknowledge_unverified_motion', result['blockers'])

    def test_interpolation_failure_is_not_waived(self):
        curve, report, _ = load_bundle(self.source/'trajectory.json')
        report['interpolation_audit']['passed'] = False
        config = read_json(self.payload)['deployment']
        self.assertIn('interpolation_audit_failed', standalone_source_gate(curve, report, config))

    def test_bytes_preserved_and_temporary_files_removed(self):
        original = {p.name: p.read_bytes() for p in self.source.glob('*.json') if p.name != 'deployment.json'}
        with restored_portable(self.payload) as (root, curve, report, config, digest, check):
            for name, raw in original.items():
                self.assertEqual((root/name).read_bytes(), raw)
            check()
            self.assertEqual(curve.record['source']['check_mode'], 'geometry_only')
        self.assertFalse(root.exists())
        for name, raw in original.items():
            self.assertEqual((self.source/name).read_bytes(), raw)

    def test_invalid_filename_and_base64_rejected(self):
        original = read_json(self.payload)
        for name, value in (('../escape.json', 'e30='), ('C:\\escape.json', 'e30='),
                            ('deployment.json', 'e30='), ('unexpected.json', 'e30='),
                            ('trajectory.json', '*not base64*')):
            with self.subTest(name=name):
                payload = copy.deepcopy(original)
                payload['files'][name] = value
                write_json(self.payload, payload)
                with self.assertRaises(ValueError):
                    portable_operation('inspect', self.payload)

    def test_missing_file_fingerprint_and_old_json_rejected(self):
        original = read_json(self.payload)
        payload = copy.deepcopy(original)
        del payload['files']['planning_report.json']
        write_json(self.payload, payload)
        with self.assertRaises(ValueError): portable_operation('inspect', self.payload)
        payload = copy.deepcopy(original)
        payload['files']['trajectory.json'] = base64.b64encode(b'{}').decode('ascii')
        write_json(self.payload, payload)
        with self.assertRaises(ValueError): portable_operation('inspect', self.payload)
        with self.assertRaises(ValueError): portable_operation('inspect', self.source/'trajectory.json')

    def test_duplicate_json_keys_rejected(self):
        text = self.payload.read_text(encoding='utf-8')
        text = text.replace('"schema": "ur7e_standalone_v1",',
                            '"schema": "ur7e_standalone_v1", "schema": "ur7e_standalone_v1",', 1)
        self.payload.write_text(text, encoding='utf-8')
        with self.assertRaises(ValueError): portable_operation('inspect', self.payload)

    def test_changed_outer_input_cannot_send(self):
        payload = self.payload
        class ChangeDuringPreflight(FakeBackend):
            def snapshot(self):
                state = super().snapshot()
                with payload.open('ab') as stream:
                    stream.write(b' ')
                return state
        backend = ChangeDuringPreflight()
        result = self.invoke(backend)
        self.assertEqual(result['fault'], 'portable_input_changed_before_send')
        self.assertEqual(backend.sent, 0)

    def test_generated_source_is_deterministic_and_cannot_overwrite_delivery(self):
        self.assertEqual(build_standalone_script(), build_standalone_script())
        with self.assertRaises(FileExistsError):
            pack_standalone(self.source/'trajectory.json', self.source/'deployment.json', self.delivery)

    def test_isolated_cli_without_ros_or_project(self):
        environment = dict(os.environ)
        environment.pop('PYTHONPATH', None)
        script = self.delivery/'ur7e_saved_trajectory_executor.py'
        completed = subprocess.run([sys.executable, '-I', str(script), 'inspect',
                                    '--trajectory', str(self.payload)], cwd=self.delivery,
                                   env=environment, capture_output=True, text=True, timeout=30)
        self.assertEqual(completed.returncode, 0, completed.stdout+completed.stderr)
        self.assertEqual(json.loads(completed.stdout)['status'], 'offline_checks_passed')
        probe = ('import runpy,sys; runpy.run_path(sys.argv[1],run_name="isolated"); '
                 'assert not any(n.split(".")[0] in '
                 '{"control","planning","perception","pipeline","rospy","mujoco","open3d","toppra"} '
                 'for n in sys.modules)')
        checked = subprocess.run([sys.executable, '-I', '-c', probe, str(script)], cwd=self.delivery,
                                 env=environment, capture_output=True, text=True, timeout=30)
        self.assertEqual(checked.returncode, 0, checked.stdout+checked.stderr)

    def test_speed_scaling_path_derivatives_phases_and_immutable_source(self):
        original, config = self.moving_payload()
        before = copy.deepcopy(original.record)
        for factor in (1., 2., .5):
            with self.subTest(factor=factor):
                curve, blockers, details = prepare_execution_curve(original, config, factor)
                self.assertEqual(blockers, [])
                self.assertAlmostEqual(curve.duration, original.duration/factor)
                np.testing.assert_array_equal(curve.q, original.q)
                np.testing.assert_array_equal(curve.tcp, original.tcp)
                np.testing.assert_allclose(curve.v, original.v*factor)
                np.testing.assert_allclose(curve.a, original.a*factor**2)
                np.testing.assert_array_equal(curve.v[[0, -1]], 0.)
                np.testing.assert_array_equal(curve.a[[0, -1]], 0.)
                for old, new in zip(original.record['phases'], curve.record['phases']):
                    self.assertEqual(old['name'], new['name'])
                    self.assertAlmostEqual(new['end_s'], old['end_s']/factor)
                for t in np.linspace(0, curve.duration, 41):
                    expected, actual = original.sample(t*factor), curve.sample(t)
                    for i, multiplier in enumerate((1., factor, factor**2)):
                        np.testing.assert_allclose(actual[i], expected[i]*multiplier, atol=1e-10)
                self.assertEqual(details['original_dynamics_audit_applies'], factor == 1.)
                if factor == 1.:
                    self.assertIs(curve, original)
        self.assertEqual(original.record, before)

    def test_scaled_goal_and_monitor_complete_at_actual_scaled_progress(self):
        self.moving_payload()
        for factor in (1., 2., .5):
            with self.subTest(factor=factor):
                result, backends = self.speed_invoke(factor)
                self.assertEqual(result['status'], 'completed', result)
                self.assertEqual(result['controller_progress_s'], 1./factor)
                self.assertEqual(result['commanded_duration_s'], 1./factor)
                self.assertEqual(backends[0].sent, 1)
                self.assertEqual(backends[0].cancelled, 0)
                self.assertEqual(result['speed_factor'], factor)
                self.assertFalse(result['collision_dynamics_verified'])

    def test_message_nanosecond_rounding_uses_same_monitor_clock(self):
        original, config = self.moving_payload()
        original.record['points'][1]['time_from_start'] = .400000001
        original = JointTrajectory(original.record)
        curve, _, details = prepare_execution_curve(original, config, 3.)
        backend = ScaledReferenceBackend(curve)
        backend.send(curve)
        np.testing.assert_allclose(backend.controller_curve.times, curve.times, atol=1e-16, rtol=0)
        for t in np.linspace(0., curve.duration, 51):
            for a, b in zip(curve.sample(t)[:3], backend.controller_curve.sample(t)[:3]):
                np.testing.assert_allclose(a, b, atol=1e-12)
            np.testing.assert_allclose(curve.sample(t)[0], original.sample(t*3)[0], atol=1e-8)
        self.assertLessEqual(details['max_clock_rounding_s'], .500001e-9)

    def test_speed_invalid_and_clock_collapse_rejected(self):
        curve, config = self.moving_payload()
        for factor in (0., -1., float('nan'), float('inf'), -float('inf'), True, '2', 1e12, 1e-12):
            with self.subTest(factor=factor), self.assertRaises(ValueError):
                prepare_execution_curve(curve, config, factor)

    def test_acceleration_requires_actual_six_joint_limits_before_backend(self):
        self.moving_payload()
        original = read_json(self.payload)
        for bad in (None, {}, {'joint_velocity_rad_s': [1.]*6},
                    {'joint_velocity_rad_s': [1.]*5, 'joint_acceleration_rad_s2': [1.]*6},
                    {'joint_velocity_rad_s': [True]*6, 'joint_acceleration_rad_s2': [1.]*6},
                    {'joint_velocity_rad_s': [1.]*6, 'joint_acceleration_rad_s2': [0.]*6}):
            with self.subTest(limits=bad):
                payload = copy.deepcopy(original)
                payload['deployment']['motion_limits'] = bad
                write_json(self.payload, payload)
                result = portable_operation('inspect', self.payload, speed_factor=2.)
                self.assertEqual(result['status'], 'blocked')
                self.assertTrue(any(b.startswith('motion_limits.') for b in result['blockers']))
        result, backends = self.speed_invoke(2.)
        self.assertEqual(backends, [])
        self.assertEqual(result['status'], 'blocked')

    def test_conservative_limits_detect_interior_velocity_and_acceleration(self):
        curve, config = self.moving_payload()
        # Stopped endpoints cannot reveal the interior quintic extrema.
        config['motion_limits'] = {'joint_velocity_rad_s': [.001]+[20.]*5,
                                   'joint_acceleration_rad_s2': [20., .001]+[20.]*4}
        _, blockers, details = prepare_execution_curve(curve, config, 2.)
        self.assertIn('motion_limits.joint_velocity_rad_s.exceeded.'+curve.names[0], blockers)
        self.assertIn('motion_limits.joint_acceleration_rad_s2.exceeded.'+curve.names[1], blockers)
        self.assertFalse(details['speed_limit_audit']['passed'])
        self.assertGreater(details['speed_limit_audit']['joint_velocity_rad_s_bound'][0], .04)

    def test_derivative_bounds_scale_and_cover_continuous_samples(self):
        original, config = self.moving_payload()
        curve, _, details = prepare_execution_curve(original, config, 2.)
        _, _, faster = prepare_execution_curve(original, config, 4.)
        for key, power in (('joint_velocity_rad_s', 1), ('joint_acceleration_rad_s2', 2)):
            bound = np.asarray(details['speed_limit_audit'][key+'_bound'])
            np.testing.assert_allclose(faster['speed_limit_audit'][key+'_bound'], bound*2**power)
            samples = [abs(curve.sample(t)[power]) for t in np.linspace(0, curve.duration, 201)]
            self.assertTrue(np.all(np.max(samples, axis=0) <= bound+1e-12))

    def test_full_policy_cannot_inherit_audit_after_retiming(self):
        self.moving_payload(full=True)
        for factor in (2., .5):
            with self.subTest(factor=factor):
                result, backends = self.speed_invoke(factor)
                self.assertEqual(backends, [])
                self.assertIn('speed_factor.full_policy_requires_reaudit_and_export', result['blockers'])
                self.assertFalse(result['original_dynamics_audit_applies'])
                self.assertFalse(result['collision_dynamics_verified'])

    def test_original_and_slower_do_not_require_new_limits(self):
        self.moving_payload()
        payload = read_json(self.payload)
        del payload['deployment']['motion_limits']
        write_json(self.payload, payload)
        for factor in (1., .5):
            result, _ = self.speed_invoke(factor)
            self.assertEqual(result['status'], 'completed', result)

    def test_speed_check_is_read_only_and_rejects_missing_limits(self):
        self.moving_payload()
        result, backends = self.speed_invoke(2., command='check')
        self.assertEqual(result['status'], 'read_only_checked')
        b = backends[0]
        self.assertEqual((b.sent, b.beats, b.cancelled, b.stops), (0, 0, 0, 0))
        payload = read_json(self.payload)
        payload['deployment']['motion_limits'] = {}
        write_json(self.payload, payload)
        result, backends = self.speed_invoke(3., command='check')
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(backends, [])

    def test_speed_preserves_original_files_hashes_and_authorization(self):
        self.moving_payload()
        before = self.payload.read_bytes()
        source = {p.name: p.read_bytes() for p in self.source.glob('*.json')}
        first, _ = self.speed_invoke(1.)
        second, _ = self.speed_invoke(2.)
        self.assertEqual(first['trajectory_sha256'], second['trajectory_sha256'])
        self.assertNotEqual(first['execution_curve_sha256'], second['execution_curve_sha256'])
        self.assertEqual(self.payload.read_bytes(), before)
        self.assertEqual(source, {p.name: p.read_bytes() for p in self.source.glob('*.json')})

    def test_scaled_force_fault_uses_scaled_phase_and_still_stops(self):
        self.moving_payload()
        payload = read_json(self.payload)
        payload['deployment']['phase_limits']['retreat']['force_n'] = .1
        write_json(self.payload, payload)
        result, backends = self.speed_invoke(2., fault=lambda s: s.update(wrench=[.2, 0, 0, 0, 0, 0]))
        self.assertEqual(result['fault'], 'force_limit')
        self.assertEqual(backends[0].step, 2)  # progress .25 is retreat after scaling
        self.assertTrue(result['stop']['confirmed'])
        self.assertEqual((backends[0].cancelled, backends[0].stops), (1, 1))

    def test_scaled_stall_and_stop_timeout_use_unchanged_wall_clock(self):
        self.moving_payload()
        result, _ = self.speed_invoke(2., stalled=True, stop_ok=False)
        self.assertEqual(result['fault'], 'controller_progress_stalled')
        self.assertFalse(result['stop']['confirmed'])
        self.assertGreaterEqual(result['stop']['elapsed_s'], .5)

    def test_scaled_monitor_rejects_original_unscaled_reference(self):
        original, _ = self.moving_payload()
        def wrong_reference(state):
            state['desired_q'] = original.sample(state['progress_s'])[0].tolist()
        result, backends = self.speed_invoke(2., fault=wrong_reference)
        self.assertEqual(result['fault'], 'controller_reference_mismatch')
        self.assertTrue(result['stop']['confirmed'])
        self.assertEqual(backends[0].stops, 1)

    def test_scaled_wall_timeout_is_not_rescaled(self):
        self.moving_payload()
        payload = read_json(self.payload)
        payload['deployment']['limits']['max_run_wall_s'] = .02
        write_json(self.payload, payload)
        result, backends = self.speed_invoke(2.)
        self.assertEqual(result['fault'], 'execution_wall_timeout')
        self.assertTrue(result['stop']['confirmed'])
        self.assertEqual(backends[0].stops, 1)

    def test_generated_speed_runtime_executes_and_cli_accepts_option(self):
        self.moving_payload()
        module = self.generated_module()
        result, _ = self.speed_invoke(2., operation=module.portable_operation)
        self.assertEqual(result['status'], 'completed')
        completed = subprocess.run([sys.executable, '-I', str(self.delivery/'ur7e_saved_trajectory_executor.py'),
            'inspect', '--trajectory', str(self.payload), '--speed-factor', '2'],
            cwd=self.delivery, capture_output=True, text=True, timeout=30)
        self.assertEqual(completed.returncode, 0, completed.stdout+completed.stderr)
        self.assertEqual(json.loads(completed.stdout)['commanded_duration_s'], .5)


if __name__ == '__main__':
    unittest.main()
