"""No ROS imports or device connections: contract, interpolation and fault lifecycle."""
import copy
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import subprocess
import sys
import threading

import numpy as np

from control.hardware.trajectory_contract import (JointTrajectory, SCHEMA, UNITS, UR_JOINTS,
    UR_MAPPING, execution_blockers, joint_order, load_bundle, sha256, write_json)
from control.hardware.ros_export import (sample_grid, source_gates, interpolation_audit)
from control.hardware.ros_runner import (IDENTITY, LIMITS, ROS_FIELDS, STREAMS, deployment_errors,
                                         run, state_fault)
from control.hardware.ros1_trajectory import (Ros1TrajectoryBackend, fill_goal, matrix_quaternion,
                                             quaternion_matrix)


def make_record(ready=True):
    gates = {'check_mode': 'full' if ready else 'geometry_only', 'execution_ready': ready,
             'simulation_only': not ready, 'robot_execution_allowed': ready, 'camera_verified': ready}
    return {'schema': SCHEMA, 'units': UNITS, 'joint_names': UR_JOINTS, 'source': gates,
            'phases': [{'name': 'trace', 'start_s': 0., 'end_s': 1.}],
            'points': [{'time_from_start': t, 'positions': [0.]*6, 'velocities': [0.]*6,
                        'accelerations': [0.]*6, 'tcp_pose': [0., 0., 0., 0., 0., 0., 1.]}
                       for t in (0., 1.)]}


def make_bundle(root, ready=True):
    record = make_record(ready)
    write_json(root/'trajectory.json', record)
    write_json(root/'planning_report.json', {'check_mode': record['source']['check_mode']})
    write_json(root/'source_manifest.json', {'gates': record['source'],
        'snapshots': {'planning_report.json': sha256(root/'planning_report.json')}})
    report = {'trajectory_sha256': sha256(root/'trajectory.json'),
              'manifest_sha256': sha256(root/'source_manifest.json'), 'source': record['source'],
              'load_verified_passed': ready, 'interpolation_audit': {'passed': True},
              'ros_motion_audit': {'passed': ready, 'collision_checked': ready}}
    write_json(root/'export_report.json', report)
    config = {'robot_model': 'ur7e', 'joint_names': UR_JOINTS, 'controller_interpolation': 'quintic',
              'command_owner': 'exclusive_follow_joint_trajectory',
              'planning_world_from_base_m': np.eye(4).tolist(),
              'identity': {key: 'test evidence only' for key in IDENTITY},
              'ros': {key: '/mock/'+key for key in ROS_FIELDS},
              'limits': {key: .1 for key in LIMITS},
              'phase_limits': {'trace': {'force_n': 1., 'torque_nm': .1}},
              'authorization': {'trajectory_sha256': sha256(root/'trajectory.json'),
                                'manifest_sha256': sha256(root/'source_manifest.json'),
                                'export_report_sha256': sha256(root/'export_report.json'),
                                'approved_by': 'unit test', 'expires_utc':
                                (datetime.now(timezone.utc)+timedelta(hours=1)).isoformat()}}
    config['limits'].update(max_state_age_s=.5, max_feedback_age_s=.5, max_stall_s=.5,
                            max_run_wall_s=5., stop_timeout_s=.5, settle_s=.1, poll_s=.05)
    write_json(root/'deployment.json', config)
    return config


def make_state(progress=0., succeeded=False):
    return {'q': [0.]*6, 'dq': [0.]*6, 'tcp': [0., 0., 0., 0., 0., 0., 1.],
            'wrench': [0.]*6, 'ages': {key: 0. for key in STREAMS+('feedback',)},
            'controller_running': True, 'conflicting_controllers': [], 'robot_mode': 7,
            'safety_mode': 1, 'program_running': True, 'watchdog_ready': True,
            'speed_scaling': .5, 'progress_s': progress, 'desired_q': [0.]*6,
            'action_state': 'succeeded' if succeeded else 'active', 'result_code': 0}


class Clock:
    def __init__(self): self.value = 0.
    def now(self): return self.value
    def sleep(self, duration): self.value += duration


class FakeBackend:
    def __init__(self, fault=None, stop_ok=True, stalled=False, progress_step=.25):
        self.sent = self.cancelled = self.stops = self.beats = 0
        self.closed = False
        self.step = 0
        self.fault, self.stop_ok, self.stalled = fault, stop_ok, stalled
        self.progress_step = progress_step

    def snapshot(self):
        if not self.sent:
            return make_state()
        if self.cancelled:
            state = make_state()
            if not self.stop_ok: state['dq'] = [1.]*6
            return state
        self.step += 1
        progress = 0. if self.stalled else min(self.step*self.progress_step, 1.)
        state = make_state(progress, progress == 1.)
        if self.fault: self.fault(state)
        return state

    def send(self, curve): self.sent += 1
    def cancel(self): self.cancelled += 1
    def stop(self): self.stops += 1
    def heartbeat(self): self.beats += 1
    def close(self): self.closed = True


class RosHardwareTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='ur7e_bridge_test_')
        self.root = Path(self.temporary.name)
        self.config = make_bundle(self.root)
        self.curve, self.report, _ = load_bundle(self.root/'trajectory.json')

    def tearDown(self): self.temporary.cleanup()

    def invoke(self, backend, *, execute=True, directory='run'):
        clock = Clock()
        return run(self.root/'trajectory.json', self.root/'deployment.json', self.root/directory,
                   lambda *_: backend, execute=execute, now=clock.now, sleep=clock.sleep)

    def test_mapping_is_explicit_bijection(self):
        names = list(UR_MAPPING)
        self.assertEqual(joint_order(names, UR_MAPPING, UR_JOINTS[::-1]), list(range(5, -1, -1)))
        for mapping in ({}, {name: 'same' for name in names}, {**UR_MAPPING, 'extra': 'x'}):
            with self.assertRaises(ValueError): joint_order(names, mapping, UR_JOINTS)

    def test_no_wrapping_and_ros_message_derivatives(self):
        record = make_record()
        for point in record['points']: point['positions'][3] = -4.774522701755688
        curve = JointTrajectory(record)
        goal = SimpleNamespace(trajectory=SimpleNamespace(joint_names=[], points=[]))
        fill_goal(curve, goal, SimpleNamespace, lambda sec, nsec: (sec, nsec))
        self.assertEqual(goal.trajectory.points[0].positions[3], record['points'][0]['positions'][3])
        self.assertEqual(goal.trajectory.points[-1].time_from_start, (1, 0))
        self.assertEqual(len(goal.trajectory.points[-1].accelerations), 6)

    def test_quintic_agrees_with_source(self):
        from planning.infrastructure.trajectory import Trajectory
        record = make_record()
        record['points'][1]['positions'] = [1.]*6
        curve = JointTrajectory(record)
        source = Trajectory([0., 1.], [[0.]*6, [1.]*6], ['trace'], [1], UR_JOINTS)
        for t in np.linspace(0, 1, 31):
            state = source.sample(t)
            for actual, key in zip(curve.sample(t)[:3], ('q', 'dq', 'ddq')):
                np.testing.assert_allclose(actual, state[key], atol=1e-12)

    def test_uniform_slowdown_derivatives(self):
        from planning.infrastructure.trajectory import Trajectory
        p = Trajectory([0., 1.], [[0.]*6, [1.]*6], ['trace'], [1], UR_JOINTS)
        times = sample_grid(p, .01, 2.)
        self.assertEqual(times[-1], 2.)
        r = make_record()
        r['phases'][0]['end_s'] = 2.
        r['points'] = []
        for t in times:
            s = p.sample(t/2)
            r['points'].append({'time_from_start': t, 'positions': s['q'].tolist(),
                'velocities': (s['dq']/2).tolist(), 'accelerations': (s['ddq']/4).tolist(),
                'tcp_pose': [0., 0., 0., 0., 0., 0., 1.]})
        curve = JointTrajectory(r)
        s = p.sample(.321)
        for x, expected in zip(curve.sample(.642)[:3], (s['q'], s['dq']/2, s['ddq']/4)):
            np.testing.assert_allclose(x, expected, atol=1e-9)

    def test_interior_bulge_is_detected(self):
        r = make_record()
        r['points'].insert(1, copy.deepcopy(r['points'][0]))
        r['points'][1]['time_from_start'] = .5
        r['points'][1]['velocities'] = [1.]*6
        curve = JointTrajectory(r)
        class Robot:
            lower, upper = np.full(6, -10.), np.full(6, 10.)
            def fk(self, q, full=False):
                pose = np.eye(4); pose[0, 3] = q[0]; return pose
        source = SimpleNamespace(duration=1., sample=lambda t: {'q': np.zeros(6)})
        audit = interpolation_audit(curve, source, Robot(), list(range(6)), 1.,
            {'joint_speed_rad_s': 100., 'joint_accel_rad_s2': 100.,
             'interpolation_position_mm': .1, 'interpolation_angle_deg': 1.})
        self.assertFalse(audit['passed'])
        self.assertGreater(audit['source_position_error_mm'], .1)

    def test_reject_nonfinite_and_nonmonotonic(self):
        for key, value in (('time_from_start', 0.), ('positions', [float('nan')]*6),
                           ('accelerations', [1.]*6)):
            r = make_record(); r['points'][1][key] = value
            with self.assertRaises(ValueError): JointTrajectory(r)

    def test_fingerprint_tampering_rejected(self):
        for name in ('trajectory.json', 'planning_report.json', 'source_manifest.json'):
            with self.subTest(name=name):
                make_bundle(self.root)
                with (self.root/name).open('a') as f: f.write(' ')
                with self.assertRaises(ValueError): load_bundle(self.root/'trajectory.json')

    def test_each_source_gate_is_mandatory(self):
        for key in ('check_mode', 'execution_ready', 'simulation_only', 'robot_execution_allowed', 'camera_verified'):
            r = make_record(); r['source'][key] = None
            self.assertIn('source.'+key, execution_blockers(JointTrajectory(r), self.report))

    def test_original_session_restrictions_preserved(self):
        source = source_gates({'check_mode': 'geometry_only', 'simulation_only': True},
            {'perception_session': {'robot_execution_allowed': False,
                                   'camera_frame_status': 'optical_frame_hypothesis_live_tf_unverified'}})
        self.assertFalse(source['camera_verified'])
        self.assertFalse(source['robot_execution_allowed'])

    def test_geometry_only_never_constructs_backend(self):
        make_bundle(self.root, ready=False)
        def forbidden(*args): self.fail('Backend must not be constructed')
        r = run(self.root/'trajectory.json', self.root/'deployment.json', self.root/'run', forbidden, execute=True)
        self.assertEqual(r['status'], 'blocked'); self.assertFalse(r['motion_sent'])

    def test_missing_deployment_never_constructs_backend(self):
        write_json(self.root/'deployment.json', {})
        backend = FakeBackend(); r = self.invoke(backend)
        self.assertEqual(r['status'], 'blocked'); self.assertEqual(backend.sent, 0)

    def test_blank_template_reports_all_missing_fields(self):
        from control.hardware.trajectory_contract import read_json
        template = read_json(Path(__file__).resolve().parents[1]/'control/hardware/ur7e_deployment.template.json')
        errors = deployment_errors(template, self.root/'trajectory.json', self.curve, self.report)
        self.assertIn('authorization.expires_utc', errors)
        self.assertIn('identity.launch_command', errors)

    def test_stale_authorization(self):
        self.config['authorization']['expires_utc'] = '2020-01-01T00:00:00+00:00'
        errors = deployment_errors(self.config, self.root/'trajectory.json', self.curve, self.report)
        self.assertIn('authorization.expired', errors)

    def test_read_only_never_commands_or_heartbeats(self):
        b = FakeBackend(); r = self.invoke(b, execute=False)
        self.assertEqual(r['status'], 'read_only_checked')
        self.assertEqual((b.sent, b.stops, b.cancelled, b.beats), (0, 0, 0, 0))

    def test_success_requires_stopped_final_feedback(self):
        b = FakeBackend(); r = self.invoke(b)
        self.assertEqual(r['status'], 'completed'); self.assertEqual(b.sent, 1)
        self.assertEqual(b.stops, 0); self.assertTrue(b.closed)

    def test_scaled_execution_uses_progress_not_original_wall_duration(self):
        b = FakeBackend(progress_step=.025)
        r = self.invoke(b)
        self.assertEqual(r['status'], 'completed')
        self.assertGreater(r['elapsed_s'], self.curve.duration)
        self.assertEqual(b.sent, 1)

    def test_runtime_import_does_not_require_planning_or_ros(self):
        program = (
            "import sys\n"
            "from control.hardware.__main__ import main\n"
            "assert main(['inspect','--trajectory',sys.argv[1]]) == 2\n"
            "assert not any(n in sys.modules for n in ['mujoco','toppra','open3d','rospy'])\n")
        completed = subprocess.run([sys.executable, '-c', program, str(self.root/'trajectory.json')],
                                   cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)

    def test_faults_cancel_stop_without_retreat_or_restart(self):
        def stale(s): s['ages']['wrench'] = 5.
        def force(s): s['wrench'][0] = 2.
        def torque(s): s['wrench'][5] = 2.
        def invalid(s): s['q'][0] = float('nan')
        def safety(s): s['safety_mode'] = 3
        def rejected(s): s['action_state'] = 'rejected'
        def watchdog(s): s['watchdog_ready'] = False
        def tracking(s): s['q'][0] = 1.
        def desired(s): s['desired_q'][0] = 1.
        def tcp(s): s['tcp'][0] = 1.
        def final_moving(s):
            if s['action_state'] == 'succeeded': s['dq'][0] = 1.
        for i, fault in enumerate((stale, force, torque, invalid, safety, rejected, watchdog, tracking,
                                   desired, tcp, final_moving)):
            with self.subTest(fault=fault.__name__):
                b = FakeBackend(fault=fault); r = self.invoke(b, directory='run'+str(i))
                self.assertEqual(r['status'], 'fault_latched')
                self.assertEqual((b.sent, b.cancelled, b.stops), (1, 1, 1))
                self.assertTrue(r['stop']['confirmed'])

    def test_stall_and_stop_not_confirmed(self):
        b = FakeBackend(stalled=True, stop_ok=False); r = self.invoke(b)
        self.assertEqual(r['fault'], 'controller_progress_stalled')
        self.assertFalse(r['stop']['confirmed'])

    def test_every_stream_requires_fresh_data(self):
        for key in STREAMS:
            s = make_state(); s['ages'].pop(key)
            self.assertEqual(state_fault(s, self.curve, self.config), 'stale_'+key)

    def test_start_mismatch_does_not_move_to_start(self):
        b = FakeBackend()
        b.snapshot = lambda: dict(make_state(), q=[1.]*6)
        r = self.invoke(b)
        self.assertEqual(r['fault'], 'start_mismatch'); self.assertEqual(b.sent, 0)

    def test_send_exception_still_stops_because_acceptance_is_unknown(self):
        b = FakeBackend()
        def partial_send(curve):
            b.sent += 1
            raise RuntimeError('connection failed after submit')
        b.send = partial_send
        r = self.invoke(b)
        self.assertEqual(r['status'], 'fault_latched')
        self.assertIsNone(r['motion_sent'])
        self.assertTrue(r['motion_send_attempted'])
        self.assertEqual((b.cancelled, b.stops), (1, 1))

    def test_keyboard_interrupt_stops_and_closes(self):
        def interrupted(state): raise KeyboardInterrupt()
        b = FakeBackend(fault=interrupted)
        r = self.invoke(b)
        self.assertEqual(r['fault'], 'KeyboardInterrupt')
        self.assertEqual((b.cancelled, b.stops), (1, 1))
        self.assertTrue(b.closed)

    def test_preflight_file_change_prevents_goal(self):
        b = FakeBackend()
        def mutate():
            with (self.root/'trajectory.json').open('a') as stream: stream.write(' ')
        b.heartbeat = mutate
        r = self.invoke(b)
        self.assertEqual(r['fault'], 'bundle_changed_during_preflight')
        self.assertEqual(b.sent, 0)

    def test_tcp_transform_quaternion_roundtrip(self):
        for q in ([0., 0., 0., 1.], [0., 0., 1., 0.], np.array([1., 2., 3., 4.])/np.sqrt(30)):
            r = quaternion_matrix(q)
            np.testing.assert_allclose(quaternion_matrix(matrix_quaternion(r)), r, atol=1e-12)

    def test_controller_progress_uses_sample_clock_not_segment_boundary(self):
        b = Ros1TrajectoryBackend.__new__(Ros1TrajectoryBackend)
        b.lock = threading.RLock()
        b.values, b.received, b.stamp, b.curve, b.error = {}, {}, {}, self.curve, None
        stamp = lambda value: SimpleNamespace(to_sec=lambda: value)
        msg = SimpleNamespace(joint_names=self.curve.names,
            actual=SimpleNamespace(time_from_start=stamp(.321)),
            desired=SimpleNamespace(time_from_start=stamp(0.), positions=[0.]*6),
            header=SimpleNamespace(stamp=stamp(100.)))
        b._controller_state(msg)
        self.assertEqual(b.values['feedback'][0], .321)
        self.assertIsNone(b.error)


if __name__ == '__main__': unittest.main()
