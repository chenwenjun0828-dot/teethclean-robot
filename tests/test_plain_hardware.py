"""Plain delivery contracts and protected execution, using only isolated Mock data."""
import ast
import copy
from datetime import datetime, timedelta, timezone
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

from control.hardware.standalone_pack import pack_standalone
from control.hardware.standalone_plain import PLAIN_CONTRACT, POINT_FIELDS, canonical_sha256
from control.hardware.standalone_runtime import portable_operation
from control.hardware.trajectory_contract import read_json, sha256, write_json
from tests.test_keyboard_hardware import ScriptedKeyboard
from tests.test_ros1_hardware import Clock, make_bundle
from tests.test_standalone_hardware import ScaledReferenceBackend


class PlainHardwareTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='ur7e_plain_test_')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.source = self.root/'source'
        self.source.mkdir()
        config = make_bundle(self.source, ready=False)
        record = read_json(self.source/'trajectory.json')
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
            export_report_sha256=sha256(self.source/'export_report.json'), approved_by=None)
        config['phase_limits']['retreat'] = dict(config['phase_limits']['trace'])
        config['motion_limits'] = {'joint_velocity_rad_s': [20.]*6,
                                   'joint_acceleration_rad_s2': [20.]*6}
        write_json(self.source/'deployment.json', config)
        self.original_config = copy.deepcopy(config)
        self.record = record
        self.delivery = self.root/'delivery'
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json', self.delivery,
                        allow_geometry_only=True, format='plain')
        self.script = self.delivery/'ur7e_saved_trajectory_executor.py'
        self.payload = self.delivery/'trajectory.json'
        self.module = self.load_module()
        self.module.SITE_DEPLOYMENT['authorization']['approved_by'] = 'mock_plain_reviewer'

    def load_module(self):
        spec = importlib.util.spec_from_file_location('isolated_plain_executor', self.script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def invoke(self, *, command='execute', factor=1., operator=None, fault=None, sleep_hook=None):
        clock = Clock()
        self.backends = []
        def factory(config, curve):
            backend = ScaledReferenceBackend(curve, fault=fault)
            self.backends.append(backend)
            return backend
        def sleep(dt):
            clock.sleep(dt)
            if sleep_hook:
                sleep_hook()
        return self.module.portable_operation(command, self.payload, self.root/'run',
            speed_factor=factor, operator=operator, backend_factory=factory,
            now=clock.now, sleep=sleep)

    def rebind_mock_input(self, payload=None):
        if payload is not None:
            write_json(self.payload, payload)
        ref = self.module.PLAIN_REFERENCE
        ref['plain_trajectory_sha256'] = sha256(self.payload)
        self.module.PLAIN_REFERENCE_SHA256 = canonical_sha256(ref)
        self.module.SITE_DEPLOYMENT['authorization'].update(
            plain_trajectory_sha256=sha256(self.payload),
            reference_sha256=self.module.PLAIN_REFERENCE_SHA256)

    def test_exact_legacy_fields_and_source_numbers(self):
        payload = read_json(self.payload)
        self.assertEqual(set(payload), {'joint_names', 'points'})
        self.assertEqual(payload['joint_names'], self.record['joint_names'])
        self.assertEqual(payload['points'], [
            {key: p[key] for key in POINT_FIELDS} for p in self.record['points']])
        for point in payload['points']:
            self.assertEqual(set(point), set(POINT_FIELDS))
        curve, report, config = self.module.PlainTrajectoryInput(self.payload,
            self.module.SITE_DEPLOYMENT, self.module.PLAIN_REFERENCE,
            self.module.PLAIN_REFERENCE_SHA256).load()
        self.assertEqual(curve.record, self.record)
        self.assertFalse(curve.record['source']['execution_ready'])
        self.assertEqual(config['authorization']['trajectory_sha256'],
                         self.original_config['authorization']['trajectory_sha256'])

    def test_legacy_dict_reader_consumes_plain_points_without_ros_or_moves(self):
        reader_setting = os.environ.get('UR7E_LEGACY_READER')
        if not reader_setting:
            self.skipTest('Set UR7E_LEGACY_READER to validate the external reference reader')
        reader_path = Path(reader_setting)
        if not reader_path.is_file():
            self.skipTest('Optional external reference reader is unavailable')
        tree = ast.parse(reader_path.read_text(encoding='utf-8'))
        reader = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                      and n.name == 'dict_to_trajectory')
        namespace = {'JointTrajectory': lambda: SimpleNamespace(points=[]),
                     'JointTrajectoryPoint': SimpleNamespace,
                     'rospy': SimpleNamespace(Duration=lambda value: value)}
        exec(compile(ast.Module(body=[reader], type_ignores=[]), str(reader_path), 'exec'), namespace)
        goal = namespace['dict_to_trajectory'](read_json(self.payload))
        self.assertEqual(goal.joint_names, self.record['joint_names'])
        for actual, expected in zip(goal.points, self.record['points']):
            for key in POINT_FIELDS:
                self.assertEqual(getattr(actual, key), expected[key])

    def test_no_pair_refuses_and_portable_builder_does_not_accept_plain_as_source(self):
        with self.assertRaisesRegex(ValueError, 'paired script'):
            portable_operation('inspect', self.payload)
        with self.assertRaises(OSError):
            pack_standalone(self.payload, self.source/'deployment.json', self.root/'invalid')

    def test_builder_rejects_corrupt_original_snapshot(self):
        manifest = read_json(self.source/'source_manifest.json')
        snapshot = self.source/next(iter(manifest['snapshots']))
        snapshot.write_bytes(snapshot.read_bytes()+b' ')
        with self.assertRaisesRegex(ValueError, 'snapshot fingerprint'):
            pack_standalone(self.source/'trajectory.json', self.source/'deployment.json',
                            self.root/'bad', format='plain')

    def test_approved_old_contract_requires_explicit_rebinding(self):
        config = copy.deepcopy(self.original_config)
        config['authorization']['approved_by'] = 'old_bundle_reviewer'
        write_json(self.source/'deployment.json', config)
        result = pack_standalone(self.source/'trajectory.json', self.source/'deployment.json',
                                self.root/'migration', allow_geometry_only=True, format='plain')
        self.assertTrue(result['authorization_rebinding_required'])
        self.script = self.root/'migration'/'ur7e_saved_trajectory_executor.py'
        module = self.load_module()
        result = module.portable_operation('inspect', self.script.parent/'trajectory.json')
        self.assertIn('authorization.input_contract', result['blockers'])
        self.assertIn('authorization.plain_trajectory_sha256', result['blockers'])
        self.assertIn('authorization.reference_sha256', result['blockers'])
        self.assertEqual(module.SITE_DEPLOYMENT['authorization']['approved_by'], 'old_bundle_reviewer')

    def test_default_builder_still_outputs_portable(self):
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json',
                        self.root/'portable', allow_geometry_only=True)
        self.assertEqual(read_json(self.root/'portable'/'trajectory.json')['schema'], 'ur7e_standalone_v1')

    def test_source_snapshots_are_not_needed_on_workstation(self):
        self.source.rename(self.root/'offline_source')
        result = self.invoke()
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(result['input_contract'], PLAIN_CONTRACT)
        self.assertEqual(result['source_validation'], 'original_bundle_verified_at_build_time')
        self.assertFalse(result['hardware_validated'])
        self.assertFalse(result['collision_dynamics_verified'])

    def test_check_sends_no_commands(self):
        result = self.invoke(command='check')
        self.assertEqual(result['status'], 'read_only_checked')
        b = self.backends[0]
        self.assertEqual((b.sent, b.cancelled, b.stops, b.beats), (0, 0, 0, 0))

    def test_twice_speed_and_keyboard_share_exact_scaled_goal(self):
        before = sha256(self.payload)
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        result = self.invoke(factor=2., operator=control)
        self.assertEqual(result['status'], 'completed')
        self.assertEqual((result['original_duration_s'], result['commanded_duration_s']), (1., .5))
        curve = self.backends[0].curve
        np.testing.assert_array_equal(curve.q, [p['positions'] for p in self.record['points']])
        np.testing.assert_array_equal(curve.v, np.array([p['velocities'] for p in self.record['points']])*2)
        np.testing.assert_array_equal(curve.a, np.array([p['accelerations'] for p in self.record['points']])*4)
        self.assertEqual(curve.record['phases'][-1]['end_s'], .5)
        self.assertEqual(sha256(self.payload), before)
        self.assertEqual(self.backends[0].sent, 1)

    def test_space_uses_original_stop_confirmation(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        result = self.invoke(operator=control, fault=lambda s: control.handle_key(' '))
        self.assertEqual(result['status'], 'operator_stopped')
        self.assertTrue(result['stop']['confirmed'])
        self.assertEqual((self.backends[0].cancelled, self.backends[0].stops), (1, 1))

    def test_force_stale_and_tracking_guards_remain(self):
        for name, fault in (
            ('force_limit', lambda s: s.update(wrench=[2.]*6)),
            ('stale_wrench', lambda s: s['ages'].update(wrench=10.)),
            ('joint_tracking', lambda s: s.update(q=[1.]*6))):
            with self.subTest(name=name):
                result = self.invoke(fault=fault)
                self.assertEqual(result['fault'], name)
                self.assertTrue(result['stop']['confirmed'])
                (self.root/'run').rename(self.root/name)

    def test_missing_site_setting_or_speed_limit_blocks_before_backend(self):
        for group, key, factor in (('identity', 'robot_ip', 1.),
                                   ('motion_limits', 'joint_velocity_rad_s', 2.)):
            with self.subTest(key=key):
                config = self.module.SITE_DEPLOYMENT
                original = config[group][key]
                config[group][key] = None
                result = self.invoke(factor=factor)
                self.assertEqual(result['status'], 'blocked')
                self.assertIn(group+'.'+key, result['blockers'])
                self.assertEqual(self.backends, [])
                config[group][key] = original
                (self.root/'run').rename(self.root/key)

    def test_modified_plain_json_or_reference_refuses_before_backend(self):
        original = self.payload.read_bytes()
        self.payload.write_bytes(original+b' ')
        with self.assertRaisesRegex(ValueError, 'does not match'):
            self.invoke()
        self.assertEqual(self.backends, [])
        self.payload.write_bytes(original)
        self.module.PLAIN_REFERENCE['tcp_poses'][0][0] = 1.
        with self.assertRaisesRegex(ValueError, 'changed_before_send'):
            self.invoke()

    def test_malformed_points_and_joint_order_refuse_even_with_matching_hash(self):
        original = read_json(self.payload)
        for kind in ('order', 'fields', 'count', 'nan', 'clock', 'endpoint'):
            with self.subTest(kind=kind):
                payload = copy.deepcopy(original)
                if kind == 'order':
                    payload['joint_names'].reverse()
                elif kind == 'fields':
                    payload['points'][0]['tcp_pose'] = [0.]*7
                elif kind == 'count':
                    payload['points'].pop()
                elif kind == 'nan':
                    payload['points'][1]['positions'][0] = float('inf')
                elif kind == 'clock':
                    payload['points'][1]['time_from_start'] = 0.
                else:
                    payload['points'][-1]['velocities'][0] = 1.
                if kind == 'nan':
                    self.payload.write_text(json.dumps(payload), encoding='utf-8')
                    self.rebind_mock_input()
                else:
                    self.rebind_mock_input(payload)
                with self.assertRaises(ValueError):
                    self.invoke()
                self.assertEqual(self.backends, [])

    def test_waiting_s_rechecks_file_script_reference_and_config(self):
        for kind in ('json', 'script', 'reference', 'config'):
            with self.subTest(kind=kind):
                original_bytes = self.payload.read_bytes(), self.script.read_bytes()
                config = copy.deepcopy(self.module.SITE_DEPLOYMENT)
                reference = copy.deepcopy(self.module.PLAIN_REFERENCE)
                def change(control):
                    if kind == 'json':
                        self.payload.write_bytes(original_bytes[0]+b' ')
                    elif kind == 'script':
                        self.script.write_bytes(original_bytes[1]+b'\n')
                    elif kind == 'reference':
                        self.module.PLAIN_REFERENCE['tcp_poses'][0][0] = 1.
                    else:
                        self.module.SITE_DEPLOYMENT['limits']['joint_tracking_rad'] *= 2
                    control.handle_key('S')
                result = self.invoke(operator=ScriptedKeyboard(change))
                self.assertEqual(result['status'], 'fault_latched')
                self.assertIn('changed_before_send', result['fault'])
                self.assertEqual(self.backends[0].sent, 0)
                self.payload.write_bytes(original_bytes[0])
                self.script.write_bytes(original_bytes[1])
                self.module.SITE_DEPLOYMENT = config
                self.module.PLAIN_REFERENCE = reference
                (self.root/'run').rename(self.root/kind)

    def test_authorization_expiry_after_wait_refuses(self):
        def expire(control):
            self.module.SITE_DEPLOYMENT['authorization']['expires_utc'] = (
                datetime.now(timezone.utc)-timedelta(seconds=1)).isoformat()
            control.handle_key('S')
        result = self.invoke(operator=ScriptedKeyboard(expire))
        self.assertEqual(result['status'], 'fault_latched')
        self.assertIn('authorization.expired', result['fault'])
        self.assertEqual(self.backends[0].sent, 0)

    def test_duplicate_json_key_refuses(self):
        self.payload.write_text('{"joint_names":[],"joint_names":[],"points":[]}', encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'Duplicate JSON key'):
            self.invoke()

    def test_isolated_cli_has_no_project_or_ros_import_requirement_for_inspect(self):
        env = dict(os.environ)
        env.pop('PYTHONPATH', None)
        result = subprocess.run([sys.executable, '-I', '-B', str(self.script), 'inspect',
                                 '--trajectory', str(self.payload)], cwd=self.delivery,
                                env=env, text=True, capture_output=True)
        self.assertEqual(result.returncode, 2, result.stdout+result.stderr)
        report = json.loads(result.stdout)
        self.assertIn('authorization.approved_by', report['blockers'])
        self.assertEqual(report['input_contract'], PLAIN_CONTRACT)


if __name__ == '__main__':
    unittest.main()
