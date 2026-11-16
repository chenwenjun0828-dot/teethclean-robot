"""Session display agreement and isolated interactive planning lifecycle checks."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import numpy as np

from perception.session import DEFAULT_SESSION, load_session_region
from planning.config import Config, SweepOptions
from planning.infrastructure.io import body_transform
from planning.pose_search.constraints import target_at
from planning.simple import orient_reference
from planning.session import build_session_simulation
from planning.visualization.pose_angles import PosePreview, create_server
from planning.visualization.planning_jobs import PlanningJobs
from planning.visualization.planning_results import actual_tcp_path, summarize


def parameters(**values):
    return {'region': 'outer_front', 'planning_mode': 'simple',
            'pose_angles_deg': [20., -20., 10.], 'standoff_mm': 3.,
            'speed_mm_s': 1., 'transfer_speed_mm_s': 10., 'pose_spacing_mm': .25,
            'local_angle_search': True, 'check_mode': 'geometry_only', **values}


class ControlledProcess:
    def __init__(self, *args, **kwargs):
        self.done = threading.Event()

    def wait(self, timeout=None):
        if not self.done.wait(timeout):
            raise AssertionError('Test process did not terminate')
        return 0

    def poll(self):
        return 0 if self.done.is_set() else None

    def terminate(self):
        self.done.set()

    kill = terminate


class PlanningJobTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.jobs = PlanningJobs(lambda region: {'jaw': 'mandibular'}, self.temporary.name)
        self.addCleanup(self.jobs.close)

    def test_configuration_whitelist_and_fixed_execution_policy(self):
        values = parameters()
        result = self.jobs.configuration(values)
        self.assertFalse(result['execute'])
        self.assertFalse(result['geometry_only'])
        self.assertEqual(result['motion_mode'], 'non-contact')
        self.assertTrue(result['headless'])
        self.assertEqual(result['orientation_audit'], 'strict')
        for extra in ({'execute': True}, {'out': '../other'}, {'picked': [1, 2, 3]},
                      {'clearance_mm': 0}, {'jaw_opening_deg': 20}):
            with self.subTest(extra=extra), self.assertRaises(ValueError):
                self.jobs.configuration({**values, **extra})
        self.assertEqual(self.jobs.configuration(parameters(standoff_mm=.01))['standoff_mm'], .01)
        for value in (0, float('nan'), True, -1):
            with self.subTest(offset=value), self.assertRaises(ValueError):
                self.jobs.configuration(parameters(standoff_mm=value))

    @patch('planning.visualization.planning_jobs.subprocess.Popen', side_effect=ControlledProcess)
    def test_snapshot_serial_execution_cancellation_and_safe_command(self, launch):
        values = parameters()
        result = self.jobs.start(values)
        job_id = result['id']
        values['pose_angles_deg'][0] = 99
        self.assertEqual(self.jobs.snapshot(job_id)['parameters']['pose_angles_deg'][0], 20.)
        configuration = json.loads((Path(result['directory'])/'parameters.json').read_text())
        self.assertFalse(Path(configuration['out']).exists())
        self.assertEqual(launch.call_args.args[0][1:4], ['-u', '-m', 'planning'])
        self.assertNotIn('shell', launch.call_args.kwargs)
        with self.assertRaises(ValueError):
            self.jobs.start(parameters())
        with self.assertRaises(ValueError):
            self.jobs.snapshot('../other')
        self.jobs.cancel(job_id)
        self.jobs.jobs[job_id]['thread'].join(timeout=5)
        self.assertEqual(self.jobs.snapshot(job_id)['state'], 'cancelled')
        self.assertIsNone(self.jobs.snapshot(job_id)['result'])
        self.assertEqual(json.loads((Path(result['directory'])/'task.json').read_text())['state'], 'cancelled')

    @patch('planning.visualization.planning_jobs.subprocess.Popen', side_effect=ControlledProcess)
    def test_zero_exit_without_report_is_failure(self, _launch):
        result = self.jobs.start(parameters())
        job = self.jobs.jobs[result['id']]
        job['process'].done.set()
        job['thread'].join(timeout=5)
        result = self.jobs.snapshot(result['id'])
        self.assertEqual(result['state'], 'failed')
        self.assertFalse(result['result']['passed'])

    def test_partial_trajectory_and_forged_success_do_not_pass(self):
        directory = Path(self.temporary.name)
        (directory/'trajectory.npz').write_bytes(b'incomplete')
        self.assertFalse(summarize(directory, 'full', 0)['passed'])
        with self.assertRaises(ValueError):
            actual_tcp_path(directory)
        report = {'status': 'geometry_only_verified', 'geometry_verified': True,
                  'execution_ready': False, 'check_mode': 'geometry_only',
                  'trace_audit': {'passed': True}, 'motion_audit': {'passed': True}}
        (directory/'planning_report.json').write_text(json.dumps(report))
        self.assertTrue(summarize(directory, 'geometry_only', 0)['passed'])
        self.assertFalse(summarize(directory, 'geometry_only', 2)['passed'])
        report['execution_ready'] = True
        (directory/'planning_report.json').write_text(json.dumps(report))
        self.assertFalse(summarize(directory, 'geometry_only', 0)['passed'])
        with patch('planning.motion_planning.execution.load_verified', side_effect=ValueError('stale input')) as load:
            result = summarize(directory, 'full', 0)
            load.assert_called_once()
            self.assertFalse(result['passed'])
            self.assertEqual(result['error'], 'stale input')

    def test_session_loader_rejects_only_explicit_jaw_override(self):
        from planning.motion_planning.execution import load_verified
        directory = Path(self.temporary.name)
        # Deliberately incomplete certificates isolate the input-policy guard.
        (directory/'request.json').write_text(json.dumps({'perception_session': {'session_id': 'test'}}))
        (directory/'planning_report.json').write_text(json.dumps({'jaw_opening_deg': 0.}))
        with self.assertRaisesRegex(ValueError, 'Expected planning report'):
            load_verified(directory)
        for opening in (0., 20.):
            with self.subTest(opening=opening), self.assertRaisesRegex(ValueError, 'override is incompatible'):
                load_verified(directory, jaw_opening_deg=opening)


class SessionPreviewTests(unittest.TestCase):
    def test_session_preview_matches_planner_for_both_frames_and_all_modes(self):
        for frame in ('declared', 'color_optical'):
            preview = PosePreview(perception_session=DEFAULT_SESSION, session_camera_frame=frame,
                                  angles=(20., -20., 10.), standoff_mm=3.)
            loaded, model, data, base, jaws, _ = build_session_simulation(
                DEFAULT_SESSION, camera_frame=frame)
            prepared = loaded.with_world_base(base)
            np.testing.assert_allclose(preview.world, prepared.world_from_scan, atol=1e-12)
            for jaw in jaws:
                np.testing.assert_allclose(body_transform(preview.model, preview.data, jaw),
                                           body_transform(model, data, jaw), atol=1e-12)
            for region in ('outer_front', 'inner_front'):
                original = load_session_region(DEFAULT_SESSION, region=region, camera_frame=frame)
                actual = preview.get_region(region)
                np.testing.assert_allclose(actual.world_from_scan, base @ original.controller_base_from_scan)
                for mode in ('simple', 'local_surface', 'continuous'):
                    cfg = Config(region=region, planning_mode=mode, tilt_deg=20.,
                                 tilt_azimuth_deg=-20., standoff_mm=3.)
                    reference = actual.reference
                    if mode != 'continuous':
                        reference = orient_reference(reference, actual.geometry, cfg,
                                                     SweepOptions(fixed_roll_deg=10.), world=preview.world)
                    for station in (0., reference.end/2, reference.end):
                        pose = preview.pose(region=region, angles=(20., -20., 10.),
                                            standoff_mm=3., mode=mode, station_mm=station*1000)
                        np.testing.assert_allclose(pose['world_from_tcp_m'],
                            target_at(reference, station, 10., preview.world, cfg), atol=1e-12)
            self.assertEqual(preview.planning_source('inner_front')['session_camera_frame'], frame)
            self.assertFalse(preview.scene['session']['robot_execution_allowed'])

    def test_conflicting_session_inputs_are_rejected(self):
        for options in ({'perception_result': 'other'}, {'jaw_opening_deg': 20}):
            with self.assertRaises(ValueError):
                PosePreview(perception_session=DEFAULT_SESSION, **options)

    def test_source_changes_require_a_new_preview(self):
        preview = PosePreview(perception_session=DEFAULT_SESSION)
        loaded = load_session_region(DEFAULT_SESSION)
        changed = replace(loaded, metadata={**loaded.metadata, 'source_files': {}})
        with patch('planning.visualization.pose_angles.load_session_region', return_value=changed):
            with self.assertRaisesRegex(ValueError, 'sources changed'):
                preview.planning_source('outer_front')

    def test_display_uses_saved_interpolant_and_fk(self):
        from planning.infrastructure.io import dump, fingerprint
        from planning.infrastructure.robot import Robot
        from planning.infrastructure.trajectory import Trajectory
        _, model, data, _, _, info = build_session_simulation(DEFAULT_SESSION)
        robot = Robot(model, data)
        joints = np.vstack([robot.home, robot.home + [.01, 0, 0, 0, 0, 0]])
        trajectory = Trajectory(np.array([0., 1.]), joints, ['transfer'], [-1], robot.joint_names)
        with tempfile.TemporaryDirectory() as name:
            directory = Path(name)
            trajectory.save(directory/'trajectory.npz')
            dump(directory/'request.json', {'perception_session': info})
            dump(directory/'planning_report.json', {'trajectory_file': fingerprint(directory/'trajectory.npz')})
            result = actual_tcp_path(directory)
        self.assertTrue(result['display_only'])
        for index in (0, len(result['points_world_m'])//2, len(result['points_world_m'])-1):
            time = index/(len(result['points_world_m'])-1)
            np.testing.assert_allclose(result['points_world_m'][index],
                                       robot.fk(trajectory.sample(time)['q'])[:3, 3], atol=1e-12)


class PlanningHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.preview = PosePreview()
        cls.jobs = PlanningJobs(cls.preview.planning_source, cls.temporary.name)
        cls.server = create_server(cls.preview, 0, jobs=cls.jobs)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.url = f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.jobs.close()
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join()
        cls.temporary.cleanup()

    def post(self, path, values, **headers):
        request = Request(self.url+path, data=json.dumps(values).encode(), headers={
            'Content-Type': 'application/json', 'Origin': self.url,
            'X-Planning-Token': self.preview.planning_token, **headers})
        with urlopen(request) as response:
            return json.load(response)

    def test_same_origin_configuration_and_invalid_origins(self):
        result = self.post('/api/configuration', parameters())
        self.assertFalse(result['execute'])
        for headers in ({'Origin': 'http://other.example'}, {'X-Planning-Token': 'wrong'},
                        {'Host': 'other.example'}):
            with self.subTest(headers=headers), self.assertRaises(HTTPError) as error:
                self.post('/api/plan', parameters(), **headers)
            self.assertEqual(error.exception.code, 403)
        with self.assertRaises(HTTPError):
            self.post('/api/configuration', parameters(out='other'))
        with urlopen(self.url+'/api/task') as response:
            self.assertIsNone(json.load(response))


if __name__ == '__main__':
    unittest.main()
