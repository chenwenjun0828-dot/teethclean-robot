"""Playback controls and verified scene restoration for planning previews."""
from contextlib import redirect_stdout, redirect_stderr
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from .infrastructure.trajectory import Trajectory
from .visualization.playback import Playback, animation_times
from .visualization import view


def sample_trajectory():
    return Trajectory([0., .5, 2., 4.], [[0.], [.1], [.3], [.4]],
                      ['transfer', 'trace', 'trace'], [0, 1, 2], ['joint'])


class PlaybackTests(unittest.TestCase):
    def test_speed_and_pause_preserve_saved_trajectory(self):
        plan = sample_trajectory()
        saved = plan.times.copy()
        clock = Playback(plan, speed=2.)
        sample = clock.advance(.3)
        self.assertAlmostEqual(clock.time_s, .6)
        np.testing.assert_allclose(sample['q'], plan.sample(.6)['q'])
        clock.key(32)
        clock.advance(100.)
        self.assertAlmostEqual(clock.time_s, .6)
        clock.key(32)
        clock.advance(.2)
        self.assertAlmostEqual(clock.time_s, 1.)
        np.testing.assert_array_equal(plan.times, saved)

    def test_stepping_uses_neighboring_nodes_and_pauses(self):
        clock = Playback(sample_trajectory(), time_s=1.)
        clock.key(ord('N'))
        self.assertEqual(clock.time_s, 2.)
        self.assertTrue(clock.paused)
        clock.key(ord('N'))
        self.assertEqual(clock.time_s, 4.)
        clock.key(ord('N'))
        self.assertEqual(clock.time_s, 4.)
        clock.key(ord('P'))
        self.assertEqual(clock.time_s, 2.)
        clock.key(ord('P'))
        self.assertEqual(clock.time_s, .5)
        clock.key(ord('R'))
        self.assertEqual(clock.time_s, 0.)
        clock.key(ord('P'))
        self.assertEqual(clock.time_s, 0.)

    def test_loop_and_invalid_start_states(self):
        plan = sample_trajectory()
        clock = Playback(plan, speed=2., time_s=3.9)
        clock.advance(.1)
        self.assertAlmostEqual(clock.time_s, .1)
        for speed in (0., -1., float('nan')):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                Playback(plan, speed=speed)
        for start in (-1., 5., float('inf')):
            with self.subTest(start=start), self.assertRaises(ValueError):
                Playback(plan, time_s=start)

    def test_gif_timing_contains_endpoints_and_matches_playback_speed(self):
        times, delay = animation_times(90., 5., 10.)
        self.assertEqual(times[0], 0.)
        self.assertEqual(times[-1], 90.)
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertAlmostEqual(len(times)*delay*.001, 18.)
        times, delay = animation_times(90., 5., 24.)
        self.assertLess(abs(len(times)*delay*.001-18.), delay*.001+1e-9)
        for values in ((90., 0., 10.), (90., 1., 51.), (90., 1., 50.)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                animation_times(*values)


class OverlayTests(unittest.TestCase):
    def test_dense_curves_keep_endpoints_and_fit_geometry_budget(self):
        curves = [(np.arange(12000*3).reshape(-1, 3)+i, view.TRACE_COLOR) for i in range(8)]
        reduced = view.bounded_curves(curves)
        self.assertLessEqual(sum(len(points)-1 for points, color in reduced), 2000)
        for (original, color), (points, reduced_color) in zip(curves, reduced):
            np.testing.assert_array_equal(points[[0, -1]], original[[0, -1]])
            self.assertEqual(color, reduced_color)

    def test_segmented_preview_keeps_gaps_between_trace_blocks(self):
        plan = Trajectory(np.arange(6, dtype=float), np.arange(6, dtype=float)[:, None],
                          ['transfer', 'trace', 'retreat', 'ingress', 'trace'], [1]*5, ['joint'])

        def fk(q):
            pose = np.eye(4)
            pose[0, 3] = q[0]
            return pose

        curves = view.preview_curves({}, np.eye(4), plan, SimpleNamespace(fk=fk), {})
        self.assertEqual(len(curves), 2)
        np.testing.assert_array_equal(curves[0][0][:, 0], [1., 2.])
        np.testing.assert_array_equal(curves[1][0][:, 0], [4., 5.])


class ViewerLoadingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.report = {'planner': 'planning', 'world_from_scan_m': np.eye(4).tolist(),
                       'timing_profile': {'jaw_opening_deg': 20.}}
        self.write_report()
        np.savez(self.root/'geometry.npz', points_scan_m=np.array([[0., 0., 0.], [.01, .002, 0.], [.02, 0., .001]]),
                 stroke_id=np.zeros(3, int), region_id=np.ones(3, int))
        (self.root/'tooth_planes.json').write_text('{}', encoding='utf-8')

    def write_report(self):
        (self.root/'planning_report.json').write_text(json.dumps(self.report), encoding='utf-8')

    def test_trajectory_preview_uses_verified_loader_and_saved_jaw(self):
        (self.root/'trajectory.npz').touch()
        model = SimpleNamespace(ngeom=0)
        robot = object()
        scene = (model, object(), sample_trajectory(), SimpleNamespace(robot=robot))
        with patch('planning.motion_planning.execution.load_verified', return_value=scene) as verified, \
                patch.object(view, 'preview_curves', return_value=[]):
            loaded = view.load_scene(self.root)
            verified.assert_called_once_with(self.root, jaw_opening_deg=20.)
            self.assertIs(loaded[2], robot)
            view.load_scene(self.root, jaw_opening_deg=15.)
            self.assertEqual(verified.call_args.kwargs['jaw_opening_deg'], 15.)

    def test_viewer_prefers_saved_planning_opening(self):
        self.report['jaw_opening_deg'] = 25.
        self.write_report()
        self.assertEqual(view.saved_jaw_opening(self.report), 25.)

    def test_failed_verification_is_not_bypassed_and_jaw_error_is_actionable(self):
        (self.root/'trajectory.npz').touch()
        for message in ('Trajectory lacks a matching validation fingerprint', 'Dental pose changed since planning'):
            with self.subTest(message=message), \
                    patch('planning.motion_planning.execution.load_verified', side_effect=ValueError(message)), \
                    self.assertRaisesRegex(ValueError, '--jaw-opening-deg' if 'Dental' in message else 'fingerprint'):
                view.load_scene(self.root)

    def test_geometry_only_preview_does_not_require_motion_verification(self):
        model = SimpleNamespace(ngeom=0)
        self.report.pop('world_from_scan_m')
        self.report['jaw_opening_deg'] = 25.
        self.write_report()
        (self.root/'request.json').write_text(json.dumps({'inputs': {'jaw': 'mandibular'}}), encoding='utf-8')
        with patch('pipeline.simulation.create_simulation', return_value=(model, object())) as create, \
                patch('planning.infrastructure.robot.Robot', return_value=object()), \
                patch('planning.infrastructure.io.body_transform', return_value=np.eye(4)), \
                patch('planning.motion_planning.execution.load_verified') as verified:
            loaded = view.load_scene(self.root)
            self.assertIsNone(loaded[6])
            verified.assert_not_called()
            self.assertEqual(create.call_args.kwargs['jaw_settings'].opening_deg, 25.)
            self.assertEqual(len(loaded[3]['preview_curves']), 1)
            np.testing.assert_array_equal(loaded[5], np.eye(4))


class ViewerCliTests(unittest.TestCase):
    def setUp(self):
        self.scene = (SimpleNamespace(vis=SimpleNamespace(map=SimpleNamespace(znear=1.))),
                      object(), object(), {}, {}, np.eye(4), sample_trajectory(), {})

    def test_live_defaults_and_time_selection(self):
        with patch.object(view, 'load_scene', return_value=self.scene), \
                patch.object(view, 'make_camera'), patch.object(view, 'show') as show, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(view.main(['--plan', 'example', '--time', '2', '--paused', '--speed', '1']), 0)
            self.assertEqual(show.call_args.args[-1], 2.)
            self.assertTrue(show.call_args.args[-2].paused)
            self.assertEqual(show.call_args.args[-2].speed, 1.)

    def test_snapshot_respects_selected_waypoint(self):
        with patch.object(view, 'load_scene', return_value=self.scene), \
                patch.object(view, 'make_camera'), patch.object(view, 'export') as export, \
                patch.object(view, 'show') as show, redirect_stdout(io.StringIO()):
            self.assertEqual(view.main(['--plan', 'example', '--waypoint', '2', '--snapshot', 'pose.png']), 0)
            self.assertEqual(export.call_args.args[-1], 2.)
            show.assert_not_called()

    def test_invalid_options_and_geometry_gif_fail_before_rendering(self):
        with patch.object(view, 'load_scene', return_value=self.scene), \
                patch.object(view, 'export') as export, redirect_stdout(io.StringIO()):
            for extra in (['--time', '99'], ['--waypoint', '-1'], ['--fps', '51']):
                self.assertEqual(view.main(['--plan', 'example', *extra]), 1)
            self.scene = (*self.scene[:6], None, {})
            with patch.object(view, 'load_scene', return_value=self.scene):
                self.assertEqual(view.main(['--plan', 'example', '--gif', 'movie.gif']), 1)
            export.assert_not_called()
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            view.main(['--plan', 'example', '--speed', 'nan'])
        self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
