"""Exact tooth/TCP targets and actuator execution on a reachable pose fixture."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from perception.frames import camera_world_transform_m
from perception.gingival_line import RegisteredPaths, resample_path
from planning.ik import rotation_error
from planning.tooth_frames import ToothFrames, define_tooth_frames
from planning.trajectory import (ToothAlignedTargets, plan_tracking,
                                 plan_tooth_aligned_tracking, registered_tracking_targets,
                                 filter_tooth_orientation_targets)
from planning.validation import configure_collision_mode
from pipeline.simulation import create_simulation
from pipeline.tracking import run_tracking
from control.position import GingivalTCPController


class TCPToothAlignmentTests(unittest.TestCase):
    def fixture(self):
        model, data = create_simulation()
        configure_collision_mode(model, 'geometry')
        scratch = mujoco.MjData(model)
        points, rotations = [], []
        for u in np.linspace(0., 1., 5):
            scratch.qpos[:] = data.qpos + u*np.array([.01, -.02, .01, .02, .015, -.01])
            mujoco.mj_kinematics(model, scratch)
            points.append(scratch.site('ur_scaler_tcp').xpos.copy())
            rotations.append(scratch.site('ur_scaler_tcp').xmat.reshape(3, 3).copy())
        frames = ToothFrames(points, rotations, 'world', 'm')
        return model, data, ToothAlignedTargets(frames, frames, {}, 0, 0.)

    def assert_pose(self, model, q, point, rotation):
        data = mujoco.MjData(model)
        data.qpos[:] = q
        mujoco.mj_kinematics(model, data)
        self.assertLess(np.linalg.norm(data.site('ur_scaler_tcp').xpos-point), 2e-5)
        self.assertLess(np.linalg.norm(rotation_error(
            rotation, data.site('ur_scaler_tcp').xmat.reshape(3, 3))), 2e-4)

    def test_per_point_ik_and_separate_endpoint_clearance_follow_targets(self):
        model, data, targets = self.fixture()
        original = data.qpos.copy()
        plan = plan_tooth_aligned_tracking(model, data, targets)
        np.testing.assert_array_equal(data.qpos, original)
        for q, point, rotation in zip(plan.trace_q, targets.tcp.origins, targets.tcp.rotations):
            self.assert_pose(model, q, point, rotation)
        self.assert_pose(model, plan.approach_q,
                         plan.path[0]+.03*plan.rotations[0, :, 2], plan.rotations[0])
        self.assert_pose(model, plan.retreat_q,
                         plan.path[-1]+.03*plan.rotations[-1, :, 2], plan.rotations[-1])
        np.testing.assert_allclose(plan.trace_transforms, targets.tooth.transforms, atol=1e-12)
        np.testing.assert_allclose(Rotation.from_euler('xyz', plan.trace_poses[:, 3:]).as_matrix(),
                                   plan.rotations, atol=1e-12)
        expected = Slerp(plan.arc, Rotation.from_matrix(plan.rotations))(plan.arc[-1]*.5).as_matrix()
        np.testing.assert_allclose(plan.sample_rotation(plan.trace_start+plan.trace_duration*.5),
                                   expected, atol=1e-12)
        np.testing.assert_allclose(plan.sample_rotation(plan.trace_end), plan.rotations[-1], atol=1e-12)

    def test_resampling_pairs_positions_with_slerp_and_retains_fixed_api(self):
        model, data, targets = self.fixture()
        p = targets.tcp.origins[[0, -1]]
        r = targets.tcp.rotations[[0, -1]]
        plan = plan_tracking(model, data, p, r, spacing_m=.001)
        expected = Slerp([0., 1.], Rotation.from_matrix(r))(np.linspace(0., 1., len(plan.path)))
        np.testing.assert_allclose(plan.rotations, expected.as_matrix(), atol=1e-12)
        fixed = plan_tracking(model, data, p, r[0], spacing_m=.001)
        np.testing.assert_allclose(fixed.rotations, np.tile(r[0], (len(fixed.path), 1, 1)), atol=1e-12)
        self.assertEqual(fixed.orientation_mode, 'fixed')

    def test_bad_rotations_and_rotation_only_duplicate_are_rejected(self):
        model, data, targets = self.fixture()
        with self.assertRaisesRegex(ValueError, 'right-handed'):
            plan_tracking(model, data, targets.tcp.origins, np.diag([1., 1., -1.]))
        with self.assertRaisesRegex(ValueError, 'Rotations'):
            plan_tracking(model, data, targets.tcp.origins, np.eye(3)[None])
        with self.assertRaisesRegex(ValueError, 'Repeated positions'):
            plan_tracking(model, data, np.tile(targets.tcp.origins[0], (2, 1)),
                          targets.tcp.rotations[[0, -1]])

    def test_registered_geometry_uses_final_reversed_points_units_and_local_offset(self):
        for jaw in ('mandibular', 'maxillary'):
            with self.subTest(jaw=jaw):
                model, data = create_simulation()
                body = data.body('dental_'+jaw)
                transform = np.eye(4)
                transform[:3, :3] = body.xmat.reshape(3, 3)
                transform[:3, 3] = body.xpos
                scan = np.array([[0., 0., 0.], [3., 1., 2.], [6., 2., 2.]])
                world = scan/1000. @ transform[:3, :3].T+transform[:3, 3]
                # Deliberately pass the farther endpoint first to exercise reversal.
                if np.linalg.norm(world[0]-data.site('ur_scaler_tcp').xpos) < np.linalg.norm(
                        world[-1]-data.site('ur_scaler_tcp').xpos):
                    world = world[::-1].copy()
                    scan = scan[::-1].copy()
                bundle = RegisteredPaths(world, [world], camera_world_transform_m(data, 'global_dental_camera'),
                                         transform, scan, jaw, 'global_dental_camera', None, Path('unused.obj'))
                seen = []

                def build(mesh, points_mm, jaw_name, **kwargs):
                    seen.append(points_mm.copy())
                    # Vary normals with position, so incorrect point ordering cannot pass.
                    normals = np.column_stack((np.ones(len(points_mm)),
                                               np.zeros(len(points_mm)), points_mm[:, 0]*.1))
                    frames = define_tooth_frames(points_mm, normals, jaw_name)
                    return frames, np.zeros(len(points_mm)), {}

                with patch('planning.trajectory.build_tooth_frames', side_effect=build):
                    aligned = registered_tracking_targets(model, data, bundle, mesh_path='unused.ply')
                    offset = registered_tracking_targets(model, data, bundle, offset_mm=2., mesh_path='unused.ply')
                sampled = resample_path(world[::-1], bundle.sampling_spacing_m)
                np.testing.assert_allclose(aligned.tcp.origins, sampled, atol=1e-12)
                np.testing.assert_allclose(seen[0], (sampled-transform[:3, 3]) @ transform[:3, :3]*1000.)
                np.testing.assert_allclose(aligned.tcp.transforms, aligned.tooth.transforms, atol=1e-12)
                np.testing.assert_allclose(offset.tcp.origins-offset.tooth.origins,
                                           .002*offset.tooth.rotations[:, :, 2], atol=1e-12)
                np.testing.assert_allclose(offset.tcp.rotations, aligned.tcp.rotations)

    def test_reachable_changing_tooth_pose_executes_through_position_actuators(self):
        model, data, targets = self.fixture()
        original = data.qpos.copy()
        bundle = RegisteredPaths(targets.tcp.origins, [targets.tcp.origins], np.eye(4), np.eye(4),
                                 targets.tcp.origins*1000., 'mandibular', 'global_dental_camera',
                                 None, Path('fixture.obj'))
        with tempfile.TemporaryDirectory() as tmp, \
                patch('pipeline.tracking.load_registered_paths', return_value=bundle), \
                patch('pipeline.tracking.registered_tracking_targets', return_value=targets):
            report = run_tracking('fixture', headless=True, speed_mm_s=20., output_dir=tmp,
                                  model_data=(model, data), max_orientation_step_deg=None)
            self.assertEqual(report['status'], 'completed')
            self.assertGreater(data.time, 0.)
            self.assertGreater(np.linalg.norm(data.qpos-original), .001)
            self.assertLess(report['trace_rotation_max_deg'], 1.)
            self.assertLess(report['trace_max_mm'], 2.)
            np.testing.assert_allclose(np.load(Path(tmp)/'T_world_tcp.npy'),
                                       np.load(Path(tmp)/'T_world_tooth.npy'), atol=1e-12)
            rows = np.genfromtxt(Path(tmp)/'tcp_tracking.csv', delimiter=',', names=True, dtype=None,
                                 encoding='utf-8')
            self.assertIn('orientation_error_deg', rows.dtype.names)
            self.assertFalse(json.loads((Path(tmp)/'tracking_report.json').read_text())['fixed_orientation'])

    def test_controller_stops_on_orientation_error_even_when_position_matches(self):
        model, data, targets = self.fixture()
        plan = plan_tooth_aligned_tracking(model, data, targets)
        controller = GingivalTCPController(model, plan)
        controller.start_time = 0.
        data.time = plan.trace_start
        changed = Rotation.from_euler('z', 10., degrees=True).as_matrix() @ plan.rotations[0]
        with patch.object(plan, 'sample_rotation', return_value=changed):
            with self.assertRaisesRegex(RuntimeError, 'orientation tracking error'):
                controller.update(data)

    def test_filter_compares_last_retained_pose_and_preserves_all_positions(self):
        angles = [0., 10., 100., 20., 30., 150.]
        positions = np.column_stack((np.arange(6)*.001, np.zeros((6, 2))))
        frames = ToothFrames(positions, Rotation.from_euler('z', angles, degrees=True).as_matrix(),
                             'world', 'm')
        raw = ToothAlignedTargets(frames, frames, {}, 0, 0.)
        filtered = filter_tooth_orientation_targets(raw, 45.)
        selection = filtered.report['orientation_filter']
        self.assertEqual(selection['retained_sample_indices'], [0, 1, 3, 4])
        self.assertEqual([v['sample_index'] for v in selection['skipped_samples']], [2, 5])
        np.testing.assert_array_equal(filtered.tcp.origins, positions)
        np.testing.assert_array_equal(filtered.tcp.rotations[[0, 1, 3, 4]], frames.rotations[[0, 1, 3, 4]])
        np.testing.assert_allclose(filtered.tcp.rotations[2], Rotation.from_euler('z', 15., degrees=True).as_matrix())
        np.testing.assert_allclose(filtered.tcp.rotations[-1], frames.rotations[4])
        np.testing.assert_array_equal(raw.tcp.rotations, frames.rotations)
        self.assertNotIn('orientation_filter', raw.report)
        for value in (0., -1., np.nan, 181.):
            with self.assertRaisesRegex(ValueError, 'Orientation step'):
                filter_tooth_orientation_targets(raw, value)

    def test_filter_handles_one_retained_pose_and_exact_threshold(self):
        positions = np.column_stack((np.arange(3)*.001, np.zeros((3, 2))))
        frames = ToothFrames(positions, Rotation.from_euler('z', [0, 100, 120], degrees=True).as_matrix(),
                             'world', 'm')
        raw = ToothAlignedTargets(frames, frames, {}, 0, 0.)
        result = filter_tooth_orientation_targets(raw, 45.)
        np.testing.assert_allclose(result.tcp.rotations, np.tile(frames.rotations[0], (3, 1, 1)))
        result = filter_tooth_orientation_targets(raw, 180.)
        np.testing.assert_array_equal(result.tcp.rotations, frames.rotations)

    def test_auxiliary_motion_samples_preserve_targets_and_reduce_joint_interpolation_drift(self):
        model, data, targets = self.fixture()
        plan = plan_tooth_aligned_tracking(model, data, targets)
        self.assertGreater(len(plan.command_q), len(plan.trace_q))
        np.testing.assert_array_equal(plan.path, targets.tcp.origins)
        np.testing.assert_array_equal(plan.rotations, targets.tcp.rotations)
        self.assertEqual(plan.command_arc[0], 0.)
        self.assertEqual(plan.command_arc[-1], plan.arc[-1])
        self.assertTrue(np.all(np.diff(plan.command_arc) > 0))
        self.assertLessEqual(np.max(np.diff(plan.command_arc)), .00025+1e-12)
        scratch = mujoco.MjData(model)
        interpolator = Slerp(plan.arc, Rotation.from_matrix(plan.rotations))
        for a in (plan.command_arc[:-1]+plan.command_arc[1:])*.5:
            scratch.qpos[:] = [np.interp(a, plan.command_arc, plan.command_q[:, i]) for i in range(6)]
            mujoco.mj_kinematics(model, scratch)
            target = np.array([np.interp(a, plan.arc, plan.path[:, i]) for i in range(3)])
            self.assertLess(np.linalg.norm(scratch.site('ur_scaler_tcp').xpos-target), .00005)
            self.assertLess(np.linalg.norm(rotation_error(interpolator(a).as_matrix(),
                scratch.site('ur_scaler_tcp').xmat.reshape(3, 3))), .001)


if __name__ == '__main__':
    unittest.main()
