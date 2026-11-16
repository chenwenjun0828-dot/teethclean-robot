import unittest
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

import numpy as np

from perception.session import (load_session_region, load_session_robot_observation,
                                load_session_scan_transform, load_session_scan_transform_chain)
from pipeline.simulation import body_transform
from planning.session import (FRAME_ADAPTER_VERSION, build_session_simulation,
                              rebuild_session_simulation)
from perception.regions.io import dump, fingerprint
from planning.__main__ import main
from planning.visualization.view import load_scene


SESSION = Path('assets/perception/sessions/20261113_220209')


class PerceptionSessionTests(unittest.TestCase):
    def test_new_session_api_defaults_match_explicit_optical_interpretation(self):
        default_region = load_session_region(SESSION)
        optical_region = load_session_region(SESSION, camera_frame='color_optical')
        self.assertEqual(default_region.metadata['camera_frame'], 'color_optical')
        np.testing.assert_array_equal(default_region.controller_base_from_scan,
                                      optical_region.controller_base_from_scan)
        np.testing.assert_array_equal(default_region.base_points_m, optical_region.base_points_m)
        for jaw in ('mandibular', 'maxillary'):
            chain = load_session_scan_transform_chain(SESSION, jaw)
            optical = load_session_scan_transform_chain(SESSION, jaw, camera_frame='color_optical')
            self.assertEqual(chain['camera_frame'], 'color_optical')
            np.testing.assert_array_equal(chain['effective_controller_base_from_scan'],
                                          optical['effective_controller_base_from_scan'])
            np.testing.assert_array_equal(load_session_scan_transform(SESSION, jaw),
                                          optical['effective_controller_base_from_scan'])
        *_, default_info = build_session_simulation(SESSION)
        *_, optical_info = build_session_simulation(SESSION, camera_frame='color_optical')
        self.assertEqual(default_info['camera_frame'], 'color_optical')
        for jaw in ('mandibular', 'maxillary'):
            np.testing.assert_array_equal(default_info['jaw_transforms_world_m'][jaw],
                                          optical_info['jaw_transforms_world_m'][jaw])

    def test_cli_without_frame_exports_and_replays_the_explicit_optical_pose(self):
        *_, expected = build_session_simulation(SESSION, camera_frame='color_optical')
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            for frame in (None, 'declared'):
                output = Path(directory) / (frame or 'default')
                arguments = ['--perception-session', str(SESSION), '--geometry-only', '--out', str(output)]
                if frame is not None:
                    arguments += ['--session-camera-frame', frame]
                self.assertEqual(main(arguments), 0)
                scene = load_scene(output)
                info = scene[-1]['perception_session']
                self.assertEqual(info['camera_frame'], frame or 'color_optical')
                np.testing.assert_array_equal(info['robot_joint_positions_rad'],
                                              expected['robot_joint_positions_rad'])
                for jaw in ('mandibular', 'maxillary'):
                    effective = load_session_scan_transform(SESSION, jaw,
                        camera_frame=frame or 'color_optical')
                    np.testing.assert_allclose(info['jaw_scan_transforms_controller_base_m'][jaw],
                                              effective, atol=1e-12, rtol=0)
                    if frame is None:
                        np.testing.assert_allclose(info['jaw_transforms_world_m'][jaw],
                            expected['jaw_transforms_world_m'][jaw], atol=1e-12, rtol=0)

    def test_optical_chain_matches_independent_point_transform(self):
        sdk_record = json.loads(Path('assets/calibration/ur7e_d405/20261113/camera_extrinsics.json')
                                .read_text(encoding='utf-8'))['raw_sdk_extrinsics']
        sdk_rotation = np.asarray(sdk_record['rotation_column_major']).reshape(3, 3, order='F')
        for jaw in ('mandibular', 'maxillary'):
            chain = load_session_scan_transform_chain(SESSION, jaw, camera_frame='color_optical')
            with np.load(SESSION / jaw / 'shared/region_mesh.npz') as mesh:
                points = mesh['vertices'][::500]
            expected = points.copy()
            for transform in (np.load(SESSION / jaw / 'T_depth_optical_scan_m.npy'),):
                expected = expected @ transform[:3, :3].T + transform[:3, 3]
            expected = expected @ sdk_rotation.T + sdk_record['translation_m']
            for name in ('T_wrist3_color_frame_m.npy', 'T_controller_base_wrist3_nominal_m.npy'):
                transform = np.load(SESSION / 'transforms' / name)
                expected = expected @ transform[:3, :3].T + transform[:3, 3]
            effective = chain['effective_controller_base_from_scan']
            np.testing.assert_allclose(points @ effective[:3, :3].T + effective[:3, 3],
                                       expected, atol=2e-8, rtol=0)
            self.assertFalse(np.allclose(effective, chain['published_controller_base_from_scan']))

    def test_optical_region_rotates_all_base_geometry_and_preserves_local_curve(self):
        for jaw in ('mandibular',):
            original = load_session_region(SESSION, jaw, camera_frame='declared')
            optical = load_session_region(SESSION, jaw, camera_frame='color_optical')
            self.assertEqual(original.prepared.reference.record, optical.prepared.reference.record)
            np.testing.assert_array_equal(original.prepared.geometry.mesh_vertices_m,
                                          optical.prepared.geometry.mesh_vertices_m)
            delta = optical.controller_base_from_scan @ np.linalg.inv(original.controller_base_from_scan)
            np.testing.assert_allclose(optical.base_points_m,
                original.base_points_m @ delta[:3, :3].T + delta[:3, 3], atol=1e-12, rtol=0)
            for name in ('base_normals', 'base_tangents'):
                np.testing.assert_allclose(getattr(optical, name),
                    getattr(original, name) @ delta[:3, :3].T, atol=1e-12, rtol=0)
            np.testing.assert_array_equal(optical.prepared.world_from_scan,
                                          optical.controller_base_from_scan)

    def test_optical_scene_places_teeth_below_tcp_and_keeps_capture_joints(self):
        original = build_session_simulation(SESSION, camera_frame='declared')
        region, model, data, world_base, transforms, info = build_session_simulation(
            SESSION, camera_frame='color_optical')
        np.testing.assert_array_equal(data.qpos, original[2].qpos)
        tcp_z = data.site_xpos[model.site('ur_scaler_tcp').id, 2]
        for jaw, expected_height_mm in (('mandibular', -91.064), ('maxillary', -53.778)):
            with np.load(SESSION / jaw / 'shared/region_mesh.npz') as mesh:
                vertices = mesh['vertices']
            world = transforms[jaw]
            center = world[:3, :3] @ vertices.mean(axis=0) + world[:3, 3]
            self.assertAlmostEqual((center[2] - tcp_z) * 1000, expected_height_mm, delta=.02)
            np.testing.assert_allclose(body_transform(model, data, 'dental_' + jaw),
                                       world, atol=3e-8, rtol=0)
        np.testing.assert_allclose(region.with_world_base(world_base).world_from_scan,
                                   transforms['mandibular'], atol=1e-12, rtol=0)
        self.assertEqual(info['camera_frame_status'], 'optical_frame_hypothesis_live_tf_unverified')

    def test_optical_mode_still_rejects_changed_original_sources(self):
        def changed(path):
            record = fingerprint(path)
            if Path(path).name == 'camera_extrinsics.json':
                record['sha256'] = 'changed'
            return record
        with patch('perception.session.fingerprint', side_effect=changed):
            with self.assertRaisesRegex(ValueError, 'source fingerprint changed'):
                load_session_scan_transform_chain(SESSION, 'mandibular', camera_frame='color_optical')

    def test_optical_mode_still_audits_original_published_chain(self):
        original_load = np.load

        def changed(path, *args, **kwargs):
            value = original_load(path, *args, **kwargs)
            if Path(path).name == 'T_color_frame_depth_optical_ros_convention_m.npy':
                value = value.copy()
                value[0, 3] += .01
            return value

        with patch('perception.session.np.load', side_effect=changed):
            with self.assertRaisesRegex(ValueError, 'chain differs from published'):
                load_session_scan_transform_chain(SESSION, 'mandibular', camera_frame='color_optical')

    def test_optical_replay_roundtrip_and_tampered_records(self):
        *_, info = build_session_simulation(SESSION, camera_frame='color_optical')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'session.json'
            dump(path, info)
            saved = json.loads(path.read_text(encoding='utf-8'))
        rebuilt = rebuild_session_simulation(saved)
        np.testing.assert_allclose(rebuilt[0].controller_base_from_scan,
                                   info['jaw_scan_transforms_controller_base_m']['mandibular'])
        for kind in ('interpretation', 'matrix', 'fingerprint', 'version'):
            altered = deepcopy(saved)
            if kind == 'interpretation':
                altered['camera_frame'] = 'declared'
            elif kind == 'matrix':
                altered['jaw_transforms_world_m']['mandibular'][0][3] += .01
            elif kind == 'fingerprint':
                altered['source_files']['camera_extrinsics.json']['sha256'] = 'changed'
            else:
                altered.pop('camera_frame_version')
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                rebuild_session_simulation(altered)

    def test_legacy_replay_defaults_to_declared(self):
        *_, info = build_session_simulation(SESSION, camera_frame='declared')
        for key in ('camera_frame', 'camera_frame_version', 'camera_frame_status', 'source_files',
                    'jaw_scan_transforms_published_controller_base_m',
                    'jaw_effective_transform_chain_m', 'jaw_published_transform_chain_m'):
            info.pop(key)
        *_, rebuilt = rebuild_session_simulation(info)
        self.assertEqual(rebuilt['camera_frame'], 'declared')

    def test_cli_optical_geometry_export_replays_with_same_region_transform(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()):
            output = Path(directory) / 'preview'
            self.assertEqual(main(['--params', 'config/planning_session_optical_preview.json',
                                   '--out', str(output)]), 0)
            model, data, _, geometry, _, world, trajectory, report = load_scene(output)
            self.assertIsNone(trajectory)
            self.assertFalse(report['execution_ready'])
            self.assertEqual(report['perception_session']['camera_frame'], 'color_optical')
            np.testing.assert_allclose(world, body_transform(model, data, 'dental_mandibular'),
                                       atol=1e-12, rtol=0)
            self.assertGreater(len(geometry['preview_curves']), 0)

    def test_cli_camera_option_requires_session_and_cli_overrides_json(self):
        with redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(main(['--session-camera-frame', 'color_optical', '--dry-run']), 1)
        self.assertIn('requires --perception-session', stream.getvalue())
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()) as stream:
            self.assertEqual(main(['--params', 'config/planning_session_optical_preview.json',
                                   '--session-camera-frame', 'declared', '--dry-run',
                                   '--out', str(Path(directory) / 'dry_run')]), 0)
        self.assertEqual(json.loads(stream.getvalue())['session_camera_frame'], 'declared')

    def test_region_load_keeps_scan_and_controller_base_contract(self):
        region = load_session_region(SESSION, 'mandibular', 'outer_front', camera_frame='declared')
        self.assertEqual(region.prepared.inputs.jaw, 'mandibular')
        self.assertEqual(region.base_points_m.shape, region.base_normals.shape)
        self.assertEqual(region.base_points_m.shape, region.base_tangents.shape)
        self.assertTrue(np.allclose(region.prepared.world_from_scan,
                                    region.controller_base_from_scan))
        # The published base points are a validation artifact; they are not
        # fed back through T_controller_base_scan during planning.
        self.assertTrue(np.isfinite(region.base_points_m).all())

    def test_session_scan_chain_reproduces_published_pose(self):
        for jaw in ('mandibular', 'maxillary'):
            chain = load_session_scan_transform_chain(SESSION, jaw, camera_frame='declared')
            self.assertTrue(np.allclose(
                chain['computed_controller_base_from_scan'],
                chain['published_controller_base_from_scan'], atol=5e-7, rtol=0))
            np.testing.assert_array_equal(chain['effective_controller_base_from_scan'],
                                          chain['published_controller_base_from_scan'])

    def test_session_robot_observation_has_capture_joint_order_and_radians(self):
        observation = load_session_robot_observation(SESSION)
        self.assertEqual(len(observation.joint_names), 6)
        self.assertEqual(observation.joint_positions_rad.shape, (6,))
        self.assertTrue(np.isfinite(observation.joint_positions_rad).all())

    def test_session_scene_places_both_jaws_from_one_controller_base_anchor(self):
        region, model, data, world_base, transforms, info = build_session_simulation(SESSION)
        self.assertEqual(info['coordinate_frame'], 'controller_base')
        self.assertFalse(info['robot_execution_allowed'])
        ur_base = body_transform(model, data, 'ur_base')
        expected_controller_base = ur_base @ np.diag([-1., -1., 1., 1.])
        self.assertTrue(np.allclose(world_base, expected_controller_base))
        self.assertEqual(info['frame_adapter_version'], FRAME_ADAPTER_VERSION)
        observation = load_session_robot_observation(SESSION)
        qadr = np.array([model.jnt_qposadr[model.joint('ur_' + name).id]
                         for name in observation.joint_names])
        self.assertTrue(np.allclose(data.qpos[qadr], observation.joint_positions_rad,
                                    atol=1e-10, rtol=0))
        for jaw, expected in transforms.items():
            self.assertTrue(np.allclose(body_transform(model, data, f'dental_{jaw}'),
                                        expected, atol=3e-8, rtol=0))
        expected_region_world = world_base @ region.controller_base_from_scan
        self.assertTrue(np.allclose(region.with_world_base(world_base).world_from_scan,
                                    expected_region_world))

    def test_replay_rejects_legacy_unadapted_session_record(self):
        _, _, _, _, _, info = build_session_simulation(SESSION)
        info.pop('frame_adapter_version')
        with self.assertRaisesRegex(ValueError, 'obsolete session base-frame adapter'):
            rebuild_session_simulation(info)


if __name__ == '__main__':
    unittest.main()
