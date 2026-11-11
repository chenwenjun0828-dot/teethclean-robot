"""Real-camera contracts tested with SDK doubles and independent synthetic poses."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import open3d as o3d

from perception.acquisition import RGBDFrame, save_real_capture
from perception.acquisition.capture import depth_roi_mask
from perception.acquisition.realsense import RealSenseCamera, _texture_colors
from perception.calibration import calibrate_eye_to_hand, load_base_calibration
from perception.global_camera import file_hash, load_capture, write_json
from perception.real import load_real_config, main, register_saved_capture, recrop_saved_capture
from perception.registration.evaluation import (evaluate_checkpoints, evaluate_registration,
                                               quality_assessment, repeatability_report)
from perception.registration.registration_math import transform_points
from tests.test_global_registration import asymmetric_cloud


def small_frame():
    raw = np.full((4, 5), 1000, np.uint16)
    y, x = np.indices(raw.shape)
    points = np.stack((x, y, np.full(raw.shape, 100.)), axis=2).astype(float)
    return RGBDFrame(np.zeros((3, 6, 3), np.uint8), raw, .0001, points,
                     np.full(points.shape, .5), np.ones(raw.shape, bool),
                     {"fx": 100., "fy": 100., "cx": 2., "cy": 1.5},
                     {"camera_serial": "TEST-D405", "depth_timestamp_ms": 123.})


def sdk_double():
    sdk = Mock()
    sdk.camera_info = SimpleNamespace(name="name", serial_number="serial", firmware_version="firmware")
    sdk.stream = SimpleNamespace(depth="depth", color="color")
    sdk.format = SimpleNamespace(z16="z16", rgb8="rgb8")
    device = Mock()
    device.get_info.side_effect = lambda key: {"name": "Intel RealSense D405", "serial": "TEST-D405", "firmware": "test"}[key]
    device.first_depth_sensor.return_value.get_depth_scale.return_value = .0001
    sdk.context.return_value.query_devices.return_value = [device]
    profile = sdk.pipeline.return_value.start.return_value
    profile.get_device.return_value = device
    depth_profile, color_profile = Mock(), Mock()
    profile.get_stream.side_effect = lambda key: depth_profile if key == "depth" else color_profile
    depth_profile.as_video_stream_profile.return_value = depth_profile
    color_profile.as_video_stream_profile.return_value = color_profile
    intrinsics = SimpleNamespace(fx=100., fy=100., ppx=2., ppy=1.5,
                                 width=5, height=4, model="none", coeffs=[0.]*5)
    depth_profile.get_intrinsics.return_value = intrinsics
    color_profile.get_intrinsics.return_value = intrinsics
    rotation = o3d.geometry.get_rotation_matrix_from_xyz([0., 0., .2])
    depth_profile.get_extrinsics_to.return_value = SimpleNamespace(
        rotation=rotation.flatten(order="F"), translation=[.01, 0., 0.])
    frame = small_frame()
    frames = sdk.pipeline.return_value.wait_for_frames.return_value
    depth, color = frames.get_depth_frame.return_value, frames.get_color_frame.return_value
    depth.get_data.return_value, color.get_data.return_value = frame.depth_raw, frame.rgb
    for value in (depth, color):
        value.get_timestamp.return_value = 123.
        value.get_frame_number.return_value = 7
        value.get_frame_timestamp_domain.return_value = "hardware_clock"
    points = sdk.pointcloud.return_value.calculate.return_value
    points.get_vertices.return_value = (frame.points_mm / 1000).astype(np.float32)
    points.get_texture_coordinates.return_value = np.full(frame.depth_raw.shape + (2,), .5, np.float32)
    return sdk


class CameraSourceTests(unittest.TestCase):
    def test_depth_scale_native_grid_and_extrinsic_column_major_order(self):
        sdk = sdk_double()
        with RealSenseCamera(load_real_config()["camera"], sdk) as camera:
            frame = camera.read()
        self.assertEqual(frame.depth_scale_m, .0001)
        np.testing.assert_allclose(frame.depth_m, .1)
        np.testing.assert_allclose(frame.points_mm, small_frame().points_mm, atol=1e-5)
        np.testing.assert_allclose(np.asarray(frame.metadata["T_color_depth_m"])[:3, :3],
                                   o3d.geometry.get_rotation_matrix_from_xyz([0., 0., .2]))
        self.assertEqual(frame.metadata["camera_serial"], "TEST-D405")
        self.assertEqual(frame.rgb.shape, (3, 6, 3))
        sdk.pipeline.return_value.stop.assert_called_once()

    def test_timeout_releases_camera_and_read_after_close_fails(self):
        sdk = sdk_double()
        options = dict(load_real_config()["camera"], warmup_frames=0)
        sdk.pipeline.return_value.wait_for_frames.side_effect = RuntimeError("Frame did not arrive")
        source = RealSenseCamera(options, sdk)
        with self.assertRaisesRegex(RuntimeError, "arrive"):
            with source:
                source.read()
        sdk.pipeline.return_value.stop.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, "not open"):
            source.read()

    def test_warmup_failure_also_releases_pipeline(self):
        sdk = sdk_double()
        sdk.pipeline.return_value.wait_for_frames.side_effect = RuntimeError("warmup")
        with self.assertRaisesRegex(RuntimeError, "warmup"):
            with RealSenseCamera(load_real_config()["camera"], sdk):
                pass
        sdk.pipeline.return_value.stop.assert_called_once()

    def test_no_matching_device_fails_before_opening_pipeline(self):
        sdk = sdk_double()
        sdk.context.return_value.query_devices.return_value = []
        with self.assertRaisesRegex(RuntimeError, "No matching D405"):
            with RealSenseCamera(load_real_config()["camera"], sdk):
                pass
        sdk.pipeline.assert_not_called()

    def test_texture_invalid_coordinates_do_not_wrap_into_color_image(self):
        rgb = np.array([[[255, 0, 0], [0, 255, 0]]], np.uint8)
        uv = np.array([[[0., 0.], [.5, 0.], [-.1, 0.], [np.nan, 0.]]])
        colors, valid = _texture_colors(rgb, uv)
        np.testing.assert_array_equal(valid, [[True, True, False, False]])
        np.testing.assert_allclose(colors[0, :2], [[1, 0, 0], [0, 1, 0]])
        np.testing.assert_allclose(colors[0, 2:], .5)

    def test_offline_imports_do_not_load_mujoco_or_camera_sdk(self):
        result = subprocess.run([sys.executable, "-c",
            "import sys; import perception.real; assert 'mujoco' not in sys.modules; "
            "assert 'pyrealsense2' not in sys.modules"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


class RealCaptureTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config = dict(load_real_config(), capture_minimum_points=3,
                           capture_outlier_neighbors=0, roi=[1, 1, 4, 3])

    def test_snapshot_roundtrip_has_no_simulation_truth_or_fake_base_pose(self):
        metadata = save_real_capture(self.root / "capture", small_frame(), "mandibular", self.config)
        loaded = load_capture(self.root / "capture")
        self.assertIsNone(loaded["segmentation"])
        self.assertEqual(metadata["valid_points"], 6)
        self.assertNotIn("T_world_camera_cv_m", metadata)
        self.assertFalse((self.root / "capture/evaluation").exists())
        self.assertEqual(metadata["depth_scale_m"], .0001)
        cloud = o3d.io.read_point_cloud(str(self.root / "capture/camera_mm.ply"))
        np.testing.assert_allclose(np.asarray(cloud.points)[:, 2], 100.)
        self.assertEqual(loaded["rgb"].shape, (3, 6, 3))
        with self.assertRaises(FileExistsError):
            save_real_capture(self.root / "capture", small_frame(), "mandibular", self.config)

    def test_tampered_cloud_and_missing_integrity_entry_are_rejected(self):
        out = self.root / "capture"
        metadata = save_real_capture(out, small_frame(), "mandibular", self.config)
        with (out / "camera_mm.ply").open("ab") as handle:
            handle.write(b"tampered")
        with self.assertRaisesRegex(ValueError, "integrity"):
            load_capture(out)
        metadata["file_hashes"].pop("camera_mm.ply")
        write_json(out / "capture_metadata.json", metadata)
        with self.assertRaisesRegex(ValueError, "fingerprints"):
            load_capture(out)

    def test_invalid_roi_scale_geometry_and_moving_jaws_are_rejected(self):
        for roi in ([0, 0, 6, 4], [1, 1, 1, 2], [0, 0, 2.5, 3]):
            with self.subTest(roi=roi), self.assertRaises(ValueError):
                depth_roi_mask(small_frame().depth_m, dict(self.config, roi=roi))
        frame = small_frame()
        frame.depth_scale_m = .001
        with self.assertRaisesRegex(ValueError, "disagree"):
            save_real_capture(self.root / "bad", frame, "mandibular", self.config)
        with self.assertRaisesRegex(ValueError, "separately"):
            save_real_capture(self.root / "bad", small_frame(), "both", self.config)

    def test_recropping_restores_points_outside_the_original_roi_without_new_acquisition(self):
        first = save_real_capture(self.root / "capture", small_frame(), "mandibular", self.config)
        second = recrop_saved_capture(self.root / "capture", self.root / "recrop", dict(self.config, roi=None))
        self.assertEqual(first["valid_points"], 6)
        self.assertEqual(second["valid_points"], 20)
        self.assertNotEqual(first["capture_id"], second["capture_id"])
        self.assertEqual(first["acquisition_id"], second["acquisition_id"])
        self.assertEqual(first["capture_id"], second["parent_capture_id"])

    def test_batch_capture_cli_records_distinct_acquisitions_without_a_display(self):
        source = Mock()
        source.__enter__ = Mock(return_value=source)
        source.__exit__ = Mock(return_value=None)
        source.read.return_value = small_frame()
        with patch("perception.real.load_real_config", return_value=self.config), \
                patch("perception.acquisition.realsense.RealSenseCamera", return_value=source):
            self.assertEqual(main(["capture", "--jaw", "mandibular", "--count", "2",
                                   "--out", str(self.root / "session")]), 0)
        session = json.loads((self.root / "session/session.json").read_text())
        self.assertTrue(session["complete"])
        acquisitions = [load_capture(item["path"])["metadata"]["acquisition_id"] for item in session["captures"]]
        self.assertEqual(len(set(acquisitions)), 2)
        source.__exit__.assert_called_once()


def calibration_samples():
    def pose(angles, translation):
        result = np.eye(4)
        result[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz(angles)
        result[:3, 3] = translation
        return result
    base_camera = pose([.2, -.3, .4], [.4, -.1, .25])
    gripper_target = pose([-.1, .2, .1], [.02, .04, .08])
    rng = np.random.default_rng(37)
    samples = []
    for _ in range(12):
        bg = pose(rng.uniform(-.8, .8, 3), rng.uniform(-.2, .2, 3))
        ct = np.linalg.inv(base_camera) @ bg @ gripper_target
        samples.append({"T_base_gripper_m": bg.tolist(), "T_camera_target_m": ct.tolist()})
    payload = {"units": "m", "camera_serial": "TEST-D405", "optical_frame": "depth", "samples": samples[:9]}
    validation = dict(payload, samples=samples[9:])
    return payload, validation, base_camera


class CalibrationTests(unittest.TestCase):
    def test_eye_to_hand_recovers_direction_and_units_with_held_out_poses(self):
        payload, validation, truth = calibration_samples()
        result = calibrate_eye_to_hand(payload, validation)
        np.testing.assert_allclose(result["T_base_camera_m"], truth, atol=1e-9)
        self.assertLess(result["held_out_validation"]["translation_rms_mm"], 1e-6)
        self.assertFalse(result["robot_execution_allowed"])

    def test_degenerate_motion_and_reused_validation_pose_are_rejected(self):
        payload, _, _ = calibration_samples()
        with self.assertRaisesRegex(ValueError, "reuses"):
            calibrate_eye_to_hand(payload, payload)
        payload["samples"] = [payload["samples"][0]] * 3
        with self.assertRaisesRegex(ValueError, "degenerate"):
            calibrate_eye_to_hand(payload)

    def test_calibration_cannot_be_applied_to_another_camera_or_optical_frame(self):
        payload, _, _ = calibration_samples()
        result = calibrate_eye_to_hand(payload)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            write_json(path, result)
            with self.assertRaisesRegex(ValueError, "camera_serial"):
                load_base_calibration(path, {"camera_serial": "another", "optical_frame": "depth"})
            with self.assertRaisesRegex(ValueError, "optical_frame"):
                load_base_calibration(path, {"camera_serial": "TEST-D405", "optical_frame": "color"})


class EvaluationTests(unittest.TestCase):
    def setUp(self):
        self.report = {"capture_id": "capture", "source_hashes": {"mandibular": "sha"}}
        self.payload = {**self.report, "source_scan_mm": [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                        "target_camera_mm": [[0, 0, 1], [1, 0, 1], [0, 1, 1]],
                        "independent_of_registration": True}

    def test_checkpoint_errors_are_withheld_3d_distances(self):
        result = evaluate_checkpoints(np.eye(4), self.payload, self.report)
        self.assertEqual(result["rms_mm"], 1.)
        self.assertEqual(result["p95_mm"], 1.)
        self.assertEqual(result["max_mm"], 1.)

    def test_capture_scan_and_landmark_reuse_are_checked(self):
        for change in ({"capture_id": "other"}, {"source_hashes": {}},
                       {"independent_of_registration": False}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                evaluate_checkpoints(np.eye(4), dict(self.payload, **change), self.report)
        with self.assertRaisesRegex(ValueError, "reuses"):
            evaluate_checkpoints(np.eye(4), self.payload, self.report, self.payload)

    def test_unconfigured_and_missing_evidence_do_not_pass_quality(self):
        self.assertEqual(quality_assessment({}, None, {})["status"], "not_configured")
        self.assertEqual(quality_assessment({}, None, {"max_checkpoint_p95_mm": .5})["status"], "not_evaluable")
        self.assertEqual(quality_assessment({"fitness": .5}, None, {"min_fitness": .9})["status"], "failed")
        self.assertFalse(quality_assessment({"fitness": 1.}, None, {"min_fitness": .9})["tooth_identity_verified"])


class RealReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.scan = asymmetric_cloud()
        points = np.asarray(self.scan.points)
        self.truth = np.eye(4)
        self.truth[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz([.7, -.4, 1.2])
        self.truth[:3, 3] = [10., -7., 150.]
        self.visible = np.flatnonzero(points[:, 2] > -5)
        xyz = transform_points(points[self.visible], self.truth)[None]
        raw = np.rint(xyz[:, :, 2] / .01).astype(np.uint16)
        frame = RGBDFrame(np.full(xyz.shape, 160, np.uint8), raw, .00001, xyz,
                          np.full(xyz.shape, .6), np.ones(raw.shape, bool),
                          {"fx": 100., "fy": 100., "cx": 0., "cy": 0.},
                          {"camera_serial": "TEST-D405", "geometry_method": "synthetic_software_test"})
        self.config = dict(load_real_config(), capture_outlier_neighbors=0)
        self.capture = self.root / "capture"
        self.metadata = save_real_capture(self.capture, frame, "mandibular", self.config)
        self.scan_path = self.root / "scan.ply"
        o3d.io.write_point_cloud(str(self.scan_path), self.scan)
        self.hashes = {"mandibular": file_hash(self.scan_path)}

    def test_automatic_real_replay_without_world_pose_and_static_repeatability(self):
        result = register_saved_capture(self.capture, self.scan_path, self.root / "run")
        registration = Path(result["registration_dir"])
        estimate = np.load(registration / "T_camera_scan_estimated.npy")
        np.testing.assert_allclose(estimate, self.truth, atol=.02)
        report = json.loads((registration / "registration_report.json").read_text())
        self.assertEqual(report["source_hashes"], self.hashes)
        self.assertEqual(result["validation_status"], "not_configured")
        self.assertFalse(report["simulation_only"])
        self.assertFalse(report["base_coordinates_available"])
        self.assertFalse((registration / "T_base_scan_m.npy").exists())
        self.assertFalse(list(registration.glob("*_world_m.csv")))
        self.assertFalse(report["robot_execution_allowed"])
        register_saved_capture(self.capture, self.scan_path, self.root / "second")
        second = self.root / "second/registration"
        with self.assertRaisesRegex(ValueError, "static scene"):
            repeatability_report([registration, second])
        repeated = repeatability_report([registration, second], static_scene=True)
        self.assertEqual(repeated["kind"], "algorithm_or_preprocessing_repeatability")
        self.assertLess(repeated["translation_rms_from_first_mm"], .03)
        with self.assertRaises(FileExistsError):
            register_saved_capture(self.capture, self.scan_path, self.root / "run")

    def test_manual_replay_base_composition_and_checkpoint_cli(self):
        indices = self.visible[:8]
        source = np.asarray(self.scan.points)[indices]
        pairs = {"capture_id": self.metadata["capture_id"], "source_hashes": self.hashes,
                 "source_scan_mm": source.tolist(),
                 "target_camera_mm": transform_points(source, self.truth).tolist()}
        pairs_path = self.root / "pairs.json"
        write_json(pairs_path, pairs)
        calibration, _, base_camera = calibration_samples()
        calibration_path = self.root / "calibration.json"
        write_json(calibration_path, calibrate_eye_to_hand(calibration))
        result = register_saved_capture(self.capture, self.scan_path, self.root / "manual",
            registration_method="manual", landmarks=pairs_path, calibration=calibration_path)
        directory = Path(result["registration_dir"])
        estimate = np.load(directory / "T_camera_scan_estimated.npy")
        estimate_m = estimate.copy()
        estimate_m[:3, 3] /= 1000
        np.testing.assert_allclose(np.load(directory / "T_base_scan_m.npy"), base_camera @ estimate_m)
        self.assertTrue(result["base_coordinates_available"])
        withheld = np.asarray(self.scan.points)[self.visible[-6:]]
        checkpoints = {"capture_id": self.metadata["capture_id"], "source_hashes": self.hashes,
                       "independent_of_registration": True, "source_scan_mm": withheld.tolist(),
                       "target_camera_mm": transform_points(withheld, self.truth).tolist()}
        cp_path, report_path = self.root / "checkpoints.json", self.root / "withheld.json"
        write_json(cp_path, checkpoints)
        self.assertEqual(main(["evaluate", "--registration", str(directory), "--checkpoints", str(cp_path),
                               "--out", str(report_path)]), 0)
        evaluated = json.loads(report_path.read_text())
        self.assertLess(evaluated["checkpoints"]["rms_mm"], .1)
        with self.assertRaises(FileExistsError):
            evaluate_registration(directory, cp_path, output_path=report_path)

    def test_registration_failure_is_reported_without_fabricated_transform(self):
        with patch("perception.registration.automatic.register_clouds_automatic",
                   side_effect=ValueError("No correspondences")):
            with self.assertRaisesRegex(ValueError, "No correspondences"):
                register_saved_capture(self.capture, self.scan_path, self.root / "failed")
        result = json.loads((self.root / "failed/run_report.json").read_text())
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["stage"], "registration")
        self.assertFalse((self.root / "failed/registration/T_camera_scan_estimated.npy").exists())

    def test_modified_registration_transform_is_rejected_by_evaluation(self):
        register_saved_capture(self.capture, self.scan_path, self.root / "run")
        directory = self.root / "run/registration"
        matrix = np.load(directory / "T_camera_scan_estimated.npy")
        matrix[0, 3] += 10
        np.save(directory / "T_camera_scan_estimated.npy", matrix)
        with self.assertRaisesRegex(ValueError, "integrity"):
            evaluate_registration(directory, output_path=self.root / "tampered.json")

    def test_configured_quality_failure_has_nonzero_cli_status_and_keeps_results(self):
        self.config["quality"] = {"max_inlier_rmse_mm": 1e-8}
        config_path = self.root / "config.json"
        write_json(config_path, self.config)
        self.assertEqual(main(["register", "--capture", str(self.capture), "--scan", str(self.scan_path),
                               "--config", str(config_path), "--out", str(self.root / "gated")]), 2)
        result = json.loads((self.root / "gated/run_report.json").read_text())
        self.assertEqual(result["validation_status"], "failed")
        self.assertTrue((self.root / "gated/registration/T_camera_scan_estimated.npy").is_file())


if __name__ == "__main__":
    unittest.main()
