"""Read D405 depth/color optical extrinsics and derive the standard ROS-frame candidate."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import sys

import numpy as np

from .realsense import RealSenseCamera, _intrinsics
from ..frames import validate_transform
from ..global_camera import PROJECT_ROOT, ensure_empty_directory, write_json

SDK_DOC = 'https://dev.realsenseai.com/docs/projection-in-realsense-sdk-2-0/'
ROS_SOURCE = ('https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/'
              'realsense2_camera/src/base_realsense_node.cpp')


def build_report(extrinsics, device):
    """Use SDK column-major rotation; all matrices map named source to target."""
    rotation = np.asarray(extrinsics.rotation, dtype=float)
    translation = np.asarray(extrinsics.translation, dtype=float)
    if rotation.shape != (9,) or translation.shape != (3,):
        raise ValueError('SDK extrinsics must contain 9 rotation and 3 translation values')
    optical = np.eye(4)
    optical[:3, :3] = rotation.reshape(3, 3, order='F')
    optical[:3, 3] = translation
    validate_transform(optical)
    # realsense-ros: optical child of stream frame, RPY=(-pi/2, 0, -pi/2).
    ros_from_optical = np.array([[0., 0., 1., 0.], [-1., 0., 0., 0.],
                                 [0., -1., 0., 0.], [0., 0., 0., 1.]])
    candidate = validate_transform(ros_from_optical @ optical)
    try:
        version = importlib.metadata.version('pyrealsense2')
    except importlib.metadata.PackageNotFoundError:
        version = None
    return dict(
        schema_version=1, recorded_at_utc=datetime.now(timezone.utc).isoformat(),
        device=dict(device), sdk_version=version, translation_unit='m',
        matrix_convention='p_target = R @ p_source + t',
        sdk_optical_axes='x right, y down, z forward',
        ros_stream_frame_axes='x forward, y left, z up',
        raw_sdk_extrinsics=dict(rotation_column_major=rotation.tolist(),
                                translation_m=translation.tolist()),
        T_color_optical_depth_optical_m=optical.tolist(),
        T_depth_optical_color_optical_m=np.linalg.inv(optical).tolist(),
        T_color_frame_color_optical_ros_convention_m=ros_from_optical.tolist(),
        T_color_frame_depth_optical_ros_convention_m=candidate.tolist(),
        sdk_extrinsics_source='device SDK get_extrinsics_to(color_profile)',
        ros_frame_status='standard_realsense_ros_convention_not_verified_against_live_tf',
        ros_frame_names_are_assumed=True, base_coordinates_available=False,
        notes=['This is not a camera-to-robot-base calibration.',
               'Verify camera_color_frame and camera_depth_optical_frame on the calibration ROS machine.',
               'Confirm the same camera serial and unchanged mounting were used for hand-eye calibration.'],
        sources=[SDK_DOC, ROS_SOURCE])


def read_camera_extrinsics(*, serial=None, config=None):
    configuration = Path(config) if config is not None else PROJECT_ROOT / 'config/real_camera_registration.json'
    options = dict(json.loads(configuration.read_text(encoding='utf-8-sig'))['camera'])
    if serial is not None:
        options['serial'] = serial
    options['warmup_frames'] = 0
    # No image acquisition or GUI; reuse existing D405 selection and pipeline cleanup.
    with RealSenseCamera(options) as camera:
        extrinsics = camera.depth_profile.get_extrinsics_to(camera.color_profile)
        report = build_report(extrinsics, camera.device_info)
        # Check direction and storage layout against the SDK's point transform.
        transform = np.asarray(report['T_color_optical_depth_optical_m'])
        for point in ([0., 0., 0.], [.03, -.02, .2]):
            sdk_point = camera.rs.rs2_transform_point_to_point(extrinsics, point)
            if not np.allclose(transform[:3, :3] @ point + transform[:3, 3], sdk_point,
                               atol=1e-6, rtol=1e-6):
                raise ValueError('Matrix disagrees with SDK point transform')
        report['sdk_point_transform_check'] = 'passed'
        report['depth_intrinsics'] = _intrinsics(camera.depth_profile)
        report['color_intrinsics'] = _intrinsics(camera.color_profile)
        report['camera_options'] = options
    return report


def save_report(report, output_dir):
    out = ensure_empty_directory(output_dir)
    for key, value in report.items():
        if key.startswith('T_'):
            matrix = validate_transform(value)
            np.save(out / (key + '.npy'), matrix)
            np.savetxt(out / (key + '.txt'), matrix, fmt='%.12f',
                       header=key + '; translation=m; p_target=T@p_source')
    write_json(out / 'camera_extrinsics.json', report)
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', help='Select one D405 when multiple cameras are connected')
    parser.add_argument('--config', type=Path, help='Camera stream settings JSON')
    parser.add_argument('--out', type=Path, help='New or empty output directory')
    args = parser.parse_args(argv)
    try:
        report = read_camera_extrinsics(serial=args.serial, config=args.config)
        out = args.out or PROJECT_ROOT / 'outputs/perception/camera_extrinsics' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        saved = save_report(report, out)
        print('Device:', report['device'])
        print('Depth optical -> color optical (SDK, translation in m):')
        print(np.array(report['T_color_optical_depth_optical_m']))
        print('Depth optical -> camera_color_frame (standard ROS convention, not live-TF verified):')
        print(np.array(report['T_color_frame_depth_optical_ros_convention_m']))
        print('Saved:', saved)
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, ImportError) as error:
        print(f'Camera extrinsics read failed: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
