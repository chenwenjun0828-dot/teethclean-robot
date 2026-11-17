"""Select an explicit interpretation without modifying original calibration evidence."""
import numpy as np

MODES = ('color_optical', 'declared')


def select_handeye_chain(calibration, sdk, mode='color_optical'):
    if mode not in MODES:
        raise ValueError(f'Unknown hand-eye frame interpretation: {mode}')
    handeye = np.asarray(calibration['T_wrist3_camera_color_frame_m'], dtype=float)
    optical = mode == 'color_optical'
    depth = np.asarray(sdk['T_color_optical_depth_optical_m'] if optical else
                       sdk['T_color_frame_depth_optical_ros_convention_m'], dtype=float)
    for matrix in (handeye, depth):
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('Hand-eye and SDK transforms must be finite 4x4 matrices')
        np.testing.assert_allclose(matrix[3], [0, 0, 0, 1], atol=1e-10)
        np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
        if not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6):
            raise ValueError('Hand-eye and SDK rotations must be proper rotations')
    metadata = dict(
        mode=mode, declared_camera_frame='camera_color_frame',
        effective_camera_frame='color_optical' if optical else 'camera_color_frame',
        effective_parent_frame='wrist_3_link', raw_calibration_modified=False,
        ros_optical_to_stream_rotation_applied=not optical,
        evidence=('User confirmed the color_optical result; original calibration TF has not been independently verified.'
                  if optical else 'Uses archived frame declaration and standard ROS axes; historical interpretation.'),
        handeye_matrix_name='T_wrist3_color_optical_m' if optical else 'T_wrist3_color_frame_m',
        depth_matrix_name=('T_color_optical_depth_optical_m' if optical else
                           'T_color_frame_depth_optical_ros_convention_m'),
        formula=('T_base_wrist3 @ T_wrist3_color_optical @ T_color_optical_depth_optical @ T_depth_optical_scan'
                 if optical else 'T_base_wrist3 @ T_wrist3_color_frame @ T_color_frame_depth_optical_ros_convention @ T_depth_optical_scan'))
    return handeye.copy(), depth.copy(), metadata
