"""Offline UR7e nominal FK; no ROS, simulator or robot connection required."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

PARAMETERS_SOURCE = 'https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/config/ur7e/default_kinematics.yaml'
CHAIN_SOURCE = 'https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/rolling/urdf/ur_macro.xacro'


def pose(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def fk(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('Provide six finite joint angles in radians')
    # Each revolute joint has local axis +Z: parent_T_child = origin @ Rz(q).
    origins = [
        ((0, 0, .1625), (0, 0, 0)),
        ((0, 0, 0), (1.570796327, 0, 0)),
        ((-.425, 0, 0), (0, 0, 0)),
        ((-.3922, 0, .1333), (0, 0, 0)),
        ((0, -.0997, -2.044881182297852e-11), (1.570796327, 0, 0)),
        ((0, .09959999999999999, -2.042830148012698e-11),
         (1.570796326589793, np.pi, np.pi)),
    ]
    # ROS base_link differs from controller base by 180 degrees around Z.
    transform = pose(rpy=(0, 0, np.pi))
    for angle, (xyz, rpy) in zip(q, origins):
        transform = transform @ pose(xyz, rpy) @ pose(rpy=(0, 0, angle))
    return transform


def dh_fk(q):
    # Independent standard DH computation, in controller base coordinates.
    transform = np.eye(4)
    for angle, a, d, alpha in zip(q, [0, -.425, -.3922, 0, 0, 0],
                                [.1625, 0, 0, .1333, .0997, .0996],
                                [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0]):
        transform = transform @ pose(rpy=(0, 0, angle)) @ pose((0, 0, d)) @ pose((a, 0, 0)) @ pose(rpy=(alpha, 0, 0))
    return transform


def calculate(joint_observation, output):
    observation = json.loads(Path(joint_observation).read_text(encoding='utf-8-sig'))
    q = np.deg2rad(observation['joint_positions_deg'])
    wrist = fk(q)
    wrist_to_tool0 = pose(rpy=(0, -np.pi/2, -np.pi/2)) @ pose(rpy=(np.pi/2, 0, np.pi/2))
    base_to_base_link = pose(rpy=(0, 0, -np.pi))
    controller_wrist = base_to_base_link @ wrist
    controller_tool0 = controller_wrist @ wrist_to_tool0
    # Joint wrapping must not change the Cartesian transform.
    np.testing.assert_allclose(fk(q + 2*np.pi*np.array([1, -1, 0, 1, 0, -1])), wrist, atol=1e-10)
    # Verify the URDF chain against an independent standard DH chain.
    np.testing.assert_allclose(controller_tool0, dh_fk(q), atol=1e-8)
    out = Path(output).resolve()
    if out.exists() and any(out.iterdir()):
        raise ValueError('Use a new empty FK output directory')
    out.mkdir(parents=True, exist_ok=True)
    matrices = dict(T_base_link_wrist3_nominal_m=wrist,
                    T_controller_base_wrist3_nominal_m=controller_wrist,
                    T_controller_base_tool0_nominal_m=controller_tool0)
    for name, matrix in matrices.items():
        np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(matrix[:3, :3]), 1.)
        np.save(out / (name + '.npy'), matrix)
        np.savetxt(out / (name + '.txt'), matrix, fmt='%.12f', header='NOMINAL MODEL; translation=m; ' + name)
    report = dict(robot='UR7e', status='nominal_model_only', robot_execution_allowed=False,
                  factory_calibration_applied=False, translation_unit='m',
                  joint_positions_deg=observation['joint_positions_deg'],
                  joint_positions_rad=q.tolist(), sources=[PARAMETERS_SOURCE, CHAIN_SOURCE],
                  matrix_convention='p_parent = R @ p_child + t',
                  T_base_link_wrist3_nominal_m=wrist.tolist(),
                  T_controller_base_wrist3_nominal_m=controller_wrist.tolist(),
                  T_controller_base_tool0_nominal_m=controller_tool0.tolist(),
                  wrist_position_in_base_link_mm=(1000*wrist[:3, 3]).tolist(),
                  wrist_quaternion_xyzw_in_base_link=Rotation.from_matrix(wrist[:3, :3]).as_quat().tolist(),
                  controller_tool0_pose_xyz_m_rotvec_rad=np.r_[controller_tool0[:3, 3], Rotation.from_matrix(controller_tool0[:3, :3]).as_rotvec()].tolist(),
                  checks=['URDF FK agrees with independent standard DH FK within 1e-8',
                          '2pi joint wrapping invariant', 'proper orthonormal rotations'],
                  note='No active TCP offset applied. Controller base and ROS base_link differ by Rz(pi). Capture-time alignment depends on robot remaining stationary.')
    (out / 'fk_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--joint-observation', type=Path, required=True,
                        help='JSON containing six joint_positions_deg in base/shoulder/elbow/wrist order')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    calculate(args.joint_observation, args.out)


if __name__ == '__main__':
    main()
