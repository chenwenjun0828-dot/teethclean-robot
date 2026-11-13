"""Fixed gravity-plane Z/Y rotations followed by a tool-local +X flip."""
import numpy as np
from scipy.spatial.transform import Rotation
from planning.infrastructure.poses import unit, TCP_FRAME_ID
from planning.infrastructure.io import validate_transform

AXIS_CONVENTION = 'gravity_fixed_z_y_tool_x_y_down_v3'
SIMPLE_AXIS_CONVENTION = 'regional_gravity_fixed_z_y_tool_x_y_down_v3'
WORLD_GRAVITY = (0., 0., -1.)


def axis_convention(cfg):
    return SIMPLE_AXIS_CONVENTION if cfg.planning_mode == 'simple' else AXIS_CONVENTION


def surface_frame(tangent, normal):
    n = unit(normal)
    t = unit(np.asarray(tangent)-n*np.dot(tangent, n))
    return np.column_stack((t, np.cross(n, t), n))


def gravity_frame(inward_normal, gravity=WORLD_GRAVITY):
    n, g = unit(inward_normal), unit(gravity)
    projected = g-n*(g@n)
    if np.linalg.norm(projected) < 1e-8:
        raise ValueError('Gravity projection onto the tooth plane is zero; choose another reference point/normal')
    z = unit(projected)
    # Fixed rotation Z is zero-pose tool Y; fixed rotation Y is zero-pose tool Z.
    return np.column_stack((n, z, np.cross(n, z)))


def tool_rotation(tangent, normal, tilt_deg=15., roll_deg=0., tilt_azimuth_deg=0., *, gravity=WORLD_GRAVITY):
    """Argument names retain compatibility: fixed-Z angle, fixed-Y azimuth, X flip.

    `normal` remains outward for geometry/offset compatibility; tangent is unused.
    """
    values = (tilt_deg, tilt_azimuth_deg, roll_deg)
    if any(isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.integer, np.floating))
           or not np.isfinite(v) or abs(v) > 180 for v in values):
        raise ValueError('angle/tilt_deg, azimuth/tilt_azimuth_deg and flip/roll_deg must be finite and within +/-180 degrees')
    frame = gravity_frame(-np.asarray(normal), gravity)
    local = Rotation.from_euler('xyz', [roll_deg, tilt_deg, tilt_azimuth_deg], degrees=True).as_matrix()
    return frame@local


def regional_angle_offsets(rotation, frame, base_angles):
    """Recover equivalent Z/Y/X angles nearest the original regional tuple."""
    local = frame.T@rotation
    cosine = np.hypot(local[2, 1], local[2, 2])
    alpha = np.degrees(np.arctan2(-local[2, 0], cosine))
    base = np.asarray(base_angles, float)
    wrap = lambda v: (np.asarray(v)+180)%360-180
    if cosine < 1e-8:
        sign = 1. if alpha >= 0 else -1.
        coupled = np.degrees(np.arctan2(-local[0, 1], local[1, 1]))
        residual = float(wrap(coupled-(base[1]-sign*base[2])))
        return np.array([float(wrap(alpha-base[0])), residual/2, -sign*residual/2])
    beta = np.degrees(np.arctan2(local[1, 0], local[0, 0]))
    gamma = np.degrees(np.arctan2(local[2, 1], local[2, 2]))
    candidates = [wrap(np.array([alpha, beta, gamma])-base),
                  wrap(np.array([180-alpha, beta+180, gamma+180])-base)]
    return min(candidates, key=lambda v: (abs(v).max(), v@v))


def normal_angle(rotation, normal):
    return float(np.degrees(np.arccos(np.clip(-rotation[:, 0]@unit(normal), -1., 1.))))


def pose_at(point, tangent, normal, roll, world, cfg):
    world = validate_transform(world)
    pose = np.eye(4)
    gravity_scan = world[:3, :3].T@np.asarray(cfg.gravity_world)
    pose[:3, :3] = world[:3, :3]@tool_rotation(
        tangent, normal, cfg.tilt_deg, roll, cfg.tilt_azimuth_deg, gravity=gravity_scan)
    pose[:3, 3] = world[:3, :3]@(np.asarray(point)+unit(normal)*cfg.standoff_mm*.001)+world[:3, 3]
    return pose


def target_at(path, s, roll, world, cfg):
    if cfg.planning_mode == 'simple':
        from ..simple import target_at as simple_target
        return simple_target(path, s, roll, world, cfg)
    return pose_at(path.evaluate(s), unit(path.evaluate(s, 1)), path.normal(s), roll, world, cfg)
