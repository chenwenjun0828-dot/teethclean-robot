"""Shaft-axial TCP +X follows the curve tangent; roll about X remains free."""
import numpy as np

TCP_FRAME_ID = 'shaft_axial_x_radial_y_v1'
AXIS_CONVENTION = 'tcp_shaft_x_along_gingival_tangent'


def unit(vector):
    vector = np.asarray(vector, float)
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError('A direction is zero or non-finite')
    return vector / length


def plane_angle(tangent, normal):
    """Acute line-to-plane angle, fixed by the curve once X=t is enforced."""
    return float(np.degrees(np.arcsin(np.clip(abs(unit(tangent) @ unit(normal)), 0, 1))))


def tool_rotation(tangent, normal, roll_deg):
    """Construct one continuous roll angle about the tangent axis."""
    x, n = unit(tangent), unit(normal)
    projected = n-x*(n@x)
    if np.linalg.norm(projected) < 1e-8:
        return None
    y0 = -unit(projected)
    z0 = np.cross(x, y0)
    c, s = np.cos(np.deg2rad(roll_deg)), np.sin(np.deg2rad(roll_deg))
    y = c*y0+s*z0
    return np.column_stack((x, y, np.cross(x, y)))


def constraint_errors(rotation, tangent, normal):
    tangent_error = np.degrees(np.arccos(np.clip(rotation[:, 0] @ unit(tangent), -1, 1)))
    actual_angle = plane_angle(rotation[:, 0], normal)
    return float(tangent_error), float(abs(actual_angle-plane_angle(tangent, normal)))
