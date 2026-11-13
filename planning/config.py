"""Shared path, pose-policy and motion parameters; lengths in millimetres."""
from dataclasses import dataclass, asdict
import numpy as np


from perception.regions.config import REGIONS, SweepOptions, continuous_path_kind
PLANNING_MODES = ('continuous', 'simple')
CONTINUOUS_PATH_KINDS = tuple(f'continuous_{region}_sweep' for region in REGIONS)
# 'full' runs the FCL/MuJoCo collision pipeline end to end. 'geometry_only' skips
# every collision query and only answers "are these poses and paths reachable",
# so its output can never become an executable plan.
CHECK_MODES = ('full', 'geometry_only')
GEOMETRY_ONLY = 'geometry_only'


@dataclass
class Config:
    spacing_mm: float = .25
    pose_spacing_mm: float = .05
    fit_tolerance_mm: float = .15
    plane_band_mm: float = 5.
    plane_rms_limit_mm: float = .8
    assignment_max_distance_mm: float = 2.
    tilt_deg: float = 15.
    standoff_mm: float | None = None
    clearance_mm: float = .15
    min_stroke_mm: float = 1.5
    min_tooth_coverage: float = .25
    joint_step_rad: float = .15
    max_orientation_step_deg: float = 20.
    sweep_step_rad: float = .004
    interpolation_position_mm: float = .1
    interpolation_angle_deg: float = 1.
    joint_speed_rad_s: float = .3
    joint_accel_rad_s2: float = .8
    orientation_speed_deg_s: float = 15.
    speed_mm_s: float = 3.
    retreat_mm: float = 15.
    motion_mode: str | None = None
    transfer_speed_mm_s: float = 20.
    tilt_azimuth_deg: float = 0.
    region: str = 'outer_front'
    speed_profile: str = 'local'
    orientation_audit: str = 'strict'
    planning_mode: str = 'simple'
    simple_angle_deg: float = -75.
    simple_angle2_deg: float = -33.
    check_mode: str = 'full'
    gravity_world: tuple = (0., 0., -1.)

    def __post_init__(self):
        continuous_path_kind(self.region)
        if self.planning_mode not in PLANNING_MODES:
            raise ValueError('planning_mode must be continuous or simple')
        if self.check_mode not in CHECK_MODES:
            raise ValueError('check_mode must be full or geometry_only')
        if self.planning_mode == 'simple' and (self.simple_angle_deg, self.simple_angle2_deg) != (-75., -33.):
            raise ValueError('The old simple two-angle parameters are retired; use pose_angles_deg (angle, azimuth, flip)')
        if self.speed_profile != 'local':
            raise ValueError('Only local TOPPRA timing is supported')
        if self.orientation_audit not in ('strict', 'diagnostic'):
            raise ValueError('orientation_audit must be strict or diagnostic')
        # Infer the mode for older saved configurations and --standoff-mm users.
        if self.motion_mode is None:
            self.motion_mode = 'contact' if self.standoff_mm == 0 else 'non-contact'
        if self.motion_mode not in ('non-contact', 'contact'):
            raise ValueError('motion_mode must be non-contact or contact')
        if self.standoff_mm is None:
            self.standoff_mm = 0. if self.motion_mode == 'contact' else 1.5
        for name, value in asdict(self).items():
            if name == 'gravity_world':
                g = np.asarray(value, float)
                if g.shape != (3,) or not np.isfinite(g).all() or np.linalg.norm(g) < 1e-12:
                    raise ValueError('gravity_world must be a finite nonzero three-vector')
                self.gravity_world = tuple(g/np.linalg.norm(g))
                continue
            if name in ('motion_mode', 'region', 'speed_profile', 'orientation_audit',
                        'planning_mode', 'check_mode'):
                continue
            if name in ('simple_angle_deg', 'simple_angle2_deg'):
                if (isinstance(value, (bool, np.bool_))
                        or not isinstance(value, (int, float, np.integer, np.floating))
                        or not np.isfinite(value)):
                    raise ValueError(f'{name} must be finite')
                continue
            if name == 'tilt_azimuth_deg':
                if (isinstance(value, (bool, np.bool_))
                        or not isinstance(value, (int, float, np.integer, np.floating))
                        or not np.isfinite(value) or abs(value) > 180):
                    raise ValueError('tilt_azimuth_deg must be finite and within +/-180 degrees')
                continue
            if name == 'tilt_deg':
                if (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating))
                        or not np.isfinite(value) or abs(value) > 180):
                    raise ValueError('tilt_deg (fixed-Z angle) must be finite and within +/-180 degrees')
                continue
            if name == 'standoff_mm':
                if isinstance(value, bool) or not np.isfinite(value) or value < 0:
                    raise ValueError('standoff_mm must be nonnegative and finite')
                continue
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.min_tooth_coverage > 1:
            raise ValueError('coverage must be at most 1')
        if self.motion_mode == 'contact':
            if self.standoff_mm != 0:
                raise ValueError('contact mode requires standoff_mm=0')
        elif self.clearance_mm >= self.standoff_mm:
            raise ValueError('Non-contact standoff must exceed clearance')

    @classmethod
    def from_saved(cls, values):
        """Restore configuration; historical timing lives only in the saved trajectory."""
        values = dict(values)
        values.pop('roll_step_deg', None)
        values.setdefault('pose_spacing_mm', values.get('spacing_mm', .25))
        values.setdefault('planning_mode', 'continuous')
        if values.get('speed_profile', 'local') not in ('local', 'constant'):
            raise ValueError('Unsupported saved speed profile')
        values['speed_profile'] = 'local'
        return cls(**values)

    @property
    def geometry_only(self):
        return self.check_mode == GEOMETRY_ONLY

    def motion_description(self):
        """Zero offset is a geometric target, not a contact-force controller."""
        if self.geometry_only:
            policy = ('Collision checking is disabled by check_mode=geometry_only: '
                      'clearance, clearance_mm, transport speed and plan clearance records are '
                      'diagnostic only and this plan is never execution-ready.')
        else:
            policy = ('Full-tool collision rejection remains enabled; '
                      'zero offset does not permit mesh penetration.')
        return {'motion_mode': self.motion_mode, 'standoff_mm': self.standoff_mm,
                'contact_force_control': False, 'contact_policy': policy}


def resolve_pose_angles(pose_angles_deg=None, *, tilt_deg=15., tilt_azimuth_deg=0.,
                        fixed_roll_deg=None):
    """Resolve the fixed-Z angle, fixed-Y azimuth and tool-X flip tuple."""
    if pose_angles_deg is not None:
        if tilt_deg != 15. or tilt_azimuth_deg != 0. or fixed_roll_deg is not None:
            raise ValueError('pose_angles_deg cannot be combined with individual angle overrides')
        try:
            values = tuple(pose_angles_deg)
        except TypeError as error:
            raise ValueError('pose_angles_deg requires three finite numeric angles') from error
        if len(values) != 3 or any(
                isinstance(value, (bool, np.bool_))
                or not isinstance(value, (int, float, np.integer, np.floating))
                or not np.isfinite(value) for value in values):
            raise ValueError('pose_angles_deg requires three finite numeric angles')
        tilt_deg, tilt_azimuth_deg, fixed_roll_deg = map(float, values)
    Config(tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg)
    if fixed_roll_deg is None:
        fixed_roll_deg = 0.
    SweepOptions(fixed_roll_deg=fixed_roll_deg)
    return tilt_deg, tilt_azimuth_deg, fixed_roll_deg


def resolve_simple_angles(values=(-75., -33.)):
    try:
        values = tuple(values)
    except TypeError as error:
        raise ValueError('simple_angles_deg requires two finite numeric angles') from error
    if len(values) != 2 or any(isinstance(v, (bool, np.bool_)) or not isinstance(
            v, (int, float, np.integer, np.floating)) or not np.isfinite(v) for v in values):
        raise ValueError('simple_angles_deg requires two finite numeric angles')
    return tuple(map(float, values))
