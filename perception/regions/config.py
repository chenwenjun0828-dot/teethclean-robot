"""Perception-owned region names and curve construction parameters."""
from dataclasses import dataclass, asdict
import numpy as np


REGION_ORDER = ('outer_left', 'outer_front', 'outer_right',
                'inner_right', 'inner_front', 'inner_left')
REGIONS = REGION_ORDER


def continuous_path_kind(region):
    if region not in REGIONS:
        raise ValueError(f'Unknown gingival region: {region}')
    return f'continuous_{region}_sweep'


@dataclass
class RegionConfig:
    spacing_mm: float = .25
    fit_tolerance_mm: float = .15
    plane_band_mm: float = 5.
    plane_rms_limit_mm: float = .8
    assignment_max_distance_mm: float = 2.
    min_stroke_mm: float = 1.5
    min_tooth_coverage: float = .25
    region: str = 'outer_front'
    planning_mode: str = 'simple'

    def __post_init__(self):
        continuous_path_kind(self.region)
        for name, value in asdict(self).items():
            if name in ('region', 'planning_mode'):
                continue
            if isinstance(value, (bool, np.bool_)) or not np.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if self.min_tooth_coverage > 1:
            raise ValueError('min_tooth_coverage must not exceed 1')

    @classmethod
    def from_config(cls, cfg):
        return cls(**{name: getattr(cfg, name) for name in cls.__dataclass_fields__})


@dataclass
class SweepOptions:
    trim_mm: float = 1.
    fixed_roll_deg: float = 0.

    def __post_init__(self):
        for name, value in asdict(self).items():
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)) or not np.isfinite(value):
                raise ValueError(f'{name} must be finite and numeric')
            if name == 'fixed_roll_deg':
                if abs(value) > 180:
                    raise ValueError('fixed_roll_deg must be within +/-180 degrees')
            elif value <= 0:
                raise ValueError(f'{name} must be positive')

    @classmethod
    def from_saved(cls, values):
        values = dict(values)
        if values.pop('roll_policy', None) is not None or values.get('fixed_roll_deg') is None:
            raise ValueError('Only fixed-angle continuous plans are supported; replan')
        values.pop('roll_step_deg', None)
        for name in ('max_trim_mm', 'trim_step_mm', 'max_bridge_mm', 'principal_min_mm'):
            values.pop(name, None)
        return cls(**values)
