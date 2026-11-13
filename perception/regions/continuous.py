"""Sample a continuous regional B-spline without reconstructing source geometry."""
import numpy as np
from scipy.interpolate import BSpline
from .io import unit


class SweepPath:
    def __init__(self, record):
        self.record = record
        self.parts = record['parts']
        self.ends = np.asarray([p['end'] for p in self.parts])
        if any(p['kind'] != 'source' for p in self.parts):
            raise ValueError('Only source-preserving regional curves are supported; rerun perception')
        self.curves = [BSpline(p['knots'], p['coefficients_m'], 3, extrapolate=False)
                       for p in self.parts]
        if not len(self.parts) or not np.isfinite(self.ends).all() or np.any(np.diff(np.r_[0., self.ends]) <= 0):
            raise ValueError('Invalid continuous reference intervals')
        for i, p in enumerate(self.parts):
            if not np.isclose(p['start'], 0. if i == 0 else self.ends[i-1], atol=1e-12):
                raise ValueError('Continuous reference has a gap')

    @property
    def end(self):
        return float(self.ends[-1])

    def part_index(self, s):
        if not np.isfinite(s) or s < -1e-12 or s > self.end+1e-12:
            raise ValueError('Sweep parameter outside reference')
        return min(int(np.searchsorted(self.ends, s, side='right')), len(self.parts)-1)

    def evaluate(self, s, derivative=0):
        i = self.part_index(s)
        p = self.parts[i]
        span = p['end']-p['start']
        f = float(np.clip((s-p['start'])/span, 0., 1.))
        du = p['u1']-p['u0']
        return self.curves[i](p['u0']+f*du, nu=derivative)*(du/span)**derivative

    def normal(self, s):
        p = self.parts[self.part_index(s)]
        f = float(np.clip((s-p['start'])/(p['end']-p['start']), 0., 1.))
        w = f**3*(10-15*f+6*f*f)
        return unit((1-w)*np.asarray(p['normal0'])+w*np.asarray(p['normal1']))

    def tooth(self, s):
        p = self.parts[self.part_index(s)]
        return int(p['tooth0'] if s < (p['start']+p['end'])/2 else p['tooth1'])

    def position(self, s, offset_m=0.):
        return self.evaluate(s)+offset_m*self.normal(s)

    def samples(self, spacing_m):
        # Include every seam and resolve rapid direction changes before IK search.
        result = [0.]
        def refine(a, b, depth=0):
            m = (a+b)/2
            p = np.array([self.evaluate(x) for x in (a, m, b)])
            tangents = [unit(self.evaluate(x, 1)) for x in (a, m, b)]
            turn = min(tangents[0]@tangents[1], tangents[1]@tangents[2])
            if depth < 12 and (np.linalg.norm(np.diff(p, axis=0), axis=1).sum() > spacing_m
                               or turn < np.cos(np.deg2rad(5))):
                refine(a, m, depth+1); refine(m, b, depth+1)
            else:
                result.append(b)
        for part in self.parts:
            count = max(2, int(np.ceil((part['end']-part['start'])/spacing_m)))
            grid = np.linspace(part['start'], part['end'], count+1)
            for a, b in zip(grid[:-1], grid[1:]): refine(a, b)
        return np.asarray(result)
