"""Fixed joint cubic and numerical TCP arc-length mapping, independent of timing."""
from functools import lru_cache
from math import comb
import numpy as np
from scipy.interpolate import BPoly, CubicSpline, PPoly


@lru_cache(None)
def _bernstein_matrix(degree):
    return np.array([[comb(k, j)/comb(degree, j) if j <= k else 0.
                      for j in range(degree+1)] for k in range(degree+1)])


def _derivative_bound(coefficients, order):
    """Conservative polynomial derivative bounds on [0,1], without root searches."""
    c = np.polynomial.polynomial.polyder(coefficients, order, axis=1)
    b = np.einsum('ij,njk->nik', _bernstein_matrix(c.shape[1]-1), c)
    return abs(b).max(axis=1)


class JointSpline:
    version = 0

    def __init__(self, start, knots, joints, arc_u, arc_s, inverse_coefficients):
        self.start = int(start)
        self.knots = np.asarray(knots, float)
        self.joints = np.asarray(joints, float)
        self.arc_u, self.arc_s = np.asarray(arc_u, float), np.asarray(arc_s, float)
        self.inverse = np.asarray(inverse_coefficients, float)
        if (self.start < 0 or len(self.knots) < 2 or self.joints.ndim != 2
                or len(self.joints) != len(self.knots)
                or len(self.arc_u) != len(self.arc_s) or len(self.arc_u) < 2
                or self.inverse.shape != (len(self.arc_s)-1, 6)
                or not all(np.isfinite(a).all() for a in
                           (self.knots, self.joints, self.arc_u, self.arc_s, self.inverse))
                or any(np.any(np.diff(a) <= 0) for a in (self.knots, self.arc_u, self.arc_s))
                or self.knots[0] != 0 or self.arc_s[0] != 0):
            raise ValueError('Invalid joint spline or arc-length map')
        if not np.allclose(self.inverse[:, 0], self.arc_u[:-1], atol=1e-12, rtol=0) or not np.allclose(
                self.inverse.sum(axis=1), self.arc_u[1:], atol=1e-12, rtol=0):
            raise ValueError('Arc-length inverse endpoints changed')
        self.path = CubicSpline(self.knots, self.joints, bc_type='natural')
        nodes = np.searchsorted(self.arc_u, self.knots)
        if np.any(nodes >= len(self.arc_u)) or not np.array_equal(self.arc_u[nodes], self.knots):
            raise ValueError('Arc-length table must retain every input spline knot')
        self.node_s = self.arc_s[nodes]

    @property
    def end(self):
        return self.start+len(self.knots)-1

    @property
    def length(self):
        return float(self.arc_s[-1])

    @classmethod
    def build(cls, joints, robot, start=0):
        joints = np.asarray(joints, float)
        if joints.ndim != 2 or len(joints) < 2 or not np.isfinite(joints).all():
            raise ValueError('Expected at least two finite joint states')
        positions = np.array([robot.fk(q)[:3, 3].copy() for q in joints])
        chord = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        if np.any(chord <= 1e-10):
            raise ValueError('Joint spline requires distinct consecutive TCP positions; '
                             f'check input interval {int(np.argmin(chord))}')
        knots = np.r_[0., np.cumsum(chord)]
        path = CubicSpline(knots, joints, bc_type='natural')

        speed_cache = {}
        def speed(u):
            q, qu = path(u), path(u, 1)
            result = []
            for x, a, b in zip(np.atleast_1d(u), np.atleast_2d(q), np.atleast_2d(qu)):
                if float(x) not in speed_cache:
                    speed_cache[float(x)] = np.linalg.norm(robot.jacobian(a)[:3]@b)
                result.append(speed_cache[float(x)])
            return np.asarray(result)

        # Only the numerical arc table is refined; input IK nodes and Q(u) stay fixed.
        gauss, weights = np.polynomial.legendre.leggauss(8)
        arc_u = np.r_[np.concatenate([np.linspace(a, b, 4, endpoint=False)
                                     for a, b in zip(knots[:-1], knots[1:])]), knots[-1]]
        for _ in range(12):
            du = np.diff(arc_u)
            grid = (arc_u[:-1, None]+du[:, None]*(gauss+1)/2).ravel()
            lengths = speed(grid).reshape(-1, len(gauss))@weights*du/2
            g = speed(arc_u)
            if np.any(g <= 1e-9) or np.any(lengths <= 0):
                raise ValueError('Joint spline has stationary TCP translation; revise the supplied poses')
            h = max(1e-10, float(du.min())*1e-3)
            dg = (speed(arc_u+h)-speed(arc_u-h))/(2*h)
            arc_s = np.r_[0., np.cumsum(lengths)]
            data = np.c_[arc_u, 1/g, -dg/g**3]
            inverse = PPoly.from_bernstein_basis(BPoly.from_derivatives(arc_s, data))
            c = inverse.c[::-1].T*np.diff(arc_s)[:, None]**np.arange(6)
            result = cls(start, knots, joints, arc_u, arc_s, c)
            probe_s = (arc_s[:-1, None]+np.diff(arc_s)[:, None]*np.linspace(0, 1, 9)).ravel()
            u, us, _ = result.parameter(probe_s)
            errors = abs(speed(u)*us-1).reshape(-1, 9).max(axis=1)
            error = errors.max()
            # Bernstein coefficients certify positivity of the inverse derivative.
            derivative = c[:, 1:]*np.arange(1, 6)
            bad = (errors > .002) | np.any(derivative@_bernstein_matrix(4).T <= 0, axis=1)
            if not bad.any():
                result.arc_speed_error = float(error)
                return result
            arc_u = np.sort(np.r_[arc_u, (arc_u[:-1][bad]+arc_u[1:][bad])/2])
        raise ValueError(f'TCP arc-length inverse did not meet its speed tolerance ({error:.3g})')

    def parameter(self, s):
        s = np.clip(np.asarray(s, float), 0., self.length)
        i = np.clip(np.searchsorted(self.arc_s, s, side='right')-1, 0, len(self.inverse)-1)
        ds = np.diff(self.arc_s)[i]
        w = (s-self.arc_s[i])/ds
        c = self.inverse[i]
        values = []
        for order in range(3):
            coefficients = c.copy()
            for _ in range(order):
                coefficients = coefficients[..., 1:]*np.arange(1, coefficients.shape[-1])
            value = np.zeros_like(w)
            for column in np.moveaxis(coefficients, -1, 0)[::-1]:
                value = value*w+column
            if order == 0:
                # Preserve exact path endpoints before inverting a stopped
                # time clock, where roundoff otherwise amplifies into time drift.
                value = np.where(s <= 0., self.knots[0],
                                 np.where(s >= self.length, self.knots[-1], value))
            values.append(value/ds**order)
        return tuple(values)

    def record(self):
        return dict(version=self.version, start=self.start, knots=self.knots.tolist(), joints=self.joints.tolist(),
                    arc_u=self.arc_u.tolist(), arc_s=self.arc_s.tolist(), inverse_coefficients=self.inverse.tolist())

    @classmethod
    def from_record(cls, record):
        values = dict(record)
        if values.pop('version', None) != cls.version:
            raise ValueError('Unsupported fixed joint trajectory version')
        return cls(**values)
