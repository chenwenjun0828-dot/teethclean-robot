"""C2 scalar timing of a fixed joint spline, with local speed reductions."""
import numpy as np
from scipy.interpolate import PPoly
from .fixed_spline import JointSpline, _bernstein_matrix, _derivative_bound


class LocalSpeedTrace(JointSpline):
    version = 2

    def __init__(self, *args, clock_u, clock_t, clock_coefficients, minimum_duration_s, **kwargs):
        kwargs.pop('ramp_fraction', None)  # Unused field in earlier version-2 exports.
        super().__init__(*args, **kwargs)
        self.clock_u = np.asarray(clock_u, float)
        self.clock_t = np.asarray(clock_t, float)
        self.clock_c = np.asarray(clock_coefficients, float)
        self.minimum_duration_s = float(minimum_duration_s)
        if (self.clock_c.shape != (len(self.clock_t)-1, 6)
                or self.clock_u.shape != self.clock_t.shape
                or not all(np.isfinite(a).all() for a in (self.clock_u, self.clock_t, self.clock_c))
                or np.any(np.diff(self.clock_t) <= 0) or np.any(np.diff(self.clock_u) <= 0)
                or not np.allclose(self.clock_t[[0, -1]], [0., 1.], atol=1e-12)
                or not np.allclose(self.clock_u[[0, -1]], self.knots[[0, -1]], atol=1e-12)
                or not np.isfinite(self.minimum_duration_s) or self.minimum_duration_s <= 0):
            raise ValueError('Invalid local-speed clock')
        if (not np.allclose(self.clock_c[:, 0], self.clock_u[:-1], atol=1e-12, rtol=0)
                or not np.allclose(self.clock_c.sum(1), self.clock_u[1:], atol=1e-12, rtol=0)):
            raise ValueError('Local-speed clock endpoints changed')
        derivative = self.clock_c[:, 1:]*np.arange(1, 6)
        if np.any(derivative@_bernstein_matrix(4).T < -1e-12):
            raise ValueError('Local-speed clock must move forward')
        widths = np.diff(self.clock_t)
        self.clock = PPoly((self.clock_c/widths[:, None]**np.arange(6))[:, ::-1].T,
                           self.clock_t)
        v0 = self.clock_c[:, 1]/widths
        v1 = derivative.sum(1)/widths
        a0 = 2*self.clock_c[:, 2]/widths**2
        a1 = (self.clock_c*(np.arange(6)*(np.arange(6)-1))).sum(1)/widths**2
        if (not np.allclose(v0[1:], v1[:-1], atol=1e-8, rtol=1e-7)
                or not np.allclose(a0[1:], a1[:-1], atol=1e-5, rtol=1e-7)
                or not np.allclose([v0[0], v1[-1], a0[0], a1[-1]], 0., atol=1e-6)):
            raise ValueError('Local-speed clock must be C2 with stopped endpoints')
        # Every scalar-clock cell must remain inside one original Q(u) cell.
        labels = np.clip(np.searchsorted(self.knots, (self.clock_u[:-1]+self.clock_u[1:])/2)-1,
                         0, len(self.knots)-2)
        if (np.any(self.clock_u[:-1] < self.knots[labels]-1e-12)
                or np.any(self.clock_u[1:] > self.knots[labels+1]+1e-12)):
            raise ValueError('Local-speed clock crosses an unsplit joint-spline knot')
        self._clock_labels = labels
        self._clock_bounds = None

    @classmethod
    def retime(cls, trace, robot, cfg):
        import toppra as ta
        from toppra.constraint import JointAccelerationConstraint, LinearConstraint

        length = trace.knots[-1]
        # The stored spline and scalar clock use metres; ECOS uses a scaled
        # parameter to keep small dental-path derivatives numerically tractable.
        class FixedPath(ta.interpolator.AbstractGeometricPath):
            dof = trace.joints.shape[1]

            def __init__(self, scale):
                self.scale = scale

            @property
            def path_interval(self):
                return np.array([0., length*self.scale])

            def __call__(self, u, order=0):
                return trace.path(np.asarray(u)/self.scale, order)/self.scale**order

        class RateCaps(LinearConstraint):
            def __init__(self):
                super().__init__()
                self._format_string = 'TCP, joint and angular rate caps'

            def compute_constraint_params(self, path, grid):
                q, qu = path(grid), path(grid, 1)
                tcp = np.array([np.linalg.norm(robot.jacobian(a)[:3]@b) for a, b in zip(q, qu)])
                cap = np.minimum.reduce([
                    cfg.joint_speed_rad_s/np.maximum(abs(qu).max(axis=1), 1e-12),
                    np.deg2rad(cfg.orientation_speed_deg_s)/np.maximum(abs(qu).sum(axis=1), 1e-12),
                    cfg.speed_mm_s*.001/np.maximum(tcp, 1e-12)])
                return None, None, None, None, None, None, np.c_[np.zeros(len(grid)), cap**2]

        # This is a timing grid on the existing curve, not additional poses or IK.
        timing_u = np.r_[np.concatenate([np.linspace(a, b, 4, endpoint=False)
                        for a, b in zip(trace.knots[:-1], trace.knots[1:])]), length]
        # ECOS can stall on almost constant-orientation paths. Retry equivalent
        # parameter units, preserving Q(u), all constraints and the timing grid.
        for parameter_scale in (1000., 100.):
            grid = timing_u*parameter_scale
            path = FixedPath(parameter_scale)
            acceleration = JointAccelerationConstraint(np.full(path.dof, cfg.joint_accel_rad_s2),
                            discretization_scheme=ta.constraint.DiscretizationType.Interpolation)
            # ECOS supports Windows/NumPy 1.x without Seidel's integer ABI issue.
            solver = ta.algorithm.TOPPRA([RateCaps(), acceleration], path, gridpoints=grid,
                                        solver_wrapper='ecos', parametrizer='ParametrizeConstAccel')
            _, velocity, _ = solver.compute_parameterization(0., 0.)
            if velocity is not None and np.isfinite(velocity).all() and np.all(velocity >= 0):
                break
        else:
            raise RuntimeError('TOPPRA could not time the fixed joint path '
                               f'(ECOS; {len(grid)} grid points; '
                               f'return code: {solver.problem_data.return_code.name}; '
                               'parameter scales: 1000, 100)')
        dt = 2*np.diff(grid)/(velocity[:-1]+velocity[1:])
        if not np.isfinite(dt).all() or np.any(dt <= 0):
            raise RuntimeError('TOPPRA produced a stationary timing interval')
        times = np.r_[0., np.cumsum(dt)]
        # Smooth only the scalar clock. Zero acceleration at timing knots gives
        # C2 joints while preserving Q(u) and every input pose exactly.
        # Form normalized quintic Hermite cells from local displacements. Using
        # absolute positions here loses C2 accuracy on very short timing cells.
        delta = np.diff(timing_u)
        left = velocity[:-1]/parameter_scale*dt
        right = velocity[1:]/parameter_scale*dt
        c = np.c_[timing_u[:-1], left, np.zeros(len(dt)),
                  10*delta-6*left-4*right,
                  -15*delta+8*left+7*right,
                  6*delta-3*left-3*right]
        record = trace.record()
        record.pop('version')
        for name in ('clock_u', 'clock_t', 'clock_coefficients', 'minimum_duration_s'):
            record.pop(name, None)
        result = cls(**record, clock_u=timing_u, clock_t=times/times[-1],
                     clock_coefficients=c, minimum_duration_s=times[-1])
        v, a = result.rate_bounds(times[-1])
        factor = max(1., v.max()/cfg.joint_speed_rad_s,
                     np.sqrt(a.max()/cfg.joint_accel_rad_s2),
                     np.degrees(v.sum(axis=1)).max()/cfg.orientation_speed_deg_s)
        # Independently probe actual Cartesian speed after smoothing the clock.
        measured = 0.
        for left, right in zip(result.clock_t[:-1], result.clock_t[1:]):
            u = np.linspace(left, right, 9)
            parameter, rate = result.clock(u), result.clock(u, 1)/times[-1]
            for q, qu, speed in zip(trace.path(parameter), trace.path(parameter, 1), rate):
                measured = max(measured, np.linalg.norm(robot.jacobian(q)[:3]@qu)*speed*1000)
        factor = max(factor, measured/cfg.speed_mm_s)*1.005
        result.minimum_duration_s *= factor
        return result

    def fraction_at(self, s):
        return self.fraction_at_u(self.parameter(s)[0])

    def fraction_at_u(self, target):
        target = np.asarray(target, float)
        low, high = np.zeros_like(target), np.ones_like(target)
        for _ in range(46):
            mid = (low+high)/2
            below = self.clock(mid) < target
            low, high = np.where(below, mid, low), np.where(below, high, mid)
        return np.where(target <= 0, 0., np.where(target >= self.knots[-1], 1., (low+high)/2))

    def sample(self, elapsed, duration):
        fraction = np.clip(elapsed/duration, 0., 1.)
        u, ud, udd = self.clock(fraction), self.clock(fraction, 1)/duration, self.clock(fraction, 2)/duration**2
        return (self.path(u), self.path(u, 1)*ud,
                self.path(u, 2)*ud**2+self.path(u, 1)*udd)

    def rate_bounds(self, duration):
        if self._clock_bounds is None:
            c = self.clock_c.copy()
            c[:, 0] = 0.
            powers = np.zeros((len(c), 16, 4))
            powers[:, 0, 0] = 1.
            powers[:, :6, 1] = c
            for k in (2, 3):
                for j in range(6):
                    powers[:, j:j+11, k] += c[:, j, None]*powers[:, :11, k-1]
            # Use each cell's own cubic at boundaries, including its third
            # derivative, which can jump at an original natural-spline knot.
            offset = self.clock_u[:-1]-self.knots[self._clock_labels]
            cubic = self.path.c[:, self._clock_labels]
            taylor = np.stack([
                ((cubic[0]*offset[:, None]+cubic[1])*offset[:, None]+cubic[2])*offset[:, None]+cubic[3],
                (3*cubic[0]*offset[:, None]+2*cubic[1])*offset[:, None]+cubic[2],
                3*cubic[0]*offset[:, None]+cubic[1], cubic[0]], axis=1)
            composed = np.einsum('npk,nkj->npj', powers, taylor)
            width = np.diff(self.clock_t)[:, None]
            v, a = _derivative_bound(composed, 1)/width, _derivative_bound(composed, 2)/width**2
            shape = (len(self.knots)-1, self.joints.shape[1])
            velocity, acceleration = np.zeros(shape), np.zeros(shape)
            np.maximum.at(velocity, self._clock_labels, v)
            np.maximum.at(acceleration, self._clock_labels, a)
            self._clock_bounds = velocity, acceleration
        v, a = self._clock_bounds
        return v/duration, a/duration**2

    def record(self):
        return dict(super().record(), clock_u=self.clock_u.tolist(), clock_t=self.clock_t.tolist(),
                    clock_coefficients=self.clock_c.tolist(), minimum_duration_s=self.minimum_duration_s)
