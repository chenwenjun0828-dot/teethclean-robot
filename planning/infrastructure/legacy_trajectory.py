"""Read-only playback support for previously exported version-1 cruise clocks."""
import numpy as np
from .fixed_spline import JointSpline, _derivative_bound


class ConstantTCPTrace(JointSpline):
    version = 1

    def __init__(self, *args, ramp_fraction=.02, **kwargs):
        super().__init__(*args, **kwargs)
        self.ramp_fraction = float(ramp_fraction)
        if not 0 < self.ramp_fraction < .5:
            raise ValueError("Invalid legacy cruise ramp")
        self._arc_rate_bounds = None

    def arc_at(self, u):
        u = np.clip(np.asarray(u, float), self.knots[0], self.knots[-1])
        i = np.clip(np.searchsorted(self.arc_u, u, side='right')-1, 0, len(self.inverse)-1)
        w = (u-self.arc_u[i])/np.diff(self.arc_u)[i]
        ds = np.diff(self.arc_s)[i]
        for _ in range(8):
            s = self.arc_s[i]+w*ds
            value, derivative, _ = self.parameter(s)
            w = np.clip(w-(value-u)/(derivative*ds), 0., 1.)
        return self.arc_s[i]+w*ds

    def progress(self, fraction):
        x = np.clip(np.asarray(fraction, float), 0., 1.)
        r = self.ramp_fraction
        y = np.minimum(x, 1-x)/r
        z = np.clip(y, 0., 1.)
        ramp = r*(z**3-.5*z**4)
        position = np.where(x < r, ramp, np.where(x > 1-r, 1-r-ramp, x-r/2))/(1-r)
        velocity = np.where(y < 1, 3*z*z-2*z**3, 1.)/(1-r)
        acceleration = np.where(y < 1, (6*z-6*z*z)/r, 0.)/(1-r)*np.where(x <= .5, 1., -1.)
        return position*self.length, velocity*self.length, acceleration*self.length

    def fraction_at(self, s):
        target = np.asarray(s, float)
        low, high = np.zeros_like(target), np.ones_like(target)
        for _ in range(46):
            mid = (low+high)/2
            below = self.progress(mid)[0] < target
            low, high = np.where(below, mid, low), np.where(below, high, mid)
        return np.where(target <= 0, 0., np.where(target >= self.length, 1., (low+high)/2))

    def sample(self, elapsed, duration):
        s, sd, sdd = self.progress(elapsed/duration)
        u, us, uss = self.parameter(s)
        ud, udd = us*sd/duration, (uss*sd*sd+us*sdd)/duration**2
        q, qu, quu = self.path(u), self.path(u, 1), self.path(u, 2)
        return q, qu*ud, quu*ud*ud+qu*udd

    def rate_bounds(self, duration):
        if self._arc_rate_bounds is None:
            # Compose cubic Q with each quintic U. Bernstein bounds include all
            # interior values and retain cancellation in d2Q/ds2.
            ds = np.diff(self.arc_s)
            c = self.inverse.copy(); c[:, 0] = 0.
            powers = np.zeros((len(c), 16, 4))
            powers[:, 0, 0] = 1.
            powers[:, :6, 1] = c
            for k in (2, 3):
                for j in range(6):
                    powers[:, j:j+11, k] += c[:, j, None]*powers[:, :11, k-1]
            taylor = np.stack([self.path(self.arc_u[:-1], k)/[1, 1, 2, 6][k] for k in range(4)], axis=1)
            composed = np.einsum('npk,nkj->npj', powers, taylor)
            first = _derivative_bound(composed, 1)/ds[:, None]
            second = _derivative_bound(composed, 2)/ds[:, None]**2
            labels = np.clip(np.searchsorted(self.knots, self.arc_u[:-1]+1e-14, side='right')-1,
                             0, len(self.knots)-2)
            qs, qss = np.zeros((len(self.knots)-1, self.joints.shape[1])), np.zeros((len(self.knots)-1, self.joints.shape[1]))
            np.maximum.at(qs, labels, first); np.maximum.at(qss, labels, second)
            self._arc_rate_bounds = qs, qss
        qs, qss = self._arc_rate_bounds
        speed = self.length/(1-self.ramp_fraction)/duration
        accel = 1.5*speed/(self.ramp_fraction*duration)
        return qs*speed, qss*speed**2+qs*accel

    def record(self):
        return dict(super().record(), ramp_fraction=self.ramp_fraction)
