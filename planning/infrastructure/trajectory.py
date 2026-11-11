"""Control contract: bounded C2 joint motion with continuous trace blocks."""
from dataclasses import dataclass
from pathlib import Path
import numpy as np


@dataclass
class Trajectory:
    times: np.ndarray
    joints: np.ndarray
    phases: np.ndarray
    teeth: np.ndarray
    joint_names: list
    coefficients: np.ndarray = None
    mount_position_m: np.ndarray = None
    constant_trace: object = None  # Historical name retained for saved-artifact compatibility.

    def __post_init__(self):
        self.times = np.asarray(self.times, float)
        self.joints = np.asarray(self.joints, float)
        self.phases = np.asarray(self.phases, str)
        self.teeth = np.asarray(self.teeth, int)
        if (self.joints.shape != (len(self.times), len(self.joint_names)) or len(self.times) < 2
                or self.phases.shape != (len(self.times)-1,) or self.teeth.shape != self.phases.shape
                or not np.isfinite(self.joints).all() or not np.isfinite(self.times).all()
                or self.times[0] != 0 or np.any(np.diff(self.times) <= 0)):
            raise ValueError('Invalid trajectory arrays or non-increasing time')
        if self.coefficients is not None:
            self.coefficients = np.asarray(self.coefficients, float)
            if self.coefficients.shape != (len(self.times)-1, 6, len(self.joint_names)) \
                    or not np.isfinite(self.coefficients).all():
                raise ValueError('Invalid trajectory interpolation coefficients')

        if self.mount_position_m is not None:
            self.mount_position_m = np.asarray(self.mount_position_m, float)
            if self.mount_position_m.shape != (3,) or not np.isfinite(self.mount_position_m).all():
                raise ValueError('Invalid trajectory mount position')
        if self.coefficients is not None:
            dt = np.diff(self.times)[:, None]
            c = self.coefficients
            if not np.allclose(c[:, 0], self.joints[:-1], atol=1e-9, rtol=0) or not np.allclose(c.sum(1), self.joints[1:], atol=1e-9, rtol=0):
                raise ValueError('Polynomial endpoints do not match trajectory joints')
            v0, v1 = c[:, 1]/dt, (c*np.arange(6)[None, :, None]).sum(1)/dt
            a0 = 2*c[:, 2]/dt**2
            a1 = (c*(np.arange(6)*(np.arange(6)-1))[None, :, None]).sum(1)/dt**2
            if (not np.allclose(v1[:-1], v0[1:], atol=1e-8, rtol=0)
                    or not np.allclose(a1[:-1], a0[1:], atol=1e-8, rtol=0)
                    or not np.allclose([v0[0], v1[-1], a0[0], a1[-1]], 0, atol=1e-8)):
                raise ValueError('Trajectory derivatives are not C2 with stopped endpoints')
        if self.constant_trace is not None:
            trace = self.constant_trace
            if (trace.end >= len(self.times) or np.any(self.phases[trace.start:trace.end] != 'trace')
                    or not np.allclose(self.joints[trace.start:trace.end+1], trace.joints, atol=1e-10, rtol=0)):
                raise ValueError('Fixed joint spline differs from saved trace nodes')
            fractions = trace.fraction_at(trace.node_s)
            expected = self.times[trace.start]+fractions*(self.times[trace.end]-self.times[trace.start])
            if not np.allclose(self.times[trace.start:trace.end+1], expected, atol=1e-8, rtol=0):
                raise ValueError('Trace waypoint times differ from the arc-length clock')

    @property
    def duration(self):
        return float(self.times[-1])

    def sample(self, time_s):
        if not np.isfinite(time_s):
            raise ValueError('Trajectory time must be finite')
        i = int(np.clip(np.searchsorted(self.times, time_s, side='right')-1, 0, len(self.times)-2))
        trace = self.constant_trace
        if trace is not None and trace.start <= i < trace.end:
            q, dq, ddq = trace.sample(time_s-self.times[trace.start],
                                      self.times[trace.end]-self.times[trace.start])
            return {'q': q, 'dq': dq, 'ddq': ddq,
                    'phase': str(self.phases[i]), 'tooth': int(self.teeth[i]), 'interval': i}
        dt = self.times[i+1]-self.times[i]
        u = float(np.clip((time_s-self.times[i])/dt, 0, 1))
        if self.coefficients is None:
            coefficients = self._smoothstep_coefficients(self.joints[i], self.joints[i+1])
        else:
            coefficients = self.coefficients[i]
        powers = np.array([u**k for k in range(6)])[:, None]
        dpowers = np.array([0., 1., 2*u, 3*u**2, 4*u**3, 5*u**4])[:, None]
        ddpowers = np.array([0., 0., 2., 6*u, 12*u**2, 20*u**3])[:, None]
        q = np.sum(coefficients*powers, axis=0)
        dq = np.sum(coefficients*dpowers, axis=0)/dt
        ddq = np.sum(coefficients*ddpowers, axis=0)/dt**2
        return {'q': q, 'dq': dq, 'ddq': ddq,
                'phase': str(self.phases[i]), 'tooth': int(self.teeth[i]), 'interval': i}

    @staticmethod
    def _smoothstep_coefficients(start, end):
        delta = np.asarray(end)-np.asarray(start)
        return np.stack((start, np.zeros_like(delta), np.zeros_like(delta),
                         10*delta, -15*delta, 6*delta))

    @staticmethod
    def _quintic_coefficients(start, end, velocity_start, velocity_end,
                              acceleration_start, acceleration_end, duration):
        start, end = np.asarray(start), np.asarray(end)
        c = np.zeros((6, len(start)), float)
        c[0] = start
        c[1] = duration*velocity_start
        c[2] = .5*duration**2*acceleration_start
        delta = end-start
        rhs = np.stack((delta-c[1]-c[2],
                        duration*velocity_end-c[1]-2*c[2],
                        duration**2*acceleration_end-2*c[2]))
        c[3:] = np.linalg.solve(np.array([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]), rhs)
        return c

    def save(self, path):
        import json
        np.savez_compressed(path, times_s=self.times, joint_positions_rad=self.joints,
                            phases=self.phases, teeth=self.teeth, joint_names=np.asarray(self.joint_names),
                            coefficients=self.coefficients if self.coefficients is not None else np.empty((0,)),
                            mount_position_m=self.mount_position_m if self.mount_position_m is not None else np.empty((0,)),
                            constant_tcp_json=json.dumps(self.constant_trace.record()) if self.constant_trace is not None else '')

    @classmethod
    def load(cls, path):
        import json
        with np.load(Path(path), allow_pickle=False) as d:
            coefficients = d['coefficients'] if 'coefficients' in d and d['coefficients'].size else None
            trace = None
            if 'constant_tcp_json' in d and str(d['constant_tcp_json']):
                record = json.loads(str(d['constant_tcp_json']))
                if record.get('version') == 2:
                    from .local_speed import LocalSpeedTrace
                    trace = LocalSpeedTrace.from_record(record)
                else:
                    from .legacy_trajectory import ConstantTCPTrace
                    trace = ConstantTCPTrace.from_record(record)
            return cls(d['times_s'], d['joint_positions_rad'], d['phases'], d['teeth'],
                       d['joint_names'].tolist(), coefficients,
                       d['mount_position_m'] if 'mount_position_m' in d and d['mount_position_m'].size else None, trace)

    def polynomial(self, interval):
        if self.constant_trace is not None and self.constant_trace.start <= interval < self.constant_trace.end:
            raise ValueError('Fixed joint trace is a composed spline, not a time polynomial')
        return (self.coefficients[interval] if self.coefficients is not None else
                self._smoothstep_coefficients(self.joints[interval], self.joints[interval+1]))

    def position_bounds(self, interval):
        trace = self.constant_trace
        if trace is not None and trace.start <= interval < trace.end:
            j = interval-trace.start
            span = trace.knots[j+1]-trace.knots[j]
            c = trace.path.c[::-1, j]*span**np.arange(4)[:, None]
            return self.extrema(c)
        return self.extrema(self.polynomial(interval))

    @staticmethod
    def extrema(coefficients, derivative=0):
        """Exact polynomial extrema over normalized time [0,1], per joint."""
        lows, highs = [], []
        for column in coefficients.T:
            c = np.polynomial.polynomial.polyder(column, derivative)
            roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(c))
            points = [0., 1.]+[float(x.real) for x in roots if abs(x.imag)<1e-9 and 0<x.real<1]
            values = np.polynomial.polynomial.polyval(points, c)
            lows.append(min(values)); highs.append(max(values))
        return np.asarray(lows), np.asarray(highs)

    def interval_times(self, interval, step_rad=.004):
        trace = self.constant_trace
        if trace is not None and trace.start <= interval < trace.end:
            j = interval-trace.start
            span = trace.knots[j+1]-trace.knots[j]
            c = trace.path.c[::-1, j]*span**np.arange(4)[:, None]
            low, high = self.extrema(c, 1)
            bound = np.maximum(abs(low), abs(high)).max()
            count = max(8, int(np.ceil(bound/step_rad)))
            u = np.linspace(trace.knots[j], trace.knots[j+1], count+1)
            fraction = (trace.fraction_at_u(u) if trace.version == 2
                        else trace.fraction_at(trace.arc_at(u)))
            return self.times[trace.start]+fraction*(self.times[trace.end]-self.times[trace.start])
        dt = self.times[interval+1]-self.times[interval]
        low, high = self.extrema(self.polynomial(interval), 1)
        bound = np.maximum(abs(low), abs(high)).max()
        count = max(8, int(np.ceil(bound/step_rad)))
        return np.linspace(self.times[interval], self.times[interval+1], count+1)

    def limits(self):
        speed, accel, angular = 0., 0., 0.
        for i, dt in enumerate(np.diff(self.times)):
            if self.constant_trace is not None and self.constant_trace.start <= i < self.constant_trace.end:
                continue
            c = self.polynomial(i)
            lo, hi = self.extrema(c, 1)
            v = np.maximum(abs(lo), abs(hi))/dt
            lo, hi = self.extrema(c, 2)
            a = np.maximum(abs(lo), abs(hi))/dt**2
            speed, accel = max(speed, v.max()), max(accel, a.max())
            # Sum of absolute joint speeds bounds TCP angular speed for this 6R robot.
            angular = max(angular, np.degrees(v.sum()))
        if self.constant_trace is not None:
            trace = self.constant_trace
            v, a = trace.rate_bounds(self.times[trace.end]-self.times[trace.start])
            speed, accel, angular = max(speed, v.max()), max(accel, a.max()), max(angular, np.degrees(v.sum(axis=1)).max())
        moving_nodes = sum(np.linalg.norm(self.sample(t)['dq'])>1e-10 for t in self.times[1:-1])
        return {'max_joint_speed_rad_s': float(speed), 'max_joint_accel_rad_s2': float(accel),
                'tcp_angular_speed_bound_deg_s': float(angular),
                'moving_internal_waypoints': int(moving_nodes),
                'stops_at_waypoints': moving_nodes == 0,
                'continuity': ('C2 fixed joint cubic with C2 scalar clock; stopped free transfers'
                               if self.constant_trace is not None else
                               'C2 quintic; continuous trace/surface transfers, stopped free transfers')}
