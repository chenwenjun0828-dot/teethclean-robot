"""Portable ROS trajectory contract. Angles are rad, time s, TCP poses m/xyzw."""
import hashlib
import json
from pathlib import Path

import numpy as np

SCHEMA = 'ur_ros1_quintic_v1'
UNITS = {'position': 'rad', 'velocity': 'rad/s', 'acceleration': 'rad/s^2',
         'time': 's', 'tcp': 'm,quaternion_xyzw', 'wrench': 'N,N*m'}
UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
UR_MAPPING = {'ur_'+name: name for name in UR_JOINTS}


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n',
                          encoding='utf-8')


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def joint_order(source, mapping, target):
    if (len(source) != 6 or len(set(source)) != 6 or len(target) != 6
            or len(set(target)) != 6 or set(mapping) != set(source)
            or set(mapping.values()) != set(target)):
        raise ValueError('Joint mapping must be an explicit six-joint bijection')
    return [source.index(next(k for k, v in mapping.items() if v == name)) for name in target]


def quintic(q0, q1, v0, v1, a0, a1, dt):
    """ROS trajectory_interface quintic Hermite coefficients in normalized time."""
    c0, c1, c2 = np.asarray(q0), dt*np.asarray(v0), .5*dt*dt*np.asarray(a0)
    p = np.asarray(q1)-c0-c1-c2
    v = dt*np.asarray(v1)-c1-2*c2
    a = dt*dt*np.asarray(a1)-2*c2
    return np.stack((c0, c1, c2, 10*p-4*v+.5*a, -15*p+7*v-a, 6*p-3*v+.5*a))


def evaluate(c, u, dt):
    return (np.polynomial.polynomial.polyval(u, c),
            np.polynomial.polynomial.polyval(u, np.arange(1, 6)[:, None]*c[1:])/dt,
            np.polynomial.polynomial.polyval(u, (np.arange(2, 6)*np.arange(1, 5))[:, None]*c[2:])/dt**2)


def orientation_error_deg(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return float(np.degrees(2*np.arccos(np.clip(abs(a@b), 0, 1))))


def interpolate_pose(a, b, u):
    a, b = np.asarray(a), np.asarray(b).copy()
    if a[3:]@b[3:] < 0:
        b[3:] *= -1
    result = (1-u)*a+u*b
    result[3:] /= np.linalg.norm(result[3:])
    return result


class JointTrajectory:
    def __init__(self, record):
        self.record = record
        if record.get('schema') != SCHEMA or record.get('units') != UNITS:
            raise ValueError('Unsupported trajectory schema or units')
        self.names = record['joint_names']
        joint_order(self.names, dict(zip(self.names, self.names)), self.names)
        points = record['points']
        self.times = np.array([p['time_from_start'] for p in points], float)
        self.q = np.array([p['positions'] for p in points], float)
        self.v = np.array([p['velocities'] for p in points], float)
        self.a = np.array([p['accelerations'] for p in points], float)
        self.tcp = np.array([p['tcp_pose'] for p in points], float)
        n = len(points)
        if (n < 2 or self.times[0] != 0 or not np.isfinite(self.times).all()
                or np.any(np.diff(np.rint(self.times*1e9).astype(np.int64)) < 1)
                or any(x.shape != (n, 6) or not np.isfinite(x).all() for x in (self.q, self.v, self.a))
                or self.tcp.shape != (n, 7) or not np.isfinite(self.tcp).all()
                or not np.allclose(np.linalg.norm(self.tcp[:, 3:], axis=1), 1, atol=1e-8)
                or not np.allclose(self.v[[0, -1]], 0, atol=1e-8)
                or not np.allclose(self.a[[0, -1]], 0, atol=1e-8)):
            raise ValueError('Invalid trajectory points, clock, TCP or stopped endpoints')
        boundaries = record['phases']
        if (not boundaries or boundaries[0]['start_s'] != 0
                or boundaries[-1]['end_s'] != self.times[-1]
                or any(p['end_s'] <= p['start_s'] or p['name'] not in ('transfer', 'retreat', 'trace', 'surface_transfer')
                       for p in boundaries)
                or any(a['end_s'] != b['start_s'] for a, b in zip(boundaries[:-1], boundaries[1:]))):
            raise ValueError('Invalid phase boundaries')
        self.coefficients = [quintic(self.q[i], self.q[i+1], self.v[i], self.v[i+1],
                                     self.a[i], self.a[i+1], dt)
                             for i, dt in enumerate(np.diff(self.times))]

    @property
    def duration(self):
        return float(self.times[-1])

    def sample(self, time_s):
        if not np.isfinite(time_s) or time_s < 0 or time_s > self.duration+1e-9:
            raise ValueError('Controller progress outside trajectory')
        i = min(max(int(np.searchsorted(self.times, time_s, side='right'))-1, 0), len(self.times)-2)
        dt = self.times[i+1]-self.times[i]
        u = float(np.clip((time_s-self.times[i])/dt, 0, 1))
        q, v, a = evaluate(self.coefficients[i], u, dt)
        phase = next(p['name'] for p in self.record['phases'] if time_s <= p['end_s'])
        return q, v, a, interpolate_pose(self.tcp[i], self.tcp[i+1], u), phase


def load_bundle(path):
    """Hash checks bind the trajectory, source snapshots and offline audit together."""
    path = Path(path)
    report = read_json(path.parent/'export_report.json')
    manifest = read_json(path.parent/'source_manifest.json')
    if (report['trajectory_sha256'] != sha256(path)
            or report['manifest_sha256'] != sha256(path.parent/'source_manifest.json')):
        raise ValueError('Export fingerprint mismatch')
    for name, digest in manifest['snapshots'].items():
        if Path(name).name != name or sha256(path.parent/name) != digest:
            raise ValueError('Source snapshot fingerprint mismatch: '+name)
    record = read_json(path)
    if record['source'] != manifest['gates'] or record['source'] != report['source']:
        raise ValueError('Source restrictions differ across bundle')
    return JointTrajectory(record), report, manifest


def execution_blockers(trajectory, report):
    source = trajectory.record['source']
    blockers = []
    required = {'check_mode': 'full', 'execution_ready': True, 'simulation_only': False,
                'robot_execution_allowed': True, 'camera_verified': True}
    for key, expected in required.items():
        value = source.get(key)
        if value != expected or (isinstance(expected, bool) and not isinstance(value, bool)):
            blockers.append('source.'+key)
    if report.get('load_verified_passed') is not True:
        blockers.append('full_load_verified_missing')
    if report.get('interpolation_audit', {}).get('passed') is not True:
        blockers.append('interpolation_audit_failed')
    motion = report.get('ros_motion_audit') or {}
    if motion.get('passed') is not True or motion.get('collision_checked') is not True:
        blockers.append('ros_full_motion_audit_missing')
    return blockers
