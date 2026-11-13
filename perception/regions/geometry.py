"""Shared curve fitting and regional geometry exports."""
from dataclasses import dataclass
import numpy as np
import open3d as o3d
from scipy.interpolate import BSpline, make_interp_spline
from perception.gingival_line import polyline_distances
from .io import dump
from .config import REGION_ORDER
from .io import unit


def fit_curve(points, tolerance_m, spacing_m):
    p = np.asarray(points, float)
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-10]]
    if len(p) < 4:
        raise ValueError('A cubic fit requires at least four distinct source points')
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    u = arc / arc[-1]
    base = make_interp_spline(u, p - p[0], k=3)
    basis = BSpline(base.t, np.eye(len(base.c)), 3)
    design = basis(u)
    penalty = np.diff(np.eye(len(base.c)), n=2, axis=0)
    fixed, free = [0, len(base.c)-1], np.arange(1, len(base.c)-1)
    end = p[[0, -1]] - p[0]
    rhs = (p - p[0]) - design[:, fixed] @ end
    dense_u = np.unique(np.r_[u, np.linspace(0, 1, max(100, int(arc[-1]/.000025)+1))])
    curve = base
    error = float('inf')
    for weight in (10., 1., .1, .01, .001, 0.):
        a = np.vstack((design[:, free], np.sqrt(weight)*penalty[:, free]))
        b = np.vstack((rhs, -np.sqrt(weight)*penalty[:, fixed] @ end))
        coeff = np.empty_like(base.c)
        coeff[fixed] = end
        coeff[free] = np.linalg.lstsq(a, b, rcond=None)[0]
        curve = BSpline(base.t, coeff+p[0], 3, extrapolate=False)
        error = float(max(np.linalg.norm(curve(u)-p, axis=1).max(),
                          polyline_distances(curve(dense_u), p).max()))
        if error <= tolerance_m:
            break
    if error > tolerance_m:
        raise ValueError('Cubic curve exceeds the source corridor')
    dense_p = curve(dense_u)
    dense_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(dense_p, axis=0), axis=1))]
    station = np.linspace(0, dense_arc[-1], max(4, int(np.ceil(dense_arc[-1]/spacing_m))+1))
    sampled_u = np.interp(station, dense_arc, dense_u)
    derivative = curve.derivative()(sampled_u)
    lengths = np.linalg.norm(derivative, axis=1)
    if np.any(lengths < 1e-10):
        raise ValueError('Cubic curve has a stationary tangent')
    return curve(sampled_u), derivative/lengths[:, None], sampled_u, {
        'degree': 3, 'knots': curve.t, 'coefficients_m': curve.c,
        'source_points_m': p, 'max_fit_error_mm': error*1000,
        'smoothing_weight': weight, 'length_mm': dense_arc[-1]*1000}


@dataclass
class Stroke:
    id: int
    curve_id: int
    tooth: int
    region: str
    points: np.ndarray
    tangents: np.ndarray
    parameters: np.ndarray
    sample_indices: np.ndarray

    @property
    def length_mm(self):
        return float(np.linalg.norm(np.diff(self.points, axis=0), axis=1).sum()*1000)


@dataclass
class Geometry:
    strokes: list
    planes: dict
    report: dict
    mesh_vertices_m: np.ndarray
    mesh_faces: np.ndarray
    face_tooth_ids: np.ndarray

    def save(self, output):
        dump(output/'geometry.json', self.report)
        dump(output/'tooth_planes.json', self.planes)
        columns = {'points_scan_m': [], 'tangents_scan': [], 'u': [], 'stroke_id': [],
                   'tooth_id': [], 'region_id': [], 'curve_id': [], 'sample_index': []}
        for stroke in self.strokes:
            for name, array in [('points_scan_m', stroke.points), ('tangents_scan', stroke.tangents),
                                ('u', stroke.parameters), ('sample_index', stroke.sample_indices)]:
                columns[name].append(array)
            for name, value in [('stroke_id', stroke.id), ('tooth_id', stroke.tooth),
                                ('region_id', self.report.get('region_names', REGION_ORDER).index(stroke.region)),
                                ('curve_id', stroke.curve_id)]:
                columns[name].append(np.full(len(stroke.points), value))
        np.savez_compressed(output/'geometry.npz', **{k: np.concatenate(v) for k, v in columns.items()})
        np.save(output/'tooth_face_ids.npy', self.face_tooth_ids)
        folder = output/'teeth'
        folder.mkdir(exist_ok=True)
        for tooth in sorted(set(self.face_tooth_ids)-{0}):
            face_ids = np.flatnonzero(self.face_tooth_ids == tooth)
            mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(self.mesh_vertices_m*1000),
                                             o3d.utility.Vector3iVector(self.mesh_faces[face_ids]))
            mesh.remove_unreferenced_vertices()
            o3d.io.write_triangle_mesh(str(folder/f'tooth_{tooth:02d}_scan_mm.ply'), mesh)
            np.save(folder/f'tooth_{tooth:02d}_source_faces.npy', face_ids)
