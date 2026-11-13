"""Area-weighted regional tooth planes and diagnostic source landmarks."""
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from scipy.spatial.distance import pdist, squareform
from .io import unit


def three_point_plane(points, outward, rms_limit_m):
    p = np.asarray(points, float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or not np.isfinite(p).all():
        raise ValueError('At least three finite surface points are required')
    n0 = unit(outward)
    # Project only to select a broad exterior footprint. Plane uses original 3D points.
    helper = np.eye(3)[np.argmin(abs(n0))]
    u = unit(np.cross(n0, helper)); v = np.cross(n0, u)
    projected = np.column_stack((p@u, p@v))
    try:
        hull = ConvexHull(projected).vertices
    except QhullError as error:
        raise ValueError('Tooth surface support is collinear') from error
    distance = squareform(pdist(p[hull]))
    a, b = np.unravel_index(np.argmax(distance), distance.shape)
    first, second = int(hull[a]), int(hull[b])
    baseline = p[second]-p[first]
    areas2 = np.linalg.norm(np.cross(baseline, p-p[first]), axis=1)
    third = int(np.argmax(areas2))
    if areas2[third] < 1e-10 or areas2[third]/np.dot(baseline, baseline) < .02:
        raise ValueError('Three-point support is nearly collinear')
    ids = np.array([first, second, third])
    triangle = p[ids]
    n = unit(np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0]))
    if n@n0 < 0: n = -n
    center = triangle.mean(0)
    residual = (p-center)@n
    rms = float(np.sqrt(np.mean(residual**2)))
    valid = rms <= rms_limit_m and n@n0 >= .5
    return {'valid': bool(valid), 'reason': '' if valid else 'three_point_plane_residual_or_orientation',
            'normal_scan': n, 'center_scan_m': center, 'landmarks_scan_m': triangle,
            'landmark_support_indices': ids, 'triangle_area_mm2': float(areas2[third]*.5e6),
            'rms_mm': rms*1000, 'max_residual_mm': float(abs(residual).max()*1000),
            'support_points': len(p), 'selection': 'projected_hull_farthest_pair_then_maximum_3d_triangle_area'}


def surface_plane(points, outward, rms_limit_m, weights):
    """Fit all patch samples; triangle centroids are weighted by face area."""
    p, w = np.asarray(points, float), np.asarray(weights, float)
    if (p.ndim != 2 or p.shape[1] != 3 or len(p) < 3 or not np.isfinite(p).all()
            or w.shape != (len(p),) or not np.isfinite(w).all() or np.any(w <= 0)):
        raise ValueError('Expected at least three finite surface points with positive area weights')
    center = np.average(p, axis=0, weights=w)
    eigenvalues, vectors = np.linalg.eigh(((p-center)*w[:, None]).T@(p-center)/w.sum())
    if eigenvalues[1] < 1e-14 or eigenvalues[1] < .0004*eigenvalues[2]:
        raise ValueError('Tooth surface support is nearly collinear')
    n = vectors[:, 0]
    n0 = unit(outward)
    if n@n0 < 0:
        n = -n
    residual = (p-center)@n
    rms = float(np.sqrt(np.average(residual**2, weights=w)))
    # Keep real source landmarks for provenance, and project only the display triangle.
    result = three_point_plane(p, outward, rms_limit_m)
    display = result['landmarks_scan_m']
    display = display-((display-center)@n)[:, None]*n
    valid = rms <= rms_limit_m and n@n0 >= .5
    result.update(valid=bool(valid), reason='' if valid else 'surface_plane_residual_or_orientation',
                  normal_scan=n, center_scan_m=center, rms_mm=rms*1000,
                  max_residual_mm=float(abs(residual).max()*1000),
                  plane_vertices_scan_m=display,
                  fit_method='triangle_area_weighted_total_least_squares',
                  residual_weighting='source_triangle_area',
                  support_span_mm=(np.sqrt(np.maximum(eigenvalues[1:], 0))*1000).tolist())
    return result
