"""Ordered gingival chains transformed from scan millimetres to world metres."""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import numpy as np
from .frames import validate_transform
ROOT = Path(__file__).resolve().parents[1]

def read_xyz(path: Path) -> np.ndarray:
    values = np.genfromtxt(path, delimiter=',', names=True, encoding='utf-8-sig')
    if not {'x', 'y', 'z'}.issubset(values.dtype.names or ()):
        raise ValueError(f'CSV requires x,y,z columns: {path}')
    points = np.column_stack([np.atleast_1d(values[k]) for k in ('x', 'y', 'z')])
    if len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('At least two finite gingival points are required')
    return points


def read_topology(path: Path) -> tuple[np.ndarray, list[tuple[int, int]]]:
    vertices, edges = [], []
    for line in path.read_text(encoding='utf-8-sig').splitlines():
        fields = line.split()
        if not fields or fields[0].startswith('#'):
            continue
        if fields[0] == 'v':
            vertices.append([float(x) for x in fields[1:4]])
        elif fields[0] == 'l':
            indices = [int(x.split('/')[0])-1 for x in fields[1:]]
            edges.extend(zip(indices[:-1], indices[1:]))
    points = np.asarray(vertices, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all() or not edges:
        raise ValueError('Topology requires finite OBJ vertices and line edges')
    if any(min(edge) < 0 or max(edge) >= len(points) for edge in edges):
        raise ValueError('Topology edge references a missing vertex')
    return points, edges


def path_length(points: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def polyline_distances(query: np.ndarray, line: np.ndarray) -> np.ndarray:
    """Exact nearest segment distance, in the input coordinate units."""
    query, line = np.asarray(query, float), np.asarray(line, float)
    if (query.ndim != 2 or query.shape[1] != 3 or line.ndim != 2
            or line.shape[1] != 3 or len(line) < 2 or not np.isfinite(query).all()
            or not np.isfinite(line).all()):
        raise ValueError('Distance calculation requires finite Nx3 points and a polyline')
    a, delta = line[:-1], np.diff(line, axis=0)
    denom = np.maximum(np.sum(delta**2, axis=1), 1e-24)
    result = []
    for chunk in np.array_split(query, max(1, int(np.ceil(len(query)/128)))):
        difference = chunk[:, None, :]-a
        fraction = np.clip(np.sum(difference*delta, axis=2)/denom, 0., 1.)
        residual = difference-fraction[:, :, None]*delta
        result.extend(np.sqrt(np.min(np.sum(residual**2, axis=2), axis=1)))
    return np.asarray(result)


def ordered_chains(points: np.ndarray, edges) -> list[np.ndarray]:
    """Split at degree != 2 vertices, preserving every unique undirected edge."""
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Points must be finite Nx3 coordinates')
    remaining = {tuple(sorted((int(a), int(b)))) for a, b in edges if a != b}
    graph = {}
    for a, b in remaining:
        if a < 0 or b >= len(points):
            raise ValueError('Topology edge references a missing vertex')
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)
    result = []

    def walk(start, neighbor):
        indices = [start, neighbor]
        remaining.remove(tuple(sorted((start, neighbor))))
        previous, current = start, neighbor
        while len(graph[current]) == 2 and current != start:
            following = next(x for x in graph[current] if x != previous)
            edge = tuple(sorted((current, following)))
            if edge not in remaining:
                break
            remaining.remove(edge)
            indices.append(following)
            previous, current = current, following
        result.append(points[indices].copy())

    for start in sorted(graph):
        if len(graph[start]) != 2:
            for neighbor in sorted(graph[start]):
                if tuple(sorted((start, neighbor))) in remaining:
                    walk(start, neighbor)
    while remaining:
        a, b = min(remaining)
        walk(a, b)
    return sorted(result, key=path_length, reverse=True)


def resample_path(points: np.ndarray, spacing_m: float = 0.0005) -> np.ndarray:
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('Path requires at least two finite Nx3 points')
    if not np.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError('Spacing must be positive and finite')
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points = points[np.r_[True, steps > 1e-12]]
    if len(points) < 2:
        raise ValueError('Path has zero length')
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    samples = np.linspace(0., arc[-1], int(np.ceil(arc[-1]/spacing_m))+1)
    return np.column_stack([np.interp(samples, arc, points[:, i]) for i in range(3)])


@dataclass
class RegisteredPaths:
    world_points: np.ndarray
    chains: list[np.ndarray]
    world_from_camera: np.ndarray
    world_from_scan: np.ndarray
    scan_points_mm: np.ndarray
    jaw: str
    camera_name: str
    registration_dir: Path | None
    topology_path: Path
    sampling_spacing_m: float = 0.0005


def load_registered_paths(registration_dir: Path, jaw: str, topology_path: Path | None = None) -> RegisteredPaths:
    if jaw not in ('mandibular', 'maxillary'):
        raise ValueError('Jaw must be mandibular or maxillary')
    registration_dir = Path(registration_dir).resolve()
    report = json.loads((registration_dir/'registration_report.json').read_text(encoding='utf-8-sig'))
    if report.get('units') != 'mm' or report.get('transform_name') != 'T_camera_scan':
        raise ValueError('Registration must supply T_camera_scan in mm')
    if report.get('jaw') not in (jaw, 'both'):
        raise ValueError('Registration does not include the requested jaw')
    capture_dir = Path(report['capture_dir'])
    if not capture_dir.is_absolute():
        capture_dir = registration_dir/capture_dir
    capture = json.loads((capture_dir/'capture_metadata.json').read_text(encoding='utf-8-sig'))
    if capture.get('capture_id') != report.get('capture_id'):
        raise ValueError('Registration and capture IDs disagree')
    # Locate the same frozen source line used by registration; avoid importing Open3D.
    source_path = Path(report['config'][jaw+'_line'])
    if not source_path.is_absolute():
        source_path = ROOT/source_path
    expected_hash = report['reference_line_hashes'][jaw]
    if hashlib.sha256(source_path.read_bytes()).hexdigest() != expected_hash:
        raise ValueError('Reference line hash differs from registration')
    scan_points = read_xyz(source_path)
    topology_path = Path(topology_path) if topology_path else ROOT/'assets/perception'/jaw/'gingival_line_segments.obj'
    vertices, edges = read_topology(topology_path)
    if vertices.shape != scan_points.shape or not np.allclose(vertices, scan_points, rtol=0., atol=1e-6):
        raise ValueError('Topology vertices do not match the registered reference line')
    camera_from_scan_mm = validate_transform(np.load(registration_dir/'T_camera_scan_estimated.npy'))
    world_from_camera = validate_transform(np.asarray(capture['T_world_camera_cv_m'], float))
    camera_from_scan_m = camera_from_scan_mm.copy()
    camera_from_scan_m[:3, 3] /= 1000.
    world_from_scan = world_from_camera @ camera_from_scan_m
    world_points = scan_points/1000. @ world_from_scan[:3, :3].T + world_from_scan[:3, 3]
    chains = ordered_chains(world_points, edges)
    chains = [chain for chain in chains if path_length(chain) >= 0.002]
    if not chains:
        raise ValueError('No continuous gingival chain at least 2 mm long')
    return RegisteredPaths(world_points, chains, world_from_camera, world_from_scan, scan_points, jaw,
                           capture['camera'], registration_dir, topology_path.resolve())

