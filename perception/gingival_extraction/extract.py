"""
Extract a gingival margin line from a colored intraoral-scan PLY mesh.

Outputs:
  - gingival_line_points.ply: point cloud sampled on the extracted margin
  - gingival_line_segments.obj: OBJ line segments for the margin
  - gingival_line_points.csv: XYZ coordinates of margin points
  - gingival_line_preview.png: quick visual QA preview
  - <jaw>_vertex_labels.ply: original mesh colored by tooth/gum labels
  - gingival_surface_filter.json: selected surface and automatic filter diagnostics
  - gingival_cleanup.json: scan-rim and isolated color-region cleanup diagnostics

The method uses vertex RGB color to separate tooth-like and gingiva-like
regions, smooths the score over mesh adjacency, then extracts the 0.5 contour
on triangle faces. By default, an arch/normal filter keeps only buccal/labial
segments. Color-region cleanup removes scan-rim artifacts and small islands.
--surface all --no-cleanup restores the original full-contour extraction.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict, deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans

if __package__:
    from .cleanup import clean_color_regions
    from .outer_surface import filter_outer_segments
else:
    from cleanup import clean_color_regions
    from outer_surface import filter_outer_segments


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Extract gingival line from colored PLY mesh.")
    parser.add_argument("--input", required=True, help="Input colored PLY mesh.")
    parser.add_argument("--outdir", default=".", help="Output directory.")
    parser.add_argument(
        "--jaw",
        default="",
        help="Jaw name used in the label-mesh filename, for example mandibular or maxillary.",
    )
    parser.add_argument("--clusters", type=int, default=5, help="KMeans color clusters.")
    parser.add_argument("--threshold", type=float, default=0.5, help="Gingiva score contour threshold.")
    parser.add_argument("--smooth-iters", type=int, default=4, help="Mesh-neighborhood score smoothing iterations.")
    parser.add_argument("--smooth-alpha", type=float, default=0.55, help="Weight of neighbor mean during smoothing.")
    parser.add_argument("--weld-mm", type=float, default=0.12, help="Point weld tolerance in model units, usually mm.")
    parser.add_argument("--min-component-mm", type=float, default=2.0, help="Discard line components shorter than this.")
    parser.add_argument("--no-view", action="store_true", help="Save outputs without opening the 3D overlay window.")
    parser.add_argument("--surface", choices=("outer", "all"), default="outer",
                        help="Keep only buccal/labial margins (default), or extract all surfaces.")
    parser.add_argument("--no-cleanup", action="store_true",
                        help="Disable removal of scan-rim artifacts and isolated color patches.")
    return parser


def read_mesh(path: Path) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(str(path))
    if not mesh.has_vertices() or not mesh.has_triangles():
        raise ValueError(f"Could not read a triangle mesh from {path}")
    if not mesh.has_vertex_colors():
        raise ValueError("This workflow needs vertex colors. For STL, use a geometry-only method instead.")
    return mesh


def classify_gum_by_color(colors: np.ndarray, clusters: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return per-vertex gum score, cluster labels, and cluster centers in RGB."""
    sample_n = min(60000, len(colors))
    rng = np.random.default_rng(7)
    sample_idx = rng.choice(len(colors), sample_n, replace=False)
    kmeans = KMeans(n_clusters=clusters, n_init=12, random_state=7)
    kmeans.fit(colors[sample_idx])
    labels = kmeans.predict(colors)
    centers = kmeans.cluster_centers_

    redness = centers[:, 0] - 0.5 * (centers[:, 1] + centers[:, 2])
    lightness = centers.mean(axis=1)

    # Gingiva in colored oral scans is typically redder and darker than enamel.
    # Convert cluster statistics to a soft score, then normalize to 0..1.
    raw_cluster_score = 2.7 * redness - 0.9 * lightness
    raw_cluster_score = (raw_cluster_score - raw_cluster_score.min()) / (
        raw_cluster_score.ptp() + 1e-8
    )
    gum_score = raw_cluster_score[labels]
    return gum_score.astype(float), labels, centers


def vertex_neighbors(faces: np.ndarray, n_vertices: int) -> list[np.ndarray]:
    neigh = [set() for _ in range(n_vertices)]
    for a, b, c in faces:
        neigh[a].update((b, c))
        neigh[b].update((a, c))
        neigh[c].update((a, b))
    return [np.fromiter(s, dtype=np.int64) for s in neigh]


def smooth_scores(scores: np.ndarray, neighbors: list[np.ndarray], iters: int, alpha: float) -> np.ndarray:
    out = scores.astype(float).copy()
    for _ in range(iters):
        nxt = out.copy()
        for i, nb in enumerate(neighbors):
            if len(nb):
                nxt[i] = (1.0 - alpha) * out[i] + alpha * out[nb].mean()
        out = nxt
    return out


def contour_segments(vertices: np.ndarray, faces: np.ndarray, scores: np.ndarray, threshold: float) -> list[tuple[np.ndarray, np.ndarray]]:
    segments: list[tuple[np.ndarray, np.ndarray]] = []
    for tri in faces:
        vals = scores[tri] - threshold
        if np.all(vals >= 0) or np.all(vals <= 0):
            continue
        pts = []
        for i, j in ((0, 1), (1, 2), (2, 0)):
            vi, vj = vals[i], vals[j]
            if vi == 0:
                pts.append(vertices[tri[i]])
            if vi * vj < 0:
                t = abs(vi) / (abs(vi) + abs(vj))
                pts.append(vertices[tri[i]] * (1.0 - t) + vertices[tri[j]] * t)
        if len(pts) >= 2:
            segments.append((pts[0], pts[1]))
    return segments


def weld_and_filter_segments(
    segments: list[tuple[np.ndarray, np.ndarray]], weld_mm: float, min_component_mm: float
) -> tuple[np.ndarray, list[tuple[int, int]], list[float]]:
    point_index: dict[tuple[int, int, int], int] = {}
    points: list[np.ndarray] = []
    edges: list[tuple[int, int]] = []

    def add_point(p: np.ndarray) -> int:
        key = tuple(np.round(p / weld_mm).astype(int))
        if key not in point_index:
            point_index[key] = len(points)
            points.append(p)
        return point_index[key]

    for p, q in segments:
        a, b = add_point(p), add_point(q)
        if a != b:
            edges.append((a, b))

    if not points:
        return np.empty((0, 3)), [], []

    graph: dict[int, list[int]] = defaultdict(list)
    for ei, (a, b) in enumerate(edges):
        graph[a].append(ei)
        graph[b].append(ei)

    kept_edges: list[tuple[int, int]] = []
    component_lengths: list[float] = []
    seen_edges: set[int] = set()
    pts = np.asarray(points)
    for start_e in range(len(edges)):
        if start_e in seen_edges:
            continue
        q = deque([start_e])
        seen_edges.add(start_e)
        comp_edges = []
        while q:
            ei = q.popleft()
            comp_edges.append(ei)
            for v in edges[ei]:
                for nei in graph[v]:
                    if nei not in seen_edges:
                        seen_edges.add(nei)
                        q.append(nei)
        length = float(sum(np.linalg.norm(pts[a] - pts[b]) for a, b in (edges[e] for e in comp_edges)))
        if length >= min_component_mm:
            component_lengths.append(length)
            kept_edges.extend(edges[e] for e in comp_edges)

    used = sorted(set(i for e in kept_edges for i in e))
    remap = {old: new for new, old in enumerate(used)}
    filtered_pts = pts[used] if used else np.empty((0, 3))
    filtered_edges = [(remap[a], remap[b]) for a, b in kept_edges]
    return filtered_pts, filtered_edges, component_lengths


def write_points_ply(path: Path, points: np.ndarray) -> None:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    if len(points):
        cloud.colors = o3d.utility.Vector3dVector(np.tile([[0.0, 0.9, 1.0]], (len(points), 1)))
    o3d.io.write_point_cloud(str(path), cloud, write_ascii=True)


def write_obj_lines(path: Path, points: np.ndarray, edges: list[tuple[int, int]]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("# Gingival line segments extracted from colored PLY\n")
        for p in points:
            f.write(f"v {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        for a, b in edges:
            f.write(f"l {a + 1} {b + 1}\n")


def write_csv(path: Path, points: np.ndarray) -> None:
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        writer.writerows(points.tolist())


def write_label_mesh(path: Path, mesh: o3d.geometry.TriangleMesh, scores: np.ndarray, threshold: float) -> None:
    label_mesh = o3d.geometry.TriangleMesh(mesh)
    colors = np.asarray(label_mesh.vertex_colors)
    tooth = scores < threshold
    colors[tooth] = np.array([0.95, 0.95, 0.90])
    colors[~tooth] = np.array([0.85, 0.18, 0.22])
    o3d.io.write_triangle_mesh(str(path), label_mesh, write_ascii=True)


def write_preview(path: Path, vertices: np.ndarray, colors: np.ndarray, line_points: np.ndarray) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), dpi=170)
    views = [(0, 2, "X-Z"), (0, 1, "X-Y"), (2, 1, "Z-Y")]
    step = max(1, len(vertices) // 45000)
    for ax, (i, j, title) in zip(axes, views):
        ax.scatter(vertices[::step, i], vertices[::step, j], c=colors[::step], s=0.25, linewidths=0)
        if len(line_points):
            ax.scatter(line_points[:, i], line_points[:, j], c="#00e5ff", s=1.0, linewidths=0)
        ax.set_title(title)
        ax.set_aspect("equal", adjustable="box")
        ax.axis("off")
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def show_3d_overlay(mesh: o3d.geometry.TriangleMesh, points: np.ndarray,
                    edges: list[tuple[int, int]]) -> None:
    """Show the original colored scan with its extracted gingival line."""
    line = o3d.geometry.LineSet()
    line.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    line.lines = o3d.utility.Vector2iVector(np.asarray(edges, dtype=np.int32).reshape(-1, 2))
    line.paint_uniform_color([0.0, 0.9, 1.0])

    markers = o3d.geometry.PointCloud()
    markers.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=float))
    markers.paint_uniform_color([0.0, 0.9, 1.0])

    viewer = o3d.visualization.Visualizer()
    if not viewer.create_window(window_name="牙齿 + 牙龈线 | 3D Overlay", width=1280, height=800):
        raise RuntimeError("Could not open the 3D overlay window; outputs were saved. Use --no-view on a headless machine.")
    try:
        viewer.add_geometry(mesh)
        viewer.add_geometry(line)
        viewer.add_geometry(markers)
        options = viewer.get_render_option()
        options.background_color = np.array([0.08, 0.10, 0.14])
        options.line_width = 3.0
        options.point_size = 3.0
        options.mesh_show_back_face = True
        viewer.run()
    finally:
        viewer.destroy_window()


def extract_gingival_line(
    input_path: Path, outdir: Path, jaw: str,
    clusters: int = 5, threshold: float = 0.5, smooth_iters: int = 4,
    smooth_alpha: float = 0.55, weld_mm: float = 0.12,
    min_component_mm: float = 2.0,
    surface: str = 'outer',
    cleanup: bool = True,
) -> dict:
    """Extract outer (or all) gingival margins in scan millimetres, without a UI."""
    input_path = Path(input_path)
    outdir = Path(outdir)
    if input_path.suffix.lower() != '.ply':
        raise ValueError('Gingival extraction requires a colored PLY mesh')
    if surface not in ('outer', 'all'):
        raise ValueError("surface must be 'outer' or 'all'")
    outdir.mkdir(parents=True, exist_ok=True)
    jaw_name = jaw.strip() or input_path.stem

    mesh = read_mesh(input_path)
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    colors = np.asarray(mesh.vertex_colors)

    gum_score, cluster_labels, centers = classify_gum_by_color(colors, clusters)
    neighbors = vertex_neighbors(faces, len(vertices))
    gum_score = smooth_scores(gum_score, neighbors, smooth_iters, smooth_alpha)
    gum_score = (gum_score - gum_score.min()) / (gum_score.ptp() + 1e-8)

    cleanup_report = {'enabled': False}
    if cleanup:
        gum_score, cleanup_report = clean_color_regions(mesh, gum_score, threshold)

    raw_segments = contour_segments(vertices, faces, gum_score, threshold)
    surface_report = {'surface': surface, 'input_segments': len(raw_segments),
                      'kept_segments': len(raw_segments), 'removed_segments': 0}
    selected_segments = raw_segments
    if surface == 'outer':
        selected_segments, surface_report = filter_outer_segments(mesh, gum_score, threshold, raw_segments)
    line_points, line_edges, component_lengths = weld_and_filter_segments(
        selected_segments, weld_mm, min_component_mm
    )
    if len(line_points) < 2 or not line_edges:
        raise ValueError(f'No connected gingival line was extracted for surface={surface}')

    write_points_ply(outdir / "gingival_line_points.ply", line_points)
    write_obj_lines(outdir / "gingival_line_segments.obj", line_points, line_edges)
    write_csv(outdir / "gingival_line_points.csv", line_points)
    write_label_mesh(outdir / f"{jaw_name}_vertex_labels.ply", mesh, gum_score, threshold)
    write_preview(outdir / "gingival_line_preview.png", vertices, colors, line_points)
    (outdir / 'gingival_surface_filter.json').write_text(
        json.dumps(surface_report, ensure_ascii=False, indent=2), encoding='utf-8')
    (outdir / 'gingival_cleanup.json').write_text(
        json.dumps(cleanup_report, ensure_ascii=False, indent=2), encoding='utf-8')

    redness = centers[:, 0] - 0.5 * (centers[:, 1] + centers[:, 2])
    lightness = centers.mean(axis=1)
    with (outdir / "gingival_line_report.txt").open("w", encoding="utf-8", newline="\n") as f:
        f.write(f"input: {input_path}\n")
        f.write(f"vertices: {len(vertices)}\n")
        f.write(f"faces: {len(faces)}\n")
        f.write(f"raw contour segments: {len(raw_segments)}\n")
        f.write(f"surface: {surface}\n")
        f.write(f"color region cleanup: {cleanup}\n")
        if cleanup:
            f.write(f"removed unsupported tooth regions: {cleanup_report['removed_tooth_components']}\n")
            f.write(f"filled small gum islands: {cleanup_report['filled_small_gum_islands']}\n")
        f.write(f"segments after surface filtering: {len(selected_segments)}\n")
        f.write(f"kept line points: {len(line_points)}\n")
        f.write(f"kept line segments: {len(line_edges)}\n")
        f.write(f"kept connected components: {len(component_lengths)}\n")
        f.write(f"component lengths mm/model-units: {', '.join(f'{x:.2f}' for x in sorted(component_lengths, reverse=True)[:30])}\n")
        f.write("cluster RGB/redness/lightness:\n")
        for i, center in enumerate(centers):
            f.write(
                f"  {i}: rgb={center[0]:.3f},{center[1]:.3f},{center[2]:.3f}; "
                f"redness={redness[i]:.3f}; lightness={lightness[i]:.3f}; "
                f"vertices={(cluster_labels == i).sum()}\n"
            )

    print(f"Wrote outputs to {outdir.resolve()}")
    print(f"Line points: {len(line_points)}")
    print(f"Line segments: {len(line_edges)}")
    print(f"Surface: {surface}")
    return {'points_mm': line_points, 'edges': line_edges,
            'line_csv': outdir / 'gingival_line_points.csv',
            'topology_obj': outdir / 'gingival_line_segments.obj',
            'preview': outdir / 'gingival_line_preview.png', 'surface': surface,
            'surface_filter': surface_report, 'cleanup': cleanup_report}


def main() -> None:
    args = build_argparser().parse_args()
    result = extract_gingival_line(args.input, args.outdir, args.jaw,
                                   args.clusters, args.threshold, args.smooth_iters,
                                   args.smooth_alpha, args.weld_mm, args.min_component_mm,
                                   surface=args.surface, cleanup=not args.no_cleanup)
    if not args.no_view:
        print("Opening 3D tooth + gingival line overlay. Close the window to exit.")
        show_3d_overlay(read_mesh(Path(args.input)), result['points_mm'], result['edges'])


if __name__ == "__main__":
    main()
