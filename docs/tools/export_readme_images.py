"""Export README illustrations from explicit simulation and prediction artifacts."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]


def project_cloud(axis, cloud, title):
    points, colors = np.asarray(cloud.points), np.asarray(cloud.colors)
    if not len(points):
        raise ValueError('Cannot illustrate an empty point cloud')
    selected = np.linspace(0, len(points) - 1, min(len(points), 18000), dtype=int)
    points = points[selected]
    axis.scatter(points[:, 0], points[:, 1], s=1,
                 c=colors[selected] if len(colors) else '#247c86', rasterized=True)
    axis.set(xlabel='X (mm)', ylabel='Y (mm)', title=title)
    axis.set_aspect('equal', adjustable='datalim')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--force', type=Path, required=True)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--regional-preview', type=Path, required=True)
    parser.add_argument('--pose', type=Path, required=True)
    parser.add_argument('--pose-slider', type=Path, help='Optional screenshot of the interactive angle viewer')
    parser.add_argument('--out', type=Path, default=ROOT / 'docs/images')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    sources = {}

    def source(name, path):
        sources[name] = {'source': path.resolve().relative_to(ROOT).as_posix(),
                         'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        return path

    copies = {
        'gingival_extraction.png': args.run / 'extraction/gingival_line_preview.png',
        'registration.png': args.run / 'registration/registration_overlay.png',
        'regional_paths.png': args.regional_preview / 'regional_preview.png',
        'tool_pose.png': args.pose,
        'trajectory.png': args.run / 'tracking/continuous_sweep.png',
        'force_control.png': args.force / 'force_tracking.png',
        'force_benchmark.png': args.benchmark / 'benchmark_force.png',
    }
    if args.pose_slider is not None:
        copies['pose_angles.png'] = args.pose_slider
    for name, path in copies.items():
        shutil.copyfile(source(name, path), args.out / name)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), layout='constrained')
    for axis, filename, title in zip(axes[:2], ('rgb.png', 'depth_preview.png'),
                                     ('Simulation RGB', 'Saved depth preview')):
        with Image.open(source(filename, args.run / 'capture' / filename)) as picture:
            axis.imshow(picture)
        axis.set_title(title)
        axis.axis('off')
    cloud = o3d.io.read_point_cloud(str(source('camera_mm.ply', args.run / 'capture/camera_mm.ply')))
    project_cloud(axes[2], cloud, 'Valid dental points: camera frame')
    fig.savefig(args.out / 'simulation_capture.png', dpi=140)
    plt.close(fig)

    path = ROOT / 'assets/perception/tooth_segmentation/crosstooth_mandibular/segmentation.ply'
    cloud = o3d.io.read_point_cloud(str(source('segmentation.ply', path)))
    points = np.asarray(cloud.points).copy()
    cloud.points = o3d.utility.Vector3dVector(points[:, [0, 2, 1]])
    fig, axis = plt.subplots(figsize=(9, 5.5), layout='constrained')
    project_cloud(axis, cloud, 'Bundled CrossTooth instance snapshot: scan X-Z projection')
    axis.set_ylabel('Z (mm)')
    fig.savefig(args.out / 'tooth_segmentation.png', dpi=140)
    plt.close(fig)

    trace = source('execution_trace.csv', args.run / 'tracking/execution_trace.csv')
    with trace.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    t = np.array([float(row['time_s']) for row in rows])
    report_path = source('execution_report.json', args.run / 'tracking/execution_report.json')
    report = json.loads(report_path.read_text(encoding='utf-8'))
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True, layout='constrained')
    for axis, field, label, color in zip(axes,
            ('position_error_mm', 'rotation_error_deg'),
            ('TCP position error (mm)', 'TCP rotation error (deg)'), ('#177a82', '#b94762')):
        axis.plot(t, [float(row[field]) for row in rows], color=color, lw=1)
        axis.set_ylabel(label)
        axis.grid(alpha=.2)
    axes[-1].set_xlabel('Executed simulation time (s)')
    fig.suptitle(f"Position servo: {report['status']} | "
                 f"minimum measured clearance {report['minimum_actual_clearance_mm']:.4f} mm")
    fig.savefig(args.out / 'position_control.png', dpi=150)
    plt.close(fig)
    (args.out / 'sources.json').write_text(json.dumps(sources, indent=2), encoding='utf-8')
    print(f'Exported {10 + int(args.pose_slider is not None)} README images to {args.out.resolve()}')


if __name__ == '__main__':
    main()
