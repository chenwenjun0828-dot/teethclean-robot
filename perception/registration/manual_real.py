"""Manual real-cloud registration from a saved PLY or a new D405 capture."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

os.environ.setdefault('OMP_NUM_THREADS', '4')
import numpy as np
import open3d as o3d

from ..global_camera import PROJECT_ROOT, ensure_empty_directory, file_hash, save_cloud, write_json
from ..artifacts import default_output, export_coordinates, load_extrinsics
from .algorithms import load_scan_cloud, load_line_csv, pick_landmark_pairs, coarse_from_landmarks
from .registration_math import transform_points
from .registration_workflow import save_overlay_png, save_registration_transforms

SAMPLE = PROJECT_ROOT / 'assets/real_captures/dental_phantom_legacy.ply'
LEGACY_ROI = (-50., 25., 100., 50., 70., 450.)


def prepare_cloud(path, *, units, frame, roi=None):
    """Normalize to CV camera millimetres; ROI is xmin,ymin,zmin,xmax,ymax,zmax."""
    if units not in ('m', 'mm') or frame not in ('cv', 'opengl'):
        raise ValueError('Specify units=m/mm and frame=cv/opengl')
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    cloud = o3d.io.read_point_cloud(str(path))
    points = np.asarray(cloud.points)
    if len(points) < 200 or not np.isfinite(points).all():
        raise ValueError('Camera cloud requires at least 200 finite points')
    scale = 1000. if units == 'm' else 1.
    conversion = np.diag([scale, scale, scale, 1.])
    if frame == 'opengl':
        conversion[1, 1] *= -1
        conversion[2, 2] *= -1
    cloud.transform(conversion)
    if roi is not None:
        bounds = np.asarray(roi, float)
        if bounds.shape != (6,) or not np.isfinite(bounds).all() or np.any(bounds[:3] >= bounds[3:]):
            raise ValueError('ROI needs finite increasing min/max bounds')
        points = np.asarray(cloud.points)
        mask = np.all((points >= bounds[:3]) & (points <= bounds[3:]), axis=1)
        cloud = cloud.select_by_index(np.flatnonzero(mask))
    if len(cloud.points) < 200:
        raise ValueError('Fewer than 200 points in ROI; check units, axes and bounds')
    return cloud, conversion


def refine_manual(scan, camera, initial):
    """Historical three-scale robust camera-to-scan ICP; return scan-to-camera."""
    current = np.linalg.inv(initial)
    stages = []
    reg = o3d.pipelines.registration
    for voxel, gate in ((1., 3.), (.5, 1.5), (.25, .75)):
        source, target = camera.voxel_down_sample(voxel), scan.voxel_down_sample(voxel)
        if min(len(source.points), len(target.points)) < 3:
            raise ValueError('Insufficient points at an ICP scale')
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3., max_nn=40))
        result = reg.registration_icp(source, target, gate, current,
            reg.TransformationEstimationPointToPlane(reg.TukeyLoss(k=gate)),
            reg.ICPConvergenceCriteria(max_iteration=100))
        if result.fitness <= 0 or not np.isfinite(result.transformation).all():
            raise ValueError('ICP found no valid correspondences')
        current = result.transformation
        stages.append(dict(voxel_mm=voxel, gate_mm=gate, fitness=result.fitness,
                           inlier_rmse_mm=result.inlier_rmse))
    return np.linalg.inv(current), stages


def register_manual_clouds(camera_path, scan_path, output_dir, *, units, frame,
                           roi=None, landmarks=None, line=None, show_view=False,
                           prepare_only=False, extrinsics=None, run_dir=None):
    """File API; optional landmarks must match the normalized cloud and scan hashes."""
    scan_path, camera_path = Path(scan_path).resolve(), Path(camera_path).resolve()
    external = load_extrinsics(extrinsics) if extrinsics is not None else None
    camera, conversion = prepare_cloud(camera_path, units=units, frame=frame, roi=roi)
    scan = load_scan_cloud([scan_path], allow_point_cloud=True)
    out = ensure_empty_directory(output_dir)
    camera_file = out / 'camera_mm.ply'
    save_cloud(camera_file, camera)
    fingerprints = dict(scan_sha256=file_hash(scan_path), camera_sha256=file_hash(camera_file))
    write_json(out / 'input.json', dict(camera_path=str(camera_path), scan_path=str(scan_path),
        original_camera_sha256=file_hash(camera_path), input_units=units, input_frame=frame,
        T_camera_mm_input=conversion.tolist(), roi_bounds_mm=roi,
        camera_points=len(camera.points), **fingerprints))
    if prepare_only:
        return out
    report = dict(status='running', registration_method='manual_multiscale_robust_icp',
                  robot_execution_allowed=False, units='mm', source_frame='oral_scan',
                  target_frame='camera_cv', **fingerprints)
    try:
        if landmarks is not None:
            pairs = json.loads(Path(landmarks).read_text())
            # The original 4.ply script called the normalized camera hash roi_sha256.
            if 'camera_sha256' not in pairs and 'roi_sha256' in pairs:
                pairs['camera_sha256'] = pairs['roi_sha256']
            if any(pairs.get(key) != value for key, value in fingerprints.items()):
                raise ValueError('Landmarks do not match this scan and normalized camera cloud')
            source, target = np.asarray(pairs['source_scan_mm']), np.asarray(pairs['target_camera_mm'])
        else:
            source, target, si, ti = pick_landmark_pairs(scan, camera, 6)
            pairs = dict(source_scan_mm=source.tolist(), target_camera_mm=target.tolist(),
                         source_indices=si, target_indices=ti, **fingerprints)
        write_json(out / 'landmarks.json', pairs)
        initial, inliers, residuals = coarse_from_landmarks(source, target)
        # Save initialization before refinement so a failed ICP retains the manual result.
        np.save(out / 'T_camera_scan_initial.npy', initial)
        np.savetxt(out / 'T_camera_scan_initial.txt', initial, header='oral_scan_mm -> camera_cv_mm')
        final, stages = refine_manual(scan, camera, initial)
        save_registration_transforms(out, initial, final)
        registered = o3d.geometry.PointCloud(scan).transform(final)
        save_cloud(out / 'scan_registered_camera_mm.ply', registered)
        save_overlay_png(out / 'registration_overlay.png', camera, registered, {})
        if line is not None:
            points = transform_points(load_line_csv(line), final)
            np.savetxt(out / 'line_camera_mm.csv', points, delimiter=',', header='x,y,z', comments='')
        artifacts = export_coordinates(run_dir or out, final, line=line, extrinsics=external)
        report.update(status='completed', landmark_pairs=len(source), landmark_inliers=int(inliers.sum()),
            landmark_initial_residuals_mm=residuals.tolist(), stages=stages,
            fitness_source_frame='camera_cv', fitness_target_frame='oral_scan',
            quality_status='not_independently_validated', coordinate_outputs=artifacts)
        if show_view:
            o3d.visualization.draw_geometries([camera, registered.paint_uniform_color([.2,.85,.3])],
                                            window_name='Manual real-cloud registration')
    except Exception as error:
        report.update(status='failed', error=str(error))
        raise
    finally:
        write_json(out / 'registration_report.json', report)
    return out


def choose_ply():
    """Open a native file chooser only when explicitly selecting an existing PLY."""
    import tkinter as tk
    from tkinter.filedialog import askopenfilename
    root = tk.Tk()
    root.withdraw()
    try:
        selected = askopenfilename(title='选择真实相机点云 PLY', filetypes=[('PLY point cloud', '*.ply')])
    finally:
        root.destroy()
    if not selected:
        raise ValueError('No PLY selected')
    return Path(selected)


def add_arguments(parser):
    """Shared CLI for perception.real manual and the legacy module command."""
    parser.add_argument('--source', choices=('sample', 'ply', 'camera'), default='sample')
    parser.add_argument('--camera', type=Path, help='Existing PLY; omit with --source ply for a file chooser')
    parser.add_argument('--scan', type=Path, help='Matching oral scan in mm')
    parser.add_argument('--jaw', choices=('mandibular', 'maxillary'), default='mandibular')
    parser.add_argument('--units', choices=('m', 'mm'))
    parser.add_argument('--frame', choices=('cv', 'opengl'))
    parser.add_argument('--roi-mm', nargs=6, type=float, metavar=('XMIN','YMIN','ZMIN','XMAX','YMAX','ZMAX'))
    parser.add_argument('--config', type=Path, default=PROJECT_ROOT / 'config/real_camera_registration.json')
    parser.add_argument('--serial')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--landmarks', type=Path)
    parser.add_argument('--line', type=Path, help='Optional scan-frame x,y,z CSV in mm')
    parser.add_argument('--extrinsics', type=Path, help='Camera CV -> base JSON; translation_unit=m')
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--view', action='store_true')
    return parser


def run_manual(args, parser):
    if args.camera and args.source != 'ply':
        parser.error('--camera requires --source ply')
    if args.source == 'ply' and (args.units is None or args.frame is None):
        parser.error('Existing PLY requires explicit --units and --frame')
    if args.source == 'sample' and args.jaw != 'mandibular':
        parser.error('The legacy sample ROI is mandibular only')
    if args.source == 'camera' and (args.units is not None or args.frame is not None):
        parser.error('D405 acquisition already provides CV camera millimetres')
    out = args.out or default_output('real')
    summary = dict(status='running', source=args.source, robot_execution_allowed=False)
    try:
        # Validate before interacting with a camera or asking for landmarks.
        if args.extrinsics is not None:
            load_extrinsics(args.extrinsics)
        out = ensure_empty_directory(out)
        scan = args.scan or PROJECT_ROOT / f'assets/source_cad/teeth/{args.jaw}_scan.ply'
        if args.source == 'camera':
            from ..real import main as real_main
            capture = out / 'capture'
            command = ['capture', '--jaw', args.jaw, '--out', str(capture),
                       '--config', str(args.config), '--select-roi', '--preview']
            if args.serial:
                command.extend(['--serial', args.serial])
            if real_main(command) != 0:
                raise RuntimeError('Camera acquisition failed or was cancelled')
            camera, units, frame, roi = capture / 'camera_mm.ply', 'mm', 'cv', args.roi_mm
        elif args.source == 'sample':
            camera, units, frame = SAMPLE, args.units or 'm', args.frame or 'opengl'
            roi = args.roi_mm if args.roi_mm is not None else LEGACY_ROI
        else:
            camera, units, frame, roi = args.camera or choose_ply(), args.units, args.frame, args.roi_mm
        result = register_manual_clouds(camera, scan, out / 'registration', units=units,
            frame=frame, roi=roi, landmarks=args.landmarks, line=args.line,
            show_view=args.view, prepare_only=args.prepare_only,
            extrinsics=args.extrinsics, run_dir=out)
        summary.update(status='prepared' if args.prepare_only else 'completed',
                       registration_dir=str(result))
        if not args.prepare_only:
            summary['coordinate_outputs'] = json.loads((result / 'registration_report.json').read_text())['coordinate_outputs']
        write_json(out / 'run_report.json', summary)
        print(f'Saved: {result}')
        return 0
    except (ValueError, RuntimeError, OSError, ImportError, TypeError, KeyError) as error:
        summary.update(status='failed', error=str(error))
        # Do not modify a pre-existing run when empty-directory validation failed.
        if 'scan' in locals():
            write_json(out / 'run_report.json', summary)
        print(f'Manual real registration failed: {error}')
        return 1


def main(argv=None):
    parser = add_arguments(argparse.ArgumentParser(description=__doc__))
    return run_manual(parser.parse_args(argv), parser)


if __name__ == '__main__':
    raise SystemExit(main())
