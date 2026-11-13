"""Compose the recorded nominal pose, hand-eye calibration and both jaw registrations."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parents[2]


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def fingerprint(path):
    path = Path(path).resolve()
    name = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
    return dict(path=name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def transform(points, matrix):
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def save_matrix(out, name, matrix):
    np.testing.assert_allclose(matrix[3], [0, 0, 0, 1], atol=1e-10)
    np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
    assert np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6)
    np.save(out / (name + '.npy'), matrix)
    np.savetxt(out / (name + '.txt'), matrix, fmt='%.12f',
               header=name + '; translation=m; nominal kinematics; not independently validated')


def save_cloud(path, cloud, matrix):
    result = o3d.geometry.PointCloud(cloud)
    result.scale(.001, center=(0, 0, 0))
    result.transform(matrix)
    if not o3d.io.write_point_cloud(str(path), result):
        raise RuntimeError('Could not save ' + str(path))


def convert(jaw_runs, fk_path, handeye_path, sdk_path, *, handeye_label='25', output=None):
    jaw_runs = {jaw: Path(run).resolve() for jaw, run in jaw_runs.items()}
    if not jaw_runs or not set(jaw_runs).issubset({'mandibular', 'maxillary'}):
        raise ValueError('Provide at least one valid jaw registration run')
    fk_path, handeye_path, sdk_path = map(Path, (fk_path, handeye_path, sdk_path))
    fk, handeye, sdk = read(fk_path), read(handeye_path), read(sdk_path)
    matches = [c for c in handeye['calibrations'] if str(c['label']) == str(handeye_label)]
    if len(matches) != 1:
        raise ValueError('Select exactly one hand-eye calibration label')
    chosen = matches[0]
    wrist_color = np.array(chosen['T_wrist3_camera_color_frame_m'])
    color_depth = np.array(sdk['T_color_frame_depth_optical_ros_convention_m'])
    base_wrist = np.array(fk['T_controller_base_wrist3_nominal_m'])
    ros_wrist = np.array(fk['T_base_link_wrist3_nominal_m'])
    base_depth = base_wrist @ wrist_color @ color_depth
    ros_depth = ros_wrist @ wrist_color @ color_depth
    captures = {jaw: read(run / 'capture/capture_metadata.json') for jaw, run in jaw_runs.items()}
    reference_capture = next(iter(captures.values()))
    for capture in captures.values():
        assert capture['camera_serial'] == sdk['device']['serial']
        assert capture['host_received_at_utc'] == reference_capture['host_received_at_utc']
        assert capture['acquisition_id'] == reference_capture['acquisition_id']
        np.testing.assert_allclose(capture['T_color_depth_m'], sdk['T_color_optical_depth_optical_m'], atol=1e-7)
    out = Path(output) if output is not None else ROOT / 'outputs/perception/base_conversion_nominal' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    if out.exists() and any(out.iterdir()):
        raise ValueError('Use a new empty conversion output directory')
    out.mkdir(parents=True, exist_ok=True)
    for name, matrix in [('T_controller_base_wrist3_nominal_m', base_wrist),
                         ('T_base_link_wrist3_nominal_m', ros_wrist),
                         ('T_wrist3_color_frame_m', wrist_color),
                         ('T_color_frame_depth_optical_ros_convention_m', color_depth),
                         ('T_controller_base_depth_optical_nominal_m', base_depth),
                         ('T_base_link_depth_optical_nominal_m', ros_depth)]:
        save_matrix(out, name, matrix)
    summary = dict(status='converted_with_nominal_kinematics', robot_execution_allowed=False,
        units='m', matrix_convention='p_target = R @ p_source + t', camera_serial=sdk['device']['serial'],
        capture_time_utc=reference_capture['host_received_at_utc'], handeye_label=str(handeye_label),
        sources=[fingerprint(p) for p in (fk_path, handeye_path, sdk_path)], jaws={},
        assumptions=['Joint angles correspond to capture pose; robot stationary until angles recorded.',
                     f'Hand-eye result {handeye_label} belongs to the same camera and unchanged mounting.',
                     'User confirmed camera_color_frame follows the standard ROS convention.'],
        limitations=['Factory robot kinematic correction not applied.',
                     'Registration and base coordinates not independently validated.',
                     'Gingival CSV is a point set, not a connected or executable robot trajectory.',
                     'No tool TCP offsets, target tool orientations or motion planning applied.'])
    for jaw, run in jaw_runs.items():
        folder = out / jaw
        folder.mkdir()
        registration = run / 'registration'
        report = read(registration / 'registration_report.json')
        assert report['status'] == 'completed'
        scan_path = ROOT / f'assets/source_cad/teeth/{jaw}_scan.ply'
        camera_path = registration / 'camera_mm.ply'
        assert fingerprint(scan_path)['sha256'] == report['scan_sha256']
        assert fingerprint(camera_path)['sha256'] == report['camera_sha256']
        scan = o3d.io.read_point_cloud(str(scan_path))
        camera = o3d.io.read_point_cloud(str(camera_path))
        depth_scan_path = run / 'transforms/T_camera_scan_m.npy'
        depth_scan = np.load(depth_scan_path)
        base_scan, ros_scan = base_depth @ depth_scan, ros_depth @ depth_scan
        save_matrix(folder, 'T_depth_optical_scan_m', depth_scan)
        save_matrix(folder, 'T_controller_base_scan_nominal_m', base_scan)
        save_matrix(folder, 'T_base_link_scan_nominal_m', ros_scan)
        points_scan_m = np.asarray(scan.points) / 1000.
        sequential = transform(transform(transform(transform(points_scan_m, depth_scan), color_depth), wrist_color), base_wrist)
        combined = transform(points_scan_m, base_scan)
        np.testing.assert_allclose(sequential, combined, atol=1e-12)
        np.testing.assert_allclose(transform(combined, np.linalg.inv(base_scan)), points_scan_m, atol=1e-12)
        save_cloud(folder / 'scan_controller_base_nominal_m.ply', scan, base_scan)
        save_cloud(folder / 'camera_roi_controller_base_nominal_m.ply', camera, base_depth)
        save_cloud(folder / 'scan_base_link_nominal_m.ply', scan, ros_scan)
        line_path = ROOT / f'assets/perception/{jaw}/gingival_line_points.csv'
        line = np.genfromtxt(line_path, delimiter=',', names=True, encoding='utf-8-sig')
        points = np.column_stack([np.atleast_1d(line[k]) for k in ('x', 'y', 'z')]) / 1000.
        for frame, matrix in [('controller_base', base_scan), ('base_link', ros_scan)]:
            np.savetxt(folder / f'gingival_points_{frame}_nominal_m.csv', transform(points, matrix),
                       delimiter=',', header=f'frame={frame}; units=m; status=nominal; point set only\nx_m,y_m,z_m')
        summary['jaws'][jaw] = dict(registration_run=fingerprint(run / 'run_report.json')['path'],
            sources=[fingerprint(p) for p in (scan_path, camera_path, depth_scan_path, line_path,
                                               run / 'capture/capture_metadata.json')],
            scan_points=len(scan.points), camera_points=len(camera.points), gingival_points=len(points),
            checks=['Input fingerprints and capture/device identity checked.',
                    'Matrix chain equals sequential point conversion within 1e-12 m.',
                    'Inverse transform restores original scan points within 1e-12 m.'],
            scan_controller_base_bbox_m=[combined.min(axis=0).tolist(), combined.max(axis=0).tolist()])
    summary['output_sha256'] = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in out.rglob('*') if p.is_file()}
    (out / 'conversion_report.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('Saved:', out)
    print(json.dumps(summary['jaws'], indent=2))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mandibular-run', type=Path)
    parser.add_argument('--maxillary-run', type=Path)
    parser.add_argument('--fk-report', type=Path, required=True)
    parser.add_argument('--handeye-inventory', type=Path, required=True)
    parser.add_argument('--camera-extrinsics', type=Path, required=True)
    parser.add_argument('--handeye-label', default='25')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args(argv)
    runs = {j: p for j, p in [('mandibular', args.mandibular_run), ('maxillary', args.maxillary_run)] if p is not None}
    convert(runs, args.fk_report, args.handeye_inventory, args.camera_extrinsics,
            handeye_label=args.handeye_label, output=args.out)


if __name__ == '__main__':
    main()
