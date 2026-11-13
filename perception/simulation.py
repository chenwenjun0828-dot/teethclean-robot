"""Standalone MuJoCo perception, without planning or robot execution."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .artifacts import default_output, export_coordinates
from .global_camera import ensure_empty_directory, load_config, write_json


def run_simulation(output_dir, *, jaw='mandibular', config_path=None, scan=None,
                   show_view=False):
    # Keep MuJoCo and the simulation workflow out of real-camera imports.
    from pipeline.simulation import create_simulation
    from .workflow import perceive_gingival_line

    config = load_config(config_path)
    scan = Path(scan or config[jaw + '_mesh']).resolve()
    out = ensure_empty_directory(output_dir)
    report = dict(source='simulation', status='running', robot_execution_allowed=False)
    try:
        model, data = create_simulation(Path(config['model']))
        result = perceive_gingival_line(model, data, jaw, scan, config, out,
                                       registration_method='automatic')
        capture = json.loads((result.capture_dir / 'capture_metadata.json').read_text())
        frozen = json.loads((out / 'registration_config.json').read_text())
        camera_from_scan = np.load(result.registration_dir / 'T_camera_scan_estimated.npy')
        artifacts = export_coordinates(out, camera_from_scan, line=frozen[jaw + '_line'],
            world_from_camera_m=np.asarray(capture['T_world_camera_cv_m']))
        report.update(status='completed', coordinate_outputs=artifacts,
                      regions_manifest=str(result.regions_manifest), registration_dir=str(result.registration_dir))
        write_json(out / 'run_report.json', report)
        if show_view:
            import open3d as o3d
            camera = o3d.io.read_point_cloud(str(result.capture_dir / 'camera_mm.ply'))
            registered = o3d.io.read_point_cloud(str(result.registration_dir / 'scan_registered_camera_mm.ply'))
            o3d.visualization.draw_geometries([camera, registered.paint_uniform_color([.2,.85,.3])],
                                             window_name='Simulated perception registration')
    except Exception as error:
        report.update(status='failed', error=str(error))
        write_json(out / 'run_report.json', report)
        raise
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jaw', choices=('mandibular', 'maxillary'), default='mandibular')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--scan', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--view', action='store_true', help='Show registration overlay, no robot motion')
    args = parser.parse_args(argv)
    try:
        out = run_simulation(args.out or default_output('simulation'), jaw=args.jaw,
            config_path=args.config, scan=args.scan, show_view=args.view)
        print(f'Saved simulated perception: {out}')
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as error:
        print(f'Simulated perception failed: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
