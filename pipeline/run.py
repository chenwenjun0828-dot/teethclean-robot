"""End-to-end MuJoCo gingival cleaning demonstration."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

from perception.global_camera import load_config
from perception.workflow import perceive_gingival_line
from perception.registration import resolve_registration_method
from pipeline.tracking import run_tracking
from pipeline.simulation import PROJECT_ROOT, create_simulation
from pipeline.mouth import load_settings, with_overrides


def run_pipeline(jaw='mandibular', scan=None, config_path=None, output_dir=None,
                 path_index=0, offset_mm=0., speed_mm_s=4.,
                 collision_mode='geometry', headless=False, record_gif=False,
                 landmark_provider=None, registration_method=None, max_orientation_step_deg=45.,
                 mouth_width_mm=None, mouth_height_mm=None, mouth_forward_mm=None,
                 no_mouth=False):
    """Extract, capture, register manually or automatically, and simulate one jaw."""
    if jaw not in ('mandibular', 'maxillary'):
        raise ValueError('Jaw must be mandibular or maxillary')
    config = load_config(config_path)
    mouth = with_overrides(load_settings(), mouth_width_mm, mouth_height_mm,
                           mouth_forward_mm, enabled=False if no_mouth else None)
    method = resolve_registration_method(config, registration_method)
    if method == 'automatic' and landmark_provider is not None:
        raise ValueError('Automatic registration does not accept a landmark provider')
    scan = Path(scan if scan is not None else config[jaw + '_mesh']).resolve()
    if not scan.is_file():
        raise FileNotFoundError(scan)
    output_dir = Path(output_dir) if output_dir else (
        PROJECT_ROOT / 'outputs' / 'pipeline' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    )
    output_dir = output_dir.resolve()
    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise FileExistsError(f'Output directory must be empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / 'pipeline_report.json'
    report = {'status': 'running', 'stage': 'extraction', 'jaw': jaw,
              'registration_method': method,
              'scan': str(scan), 'collision_mode': collision_mode,
              'dental_collision_enabled': collision_mode == 'strict',
              'dental_collision_validated': False,
              'simulation_only': True, 'hardware_execution_allowed': False,
              'mouth_opening': {'enabled': mouth.enabled, 'width_mm': mouth.width_mm,
                                'height_mm': mouth.height_mm, 'center_mm': mouth.center_mm,
                                'collision_enabled': False},
              'stages': {}}

    def save_report():
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                          allow_nan=False), encoding='utf-8')

    def on_stage(name, details=None):
        report['stage'] = name
        if details is not None:
            report['stages'][name] = details
        save_report()

    save_report()
    try:
        model, data = create_simulation(Path(config['model']), mouth_settings=mouth)
        perception = perceive_gingival_line(model, data, jaw, scan, config, output_dir,
                                            landmark_provider=landmark_provider,
                                            on_stage=on_stage, registration_method=method)
        report['stage'] = 'planning_and_execution'
        save_report()
        tracking = run_tracking(perception.registration_dir, jaw, path_index, offset_mm,
                                speed_mm_s, collision_mode, headless,
                                output_dir / 'tracking', perception.topology_path,
                                record_gif, model_data=(model, data),
                                max_orientation_step_deg=max_orientation_step_deg)
        report['stages']['tracking'] = {'status': tracking['status'],
                                         'duration_s': tracking.get('duration_s'),
                                         'trace_rms_mm': tracking.get('trace_rms_mm'),
                                         'trace_rotation_max_deg': tracking.get('trace_rotation_max_deg'),
                                         'orientation_filter': tracking.get('orientation_filter')}
        report['status'] = tracking['status']
        report['dental_collision_validated'] = (
            collision_mode == 'strict' and tracking['status'] == 'completed'
        )
        report['stage'] = 'done'
        return report
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        save_report()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jaw', choices=('mandibular', 'maxillary'), default='mandibular')
    parser.add_argument('--scan', type=Path, help='Colored oral-scan PLY; defaults to scene config')
    parser.add_argument('--config', type=Path, help='Global-camera registration config')
    parser.add_argument('--output', type=Path, help='Empty output directory')
    parser.add_argument('--path-index', type=int, default=0)
    parser.add_argument('--offset-mm', type=float, default=0.)
    parser.add_argument('--speed-mm-s', type=float, default=4.)
    parser.add_argument('--max-orientation-step-deg', type=float, default=45.,
                        help='Skip tooth orientation jumps above this angle; default 45 degrees')
    parser.add_argument('--no-orientation-filter', action='store_true',
                        help='Require exact tooth orientation at every point, rejecting unreachable poses')
    parser.add_argument('--collision-mode', choices=('geometry', 'strict'), default='geometry')
    parser.add_argument('--headless', action='store_true',
                        help='Hide execution viewer; manual registration still opens the point picker')
    parser.add_argument('--record-gif', action='store_true')
    parser.add_argument('--mouth-width-mm', type=float, help='Clear oval opening width, 10-110 mm')
    parser.add_argument('--mouth-height-mm', type=float, help='Clear oval opening height, 10-130 mm')
    parser.add_argument('--mouth-forward-mm', type=float,
                        help='Mask centre along scan +Z in mm; default 45, toward the front camera')
    parser.add_argument('--no-mouth', action='store_true', help='Hide the white mouth mask')
    parser.add_argument('--registration-method', choices=('manual', 'automatic'),
                        help='Override registration method in config; automatic requires no point picking')
    args = parser.parse_args(argv)
    try:
        report = run_pipeline(args.jaw, args.scan, args.config, args.output,
                              args.path_index, args.offset_mm, args.speed_mm_s,
                              args.collision_mode, args.headless, args.record_gif,
                              registration_method=args.registration_method,
                              max_orientation_step_deg=(None if args.no_orientation_filter
                                                        else args.max_orientation_step_deg),
                              mouth_width_mm=args.mouth_width_mm,
                              mouth_height_mm=args.mouth_height_mm,
                              mouth_forward_mm=args.mouth_forward_mm, no_mouth=args.no_mouth)
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Pipeline failed: {error}', file=sys.stderr)
        return 1
    print(f"Pipeline {report['status']}")
    return 0 if report['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
