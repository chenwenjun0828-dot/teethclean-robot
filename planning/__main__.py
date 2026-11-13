"""Regional planning with surface-following or regionally fixed orientation."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path
import json
import sys
import tempfile

import numpy as np

from .infrastructure.io import Inputs, dump, ROOT
from .config import (Config, SweepOptions, REGIONS, PLANNING_MODES, CHECK_MODES,
                     GEOMETRY_ONLY, resolve_pose_angles)
from .pipeline import run
from perception.regions import PreparedRegion, prepare_region
from perception.regions.result import resolve_regional_dataset, validate_region_inputs
from pipeline.oral import jaw_overrides, load_jaw_settings
from pipeline.contracts import OutputPolicy
from pipeline.parameters import add_parameter_file, parse_parameters


def build_argparser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--jaw', choices=['mandibular', 'maxillary'], default='mandibular')
    parser.add_argument('--region', choices=REGIONS, default='outer_front')
    parser.add_argument('--planning-mode', choices=PLANNING_MODES, default='simple',
                        help='continuous: follow local surface frame; simple: regional fixed frame with +/-5 degree corrections')
    parser.add_argument('--simple-angles-deg', type=float, nargs=2, default=(-75., -33.),
                        metavar=('TANGENT', 'LOCAL_Y'), help=argparse.SUPPRESS)
    parser.add_argument('--pose-angles-deg', type=float, nargs=3, metavar=('ANGLE', 'AZIMUTH', 'FLIP'),
                        help='Fixed gravity-Z angle, fixed (inward-normal cross Z) azimuth, tool-X flip (degrees)')
    parser.add_argument('--out', type=Path, required=False)
    parser.add_argument('--scan', type=Path)
    parser.add_argument('--topology', type=Path)
    parser.add_argument('--prediction-dir', type=Path)
    parser.add_argument('--perception-result', type=Path,
                        help='Prepared perception region directory or perception_region.json')
    parser.add_argument('--regional-dataset', type=Path,
                        help='Six-region source dataset, used during perception preprocessing')
    parser.add_argument('--world-from-scan', type=Path)
    parser.add_argument('--jaw-opening-deg', type=float,
                        help='Set the lower-jaw opening angle in degrees (default: oral_settings.py)')
    parser.add_argument('--geometry-only', action=argparse.BooleanOptionalAction, default=False,
                        help='Stop after geometry export (no pose search, IK, timing or collision)')
    parser.add_argument('--check-mode', choices=CHECK_MODES, default='full',
                        help='full: plan with collision checking (default); '
                             'geometry_only: plan without any collision query to test whether the '
                             'pose/path geometry is reachable. Never execution-ready')
    parser.add_argument('--sweep-trim-mm', type=float, default=1.)
    parser.add_argument('--flip-deg', '--fixed-roll-deg', dest='fixed_roll_deg', type=float, help='Flip about tool +X, in degrees')
    parser.add_argument('--motion-mode', choices=['non-contact', 'contact'])
    parser.add_argument('--standoff-mm', type=float)
    parser.add_argument('--angle-deg', '--tilt-deg', dest='tilt_deg', type=float, default=15., help='Angle about fixed projected-gravity Z')
    parser.add_argument('--azimuth-deg', '--tilt-azimuth-deg', dest='tilt_azimuth_deg', type=float, default=0.,
                        help='Angle about fixed inward-normal cross Z')
    parser.add_argument('--joint-step-rad', type=float, default=.15,
                        help='Maximum adjacent IK joint change; interpolation and collision audits remain enabled')
    parser.add_argument('--pose-spacing-mm', type=float, default=Config().pose_spacing_mm,
                        help='Pose-stage sampling interval in mm (default: 0.05); geometry sampling is unchanged')
    parser.add_argument('--max-orientation-step-deg', type=float, default=Config().max_orientation_step_deg,
                        help='Maximum adjacent target rotation for IK connectivity (default: 20 degrees)')
    parser.add_argument('--mount-position-mm', type=float, nargs=3)
    parser.add_argument('--execute', action=argparse.BooleanOptionalAction, default=False,
                        help='Run validated trajectory in MuJoCo only')
    parser.add_argument('--duration', type=float, help='Target total duration including entry and exit, e.g. 90')
    parser.add_argument('--speed-profile', choices=['local'], default='local',
                        help='Fixed joint spline with TOPPRA local slowdown')
    parser.add_argument('--orientation-audit', choices=['strict', 'diagnostic'], default='strict',
                        help='Diagnostic mode records orientation error without rejecting the simulation')
    parser.add_argument('--speed-mm-s', type=float, default=1., help='Cleaning TCP speed cap in mm/s')
    parser.add_argument('--transfer-speed-mm-s', type=float, default=150., help='Entry/exit TCP speed cap in mm/s')
    parser.add_argument('--headless', action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument('--no-output', action='store_true', help='Run using an ephemeral workspace without saving artifacts')
    parser.add_argument('--save-output', dest='no_output', action='store_false', default=False,
                        help='Override no_output=true in a parameter file')
    parser.add_argument('--dry-run', action=argparse.BooleanOptionalAction, default=False,
                        help='Validate inputs and configuration without planning')
    return parser


def main(argv=None):
    parser = build_argparser()
    args = parse_parameters(parser, argv)
    supplied = sys.argv[1:] if argv is None else argv
    try:
        OutputPolicy(no_output=args.no_output, dry_run=args.dry_run)
    except ValueError as error:
        parser.error(str(error))
    if args.execute and (args.no_output or args.dry_run):
        parser.error('--execute requires a persistent planning output')
    if args.geometry_only and args.execute:
        parser.error('Cannot execute geometry-only output')
    if args.check_mode == GEOMETRY_ONLY and args.execute:
        parser.error('--check-mode geometry_only never produces an executable plan; '
                     'its trajectory only verifies pose/path reachability')
    if args.check_mode == GEOMETRY_ONLY and args.geometry_only:
        parser.error('--check-mode geometry_only plans poses and joints; it cannot be combined '
                     'with --geometry-only, which stops before pose search')
    if not args.no_output and not args.dry_run and args.out is None:
        parser.error('--out is required unless --no-output or --dry-run is used')
    if args.out is not None and args.out.exists() and any(args.out.iterdir()):
        parser.error('Use a new empty output directory')

    inputs = Inputs.defaults(args.jaw)
    for name, value in [('scan', args.scan), ('topology', args.topology), ('prediction', args.prediction_dir)]:
        if value is not None:
            setattr(inputs, name, value.resolve())
    try:
        prepared = None
        if args.perception_result is not None:
            if any(value is not None for value in (args.scan, args.topology, args.prediction_dir, args.regional_dataset)):
                raise ValueError('--perception-result cannot be combined with raw scan/path inputs')
            prepared = PreparedRegion.load(args.perception_result)
            if 'region' not in args._file_parameters and not any(value.split('=', 1)[0] == '--region' for value in supplied):
                args.region = prepared.region
            if args.region != prepared.region:
                raise ValueError('Requested region differs from prepared perception output')
            inputs = prepared.inputs
            if ('jaw' in args._file_parameters or any(value.split('=', 1)[0] == '--jaw' for value in supplied)) and args.jaw != inputs.jaw:
                raise ValueError('Requested jaw differs from prepared perception output')
            args.jaw = inputs.jaw
        jaw_settings = jaw_overrides(load_jaw_settings(), args.jaw_opening_deg)
        tilt, azimuth, roll = resolve_pose_angles(args.pose_angles_deg, tilt_deg=args.tilt_deg,
                                                tilt_azimuth_deg=args.tilt_azimuth_deg,
                                                fixed_roll_deg=args.fixed_roll_deg)
        cfg = Config(motion_mode=args.motion_mode, standoff_mm=args.standoff_mm,
                     planning_mode=args.planning_mode, check_mode=args.check_mode,
                     simple_angle_deg=args.simple_angles_deg[0], simple_angle2_deg=args.simple_angles_deg[1],
                     pose_spacing_mm=args.pose_spacing_mm,
                     speed_profile=args.speed_profile, orientation_audit=args.orientation_audit,
                     speed_mm_s=args.speed_mm_s, transfer_speed_mm_s=args.transfer_speed_mm_s,
                     joint_speed_rad_s=2.2 if args.duration is not None else Config().joint_speed_rad_s,
                     joint_accel_rad_s2=12. if args.duration is not None else Config().joint_accel_rad_s2,
                     orientation_speed_deg_s=200. if args.duration is not None else Config().orientation_speed_deg_s,
                     tilt_deg=tilt, region=args.region,
                     joint_step_rad=args.joint_step_rad, tilt_azimuth_deg=azimuth,
                     max_orientation_step_deg=args.max_orientation_step_deg)
        options = SweepOptions(trim_mm=args.sweep_trim_mm,
                               fixed_roll_deg=roll)
        if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
            raise ValueError('--duration must be positive and finite')
        if prepared is None:
            validate_region_inputs(inputs, args.regional_dataset)
        if args.dry_run:
            for path in (() if prepared is not None else (inputs.scan, inputs.prediction)):
                if path is not None and not Path(path).is_file() and not Path(path).is_dir():
                    raise FileNotFoundError(path)
            print(json.dumps({'status': 'validated', 'mode': 'dry-run',
                              'jaw': args.jaw, 'jaw_opening_deg': jaw_settings.opening_deg,
                              'region': cfg.region, 'config': asdict(cfg),
                              'planning_mode': cfg.planning_mode,
                              'target_duration_s': args.duration,
                              'sweep_options': asdict(options) if options is not None else None}, indent=2))
            return 0
        mount = None if args.mount_position_mm is None else np.asarray(args.mount_position_mm) * .001
        world = None if args.world_from_scan is None else np.load(args.world_from_scan, allow_pickle=False)
        if prepared is None:
            dataset = resolve_regional_dataset(inputs, args.regional_dataset)
            prepared = prepare_region(inputs, cfg, options, world_from_scan=world, regional_dataset=dataset)
        if args.no_output:
            with tempfile.TemporaryDirectory(prefix='dental_planning_') as directory:
                _, plan, report, scene = run(prepared, Path(directory), cfg, args.geometry_only,
                                             mount, world, options,
                                             jaw_opening_deg=jaw_settings.opening_deg, duration_s=args.duration)
        else:
            _, plan, report, scene = run(prepared, args.out, cfg, args.geometry_only,
                                         mount, world, options,
                                         jaw_opening_deg=jaw_settings.opening_deg, duration_s=args.duration)
        print(json.dumps({k: report[k] for k in ('status', 'coverage_fraction',
                                                  'region', 'jaw_opening_deg',
                                                  'selected_teeth', 'pose_mode', 'planning_mode',
                                                  'check_mode', 'collision_checking_enabled',
                                                  'execution_ready',
                                                  'all_selected_teeth_have_motion', 'duration_s')
                          if k in report}, default=str), flush=True)
        if args.execute:
            if plan is None or not report['execution_ready']:
                raise RuntimeError('No validated trajectory')
            from control.position import execute
            model, data, checker = scene
            if not (args.headless and report.get('fast_dynamics_verified')):
                execute(model, data, plan, checker, args.out, headless=args.headless)
                from planning.infrastructure.io import fingerprint
                report['execution_report_file'] = fingerprint(args.out/'execution_report.json')
                dump(args.out/'planning_report.json', report)
        if args.check_mode == GEOMETRY_ONLY:
            # 0 means "the geometry check ran and passed"; the trajectory is not executable.
            return 0 if report.get('status') == 'geometry_only_verified' else 2
        return 0 if args.geometry_only or plan is not None else 2
    except (ValueError, RuntimeError, OSError) as error:
        if args.out is not None and args.out.exists() and not args.no_output:
            dump(args.out / 'failure.json', {'status': 'failed', 'error': str(error)})
        print(f'planning failed: {error}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
