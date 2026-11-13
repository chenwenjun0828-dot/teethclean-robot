"""End-to-end MuJoCo gingival cleaning demonstration."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import tempfile
from time import perf_counter

from perception.global_camera import load_config
from perception.workflow import perceive_gingival_line
from perception.registration import resolve_registration_method
from pipeline.tracking import DEFAULT_JOINT_STEP_RAD, run_tracking, GEOMETRY_ONLY_STATUS
from pipeline.simulation import PROJECT_ROOT, create_simulation
from pipeline.oral import (MouthOpening, load_mouth_settings, mouth_overrides,
                            load_jaw_settings, jaw_overrides)
from pipeline.contracts import OutputPolicy
from planning.config import (Config, SweepOptions, REGIONS, PLANNING_MODES, CHECK_MODES,
                             GEOMETRY_ONLY, resolve_pose_angles, resolve_simple_angles)


def run_pipeline(jaw='mandibular', scan=None, config_path=None, output_dir=None,
                 path_index=0, offset_mm=1.5, speed_mm_s=15.,
                 collision_mode='mesh', headless=False, record_gif=False,
                 landmark_provider=None, registration_method=None, max_orientation_step_deg=None,
                 mouth_width_mm=None, mouth_height_mm=None, mouth_forward_mm=None,
                 no_mouth=False, mouth_distance_mm=None, jaw_opening_deg=None,
                 no_output=False, dry_run=False, fixed_roll_deg=None,
                 joint_step_rad=DEFAULT_JOINT_STEP_RAD, tilt_deg=15., tilt_azimuth_deg=0.,
                 region='outer_front', pose_angles_deg=None,
                 sweep_trim_mm=1., duration_s=None,
                 pose_spacing_mm=Config().pose_spacing_mm, speed_profile='local',
                 orientation_audit='strict', transfer_speed_mm_s=150., planning_mode='simple',
                 simple_angles_deg=(-75., -33.), check_mode='full',
                 regional_dataset=None, prediction_dir=None):
    """Extract, capture, register manually or automatically, and simulate one jaw."""
    if jaw not in ('mandibular', 'maxillary'):
        raise ValueError('Jaw must be mandibular or maxillary')
    if duration_s is not None and (isinstance(duration_s, bool)
            or not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError('duration_s must be positive and finite')
    if check_mode not in CHECK_MODES:
        raise ValueError('check_mode must be full or geometry_only')
    tilt_deg, tilt_azimuth_deg, fixed_roll_deg = resolve_pose_angles(
        pose_angles_deg, tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg,
        fixed_roll_deg=fixed_roll_deg)
    orientation_step = (Config().max_orientation_step_deg if max_orientation_step_deg is None
                        else max_orientation_step_deg)
    simple_angles_deg = resolve_simple_angles(simple_angles_deg)
    planning_config = Config(joint_step_rad=joint_step_rad, tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg,
           planning_mode=planning_mode, check_mode=check_mode,
           simple_angle_deg=simple_angles_deg[0], simple_angle2_deg=simple_angles_deg[1],
           pose_spacing_mm=pose_spacing_mm,
           speed_profile=speed_profile, orientation_audit=orientation_audit,
           transfer_speed_mm_s=transfer_speed_mm_s,
           region=region, standoff_mm=offset_mm, speed_mm_s=speed_mm_s,
           max_orientation_step_deg=orientation_step)
    sweep_options = SweepOptions(trim_mm=sweep_trim_mm,
                 fixed_roll_deg=fixed_roll_deg)
    policy = OutputPolicy(no_output=no_output, dry_run=dry_run)
    if policy.no_output and output_dir is not None:
        raise ValueError('--no-output cannot be combined with --output')
    config = load_config(config_path)
    mouth = mouth_overrides(load_mouth_settings(), mouth_width_mm, mouth_height_mm,
                           mouth_forward_mm, enabled=False if no_mouth else None,
                           distance_mm=mouth_distance_mm)
    jaw_settings = jaw_overrides(load_jaw_settings(), jaw_opening_deg)
    method = resolve_registration_method(config, registration_method)
    if method == 'automatic' and landmark_provider is not None:
        raise ValueError('Automatic registration does not accept a landmark provider')
    scan = Path(scan if scan is not None else config[jaw + '_mesh']).resolve()
    if not scan.is_file():
        raise FileNotFoundError(scan)
    model_path = Path(config['model']).resolve()
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    from perception.regions.io import Inputs
    from perception.regions.result import validate_region_inputs
    region_inputs = Inputs.defaults(jaw)
    region_inputs.scan = scan
    if prediction_dir is not None:
        region_inputs.prediction = Path(prediction_dir).resolve()
    dataset = validate_region_inputs(region_inputs, regional_dataset)
    regional_dataset = Path(dataset.metadata['dataset_dir'])
    if dry_run:
        return {'status': 'dry_run', 'jaw': jaw, 'region': region, 'scan': str(scan),
                'path_source': 'regional', 'geometry_owner': 'perception',
                'model': str(model_path), 'registration_method': method,
                'planning_mode': planning_mode, 'fixed_roll_deg': fixed_roll_deg,
                'check_mode': check_mode, 'collision_checking_enabled': check_mode != GEOMETRY_ONLY,
                'pose_angles_deg': [tilt_deg, tilt_azimuth_deg, fixed_roll_deg],
                'joint_step_rad': joint_step_rad,
                'pose_spacing_mm': pose_spacing_mm,
                'speed_profile': speed_profile, 'orientation_audit': orientation_audit,
                'transfer_speed_mm_s': transfer_speed_mm_s,
                'max_orientation_step_deg': orientation_step,
                'sweep_trim_mm': sweep_trim_mm,
                'standoff_mm': offset_mm,
                'target_duration_s': duration_s,
                'tilt_deg': tilt_deg, 'tilt_azimuth_deg': tilt_azimuth_deg}
    if no_output:
        with tempfile.TemporaryDirectory(prefix='teethclean_pipeline_') as temp_dir:
            return run_pipeline(
                jaw=jaw, scan=scan, config_path=config_path, output_dir=Path(temp_dir),
                path_index=path_index, offset_mm=offset_mm, speed_mm_s=speed_mm_s,
                collision_mode=collision_mode, headless=headless, record_gif=record_gif,
                landmark_provider=landmark_provider, registration_method=registration_method,
                max_orientation_step_deg=max_orientation_step_deg,
                mouth_width_mm=mouth_width_mm, mouth_height_mm=mouth_height_mm,
                mouth_forward_mm=mouth_forward_mm, no_mouth=no_mouth,
                mouth_distance_mm=mouth_distance_mm, jaw_opening_deg=jaw_opening_deg,
                fixed_roll_deg=fixed_roll_deg,
                joint_step_rad=joint_step_rad, tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg,
                region=region, sweep_trim_mm=sweep_trim_mm,
                duration_s=duration_s, pose_spacing_mm=pose_spacing_mm,
                speed_profile=speed_profile, orientation_audit=orientation_audit,
                transfer_speed_mm_s=transfer_speed_mm_s, planning_mode=planning_mode,
                simple_angles_deg=simple_angles_deg, check_mode=check_mode,
                regional_dataset=regional_dataset, prediction_dir=prediction_dir)
    output_dir = Path(output_dir) if output_dir else (
        PROJECT_ROOT / 'outputs' / 'pipeline' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    )
    output_dir = output_dir.resolve()
    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise FileExistsError(f'Output directory must be empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / 'pipeline_report.json'
    run_started = perf_counter()
    stage_started = {}
    report = {'status': 'running', 'stage': 'extraction', 'jaw': jaw, 'region': region,
              'path_source': 'regional', 'geometry_owner': 'perception',
              'registration_method': method,
              'planning_mode': planning_mode,
              'pose_angles_deg': [tilt_deg, tilt_azimuth_deg, fixed_roll_deg],
              'fixed_roll_deg': fixed_roll_deg,
              'joint_step_rad': joint_step_rad,
              'pose_spacing_mm': pose_spacing_mm,
              'speed_profile': speed_profile, 'orientation_audit': orientation_audit,
              'transfer_speed_mm_s': transfer_speed_mm_s,
              'max_orientation_step_deg': orientation_step,
              'sweep_trim_mm': sweep_trim_mm,
              'standoff_mm': offset_mm,
              'target_duration_s': duration_s,
              'tilt_deg': tilt_deg, 'tilt_azimuth_deg': tilt_azimuth_deg,
              'scan': str(scan), 'collision_mode': collision_mode,
              'dental_collision_enabled': collision_mode == 'mesh',
              'dental_collision_validated': False,
              'jaw_opening': {'opening_deg': jaw_settings.opening_deg,
                              'hinge_center_mm': jaw_settings.hinge_center_mm,
                              'reference': 'original_scan_pose', 'moving_jaw': 'mandibular'},
              'simulation_only': True, 'hardware_execution_allowed': False,
              'mouth_opening': {'enabled': mouth.enabled, 'width_mm': mouth.width_mm,
                                'height_mm': mouth.height_mm, 'center_mm': mouth.center_mm,
                                'distance_mm': mouth.distance_mm,
                                'collision_enabled': False},
              'stages': {}}

    def save_report():
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2,
                                          allow_nan=False), encoding='utf-8')

    def on_stage(name, details=None):
        report['stage'] = name
        if details is None:
            stage_started[name] = perf_counter()
        else:
            details = dict(details)
            if name in stage_started:
                details['wall_time_s'] = perf_counter() - stage_started.pop(name)
            report['stages'][name] = details
        save_report()

    save_report()
    try:
        started = perf_counter()
        model, data = create_simulation(model_path, mouth_settings=mouth,
                                         jaw_settings=jaw_settings)
        report['stages']['scene_setup'] = {'wall_time_s': perf_counter() - started}
        if 'dental_mouth_opening' in [model.body(i).name for i in range(model.nbody)]:
            scene_mouth = MouthOpening(model)
            report['mouth_opening'].update(center_mm=scene_mouth.center_mm(mouth).tolist(),
                                            distance_mm=scene_mouth.distance_mm(mouth))
        perception_started = perf_counter()
        perception = perceive_gingival_line(model, data, jaw, scan, config, output_dir,
                                            landmark_provider=landmark_provider,
                                            on_stage=on_stage, registration_method=method,
                                            region_config=planning_config, sweep_options=sweep_options,
                                            prediction_dir=prediction_dir,
                                            regional_dataset=regional_dataset)
        report['stages']['perception_total'] = {
            'wall_time_s': perf_counter() - perception_started}
        report['stage'] = 'planning_and_execution'
        save_report()
        tracking = run_tracking(perception.registration_dir, jaw, path_index, offset_mm,
                                speed_mm_s, collision_mode, headless,
                                output_dir / 'tracking', perception.topology_path,
                                record_gif, model_data=(model, data),
                                max_orientation_step_deg=max_orientation_step_deg,
                                fixed_roll_deg=fixed_roll_deg,
                                joint_step_rad=joint_step_rad,
                                tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg, region=region,
                                sweep_trim_mm=sweep_trim_mm,
                                duration_s=duration_s, jaw_opening_deg=jaw_settings.opening_deg,
                                pose_spacing_mm=pose_spacing_mm,
                                speed_profile=speed_profile, orientation_audit=orientation_audit,
                                transfer_speed_mm_s=transfer_speed_mm_s, planning_mode=planning_mode,
                                simple_angles_deg=simple_angles_deg, check_mode=check_mode,
                                perception_result=perception)
        report['stages']['planning'] = {
            'status': tracking.get('planning_status'),
            'mode': planning_mode,
            'check_mode': check_mode,
            'path_kind': tracking.get('path_kind'),
            'continuity': tracking.get('continuity'),
            'wall_time_s': tracking.get('planning_wall_time_s'),
            'substage_wall_time_s': tracking.get('planning_substage_times_s', {})}
        report['stages']['execution'] = {
            'status': tracking['status'],
            'wall_time_s': tracking.get('execution_wall_time_s'),
            # No dynamics run happens without collision checking, so the planned
            # timing is reported separately from an actually simulated duration.
            'simulated_duration_s': (None if check_mode == GEOMETRY_ONLY
                                     else tracking.get('duration_s')),
            'planned_duration_s': tracking.get('duration_s')}
        report['stages']['execution']['collision_checked'] = tracking.get('collision_checking_enabled', True)
        if duration_s is not None:
            report['stages']['retiming'] = {
                'wall_time_s': tracking.get('retiming_wall_time_s'),
                'original_duration_s': tracking.get('original_duration_s'),
                'target_duration_s': duration_s,
                'execution_plan_dir': tracking.get('execution_plan_dir')}
        report['stages']['tracking'] = {'status': tracking['status'],
                                         'wall_time_s': tracking.get('tracking_wall_time_s'),
                                         'tracking_setup_wall_time_s': tracking.get('tracking_setup_wall_time_s'),
                                         'duration_s': tracking.get('duration_s'),
                                         'trace_rms_mm': tracking.get('trace_rms_mm'),
                                         'trace_rotation_max_deg': tracking.get('trace_rotation_max_deg'),
                                         'coverage_fraction': tracking.get('coverage_fraction')}
        report['status'] = tracking['status']
        report['check_mode'] = check_mode
        report['collision_checking_enabled'] = check_mode != GEOMETRY_ONLY
        report['dental_collision_validated'] = (
            collision_mode == 'mesh' and check_mode != GEOMETRY_ONLY
            and tracking['status'] == 'completed'
        )
        report['stage'] = 'done'
        return report
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        raise
    finally:
        report['total_wall_time_s'] = perf_counter() - run_started
        save_report()


def _print_timing_summary(report):
    stages = report.get('stages', {})
    print('Wall-clock stage timings:')
    for name in ('scene_setup', 'extraction', 'capture', 'registration', 'perception_total'):
        elapsed = stages.get(name, {}).get('wall_time_s')
        if elapsed is not None:
            print(f'  {name}: {elapsed:.2f} s')
    for name in ('tracking_setup_wall_time_s',):
        elapsed = stages.get('tracking', {}).get(name)
        if elapsed is not None:
            print(f'  tracking setup: {elapsed:.2f} s')
    planning = stages.get('planning', {})
    if planning.get('wall_time_s') is not None:
        print(f"  planning total: {planning['wall_time_s']:.2f} s")
    for name, elapsed in planning.get('substage_wall_time_s', {}).items():
        print(f'    {name}: {elapsed:.2f} s')
    retiming = stages.get('retiming', {})
    if retiming.get('wall_time_s') is not None:
        print(f"  retiming and dynamics validation: {retiming['wall_time_s']:.2f} s")
    execution = stages.get('execution', {})
    if execution.get('wall_time_s') is not None:
        print(f"  execution wall time: {execution['wall_time_s']:.2f} s")
    if execution.get('simulated_duration_s') is not None:
        print(f"  trajectory simulation time: {execution['simulated_duration_s']:.2f} s")
    elif execution.get('planned_duration_s') is not None:
        print(f"  planned trajectory duration (not simulated): "
              f"{execution['planned_duration_s']:.2f} s")
    if report.get('total_wall_time_s') is not None:
        print(f"  pipeline total: {report['total_wall_time_s']:.2f} s")


def build_argparser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', type=Path,
                        help='Pipeline run parameters JSON; explicit CLI options override its values')
    parser.add_argument('--jaw', choices=('mandibular', 'maxillary'), default='mandibular')
    parser.add_argument('--region', choices=REGIONS, default='outer_front')
    parser.add_argument('--planning-mode', choices=PLANNING_MODES, default='simple',
                        help='continuous: follow local surface frame; simple: regional fixed frame with +/-5 degree corrections')
    parser.add_argument('--simple-angles-deg', type=float, nargs=2, default=(-75., -33.),
                        metavar=('TANGENT', 'LOCAL_Y'), help=argparse.SUPPRESS)
    parser.add_argument('--check-mode', choices=CHECK_MODES, default='full',
                        help='full: plan and execute with mesh collision checking (default); '
                             'geometry_only: plan poses and paths without any collision query, to test '
                             'whether the geometry is reachable. Skips collision audit and dynamics '
                             'execution; never execution-ready')
    parser.add_argument('--pose-angles-deg', type=float, nargs=3, metavar=('ANGLE', 'AZIMUTH', 'FLIP'),
                        help='Fixed gravity-Z angle, fixed (inward-normal cross Z) azimuth, tool-X flip (degrees)')
    parser.add_argument('--scan', type=Path, help='Colored oral-scan PLY; defaults to scene config')
    parser.add_argument('--regional-dataset', type=Path)
    parser.add_argument('--prediction-dir', type=Path)
    parser.add_argument('--config', type=Path, help='Global-camera registration config')
    parser.add_argument('--output', type=Path, help='Empty output directory')
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument('--no-output', action='store_true',
                             help='Run the complete pipeline and remove all run files afterwards')
    output_mode.add_argument('--dry-run', action='store_true',
                             help='Validate inputs and configuration without running simulation')
    parser.add_argument('--offset-mm', type=float, default=1.5,
                        help='Noncontact offset along the reference normal (default: 1.5 mm)')
    parser.add_argument('--speed-mm-s', type=float, default=15.,
                        help='Nominal TCP speed along the gingival trace in simulation (mm/s)')
    parser.add_argument('--duration', type=float,
                        help='Validated target motion duration in seconds, e.g. 90; '
                             'omitted: derive timing from speed limits. Planning time is excluded')
    parser.add_argument('--speed-profile', choices=['local'], default='local',
                        help='Fixed joint spline with TOPPRA local slowdown')
    parser.add_argument('--orientation-audit', choices=['strict', 'diagnostic'], default='strict',
                        help='Record orientation errors without rejecting the simulation in diagnostic mode')
    parser.add_argument('--transfer-speed-mm-s', type=float, default=150.,
                        help='Entry/exit TCP speed cap in mm/s (default: 150)')
    parser.add_argument('--flip-deg', '--fixed-roll-deg', dest='fixed_roll_deg', type=float,
                        help='Flip about tool +X, within +/-180 degrees')
    parser.add_argument('--angle-deg', '--tilt-deg', dest='tilt_deg', type=float, default=15.,
                        help='Angle about fixed projected-gravity Z, within +/-180 degrees')
    parser.add_argument('--azimuth-deg', '--tilt-azimuth-deg', dest='tilt_azimuth_deg', type=float, default=0.,
                        help='Angle about fixed inward-normal cross Z, within +/-180 degrees')
    parser.add_argument('--joint-step-rad', type=float, default=DEFAULT_JOINT_STEP_RAD,
                        help='Maximum adjacent IK joint change; timed speed and collision checks still apply '
                             f'(default: {DEFAULT_JOINT_STEP_RAD:g} rad)')
    parser.add_argument('--pose-spacing-mm', type=float, default=Config().pose_spacing_mm,
                        help='Pose-stage sampling interval in mm (default: 0.05); geometry sampling is unchanged')
    parser.add_argument('--max-orientation-step-deg', type=float,
                        help='Maximum adjacent target rotation for IK connectivity (default: 20 degrees); '
                             'fixed poses, speed and collision audits still apply')
    parser.add_argument('--sweep-trim-mm', type=float, default=1.,
                        help='Initial source endpoint trim in mm (default: 1)')
    parser.add_argument('--headless', action=argparse.BooleanOptionalAction, default=False,
                        help='Hide execution viewer; manual registration still opens the point picker')
    parser.add_argument('--mouth-width-mm', type=float, help='Clear oval opening width, 10-110 mm')
    parser.add_argument('--jaw-opening-deg', type=float,
                        help='Lower-jaw angle relative to original scan pose, -10 to 40 degrees')
    parser.add_argument('--mouth-height-mm', type=float, help='Clear oval opening height, 10-130 mm')
    position = parser.add_mutually_exclusive_group()
    position.add_argument('--mouth-forward-mm', type=float,
                        help='Mask centre along scan +Z in mm; default 45, toward the front camera')
    position.add_argument('--mouth-distance-mm', type=float,
                          help='Gap from frontmost tooth-model vertex to opening plane, in mm (>=0)')
    parser.add_argument('--no-mouth', action='store_true', help='Hide the white mouth mask')
    parser.add_argument('--registration-method', choices=('manual', 'automatic'),
                        help='Override registration method in config; automatic requires no point picking')
    return parser


def main(argv=None):
    from pipeline.parameters import parse_parameters
    args = parse_parameters(build_argparser(), argv)
    try:
        report = run_pipeline(args.jaw, args.scan, args.config, args.output,
                              0, args.offset_mm, args.speed_mm_s,
                              'mesh', args.headless, False,
                              registration_method=args.registration_method,
                              mouth_width_mm=args.mouth_width_mm,
                              mouth_height_mm=args.mouth_height_mm,
                              mouth_forward_mm=args.mouth_forward_mm, no_mouth=args.no_mouth,
                              mouth_distance_mm=args.mouth_distance_mm,
                              jaw_opening_deg=args.jaw_opening_deg,
                              no_output=args.no_output, dry_run=args.dry_run,
                              fixed_roll_deg=args.fixed_roll_deg,
                              joint_step_rad=args.joint_step_rad, region=args.region,
                              pose_spacing_mm=args.pose_spacing_mm,
                              speed_profile=args.speed_profile, orientation_audit=args.orientation_audit,
                              transfer_speed_mm_s=args.transfer_speed_mm_s,
                              planning_mode=args.planning_mode,
                              simple_angles_deg=args.simple_angles_deg,
                              check_mode=args.check_mode,
                              regional_dataset=args.regional_dataset,
                              prediction_dir=args.prediction_dir,
                              max_orientation_step_deg=args.max_orientation_step_deg,
                              sweep_trim_mm=args.sweep_trim_mm,
                              duration_s=args.duration,
                              pose_angles_deg=args.pose_angles_deg,
                              tilt_deg=args.tilt_deg, tilt_azimuth_deg=args.tilt_azimuth_deg)
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Pipeline failed: {error}', file=sys.stderr)
        return 1
    if report['status'] == 'dry_run':
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    print(f"Pipeline {report['status']}")
    _print_timing_summary(report)
    if report.get('check_mode') == GEOMETRY_ONLY:
        print('Geometry-only run: poses and paths were checked without collisions; '
              'no executable plan was produced. Rerun without --check-mode geometry_only '
              'to validate clearance and dynamics.', flush=True)
    # A geometry-only verification is a completed check, not an executed plan.
    return 0 if report['status'] in ('completed', 'dry_run', GEOMETRY_ONLY_STATUS) else 1


if __name__ == '__main__':
    raise SystemExit(main())
