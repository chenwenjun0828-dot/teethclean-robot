"""Adapt frozen perception results to regional motion planning and control."""
from pathlib import Path
import json
from time import perf_counter
from planning.infrastructure.io import Inputs, ROOT, dump, fingerprint
from planning.config import (Config, SweepOptions, continuous_path_kind, resolve_pose_angles,
                             resolve_simple_angles, CHECK_MODES, GEOMETRY_ONLY)
from planning.motion_planning.fixed_path import TIMING_METHOD
from planning.pipeline import run as plan
from planning.motion_planning.contract import load_sweep_reference, validate_sweep_contract
from perception.gingival_line import load_registered_paths
from perception.frames import camera_world_transform_m
from control.position import execute
import numpy as np


DEFAULT_JOINT_STEP_RAD = .3
GEOMETRY_ONLY_STATUS = 'geometry_only_verified'


def run_tracking(registration_dir, jaw='mandibular', path_index=0, offset_mm=1.5,
                 speed_mm_s=15., collision_mode='mesh', headless=False, output_dir=None,
                 topology_path=None, record_gif=False, model_data=None,
                 max_orientation_step_deg=None, prediction_dir=None,
                 fixed_roll_deg=None, joint_step_rad=DEFAULT_JOINT_STEP_RAD,
                 tilt_deg=15., tilt_azimuth_deg=0., region='outer_front', pose_angles_deg=None,
                 sweep_trim_mm=1., duration_s=None, jaw_opening_deg=None,
                 pose_spacing_mm=Config().pose_spacing_mm, speed_profile='local',
                 orientation_audit='strict', transfer_speed_mm_s=150., planning_mode='simple',
                 simple_angles_deg=(-75., -33.), check_mode='full', perception_result=None):
    if duration_s is not None and (isinstance(duration_s, bool)
            or not np.isfinite(duration_s) or duration_s <= 0):
        raise ValueError('duration_s must be positive and finite')
    if check_mode not in CHECK_MODES:
        raise ValueError('check_mode must be full or geometry_only')
    tilt_deg, tilt_azimuth_deg, fixed_roll_deg = resolve_pose_angles(
        pose_angles_deg, tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg,
        fixed_roll_deg=fixed_roll_deg)
    options = SweepOptions(fixed_roll_deg=fixed_roll_deg, trim_mm=sweep_trim_mm)
    if max_orientation_step_deg is None:
        max_orientation_step_deg = Config().max_orientation_step_deg
    simple_angles_deg = resolve_simple_angles(simple_angles_deg)
    cfg = Config(standoff_mm=offset_mm, speed_mm_s=speed_mm_s, joint_step_rad=joint_step_rad,
                 planning_mode=planning_mode, check_mode=check_mode,
                 simple_angle_deg=simple_angles_deg[0], simple_angle2_deg=simple_angles_deg[1],
                 pose_spacing_mm=pose_spacing_mm,
                 speed_profile=speed_profile, orientation_audit=orientation_audit,
                 tilt_deg=tilt_deg, tilt_azimuth_deg=tilt_azimuth_deg, region=region,
                 max_orientation_step_deg=max_orientation_step_deg,
                 joint_speed_rad_s=2.2 if duration_s is not None else Config().joint_speed_rad_s,
                 joint_accel_rad_s2=12. if duration_s is not None else Config().joint_accel_rad_s2,
                 orientation_speed_deg_s=200. if duration_s is not None else Config().orientation_speed_deg_s,
                 transfer_speed_mm_s=transfer_speed_mm_s)
    if model_data is None or output_dir is None:
        raise ValueError('Tracking requires the capture scene and an output directory')
    if path_index != 0:
        raise ValueError('Select one of the six canonical regions with region; path-index is retired')
    if collision_mode != 'mesh':
        raise ValueError('Use collision-mode mesh: the rebuilt planner checks actual tooth meshes')
    if record_gif:
        raise ValueError('GIF export is retired; use the planning viewer for the new trajectory')
    tracking_started = perf_counter()
    registration_dir = Path(registration_dir)
    registration = json.loads((registration_dir/'registration_report.json').read_text(encoding='utf-8-sig'))
    inputs = Inputs.defaults(jaw)
    inputs.scan = Path(registration['config'][jaw+'_mesh'])
    if not inputs.scan.is_absolute(): inputs.scan = ROOT/inputs.scan
    inputs.topology = Path(topology_path) if topology_path else Path(registration['config'][jaw+'_line']).parent/'gingival_line_segments.obj'
    if not inputs.topology.is_absolute(): inputs.topology = ROOT/inputs.topology
    if prediction_dir is not None: inputs.prediction = Path(prediction_dir)
    from perception.regions import PreparedRegion
    if isinstance(perception_result, PreparedRegion):
        prepared = perception_result
    elif perception_result is not None:
        name = region
        if name not in perception_result.regions:
            raise ValueError(f'Perception did not produce region {region}')
        prepared = perception_result.regions[name]
    else:
        manifest_path = registration_dir.parent/'regions'/'perception_regions.json'
        if not manifest_path.is_file():
            raise ValueError('Tracking requires perception regional output; rerun perception first')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        name = region
        if name not in manifest['regions']:
            raise ValueError(f'Perception did not produce region {region}')
        prepared = PreparedRegion.load_from_manifest(manifest_path, name)
    prepared.validate(region)
    if prepared.inputs.jaw != jaw or fingerprint(prepared.inputs.scan)['sha256'] != fingerprint(inputs.scan)['sha256']:
        raise ValueError('Tracking registration and perception source scan differ')
    bundle = load_registered_paths(registration_dir, jaw, inputs.topology)
    model, data = model_data
    if not np.allclose(camera_world_transform_m(data, bundle.camera_name), bundle.world_from_camera, atol=1e-6):
        raise ValueError('Camera moved since capture; a new registration is required')
    setup_wall_time_s = perf_counter() - tracking_started
    planning_started = perf_counter()
    output_dir = Path(output_dir)
    planning_dir = output_dir
    geometry_only = check_mode == GEOMETRY_ONLY
    print(f'Planning mode: {planning_mode}; region: {region}; check mode: {check_mode}', flush=True)
    if geometry_only:
        print('Collision checking disabled: this run verifies pose and path geometry only '
              'and produces no executable plan', flush=True)
    if planning_mode == 'simple':
        print(f'Regional fixed Z-angle/Y-azimuth/X-flip: {[cfg.tilt_deg, cfg.tilt_azimuth_deg, fixed_roll_deg]}; '
              'collision corrections bounded to +/-5 degrees per angle', flush=True)
    else:
        print(f'Fixed gravity-Z angle: {cfg.tilt_deg:g} degrees; fixed Y azimuth: {cfg.tilt_azimuth_deg:g} degrees', flush=True)
    print(f'IK orientation step limit: {cfg.max_orientation_step_deg:g} degrees; '
          f'region endpoint trim: {options.trim_mm:g} mm', flush=True)
    if planning_mode == 'continuous' and fixed_roll_deg is not None:
        print(f'Tool +X flip: {fixed_roll_deg:g} degrees', flush=True)
    # Explicit duration requests use the same simulation profile as the standalone planner.
    _, trajectory, report, scene = plan(
        prepared, planning_dir, cfg,
        world_from_scan=bundle.world_from_scan, model_data=model_data,
        sweep_options=options, jaw_opening_deg=jaw_opening_deg, duration_s=duration_s)
    planning_wall_time_s = perf_counter() - planning_started
    if geometry_only:
        # No clearance was measured and no dynamics run happened, so only a
        # completed geometry-only verification may continue.
        if trajectory is None or report.get('status') != GEOMETRY_ONLY_STATUS:
            reasons = dict.fromkeys(attempt['reason'] for attempt in report.get('attempts', [])
                                    if attempt.get('reason'))
            detail = (report.get('error') or report.get('failure')
                      or '; '.join(reasons) or report.get('status'))
            raise ValueError(f'No reachable {region}-region {planning_mode} geometry-only plan: {detail}')
    elif trajectory is None or not report.get('execution_ready'):
        reasons = dict.fromkeys(attempt['reason'] for attempt in report.get('attempts', [])
                                if attempt.get('reason'))
        detail = (report.get('error') or report.get('failure')
                  or '; '.join(reasons) or report.get('status'))
        raise ValueError(f'No executable {region}-region {planning_mode} plan: {detail}')
    if report.get('path_kind') != continuous_path_kind(region):
        raise ValueError('Continuous planning returned a different path kind')
    if report.get('planning_mode', 'continuous') != planning_mode:
        raise ValueError('Planning returned a different planning mode')
    if report.get('check_mode', 'full') != check_mode:
        raise ValueError('Planning returned a different collision check mode')
    continuity = validate_sweep_contract(
        trajectory, report, load_sweep_reference(planning_dir, report))
    original_duration_s = trajectory.duration
    if geometry_only:
        result = {'status': GEOMETRY_ONLY_STATUS, 'simulation_only': True, 'execution_ready': False,
                  'check_mode': check_mode, 'collision_checking_enabled': False,
                  'clearance_measured': False, 'dental_collision_validated': False,
                  'dynamics_verified': False, 'duration_s': trajectory.duration,
                  'max_rotation_error_deg': None, 'max_position_error_mm': None,
                  'minimum_actual_clearance_mm': None,
                  'trajectory_file': report.get('trajectory_file'),
                  'note': 'Pose and path geometry verified without collision checking; '
                          'replan with check_mode=full before execution'}
        result.update(coverage_fraction=report['coverage_fraction'],
                      trace_rotation_max_deg=None,
                      planning_mode=planning_mode,
                      region=region,
                      pose_angles_deg=[cfg.tilt_deg, cfg.tilt_azimuth_deg, fixed_roll_deg],
                      regional_orientation=report.get('regional_orientation'),
                      joint_step_rad=joint_step_rad,
                      pose_spacing_mm=cfg.pose_spacing_mm,
                      max_orientation_step_deg=cfg.max_orientation_step_deg,
                      sweep_trim_mm=options.trim_mm,
                      standoff_mm=cfg.standoff_mm,
                      target_duration_s=duration_s, original_duration_s=original_duration_s,
                      source_plan_dir=str(planning_dir.resolve()),
                      execution_plan_dir=None,
                      retiming_wall_time_s=0.,
                      tilt_deg=cfg.tilt_deg, tilt_azimuth_deg=cfg.tilt_azimuth_deg,
                      path_kind=report['path_kind'],
                      continuity=continuity,
                      planning_status=report.get('status'),
                      planning_wall_time_s=planning_wall_time_s,
                      planning_substage_times_s=report.get('timings_wall_s', {}),
                      motion_audit=report.get('motion_audit'),
                      trace_audit=report.get('trace_audit'),
                      tracking_setup_wall_time_s=setup_wall_time_s,
                      execution_wall_time_s=0.,
                      tracking_wall_time_s=perf_counter()-tracking_started)
        dump(Path(output_dir)/'tracking_report.json', result)
        return result
    retiming_wall_time_s = 0.
    if duration_s is not None:
        if (not report.get('fast_dynamics_verified') or abs(trajectory.duration-duration_s) > 1e-6):
            raise ValueError('Target-duration trajectory has not passed execution validation')
    _, _, checker = scene
    execution_started = perf_counter()
    if headless and report.get('timing_method') == TIMING_METHOD and report.get('fast_dynamics_verified'):
        result = dict(report['fast_execution_summary'])
        print('Using the completed headless dynamics run; no second simulation is needed', flush=True)
    else:
        result = execute(model, data, trajectory, checker, output_dir, headless=headless)
    execution_wall_time_s = perf_counter() - execution_started
    if (output_dir/'execution_report.json').is_file():
        report['execution_report_file'] = fingerprint(output_dir/'execution_report.json')
        dump(output_dir/'planning_report.json', report)
    result.update(coverage_fraction=report['coverage_fraction'], dental_collision_validated=result['status']=='completed',
                  trace_rotation_max_deg=result['max_rotation_error_deg'],
                  planning_mode=planning_mode,
                  region=region,
                  pose_angles_deg=[cfg.tilt_deg, cfg.tilt_azimuth_deg, fixed_roll_deg],
                  regional_orientation=report.get('regional_orientation'),
                  joint_step_rad=joint_step_rad,
                  pose_spacing_mm=cfg.pose_spacing_mm,
                  max_orientation_step_deg=cfg.max_orientation_step_deg,
                  sweep_trim_mm=options.trim_mm,
                  standoff_mm=cfg.standoff_mm,
                  target_duration_s=duration_s, original_duration_s=original_duration_s,
                  source_plan_dir=str(planning_dir.resolve()),
                  execution_plan_dir=str(output_dir.resolve()),
                  retiming_wall_time_s=retiming_wall_time_s,
                  tilt_deg=cfg.tilt_deg, tilt_azimuth_deg=cfg.tilt_azimuth_deg,
                  path_kind=report['path_kind'],
                  continuity=continuity,
                  planning_status=report.get('status'),
                  planning_wall_time_s=planning_wall_time_s,
                  planning_substage_times_s=report.get('timings_wall_s', {}),
                  tracking_setup_wall_time_s=setup_wall_time_s,
                  execution_wall_time_s=execution_wall_time_s,
                  tracking_wall_time_s=perf_counter()-tracking_started)
    dump(Path(output_dir)/'tracking_report.json', result)
    return result
