"""Load an independently validated planning trajectory and replay in MuJoCo."""
import argparse
from pathlib import Path
import numpy as np
from planning.infrastructure.poses import TCP_FRAME_ID
from ..pose_search.constraints import axis_convention
from ..config import Config, SweepOptions, CONTINUOUS_PATH_KINDS
from .fixed_path import TIMING_METHOD

# Recognize the retired timing label only when loading validated artifacts.
LEGACY_TIMING_METHOD = 'fixed_joint_cubic_constant_tcp'
SAVED_SPLINE_METHODS = (TIMING_METHOD, LEGACY_TIMING_METHOD)


def validate_motion_config(request, report, geometry):
    """A saved trajectory cannot switch offset/mode without replanning."""
    from dataclasses import asdict
    if report.get('path_kind') not in CONTINUOUS_PATH_KINDS:
        raise ValueError('Only fixed-angle continuous plans are supported; replan')
    if request.get('path_kind') != report.get('path_kind'):
        raise ValueError('Request/report path kind changed; replan')
    requested = SweepOptions.from_saved(request.get('sweep_options', {}))
    reported = SweepOptions.from_saved(report.get('sweep_options', {}))
    if requested.fixed_roll_deg != reported.fixed_roll_deg:
        raise ValueError('Request/report fixed roll changed; replan')
    cfg = Config.from_saved(report['config'])
    if asdict(cfg) != asdict(Config.from_saved(request['config'])):
        raise ValueError('Request/report motion configuration changed; replan')
    original = Config.from_saved(geometry['config'])
    if cfg.planning_mode != original.planning_mode or any(
            saved.get('planning_mode', 'continuous') != cfg.planning_mode
            for saved in (request, report)):
        raise ValueError('Planning mode differs from validated geometry/metadata; replan')
    if (cfg.simple_angle_deg != original.simple_angle_deg or cfg.simple_angle2_deg != original.simple_angle2_deg):
        raise ValueError('Simple rotation angles differ from validated geometry; replan')
    if cfg.region != original.region or any(
            saved.get('region', 'outer_front') != cfg.region for saved in (request, report, geometry)):
        raise ValueError('Selected region differs from validated geometry/metadata; replan')
    if (cfg.motion_mode != original.motion_mode or cfg.standoff_mm != original.standoff_mm):
        raise ValueError('Motion mode or standoff differs from validated geometry; replan')
    if (cfg.tilt_deg != original.tilt_deg or cfg.tilt_azimuth_deg != original.tilt_azimuth_deg):
        raise ValueError('Tool tilt or azimuth differs from validated geometry; replan')
    if not np.allclose(cfg.gravity_world, original.gravity_world, atol=1e-12):
        raise ValueError('Gravity direction differs from validated geometry; replan')
    for saved in (request, report):
        for key in ('motion_mode', 'standoff_mm'):
            if key in saved and saved[key] != getattr(cfg, key):
                raise ValueError('Saved motion mode/standoff metadata is inconsistent; replan')
    return cfg


def resolve_jaw_opening(report, request=None, override=None):
    """Prefer an explicit override, otherwise restore the opening saved with the plan."""
    if override is not None:
        return override
    request = request or {}
    for record in (report, report.get('timing_profile') or {}, request,
                   request.get('timing_profile') or {}):
        value = record.get('jaw_opening_deg')
        if value is not None:
            return value
    return None


def load_verified(directory, *, jaw_opening_deg=None):
    """Reject stale/modified exported plans before they can reach the servos."""
    import json
    from planning.infrastructure.io import fingerprint, body_transform
    from planning.infrastructure.robot import Robot
    from planning.infrastructure.collision import CollisionChecker
    from planning.infrastructure.trajectory import Trajectory
    from pipeline.simulation import create_simulation
    directory = Path(directory)
    report = json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
    request = json.loads((directory/'request.json').read_text(encoding='utf-8'))
    jaw_opening_deg = resolve_jaw_opening(report, request, jaw_opening_deg)
    if report.get('planner') != 'planning':
        raise ValueError('Expected planning report')
    if report.get('check_mode', 'full') != 'full' or report.get('collision_checking_enabled') is False:
        raise ValueError('This plan was produced with check_mode=geometry_only, which never evaluates '
                         'collision clearance; it cannot be executed. Replan with check_mode=full.')
    if fingerprint(directory/'tooth_planes.json')['sha256'] != report.get('planes_file',{}).get('sha256'):
        raise ValueError('Three-point planes changed since validation')
    if not report.get('execution_ready'):
        raise ValueError('This plan has not passed execution validation')
    if (report.get('axis_convention') != axis_convention(Config.from_saved(report['config']))
            or report.get('tcp_frame_id') != TCP_FRAME_ID):
        raise ValueError('Expected the saved planning mode axis convention')
    if report.get('schema_version', 0) >= 5 and (
            not report.get('motion_audit', {}).get('passed') or
            not report.get('trace_audit', {}).get('actual_time_interpolant')):
        raise ValueError('Actual continuous motion audit missing; replan')
    if not report.get('trace_audit', {}).get('passed'):
        raise ValueError('Independent trace audit is missing or failed; replan')
    if fingerprint(directory/'geometry.json')['sha256'] != report['geometry_file']['sha256']:
        raise ValueError('Geometry export changed since planning; replan')
    if 'trajectory_file' not in report or fingerprint(directory/'trajectory.npz')['sha256'] != report['trajectory_file']['sha256']:
        raise ValueError('Trajectory lacks a matching validation fingerprint; replan')
    for key in ('scene', 'robot_model', 'oral_settings'):
        saved = request[key]
        if fingerprint(saved['path'])['sha256'] != saved['sha256']:
            raise ValueError('Scene/configuration changed since planning: '+key)
    geometry = json.loads((directory/'geometry.json').read_text(encoding='utf-8'))
    cfg = validate_motion_config(request, report, geometry)
    if report['trace_audit'].get('orientation_enforced', True) != (cfg.orientation_audit == 'strict'):
        raise ValueError('Orientation audit policy differs from its validation report; replan')
    if cfg.planning_mode == 'simple' and (
            not report['trace_audit'].get('regional_angle_bounds_passed')
            or report['trace_audit'].get('regional_angle_limit_deg') != 5.):
        raise ValueError('Regional +/-5 degree FK validation is missing or failed; replan')
    if 'timing_profile' in report or report.get('timing_method') in SAVED_SPLINE_METHODS:
        if request.get('timing_profile') != report.get('timing_profile'):
            raise ValueError('Request/report timing profile changed; retime')
        proof = directory/'dynamics_validation.json'
        if (not report.get('fast_dynamics_verified') or not proof.is_file()
                or fingerprint(proof)['sha256'] != report.get('dynamics_validation_file', {}).get('sha256')):
            raise ValueError('Timed plan dynamics validation missing or changed; retime')
        dynamics = json.loads(proof.read_text(encoding='utf-8'))
        if (dynamics != report.get('fast_execution_summary') or dynamics.get('status') != 'completed'
                or dynamics.get('minimum_actual_clearance_mm', 0) <= 0):
            raise ValueError('Timed plan dynamics validation failed; retime')
    for key, saved in geometry['inputs'].items():
        if fingerprint(saved['path'])['sha256'] != saved['sha256']:
            raise ValueError('Geometry source changed since planning: '+key)
    if report.get('geometry_owner') == 'perception':
        actual = fingerprint(directory/'perception_reference.json')['sha256']
        if any(saved.get('perception_reference_file', {}).get('sha256') != actual
               for saved in (request, report)):
            raise ValueError('Perception reference changed since planning; replan')
    simulation_options = {'robot_mount_position_m': report['robot_mount_position_m']}
    if jaw_opening_deg is not None:
        from pipeline.oral import jaw_overrides, load_jaw_settings
        simulation_options['jaw_settings'] = jaw_overrides(load_jaw_settings(), jaw_opening_deg)
    model, data = create_simulation(**simulation_options)
    mount = model.body('robot_mount').id
    if (not np.allclose(model.body_pos[mount], report['robot_mount_position_m'], atol=1e-10)
            or not np.allclose(model.body_quat[mount], report['robot_mount_quaternion'], atol=1e-10)):
        raise ValueError('Robot mounting pose changed since planning')
    for jaw, transform in report['scene_tooth_transforms'].items():
        if not np.allclose(body_transform(model, data, jaw), transform, atol=1e-9):
            raise ValueError('Dental pose changed since planning')
    robot = Robot(model, data)
    jaw = request['inputs']['jaw']
    checker = CollisionChecker(robot, {k: np.asarray(v) for k, v in report['tooth_world_transforms'].items()},
                                {jaw: Path(request['inputs']['scan'])})
    trajectory = Trajectory.load(directory/'trajectory.npz')
    if (report.get('timing_method') in SAVED_SPLINE_METHODS) != (trajectory.constant_trace is not None):
        raise ValueError('Saved interpolation method differs from its validation report')
    if trajectory.constant_trace is not None:
        expected = TIMING_METHOD if trajectory.constant_trace.version == 2 else LEGACY_TIMING_METHOD
        if expected != report.get('timing_method'):
            raise ValueError('Saved speed profile differs from its validation report')
    if report.get('path_kind') in CONTINUOUS_PATH_KINDS:
        from .contract import load_sweep_reference, validate_sweep_contract
        validate_sweep_contract(trajectory, report, load_sweep_reference(directory, report))
    if trajectory.coefficients is not None and report.get('schema_version', 0) < 5:
        raise ValueError('Continuous interpolant lacks an actual-motion audit; replan')
    if trajectory.mount_position_m is not None and (
            not np.allclose(trajectory.mount_position_m, report['robot_mount_position_m'], rtol=0, atol=1e-10)
            or not np.allclose(trajectory.mount_position_m, request['selected_robot_mount_position_m'], rtol=0, atol=1e-10)):
        raise ValueError('Trajectory mount does not match validated request/report')
    return model, data, trajectory, checker

def main(argv=None):
    from pipeline.parameters import add_parameter_file, parse_parameters
    parser=argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--headless',action=argparse.BooleanOptionalAction,default=False)
    parser.add_argument('--jaw-opening-deg', type=float,
                        help='Restore a runtime jaw opening; must match the validated dental poses')
    parser.add_argument('--duration', type=float, default=90., help='Validated target duration in seconds (default: 90)')
    parser.add_argument('--trace-speed-mm-s', type=float, default=1.)
    parser.add_argument('--transfer-speed-mm-s', type=float, default=150.)
    parser.add_argument('--original-timing', action=argparse.BooleanOptionalAction, default=False,
                        help='Replay saved timing without retiming')
    args=parse_parameters(parser, argv)
    from control.position import execute
    import json
    try:
        directory = args.plan.resolve()
        if not args.original_timing:
            directory = prepare_timed_plan(directory, args.duration,
                jaw_opening_deg=args.jaw_opening_deg, trace_speed_mm_s=args.trace_speed_mm_s,
                transfer_speed_mm_s=args.transfer_speed_mm_s)
        model,data,plan,checker=load_verified(directory, jaw_opening_deg=args.jaw_opening_deg)
        request=json.loads((directory/'request.json').read_text(encoding='utf-8'))
        cfg=Config.from_saved(request['config'])
        saved=json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
        print(f'Execution plan: {directory}', flush=True)
        print(f'Planning mode: {cfg.planning_mode}', flush=True)
        print(f'Path kind: {saved["path_kind"]}', flush=True)
        print(f'Motion mode: {cfg.motion_mode}; standoff: {cfg.standoff_mm:g} mm; duration: {plan.duration:.3f} s', flush=True)
        if 'phase_durations_s' in saved:
            print('Phase durations (s): '+json.dumps(saved['phase_durations_s']), flush=True)
        if (args.headless and saved.get('timing_method') in SAVED_SPLINE_METHODS
                and saved.get('fast_dynamics_verified')):
            report = dict(saved['fast_execution_summary'])
            print('Using the completed headless dynamics run', flush=True)
        else:
            report=execute(model,data,plan,checker,directory,headless=args.headless)
        from planning.infrastructure.io import dump, fingerprint
        if (directory/'execution_report.json').is_file():
            saved['execution_report_file'] = fingerprint(directory/'execution_report.json')
            dump(directory/'planning_report.json', saved)
        print(report)
        return 0 if report['status']=='completed' else 1
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Execution failed: {error}', flush=True)
        return 1


def prepare_timed_plan(directory, duration_s=90., **options):
    """Cache only independently validated timing derived from the current source."""
    import hashlib
    import json
    from planning.infrastructure.io import fingerprint
    from .timing import PROFILE_VERSION, run
    directory = Path(directory).resolve()
    source_report = json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
    request_path = directory/'request.json'
    source_request = (json.loads(request_path.read_text(encoding='utf-8'))
                      if request_path.exists() else {})
    options['jaw_opening_deg'] = resolve_jaw_opening(
        source_report, source_request, options.get('jaw_opening_deg'))
    # Validate source fingerprints and scene even when reusing a cached result.
    load_verified(directory, jaw_opening_deg=options.get('jaw_opening_deg'))
    profile = {'duration_s': duration_s, 'trace_speed_mm_s': options.get('trace_speed_mm_s', 1.),
               'transfer_speed_mm_s': options.get('transfer_speed_mm_s', 150.),
               'jaw_opening_deg': options.get('jaw_opening_deg'), 'version': PROFILE_VERSION}
    for name in ('duration_s', 'trace_speed_mm_s', 'transfer_speed_mm_s'):
        if isinstance(profile[name], bool) or not np.isfinite(profile[name]) or profile[name] <= 0:
            raise ValueError(name+' must be positive and finite')
    if (source_report.get('timing_method') == TIMING_METHOD
            and source_report.get('fast_dynamics_verified')
            and abs(source_report['duration_s']-duration_s) < 1e-6
            and source_report['motion_audit']['max_trace_tcp_speed_mm_s'] <= profile['trace_speed_mm_s']+1e-8
            and source_report['motion_audit']['max_free_tcp_speed_mm_s'] <= profile['transfer_speed_mm_s']+1e-8):
        print(f'Reusing validated fixed-path plan: {directory}', flush=True)
        return directory
    if (source_report.get('timing_profile') == profile and source_report.get('fast_dynamics_verified')
            and abs(source_report['duration_s']-duration_s) < 1e-6):
        return directory
    source = {'trajectory': fingerprint(directory/'trajectory.npz'),
              'report': fingerprint(directory/'planning_report.json'),
              'duration_s': source_report['duration_s']}
    key = hashlib.sha256(json.dumps({'source': source, 'profile': profile}, sort_keys=True).encode()).hexdigest()[:10]
    output = directory.with_name(f'{directory.name}_{duration_s:g}s_{key}')
    if output.exists():
        cached = json.loads((output/'planning_report.json').read_text(encoding='utf-8'))
        if (cached.get('retime_source') != source or cached.get('timing_profile') != profile
                or not cached.get('fast_dynamics_verified') or not cached.get('execution_ready')
                or abs(cached.get('duration_s', 0)-duration_s) > 1e-6):
            raise ValueError(f'Cached timing is incomplete or stale; use planning.retime with a new --out: {output}')
        load_verified(output, jaw_opening_deg=options.get('jaw_opening_deg'))
        print(f'Reusing validated {duration_s:g} s plan: {output}', flush=True)
    else:
        print(f'Preparing validated {duration_s:g} s plan: {output}', flush=True)
        run(directory, output, duration_s, **options)
    return output


if __name__=='__main__':
    raise SystemExit(main())
