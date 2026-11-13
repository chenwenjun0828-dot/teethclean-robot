"""Re-time, audit and dynamically validate a saved plan within a target duration."""
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import shutil
import numpy as np
from planning.infrastructure.io import dump, fingerprint
from ..config import Config


PROFILE_VERSION = 4


def run(source_dir, output, duration_s=90., *, jaw_opening_deg=None,
        trace_speed_mm_s=1., transfer_speed_mm_s=150.):
    """Re-clock a fixed spline once; retain all supplied IK nodes."""
    from .execution import load_verified
    from .fixed_path import build, TIMING_METHOD
    from .validation import validate
    if not np.isfinite(duration_s) or duration_s <= 0 or isinstance(duration_s, bool):
        raise ValueError('Target duration must be positive and finite')
    source_dir, output = Path(source_dir).resolve(), Path(output).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValueError('Use a new empty output directory')
    _, _, original, checker = load_verified(source_dir, jaw_opening_deg=jaw_opening_deg)
    report = json.loads((source_dir/'planning_report.json').read_text(encoding='utf-8'))
    request = json.loads((source_dir/'request.json').read_text(encoding='utf-8'))
    # Retiming changes the clock, retaining the source's validated joint limits.
    cfg = replace(Config.from_saved(report['config']), speed_profile='local',
                  speed_mm_s=trace_speed_mm_s, transfer_speed_mm_s=transfer_speed_mm_s)
    source_record = {'trajectory': fingerprint(source_dir/'trajectory.npz'),
                     'report': fingerprint(source_dir/'planning_report.json'), 'duration_s': original.duration}
    output.mkdir(parents=True, exist_ok=True)
    for name in ('geometry.json', 'geometry.npz', 'tooth_planes.json', 'tooth_lookup.npz',
                 'tooth_face_ids.npy', 'sweep_reference.json', 'perception_reference.json',
                 'pose_targets_world_m.npy', 'pose_joints_rad.npy', 'pose_parameters_m.npy', 'pose_search_report.json'):
        if (source_dir/name).exists():
            shutil.copy2(source_dir/name, output/name)
    profile = {'duration_s': duration_s, 'trace_speed_mm_s': trace_speed_mm_s,
               'transfer_speed_mm_s': transfer_speed_mm_s,
               'jaw_opening_deg': jaw_opening_deg, 'version': PROFILE_VERSION}
    request.update(config=asdict(cfg), retime_source=source_record, timing_profile=profile,
                   target_duration_s=duration_s, **cfg.motion_description())
    if (output/'perception_reference.json').is_file():
        reference_file = fingerprint(output/'perception_reference.json')
        request['perception_reference_file'] = report['perception_reference_file'] = reference_file
    for key in ('trace_audit', 'motion_audit', 'motion', 'trajectory_file', 'trace_velocity_scale',
                'spin_audit', 'fast_dynamics_verified', 'fast_execution_summary',
                'execution_report_file', 'dynamics_validation_file', 'retime_attempts',
                'downsampling', 'adaptive_trace_nodes_added'):
        report.pop(key, None)
    report.update(config=asdict(cfg), execution_ready=False, status='building_fixed_path',
                  retime_source=source_record, target_duration_s=duration_s, timing_profile=profile,
                  timing_method=TIMING_METHOD, schema_version=7,
                  **cfg.motion_description())
    dump(output/'request.json', request)
    dump(output/'planning_report.json', report)
    try:
        print(f'Fixed spline: {len(original.times)} retained waypoints; target {duration_s:g} s', flush=True)
        plan = build(original.joints, original.phases, original.teeth, checker.robot, cfg,
                     duration_s, fixed_trace=original.constant_trace)
        np.save(output/'trajectory_tcp_world_m.npy',
                [checker.robot.fk(q, full=True) for q in plan.joints])
        plan, report = validate(output, plan, report, checker.robot, checker, cfg)
        if plan is None:
            raise RuntimeError(report.get('error', 'Fixed-path validation failed'))
        return plan, report
    except (ValueError, RuntimeError) as error:
        report.update(execution_ready=False, status='fixed_path_validation_failed', error=str(error))
        dump(output/'planning_report.json', report)
        raise


def main(argv=None):
    from pipeline.parameters import add_parameter_file, parse_parameters
    parser = argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=90.)
    parser.add_argument('--jaw-opening-deg', type=float)
    parser.add_argument('--trace-speed-mm-s', type=float, default=1.)
    parser.add_argument('--transfer-speed-mm-s', type=float, default=150.)
    args = parse_parameters(parser, argv)
    try:
        plan, report = run(args.plan, args.out, args.duration, jaw_opening_deg=args.jaw_opening_deg,
                           trace_speed_mm_s=args.trace_speed_mm_s,
                           transfer_speed_mm_s=args.transfer_speed_mm_s)
        print(json.dumps({'duration_s': plan.duration, 'fast_dynamics_verified': report['fast_dynamics_verified'],
                          'phase_durations_s': report['phase_durations_s'],
                          'moving_internal_waypoints': report['motion']['moving_internal_waypoints']}))
        return 0
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Retiming failed: {error}', flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
