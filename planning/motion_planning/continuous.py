"""Connect the approach and fit one fixed spline through selected IK poses."""
from dataclasses import asdict
import numpy as np
from planning.infrastructure.io import dump, fingerprint, body_transform
from planning.infrastructure.poses import TCP_FRAME_ID
from planning.infrastructure.transfer import initial_approach, final_retreat
from copy import deepcopy
from perception.regions.continuous import SweepPath
from ..pose_search.constraints import axis_convention
from ..pose_search.continuous import SweepFailure, sample_sweep, solve_sweep
from ..config import REGIONS, continuous_path_kind
from .fixed_path import build, TIMING_METHOD


def plan_sweep(geometry, robot, checker, world, cfg, output, options, jaw_opening_deg=None, duration_s=None,
               *, reference):
    path_kind = continuous_path_kind(cfg.region)
    selected_teeth = list(map(int, geometry.report['regions'][cfg.region]))
    report = {'schema_version': 7, 'planner': 'planning', 'path_kind': path_kind,
              'status': 'planning_continuous_sweep', 'execution_ready': False, 'simulation_only': True,
              'region': cfg.region, 'selected_teeth': selected_teeth,
              'jaw_opening_deg': jaw_opening_deg,
              'planning_mode': cfg.planning_mode,
              'pose_mode': 'regional_fixed_bounded' if cfg.planning_mode == 'simple' else 'fixed_angles',
              'config': asdict(cfg), 'sweep_options': asdict(options),
              **cfg.motion_description(), 'axis_convention': axis_convention(cfg), 'tcp_frame_id': TCP_FRAME_ID,
              'normal_angle_policy': f'{cfg.tilt_deg:g} degrees about fixed projected-gravity Z',
              'tilt_azimuth_policy': f'{cfg.tilt_azimuth_deg:g} degrees about fixed inward-normal cross Z',
              'pose_angle_names': ['fixed_z_angle', 'fixed_y_azimuth', 'tool_x_flip'],
              'world_from_scan_m': world, 'robot_joint_names': robot.joint_names, 'scene_qpos': robot.initial_qpos,
              'robot_mount_position_m': robot.model.body_pos[robot.model.body('robot_mount').id].copy(),
              'robot_mount_quaternion': robot.model.body_quat[robot.model.body('robot_mount').id].copy(),
              'tooth_world_transforms': checker.dental_transforms,
              'scene_tooth_transforms': {jaw: body_transform(robot.model, robot.data, jaw)
                                         for jaw in ('mandibular', 'maxillary')},
              'collision_method': ('Disabled (check_mode=geometry_only): reachability only, never execution-ready'
                                   if cfg.geometry_only else
                                   'Full mesh FCL dental clearance and MuJoCo self contacts'),
              'check_mode': cfg.check_mode,
              'collision_checking_enabled': not cfg.geometry_only,
              'sweep_max_joint_step_rad': cfg.sweep_step_rad,
              'attempts': [], 'motion_segments': [], 'strokes': geometry.report['strokes'],
              'rejected_intervals': [], 'executed_regions': [cfg.region],
              'unplanned_regions': [region for region in REGIONS if region != cfg.region]}
    if cfg.planning_mode == 'simple':
        report['normal_angle_policy'] = 'One representative source frame for the entire selected region'
        report['tilt_azimuth_policy'] = 'Fixed regional gravity Z/Y, then tool-X flip; corrections bounded to +/-5 degrees each'
    dump(output/'planning_report.json', report)
    if not cfg.geometry_only and not checker.check(robot.home, cfg.clearance_mm*.001).valid:
        raise ValueError('Initial robot configuration is in collision')
    if reference.record.get('region') != cfg.region:
        raise ValueError('Perception line region differs from requested planning region')
    candidates = [(float(reference.record['trim_mm']), {})]
    for trim, endpoint_trims in candidates:
        attempt = {'trim_mm': float(trim), 'endpoint_trims_mm': endpoint_trims}
        report['attempts'].append(attempt)
        try:
            path = SweepPath(deepcopy(reference.record))
            path.record.update(planning_mode=cfg.planning_mode)
            path.record['options']['fixed_roll_deg'] = options.fixed_roll_deg
            if cfg.planning_mode == 'simple':
                from ..simple import orient_reference
                path = orient_reference(path, geometry, cfg, options, world=world)
            if cfg.planning_mode == 'simple':
                report['regional_orientation'] = path.record['regional_orientation']
            dump(output/f'sweep_candidate_{len(report["attempts"])}.json', path.record)
            parameters = sample_sweep(path, cfg)
            print(f'Continuous sweep: trim {trim:g} mm, {len(parameters)} positions; '
                  f'pose spacing {cfg.pose_spacing_mm:g} mm', flush=True)
            selected = solve_sweep(path, parameters, robot, checker, world, cfg,
                                   options.fixed_roll_deg)
            if cfg.planning_mode == 'simple':
                report['regional_orientation'] = path.record['regional_orientation']
                dump(output/f'sweep_candidate_{len(report["attempts"])}.json', path.record)
            print(f'Whole-region IK path found; fitting one fixed joint spline through '
                  f'{len(parameters)} poses', flush=True)
            trace = [(s, node['q'], node['roll']) for s, node in zip(parameters, selected)]
            np.save(output/'pose_targets_world_m.npy', [node['pose'] for node in selected])
            np.save(output/'pose_joints_rad.npy', [node['q'] for node in selected])
            np.save(output/'pose_parameters_m.npy', parameters)
            dump(output/'pose_search_report.json', {
                'status': 'completed', 'pose_spacing_mm': cfg.pose_spacing_mm,
                'planning_mode': cfg.planning_mode,
                'pose_count': len(parameters),
                'targets_file': fingerprint(output/'pose_targets_world_m.npy'),
                'joints_file': fingerprint(output/'pose_joints_rad.npy'),
                'parameters_file': fingerprint(output/'pose_parameters_m.npy')})
            transition = initial_approach(robot, checker, cfg, trace[0][1],
                                          world[:3, :3]@path.normal(0.))
            if transition is None: raise SweepFailure('no_initial_approach')
            joints, phases, teeth = [robot.home.copy()], [], []
            for phase, bank in transition['segments']:
                for q in bank:
                    if np.max(abs(q-joints[-1])) < 1e-10: continue
                    joints.append(q); phases.append(phase); teeth.append(path.tooth(0.))
            start = len(joints)-1
            if not np.allclose(joints[-1], trace[0][1], atol=1e-8, rtol=0):
                raise SweepFailure('approach_endpoint_mismatch')
            for a, b in zip(trace[:-1], trace[1:]):
                joints.append(b[1]); phases.append('trace'); teeth.append(path.tooth((a[0]+b[0])/2))
            end = len(joints)-1
            retreat = final_retreat(robot, checker, cfg, joints[-1], world[:3, :3]@path.normal(path.end))
            if retreat is None:
                raise SweepFailure('no_final_retreat')
            for q in retreat:
                joints.append(q); phases.append('retreat'); teeth.append(path.tooth(path.end))
            dump(output/'sweep_reference.json', path.record)
            segment = {'status': 'planned', 'reference_kind': path_kind, 'stroke_id': -1,
                       'tooth': 0, 'teeth': selected_teeth,
                       'length_mm': path.record['sweep_length_mm'], 'parameters': [float(x[0]) for x in trace],
                       'rolls_deg': [float(x[2]) for x in trace], 'samples': len(trace),
                       'trace_start_waypoint': start, 'trace_end_waypoint': end,
                       'transfer_method': transition['method'],
                       'retreat_mm': cfg.retreat_mm}
            report['roll_summary'] = {'mode': 'fixed', 'fixed_roll_deg': options.fixed_roll_deg,
                                     'total_roll_variation_deg': 0.}
            if cfg.planning_mode == 'simple':
                rolls = np.asarray([x[2] for x in trace])
                report['roll_summary'].update(mode=path.record['regional_orientation']['search_mode'],
                    total_roll_variation_deg=float(abs(np.diff(rolls)).sum()),
                    maximum_reference_deviation_deg=float(abs(rolls-options.fixed_roll_deg).max()))
            report.update(motion_segments=[segment], per_tooth=path.record['per_tooth'],
                          sweep_reference_file=fingerprint(output/'sweep_reference.json'))
            try:
                plan = build(joints, phases, teeth, robot, cfg, duration_s)
            except (ValueError, RuntimeError) as error:
                report.update(status='fixed_path_build_failed', execution_ready=False, error=str(error),
                              timing_method=TIMING_METHOD, input_pose_count=len(parameters),
                              target_duration_s=duration_s, adaptive_trace_nodes_added=0)
                dump(output/'planning_report.json', report)
                return None, report
            plan.save(output/'trajectory.npz')
            np.save(output/'trajectory_tcp_world_m.npy', [robot.fk(q, full=True) for q in plan.joints])
            report.update(status='continuous_sweep_pending_audit', motion_segments=[segment],
                          per_tooth=path.record['per_tooth'],
                          **{k: path.record[k] for k in ('source_region_length_mm', 'retained_length_mm',
                                                        'trimmed_or_omitted_length_mm', 'synthetic_bridge_length_mm',
                                                        'sweep_length_mm')},
                          coverage_fraction=path.record['retained_length_mm']/path.record['source_region_length_mm'],
                          sweep_reference_file=fingerprint(output/'sweep_reference.json'),
                          geometry_file=fingerprint(output/'geometry.json'), planes_file=fingerprint(output/'tooth_planes.json'),
                          trajectory_file=fingerprint(output/'trajectory.npz'), duration_s=plan.duration,
                          timing_method=TIMING_METHOD, target_duration_s=duration_s,
                          input_pose_count=len(parameters), adaptive_trace_nodes_added=0,
                          initial_approach_method=transition['method'],
                          no_retreat_inside_sweep=True, continuous_trace_blocks=1,
                          normal_reference='regional side planes; smooth normalized blend at tooth boundaries')
            attempt.update(status='connected', trace_waypoints=len(trace))
            dump(output/'planning_report.json', report)
            return plan, report
        except (SweepFailure, ValueError) as error:
            attempt.update(status='failed', **(error.details if isinstance(error, SweepFailure) else {'reason': str(error)}))
            print(f'Continuous sweep candidate failed: {attempt}', flush=True)
            dump(output/'planning_report.json', report)
    report.update(status=f'no_{path_kind}', execution_ready=False,
                  collision_checking_enabled=not cfg.geometry_only, collision_queries=checker.calls)
    dump(output/'planning_report.json', report)
    return None, report
