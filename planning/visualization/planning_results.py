"""Read planning certificates and sample actual FK for display, never execution."""
from pathlib import Path
import json

import numpy as np

from planning.infrastructure.io import fingerprint


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def summarize(directory, check_mode, returncode):
    directory = Path(directory)
    report = read_json(directory / 'planning_report.json')
    failure = read_json(directory / 'failure.json')
    result = {
        'passed': False, 'validation': 'failed',
        'status': report.get('status', 'no_report'),
        'error': failure.get('error') or report.get('error'),
        'duration_s': report.get('duration_s'),
        'minimum_clearance_mm': report.get('minimum_clearance_mm'),
        'trace_passed': report.get('trace_audit', {}).get('passed', False),
        'motion_passed': report.get('motion_audit', {}).get('passed', False),
        'dynamics_passed': report.get('fast_dynamics_verified', False),
        'coverage_passed': report.get('all_selected_meet_coverage', False),
        'attempts': report.get('attempts', []),
        'orientation': report.get('regional_orientation') or report.get('local_surface_orientation'),
        'robot_execution_allowed': False,
    }
    if returncode != 0 or failure or not report:
        return result
    if check_mode == 'geometry_only':
        result['passed'] = (report.get('status') == 'geometry_only_verified'
                            and report.get('check_mode') == 'geometry_only'
                            and report.get('geometry_verified') is True
                            and report.get('execution_ready') is False
                            and result['trace_passed'] and result['motion_passed'])
        result['validation'] = 'reachability' if result['passed'] else 'failed'
    else:
        from planning.motion_planning.execution import load_verified
        try:
            load_verified(directory)
            result['passed'] = (report.get('check_mode') == 'full'
                                and report.get('collision_checking_enabled') is True
                                and report.get('execution_ready') is True
                                and result['trace_passed'] and result['motion_passed']
                                and result['coverage_passed'] and result['dynamics_passed']
                                and report.get('fast_execution_summary', {}).get('status') == 'completed')
            result['validation'] = 'simulation' if result['passed'] else 'failed'
        except (ValueError, RuntimeError, OSError, KeyError) as error:
            result['error'] = str(error)
    if not result['passed'] and not result['error']:
        result['error'] = 'Planning certificates did not pass; inspect attempts and planner.log'
    return result


def actual_tcp_path(directory):
    """Display-only samples of the saved interpolant, in world metres."""
    from planning.infrastructure.robot import Robot
    from planning.infrastructure.trajectory import Trajectory
    from pipeline.simulation import create_simulation
    from planning.session import rebuild_session_simulation

    directory = Path(directory)
    report = read_json(directory / 'planning_report.json')
    request = read_json(directory / 'request.json')
    trajectory_file = directory / 'trajectory.npz'
    if not trajectory_file.is_file():
        return None
    if fingerprint(trajectory_file)['sha256'] != report.get('trajectory_file', {}).get('sha256'):
        raise ValueError('Trajectory fingerprint is absent or changed; no display path')
    session = request.get('perception_session')
    if session:
        _, model, data, _, _, _ = rebuild_session_simulation(session)
    else:
        from pipeline.oral import jaw_overrides, load_jaw_settings
        model, data = create_simulation(jaw_settings=jaw_overrides(
            load_jaw_settings(), report.get('jaw_opening_deg')),
            robot_mount_position_m=report.get('robot_mount_position_m'))
        data.qpos[:] = report['scene_qpos']
    robot = Robot(model, data)
    trajectory = Trajectory.load(trajectory_file)
    if trajectory.joint_names != robot.joint_names:
        raise ValueError('Display trajectory joint order differs from robot')
    node_times = trajectory.times[np.linspace(0, len(trajectory.times)-1,
                                             min(600, len(trajectory.times))).astype(int)]
    times = np.unique(np.concatenate((node_times, np.linspace(0., trajectory.duration, 600))))
    samples = [trajectory.sample(float(t)) for t in times]
    positions = [robot.fk(sample['q'])[:3, 3].tolist() for sample in samples]
    return {'points_world_m': positions, 'phases': [sample['phase'] for sample in samples],
            'display_only': True}
