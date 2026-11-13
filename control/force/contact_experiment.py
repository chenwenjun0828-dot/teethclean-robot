"""Shared collision checks, signal logging, and isolated controller test probes.

Production regional force execution is provided by force_trajectory. The probe
primitive below is used to verify stop behavior independently of a saved plan.
"""
import csv

import mujoco
import numpy as np
from planning.infrastructure.trajectory import Trajectory


def fixed_jaws(model):
    for name in ('dental_mandibular', 'dental_maxillary'):
        body = model.body(name).id
        while body:
            if model.body_jntnum[body] or model.body_mocapid[body] >= 0:
                raise ValueError('Dental model must be fixed through all ancestors: ' + name)
            body = int(model.body_parentid[body])


def contact_allowed(names, distance_from_tip_m, jaw, tip_radius_m=.003):
    tool = lambda n: n == 'ur_end_scaler_geom' or n.startswith('ur_force_tool_')
    tooth = lambda n: n == 'dental_' + jaw + '_visual' or n.startswith('force_' + jaw + '_tooth_')
    return (len(names) == 2 and ((tool(names[0]) and tooth(names[1]))
            or (tool(names[1]) and tooth(names[0]))) and distance_from_tip_m <= tip_radius_m)


def contacts(model, data, robot, jaw, allowed_teeth=None):
    rows, forbidden = [], []
    tip = data.site_xpos[robot.site]
    for i, contact in enumerate(data.contact):
        names = [model.geom(int(g)).name for g in contact.geom]
        bodies = [model.body(int(model.geom_bodyid[g])).name for g in contact.geom]
        if not any(name.startswith('ur_') for name in bodies):
            continue
        distance = float(np.linalg.norm(contact.pos - tip))
        allowed = contact_allowed(names, distance, jaw)
        if allowed and allowed_teeth is not None:
            for name in names:
                if name.startswith('force_'+jaw+'_tooth_'):
                    tooth=int(name.split('_tooth_',1)[1].split('_',1)[0])
                    allowed &= tooth in allowed_teeth
        wrench = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, wrench)
        world_force = contact.frame.reshape(3, 3).T @ wrench[:3]
        if not bodies[1].startswith('ur_'):
            world_force *= -1
        rows.append({'geom1': names[0], 'geom2': names[1], 'allowed': allowed,
                     'tip_distance_mm': distance * 1000, 'distance_mm': float(contact.dist * 1000),
                     'normal_force_N': float(wrench[0]), 'tangent1_force_N': float(wrench[1]),
                     'tangent2_force_N': float(wrench[2]),
                     'force_on_robot_world_x_N': float(world_force[0]),
                     'force_on_robot_world_y_N': float(world_force[1]),
                     'force_on_robot_world_z_N': float(world_force[2]),
                     'x_m': float(contact.pos[0]), 'y_m': float(contact.pos[1]), 'z_m': float(contact.pos[2])})
        if not allowed:
            forbidden.append(' / '.join(names))
    return rows, forbidden


def probe_trajectory(robot, start, goal, sweep_goal=None, sweep_end=None):
    # 2 s settle, 4 s approach, 2 s hold, 4 s retract, 2 s settle.
    times = np.array([0., 2., 6., 8., 12., 14.])
    if (sweep_goal is None) != (sweep_end is None):
        raise ValueError('Sweep requires both contact endpoint and retracted endpoint')
    q = np.array([start, start, goal, goal, start, start] if sweep_goal is None
                 else [start, start, goal, sweep_goal, sweep_end, sweep_end])
    phases = ['settle', 'approach', 'contact_hold', 'retract', 'settle_after']
    if sweep_goal is not None:
        phases[2] = 'trace'
    coefficients = np.array([Trajectory._quintic_coefficients(a, b, np.zeros(6), np.zeros(6),
                              np.zeros(6), np.zeros(6), dt)
                             for a, b, dt in zip(q[:-1], q[1:], np.diff(times))])
    return Trajectory(times, q, phases, [0] * 5, robot.joint_names, coefficients)


def run_probe(model, robot, jaw, tooth, probe, output, force_limit=10., force_config=None):
    plan = probe_trajectory(robot, probe['start_q'], probe['goal_q'],
                            probe.get('sweep_goal_q'), probe.get('sweep_end_q'))
    plan.save(output / f'tooth_{tooth}_probe_trajectory.npz')
    data = mujoco.MjData(model)
    data.qpos[:] = robot.initial_qpos
    data.qpos[robot.qadr] = probe['start_q']
    data.ctrl[:] = probe['start_q']
    mujoco.mj_forward(model, data)
    bodies = [model.body('dental_' + x).id for x in ('mandibular', 'maxillary')]
    fixed_positions = data.xpos[bodies].copy()
    fixed_rotations = data.xmat[bodies].copy()
    gain = model.actuator_gainprm[:, 0]
    damping = -model.actuator_biasprm[:, 2]
    sensor_names = ['ur_scaler_force', 'ur_scaler_torque']
    addresses = [model.sensor_adr[model.sensor(name).id] for name in sensor_names]
    if any(model.sensor_dim[model.sensor(n).id] != 3 for n in sensor_names):
        raise ValueError('Expected three-axis force and torque sensors')
    controller = None
    if force_config is not None:
        from .compliance import CompliantTCPController
        controller = CompliantTCPController(model, robot, plan, probe['normal_world'], force_config)
    rows, contact_rows = [], []
    status, reason = 'completed', ''
    for _ in range(int(round(plan.duration / model.opt.timestep)) + 1):
        state = plan.sample(data.time)
        force_state = {}
        if controller is not None:
            mujoco.mj_forward(model, data)
            force_state = controller.update(data, state)
            commanded_q = force_state['q']
        else:
            inertia = np.empty((model.nv, model.nv))
            if hasattr(data, 'qM'):
                mujoco.mj_fullM(model, inertia, data.qM)
            else:
                mujoco.mj_fullM(model, data, inertia)
            torque = inertia[np.ix_(robot.dadr, robot.dadr)] @ state['ddq'] + data.qfrc_bias[robot.dadr]
            data.ctrl[:] = np.clip(state['q'] + (torque + damping * state['dq']) / gain,
                                   model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
            commanded_q = state['q']
        # Refresh sensors and constraint forces at the logged q/time, before integration.
        mujoco.mj_forward(model, data)
        hits, bad = contacts(model, data, robot, jaw)
        wrench = np.concatenate([data.sensordata[a:a+3].copy() for a in addresses])
        normal_force = sum(max(0., c['normal_force_N']) for c in hits if c['allowed'])
        desired = robot.fk(commanded_q).copy()
        tip = data.site_xpos[robot.site].copy()
        position_error = np.linalg.norm(tip - desired[:3, 3]) * 1000
        jaw_drift = np.max(np.linalg.norm(data.xpos[bodies] - fixed_positions, axis=1)) * 1000
        jaw_rotation_drift = np.max(abs(data.xmat[bodies] - fixed_rotations))
        rows.append({'time_s': float(data.time), 'tooth': tooth, 'phase': state['phase'],
                     **dict(zip(['Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm'], wrench)),
                     'contact_normal_sum_N': normal_force, 'contact_count': len(hits),
                     'unexpected_contact_count': len(bad), 'position_error_mm': position_error,
                     'actual_normal_offset_mm': float((tip - probe['target_world_m'][:3, 3]) @ probe['normal_world'] * 1000),
                     'commanded_normal_offset_mm': float((desired[:3, 3] - probe['target_world_m'][:3, 3]) @ probe['normal_world'] * 1000),
                     'max_penetration_mm': max([0.] + [-c['distance_mm'] for c in hits]),
                     'jaw_translation_drift_mm': jaw_drift, 'jaw_rotation_matrix_drift': jaw_rotation_drift,
                     **dict(zip(robot.joint_names, data.qpos[robot.qadr]))})
        if controller is not None:
            observed = controller.observer.read(data)
            independent_force = np.sum([[c[f'force_on_robot_world_{axis}_N'] for axis in 'xyz']
                                        for c in hits if c['allowed']], axis=0) if hits else np.zeros(3)
            rows[-1].update({key: value for key, value in force_state.items() if key not in ('q', 'dq', 'ddq')})
            rows[-1].update(compensated_force_N=observed['force_N'],
                           independent_contact_resultant_N=float(np.linalg.norm(independent_force)),
                           sensor_solver_force_difference_N=float(np.linalg.norm(observed['external_force_world_N']-independent_force)),
                           max_command_joint_speed_rad_s=float(np.max(abs(force_state['dq']))),
                           max_command_joint_accel_rad_s2=float(np.max(abs(force_state['ddq']))))
        contact_rows.extend([{'time_s': float(data.time), 'tooth': tooth, **c} for c in hits])
        if bad:
            reason = 'Unexpected contact: ' + '; '.join(sorted(set(bad)))
        elif controller is not None and controller.fault_reason:
            reason = controller.fault_reason
        elif controller is not None and rows[-1]['compensated_force_N'] > force_config.force_stop_N:
            reason = 'Compensated contact force exceeds stop limit'
        elif normal_force > force_limit:
            reason = f'Contact force exceeded experiment stop limit {force_limit:g} N'
        elif jaw_drift > 1e-8 or jaw_rotation_drift > 1e-10:
            reason = 'Fixed jaw moved'
        elif position_error > 2.:
            reason = 'TCP tracking error exceeded 2 mm'
        elif not np.isfinite(data.qpos).all() or not np.isfinite(wrench).all() or any(w.number for w in data.warning):
            reason = 'Nonfinite state/signal or MuJoCo warning'
        if reason:
            status = 'stopped'; break
        if data.time >= plan.duration - 1e-9:
            break
        mujoco.mj_step(model, data)
    # This baseline only removes the starting offset; it is not gravity/inertia compensation.
    baseline_rows = [r for r in rows if 1. <= r['time_s'] < 2.]
    baseline = {key: float(np.mean([r[key] for r in baseline_rows])) if baseline_rows else None
                for key in ['Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm']}
    for row in rows:
        for key, value in baseline.items():
            row['baseline_subtracted_' + key] = row[key] - value if value is not None else ''
    hold = [r for r in rows if r['phase'] in ('contact_hold', 'trace')]
    summary = {'tooth': tooth, 'status': status, 'stop_reason': reason, 'recorded_duration_s': rows[-1]['time_s'],
               'samples': len(rows), 'contact_samples': sum(r['contact_normal_sum_N'] > 1e-5 for r in rows),
               'peak_contact_normal_sum_N': max(r['contact_normal_sum_N'] for r in rows),
               'peak_raw_force_norm_N': max(np.linalg.norm([r[k] for k in ('Fx_N', 'Fy_N', 'Fz_N')]) for r in rows),
               'max_position_error_mm': max(r['position_error_mm'] for r in rows),
               'max_jaw_translation_drift_mm': max(r['jaw_translation_drift_mm'] for r in rows),
               'max_jaw_rotation_matrix_drift': max(r['jaw_rotation_matrix_drift'] for r in rows),
               'hold_actual_normal_offset_mm_range': ([min(r['actual_normal_offset_mm'] for r in hold),
                                                       max(r['actual_normal_offset_mm'] for r in hold)] if hold else None),
               'zero_offset_target_reached_within_0_1_mm': bool(hold and max(r['position_error_mm'] for r in hold) <= .1),
               'unexpected_contact_samples': sum(r['unexpected_contact_count'] > 0 for r in rows),
               'baseline_sensor_frame': baseline, 'probe': probe, 'trajectory_limits': plan.limits()}
    if controller is not None:
        # Last second of the scheduled hold; approach/retract are not scored as 1 N contact.
        steady = [r for r in hold if r['time_s'] >= 7.]
        errors = np.array([r['compensated_force_N']-force_config.target_force_N for r in steady])
        metrics = {'target_N': force_config.target_force_N, 'steady_window_s': [7., 8.],
                   'steady_samples': len(steady),
                   'steady_mean_N': float(np.mean([r['compensated_force_N'] for r in steady])) if steady else None,
                   'steady_rmse_N': float(np.sqrt(np.mean(errors**2))) if steady else None,
                   'steady_max_abs_error_N': float(np.max(abs(errors))) if steady else None,
                   'steady_fraction_within_0_1_N': float(np.mean(abs(errors) <= .1)) if steady else 0.,
                   'peak_compensated_force_N': max(r['compensated_force_N'] for r in rows),
                   'max_sensor_solver_difference_N': max(r['sensor_solver_force_difference_N'] for r in rows),
                   'max_command_joint_speed_rad_s': max(r['max_command_joint_speed_rad_s'] for r in rows),
                   'max_command_joint_accel_rad_s2': max(r['max_command_joint_accel_rad_s2'] for r in rows)}
        metrics['passed'] = bool(status == 'completed' and len(steady) >= 490
                                 and metrics['steady_max_abs_error_N'] <= .1
                                 and metrics['peak_compensated_force_N'] <= force_config.target_force_N+.5
                                 and metrics['max_sensor_solver_difference_N'] < 1e-6)
        summary.update(contact_force_control=True, force_control=metrics)
    write_csv(output / f'tooth_{tooth}_force.csv', rows)
    if contact_rows:
        write_csv(output / f'tooth_{tooth}_contacts.csv', contact_rows)
    return summary, rows


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
