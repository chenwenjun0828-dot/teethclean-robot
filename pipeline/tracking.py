"""Run actuator-based gingival TCP tracking, with optional headless evidence."""
from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from perception.gingival_line import load_registered_paths, path_length, polyline_distances
from planning.validation import configure_collision_mode
from planning.trajectory import (registered_tracking_targets, plan_tooth_aligned_tracking,
                                 filter_tooth_orientation_targets)
from planning.ik import rotation_error
from control.position import GingivalTCPController, step_checked
from pipeline.simulation import PROJECT_ROOT, DENTAL_MODEL_PATH, add_trajectory_line, draw_tcp_axes

def draw_paths(scene, bundle, plan, actual, data=None):
    for chain in bundle.chains:
        # Keep every edge: decimating vertices would draw shortcuts at sharp turns.
        for a, b in zip(chain[:-1], chain[1:]):
            add_trajectory_line(scene, a, b, (0.8, 0.15, 0.5, 0.55), 1.)
    for a, b in zip(plan.path[:-1], plan.path[1:]):
        add_trajectory_line(scene, a, b, (0., 0.85, 1., 1.), 3.)
    if len(actual) > 1:
        sampled = np.asarray(actual)[::max(1, len(actual)//500)]
        for a, b in zip(sampled[:-1], sampled[1:]):
            add_trajectory_line(scene, a, b, (1., 0.45, 0., 1.), 2.)
    if data is not None:
        draw_tcp_axes(scene, data, plan.site_id)


def set_camera(camera, plan):
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = np.mean(plan.path, axis=0)
    camera.distance = 0.24
    camera.azimuth = 100
    camera.elevation = -20


def preview(renderer, data, bundle, plan, actual):
    camera = mujoco.MjvCamera()
    set_camera(camera, plan)
    option = mujoco.MjvOption()
    # The chair is static scenery with no collision, and would hide the close-up.
    renderer.update_scene(data, camera=camera, scene_option=option)
    # Rendering only: remove the chair from this close-up without changing model properties.
    chair_id = plan.model.geom('dental_dental_chair_visual').id
    for geom in renderer.scene.geoms[:renderer.scene.ngeom]:
        if geom.objtype == mujoco.mjtObj.mjOBJ_GEOM and geom.objid == chair_id:
            geom.rgba[3] = 0
    draw_paths(renderer.scene, bundle, plan, actual, data)
    return renderer.render().copy()


def run_tracking(registration_dir, jaw='mandibular', path_index=0,
                 offset_mm=0., speed_mm_s=4., collision_mode='geometry', headless=False,
                 output_dir=None, topology_path=None, record_gif=False,
                 model_data=None, max_orientation_step_deg=45.):
    if jaw not in ('mandibular', 'maxillary'):
        raise ValueError('Jaw must be mandibular or maxillary')
    if model_data is None:
        raise ValueError('Tracking requires the capture scene model and data')
    model, data = model_data
    bundle = load_registered_paths(Path(registration_dir), jaw, topology_path)
    output_dir = Path(output_dir) if output_dir else PROJECT_ROOT/'outputs/pipeline_tracking'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise FileExistsError(f'Output directory must be empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    report = {
        'status': 'planning', 'simulation_only': True, 'hardware_execution_allowed': False,
        'jaw': jaw, 'path_index': path_index, 'path_source': 'registered',
        'jaw_scene_pose': 'capture',
        'registration_dir': str(bundle.registration_dir) if bundle.registration_dir else None,
        'coordinate_mapping': 'estimated scan-to-camera registration and saved camera pose',
        'T_world_scan_m': bundle.world_from_scan.tolist(),
        'source_sampling_mm': bundle.sampling_spacing_m*1000.,
        'topology_path': str(bundle.topology_path), 'collision_mode': collision_mode,
        'dental_collision_enabled': collision_mode == 'strict',
        'offset_mm': float(offset_mm) if np.isfinite(offset_mm) else None,
        'speed_limit_mm_s': float(speed_mm_s) if np.isfinite(speed_mm_s) else None,
        'tool_pose': ('tooth_aligned_filtered' if max_orientation_step_deg is not None
                      else 'tooth_aligned'), 'fixed_orientation': False,
        'retained_tcp_tooth_relative_rotation': np.eye(3).tolist(),
        'offset_direction': 'per_point_tooth_positive_z',
        'scene': str(DENTAL_MODEL_PATH), 'mujoco_version': mujoco.__version__,
    }
    rows, actual, frames, trace_angles = [], [], [], []
    maximum_joint_speed = 0.
    renderer = None
    active_viewer = None
    plan = None
    try:
        configure_collision_mode(model, collision_mode)
        mujoco.mj_forward(model, data)
        targets = registered_tracking_targets(model, data, bundle, path_index, offset_mm)
        np.save(output_dir/'requested_T_world_tcp.npy', targets.tcp.transforms)
        if max_orientation_step_deg is not None:
            targets = filter_tooth_orientation_targets(targets, max_orientation_step_deg)
            selection = targets.report['orientation_filter']
            report['orientation_filter'] = selection
            kept = set(selection['retained_sample_indices'])
            skipped = {row['sample_index']: row for row in selection['skipped_samples']}
            np.savetxt(output_dir/'orientation_selection.csv',
                       [[i, int(i in kept), skipped[i]['angle_deg'] if i in skipped else 0.]
                        for i in range(len(targets.tcp.origins))], delimiter=',',
                       header='sample_index,retained_tooth_orientation,rejected_angle_deg', comments='')
        # Preserve reviewable geometric targets even if IK/preflight rejects execution.
        report.update(orientation_world=targets.tcp.rotations.tolist(),
                      tooth_frame_geometry=targets.report, trace_waypoints=len(targets.tcp.origins))
        np.save(output_dir/'target_rotations_world.npy', targets.tcp.rotations)
        np.save(output_dir/'T_world_tcp.npy', targets.tcp.transforms)
        np.save(output_dir/'T_world_tooth.npy', targets.tooth.transforms)
        np.savetxt(output_dir/'target_path_world_m.csv', targets.tcp.origins, delimiter=',',
                   header='x_m,y_m,z_m', comments='')
        np.savetxt(output_dir/'target_poses_world_m_rad.csv',
                   np.column_stack((targets.tcp.origins,
                                    Rotation.from_matrix(targets.tcp.rotations).as_euler('xyz'))),
                   delimiter=',', header='x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad', comments='')
        plan = plan_tooth_aligned_tracking(model, data, targets, speed_mm_s)
        controller = GingivalTCPController(model, plan)
        report.update(path_length_mm=path_length(plan.path)*1000., duration_s=plan.duration,
                      original_chain_length_mm=path_length(bundle.chains[path_index])*1000.,
                      continuous_chains=len(bundle.chains), trace_start_s=plan.trace_start,
                      trace_end_s=plan.trace_end, motion_ik_samples=len(plan.command_q))
        np.savetxt(output_dir/'joint_waypoints_rad.csv',
                   np.vstack((plan.home_q, plan.approach_q, plan.command_q, plan.retreat_q)),
                   delimiter=',', header='q1,q2,q3,q4,q5,q6', comments='')
        np.savetxt(output_dir/'trace_joint_commands.csv',
                   np.column_stack((plan.command_arc, plan.command_q)), delimiter=',',
                   header='arc_m,q1,q2,q3,q4,q5,q6', comments='')
        print(f'{jaw}: continuous path {path_index}, length {path_length(plan.path)*1000:.2f} mm, '
              f'offset {offset_mm:.1f} mm, max speed {speed_mm_s:.1f} mm/s.')
        print(f'Collision mode: {collision_mode}. Simulation duration: {plan.duration:.1f} s.')
        if collision_mode == 'geometry':
            print('GEOMETRY PLAYBACK: jaw convex collision proxies are disabled; this run does not validate contact.')
        print('Cyan: target TCP path; orange: actual TCP trace; pink: registered source curves.')
        renderer = mujoco.Renderer(model, height=600, width=800)
        if not headless:
            from mujoco import viewer
            active_viewer = viewer.launch_passive(model, data)
            with active_viewer.lock():
                set_camera(active_viewer.cam, plan)
                # Hide only the chair in the viewer. This model is local to this run.
                model.geom_rgba[model.geom('dental_dental_chair_visual').id, 3] = 0
        steps = max(1, round(0.02/model.opt.timestep))
        period = steps*model.opt.timestep
        next_frame = 0.
        preview_saved = False
        while data.time < plan.duration+0.5:
            wall_started = time.perf_counter()
            if active_viewer is not None and not active_viewer.is_running():
                report['status'] = 'interrupted'
                break
            controller.update(data)
            step_checked(model, data, steps)
            target, q, phase = plan.sample(data.time)
            position = data.site_xpos[plan.site_id].copy()
            error_mm = float(np.linalg.norm(position-target)*1000.)
            maximum_joint_speed = max(maximum_joint_speed, float(np.max(np.abs(data.qvel))))
            orientation_error = None
            if phase == 'trace':
                actual.append(position)
                actual_rotation = data.site_xmat[plan.site_id].reshape(3, 3)
                desired_rotation = plan.sample_rotation(data.time)
                orientation_error = float(np.rad2deg(np.linalg.norm(rotation_error(
                    desired_rotation, actual_rotation))))
                trace_angles.append(orientation_error)
            rows.append([data.time, *target, *position, error_mm, phase, orientation_error])
            if record_gif and data.time >= next_frame:
                from PIL import Image
                frames.append(Image.fromarray(preview(renderer, data, bundle, plan, actual)).resize((640, 480)))
                next_frame += (plan.duration+0.5)/48
            if not preview_saved and data.time >= plan.trace_start+plan.trace_duration*0.6:
                from PIL import Image
                Image.fromarray(preview(renderer, data, bundle, plan, actual)).save(output_dir/'tracking_preview.png')
                preview_saved = True
            if active_viewer is not None:
                with active_viewer.lock():
                    active_viewer.user_scn.ngeom = 0
                    draw_paths(active_viewer.user_scn, bundle, plan, actual, data)
                active_viewer.sync()
                time.sleep(max(0., period-(time.perf_counter()-wall_started)))
        else:
            report['status'] = 'completed'
        if not preview_saved:
            from PIL import Image
            Image.fromarray(preview(renderer, data, bundle, plan, actual)).save(output_dir/'tracking_preview.png')
        trace_errors = np.array([row[7] for row in rows if row[8] == 'trace'])
        if len(trace_errors):
            report.update(trace_rms_mm=float(np.sqrt(np.mean(trace_errors**2))),
                          trace_max_mm=float(trace_errors.max()), trace_samples=len(trace_errors),
                          trace_rotation_max_deg=max(trace_angles),
                          trace_rotation_rms_deg=float(np.sqrt(np.mean(np.square(trace_angles)))))
            # Distance to the original registered topology includes any requested offset.
            line_distances_mm = 1000*polyline_distances(np.asarray(actual), bundle.chains[path_index])
            report.update(original_line_distance_rms_mm=float(np.sqrt(np.mean(line_distances_mm**2))),
                          original_line_distance_max_mm=float(line_distances_mm.max()))
            if report['status'] == 'completed' and (report['trace_rms_mm'] > 1. or report['trace_max_mm'] > 2.):
                raise RuntimeError('Trace exceeds acceptance limits: RMS 1 mm / maximum 2 mm')
            if report['status'] == 'completed' and report['trace_rotation_max_deg'] > 1.:
                raise RuntimeError('Trace orientation exceeds acceptance limit: maximum 1 degree')
        report['simulated_time_s'] = float(data.time)
        report['maximum_joint_speed_rad_s'] = maximum_joint_speed
        report['robot_contacts_detected'] = 0
        if frames:
            frames[0].save(output_dir/'tracking.gif', save_all=True, append_images=frames[1:], duration=160, loop=0)
    except (ValueError, RuntimeError, FileNotFoundError) as error:
        report['status'] = 'failed'
        report['error'] = str(error)
        report['simulated_time_s'] = float(data.time)
        raise
    finally:
        if active_viewer is not None:
            active_viewer.close()
        if renderer is not None:
            renderer.close()
        import csv
        with (output_dir/'tcp_tracking.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['time_s', 'target_x_m', 'target_y_m', 'target_z_m',
                             'actual_x_m', 'actual_y_m', 'actual_z_m', 'error_mm', 'phase',
                             'orientation_error_deg'])
            writer.writerows(rows)
        (output_dir/'tracking_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(f"{report['status']}: trace RMS {report.get('trace_rms_mm', float('nan')):.3f} mm, "
          f"max {report.get('trace_max_mm', float('nan')):.3f} mm, "
          f"rotation max {report.get('trace_rotation_max_deg', float('nan')):.3f} deg. "
          f"Results: {output_dir.resolve()}")
    return report


