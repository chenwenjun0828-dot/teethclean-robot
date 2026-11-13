"""Static MuJoCo inspection of regional paths. Never steps or controls the robot."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from PIL import Image

from perception.regional_paths import (
    DEFAULT_DATASET, REGION_ORDER, load_regional_paths, scan_points_to_world_m,
    evaluate_regional_alignment,
)
from .simulation import PROJECT_ROOT, create_simulation, add_trajectory_line

COLORS = ((.95, .45, .12, 1.), (.95, .8, .12, 1.), (.9, .3, .55, 1.),
          (.15, .75, .8, 1.), (.2, .75, .35, 1.), (.2, .4, .9, 1.))


def draw_regional_paths(scene, chains, selected=None):
    """Each chain is drawn separately; no cross-region connector is created."""
    for chain, points in chains:
        if selected is not None and chain.region != selected:
            continue
        for a, b in zip(points[:-1], points[1:]):
            add_trajectory_line(scene, a, b, COLORS[chain.order - 1], 4.)
        for point, color, label in (
                (points[0], (0., 1., .2, 1.), str(chain.order)),
                (points[-1], (1., .05, .05, 1.), '')):
            if scene.ngeom >= scene.maxgeom:
                raise ValueError('MuJoCo preview geometry capacity exceeded')
            geom = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                               np.full(3, .0006), point, np.eye(3).ravel(),
                               np.asarray(color, dtype=np.float32))
            geom.label = label
            scene.ngeom += 1
        if scene.ngeom >= scene.maxgeom:
            raise ValueError('MuJoCo preview geometry capacity exceeded')
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW,
                           np.ones(3), np.zeros(3), np.eye(3).ravel(),
                           np.asarray(COLORS[chain.order - 1], dtype=np.float32))
        direction = points[min(4, len(points) - 1)] - points[0]
        if np.linalg.norm(direction) > 1e-12:
            end = points[0] + direction / np.linalg.norm(direction) * .003
            mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .00025,
                                points[0], end)
            scene.ngeom += 1


def _surface_distances(dataset):
    """Independent of old camera pose: compare recovered points to the source mesh."""
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(dataset.mesh_path))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return {
        chain.region: float(scene.compute_distance(o3d.core.Tensor(
            chain.scan_points_mm.astype(np.float32))).numpy().max())
        for chain in dataset.chains
    }


def draw_reference_paths(scene, chains, selected=None):
    """Thin white reference geometry is evaluation-only, never a fitted result."""
    for chain, points in chains:
        if selected is None or chain.region == selected:
            for a, b in zip(points[:-1], points[1:]):
                add_trajectory_line(scene, a, b, (.85, .85, .85, 1.), 1.)


def _save_world_paths(path, chains):
    with Path(path).open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(['chain_order', 'region', 'point_order', 'x_m', 'y_m', 'z_m'])
        for chain, points in chains:
            for index, point in enumerate(points):
                writer.writerow([chain.order, chain.region, index, *point])


def preview_paths(dataset_dir=DEFAULT_DATASET, output_dir=None, region=None,
                  headless=False, context=False, placement='known'):
    if placement not in ('known', 'automatic'):
        raise ValueError('Placement must be known or automatic')
    dataset = load_regional_paths(dataset_dir)
    model, data = create_simulation()
    distances = _surface_distances(dataset)
    if max(distances.values()) > .35:
        raise ValueError(f'Recovered regional points exceed 0.35 mm from scan: {distances}')
    output_dir = Path(output_dir) if output_dir else (
        PROJECT_ROOT / 'outputs/regional_preview' / datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    output_dir = output_dir.resolve()
    if output_dir.exists() and (not output_dir.is_dir() or any(output_dir.iterdir())):
        raise FileExistsError(f'Output directory must be empty: {output_dir}')
    output_dir.mkdir(parents=True, exist_ok=True)
    # Capture and fitting precede all visualization edits and truth evaluation.
    registration = None
    stage_report = {'status': 'running', 'stage': 'registration' if placement == 'automatic'
                    else 'preview', 'placement_mode': placement, 'robot_motion_executed': False}
    report_path = output_dir / 'preview_report.json'
    report_path.write_text(json.dumps(stage_report, indent=2), encoding='utf-8')
    if placement == 'automatic':
        from perception.regional_registration import register_regional_paths
        try:
            registration = register_regional_paths(model, data, dataset, output_dir)
        except Exception as error:
            stage_report.update(status='failed', error=str(error))
            report_path.write_text(json.dumps(stage_report, indent=2), encoding='utf-8')
            raise
    body = data.body('dental_' + dataset.jaw)
    reference_pose = np.eye(4)
    reference_pose[:3, :3] = body.xmat.reshape(3, 3)
    reference_pose[:3, 3] = body.xpos
    world_from_scan = (registration.world_from_scan_m if registration is not None
                       else reference_pose)
    chains = [(chain, scan_points_to_world_m(chain.scan_points_mm, world_from_scan))
              for chain in dataset.chains]
    references = [(chain, scan_points_to_world_m(chain.scan_points_mm, reference_pose))
                  for chain in dataset.chains] if registration is not None else []
    evaluation = None
    if registration is not None:
        from perception.global_camera import load_config
        config = load_config()
        evaluation = evaluate_regional_alignment(dataset, world_from_scan, reference_pose)
        evaluation['thresholds_mm'] = {
            'translation': config['ideal_translation_limit_mm'],
            'path_rms': config['ideal_line_rms_limit_mm'],
            'path_max': config['ideal_line_max_limit_mm']}
        evaluation['rotation_limit_deg'] = config['ideal_rotation_limit_deg']
        evaluation['alignment_passed'] = bool(
            evaluation['translation_error_mm'] <= config['ideal_translation_limit_mm'] and
            evaluation['rotation_error_deg'] <= config['ideal_rotation_limit_deg'] and
            evaluation['path_rms_mm'] <= config['ideal_line_rms_limit_mm'] and
            evaluation['path_max_mm'] <= config['ideal_line_max_limit_mm'])
    _save_world_paths(output_dir / 'regional_paths_world_m.csv', chains)
    if references:
        _save_world_paths(output_dir / 'reference_paths_world_m.csv', references)
    # Focus this local preview model on the selected jaw; XML assets stay untouched.
    if not context:
        target = model.geom('dental_' + dataset.jaw + '_visual').id
        model.geom_rgba[np.arange(model.ngeom) != target, 3] = 0.
        model.geom_rgba[target, 3] = .4
        material = model.geom_matid[target]
        if material >= 0:
            model.mat_rgba[material, 3] = .4
    all_points = np.vstack([points for chain, points in (references or chains)
                            if region is None or chain.region == region])
    camera = mujoco.MjvCamera()
    camera.lookat[:] = np.mean(all_points, axis=0)
    camera.distance = .115
    camera.azimuth = 90
    camera.elevation = -90
    option = mujoco.MjvOption()
    option.label = mujoco.mjtLabel.mjLABEL_SELECTION
    with mujoco.Renderer(model, height=720, width=1200) as renderer:
        renderer.update_scene(data, camera=camera, scene_option=option)
        draw_reference_paths(renderer.scene, references, region)
        draw_regional_paths(renderer.scene, chains, region)
        Image.fromarray(renderer.render()).save(output_dir / 'regional_preview.png')
    report = {
        'status': 'preview_ready', 'jaw': dataset.jaw, 'selected_region': region,
        'source_version': dataset.metadata['source_version'],
        'mesh_sha256': dataset.metadata['mesh_sha256'],
        'placement_mode': placement,
        'placement': ('new RGB-D automatic scan-to-camera estimate + captured camera world pose'
                      if registration is not None else
                      'known MuJoCo scan-body pose; historical D405 transform inverted'),
        'registration_accuracy_validated': False, 'robot_execution_allowed': False,
        'simulation_alignment_evaluated': evaluation is not None,
        'robot_motion_executed': False, 'simulation_time_s': float(data.time),
        'T_world_scan_m': world_from_scan.tolist(),
        'registration': ({'method': 'automatic',
                          'capture_dir': 'capture', 'registration_dir': 'registration',
                          'capture_id': registration.report['capture_id'],
                          'mask_mode': registration.report['mask_mode'],
                          'fitness': registration.report['fitness'],
                          'icp_rmse_mm': registration.report['icp_fine_rmse_mm']}
                         if registration is not None else None),
        'evaluation': evaluation,
        'chains': [{'order': chain.order, 'region': chain.region, 'points': len(points),
                    'length_mm': float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()*1000),
                    'max_scan_surface_distance_mm': distances[chain.region]}
                   for chain, points in chains],
    }
    (output_dir / 'preview_report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    print(f'Static preview saved: {output_dir}', flush=True)
    print('0: all regions; 1-6: select region. Green: start; red: end. Robot stays still.',
          flush=True)
    if evaluation is not None:
        print(f"Alignment check: {evaluation['alignment_passed']}; "
              f"path RMS {evaluation['path_rms_mm']:.4f} mm, "
              f"max {evaluation['path_max_mm']:.4f} mm. "
              'White: known-pose reference (R toggles). Colors: estimated placement.', flush=True)
        if not evaluation['alignment_passed']:
            print('Alignment exceeds simulator evaluation limits; inspect the failed fit. '
                  'The estimate has NOT been replaced by the reference.', flush=True)
    if not headless:
        from mujoco import viewer
        state = {'region': region, 'reference': True}

        def on_key(key):
            if key == ord('0'):
                state['region'] = None
            elif ord('1') <= key <= ord('6'):
                state['region'] = REGION_ORDER[key - ord('1')]
            elif key == ord('R'):
                state['reference'] = not state['reference']

        with viewer.launch_passive(model, data, key_callback=on_key) as window:
            with window.lock():
                window.cam.lookat[:] = camera.lookat
                window.cam.distance = camera.distance
                window.cam.azimuth = camera.azimuth
                window.cam.elevation = camera.elevation
                window.opt.label = option.label
            while window.is_running():
                with window.lock():
                    window.user_scn.ngeom = 0
                    if state['reference']:
                        draw_reference_paths(window.user_scn, references, state['region'])
                    draw_regional_paths(window.user_scn, chains, state['region'])
                window.sync()
                time.sleep(.03)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, default=DEFAULT_DATASET)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--region', choices=REGION_ORDER)
    parser.add_argument('--headless', action='store_true', help='Save preview without opening viewer')
    parser.add_argument('--context', action='store_true', help='Also show robot, chair and other jaw')
    parser.add_argument('--placement', choices=('known', 'automatic'), default='known',
                        help='Known scan pose, or a NEW virtual RGB-D automatic registration')
    args = parser.parse_args(argv)
    preview_paths(args.dataset, args.output, args.region, args.headless, args.context,
                  args.placement)


if __name__ == '__main__':
    main()
