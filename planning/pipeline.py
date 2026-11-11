"""Plan robot motion using a continuous gingival region emitted by perception."""
from dataclasses import asdict, replace
from pathlib import Path
from time import perf_counter
import numpy as np
from planning.infrastructure.io import ROOT, dump, fingerprint, body_transform, validate_transform
from planning.infrastructure.robot import Robot
from planning.infrastructure.collision import CollisionChecker, NoCollisionChecker
from planning.infrastructure.mount import apply_mount_position
from pipeline.oral import jaw_overrides, load_jaw_settings
from .config import Config, SweepOptions, continuous_path_kind
from perception.regions import PreparedRegion
from .motion_planning.continuous import plan_sweep
from .motion_planning.validation import validate


def run(inputs, output, cfg=None, geometry_only=False, mount_position_m=None,
        world_from_scan=None, sweep_options=None, model_data=None, jaw_opening_deg=None, duration_s=None):
    from pipeline.simulation import create_simulation, DENTAL_MODEL_PATH
    prepared = PreparedRegion.load(inputs) if isinstance(inputs, (str, Path)) else inputs
    if not isinstance(prepared, PreparedRegion):
        raise TypeError('Planning requires a PreparedRegion or perception_region.json; run perception first')
    cfg = cfg or Config(region=prepared.region)
    prepared.validate(cfg.region)
    inputs = prepared.inputs
    if duration_s is not None and (isinstance(duration_s, bool) or not np.isfinite(duration_s) or duration_s <= 0):
        raise ValueError('Target duration must be positive and finite')
    sweep_options = sweep_options or SweepOptions()
    jaw_settings = jaw_overrides(load_jaw_settings(), jaw_opening_deg)
    output = Path(output).resolve()
    if output.exists() and any(output.iterdir()): raise ValueError('Use a new empty output directory')
    output.mkdir(parents=True, exist_ok=True)
    request = {'inputs': asdict(inputs), 'config': asdict(cfg), 'planner': 'planning', 'region': cfg.region,
               'planning_mode': cfg.planning_mode, 'check_mode': cfg.check_mode,
               'jaw_opening_deg': jaw_settings.opening_deg, 'target_duration_s': duration_s,
               **cfg.motion_description(),
               'scene': fingerprint(DENTAL_MODEL_PATH),
               'robot_model': fingerprint(ROOT/'assets/simulation/robot/ur7e_with_tool.xml'),
               'oral_settings': fingerprint(ROOT/'config/oral_settings.py')}
    dump(output/'request.json', request)
    if sweep_options is not None:
        request.update(path_kind=continuous_path_kind(cfg.region), sweep_options=asdict(sweep_options))
        dump(output/'request.json', request)
    geometry_started = perf_counter()
    geometry = replace(prepared.geometry, report={**prepared.geometry.report,
        'perception_config': prepared.geometry.report['config'], 'config': asdict(cfg)})
    lookup = prepared.lookup
    geometry.save(output)
    np.savez_compressed(output/'tooth_lookup.npz', **lookup)
    dump(output/'perception_reference.json', prepared.reference.record)
    request['perception_reference_file'] = fingerprint(output/'perception_reference.json')
    request['geometry_owner'] = 'perception'
    dump(output/'request.json', request)
    geometry_wall_time_s = perf_counter() - geometry_started
    print(f'Geometry: {len(geometry.strokes)} {cfg.region} strokes; planes '+
          str({t: {'valid': p['valid'], 'rms_mm': p.get('rms_mm')} for t,p in geometry.planes.items()}), flush=True)
    if geometry_only:
        report = {'status': 'geometry_only', 'execution_ready': False, 'planner': 'planning',
                  'geometry_owner': 'perception',
                  'planning_mode': cfg.planning_mode, 'check_mode': cfg.check_mode,
                  'collision_checking_enabled': not cfg.geometry_only,
                  'region': cfg.region, 'jaw_opening_deg': jaw_settings.opening_deg,
                  'config': asdict(cfg), 'timings_wall_s': {
                      'geometry_import_s': geometry_wall_time_s},
                  **cfg.motion_description()}
        dump(output/'planning_report.json', report)
        return geometry, None, report, None
    planning_setup_started = perf_counter()
    model, data = create_simulation(jaw_settings=jaw_settings) if model_data is None else model_data
    if mount_position_m is not None: apply_mount_position(model, mount_position_m, data)
    supplied_world = prepared.world_from_scan if world_from_scan is None else world_from_scan
    if prepared.world_from_scan is not None and world_from_scan is not None and not np.allclose(
            prepared.world_from_scan, validate_transform(world_from_scan), atol=1e-10, rtol=0):
        raise ValueError('Planning transform differs from registered perception output')
    world = body_transform(model, data, inputs.jaw) if supplied_world is None else validate_transform(supplied_world)
    request.update(selected_robot_mount_position_m=model.body_pos[model.body('robot_mount').id].copy(),
                   world_transform_source='simulation' if supplied_world is None else 'supplied_registration',
                   world_from_scan_m=world)
    dump(output/'request.json', request)
    robot = Robot(model, data)
    if cfg.geometry_only:
        # No tooth meshes are loaded: geometry-only runs never query collisions.
        checker = NoCollisionChecker(robot)
        print('Collision checking disabled (check_mode=geometry_only): '
              'poses, IK connectivity and path geometry only', flush=True)
    else:
        checker = CollisionChecker(robot, {inputs.jaw: world}, {inputs.jaw: inputs.scan})
    planning_setup_wall_time_s = perf_counter() - planning_setup_started
    motion_planning_started = perf_counter()
    plan, report = plan_sweep(geometry, robot, checker, world, cfg, output, sweep_options,
                              jaw_opening_deg=jaw_settings.opening_deg, duration_s=duration_s,
                              reference=prepared.reference)
    report['geometry_owner'] = 'perception'
    report['perception_reference_file'] = request['perception_reference_file']
    report['jaw_opening_deg'] = jaw_settings.opening_deg
    report['check_mode'] = cfg.check_mode
    report['collision_checking_enabled'] = not cfg.geometry_only
    report['collision_queries'] = checker.calls
    dump(output/'planning_report.json', report)
    motion_planning_wall_time_s = perf_counter() - motion_planning_started
    validation_started = perf_counter()
    plan, report = validate(output, plan, report, robot, checker, cfg)
    report['timings_wall_s'] = {
        'geometry_import_s': geometry_wall_time_s,
        'planning_setup_s': planning_setup_wall_time_s,
        'pose_search_and_motion_planning_s': motion_planning_wall_time_s,
        'trajectory_validation_s': perf_counter() - validation_started,
    }
    dump(output/'planning_report.json', report)
    return geometry, plan, report, (model, data, checker)
