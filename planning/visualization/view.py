"""Replay saved planning trajectories in MuJoCo without executing control."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import mujoco

from pipeline.simulation import add_trajectory_line
from .playback import Playback, animation_times
from ..config import CONTINUOUS_PATH_KINDS


TRACE_COLOR = (.05, .72, .65, 1.)
BRIDGE_COLOR = (1., .48, .03, 1.)


def saved_jaw_opening(report, override=None):
    if override is not None:
        return override
    for record in (report, report.get('timing_profile') or {}):
        value = record.get('jaw_opening_deg')
        if value is not None:
            return value
    return None


def bounded_curves(curves, budget=2000):
    curves = [(np.asarray(points), color) for points, color in curves if len(points) >= 2]
    if not curves:
        return []
    curves = curves[:budget]
    per_curve = max(1, budget//len(curves))
    return [(points[np.linspace(0, len(points)-1, min(len(points), per_curve+1), dtype=int)], color)
            for points, color in curves]


def preview_curves(geometry, world, trajectory, robot, report):
    transform = lambda points: np.asarray(points)@world[:3, :3].T+world[:3, 3]
    curves = []
    if 'continuous_reference' in geometry:
        path = geometry['continuous_reference']
        for part in path.parts:
            points = [path.position(s, report['config']['standoff_mm']*.001)
                      for s in np.linspace(part['start'], part['end'], 90)]
            color = TRACE_COLOR if part['kind'] == 'source' else BRIDGE_COLOR
            curves.append((transform(points), color))
    elif trajectory is not None:
        boundaries = np.r_[0, np.flatnonzero(trajectory.phases[1:] != trajectory.phases[:-1])+1,
                           len(trajectory.phases)]
        for begin, end in zip(boundaries[:-1], boundaries[1:]):
            phase = trajectory.phases[begin]
            if phase not in ('trace', 'surface_transfer'):
                continue
            indices = np.linspace(begin, end, min(end-begin+1, 256), dtype=int)
            points = [robot.fk(trajectory.joints[i])[:3, 3] for i in indices]
            curves.append((points, TRACE_COLOR if phase == 'trace' else BRIDGE_COLOR))
    else:
        palette = [(.1, .45, .95, 1.), BRIDGE_COLOR, (.15, .75, .38, 1.)]
        for stroke in np.unique(geometry['stroke_id']):
            ids = np.flatnonzero(geometry['stroke_id'] == stroke)
            curves.append((transform(geometry['points_scan_m'][ids]),
                           palette[int(geometry['region_id'][ids[0]])]))
    return bounded_curves(curves)


def preview_only_scene(directory, report, request, opening):
    """Rebuild the scene for a geometry-only plan without any verification claim.

    Watching an unvalidated plan is safe because nothing is executed; this path
    deliberately does not call load_verified, so the result can never be handed
    to control.
    """
    from planning.infrastructure.io import body_transform
    from planning.infrastructure.robot import Robot
    from planning.infrastructure.trajectory import Trajectory
    from pipeline.simulation import create_simulation
    from pipeline.oral import jaw_overrides, load_jaw_settings
    options = {'robot_mount_position_m': report.get('robot_mount_position_m')}
    if opening is not None:
        options['jaw_settings'] = jaw_overrides(load_jaw_settings(), opening)
    model, data = create_simulation(**options)
    for jaw, transform in report.get('scene_tooth_transforms', {}).items():
        if not np.allclose(body_transform(model, data, jaw), transform, atol=1e-9):
            raise ValueError('Dental pose changed; specify the original --jaw-opening-deg')
    return model, data, Trajectory.load(Path(directory)/'trajectory.npz'), Robot(model, data)


def load_scene(directory, *, jaw_opening_deg=None):
    from planning.infrastructure.io import body_transform
    from planning.infrastructure.robot import Robot
    from planning.motion_planning.execution import load_verified
    directory = Path(directory)
    report = json.loads((directory/'planning_report.json').read_text(encoding='utf-8'))
    request_path = directory/'request.json'
    request = json.loads(request_path.read_text(encoding='utf-8')) if request_path.exists() else {}
    if report.get('planner') != 'planning':
        raise ValueError('Expected planning output')
    opening = saved_jaw_opening(report, jaw_opening_deg)
    if opening is None:
        opening = request.get('jaw_opening_deg')
        if opening is None:
            opening = (request.get('timing_profile') or {}).get('jaw_opening_deg')
    if (directory/'trajectory.npz').exists() and report.get('check_mode') == 'geometry_only':
        # A geometry-only plan may be watched, but it is never load_verified and
        # therefore never executable: this branch only rebuilds the scene.
        model, data, trajectory, robot = preview_only_scene(directory, report, request, opening)
        print('Geometry-only plan: collision clearance was never evaluated; preview only', flush=True)
    elif (directory/'trajectory.npz').exists():
        try:
            model, data, trajectory, checker = load_verified(directory, jaw_opening_deg=opening)
        except ValueError as error:
            if 'Dental pose changed' in str(error):
                raise ValueError(f'{error}; specify the original --jaw-opening-deg') from error
            raise
        robot = checker.robot
    else:
        from pipeline.simulation import create_simulation
        from pipeline.oral import jaw_overrides, load_jaw_settings
        options = {'robot_mount_position_m': report.get('robot_mount_position_m')}
        if opening is not None:
            options['jaw_settings'] = jaw_overrides(load_jaw_settings(), opening)
        model, data = create_simulation(**options)
        for jaw, transform in report.get('scene_tooth_transforms', {}).items():
            if not np.allclose(body_transform(model, data, jaw), transform, atol=1e-9):
                raise ValueError('Dental pose changed; specify the original --jaw-opening-deg')
        trajectory, robot = None, Robot(model, data)
    with np.load(directory/'geometry.npz', allow_pickle=False) as saved:
        geometry = dict(saved)
    planes = json.loads((directory/'tooth_planes.json').read_text(encoding='utf-8'))
    world = report.get('world_from_scan_m')
    if world is None:
        world = request.get('world_from_scan_m')
        if world is None:
            world = body_transform(model, data, request['inputs']['jaw'])
    world = np.asarray(world)
    if report.get('path_kind') in CONTINUOUS_PATH_KINDS:
        from planning.motion_planning.contract import load_sweep_reference
        geometry['continuous_reference'] = load_sweep_reference(directory, report)
    geometry['preview_curves'] = preview_curves(geometry, world, trajectory, robot, report)
    for i in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) or ''
        if 'chair' in name:
            model.geom_rgba[i, 3] = 0.
    return model, data, robot, geometry, planes, world, trajectory, report


def draw(scene, data, robot, geometry, planes, world, report, selected):
    for points, color in geometry['preview_curves']:
        for a, b in zip(points[:-1], points[1:]):
            if scene.ngeom >= scene.maxgeom-1:
                break
            add_trajectory_line(scene, a, b, color, 3.)
    if scene.ngeom < scene.maxgeom:
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.full(3, .00045), data.site_xpos[robot.site], np.eye(3).ravel(),
                            np.array([1., .15, .2, 1.], dtype=np.float32))
        scene.ngeom += 1


def make_camera(geometry, world, preset=10, distance=None):
    curves = geometry['preview_curves']
    points = (np.concatenate([points for points, color in curves]) if curves else
              geometry['points_scan_m']@world[:3, :3].T+world[:3, 3])
    if len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('At least three finite points are required for a preview')
    _, _, principal = np.linalg.svd(points-points.mean(0), full_matrices=False)
    if 'continuous_reference' in geometry:
        from planning.pose_search.constraints import surface_frame
        path = geometry['continuous_reference']
        s = path.end*.5
        tangent, binormal, normal = (world[:3, :3]@surface_frame(path.evaluate(s, 1), path.normal(s))).T
    else:
        tangent, binormal, normal = principal
    directions = [binormal, -binormal, .8*binormal+.6*normal, -.8*binormal+.6*normal,
                  normal, .65*normal+.76*tangent, .65*normal-.76*tangent,
                  .65*binormal+.4*normal+.65*tangent,
                  principal[-1], -principal[-1],
                  .9*principal[-1]+.4*principal[1], -.9*principal[-1]+.4*principal[1]]
    direction = directions[preset]/np.linalg.norm(directions[preset])
    camera = mujoco.MjvCamera()
    camera.lookat[:] = (points.min(0)+points.max(0))*.5
    camera.distance = distance if distance is not None else max(.075, 1.8*np.linalg.norm(np.ptp(points, axis=0)))
    camera.azimuth = np.degrees(np.arctan2(direction[1], direction[0]))
    camera.elevation = -np.degrees(np.arcsin(np.clip(direction[2], -1., 1.)))
    return camera


def export(scene, camera, options, args, start_time):
    from PIL import Image, ImageDraw, ImageFont
    model, data, robot, geometry, planes, world, trajectory, report = scene
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, 960)
    model.vis.global_.offheight = max(model.vis.global_.offheight, 720)

    def render(renderer, time_s):
        if trajectory is not None:
            data.qpos[robot.qadr] = trajectory.sample(time_s)['q']
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera, scene_option=options)
        draw(renderer.scene, data, robot, geometry, planes, world, report, 0)
        return Image.fromarray(renderer.render())

    with mujoco.Renderer(model, height=720, width=960) as renderer:
        if args.snapshot:
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            render(renderer, start_time).save(args.snapshot)
            print(args.snapshot.resolve(), flush=True)
        if args.gif:
            times, delay_ms = animation_times(trajectory.duration, args.speed, args.fps)
            images = []
            try:
                font = ImageFont.load_default(size=18)
            except TypeError:
                font = ImageFont.load_default()
            for i, time_s in enumerate(times):
                image = render(renderer, time_s).resize((768, 576))
                label = ImageDraw.Draw(image)
                label.rectangle((0, 0, 768, 32), fill=(247, 248, 250))
                phase = trajectory.sample(time_s)['phase'].upper()
                label.text((12, 10), f'{time_s:05.1f} / {trajectory.duration:g} s | {phase} | {args.speed:g}x preview',
                           fill=(26, 30, 35), font=font)
                images.append(image.convert('P', palette=Image.Palette.ADAPTIVE))
                if i % 50 == 0:
                    print(f'Rendered {i+1}/{len(times)} frames', flush=True)
            args.gif.parent.mkdir(parents=True, exist_ok=True)
            images[0].save(args.gif, format='GIF', save_all=True, append_images=images[1:],
                           duration=delay_ms, loop=0, optimize=False)
            print(args.gif.resolve(), flush=True)


def show(scene, camera, options, args, start_time):
    from mujoco import viewer
    model, data, robot, geometry, planes, world, trajectory, report = scene
    playback = Playback(trajectory, args.speed, start_time, args.paused or args.waypoint is not None) if trajectory is not None else None
    # GLFW callbacks enqueue input; the render loop owns the playback state.
    from collections import deque
    keys = deque()
    print('Space: pause/resume | N/P: next/previous waypoint | R: restart | mouse: camera', flush=True)
    with viewer.launch_passive(model, data, key_callback=keys.append) as active:
        active.cam.lookat[:] = camera.lookat
        active.cam.distance, active.cam.azimuth, active.cam.elevation = camera.distance, camera.azimuth, camera.elevation
        active.opt.geomgroup[:] = options.geomgroup
        active.opt.sitegroup[:] = options.sitegroup
        previous = time.monotonic()
        while active.is_running():
            now = time.monotonic()
            with active.lock():
                if playback is not None:
                    playback.advance(now-previous)
                    while keys:
                        playback.key(keys.popleft())
                    sample = trajectory.sample(playback.time_s)
                    data.qpos[robot.qadr] = sample['q']
                else:
                    keys.clear()
                mujoco.mj_forward(model, data)
                active.user_scn.ngeom = 0
                draw(active.user_scn, data, robot, geometry, planes, world, report, 0)
            if playback is not None and hasattr(active, 'set_texts'):
                status = 'PAUSED' if playback.paused else 'PLAYING'
                active.set_texts([(mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                   'Plan time\nPhase\nPlayback',
                                   f'{playback.time_s:05.1f} / {trajectory.duration:g} s\n{sample["phase"]}\n{args.speed:g}x | {status}')])
            active.sync()
            previous = now
            time.sleep(.02)


def positive(value):
    number = float(value)
    if not np.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be positive and finite')
    return number


def main(argv=None):
    from pipeline.parameters import add_parameter_file, parse_parameters
    parser = argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--jaw-opening-deg', type=float,
                        help='Override saved jaw opening; must match the validated dental pose')
    parser.add_argument('--speed', type=positive, default=5., help='Preview playback multiplier, without changing saved timing (default: 5)')
    parser.add_argument('--paused', action=argparse.BooleanOptionalAction, default=False)
    start = parser.add_mutually_exclusive_group()
    start.add_argument('--time', type=float, help='Initial saved-plan time in seconds')
    start.add_argument('--waypoint', type=int, help='Select a waypoint and start paused')
    parser.add_argument('--camera', type=int, choices=range(12), default=10, help='Camera preset (default: 10)')
    parser.add_argument('--distance', type=positive, help='Camera distance in meters (default: fit path)')
    parser.add_argument('--snapshot', type=Path, help='Export a PNG and exit')
    parser.add_argument('--gif', type=Path, help='Export a full-trajectory GIF and exit')
    parser.add_argument('--fps', type=positive, default=10., help='GIF frame rate, up to 50 (default: 10)')
    args = parse_parameters(parser, argv)
    try:
        if args.fps > 50:
            raise ValueError('--fps must not exceed 50')
        if args.jaw_opening_deg is not None and not np.isfinite(args.jaw_opening_deg):
            raise ValueError('--jaw-opening-deg must be finite')
        scene = load_scene(args.plan, jaw_opening_deg=args.jaw_opening_deg)
        model, data, robot, geometry, planes, world, trajectory, report = scene
        if trajectory is None:
            if args.gif or args.time is not None or args.waypoint is not None:
                raise ValueError('This is a geometry-only plan; motion playback/export requires trajectory.npz')
            start_time = 0.
        elif args.waypoint is not None:
            if not 0 <= args.waypoint < len(trajectory.times):
                raise ValueError(f'--waypoint must be between 0 and {len(trajectory.times)-1}')
            start_time = float(trajectory.times[args.waypoint])
        else:
            start_time = args.time if args.time is not None else (trajectory.duration*.5 if args.snapshot else 0.)
            Playback(trajectory, args.speed, start_time)
        if args.gif:
            animation_times(trajectory.duration, args.speed, args.fps)
        camera = make_camera(geometry, world, args.camera, args.distance)
        options = mujoco.MjvOption()
        options.geomgroup[3] = 0
        options.sitegroup[:] = 0
        model.vis.map.znear = .0001
        print(f'Preview plan: {args.plan.resolve()}', flush=True)
        print(f'Path: {report.get("path_kind", "geometry_only")} | '
              f'Duration: {trajectory.duration if trajectory is not None else 0.:g} s | '
              f'Playback: {args.speed:g}x | simulation preview only', flush=True)
        if args.snapshot or args.gif:
            export(scene, camera, options, args, start_time)
        else:
            show(scene, camera, options, args, start_time)
    except (ValueError, RuntimeError, OSError, KeyError) as error:
        print(f'Preview failed: {error}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
