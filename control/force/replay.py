"""Interactive, read-only replay of a completed single-tooth force experiment."""
import argparse
import csv
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from planning.infrastructure.io import fingerprint
from pipeline.simulation import DENTAL_MODEL_PATH, create_simulation
from pipeline.oral import jaw_overrides, load_jaw_settings
from .compliance import ForceControlConfig
from .contact_path import ContactPathConfig
from .force_history import CHANNELS
from .force_panel import panel_rows, render_panel


@dataclass
class Recording:
    report: dict
    rows: list
    times: np.ndarray
    joints: np.ndarray
    history: np.ndarray
    contact_times: np.ndarray
    contact_points: np.ndarray
    trace_start: float
    trace_end: float
    first_contact: float
    release_end: float

    def index_at(self, t):
        return min(len(self.times)-1, max(0, int(np.searchsorted(self.times, t, side='left'))))

    def contacts_at(self, index):
        t = self.times[index]
        lo = np.searchsorted(self.contact_times, t-.0005, side='left')
        hi = np.searchsorted(self.contact_times, t+.0005, side='right')
        return self.contact_points[lo:hi]


def load_recording(run):
    run = Path(run).resolve()
    report = json.loads((run/'force_trajectory_report.json').read_text(encoding='utf-8'))
    if (report.get('status') != 'completed' or report.get('stage_passed') is not True
            or report.get('experiment_stage') not in ('hold', 'short')):
        raise ValueError('Replay requires a completed, passed hold or short experiment')
    if report.get('source_scene', {}).get('sha256') != fingerprint(DENTAL_MODEL_PATH)['sha256']:
        raise ValueError('Saved experiment scene differs from the current MuJoCo scene')
    with (run/'trajectory_force.csv').open(encoding='utf-8-sig', newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('Force trajectory is empty')
    names = ('ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
             'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint')
    times = np.asarray([float(row['time_s']) for row in rows])
    joints = np.asarray([[float(row[name]) for name in names] for row in rows])
    history = np.asarray([[float(row['time_s']), *(float(row[k]) for k in CHANNELS)] for row in rows])
    if (not np.isfinite(times).all() or not np.isfinite(joints).all()
            or not np.isfinite(history).all() or np.any(np.diff(times) <= 0)):
        raise ValueError('Force trajectory contains invalid time, pose or signal samples')
    trace = [i for i, row in enumerate(rows) if row['phase'] == 'trace']
    release = [i for i, row in enumerate(rows) if row['phase'] == 'release']
    if not trace or not release:
        raise ValueError('Passed recording lacks its contact or release phase')
    with (run/'trajectory_contacts.csv').open(encoding='utf-8-sig', newline='') as stream:
        hits = list(csv.DictReader(stream))
    if not hits:
        raise ValueError('Passed recording has no saved contact points')
    contact_times = np.asarray([float(row['time_s']) for row in hits])
    contact_points = np.asarray([[float(row[k]) for k in ('x_m', 'y_m', 'z_m')] for row in hits])
    if (not np.isfinite(contact_times).all() or not np.isfinite(contact_points).all()
            or np.any(np.diff(contact_times) < 0)):
        raise ValueError('Contact log contains invalid coordinates or order')
    return Recording(report, rows, times, joints, history, contact_times, contact_points,
                     float(times[trace[0]]), float(times[trace[-1]]), float(contact_times[0]),
                     float(times[release[-1]]))


@dataclass
class Playback:
    start: float
    end: float
    time_s: float
    speed: float = 1.
    paused: bool = False
    show_sensor: bool = False

    def advance(self, wall_seconds):
        if not self.paused:
            self.time_s = min(self.end, self.time_s + max(0., wall_seconds)*self.speed)
            if self.time_s >= self.end:
                self.paused = True

    def key(self, key):
        if key == ord(' '):
            self.paused = not self.paused
        elif key in (ord('R'), ord('r')):
            self.time_s, self.paused = self.start, False
        elif key in (ord('+'), ord('='), 334):
            self.speed = min(8., self.speed*2.)
        elif key in (ord('-'), 333):
            self.speed = max(.125, self.speed/2.)
        elif key in (ord('H'), ord('h')):
            self.show_sensor = not self.show_sensor


def make_scene(recording):
    report = recording.report
    source = Path(report['source'])
    saved = json.loads((source/'planning_report.json').read_text(encoding='utf-8'))
    options = {'robot_mount_position_m': saved['robot_mount_position_m']}
    if report.get('source_jaw_opening_deg') is not None:
        options['jaw_settings'] = jaw_overrides(load_jaw_settings(), report['source_jaw_opening_deg'])
    model, data = create_simulation(**options)
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    sensor = model.geom('ur_force_sensor_geom').id
    sensor_rgba = model.geom_rgba[sensor].copy()
    model.geom_rgba[sensor, 3] = 0.
    from planning.infrastructure.robot import Robot
    robot = Robot(model, data)
    return model, data, robot, sensor, sensor_rgba


def draw_contacts(scene, points):
    scene.ngeom = 0
    for point in points:
        if scene.ngeom >= scene.maxgeom:
            break
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                           np.full(3, .00035), point, np.eye(3).ravel(),
                           np.array([1., .12, .16, 1.], dtype=np.float32))
        scene.ngeom += 1


def show(recording, start=None, speed=1.):
    from mujoco import viewer
    if not np.isfinite(speed) or not .125 <= speed <= 8.:
        raise ValueError('Playback speed must be between 0.125 and 8')
    end = min(float(recording.times[-1]), recording.release_end+1.)
    begin = max(float(recording.times[0]), recording.first_contact-1.) if start is None else float(start)
    if not np.isfinite(begin) or not recording.times[0] <= begin < end:
        raise ValueError('Replay start must precede release and lie within the recording')
    playback = Playback(begin, end, begin, speed)
    model, data, robot, sensor, sensor_rgba = make_scene(recording)
    force = ForceControlConfig(**recording.report['force_control'])
    path = ContactPathConfig(**recording.report['path_control'])
    backdrop = mujoco.MjvFigure()
    backdrop.figurergba[:] = (.063, .110, .173, 1.)
    backdrop.panergba[:] = backdrop.figurergba
    backdrop.flg_ticklabel[:] = False
    backdrop.gridsize[:] = 0
    keys = deque()
    print('记录回放：空格 暂停/继续 | R 重播 | +/- 调速 | H 显示/隐藏黄色传感器外壳', flush=True)
    with viewer.launch_passive(model, data, key_callback=keys.append,
                               show_left_ui=False, show_right_ui=False) as window:
        with window.lock():
            root = data.body('dental_teeth_root')
            direction = root.xmat.reshape(3, 3)[:, 2]
            contact = recording.contact_points[np.searchsorted(recording.contact_times, recording.trace_start)]
            window.cam.lookat[:] = contact
            window.cam.distance = .09
            window.cam.azimuth = np.degrees(np.arctan2(direction[1], direction[0]))+180.
            window.cam.elevation = -np.degrees(np.arcsin(np.clip(direction[2], -1., 1.)))
            window.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        last = time.monotonic()
        last_panel = -float('inf')
        while window.is_running():
            now = time.monotonic()
            while keys:
                playback.key(keys.popleft())
            playback.advance(now-last)
            last = now
            index = recording.index_at(playback.time_s)
            row = recording.rows[index]
            with window.lock():
                model.geom_rgba[sensor] = sensor_rgba if playback.show_sensor else np.r_[sensor_rgba[:3], 0.]
                data.qpos[robot.qadr] = recording.joints[index]
                mujoco.mj_forward(model, data)
                draw_contacts(window.user_scn, recording.contacts_at(index))
            window.sync()
            if now-last_panel >= .1:
                last_panel = now
                viewport = window.viewport
                if viewport is not None and viewport.width > 0 and viewport.height > 0:
                    width = min(560, max(1, int(viewport.width*.42)))
                    phase_progress = np.clip((playback.time_s-recording.trace_start)/
                                             max(recording.trace_end-recording.trace_start, 1e-9), 0., 1.)
                    status = f'记录回放 / {playback.speed:g}x' + (' / 已暂停' if playback.paused else '')
                    rows = panel_rows(row, force, path, .001, phase_progress, status,
                                      stage=recording.report['experiment_stage'])
                    lo = np.searchsorted(recording.times, playback.time_s-10., side='left')
                    history = recording.history[lo:index+1]
                    panel = render_panel(rows, width, viewport.height, history=history)
                    rect = mujoco.MjrRect(viewport.left+viewport.width-width,
                                          viewport.bottom, width, viewport.height)
                    window.set_figures((rect, backdrop))
                    window.set_images((rect, panel))
            time.sleep(max(0., 1/30-(time.monotonic()-now)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Completed hold or short experiment directory')
    parser.add_argument('--start', type=float, help='Replay start in recorded simulation seconds')
    parser.add_argument('--speed', type=float, default=1., help='Playback speed; default 1x')
    args = parser.parse_args(argv)
    try:
        show(load_recording(args.run), args.start, args.speed)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
