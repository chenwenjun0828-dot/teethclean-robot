"""Preview and resize the white oral aperture without running registration/IK."""
from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
from queue import Empty, SimpleQueue
import time

import mujoco
import numpy as np

from pipeline.mouth import (CONFIG_PATH, MouthOpening, MouthSettings, load_settings,
                             save_settings, with_overrides)
from pipeline.simulation import create_simulation


def closeup_camera(camera, data):
    root = data.body('dental_teeth_root')
    rotation = root.xmat.reshape(3, 3)
    target = root.xpos + rotation @ np.array([0., .010, .025])
    direction = rotation[:, 2]
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = target
    camera.distance = .28
    # MuJoCo azimuth/elevation describe the direction from eye to target.
    camera.azimuth = np.degrees(np.arctan2(direction[1], direction[0])) + 180.
    camera.elevation = -np.degrees(np.arcsin(direction[2]))


def render_preview(model, data, path):
    from PIL import Image
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(model, height=720, width=960) as renderer:
        camera = mujoco.MjvCamera()
        closeup_camera(camera, data)
        renderer.update_scene(data, camera=camera)
        Image.fromarray(renderer.render()).save(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--width-mm', type=float)
    parser.add_argument('--height-mm', type=float)
    parser.add_argument('--forward-mm', type=float)
    parser.add_argument('--save', action='store_true', help='Save supplied settings as pipeline defaults')
    parser.add_argument('--render', type=Path, help='Save a preview PNG and exit without a viewer')
    args = parser.parse_args(argv)
    settings = with_overrides(load_settings(), args.width_mm, args.height_mm,
                              args.forward_mm, enabled=True)
    model, data = create_simulation(mouth_settings=settings)
    # The full chair obscures a close-up when orbiting behind the teeth.
    model.geom_rgba[model.geom('dental_dental_chair_visual').id, 3] = 0.
    mouth = MouthOpening(model)
    if args.save:
        save_settings(settings)
        print(f'Saved: {CONFIG_PATH}')
    if args.render:
        render_preview(model, data, args.render)
        print(f'Preview: {args.render.resolve()}')
        return 0

    from mujoco import viewer
    pending = SimpleQueue()
    print('White mouth preview (no robot motion). Dimensions in mm.')
    print('J/L: width -/+ 2; K/I: height -/+ 2; U/O: forward -/+ 2')
    print('Arrows: shift left/right/up/down; S: save; R: reset; M: show/hide; C: front view')
    print('Mouse: orbit/zoom. Close the window to exit. Changes persist only after S.')
    with viewer.launch_passive(model, data, key_callback=pending.put) as window:
        with window.lock():
            closeup_camera(window.cam, data)
        while window.is_running():
            changed = False
            while True:
                try:
                    key = pending.get_nowait()
                except Empty:
                    break
                try:
                    if key in (ord('J'), ord('L')):
                        settings = replace(settings, width_mm=settings.width_mm+(-2 if key == ord('J') else 2))
                    elif key in (ord('K'), ord('I')):
                        settings = replace(settings, height_mm=settings.height_mm+(-2 if key == ord('K') else 2))
                    elif key in (ord('U'), ord('O'), 262, 263, 264, 265):
                        center = list(settings.center_mm)
                        axis, amount = {ord('U'): (2, -2), ord('O'): (2, 2),
                                        262: (0, -2), 263: (0, 2),
                                        264: (1, -2), 265: (1, 2)}[key]
                        center[axis] += amount
                        settings = replace(settings, center_mm=center)
                    elif key == ord('R'):
                        settings = MouthSettings()
                    elif key == ord('M'):
                        settings = replace(settings, enabled=not settings.enabled)
                    elif key == ord('S'):
                        save_settings(settings)
                        print(f'Saved: {CONFIG_PATH}')
                        continue
                    elif key == ord('C'):
                        with window.lock():
                            closeup_camera(window.cam, data)
                        continue
                    else:
                        continue
                    changed = True
                except ValueError as error:
                    print(error)
            if changed:
                with window.lock():
                    mouth.update(settings, data)
                # MuJoCo's upload takes its own lock; calling inside lock deadlocks.
                window.update_mesh(mouth.mesh_id)
                print(f'Opening {settings.width_mm:g} x {settings.height_mm:g}; '
                      f'centre {settings.center_mm}; visible={settings.enabled}')
            window.sync()
            time.sleep(.02)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
