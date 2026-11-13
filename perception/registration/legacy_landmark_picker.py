"""Sequential landmark picking using the legacy OpenGL visualizer."""
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import open3d as o3d

from .landmark_selection import PairSelection


def _camera_reference(viewer, cloud, indices):
    """Keep the chosen view and number the selected points from one."""
    import cv2
    rgb = np.asarray(viewer.capture_screen_float_buffer(do_render=True))
    image = np.rint(rgb[:, :, ::-1] * 255).astype(np.uint8)
    parameters = viewer.get_view_control().convert_to_pinhole_camera_parameters()
    points = np.asarray(cloud.points)[indices]
    camera_points = points @ parameters.extrinsic[:3, :3].T + parameters.extrinsic[:3, 3]
    pixels = camera_points @ parameters.intrinsic.intrinsic_matrix.T
    for number, (pixel, point) in enumerate(zip(pixels, camera_points), 1):
        if point[2] <= 0:
            continue
        x, y = np.rint(pixel[:2] / pixel[2]).astype(int)
        if not (0 <= x < image.shape[1] and 0 <= y < image.shape[0]):
            continue
        cv2.circle(image, (x, y), 6, (0, 180, 255), -1)
        label = (max(0, min(x + 8, image.shape[1] - 35)), max(25, y - 8))
        cv2.putText(image, str(number), label, cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 0, 0), 5)
        cv2.putText(image, str(number), label, cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    return image


def _pick_cloud(cloud, title, *, camera=False, reference=None):
    viewer = o3d.visualization.VisualizerWithEditing()
    try:
        if not viewer.create_window(title, width=1100, height=800):
            raise RuntimeError('Could not create the compatibility landmark window')
        viewer.add_geometry(cloud)
        viewer.get_render_option().point_size = 3.
        control = viewer.get_view_control()
        control.set_lookat(cloud.get_center())
        control.set_front([0, 0, -1] if camera else [0, 0, 1])
        control.set_up([0, -1, 0] if camera else [0, 1, 0])
        control.set_zoom(.7)
        viewer.run()
        indices = list(viewer.get_picked_points())
        if camera and reference is not None and indices:
            reference['image'] = _camera_reference(viewer, cloud, indices)
        return indices
    finally:
        viewer.destroy_window()


def _cloud_hash(cloud):
    return hashlib.sha256(np.asarray(cloud.points, dtype='<f8').tobytes()).hexdigest()


@contextmanager
def _reference_window(path):
    """Keep the reference UI in another process, away from the GLFW event loop."""
    process = None
    try:
        if path is not None:
            process = subprocess.Popen(
                [sys.executable, '-B', '-m', 'perception.registration.landmark_reference', str(path)],
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == 'win32' else 0)
        yield
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def pick_pairs_legacy(scan_cloud, camera_cloud, minimum_pairs=6, *, reference_path=None,
                      camera_landmarks=None):
    """Pick camera landmarks, then the same ordered landmarks on the scan."""
    selection = PairSelection(np.asarray(scan_cloud.points),
                              np.asarray(camera_cloud.points), minimum_pairs)
    print('Compatibility picker: left-drag rotates, wheel zooms; Shift+left picks, '
          'Shift+right undoes; Q closes each window.', flush=True)
    print(f'1/2: select at least {minimum_pairs} CAMERA points. '
          'Remember their order, then press Q.', flush=True)
    reference = {}
    fingerprints = dict(camera_points_sha256=_cloud_hash(camera_cloud),
                        scan_points_sha256=_cloud_hash(scan_cloud))
    reference_path = Path(reference_path) if reference_path is not None else None
    if camera_landmarks is not None:
        checkpoint_path = Path(camera_landmarks)
        checkpoint = json.loads(checkpoint_path.read_text(encoding='utf-8'))
        if any(checkpoint.get(key) != value for key, value in fingerprints.items()):
            raise ValueError('Camera landmark checkpoint does not match this camera cloud and scan')
        camera_indices = checkpoint['camera_indices']
        image_path = checkpoint_path.parent / checkpoint['reference_image']
        if not image_path.is_file():
            raise ValueError('Camera landmark reference image is missing')
        if reference_path is not None and image_path.resolve() != reference_path.resolve():
            shutil.copyfile(image_path, reference_path)
        elif reference_path is None:
            reference_path = image_path
        print(f'Resumed {len(camera_indices)} saved CAMERA landmarks.', flush=True)
    else:
        camera_indices = _pick_cloud(camera_cloud,
            f'1/2 CAMERA - select at least {minimum_pairs} points, Q to continue',
            camera=True, reference=reference)
    if len(camera_indices) < minimum_pairs:
        raise ValueError(f'Camera selection cancelled or incomplete: need at least '
                         f'{minimum_pairs} points, got {len(camera_indices)}')
    if (len(set(camera_indices)) != len(camera_indices) or
            any(isinstance(i, bool) or not isinstance(i, int) or
                not 0 <= i < len(camera_cloud.points) for i in camera_indices)):
        raise ValueError('Invalid or duplicate camera landmark indices')
    if reference_path is not None and 'image' in reference:
        import cv2
        if not cv2.imwrite(str(reference_path), reference['image']):
            raise RuntimeError(f'Could not save camera landmark reference: {reference_path}')
    if reference_path is not None:
        checkpoint = dict(**fingerprints, camera_indices=camera_indices,
                          reference_image=reference_path.name)
        reference_path.with_suffix('.json').write_text(json.dumps(checkpoint, indent=2), encoding='utf-8')
    print(f'2/2: select exactly {len(camera_indices)} SCAN points in the same order, '
          'then press Q.', flush=True)
    print('The CAMERA reference image remains open with numbered landmarks 1, 2, 3... .', flush=True)
    with _reference_window(reference_path):
        scan_indices = _pick_cloud(scan_cloud,
            f'2/2 SCAN - select {len(camera_indices)} matching points in order, Q to register')
    if len(scan_indices) != len(camera_indices):
        raise ValueError(f'Correspondence counts differ: camera={len(camera_indices)}, '
                         f'scan={len(scan_indices)}; select the same number in the same order')
    for camera_index, scan_index in zip(camera_indices, scan_indices):
        selection.select('camera', camera_index)
        selection.select('scan', scan_index)
    return selection.result()
