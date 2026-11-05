"""Actual Open3D GUI smoke test in a child process; no human landmarks implied."""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


class PairedPickerGUITests(unittest.TestCase):
    def run_gui_probe(self, mode):
        self.assertTrue((ROOT/"perception/registration/landmark_picker.py").is_file(),
                        "Side-by-side GUI module is missing")
        environment = dict(os.environ, OMP_NUM_THREADS="4")
        with subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--smoke", mode, "--debug"],
                              cwd=ROOT, env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True, encoding="utf-8") as process:
            try:
                stdout, stderr = process.communicate(timeout=65)
            except subprocess.TimeoutExpired:
                if os.name == "nt":
                    subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True)
                else:
                    process.kill()
                stdout, stderr = process.communicate()
                self.fail("GUI probe timed out:\n"+stdout+stderr)
        self.assertEqual(process.returncode, 0, stdout+stderr)
        self.assertIn("PAIRED_GUI_OK", stdout)

    def test_real_window_depth_pick_pairs_and_finish(self):
        self.run_gui_probe("--finish")

    def test_cancel_during_pending_depth_pick_returns_no_pairs(self):
        self.run_gui_probe("--cancel")


def smoke():
    import numpy as np
    import open3d as o3d
    from open3d.visualization import gui
    from perception.registration.landmark_picker import PairedLandmarkWindow
    points = np.array([[x, y, 0.] for y in (-10., 10.) for x in (-15., 0., 15.)])
    # Give the renderer a genuine 3-D extent, with visible landmarks in front.
    rng = np.random.default_rng(7)
    background = np.column_stack((rng.uniform(-15, 15, 600), rng.uniform(-10, 10, 600),
                                  rng.uniform(-10, -5, 600)))
    scan_points = np.vstack((points, background))
    rotation = np.diag([1., -1., -1.])
    camera_points = scan_points@rotation.T+[10, -20, 150]
    scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(scan_points))
    camera = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(camera_points))
    app = gui.Application.instance
    app.initialize()
    picker = PairedLandmarkWindow(scan, camera, 6)
    if "--debug" in sys.argv:
        original_apply = picker._apply_depth
        def trace_depth(*args):
            print("DEPTH_READY", args[0], args[5].shape, "serial", args[6], "pending", picker._pending_depth, flush=True)
            original_apply(*args)
        picker._apply_depth = trace_depth
    started = time.monotonic()
    background_done = False
    undo_done = False
    resized = False
    timed_out = False
    last_notice = ""
    def drive_ui():
        nonlocal background_done, undo_done, resized, timed_out, last_notice
        if time.monotonic()-started > 45 and not picker._closed:
            timed_out = True
            picker._cancel()
            return
        if picker._closed or picker._busy:
            return
        if "--debug" in sys.argv and picker.status.text != last_notice:
            last_notice = picker.status.text
            rectangle = picker.views["camera"].frame
            content = picker.window.content_rect
            print("GUI_STATE", last_notice, rectangle.x, rectangle.y, rectangle.width, rectangle.height,
                  "window", content.width, content.height, "visible", picker.window.is_visible, flush=True)
        widget = picker.views["camera"]
        if widget.frame.width < 100:
            return
        if time.monotonic()-started < .5:
            return
        if not background_done:
            assert not picker.finish_button.enabled
            picker.request_pick("scan", 10, 10)
            assert picker.selection.pending_camera is None
            picker._finish()
            assert not picker._closed
            picker.request_pick("camera", 0, 0)
            background_done = True
            return
        if "--cancel" in sys.argv:
            picker.request_pick("camera", widget.frame.width//2, widget.frame.height//2)
            picker._cancel()
            return
        if picker.selection.pair_count == 1 and not undo_done:
            # Undo via the same mouse callback used by Shift+right.
            from types import SimpleNamespace
            picker._on_mouse("camera", SimpleNamespace(
                type=gui.MouseEvent.Type.BUTTON_DOWN,
                is_modifier_down=lambda modifier: modifier==gui.KeyModifier.SHIFT,
                is_button_down=lambda button: button==gui.MouseButton.RIGHT))
            assert picker.selection.pair_count == 0
            undo_done = True
            return
        if picker.selection.pair_count == 2 and not resized:
            frame = picker.window.os_frame
            picker.window.os_frame = gui.Rect(frame.x, frame.y, 1200, 760)
            resized = True
            return
        if picker.selection.pair_count == 6:
            assert picker.finish_button.enabled
            camera_rect = picker.views["camera"].frame
            scan_rect = picker.views["scan"].frame
            assert camera_rect.x + camera_rect.width <= scan_rect.x
            assert camera_rect.height == scan_rect.height
            assert picker.headings["camera"].frame.x == camera_rect.x
            assert picker.headings["scan"].frame.x == scan_rect.x
            result = picker.selection.result()
            np.testing.assert_array_equal(result[0], points)
            np.testing.assert_array_equal(result[1], camera_points[:6])
            assert result[2] == list(range(6)) and result[3] == list(range(6))
            picker._update_label_positions()
            for side in ("camera", "scan"):
                assert len(picker._marker_names[side]) == 6
                assert len(picker._label_points[side]) == 6
                for point, label in picker._label_points[side]:
                    assert label.visible
                    assert label.frame.y >= picker.views[side].frame.y
            picker._finish()
            return
        side = picker.selection.next_side
        index = picker.selection.pair_count
        widget = picker.views[side]
        point = camera_points[index] if side == "camera" else points[index]
        clip = np.asarray(widget.scene.camera.get_projection_matrix()) @ np.asarray(widget.scene.camera.get_view_matrix()) @ np.r_[point, 1.]
        ndc = clip[:3]/clip[3]
        x = int(round((ndc[0]+1)*widget.frame.width/2))
        y = int(round((1-ndc[1])*widget.frame.height/2))
        from types import SimpleNamespace
        picker._on_mouse(side, SimpleNamespace(
            type=gui.MouseEvent.Type.BUTTON_DOWN,
            is_modifier_down=lambda modifier: modifier==gui.KeyModifier.SHIFT,
            is_button_down=lambda button: button==gui.MouseButton.LEFT,
            x=x+widget.frame.x, y=y+widget.frame.y))

    while app.run_one_tick():
        if not picker._closed:
            # Open3D activates the correct GL context for posted GUI callbacks.
            app.post_to_main_thread(picker.window, drive_ui)
    assert not timed_out, "Timed out waiting for asynchronous GUI depth picking"
    assert (picker.result_value is None) == ("--cancel" in sys.argv)
    print("PAIRED_GUI_OK", flush=True)


if __name__ == "__main__":
    if "--smoke" in sys.argv:
        sys.path.insert(0, str(ROOT))
        smoke()
    else:
        unittest.main()
