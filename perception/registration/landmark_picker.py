"""One-window, left-camera/right-scan, pair-by-pair landmark selection."""
from __future__ import annotations
import numpy as np
import open3d as o3d
from open3d.visualization import gui, rendering
from .landmark_selection import PairSelection, nearest_depth_pixel, project_label_pixel


class PairedLandmarkWindow:
    COLORS = ((.95, .22, .22), (.10, .65, 1.), (.15, .8, .3), (.95, .45, .1),
              (.7, .3, 1.), (.1, .85, .8), (1., .25, .65), (.65, .65, .1))

    def __init__(self, scan_cloud, camera_cloud, minimum_pairs=6):
        self.app = gui.Application.instance
        self.selection = PairSelection(np.asarray(scan_cloud.points),
                                       np.asarray(camera_cloud.points), minimum_pairs)
        self.window = self.app.create_window("Paired Landmarks - Camera | Scan", 1440, 900)
        self.result_value = None
        self._closed = False
        self._busy = False
        self._pending_depth = 0
        self._cancel_requested = False
        self._request_serial = 0
        self.views = {}
        self.clouds = {"camera": camera_cloud, "scan": scan_cloud}
        self.trees = {side: o3d.geometry.KDTreeFlann(cloud) for side, cloud in self.clouds.items()}
        self._marker_names = {"camera": [], "scan": []}
        self._labels = {"camera": [], "scan": []}
        self._label_points = {"camera": [], "scan": []}
        self.headings = {"camera": gui.Label("LEFT: Camera cloud [mm]"),
                         "scan": gui.Label("RIGHT: Original scan model [mm]")}
        self.status = gui.Label("")
        self.help = gui.Label("Shift+left: pick | Shift+right / Backspace: undo | Q / Enter: finish | Esc: cancel")
        self.header = gui.Vert(6)
        self.header.add_child(self.status)
        self.header.add_child(self.help)
        self.toolbar = gui.Horiz(8)
        for text, handler in (("Undo", self._undo), ("Clear pairs", self._clear),
                              ("Reset views", self._reset_views), ("Cancel", self._cancel)):
            button = gui.Button(text)
            button.set_on_clicked(handler)
            self.toolbar.add_child(button)
        self.finish_button = gui.Button("Finish & Register")
        self.finish_button.set_on_clicked(self._finish)
        self.toolbar.add_child(self.finish_button)
        for side, cloud in self.clouds.items():
            widget = gui.SceneWidget()
            widget.scene = rendering.Open3DScene(self.window.renderer)
            widget.scene.set_background([.10, .12, .15, 1.])
            material = rendering.MaterialRecord()
            material.shader = "defaultUnlit"
            material.point_size = 3*self.window.scaling
            material.base_color = [.85, .85, .85, 1.]
            widget.scene.add_geometry("cloud", cloud, material)
            widget.set_on_mouse(lambda event, name=side: self._on_mouse(name, event))
            self.views[side] = widget
            self.window.add_child(widget)
        for child in (self.header, self.toolbar):
            self.window.add_child(child)
        for heading in self.headings.values():
            self.window.add_child(heading)
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_key(self._on_key)
        self.window.set_on_close(self._on_close)
        self.window.set_on_tick_event(self._update_label_positions)
        self._reset_views()
        self._refresh()

    def _on_layout(self, context):
        rectangle = self.window.content_rect
        # Native minimize events can report 0x0. Keep valid camera viewports.
        if rectangle.width < 100 or rectangle.height < 100:
            return
        em = self.window.theme.font_size
        margin, gap = int(em*.6), int(em*.8)
        x = rectangle.x+margin
        width = max(1, rectangle.width-2*margin)
        constraints = gui.Widget.Constraints()
        constraints.width = width
        header_height = self.header.calc_preferred_size(context, constraints).height
        heading_height = int(em*1.8)
        top = header_height+2*margin+heading_height
        half = max(1, (width-gap)//2)
        height = max(1, rectangle.height-top-int(em*2.2)-margin)
        y = rectangle.y+top
        self.views["camera"].frame = gui.Rect(x, y, half, height)
        self.views["scan"].frame = gui.Rect(x+half+gap, y, width-half-gap, height)
        self.header.frame = gui.Rect(x, rectangle.y+margin, width, header_height)
        self.headings["camera"].frame = gui.Rect(x, y-heading_height, half, heading_height)
        self.headings["scan"].frame = gui.Rect(x+half+gap, y-heading_height, half, heading_height)
        self.toolbar.frame = gui.Rect(x, y+height+int(em*.3), width, int(em*1.8))

    def _invalidate_request(self):
        self._request_serial += 1
        self._busy = False

    def _reset_views(self):
        self._invalidate_request()
        for side, widget in self.views.items():
            bounds = self.clouds[side].get_axis_aligned_bounding_box()
            center = bounds.get_center()
            distance = max(float(np.linalg.norm(bounds.get_extent()))*1.25, 1.)
            widget.setup_camera(45, bounds, center)
            if side == "camera":
                widget.look_at(center, center-[0, 0, distance], [0, -1, 0])
            else:
                widget.look_at(center, center+[0, 0, distance], [0, 1, 0])
        if hasattr(self, "status"):
            self._refresh()

    def _refresh(self, notice=""):
        if self._closed:
            return
        count = self.selection.pair_count
        side = "LEFT camera" if self.selection.next_side == "camera" else "matching RIGHT scan"
        text = f"{count} complete pairs / {self.selection.minimum_pairs} required. Pair {count+1}: pick {side}."
        self.status.text = (notice+"  |  " if notice else "")+text
        self.finish_button.enabled = self.selection.can_finish and self._pending_depth == 0 and not self._cancel_requested
        self.window.post_redraw()

    def _update_markers(self):
        for side, widget in self.views.items():
            for name in self._marker_names[side]:
                widget.scene.remove_geometry(name)
            for label in self._labels[side]:
                label.visible = False
            self._marker_names[side].clear()
            self._label_points[side].clear()
            points = self.selection.camera_points if side == "camera" else self.selection.scan_points
            indices = self.selection.camera_indices if side == "camera" else self.selection.scan_indices
            radius = max(float(np.linalg.norm(self.clouds[side].get_axis_aligned_bounding_box().get_extent()))*.004, .05)
            marks = [(index, str(i+1), self.COLORS[i % len(self.COLORS)])
                     for i, index in enumerate(indices)]
            if side == "camera" and self.selection.pending_camera is not None:
                marks.append((self.selection.pending_camera, f"{len(indices)+1}?", (1., .85, .1)))
            for i, (index, text, color) in enumerate(marks):
                sphere = o3d.geometry.TriangleMesh.create_sphere(radius, resolution=8)
                sphere.compute_vertex_normals()
                sphere.translate(points[index])
                material = rendering.MaterialRecord()
                material.shader = "defaultUnlit"
                material.base_color = [*color, 1.]
                name = f"landmark_{i}"
                widget.scene.add_geometry(name, sphere, material)
                if i == len(self._labels[side]):
                    label = gui.Label("")
                    label.background_color = gui.Color(0, 0, 0, 0)
                    self._labels[side].append(label)
                    self.window.add_child(label)
                label = self._labels[side][i]
                label.text = text
                label.text_color = gui.Color(*color)
                label.visible = True
                self._marker_names[side].append(name)
                self._label_points[side].append((points[index], label))
            widget.force_redraw()
        self._update_label_positions()

    def _update_label_positions(self):
        if self._closed:
            return False
        changed = False
        for side, widget in self.views.items():
            rectangle = widget.frame
            view = np.asarray(widget.scene.camera.get_view_matrix())
            projection = np.asarray(widget.scene.camera.get_projection_matrix())
            for point, label in self._label_points[side]:
                pixel = project_label_pixel(point, view, projection,
                    (rectangle.x, rectangle.y, rectangle.width, rectangle.height))
                if pixel is None:
                    changed |= label.visible
                    label.visible = False
                    continue
                x, y = pixel
                size = int(self.window.theme.font_size*1.6)
                x = min(x+6, rectangle.x+rectangle.width-size*2)
                y = max(y-size, rectangle.y)
                previous = label.frame
                if not label.visible or (previous.x, previous.y) != (x, y):
                    label.frame = gui.Rect(x, y, max(size, len(label.text)*self.window.theme.font_size), size)
                    label.visible = True
                    changed = True
        return changed

    def _on_mouse(self, side, event):
        if event.type == gui.MouseEvent.Type.BUTTON_DOWN and event.is_modifier_down(gui.KeyModifier.SHIFT):
            if event.is_button_down(gui.MouseButton.RIGHT):
                self._undo()
            elif event.is_button_down(gui.MouseButton.LEFT):
                rectangle = self.views[side].frame
                self.request_pick(side, event.x-rectangle.x, event.y-rectangle.y)
            return gui.Widget.EventCallbackResult.CONSUMED
        if self._busy:
            return gui.Widget.EventCallbackResult.CONSUMED
        return gui.Widget.EventCallbackResult.IGNORED

    def request_pick(self, side, x, y):
        """Pick by local viewport pixels; used by both mouse input and GUI tests."""
        if self._closed or self._busy or self._cancel_requested:
            return
        if side != self.selection.next_side:
            self._refresh("Select the LEFT point first" if self.selection.next_side == "camera"
                          else "Complete the pair on the RIGHT before picking another LEFT point")
            return
        widget = self.views[side]
        width, height = widget.frame.width, widget.frame.height
        if not (0 <= x < width and 0 <= y < height):
            self._refresh("Click inside the point cloud")
            return
        self._busy = True
        self._pending_depth += 1
        self._request_serial += 1
        serial = self._request_serial
        view_matrix = np.asarray(widget.scene.camera.get_view_matrix()).copy()
        projection = np.asarray(widget.scene.camera.get_projection_matrix()).copy()
        self._refresh("Reading point under cursor")

        def depth_callback(image):
            if self._closed:
                return
            depth = np.asarray(image).copy()
            # GUI changes and camera unprojection happen on the main thread.
            self.app.post_to_main_thread(self.window, lambda: self._apply_depth(
                side, x, y, width, height, depth, serial, view_matrix, projection))

        widget.scene.scene.render_to_depth_image(depth_callback)

    def _apply_depth(self, side, x, y, width, height, depth, serial, view_matrix, projection):
        if self._closed:
            return
        self._pending_depth -= 1
        if self._cancel_requested:
            if self._pending_depth == 0:
                self.window.close()
            return
        if serial != self._request_serial:
            self._refresh()
            return
        self._busy = False
        widget = self.views[side]
        if (widget.frame.width != width or widget.frame.height != height or
            depth.shape != (height, width) or
            not np.allclose(widget.scene.camera.get_view_matrix(), view_matrix) or
            not np.allclose(widget.scene.camera.get_projection_matrix(), projection)):
            self._refresh("View changed during picking; click again")
            return
        hit = nearest_depth_pixel(depth, int(x), int(y), radius=max(4, int(4*self.window.scaling)))
        if hit is None:
            self._refresh("No visible point here; click a tooth surface")
            return
        px, py, value = hit
        world = widget.scene.camera.unproject(px, py, value, width, height)
        count, indices, _ = self.trees[side].search_knn_vector_3d(world, 1)
        if not count:
            self._refresh("No point found")
            return
        index = int(indices[0])
        points = self.selection.camera_points if side == "camera" else self.selection.scan_points
        clip = projection @ view_matrix @ np.r_[points[index], 1.]
        if clip[3] <= 0:
            self._refresh("Point is outside the visible view")
            return
        ndc = clip[:3]/clip[3]
        distance = np.hypot((ndc[0]+1)*width/2-px, (1-ndc[1])*height/2-py)
        if distance > max(8, 8*self.window.scaling):
            self._refresh("Click closer to a visible scan vertex")
            return
        try:
            self.selection.select(side, index)
        except ValueError as exc:
            self._refresh(str(exc))
            return
        self._update_markers()
        self._refresh()

    def _undo(self):
        self._invalidate_request()
        self.selection.undo()
        self._update_markers()
        self._refresh("Undone")

    def _clear(self):
        self._invalidate_request()
        self.selection.clear()
        self._update_markers()
        self._refresh("Cleared")

    def _finish(self):
        if self._pending_depth:
            self._refresh("Wait for the current point pick")
            return
        try:
            self.result_value = self.selection.result()
        except ValueError as exc:
            self._refresh(str(exc))
            return
        self.window.close()

    def _cancel(self):
        self.result_value = None
        self._cancel_requested = True
        if self._pending_depth:
            self._refresh("Cancelling: waiting for the current depth read")
        else:
            self.window.close()

    def _on_close(self):
        if self._pending_depth:
            self.result_value = None
            self._cancel_requested = True
            self._refresh("Cancelling: waiting for the current depth read")
            return False
        self._closed = True
        self._invalidate_request()
        return True

    def _on_key(self, event):
        if event.type != gui.KeyEvent.Type.DOWN:
            return False
        if event.key in (gui.KeyName.Q, gui.KeyName.ENTER):
            self._finish()
        elif event.key == gui.KeyName.ESCAPE:
            self._cancel()
        elif event.key in (gui.KeyName.BACKSPACE, gui.KeyName.DELETE):
            self._undo()
        else:
            return False
        return True


def pick_pairs(scan_cloud, camera_cloud, minimum_pairs=6):
    app = gui.Application.instance
    app.initialize()
    picker = PairedLandmarkWindow(scan_cloud, camera_cloud, minimum_pairs)
    app.run()
    if picker.result_value is None:
        raise ValueError("Landmark selection cancelled; registration was not started")
    return picker.result_value
