"""Tooth frames: X crown-to-gingiva, Y lateral tangent, Z outward normal.

Rotations store axis vectors in COLUMNS. T_scan_tooth maps tooth-local mm
to scan mm; camera uses mm and MuJoCo world uses m. Original point order and
positions are retained, including across disconnected gingival chains.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from perception.frames import validate_transform
from perception.gingival_line import ROOT, RegisteredPaths, load_registered_paths


def _points(values, name):
    result = np.asarray(values, dtype=float)
    if (result.ndim != 2 or result.shape[1] != 3 or not len(result)
            or not np.isfinite(result).all()):
        raise ValueError(f'{name} must be finite nonempty Nx3 values')
    return result


def _unit(values, name):
    lengths = np.linalg.norm(values, axis=-1, keepdims=True)
    invalid = ~np.isfinite(lengths[:, 0]) | (lengths[:, 0] < 1e-10)
    if invalid.any():
        raise ValueError(f'{name} is degenerate at point indices {np.flatnonzero(invalid)[:10].tolist()}')
    return values / lengths


def gingiva_direction(jaw):
    """Reference for the current 2026615-gch scans; other orientations need override."""
    if jaw not in ('mandibular', 'maxillary'):
        raise ValueError('Jaw must be mandibular or maxillary')
    return np.array([0., -1. if jaw == 'mandibular' else 1., 0.])


def tooth_frame_axes(normals, gingivaward):
    """Construct right-handed Nx3x3 rotations from outward normals and reference.

gingivaward can be one vector or one per point, in the same frame as normals.
Its tangent projection must be nonzero; ambiguous points raise an error.
Y = Z cross X, rather than a possibly disconnected gingival-line tangent.
"""
    z = _unit(_points(normals, 'Outward normals'), 'Outward normal')
    direction = np.asarray(gingivaward, float)
    if direction.shape == (3,):
        direction = np.broadcast_to(direction, z.shape)
    if direction.shape != z.shape or not np.isfinite(direction).all():
        raise ValueError('Gingivaward reference must be finite shape (3,) or (N,3)')
    g = _unit(direction, 'Gingivaward reference')
    x = _unit(g - np.sum(z * g, axis=1)[:, None] * z, 'Gingivaward surface tangent')
    y = _unit(np.cross(z, x), 'Lateral surface tangent')
    return np.stack((x, y, np.cross(x, y)), axis=2)


@dataclass
class ToothFrames:
    origins: np.ndarray
    rotations: np.ndarray
    coordinate_frame: str = 'oral_scan'
    units: str = 'mm'

    def __post_init__(self):
        self.origins = _points(self.origins, 'Frame origins').copy()
        self.rotations = np.asarray(self.rotations, float).copy()
        if self.units not in ('mm', 'm'):
            raise ValueError('Frame units must be mm or m')
        if self.rotations.shape != (len(self.origins), 3, 3) or not np.isfinite(self.rotations).all():
            raise ValueError('Rotations must be finite Nx3x3 matrices')
        gram = self.rotations.transpose(0, 2, 1) @ self.rotations
        if (not np.allclose(gram, np.eye(3), atol=1e-6, rtol=0)
                or not np.allclose(np.linalg.det(self.rotations), 1., atol=1e-6, rtol=0)):
            raise ValueError('Frame rotations must be right-handed and orthonormal')

    @property
    def transforms(self):
        result = np.tile(np.eye(4), (len(self.origins), 1, 1))
        result[:, :3, :3] = self.rotations
        result[:, :3, 3] = self.origins
        return result

    def transformed(self, target_from_source, coordinate_frame, units):
        """Rigid conversion; transform translation must use TARGET units.

        Both the origins and tooth-local translation units change to `units`.
        Axes are rotated without scaling. No input array is modified.
        """
        transform = validate_transform(target_from_source)
        if units not in ('mm', 'm'):
            raise ValueError('Target units must be mm or m')
        scale = {'mm': 0.001, 'm': 1.}[self.units] / {'mm': 0.001, 'm': 1.}[units]
        origins = self.origins * scale @ transform[:3, :3].T + transform[:3, 3]
        return ToothFrames(origins, transform[:3, :3] @ self.rotations, coordinate_frame, units)


def define_tooth_frames(points_mm, outward_normals, jaw, gingivaward=None):
    """Define frames from already estimated normals, preserving scan point indices."""
    default_direction = gingiva_direction(jaw)
    return ToothFrames(points_mm, tooth_frame_axes(
        outward_normals, default_direction if gingivaward is None else gingivaward))


def estimate_surface_normals(mesh, points_mm, tooth_mask, normal_orientation_sign,
                             max_surface_distance_mm=2.):
    """Interpolate vertex normals on the closest tooth-side mesh triangles.

    Triangles require at least two tooth-labelled vertices. Normal interpolation
    uses only this tooth-side submesh. A supplied global winding sign must be
    +1/-1 (estimated from a complete arch by build_tooth_frames).
    No inferred radial normal is substituted when surface support is missing.
    """
    import open3d as o3d

    points = _points(points_mm, 'Scan points')
    vertices = _points(np.asarray(mesh.vertices), 'Mesh vertices')
    faces = np.asarray(mesh.triangles)
    mask = np.asarray(tooth_mask)
    if mask.shape != (len(vertices),) or mask.dtype != np.bool_:
        raise ValueError('Tooth mask must be one boolean per mesh vertex')
    if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces):
        raise ValueError('Surface normal estimation requires mesh triangles')
    if normal_orientation_sign not in (-1., 1.):
        raise ValueError('Normal orientation sign must be +1 or -1')
    if not np.isfinite(max_surface_distance_mm) or max_surface_distance_mm <= 0:
        raise ValueError('Maximum surface distance must be positive and finite')
    selected = faces[mask[faces].sum(axis=1) >= 2]
    if not len(selected):
        raise ValueError('No tooth-side triangles available')
    areas = np.linalg.norm(np.cross(vertices[selected[:, 1]] - vertices[selected[:, 0]],
                                    vertices[selected[:, 2]] - vertices[selected[:, 0]]), axis=1)
    selected = selected[areas > 1e-10]
    if not len(selected):
        raise ValueError('No nondegenerate tooth-side triangles available')
    surface = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(selected))
    surface.compute_vertex_normals()
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(surface))
    closest = scene.compute_closest_points(o3d.core.Tensor(points.astype(np.float32)))
    distances = np.linalg.norm(closest['points'].numpy() - points, axis=1)
    invalid = ~np.isfinite(distances) | (distances > max_surface_distance_mm)
    if invalid.any():
        raise ValueError(f'Points too far from tooth surface at indices {np.flatnonzero(invalid)[:10].tolist()}')
    uv = closest['primitive_uvs'].numpy()
    weights = np.column_stack((1. - uv.sum(axis=1), uv))
    triangles = selected[closest['primitive_ids'].numpy().astype(np.int64)]
    normals = np.sum(np.asarray(surface.vertex_normals)[triangles] * weights[:, :, None], axis=1)
    return _unit(normals * normal_orientation_sign, 'Interpolated surface normal'), distances


def _tooth_mask(mesh, labelled_mesh_path):
    import open3d as o3d

    if labelled_mesh_path is not None:
        labelled = o3d.io.read_triangle_mesh(str(labelled_mesh_path))
        if (np.asarray(labelled.vertices).shape != np.asarray(mesh.vertices).shape
                # Existing extraction writes ASCII PLY at six significant digits.
                or not np.allclose(labelled.vertices, mesh.vertices, atol=1e-4, rtol=0)
                or not np.array_equal(labelled.triangles, mesh.triangles)):
            raise ValueError('Label mesh geometry does not match the oral scan')
        colors = np.asarray(labelled.vertex_colors)
        if colors.shape != np.asarray(mesh.vertices).shape:
            raise ValueError('Label mesh requires per-vertex colors')
        tooth_color, gum_color = np.array([.95, .95, .90]), np.array([.85, .18, .22])
        tooth_distance = np.linalg.norm(colors - tooth_color, axis=1)
        gum_distance = np.linalg.norm(colors - gum_color, axis=1)
        if np.any(np.minimum(tooth_distance, gum_distance) > .02):
            raise ValueError('Expected tooth/gum label colors from current gingival extraction')
        return tooth_distance < gum_distance
    from perception.gingival_extraction.extract import classify_gum_by_color, smooth_scores, vertex_neighbors
    from perception.gingival_extraction.cleanup import clean_color_regions

    if not mesh.has_vertex_colors():
        raise ValueError('Colored scan or an extraction label mesh is required')
    scores, _, _ = classify_gum_by_color(np.asarray(mesh.vertex_colors), 5)
    scores = smooth_scores(scores, vertex_neighbors(np.asarray(mesh.triangles), len(scores)), 4, .55)
    scores = (scores - scores.min()) / (np.ptp(scores) + 1e-8)
    scores, _ = clean_color_regions(mesh, scores, .5)
    return scores < .5


def build_tooth_frames(mesh_path, points_mm, jaw, *, labelled_mesh_path=None,
                       gingivaward=None, max_surface_distance_mm=2.):
    """Build scan frames from a complete oral scan and current extraction labels.

    Returns (frames, distances_mm, report). Uses the existing arch estimator to
    determine outward winding, including globally reversed mesh winding.
    """
    import open3d as o3d
    from perception.gingival_extraction.outer_surface import filter_outer_segments

    gingiva_direction(jaw)
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    if not mesh.has_vertices() or not mesh.has_triangles():
        raise ValueError(f'Cannot read oral scan triangle mesh: {mesh_path}')
    mask = _tooth_mask(mesh, labelled_mesh_path)
    _, report = filter_outer_segments(mesh, (~mask).astype(float), .5, [])
    normals, distances = estimate_surface_normals(
        mesh, points_mm, mask, report['normal_orientation_sign'], max_surface_distance_mm)
    frames = define_tooth_frames(points_mm, normals, jaw, gingivaward)
    report.update(jaw=jaw, normal_method='closest_tooth_triangle_barycentric_vertex_normals',
                  max_surface_distance_mm=float(max_surface_distance_mm),
                  measured_max_surface_distance_mm=float(distances.max()),
                  gingivaward_reference_scan=(gingiva_direction(jaw) if gingivaward is None
                                              else np.asarray(gingivaward)).tolist())
    return frames, distances, report


@dataclass
class RegisteredToothFrames:
    scan: ToothFrames
    camera: ToothFrames
    world: ToothFrames
    surface_distances_mm: np.ndarray
    report: dict


def registered_tooth_frames(paths: RegisteredPaths, mesh_path, *, labelled_mesh_path=None,
                            gingivaward=None, max_surface_distance_mm=2.):
    """Use the same frozen gingival points and estimated registration as planning."""
    scan, distances, report = build_tooth_frames(
        mesh_path, paths.scan_points_mm, paths.jaw, labelled_mesh_path=labelled_mesh_path,
        gingivaward=gingivaward, max_surface_distance_mm=max_surface_distance_mm)
    world_from_scan = validate_transform(paths.world_from_scan)
    world_from_camera = validate_transform(paths.world_from_camera)
    camera_from_scan_mm = np.linalg.inv(world_from_camera) @ world_from_scan
    camera_from_scan_mm[:3, 3] *= 1000.
    camera = scan.transformed(camera_from_scan_mm, 'camera_cv', 'mm')
    world = scan.transformed(world_from_scan, 'world', 'm')
    if not np.allclose(world.origins, paths.world_points, rtol=0, atol=1e-9):
        raise ValueError('Registered world points disagree with scan-to-world transform')
    report.update(registration_dir=str(paths.registration_dir), topology_path=str(paths.topology_path))
    return RegisteredToothFrames(scan, camera, world, distances, report)


def export_tooth_frames(frames, output_dir, point_index=250):
    """Export original-order matrices, XYZ+axis CSVs and an explicit convention report."""
    scan = frames.scan if isinstance(frames, RegisteredToothFrames) else frames
    if not 0 <= point_index < len(scan.origins):
        raise ValueError('Preview point index is outside the original point array')
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    entries = [('scan', scan)]
    report = {}
    if isinstance(frames, RegisteredToothFrames):
        entries.extend([('camera', frames.camera), ('world', frames.world)])
        report.update(frames.report)
        np.save(output_dir / 'surface_distances_mm.npy', frames.surface_distances_mm)
    for name, value in entries:
        np.save(output_dir / f'T_{name}_tooth.npy', value.transforms)
        axes = value.rotations.transpose(0, 2, 1).reshape(len(value.origins), 9)
        np.savetxt(output_dir / f'tooth_frames_{name}.csv',
                   np.column_stack((np.arange(len(value.origins)), value.origins, axes)), delimiter=',',
                   header='point_index,x,y,z,Xx,Xy,Xz,Yx,Yy,Yz,Zx,Zy,Zz', comments='')
    report.update(point_count=len(scan.origins), point_index=int(point_index), index_base=0,
                  axis_definition={'X': 'crown_to_gingiva_tangent', 'Y': 'Z_cross_X_lateral_tangent',
                                   'Z': 'outward_tooth_surface_normal'},
                  matrix_convention='T_target_tooth maps tooth-local coordinates to target; axes are columns',
                  frames={name: {'coordinate_frame': value.coordinate_frame, 'units': value.units,
                                 'selected_transform': value.transforms[point_index].tolist()}
                          for name, value in entries})
    (output_dir / 'tooth_frames_report.json').write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    return output_dir


def _tooth_view_geometry(mesh, frames, point_index, radius_mm=10., local=True, context_points_mm=None):
    """Create actual scan-space geometry; clipping affects the display only."""
    import open3d as o3d

    if frames.coordinate_frame != 'oral_scan' or frames.units != 'mm':
        raise ValueError('Visualization requires oral_scan frames in mm')
    if not 0 <= point_index < len(frames.origins):
        raise ValueError('Point index is outside the original point array')
    if not np.isfinite(radius_mm) or radius_mm <= 0:
        raise ValueError('View radius must be positive and finite')
    origin, rotation = frames.origins[point_index], frames.rotations[point_index]
    surface = o3d.geometry.TriangleMesh(mesh)
    if local:
        vertices = np.asarray(surface.vertices)
        keep = np.linalg.norm(vertices - origin, axis=1) <= radius_mm
        surface.remove_triangles_by_mask(~keep[np.asarray(surface.triangles)].all(axis=1))
        surface.remove_unreferenced_vertices()
    if not surface.has_triangles():
        raise ValueError('No mesh triangles within view radius')
    surface.compute_vertex_normals()
    # The original scan colors remain unchanged. These are display-only colors.
    if surface.has_vertex_colors():
        colors = np.asarray(surface.vertex_colors)
        gum = colors[:, 0] - (colors[:, 1] + colors[:, 2]) * .5 > .12
        surface.vertex_colors = o3d.utility.Vector3dVector(
            np.where(gum[:, None], [.78, .36, .43], [.91, .88, .76]))
    else:
        surface.paint_uniform_color([.91, .88, .76])
    points = frames.origins if context_points_mm is None else _points(context_points_mm, 'Context points')
    if local:
        points = points[np.linalg.norm(points - origin, axis=1) <= radius_mm]
    markers = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    markers.paint_uniform_color([0., .85, .85])
    origin_ball = o3d.geometry.TriangleMesh.create_sphere(radius=.22)
    origin_ball.translate(origin)
    origin_ball.compute_vertex_normals()
    origin_ball.paint_uniform_color([.04, .07, .12])
    result = [('surface', surface, 'mesh'), ('gingival_points', markers, 'points'),
              ('origin', origin_ball, 'unlit')]
    axis_length = radius_mm * .55
    for i, color in enumerate(((.95, .62, .02), (.03, .45, 1.), (1., .12, .12))):
        arrow = o3d.geometry.TriangleMesh.create_arrow(
            cylinder_radius=.10, cone_radius=.28, cylinder_height=axis_length * .78,
            cone_height=axis_length * .22, resolution=32)
        # Open3D arrows start at the origin and point along their local +Z.
        direction = rotation[:, i]
        helper = np.eye(3)[np.argmin(np.abs(direction))]
        first = np.cross(helper, direction)
        first /= np.linalg.norm(first)
        arrow.rotate(np.column_stack((first, np.cross(direction, first), direction)), center=[0, 0, 0])
        arrow.translate(origin)
        arrow.compute_vertex_normals()
        arrow.paint_uniform_color(color)
        result.append(('axis_' + 'XYZ'[i], arrow, 'unlit'))
    label_points = origin + axis_length * 1.16 * rotation.T
    return result, label_points


def _populate_tooth_scene(scene, geometry):
    from open3d.visualization import rendering

    scene.clear_geometry()
    scene.set_background([.96, .97, .99, 1.])
    scene.set_lighting(rendering.Open3DScene.LightingProfile.NO_SHADOWS, [0., -.5, -.8])
    for name, item, kind in geometry:
        material = rendering.MaterialRecord()
        material.shader = 'defaultLit' if kind == 'mesh' else 'defaultUnlit'
        material.base_color = [1., 1., 1., 1.]
        material.base_roughness = .8
        if kind == 'points':
            material.point_size = 4.
        scene.add_geometry(name, item, material)


def _tooth_view_camera(frames, point_index, radius_mm):
    origin, rotation = frames.origins[point_index], frames.rotations[point_index]
    # View from the outward-normal and crown (-X) side, so gingiva does not
    # hide the selected margin behind its convex surface.
    direction = np.array([-1.2, -1.6, 2.7])
    direction /= np.linalg.norm(direction)
    front = rotation @ direction
    up = rotation[:, 2] - np.dot(rotation[:, 2], front) * front
    up /= np.linalg.norm(up)
    return origin, origin + front * radius_mm * 3.5, up


def preview_tooth_frame(mesh_path, frames, output_path, point_index=250, radius_mm=10.,
                        point_label=None, context_points_mm=None):
    """Save a shaded, depth-tested local view and an unobstructed axis inset.

    Uses the same mesh, origin and arrows as the interactive viewer. The inset
    intentionally shows axes alone; the main view preserves actual occlusion.
    """
    import open3d as o3d
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    geometry, labels = _tooth_view_geometry(mesh, frames, point_index, radius_mm,
                                           context_points_mm=context_points_mm)
    selected_label = point_label or f'Point {point_index}'
    # The installed Windows build has no EGL OffscreenRenderer. A hidden
    # native OpenGL window provides the same depth-tested raster without EGL.
    viewer = o3d.visualization.Visualizer()
    if not viewer.create_window(window_name='Tooth frame preview', width=1200, height=900, visible=False):
        raise RuntimeError('Could not create a hidden OpenGL window for the preview')
    center, eye, up = _tooth_view_camera(frames, point_index, radius_mm)
    try:
        for _, item, _ in geometry:
            viewer.add_geometry(item)
        options = viewer.get_render_option()
        options.background_color = np.array([.96, .97, .99])
        options.mesh_show_back_face = True
        options.point_size = 4.
        control = viewer.get_view_control()
        control.set_lookat(center)
        control.set_front((eye - center) / np.linalg.norm(eye - center))
        control.set_up(up)
        control.set_zoom(.72)
        viewer.poll_events()
        viewer.update_renderer()
        raster = np.clip(np.asarray(viewer.capture_screen_float_buffer(do_render=True)) * 255,
                         0, 255).astype(np.uint8)
        camera = control.convert_to_pinhole_camera_parameters()
        projection = camera.intrinsic.intrinsic_matrix @ camera.extrinsic[:3]
    finally:
        viewer.destroy_window()

    def pixels(point):
        clip = projection @ np.r_[point, 1.]
        return clip[:2] / clip[2]

    projected_labels = np.array([pixels(p) for p in labels])
    projected_origin = pixels(center)
    fig = Figure(figsize=(12, 8), facecolor='#f5f7fb')
    FigureCanvasAgg(fig)
    ax = fig.add_axes([.02, .10, .74, .83])
    ax.imshow(raster)
    ax.axis('off')
    colors = ('#c67e00', '#0876dd', '#e62b2b')
    for label, pixel, color in zip(('+X', '+Y', '+Z'), projected_labels, colors):
        ax.text(*pixel, label, color=color, weight='bold', fontsize=16,
                bbox={'facecolor': 'white', 'alpha': .85, 'edgecolor': 'none', 'pad': 2})
    ax.annotate(selected_label, projected_origin, xytext=(-75, 40),
                textcoords='offset points', fontsize=12,
                arrowprops={'arrowstyle': '-', 'color': '#172639'},
                bbox={'facecolor': 'white', 'alpha': .9, 'edgecolor': 'none'})
    # Reuse the real camera projection for an axis-only inset.
    inset = fig.add_axes([.77, .48, .21, .32])
    inset.set_aspect('equal')
    for pixel, color, label in zip(projected_labels, colors, ('+X', '+Y', '+Z')):
        delta = pixel - projected_origin
        inset.annotate('', (delta[0], -delta[1]), (0, 0),
                       arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 3})
        inset.text(delta[0] * 1.13, -delta[1] * 1.13, label, color=color, fontsize=14, weight='bold')
    extent = np.max(np.abs(projected_labels - projected_origin)) * 1.5
    inset.set(xlim=(-extent, extent), ylim=(-extent, extent), title='Axes only / same view')
    inset.axis('off')
    fig.text(.78, .40, '+X  Crown to gingiva\n\n+Y  Lateral tangent\n\n+Z  Outward normal', fontsize=12)
    fig.text(.05, .94, f'Tooth surface frame | {selected_label}', fontsize=20, weight='bold')
    fig.text(.05, .035, 'Cyan: original gingival points    Black: selected origin    Local scan geometry [mm]', fontsize=11)
    fig.savefig(output_path, dpi=160, facecolor=fig.get_facecolor())


def show_tooth_frames(mesh_path, frames, point_index=250, radius_mm=10.,
                      point_label=None, context_points_mm=None, surface_selector=None,
                      vertex_index=None):
    """Interactive shaded tooth frames; rotate/zoom, select indices, local/full arch."""
    import open3d as o3d
    from open3d.visualization import gui, rendering

    if surface_selector is not None:
        frames, _ = surface_selector.frame(vertex_index)
        point_index = 0
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    # Validate before initializing a native window.
    _tooth_view_geometry(mesh, frames, point_index, radius_mm, context_points_mm=context_points_mm)
    app = gui.Application.instance
    app.initialize()
    window = app.create_window('Tooth Frames | X crown-to-gingiva, Y lateral, Z outward', 1440, 950)
    widget = gui.SceneWidget()
    widget.scene = rendering.Open3DScene(window.renderer)
    window.add_child(widget)
    panel = gui.Vert(10, gui.Margins(12, 12, 12, 12))
    panel.add_child(gui.Label('Mesh vertex index (0-based)' if surface_selector is not None
                             else point_label or 'Original gingival point index (0-based)'))
    index = gui.NumberEdit(gui.NumberEdit.INT)
    maximum = len(surface_selector.vertices) - 1 if surface_selector is not None else len(frames.origins) - 1
    index.set_limits(0, maximum)
    index.int_value = vertex_index if surface_selector is not None else point_index
    index.enabled = surface_selector is not None or len(frames.origins) > 1
    panel.add_child(index)
    status = gui.Label('')
    local = gui.Checkbox('Local tooth surface')
    local.checked = True
    panel.add_child(local)
    panel.add_child(gui.Label('Gold +X: crown to gingiva\nBlue +Y: lateral tangent\nRed +Z: outward normal\nCyan: gingival points'))
    panel.add_child(gui.Label('Drag: rotate | Wheel: zoom\nCtrl + drag: pan'))
    panel.add_child(status)
    window.add_child(panel)
    labels = []

    def refresh():
        nonlocal labels
        selected = index.int_value
        try:
            active_frames = frames
            frame_index = selected
            label_text = point_label or f'Point {selected}'
            if surface_selector is not None:
                active_frames, _ = surface_selector.frame(selected)
                frame_index = 0
                label_text = f'Tooth vertex {selected}'
            items, endpoints = _tooth_view_geometry(mesh, active_frames, frame_index, radius_mm,
                                                   local.checked, context_points_mm=context_points_mm)
        except ValueError as error:
            status.text = str(error)
            return
        for label in labels:
            widget.remove_3d_label(label)
        labels = []
        _populate_tooth_scene(widget.scene, items)
        for location, text, color in zip(endpoints, ('+X', '+Y', '+Z'),
                                         ((.8, .5, 0.), (0., .35, .9), (.9, .1, .1))):
            label = widget.add_3d_label(location.astype(np.float32), text)
            label.color = gui.Color(*color)
            label.scale = 1.5
            labels.append(label)
        labels.append(widget.add_3d_label(active_frames.origins[frame_index].astype(np.float32), label_text))
        bounds = items[0][1].get_axis_aligned_bounding_box()
        widget.setup_camera(35., bounds, bounds.get_center())
        span = radius_mm if local.checked else np.linalg.norm(bounds.get_extent()) * .55
        center, eye, up = _tooth_view_camera(active_frames, frame_index, span)
        if not local.checked:
            offset = bounds.get_center() - center
            center, eye = center + offset, eye + offset
        widget.look_at(center.astype(np.float32), eye.astype(np.float32), up.astype(np.float32))
        status.text = f'{label_text} / {maximum}\nOrigin [mm]:\n' + '\n'.join(
            f'{v:.3f}' for v in active_frames.origins[frame_index])

    def step(delta):
        if surface_selector is not None:
            valid = surface_selector.tooth_indices
            insertion = np.searchsorted(valid, index.int_value, side='right' if delta > 0 else 'left')
            index.int_value = int(valid[np.clip(insertion if delta > 0 else insertion - 1, 0, len(valid) - 1)])
        else:
            index.int_value = int(np.clip(index.int_value + delta, 0, maximum))
        refresh()

    buttons = gui.Horiz(8)
    for text, action in (('Previous', lambda: step(-1)), ('Next', lambda: step(1)), ('Reset view', refresh)):
        button = gui.Button(text)
        button.enabled = text == 'Reset view' or surface_selector is not None or len(frames.origins) > 1
        button.set_on_clicked(action)
        buttons.add_child(button)
    panel.add_child(buttons)
    index.set_on_value_changed(lambda _: refresh())
    local.set_on_checked(lambda _: refresh())

    def layout(_):
        rect = window.content_rect
        sidebar = 320
        if rect.width > sidebar + 100 and rect.height > 100:
            widget.frame = gui.Rect(rect.x, rect.y, rect.width - sidebar, rect.height)
            panel.frame = gui.Rect(rect.get_right() - sidebar, rect.y, sidebar, rect.height)

    window.set_on_layout(layout)
    refresh()
    app.run()


class ToothSurfacePointSelector:
    """Select exact original mesh vertex IDs; cache labels and outward orientation."""

    def __init__(self, mesh_path, jaw, *, labelled_mesh_path=None, gingivaward=None):
        import open3d as o3d
        from perception.gingival_extraction.outer_surface import filter_outer_segments

        self.jaw = jaw
        self.gingivaward = gingiva_direction(jaw) if gingivaward is None else gingivaward
        self.mesh = o3d.io.read_triangle_mesh(str(mesh_path))
        if not self.mesh.has_vertices() or not self.mesh.has_triangles():
            raise ValueError('Surface selection requires an oral scan triangle mesh')
        self.vertices = np.asarray(self.mesh.vertices)
        self.tooth_mask = _tooth_mask(self.mesh, labelled_mesh_path)
        self.tooth_indices = np.flatnonzero(self.tooth_mask)
        _, self.report = filter_outer_segments(self.mesh, (~self.tooth_mask).astype(float), .5, [])

    def frame(self, vertex_index):
        if (not isinstance(vertex_index, (int, np.integer)) or isinstance(vertex_index, (bool, np.bool_))
                or not 0 <= vertex_index < len(self.vertices)):
            raise ValueError(f'Mesh vertex index must be an integer from 0 to {len(self.vertices)-1}')
        if not self.tooth_mask[vertex_index]:
            raise ValueError(f'Vertex {vertex_index} is labelled gingiva. Choose a tooth vertex or use Next/Previous.')
        points = self.vertices[[vertex_index]]
        normals, distances = estimate_surface_normals(
            self.mesh, points, self.tooth_mask, self.report['normal_orientation_sign'])
        sample = define_tooth_frames(points, normals, self.jaw, self.gingivaward)
        report = dict(self.report, jaw=self.jaw, point_type='selected_tooth_surface_vertex',
                      mesh_vertex_index=int(vertex_index), source_frame='oral_scan', units='mm',
                      transform_name='T_scan_tooth', origin_scan_mm=points[0].tolist(),
                      surface_distance_mm=float(distances[0]),
                      gingivaward_reference_scan=np.asarray(self.gingivaward).tolist())
        return sample, report


def tooth_surface_example(mesh_path, reference_frames, jaw, point_index=250, *,
                          labelled_mesh_path=None, crownward_distance_mm=5.):
    """Choose a labelled tooth vertex crownward of a gingival anchor for visualization.

    Returns a separate one-point frame and selection report. The original
    gingival point and frame arrays remain untouched.
    """
    import open3d as o3d

    if reference_frames.coordinate_frame != 'oral_scan' or reference_frames.units != 'mm':
        raise ValueError('Surface example requires oral_scan frames in mm')
    if not 0 <= point_index < len(reference_frames.origins):
        raise ValueError('Anchor index is outside the gingival point array')
    if not np.isfinite(crownward_distance_mm) or crownward_distance_mm <= 0:
        raise ValueError('Crownward distance must be positive and finite')
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))
    vertices = np.asarray(mesh.vertices)
    mask = _tooth_mask(mesh, labelled_mesh_path)
    candidates = np.flatnonzero(mask)
    if not len(candidates):
        raise ValueError('No tooth-labelled vertices for the surface example')
    anchor = reference_frames.origins[point_index]
    target = anchor - crownward_distance_mm * reference_frames.rotations[point_index, :, 0]
    vertex_index = int(candidates[np.argmin(np.linalg.norm(vertices[mask] - target, axis=1))])
    sample, _, report = build_tooth_frames(mesh_path, vertices[[vertex_index]], jaw,
                                          labelled_mesh_path=labelled_mesh_path)
    report.update(point_type='tooth_surface_example', mesh_vertex_index=vertex_index,
                  gingival_anchor_index=int(point_index), gingival_anchor_scan_mm=anchor.tolist(),
                  requested_crownward_distance_mm=float(crownward_distance_mm),
                  distance_from_anchor_mm=float(np.linalg.norm(sample.origins[0] - anchor)))
    return sample, report


def main(argv=None):
    import argparse
    from perception.gingival_line import read_xyz

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jaw', required=True, choices=('mandibular', 'maxillary'))
    parser.add_argument('--scan', type=Path, help='Oral scan mesh in mm; defaults to registration config')
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--line', type=Path, help='Scan-frame gingival x,y,z CSV in mm')
    source.add_argument('--registration-dir', type=Path)
    parser.add_argument('--topology', type=Path, help='OBJ matching the registered line')
    parser.add_argument('--labels', type=Path, help='Tooth/gum label mesh from current extraction')
    parser.add_argument('--gingiva-direction', type=float, nargs=3, metavar=('GX', 'GY', 'GZ'))
    parser.add_argument('--max-distance-mm', type=float, default=2.)
    parser.add_argument('--point-index', type=int, default=250, help='Zero-based original CSV row index')
    parser.add_argument('--view', action='store_true', help='Open interactive tooth-frame viewer after exporting')
    parser.add_argument('--view-radius-mm', type=float, default=10., help='Local preview/viewer crop radius')
    parser.add_argument('--tooth-surface-demo', action='store_true',
                        help='Also visualize a real tooth vertex crownward of the selected gingival point')
    parser.add_argument('--vertex-index', type=int,
                        help='Select a tooth surface point by original zero-based oral scan mesh vertex ID')
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.vertex_index is not None and args.tooth_surface_demo:
        parser.error('Use either --vertex-index or --tooth-surface-demo')
    mesh_path = args.scan
    if args.registration_dir:
        registration_report = json.loads((args.registration_dir / 'registration_report.json').read_text(encoding='utf-8-sig'))
        if mesh_path is None:
            mesh_path = Path(registration_report['config'][args.jaw + '_mesh'])
            if not mesh_path.is_absolute():
                mesh_path = ROOT / mesh_path
        # Use topology alongside the frozen source line, rather than a different extraction.
        topology = args.topology
        if topology is None:
            line = Path(registration_report['config'][args.jaw + '_line'])
            if not line.is_absolute():
                line = ROOT / line
            topology = line.parent / 'gingival_line_segments.obj'
        paths = load_registered_paths(args.registration_dir, args.jaw, topology)
        import hashlib
        if hashlib.sha256(mesh_path.read_bytes()).hexdigest() != registration_report['source_hashes'][args.jaw]:
            raise ValueError('Oral scan hash differs from the registered source mesh')
        result = registered_tooth_frames(paths, mesh_path, labelled_mesh_path=args.labels,
                    gingivaward=args.gingiva_direction, max_surface_distance_mm=args.max_distance_mm)
        scan_frames = result.scan
    else:
        if mesh_path is None:
            parser.error('--scan is required with --line')
        scan_frames, distances, report = build_tooth_frames(mesh_path, read_xyz(args.line), args.jaw,
                    labelled_mesh_path=args.labels, gingivaward=args.gingiva_direction,
                    max_surface_distance_mm=args.max_distance_mm)
        result = scan_frames
    export_tooth_frames(result, args.out, args.point_index)
    if not args.registration_dir:
        np.save(args.out / 'surface_distances_mm.npy', distances)
        report_path = args.out / 'tooth_frames_report.json'
        report.update(json.loads(report_path.read_text(encoding='utf-8')))
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    preview_tooth_frame(mesh_path, scan_frames, args.out / 'tooth_frame_preview.png', args.point_index,
                        args.view_radius_mm)
    print(f'Exported {len(scan_frames.origins)} tooth frames to {args.out.resolve()}')
    if args.tooth_surface_demo or args.vertex_index is not None:
        selector = ToothSurfacePointSelector(mesh_path, args.jaw, labelled_mesh_path=args.labels,
                                             gingivaward=args.gingiva_direction)
        if args.vertex_index is not None:
            example, selection = selector.frame(args.vertex_index)
        else:
            _, selection = tooth_surface_example(mesh_path, scan_frames, args.jaw, args.point_index,
                                                  labelled_mesh_path=args.labels)
            example, exact_selection = selector.frame(selection['mesh_vertex_index'])
            selection.update(exact_selection)
        label = f"Tooth vertex {selection['mesh_vertex_index']}"
        preview_tooth_frame(mesh_path, example, args.out / 'tooth_surface_preview.png', 0,
                            min(args.view_radius_mm, 6.), label, scan_frames.origins)
        np.save(args.out / 'T_scan_tooth_surface_demo.npy', example.transforms)
        (args.out / 'tooth_surface_selection.json').write_text(
            json.dumps(selection, indent=2, allow_nan=False), encoding='utf-8')
        if args.view:
            show_tooth_frames(mesh_path, example, 0, min(args.view_radius_mm, 6.),
                              label, scan_frames.origins, selector, selection['mesh_vertex_index'])
    elif args.view:
        show_tooth_frames(mesh_path, scan_frames, args.point_index, args.view_radius_mm)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
