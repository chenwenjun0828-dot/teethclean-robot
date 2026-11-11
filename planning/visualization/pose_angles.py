"""Interactive, offline browser preview of the planner's three tool angles."""
from __future__ import annotations

import argparse
from functools import partial
import gzip
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from urllib.parse import parse_qs, urlsplit
import webbrowser

import mujoco
import numpy as np
import open3d as o3d

from perception.regions import PreparedRegion, RegionConfig, SweepOptions, prepare_region
from perception.regions.config import REGIONS
from perception.regions.io import Inputs, body_transform
from pipeline.oral import jaw_overrides, load_jaw_settings
from pipeline.simulation import create_simulation
from pipeline.parameters import add_parameter_file, parse_parameters
from planning.config import Config, resolve_pose_angles
from planning.infrastructure.collision import mesh_from_geom
from planning.infrastructure.poses import TCP_FRAME_ID, unit
from planning.pose_search.constraints import axis_convention, gravity_frame, target_at, tool_rotation
from planning.simple import orient_reference, representative_source

WEB = Path(__file__).with_name('pose_angles_web')


def mesh_record(mesh, limit=24000):
    mesh = o3d.geometry.TriangleMesh(mesh)
    if len(mesh.triangles) > limit:
        mesh = mesh.simplify_quadric_decimation(limit)
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return {'positions': np.round(np.asarray(mesh.vertices), 5).ravel().tolist(),
            'indices': np.asarray(mesh.triangles).ravel().tolist(),
            'normals': np.round(np.asarray(mesh.vertex_normals), 6).ravel().tolist(),
            'colors': (np.round(np.asarray(mesh.vertex_colors), 4).ravel().tolist()
                       if mesh.has_vertex_colors() else None)}


def tool_mesh_tcp(model, data):
    """Use compiled geom placement, including MuJoCo's mesh recentering."""
    geom, site = model.geom('ur_end_scaler_geom').id, model.site('ur_scaler_tcp').id
    vertices, faces = mesh_from_geom(model, geom)
    world = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    vertices_tcp = (world-data.site_xpos[site]) @ data.site_xmat[site].reshape(3, 3)
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices_tcp*1000),
                                   o3d.utility.Vector3iVector(faces))


class PosePreview:
    def __init__(self, *, perception_result=None, region='outer_front', angles=(15., 0., 0.),
                 standoff_mm=1.8, jaw_opening_deg=None, planning_mode='simple'):
        self.angles = resolve_pose_angles(angles)
        self.offset = standoff_mm
        self.mode = planning_mode
        self.jaw = jaw_overrides(load_jaw_settings(), jaw_opening_deg)
        self.model, self.data = create_simulation(jaw_settings=self.jaw)
        self.lock = threading.RLock()
        self.prepared = {}
        self.source = None
        if perception_result is not None:
            prepared = PreparedRegion.load(perception_result)
            self.source = prepared.output_dir
            region = prepared.region
            self.prepared[region] = prepared
            self.available = [name for name in REGIONS if name == region or
                              (self.source.parent/name/'perception_region.json').is_file()]
        else:
            self.available = list(REGIONS)
        self.region = region
        prepared = self.get_region(region)
        self.world = (body_transform(self.model, self.data, prepared.inputs.jaw)
                      if prepared.world_from_scan is None else prepared.world_from_scan)
        anchor = representative_source(prepared.geometry, SweepOptions())
        self.origin = self.world[:3, :3] @ anchor['point_scan_m'] + self.world[:3, 3]
        self.mesh = o3d.io.read_triangle_mesh(str(prepared.inputs.scan))
        if not self.mesh.has_triangles():
            raise ValueError('The selected scan has no triangle mesh')
        self.mesh.vertices = o3d.utility.Vector3dVector(self.display(np.asarray(self.mesh.vertices)*.001))
        self.scene = self.scene_record()
        self.pose(region=region, angles=self.angles, standoff_mm=standoff_mm, mode=planning_mode)

    def get_region(self, name):
        if name not in self.available:
            raise ValueError('Region is not available in this perception result')
        with self.lock:
            if name not in self.prepared:
                if self.source is None:
                    self.prepared[name] = prepare_region(Inputs.defaults('mandibular'), RegionConfig(region=name))
                else:
                    self.prepared[name] = PreparedRegion.load(self.source.parent/name)
                    current = self.prepared[self.region]
                    other = self.prepared[name]
                    if (other.inputs != current.inputs or
                            (other.world_from_scan is None) != (current.world_from_scan is None) or
                            (other.world_from_scan is not None and not np.allclose(
                                other.world_from_scan, current.world_from_scan, atol=1e-10, rtol=0))):
                        del self.prepared[name]
                        raise ValueError('Sibling region does not share the same inputs and registration')
            return self.prepared[name]

    def display(self, points):
        return (np.asarray(points) @ self.world[:3, :3].T + self.world[:3, 3]-self.origin)*1000

    def region_record(self, name):
        prepared = self.get_region(name)
        anchor = representative_source(prepared.geometry, SweepOptions())
        stations = prepared.reference.samples(.00025)
        points = np.array([prepared.reference.evaluate(s) for s in stations])
        index = int(np.argmin(np.linalg.norm(points-anchor['point_scan_m'], axis=1)))
        mesh = o3d.geometry.TriangleMesh(self.mesh)
        faces = np.asarray(prepared.geometry.mesh_faces)
        mesh.triangles = o3d.utility.Vector3iVector(faces[prepared.geometry.face_tooth_ids == anchor['tooth']])
        mesh.remove_unreferenced_vertices()
        return {'region': name, 'length_mm': prepared.reference.end*1000,
                'station_mm': float(stations[index]*1000), 'tooth': anchor['tooth'],
                'path': self.display(points).tolist(), 'tooth_mesh': mesh_record(mesh, 5000)}

    def scene_record(self):
        other_jaw = 'maxillary' if self.get_region(self.region).inputs.jaw == 'mandibular' else 'mandibular'
        other = o3d.io.read_triangle_mesh(str(Inputs.defaults(other_jaw).scan))
        transform = body_transform(self.model, self.data, other_jaw)
        other.vertices = o3d.utility.Vector3dVector(
            np.asarray(other.vertices) @ transform[:3, :3].T + (transform[:3, 3]-self.origin)*1000)
        return {'jaw_mesh': mesh_record(self.mesh), 'opposing_mesh': mesh_record(other),
                'tool_mesh': mesh_record(tool_mesh_tcp(self.model, self.data)),
                'regions': self.available, 'initial': self.region_record(self.region),
                'angles_deg': self.angles, 'standoff_mm': self.offset, 'planning_mode': self.mode,
                'jaw_opening_deg': self.jaw.opening_deg, 'display_origin_world_m': self.origin.tolist(),
                'perception_result': str(self.source) if self.source else None}

    def pose(self, *, region, angles, standoff_mm, mode='simple', station_mm=None, picked=None):
        angles = resolve_pose_angles(angles)
        if not np.isfinite(standoff_mm) or not 0 <= standoff_mm <= 30:
            raise ValueError('Offset must be within 0..30 mm')
        prepared = self.get_region(region)
        cfg = Config(region=region, planning_mode=mode, tilt_deg=angles[0], tilt_azimuth_deg=angles[1],
                     standoff_mm=standoff_mm, clearance_mm=min(.15, standoff_mm/2) if standoff_mm else .15)
        options = SweepOptions(fixed_roll_deg=angles[2])
        reference = prepared.reference
        station = reference.end/2 if station_mm is None else station_mm*.001
        if not np.isfinite(station) or not 0 <= station <= reference.end+1e-12:
            raise ValueError('Station is outside the selected path')
        station = min(station, reference.end)
        point, normal = reference.evaluate(station), reference.normal(station)
        if picked is not None:
            picked = np.asarray(picked, float)
            if picked.shape != (6,) or not np.isfinite(picked).all():
                raise ValueError('Picked surface requires a finite point and normal')
            position, normal_world = picked[:3]*.001+self.origin, unit(picked[3:])
            pose = np.eye(4)
            pose[:3, :3] = tool_rotation(None, normal_world, angles[0], angles[2], angles[1])
            pose[:3, 3] = position + normal_world*standoff_mm*.001
            frame = gravity_frame(-normal_world)
            point = (position-self.world[:3, 3]) @ self.world[:3, :3]
        else:
            if mode == 'simple':
                reference = orient_reference(reference, prepared.geometry, cfg, options, world=self.world)
                normal = reference.anchor_normal
            pose = target_at(reference, station, angles[2], self.world, cfg)
            frame = self.world[:3, :3] @ gravity_frame(-normal, self.world[:3, :3].T @ cfg.gravity_world)
        return {'execution_ready': False, 'preview_only': True, 'region': region,
                'reference': 'picked_surface' if picked is not None else 'perception_path',
                'axis_convention': axis_convention(cfg), 'tcp_frame_id': TCP_FRAME_ID,
                'planning_mode': mode, 'pose_angles_deg': angles, 'standoff_mm': standoff_mm,
                'jaw_opening_deg': self.jaw.opening_deg, 'station_mm': station*1000,
                'point_scan_mm': (point*1000).tolist(), 'world_from_tcp_m': pose.tolist(),
                'position_display_mm': ((pose[:3, 3]-self.origin)*1000).tolist(),
                'rotation_world': pose[:3, :3].tolist(), 'reference_frame_world': frame.tolist()}


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {**SimpleHTTPRequestHandler.extensions_map, '.js': 'text/javascript', '.mjs': 'text/javascript'}

    def __init__(self, *args, preview, **kwargs):
        self.preview = preview
        super().__init__(*args, directory=str(WEB), **kwargs)

    def json_response(self, value, status=200):
        payload = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        if 'gzip' in self.headers.get('Accept-Encoding', ''):
            payload = gzip.compress(payload)
            self.send_header('Content-Encoding', 'gzip')
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        try:
            self.serve_get()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def serve_get(self):
        path = urlsplit(self.path)
        if not path.path.startswith('/api/'):
            return super().do_GET()
        try:
            query = parse_qs(path.query)
            value = lambda key, default: query.get(key, [default])[0]
            if path.path == '/api/scene':
                result = self.preview.scene
            elif path.path == '/api/region':
                result = self.preview.region_record(value('region', self.preview.region))
            elif path.path == '/api/pose':
                result = self.preview.pose(
                    region=value('region', self.preview.region),
                    angles=[float(value(key, default)) for key, default in zip(('angle', 'azimuth', 'flip'), self.preview.angles)],
                    standoff_mm=float(value('offset', self.preview.offset)), mode=value('mode', self.preview.mode),
                    station_mm=float(value('station', self.preview.scene['initial']['station_mm'])),
                    picked=[float(v) for v in query['picked'][0].split(',')] if 'picked' in query else None)
            else:
                return self.json_response({'error': 'Unknown preview endpoint'}, 404)
            self.json_response(result)
        except (ValueError, KeyError, OSError) as error:
            self.json_response({'error': str(error)}, 400)

    def log_message(self, *_args):
        pass


def create_server(preview, port=8765):
    return ThreadingHTTPServer(('127.0.0.1', port), partial(Handler, preview=preview))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_parameter_file(parser)
    parser.add_argument('--perception-result', type=Path, help='PreparedRegion folder; sibling regions are selectable')
    parser.add_argument('--region', choices=REGIONS, default='outer_front')
    parser.add_argument('--pose-angles-deg', type=float, nargs=3, default=(15., 0., 0.))
    parser.add_argument('--standoff-mm', type=float, default=1.8)
    parser.add_argument('--planning-mode', choices=('simple', 'continuous'), default='simple')
    parser.add_argument('--jaw-opening-deg', type=float)
    parser.add_argument('--model-scope', choices=('tooth', 'mandibular', 'both'), default='mandibular')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--browser', dest='no_browser', action='store_false', default=False,
                        help='Override no_browser=true in a parameter file')
    args = parse_parameters(parser, argv)
    try:
        preview = PosePreview(perception_result=args.perception_result, region=args.region,
                              angles=args.pose_angles_deg, standoff_mm=args.standoff_mm,
                              jaw_opening_deg=args.jaw_opening_deg, planning_mode=args.planning_mode)
        try:
            server = create_server(preview, args.port)
        except OSError as error:
            if error.errno not in (48, 98, 10048):
                raise
            server = create_server(preview, 0)
        url = f'http://127.0.0.1:{server.server_port}/?scope={args.model_scope}'
        print(f'Tool angle preview: {url}', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        with server:
            try:
                server.serve_forever()
            except KeyboardInterrupt:
                pass
    except (ValueError, RuntimeError, OSError) as error:
        print(f'Angle preview failed: {error}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
