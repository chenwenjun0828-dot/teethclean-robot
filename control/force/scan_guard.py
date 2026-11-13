"""Online complete original-mesh guard, independent of MuJoCo contact proxies."""
import json
from pathlib import Path

import fcl
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .local_surface import scene


class OriginalScanGuard:
    def __init__(self, checker, source, model, robot, manifest):
        self.checker, self.model, self.robot = checker, model, robot
        self.info = json.loads(Path(manifest).read_text())
        self.directory = Path(manifest).parent
        self.labels = np.load(Path(source)/'tooth_face_ids.npy', allow_pickle=False)
        self.allowed = set(self.info['selected_teeth'])
        self.jaw = self.info['jaw']
        self.parts = {}
        for i,p in enumerate(self.info['parts']):
            name = f'ur_force_tool_{i:04d}' if p['role']=='tool' else f'force_{self.jaw}_tooth_{p["tooth"]:02d}_{i:04d}'
            self.parts[name] = p
        self.scenes = {}; self.cache = {}
        with np.load(self.directory/'source_geometry.npz',allow_pickle=False) as z:
            self.band = cKDTree(z['work_points'])
            self.original = scene(z['vertices'],z['faces'])
            self.covered = set(z['band_ids'].tolist())

    def assess(self, data, phase):
        q = data.qpos[self.robot.qadr]
        key = q.tobytes()
        if key not in self.cache:
            if len(self.cache)>8: self.cache.clear()
            self.cache[key] = self._original(q)
        original = self.cache[key]
        if not original['safe']: return original
        tip = data.site_xpos[self.robot.site]
        body = self.model.body('dental_'+self.jaw).id
        local_tip = (tip-data.xpos[body])@data.xmat[body].reshape(3,3)*1000
        if phase in ('acquire','trace','release') and self.band.query(local_tip)[0] > self.info['band_radius_mm']-1.:
            return dict(safe=False,reason='TCP left verified local work-band coverage',tcp_scan_mm=local_tip)
        for contact in data.contact:
            names = [self.model.geom(int(g)).name for g in contact.geom]
            if not any(n in self.parts and self.parts[n]['role']=='tool' for n in names): continue
            if contact.dist < -.0001:
                return dict(safe=False,reason='Local contact penetration exceeds 0.1 mm',geometries=names,
                            contact_world_m=contact.pos.copy(),penetration_mm=float(-contact.dist*1000))
            for name in names:
                if name not in self.parts: continue
                part=self.parts[name]; bid=self.model.body(part['body']).id
                point=((contact.pos-data.xpos[bid])@data.xmat[bid].reshape(3,3)*1000).astype(np.float32)
                if name not in self.scenes:
                    mesh=o3d.io.read_triangle_mesh(str(self.directory/part['file']))
                    self.scenes[name]=scene(np.asarray(mesh.vertices),np.asarray(mesh.triangles))
                result=self.scenes[name].compute_closest_points(o3d.core.Tensor(point[None]))
                face=int(result['primitive_ids'].numpy()[0])
                if part['face_kinds'][face]!='working':
                    return dict(safe=False,reason='Contact on internal extrusion '+part['face_kinds'][face],
                        geometry=name,facet=face,contact_world_m=contact.pos.copy())
                if part['role']=='tooth':
                    nearest=self.original.compute_closest_points(o3d.core.Tensor(point[None]))
                    fid=int(nearest['primitive_ids'].numpy()[0])
                    gap=np.linalg.norm(nearest['points'].numpy()[0]-point)
                    if fid not in self.covered or int(self.labels[fid]) not in self.allowed or gap>.1:
                        return dict(safe=False,reason='Local force contact does not match original selected scan surface',
                                    source_face=fid,scan_distance_mm=float(gap),contact_world_m=contact.pos.copy())
        return dict(safe=True,reason='')

    def _original(self,q):
        checker=self.checker; robot=checker.robot; model=checker.model
        robot.fk(q,full=True); data=robot.verify_data
        tip=data.site_xpos[robot.site]
        # Self-collision and fixtures use the unchanged original robot geometry.
        for c in data.contact:
            names=[model.body(int(model.geom_bodyid[g])).name for g in c.geom]
            if any(n.startswith('ur_') for n in names):
                return dict(safe=False,reason='Original robot/fixture or self collision',bodies=names,contact_world_m=c.pos.copy())
        for geom,body,obj in checker.parts:
            obj.setTransform(fcl.Transform(data.geom_xmat[geom].reshape(3,3),data.geom_xpos[geom]))
            corners=checker.part_bounds[geom]@data.geom_xmat[geom].reshape(3,3).T+data.geom_xpos[geom]
            low,high=corners.min(0),corners.max(0)
            for jaw,tooth in checker.teeth:
                tl,th=checker.tooth_bounds[jaw]
                if np.linalg.norm(np.maximum(0.,np.maximum(tl-high,low-th)))>.0001: continue
                result=fcl.CollisionResult()
                count=fcl.collide(obj,tooth,fcl.CollisionRequest(num_max_contacts=1000,enable_contact=True),result)
                if not count: continue
                name=model.geom(geom).name
                for c in result.contacts:
                    fid=int(c.b2)
                    allowed=(name=='ur_end_scaler_geom' and jaw==self.jaw
                        and np.linalg.norm(c.pos-tip)<=.003 and c.penetration_depth<=.0001
                        and 0<=fid<len(self.labels) and int(self.labels[fid]) in self.allowed and fid in self.covered)
                    if not allowed:
                        return dict(safe=False,reason='Forbidden original complete-mesh collision',geometry=name,jaw=jaw,
                            source_face=fid,contact_world_m=np.asarray(c.pos),tip_distance_mm=float(np.linalg.norm(c.pos-tip)*1000),
                            penetration_mm=float(c.penetration_depth*1000))
                if count>=1000:
                    return dict(safe=False,reason='Original mesh contact query saturated; cannot certify all contacts')
        return dict(safe=True,reason='')
