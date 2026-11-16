"""Online complete original-mesh guard, independent of MuJoCo contact proxies."""
import json
from pathlib import Path

import fcl
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from .local_surface import scene, outward_normals, contact_metrics, closest_on_triangles


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
            self.tip_mm = z['tip'].copy()
            self.references = {}; self.reference_normals = {}; self.triangles = {}
            for role, prefix in (('tooth',''),('tool','tool_')):
                v, f = z[prefix+'vertices'], z[prefix+'faces']
                labels = z['labels'] if role=='tooth' else None
                self.references[role] = scene(v,f)
                self.reference_normals[role] = outward_normals(v,f,np.arange(len(f)),labels)
                self.triangles[role] = v[f].copy()

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
        envelope=self.info.get('hold_envelope')
        if envelope is not None and phase in ('acquire','trace','release'):
            line=self.info.get('line_trace')
            if line is not None:
                positions=np.asarray(line['point_scan_mm'])+1.8*np.asarray(line['normal_scan'])
                nearest=int(np.argmin(np.linalg.norm(positions-local_tip,axis=1)))
                delta=local_tip-positions[nearest]
                n=np.asarray(line['normal_scan'][nearest])
            else:
                delta=local_tip-np.asarray(envelope['tip_mm'])
                n=np.asarray(envelope['normal_scan'])
            along=float(delta@n)
            lateral=float(np.linalg.norm(delta-along*n))
            tool_body=self.model.body('ur_end_scaler_link').id
            current=data.xmat[body].reshape(3,3).T@data.xmat[tool_body].reshape(3,3)
            previous=np.asarray(envelope['tool_rotation_scan'])
            angle=float(np.degrees(np.arccos(np.clip((np.trace(previous.T@current)-1)/2,-1,1))))
            if abs(along)>3. or lateral>1. or angle>5.:
                return dict(safe=False,reason='Tool left certified single-tooth hold reachability',
                            along_mm=along,lateral_mm=lateral,rotation_deg=angle)
        if phase in ('acquire','trace','release') and self.band.query(local_tip)[0] > self.info['band_radius_mm']-1.:
            return dict(safe=False,reason='TCP left verified local work-band coverage',tcp_scan_mm=local_tip)
        evidence = []
        for contact in data.contact:
            names = [self.model.geom(int(g)).name for g in contact.geom]
            if not any(n in self.parts and self.parts[n]['role']=='tool' for n in names): continue
            if contact.dist < -.0001:
                return dict(safe=False,reason='Local contact penetration exceeds 0.1 mm',geometries=names,
                            contact_world_m=contact.pos.copy(),penetration_mm=float(-contact.dist*1000))
            for side, name in enumerate(names):
                if name not in self.parts: continue
                part=self.parts[name]; bid=self.model.body(part['body']).id
                point=(contact.pos-data.xpos[bid])@data.xmat[bid].reshape(3,3)*1000
                if name not in self.scenes:
                    mesh=o3d.io.read_triangle_mesh(str(self.directory/part['file']))
                    self.scenes[name]=scene(np.asarray(mesh.vertices),np.asarray(mesh.triangles))
                result=self.scenes[name].compute_closest_points(o3d.core.Tensor(point[None].astype(np.float32)))
                face=int(result['primitive_ids'].numpy()[0])
                role = part['role']
                # MuJoCo's first frame axis points from geom[0] to geom[1].
                # Each reference needs that body's outward normal, in its frame.
                world_normal = np.asarray(contact.frame).reshape(3,3)[0]*(1 if side==0 else -1)
                local_normal = world_normal@data.xmat[bid].reshape(3,3)
                source_ids = np.asarray(part['source_face_ids'],int)
                nearest, indices = closest_on_triangles(point[None], self.triangles[role][source_ids])
                metric = contact_metrics(self.references[role], point[None], local_normal[None],
                    self.reference_normals[role], reference=(nearest,source_ids[indices]))
                record = dict(geometry=name,facet=face,hit_kind=part['face_kinds'][face],
                    source_face=int(metric['source_faces'][0]),
                    scan_distance_mm=float(metric['distance_mm'][0]),
                    signed_distance_mm=float(metric['signed_distance_mm'][0]),
                    normal_error_deg=float(metric['normal_error_deg'][0]))
                if not metric['geometry_passed'][0]:
                    return dict(safe=False,reason='Local contact distance differs from original surface',
                                **record,contact_world_m=contact.pos.copy())
                evidence.append(record)
                if role=='tool' and np.linalg.norm(point-self.tip_mm)>3.:
                    return dict(safe=False,reason='Local contact is outside actual tool tip', **record)
                if part['role']=='tooth':
                    fid=int(metric['source_faces'][0])
                    gap=float(metric['distance_mm'][0])
                    if fid not in self.covered or int(self.labels[fid]) not in self.allowed or gap>.1:
                        return dict(safe=False,reason='Local force contact does not match original selected scan surface',
                                    source_face=fid,scan_distance_mm=float(gap),contact_world_m=contact.pos.copy())
        return dict(safe=True,reason='',contacts=evidence)

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
