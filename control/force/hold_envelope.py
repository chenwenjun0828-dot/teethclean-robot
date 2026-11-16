"""Conservative reachable volume for the first-tooth stationary hold (mm)."""
import mujoco
import numpy as np


def from_plan(model, data, plan, checker, saved, vertices, faces, tool_ids, normal_scan,
              hold_face_id=None, scan_vertices=None, scan_faces=None, first_normal_scan=None):
    robot=checker.robot
    first=saved['motion_segments'][0]['trace_start_waypoint']
    first_q=plan.joints[first]
    data.qpos[robot.qadr]=first_q
    mujoco.mj_forward(model,data)
    jaw=model.body('dental_'+saved['jaw']).id if 'jaw' in saved else model.body('dental_mandibular').id
    tool=model.body('ur_end_scaler_link').id
    tip=model.site('ur_scaler_tcp').id
    jaw_rotation=data.xmat[jaw].reshape(3,3)
    normal_scan=np.asarray(normal_scan,float)
    normal_world=jaw_rotation@normal_scan
    first_normal_world=jaw_rotation@np.asarray(
        normal_scan if first_normal_scan is None else first_normal_scan,float)
    first_pose=robot.fk(first_q,full=True).copy()
    target_pose=first_pose.copy()
    if hold_face_id is not None:
        center=scan_vertices[scan_faces[hold_face_id]].mean(0)
        target_pose[:3,3]=data.xpos[jaw]+jaw_rotation@((center+1.8*normal_scan)/1000)
    target_q=robot.solutions(target_pose,seed=first_q)
    if not target_q: raise ValueError('No bounded IK for alternate tooth-12 hold point')
    target_q=target_q[0]
    first_clearance=first_pose.copy();first_clearance[:3,3]+=first_normal_world*.002
    target_clearance=target_pose.copy();target_clearance[:3,3]+=normal_world*.002
    first_clear_q=robot.solutions(first_clearance,seed=first_q)
    target_clear_q=robot.solutions(target_clearance,seed=target_q)
    if not first_clear_q or not target_clear_q:
        raise ValueError('No bounded IK for hold clearance transfer')
    first_clear_q,target_clear_q=first_clear_q[0],target_clear_q[0]
    if hold_face_id is not None:
        if not checker.check(target_q,.0001).valid or not checker.edge(first_clear_q,target_clear_q,.0001).valid:
            raise ValueError('Alternate hold target or clearance transfer collides with original meshes')
    data.qpos[robot.qadr]=target_q
    mujoco.mj_forward(model,data)
    tool_world=vertices@data.xmat[tool].reshape(3,3).T/1000+data.xpos[tool]
    tool_scan=(tool_world-data.xpos[jaw])@jaw_rotation*1000
    triangles=tool_scan[faces[tool_ids]]
    centers=triangles.mean(1)
    radii=np.linalg.norm(triangles-centers[:,None],axis=2).max(1)
    return dict(centers_mm=centers,radii_mm=radii,
        tip_mm=(data.site_xpos[tip]-data.xpos[jaw])@jaw_rotation*1000,
        normal_scan=normal_scan,
        tool_rotation_scan=jaw_rotation.T@data.xmat[tool].reshape(3,3),
        target_q_rad=target_q,first_clearance_q_rad=first_clear_q,
        target_clearance_q_rad=target_clear_q)


def distance_mm(points, envelope):
    """Lower bound to any swept tool triangle under +/-3 mm normal travel.

    The 1.5 mm allowance covers lateral tracking and small orientation error.
    An out-of-envelope failure may be excluded only if this bound exceeds it.
    """
    points=np.asarray(points,float).reshape(-1,3)
    centers=envelope['centers_mm']; n=envelope['normal_scan']
    delta=points[:,None,:]-centers[None]
    along=np.einsum('ijk,k->ij',delta,n)
    nearest=delta-np.clip(along,-3.,3.)[:,:,None]*n
    return (np.linalg.norm(nearest,axis=2)-envelope['radii_mm'][None]).min(1)-1.5


def reachable_failures(records, envelope):
    failed=[r for r in records if not r['geometry_passed'] and r.get('probe_valid',True)]
    if not failed: return [],[]
    points=np.array([[r[k] for k in ('x_mm','y_mm','z_mm')] for r in failed])
    distances=distance_mm(points,envelope)
    return [r for r,d in zip(failed,distances) if d<=.1], distances.tolist()
