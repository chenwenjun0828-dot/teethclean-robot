"""Conservative tool reachability for an evidence-backed 12-tooth line candidate."""
import mujoco
import numpy as np
from types import SimpleNamespace
from pathlib import Path

from planning.infrastructure.io import fingerprint
from perception.tooth_segmentation.algorithm import face_neighbours
from .hold_envelope import distance_mm
from .local_surface import outward_normals


def verify_candidate(candidate, source, scan_file, vertices, faces, labels, reference):
    import json
    request = json.loads((source/'request.json').read_text(encoding='utf-8'))
    prediction = request['inputs']['prediction']
    if (candidate.get('source_plan_sha256') != fingerprint(source/'planning_report.json')['sha256']
            or candidate.get('source_scan_sha256') != fingerprint(scan_file)['sha256']
            or candidate.get('source_prediction_sha256') != fingerprint(
                Path(prediction)/'prediction.npz')['sha256']
            or candidate.get('tooth') != 12 or candidate.get('roll_deg') != 30.):
        raise ValueError('Line candidate source or pose policy changed')
    ids = np.asarray(candidate['face_ids'], int)
    if len(ids) < 2 or np.any((ids < 0) | (ids >= len(faces))) or np.any(labels[ids] != 12):
        raise ValueError('Line candidate must use connected original tooth-12 faces')
    edges = {tuple(sorted(pair)) for pair in face_neighbours(faces) if labels[pair[0]] == labels[pair[1]] == 12}
    if any(tuple(sorted((int(a), int(b)))) not in edges for a, b in zip(ids[:-1], ids[1:])):
        raise ValueError('Line candidate crosses a label or mesh gap')
    points = vertices[faces[ids]].mean(1)
    length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
    if not 1.8 <= length <= 2.2 or not np.isclose(length, candidate['length_mm'], atol=1e-6):
        raise ValueError('Line candidate must be a measured ~2 mm face-connected path')
    original = np.array([reference.evaluate(s) for s in np.linspace(0., reference.parts[0]['end'], 2000)])*1000
    from scipy.spatial import cKDTree
    distances = cKDTree(original).query(points)[0]
    if np.any(distances > 1.+1e-6):
        raise ValueError('Line candidate leaves the 1 mm tooth-side gingival band')
    normals = outward_normals(vertices, faces, ids, labels)
    q = np.asarray(candidate['q_standoff_rad'], float)
    qc = np.asarray(candidate['q_contact_rad'], float)
    if q.shape != (len(ids), 6) or not np.isfinite(q).all() or qc.shape != q.shape or not np.isfinite(qc).all():
        raise ValueError('Line candidate has invalid nominal joint poses')
    return ids, points, normals, q, distances


def from_candidate(model, data, robot, jaw, tool, vertices, faces, tool_ids,
                   points, normals, q_standoff, nominal_standoff_mm):
    envelopes = []
    for point, normal, q in zip(points, normals, q_standoff):
        data.qpos[robot.qadr] = q
        mujoco.mj_forward(model, data)
        jaw_rotation = data.xmat[jaw].reshape(3, 3)
        tip_scan = (data.site_xpos[robot.site]-data.xpos[jaw])@jaw_rotation*1000
        if np.linalg.norm(tip_scan-(point+nominal_standoff_mm*normal)) > .02:
            raise ValueError('Line candidate joint pose disagrees with original scan/standoff')
        tool_world = vertices@data.xmat[tool].reshape(3, 3).T/1000+data.xpos[tool]
        tool_scan = (tool_world-data.xpos[jaw])@jaw_rotation*1000
        triangles = tool_scan[faces[tool_ids]]
        centers = triangles.mean(1)
        radii = np.linalg.norm(triangles-centers[:, None], axis=2).max(1)
        envelopes.append(dict(centers_mm=centers, radii_mm=radii, normal_scan=normal))
    return envelopes


def prepare_motion(model, data, plan, checker, saved, reference, points, normals,
                   q_standoff, q_contact, labels):
    """Rebuild line poses and clearance transfers against the original meshes."""
    from .scan_guard import OriginalScanGuard
    from .force_servo import apply_tool_roll
    robot = checker.robot
    first = saved['motion_segments'][0]['trace_start_waypoint']
    first_q = plan.joints[first]
    jaw_name = saved.get('jaw', 'mandibular')
    jaw = model.body('dental_'+jaw_name).id
    data.qpos[robot.qadr] = first_q
    mujoco.mj_forward(model, data)
    jaw_rot = data.xmat[jaw].reshape(3, 3)
    origin = data.xpos[jaw].copy()
    first_pose = robot.fk(first_q).copy()
    first_clear_pose = first_pose.copy()
    first_clear_pose[:3, 3] += jaw_rot@reference.normal(0.)*.002
    first_clear = robot.solutions(first_clear_pose, seed=first_q)
    if not first_clear:
        raise ValueError('No IK for original first-tooth clearance')
    first_clear = first_clear[0]
    rolled = apply_tool_roll(first_pose, np.radians(30.))
    q_contact = np.asarray(q_contact, float)
    clear = []
    guard = SimpleNamespace(checker=checker, labels=labels, allowed={12}, jaw=jaw_name,
                            covered=set(np.flatnonzero(labels == 12).tolist()))
    for point, normal, qs, qc in zip(points, normals, q_standoff, q_contact):
        for offset, q in ((1.8, qs), (-.02, qc)):
            pose = robot.fk(q, full=True)
            expected = origin+jaw_rot@((point+offset*normal)/1000)
            if (np.linalg.norm(pose[:3, 3]-expected) > 2e-5
                    or np.linalg.norm(pose[:3, :3]-rolled[:3, :3]) > 1e-4):
                raise ValueError('Line pose differs from scanned surface or fixed orientation')
            if offset > 0 and not checker.check(q, .0001).valid:
                raise ValueError('Line nominal standoff collides with original scan')
            if offset < 0 and not OriginalScanGuard._original(guard, q)['safe']:
                raise ValueError('Line diagnostic contact collides with forbidden original surface')
        pose = rolled.copy()
        pose[:3, 3] = origin+jaw_rot@((point+3.8*normal)/1000)
        solutions = robot.solutions(pose, seed=qs)
        if not solutions or not checker.check(solutions[0], .0001).valid:
            raise ValueError('No collision-free line clearance pose')
        clear.append(solutions[0])
    clear = np.asarray(clear)
    if not checker.edge(first_clear, clear[0], .0001).valid:
        raise ValueError('Original ingress to line clearance collides')
    for a, b in zip(clear[:-1], clear[1:]):
        if not checker.edge(a, b, .0001).valid:
            raise ValueError('Line clearance traverse collides')
    for a, b in zip(q_contact[:-1], q_contact[1:]):
        for alpha in (.25, .5, .75):
            if not OriginalScanGuard._original(guard, (1-alpha)*a+alpha*b)['safe']:
                raise ValueError('Line contact interpolation collides with forbidden original surface')
    return first_clear, clear


def reachable_failures(records, envelopes):
    failed = [r for r in records if not r['geometry_passed'] and r.get('probe_valid', True)]
    if not failed:
        return [], []
    points = np.array([[r[k] for k in ('x_mm', 'y_mm', 'z_mm')] for r in failed])
    distances = np.min([distance_mm(points, envelope) for envelope in envelopes], axis=0)
    return [r for r, d in zip(failed, distances) if d <= .1], distances.tolist()
