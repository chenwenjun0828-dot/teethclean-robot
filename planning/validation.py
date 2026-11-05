"""Planning collision setup and joint-sweep preflight."""
from __future__ import annotations
import mujoco
import numpy as np

def configure_collision_mode(model, mode):
    """Geometry playback disables jaw convex proxies in this in-memory model only."""
    if mode not in ('geometry', 'strict'):
        raise ValueError('Collision mode must be geometry or strict')
    if mode == 'geometry':
        for jaw in ('mandibular', 'maxillary'):
            geom_id = model.geom('dental_'+jaw+'_visual').id
            model.geom_contype[geom_id] = 0
            model.geom_conaffinity[geom_id] = 0
    else:
        for jaw in ('mandibular', 'maxillary'):
            geom_id = model.geom('dental_'+jaw+'_visual').id
            if model.geom_contype[geom_id] == 0 and model.geom_conaffinity[geom_id] == 0:
                raise ValueError('Dental collision already disabled: reload scene for strict mode')


def robot_contacts(model, data) -> list[str]:
    result = []
    for contact in data.contact[:data.ncon]:
        bodies = [model.body(int(model.geom_bodyid[g])).name for g in (contact.geom1, contact.geom2)]
        if any(body.startswith('ur_') for body in bodies):
            names = [model.geom(int(g)).name or bodies[i] for i, g in enumerate((contact.geom1, contact.geom2))]
            result.append(' / '.join(names))
    return result


def _validate_sweep(ik, q_start, q_end):
    steps = max(2, int(np.ceil(np.max(np.abs(q_end-q_start))/0.005))+1)
    for u in np.linspace(0., 1., steps):
        ik.data.qpos[:] = (1-u)*q_start+u*q_end
        ik.data.qvel[:] = 0
        mujoco.mj_forward(ik.model, ik.data)
        contacts = robot_contacts(ik.model, ik.data)
        if contacts:
            raise ValueError('Collision preflight failed: '+contacts[0])

