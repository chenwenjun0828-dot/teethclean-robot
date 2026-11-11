"""Bounded, deterministic bidirectional joint-space search for free transfers."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .robot import rotation_error


def straight_tcp_motion(robot, checker, start_q, target, cfg):
    start = robot.fk(start_q)
    distance = np.linalg.norm(target[:3, 3]-start[:3, 3])
    angle = np.linalg.norm(rotation_error(target[:3, :3], start[:3, :3]))
    count = max(2, int(np.ceil(distance/.001)), int(np.ceil(angle/np.deg2rad(3))))
    interpolation = Slerp([0, 1], Rotation.from_matrix([start[:3, :3], target[:3, :3]]))
    result = [start_q]
    for fraction in np.linspace(0, 1, count+1)[1:]:
        pose = np.eye(4)
        pose[:3, 3] = (1-fraction)*start[:3, 3]+fraction*target[:3, 3]
        pose[:3, :3] = interpolation(fraction).as_matrix()
        bank = robot.solutions(pose, result[-1])
        chosen = next((q for q in bank if np.max(abs(q-result[-1])) <= cfg.joint_step_rad
                       and checker.edge(result[-1], q, cfg.clearance_mm*.001, cfg.sweep_step_rad).valid), None)
        if chosen is None:
            return None
        result.append(chosen)
    return result


def _edge_valid(cfg, checker):
    """Segment predicate used by every free-space search.

    geometry_only has no collision predicate at all, so a segment is valid once
    it exists; the transport time law (not this predicate) bounds the joint and
    Cartesian rates.
    """
    if getattr(cfg, 'geometry_only', False):
        return lambda a, b: True
    return lambda a, b: checker.edge(a, b, cfg.clearance_mm*.001, cfg.sweep_step_rad).valid


def initial_approach(robot, checker, cfg, target_q, outward_normal):
    """Connect home to the sweep start without adding any internal transfers."""
    clearance = cfg.clearance_mm*.001
    valid = _edge_valid(cfg, checker)
    direct = checker.edge(robot.home, target_q, clearance, cfg.sweep_step_rad)
    if valid(robot.home, target_q):
        return {'segments': [('transfer', [target_q])], 'retreat_mm': 0.,
                'method': 'direct_initial_approach', 'clearance_m': direct.clearance_m}
    distances = sorted({value for value in (2., 5., 10., 15., cfg.retreat_mm)
                        if value <= cfg.retreat_mm+1e-9})
    for distance in distances:
        target = robot.fk(target_q)
        target[:3, 3] += distance*.001*outward_normal
        reverse = straight_tcp_motion(robot, checker, target_q, target, cfg)
        if reverse is None:
            continue
        ingress = reverse[::-1]
        if valid(robot.home, ingress[0]):
            path, method, minimum = [robot.home, ingress[0]], 'offset_initial_approach', direct.clearance_m
        else:
            path = search_transfer(robot.home, ingress[0], robot.lower, robot.upper, valid)
            if path is None:
                continue
            method = 'joint_search_initial_approach'
            minimum = min(checker.edge(a, b, clearance, cfg.sweep_step_rad).clearance_m
                          for a, b in zip(path[:-1], path[1:]))
        segments = [('transfer', path[1:])]
        if len(ingress) > 1:
            segments.append(('ingress', ingress[1:]))
        return {'segments': segments, 'retreat_mm': distance,
                'method': method, 'clearance_m': minimum}
    return None


def final_retreat(robot, checker, cfg, start_q, outward_normal):
    """Leave the completed sweep outward, with checked free-space shortcuts."""
    target = robot.fk(start_q)
    target[:3, 3] += cfg.retreat_mm*.001*np.asarray(outward_normal)
    path = straight_tcp_motion(robot, checker, start_q, target, cfg)
    if path is None:
        return None
    valid = _edge_valid(cfg, checker)
    result, i = [], 0
    while i < len(path)-1:
        j = len(path)-1
        while j > i+1 and not valid(path[i], path[j]):
            j -= 1
        result.append(path[j])
        i = j
    return result


def search_transfer(start, goal, lower, upper, edge_valid, max_iterations=400, seed=304):
    """Return both endpoints and checked edges, or None. No Cartesian tracing here."""
    start, goal = np.asarray(start, float), np.asarray(goal, float)
    lower, upper = np.asarray(lower), np.asarray(upper)
    if np.any(start < lower) or np.any(start > upper) or np.any(goal < lower) or np.any(goal > upper):
        return None
    if not edge_valid(start, start) or not edge_valid(goal, goal):
        return None
    if edge_valid(start, goal):
        return [start.copy(), goal.copy()]
    trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]
    rng = np.random.default_rng(seed)
    local_low = np.maximum(lower, np.minimum(start, goal)-.8)
    local_high = np.minimum(upper, np.maximum(start, goal)+.8)

    def extend(tree, target):
        nodes, parents = tree
        nearest = int(np.argmin(np.linalg.norm(np.asarray(nodes)-target, axis=1)))
        delta = target-nodes[nearest]
        scale = min(1., .35/max(float(np.max(abs(delta))), 1e-12))
        candidate = nodes[nearest]+scale*delta
        if not edge_valid(nodes[nearest], candidate):
            return None, False
        nodes.append(candidate); parents.append(nearest)
        return len(nodes)-1, scale == 1.

    def ancestry(tree, index):
        chain = []
        while index >= 0:
            chain.append(tree[0][index]); index = tree[1][index]
        return chain[::-1]

    for iteration in range(max_iterations):
        side = iteration % 2
        first, second = trees[side], trees[1-side]
        if iteration % 5 == 0:
            sample = second[0][int(rng.integers(len(second[0])))]
        elif iteration % 7 == 0:
            sample = rng.uniform(lower, upper)
        else:
            sample = rng.uniform(local_low, local_high)
        a, _ = extend(first, sample)
        if a is None:
            continue
        for _ in range(64):
            b, reached = extend(second, first[0][a])
            if b is None:
                break
            if reached:
                path = ancestry(first, a)+ancestry(second, b)[-2::-1]
                if side == 1:
                    path.reverse()
                # Greedy shortening keeps every replacement edge validated.
                result, i = [path[0]], 0
                while i < len(path)-1:
                    j = len(path)-1
                    while j > i+1 and not edge_valid(path[i], path[j]):
                        j -= 1
                    result.append(path[j]); i = j
                return result
    return None
