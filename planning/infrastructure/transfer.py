"""Cartesian IK connections and independent joint-space search utilities."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .robot import rotation_error


def straight_tcp_motion(robot, checker, start_q, target, cfg, *, target_q=None, refinement=1):
    from planning.pose_search.continuous import whole_path_search
    start = robot.fk(start_q)
    distance = np.linalg.norm(target[:3, 3]-start[:3, 3])
    angle = np.linalg.norm(rotation_error(target[:3, :3], start[:3, :3]))
    count = refinement*max(2, int(np.ceil(distance/.001)), int(np.ceil(angle/np.deg2rad(3))))
    interpolation = Slerp([0, 1], Rotation.from_matrix([start[:3, :3], target[:3, :3]]))
    nodes = [[{'q': np.array(start_q, float), 'pose': start}]]
    for fraction in np.linspace(0, 1, count+1)[1:]:
        pose = np.eye(4)
        pose[:3, 3] = (1-fraction)*start[:3, 3]+fraction*target[:3, 3]
        pose[:3, :3] = interpolation(fraction).as_matrix()
        seed = start_q if target_q is None else (1-fraction)*start_q+fraction*target_q
        bank = ([np.array(target_q, float)] if target_q is not None and fraction == 1.
                else robot.solutions(pose, seed))
        nodes.append([{'q': q, 'pose': pose} for q in bank])
    def node_valid(node):
        q = node['q']
        if np.any(q < robot.lower) or np.any(q > robot.upper):
            return False, 'joint_limits'
        actual = robot.fk(q)
        if (np.linalg.norm(actual[:3, 3]-node['pose'][:3, 3])*1000 > cfg.interpolation_position_mm
                or np.degrees(np.linalg.norm(rotation_error(node['pose'][:3, :3], actual[:3, :3])))
                > cfg.interpolation_angle_deg):
            return False, 'endpoint_pose_mismatch'
        if cfg.geometry_only:
            return True, ''
        result = checker.check(q, cfg.clearance_mm*.001)
        return result.valid, result.pair
    def edge_valid(a, b):
        if cfg.geometry_only:
            return True, ''
        result = checker.edge(a['q'], b['q'], cfg.clearance_mm*.001, cfg.sweep_step_rad)
        return result.valid, result.pair
    selected = whole_path_search(nodes, cfg, edge_valid, start_q, node_valid)
    return [node['q'] for node in selected]


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
    """Prepare straight TCP entry; outward_normal is retained for call compatibility."""
    from planning.motion_planning.free_motion import prepare_transfer
    from planning.pose_search.continuous import SweepFailure
    try:
        result = prepare_transfer(robot, checker, cfg, robot.home, target_q)
    except SweepFailure as error:
        error.details['phase'] = 'transfer'
        raise
    return dict(result, segments=[('transfer', result['joints'][1:])], retreat_mm=0.,
                method='straight_tcp_entry')


def final_retreat(robot, checker, cfg, start_q, outward_normal):
    """Prepare straight TCP return to the exact initial joint state."""
    from planning.motion_planning.free_motion import prepare_transfer
    from planning.pose_search.continuous import SweepFailure
    try:
        return prepare_transfer(robot, checker, cfg, start_q, robot.home)
    except SweepFailure as error:
        error.details['phase'] = 'retreat'
        raise


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
