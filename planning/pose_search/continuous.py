"""Whole-path IK branch selection with three prescribed local tool angles."""
import numpy as np
from scipy.spatial.transform import Rotation
from .constraints import pose_at, target_at
from ..config import SweepOptions


class SweepFailure(RuntimeError):
    def __init__(self, reason, **details):
        super().__init__(reason)
        self.details = {'reason': reason, **details}


def sample_sweep(path, cfg):
    """Pose-stage sampling is independent of geometry fitting and motion timing."""
    return path.samples(cfg.pose_spacing_mm*.001)


def whole_path_search(nodes, cfg, edge_valid, seed, node_valid=None):
    """Lazy shortest path on a layered graph; never restarts after an empty cut."""
    if len(nodes) < 2:
        raise SweepFailure('insufficient_path_samples')
    if not nodes or any(not bank for bank in nodes):
        layer = next((i for i, bank in enumerate(nodes) if not bank), 0)
        raise SweepFailure('no_collision_free_ik', layer=layer)
    transitions = []
    for i in range(1, len(nodes)):
        a, b = nodes[i-1], nodes[i]
        delta = np.asarray([x['q'] for x in a])[:, None]-np.asarray([x['q'] for x in b])[None, :]
        qa = Rotation.from_matrix(np.array([x['pose'][:3, :3] for x in a])).as_quat()
        qb = Rotation.from_matrix(np.array([x['pose'][:3, :3] for x in b])).as_quat()
        angle = np.degrees(2*np.arccos(np.clip(abs(qa@qb.T), 0., 1.)))
        cost = np.sum(delta**2, axis=2)+.0005*angle**2
        cost[(np.max(abs(delta), axis=2) > cfg.joint_step_rad) |
             (angle > cfg.max_orientation_step_deg)] = np.inf
        transitions.append(cost)
    checked, blocked = set(), []
    checked_nodes, blocked_nodes, blocked_node_details = set(), set(), []
    while True:
        costs = np.asarray([.05*np.sum((n['q']-seed)**2)+n.get('pose_penalty', 0.) for n in nodes[0]])
        for layer, index in blocked_nodes:
            if layer == 0: costs[index] = np.inf
        parents = []
        for i, edge in enumerate(transitions):
            matrix = costs[:, None]+edge
            best = np.argmin(matrix, axis=0)
            costs = matrix[best, np.arange(len(nodes[i+1]))]
            costs += np.asarray([n.get('pose_penalty', 0.) for n in nodes[i+1]])
            if not np.isfinite(costs).any():
                raise SweepFailure('no_continuous_ik_path', layer=i+1,
                                   rejected_edges=blocked[-20:], blocked_edge_count=len(blocked),
                                   blocked_node_count=len(blocked_nodes), rejected_nodes=blocked_node_details[-20:])
            parents.append(best)
        selected = [int(np.argmin(costs))]
        for parent in reversed(parents): selected.append(int(parent[selected[-1]]))
        selected.reverse()
        failed = False
        if node_valid is not None:
            for i, index in enumerate(selected):
                key = (i, index)
                if key in checked_nodes: continue
                valid, reason = node_valid(nodes[i][index])
                if valid:
                    checked_nodes.add(key)
                else:
                    blocked_nodes.add(key)
                    blocked_node_details.append({'layer': i, 'reason': reason})
                    if i: transitions[i-1][:, index] = np.inf
                    if i < len(transitions): transitions[i][index, :] = np.inf
                    failed = True
            if failed: continue
        for i, (a, b) in enumerate(zip(selected[:-1], selected[1:])):
            key = (i, a, b)
            if key in checked: continue
            valid, reason = edge_valid(nodes[i][a], nodes[i+1][b])
            if valid:
                checked.add(key)
            else:
                transitions[i][a, b] = np.inf
                blocked.append({'layer': i+1, 'reason': reason})
                failed = True
                break
        if not failed: return [bank[i] for bank, i in zip(nodes, selected)]


def candidate_nodes(path, parameters, robot, checker, world, cfg, fixed_roll_deg=0.):
    nodes, rejected = [], []
    SweepOptions(fixed_roll_deg=fixed_roll_deg)
    for i, s in enumerate(parameters):
        bank = []
        normal = path.normal(s)
        target = target_at(path, s, fixed_roll_deg, world, cfg)
        local_r = world[:3, :3].T@target[:3, :3]
        if (cfg.geometry_only or cfg.planning_mode == 'simple'
                or (local_r@checker.tool_center_tcp)@normal > .001):
            # Collision is evaluated lazily for globally selected candidates below.
            for q in robot.solutions(target):
                bank.append({'q': q, 'roll': float(fixed_roll_deg), 'pose': target})
        nodes.append(bank)
        if not bank:
            rejected.append({'layer': i, 'parameter': float(s), 'tooth': path.tooth(s),
                             'part': path.part_index(s)})
            raise SweepFailure('no_ik_candidates', layer=i, rejected_samples=rejected)
        if i % 40 == 0 or i == len(parameters)-1:
            print(f'Continuous candidates {i+1}/{len(parameters)}; invalid positions {len(rejected)}', flush=True)
    if rejected:
        raise SweepFailure('no_collision_free_ik', layer=rejected[0]['layer'], rejected_samples=rejected)
    return nodes


def solve_sweep(path, parameters, robot, checker, world, cfg, fixed_roll_deg=0.):
    """Select one IK solution per fixed-angle pose, checking full connectivity."""
    if cfg.planning_mode == 'simple':
        from ..simple import solve_sweep as solve_regional
        return solve_regional(path, parameters, robot, checker, world, cfg)
    nodes = candidate_nodes(path, parameters, robot, checker, world, cfg, fixed_roll_deg)
    if cfg.geometry_only:
        # No collision predicate exists, so the sequence only has to be
        # connected: adjacent node and orientation limits are enforced in the
        # graph itself by whole_path_search.
        def connected(a, b):
            return np.max(abs(a['q']-b['q'])) <= cfg.joint_step_rad, ''

        return whole_path_search(nodes, cfg, connected, robot.home)

    def check_edge(a, b):
        check = checker.edge(a['q'], b['q'], cfg.clearance_mm*.001, cfg.sweep_step_rad)
        return check.valid, check.pair

    def check_node(node):
        check = checker.check(node['q'], cfg.clearance_mm*.001)
        return check.valid, check.pair

    return whole_path_search(nodes, cfg, check_edge, robot.home, check_node)
