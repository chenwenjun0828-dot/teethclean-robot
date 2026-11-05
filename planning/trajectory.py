"""Six-dimensional gingival TCP trajectory and joint plan."""
from __future__ import annotations
from dataclasses import dataclass, field
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from perception.gingival_line import ROOT, RegisteredPaths, resample_path
from perception.frames import camera_world_transform_m, validate_transform
from .ik import FixedPoseIK
from .validation import _validate_sweep
from .tooth_frames import ToothFrames, build_tooth_frames

def smoothstep(value: float) -> float:
    value = float(np.clip(value, 0, 1))
    return value**3*(10+value*(-15+6*value))


@dataclass
class TrackingPlan:
    model: object
    site_id: int
    rotations: np.ndarray
    path: np.ndarray
    trace_q: np.ndarray
    arc: np.ndarray
    home_q: np.ndarray
    approach_q: np.ndarray
    retreat_q: np.ndarray
    approach_duration: float
    ingress_duration: float
    trace_duration: float
    retreat_duration: float
    path_index: int | None = None
    offset_mm: float = 0.
    orientation_mode: str = 'fixed'
    tooth_frames: ToothFrames | None = None
    frame_report: dict | None = None
    command_q: np.ndarray | None = None
    command_arc: np.ndarray | None = None
    scratch: object = field(init=False, repr=False)
    _rotation_interpolator: object = field(init=False, repr=False)

    def __post_init__(self):
        self.scratch = mujoco.MjData(self.model)
        self._rotation_interpolator = Slerp(self.arc, Rotation.from_matrix(self.rotations))
        if self.command_q is None:
            self.command_q = self.trace_q
            self.command_arc = self.arc

    @property
    def rotation(self):
        """First waypoint orientation; use rotations/sample_rotation for tracking."""
        return self.rotations[0]

    @property
    def trace_transforms(self):
        """TCP-local -> world targets, translation in metres."""
        return ToothFrames(self.path, self.rotations, 'world', 'm').transforms

    @property
    def trace_poses(self) -> np.ndarray:
        """Ordered [x_m,y_m,z_m,roll,pitch,yaw]; extrinsic xyz, radians."""
        rpy = Rotation.from_matrix(self.rotations).as_euler('xyz')
        return np.column_stack((self.path, rpy))

    @property
    def waypoint_poses(self) -> np.ndarray:
        """Six-dimensional home, approach, trace, and retreat waypoints."""
        poses = []
        for q in np.vstack((self.home_q, self.approach_q, self.command_q, self.retreat_q)):
            self.scratch.qpos[:] = q
            mujoco.mj_kinematics(self.model, self.scratch)
            xyz = self.scratch.site_xpos[self.site_id].copy()
            rpy = Rotation.from_matrix(
                self.scratch.site_xmat[self.site_id].reshape(3, 3)
            ).as_euler('xyz')
            poses.append(np.r_[xyz, rpy])
        return np.asarray(poses)

    @property
    def trace_start(self):
        return self.approach_duration+self.ingress_duration

    @property
    def trace_end(self):
        return self.trace_start+self.trace_duration

    @property
    def duration(self):
        return self.trace_end+self.retreat_duration

    def trace_station(self, time_s):
        return self.arc[-1]*smoothstep((time_s-self.trace_start)/self.trace_duration)

    def sample_rotation(self, time_s):
        """Desired world TCP orientation; SO(3) interpolation during tracing."""
        if self.trace_start <= time_s <= self.trace_end:
            return self._rotation_interpolator(self.trace_station(time_s)).as_matrix()
        _, q, _ = self.sample(time_s)
        self.scratch.qpos[:] = q
        mujoco.mj_kinematics(self.model, self.scratch)
        return self.scratch.site_xmat[self.site_id].reshape(3, 3).copy()

    def sample(self, time_s):
        """Return world TCP position, joint command and phase at simulation time."""
        time_s = float(np.clip(time_s, 0, self.duration))
        if time_s < self.approach_duration:
            u = smoothstep(time_s/self.approach_duration)
            q = (1-u)*self.home_q+u*self.approach_q
            phase = 'approach'
        elif time_s < self.trace_start:
            u = smoothstep((time_s-self.approach_duration)/self.ingress_duration)
            q = (1-u)*self.approach_q+u*self.trace_q[0]
            phase = 'ingress'
        elif time_s <= self.trace_end:
            s = self.trace_station(time_s)
            position = np.array([np.interp(s, self.arc, self.path[:, i]) for i in range(3)])
            q = np.array([np.interp(s, self.command_arc, self.command_q[:, i]) for i in range(6)])
            return position, q, 'trace'
        else:
            u = smoothstep((time_s-self.trace_end)/self.retreat_duration)
            q = (1-u)*self.trace_q[-1]+u*self.retreat_q
            phase = 'retreat'
        # Scratch kinematics only; never teleport the live simulation.
        self.scratch.qpos[:] = q
        mujoco.mj_kinematics(self.model, self.scratch)
        return self.scratch.site_xpos[self.site_id].copy(), q, phase


def plan_tracking(model, data, path, rotation, speed_m_s=0.004, retreat_m=0.03,
                  spacing_m=0.0005, outward=None, *, resample=True, motion_joint_step_rad=None):
    """Plan a fixed 3x3 or per-point Nx3x3 TCP orientation in world axes.

    Per-point poses are resampled together using arc-length SO(3) interpolation.
    Call with resample=False when tooth frames were built at final path samples.
    outward accepts a common direction or separate (start, end) directions.
    """
    if not np.isfinite(speed_m_s) or not 0 < speed_m_s <= 0.02:
        raise ValueError('Tracing speed must be in (0, 20] mm/s')
    if not np.isfinite(retreat_m) or not 0.005 <= retreat_m <= 0.1:
        raise ValueError('Approach clearance must be in [5, 100] mm')
    if motion_joint_step_rad is not None and (
            not np.isfinite(motion_joint_step_rad) or motion_joint_step_rad <= 0):
        raise ValueError('Motion joint step must be positive and finite')
    path = np.asarray(path, float)
    rotations = np.asarray(rotation, float)
    fixed = rotations.shape == (3, 3)
    if fixed:
        rotations = np.broadcast_to(rotations, (len(path), 3, 3)).copy()
    poses = ToothFrames(path, rotations, 'world', 'm')
    if len(path) < 2:
        raise ValueError('Path requires at least two points')
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    keep = np.r_[True, steps > 1e-12]
    if np.any(~keep) and not np.allclose(rotations[1:][~keep[1:]],
                                        rotations[:-1][~keep[1:]], atol=1e-8, rtol=0):
        raise ValueError('Repeated positions with different orientations are unsupported')
    path, rotations = poses.origins[keep], poses.rotations[keep]
    if len(path) < 2:
        raise ValueError('Path has zero length')
    if resample:
        source_arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
        sampled_path = resample_path(path, spacing_m)
        stations = np.linspace(0., source_arc[-1], len(sampled_path))
        rotations = Slerp(source_arc, Rotation.from_matrix(rotations))(stations).as_matrix()
        path = sampled_path
    ik = FixedPoseIK(model)
    home_q = data.qpos.copy()
    directions = -rotations[[0, -1], :, 2] if outward is None else np.asarray(outward, float)
    if directions.shape == (3,):
        directions = np.tile(directions, (2, 1))
    if (directions.shape != (2, 3) or not np.isfinite(directions).all()
            or np.any(np.linalg.norm(directions, axis=1) < 1e-10)):
        raise ValueError('Outward must contain one or two finite nonzero directions')
    directions = directions / np.linalg.norm(directions, axis=1)[:, None]
    approach_q = ik.solve(path[0]+retreat_m*directions[0], rotations[0], home_q)
    trace_q = []
    command_q, command_arc = [], []
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    seed = approach_q
    for index, point in enumerate(path):
        try:
            endpoint = ik.solve(point, rotations[index], seed)
            if index and motion_joint_step_rad is not None:
                # Auxiliary IK samples follow the Cartesian/SO(3) interpolant. They
                # reduce TCP drift caused by interpolating widely spaced joint poses.
                count = max(1, int(np.ceil(np.max(np.abs(endpoint-seed))/motion_joint_step_rad)),
                            int(np.ceil((arc[index]-arc[index-1])/.00025)))
                interp = Slerp([0., 1.], Rotation.from_matrix(rotations[index-1:index+1]))
                for fraction in np.arange(1, count)/count:
                    position = (1-fraction)*path[index-1]+fraction*point
                    seed = ik.solve(position, interp(fraction).as_matrix(), seed)
                    command_q.append(seed.copy())
                    command_arc.append(arc[index-1]+fraction*(arc[index]-arc[index-1]))
                endpoint = ik.solve(point, rotations[index], seed)
            seed = endpoint
        except ValueError as error:
            raise ValueError(f'Path sample {index}: {error}') from error
        trace_q.append(seed.copy())
        command_q.append(seed.copy())
        command_arc.append(arc[index])
    trace_q = np.asarray(trace_q)
    command_q, command_arc = np.asarray(command_q), np.asarray(command_arc)
    retreat_q = ik.solve(path[-1]+retreat_m*directions[1], rotations[-1], trace_q[-1])
    # Validate approach, ingress, each trace interval and retreat in the same model.
    sequence = np.vstack((home_q, approach_q, command_q, retreat_q))
    for index, (a, b) in enumerate(zip(sequence[:-1], sequence[1:])):
        try:
            _validate_sweep(ik, a, b)
        except ValueError as error:
            raise ValueError(f'Joint interval {index}: {error}') from error
    # Whole-phase quintic pacing has peak speed 1.875 times its average.
    approach_duration = max(8., 1.875*np.max(np.abs(approach_q-home_q))/0.3)
    ingress_duration = max(4., 1.875*np.max(np.abs(trace_q[0]-approach_q))/0.3)
    retreat_duration = max(4., 1.875*np.max(np.abs(retreat_q-trace_q[-1]))/0.3)
    trace_duration = max(2., 1.875*arc[-1]/speed_m_s)
    joint_slope = np.max(np.abs(np.diff(command_q, axis=0))/np.diff(command_arc)[:, None])
    trace_duration = max(trace_duration, 1.875*arc[-1]*joint_slope/0.3)
    plan = TrackingPlan(model, ik.site_id, rotations.copy(), path, trace_q, arc, home_q,
                        approach_q, retreat_q, approach_duration, ingress_duration,
                        trace_duration, retreat_duration)
    plan.orientation_mode = 'fixed' if fixed else 'per_point'
    plan.command_q, plan.command_arc = command_q, command_arc
    return plan


@dataclass
class ToothAlignedTargets:
    tooth: ToothFrames
    tcp: ToothFrames
    report: dict
    path_index: int
    offset_mm: float


def registered_tracking_targets(model, data, bundle: RegisteredPaths, path_index=0,
                                offset_mm=0., *, mesh_path=None, labelled_mesh_path=None,
                                gingivaward=None):
    """Build exact per-point tooth/TCP targets without executing or solving IK."""
    if not isinstance(path_index, int) or not 0 <= path_index < len(bundle.chains):
        raise ValueError(f'Path index must be in [0, {len(bundle.chains)-1}]')
    if not np.isfinite(offset_mm) or not 0 <= offset_mm <= 30:
        raise ValueError('Offset must be in [0, 30] mm')
    live_camera = camera_world_transform_m(data, bundle.camera_name)
    if not np.allclose(live_camera, bundle.world_from_camera, atol=1e-6):
        raise ValueError('Camera pose changed since capture; register the current scene again')
    body = data.body('dental_'+bundle.jaw)
    # Simulator-only freshness check; trajectory still uses the estimated registration.
    truth = bundle.scan_points_mm/1000. @ body.xmat.reshape(3, 3).T+body.xpos
    if np.sqrt(np.mean(np.sum((truth-bundle.world_points)**2, axis=1))) > 0.002:
        raise ValueError('Registered line does not match current dental pose within 2 mm')
    path = bundle.chains[path_index].copy()
    if np.linalg.norm(path[-1]-data.site('ur_scaler_tcp').xpos) < np.linalg.norm(path[0]-data.site('ur_scaler_tcp').xpos):
        path = path[::-1]
    # Compute geometry AFTER reversal and resampling, never match poses by CSV row.
    path = resample_path(path, bundle.sampling_spacing_m)
    if bundle.registration_dir is not None:
        report = json.loads((Path(bundle.registration_dir)/'registration_report.json')
                            .read_text(encoding='utf-8-sig'))
        if mesh_path is None:
            mesh_path = Path(report['config'][bundle.jaw+'_mesh'])
        mesh_path = Path(mesh_path)
        if not mesh_path.is_absolute():
            mesh_path = ROOT/mesh_path
        if hashlib.sha256(mesh_path.read_bytes()).hexdigest() != report['source_hashes'][bundle.jaw]:
            raise ValueError('Oral scan hash differs from the registered source mesh')
        if labelled_mesh_path is None:
            line = Path(report['config'][bundle.jaw+'_line'])
            if not line.is_absolute():
                line = ROOT/line
            candidate = line.parent/(bundle.jaw+'_vertex_labels.ply')
            if candidate.is_file():
                labelled_mesh_path = candidate
    if mesh_path is None:
        raise ValueError('Tooth-aligned planning requires the registered oral scan mesh')
    transform = validate_transform(bundle.world_from_scan)
    points_mm = (path-transform[:3, 3]) @ transform[:3, :3] * 1000.
    scan_frames, distances, frame_report = build_tooth_frames(
        mesh_path, points_mm, bundle.jaw, labelled_mesh_path=labelled_mesh_path,
        gingivaward=gingivaward)
    world_frames = scan_frames.transformed(transform, 'world', 'm')
    path = world_frames.origins + world_frames.rotations[:, :, 2]*offset_mm/1000.
    frame_report = dict(frame_report, measured_max_surface_distance_mm=float(distances.max()),
                        mesh_path=str(Path(mesh_path).resolve()),
                        labelled_mesh_path=(str(Path(labelled_mesh_path).resolve())
                                            if labelled_mesh_path is not None else None))
    return ToothAlignedTargets(world_frames, ToothFrames(path, world_frames.rotations, 'world', 'm'),
                               frame_report, path_index, offset_mm)


def filter_tooth_orientation_targets(targets: ToothAlignedTargets, max_step_deg=45.):
    """Skip excessive orientation constraints, preserving the entire spatial path.

    Compare each tooth pose to the last accepted pose using the SO(3) angle.
    Rejected rotations are replaced by arc-length Slerp between accepted poses;
    leading/trailing positions keep the nearest accepted rotation.
    """
    if not np.isfinite(max_step_deg) or not 0 < max_step_deg <= 180:
        raise ValueError('Orientation step must be in (0, 180] degrees')
    rotations = targets.tcp.rotations
    keep, skipped = [0], []
    for index in range(1, len(rotations)):
        previous = keep[-1]
        angle = float(np.rad2deg(Rotation.from_matrix(
            rotations[index] @ rotations[previous].T).magnitude()))
        if angle > max_step_deg:
            skipped.append({'sample_index': index, 'previous_retained_index': previous,
                            'angle_deg': angle, 'reason': 'orientation_step_exceeds_limit'})
        else:
            keep.append(index)
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(targets.tcp.origins, axis=0), axis=1))]
    if len(keep) == 1:
        filtered = np.tile(rotations[0], (len(rotations), 1, 1))
    else:
        filtered = Slerp(arc[keep], Rotation.from_matrix(rotations[keep]))(
            np.clip(arc, arc[keep[0]], arc[keep[-1]])).as_matrix()
    # Retained target matrices stay exactly as requested, without round-off conversion.
    filtered[keep] = rotations[keep]
    report = dict(targets.report, orientation_filter={
        'max_step_deg': float(max_step_deg), 'comparison': 'last_retained_sample',
        'source_samples': len(rotations), 'retained_orientation_count': len(keep),
        'skipped_orientation_count': len(skipped), 'retained_sample_indices': keep,
        'skipped_samples': skipped, 'spatial_path_preserved': True,
        'replacement': 'arc_length_slerp_and_hold_at_ends',
    })
    return ToothAlignedTargets(targets.tooth,
        ToothFrames(targets.tcp.origins, filtered, 'world', 'm'), report,
        targets.path_index, targets.offset_mm)


def plan_tooth_aligned_tracking(model, data, targets: ToothAlignedTargets, speed_mm_s=4.):
    """Solve all exact targets and validate the entire joint motion before execution."""
    plan = plan_tracking(model, data, targets.tcp.origins, targets.tcp.rotations,
                         speed_m_s=speed_mm_s/1000., resample=False, motion_joint_step_rad=.02,
                         outward=targets.tcp.rotations[[0, -1], :, 2])
    plan.path_index = targets.path_index
    plan.offset_mm = targets.offset_mm
    plan.orientation_mode = ('tooth_aligned_filtered' if 'orientation_filter' in targets.report
                             else 'tooth_aligned')
    plan.tooth_frames = targets.tooth
    plan.frame_report = targets.report
    return plan


def plan_registered_tracking(model, data, bundle: RegisteredPaths, path_index=0, offset_mm=0.,
                             speed_mm_s=4., *, mesh_path=None, labelled_mesh_path=None,
                             gingivaward=None, max_orientation_step_deg=None):
    targets = registered_tracking_targets(model, data, bundle, path_index, offset_mm,
                    mesh_path=mesh_path, labelled_mesh_path=labelled_mesh_path, gingivaward=gingivaward)
    if max_orientation_step_deg is not None:
        targets = filter_tooth_orientation_targets(targets, max_orientation_step_deg)
    return plan_tooth_aligned_tracking(model, data, targets, speed_mm_s)

