"""Load checked-in real-camera perception sessions for downstream planning."""
from __future__ import annotations

from dataclasses import dataclass, replace
import json
from pathlib import Path

import numpy as np

from .regions import PreparedRegion
from .regions.io import fingerprint, manifest_path, validate_transform


DEFAULT_SESSION = Path('assets/perception/sessions/20261113_220209')
CAMERA_FRAMES = ('declared', 'color_optical')
DEFAULT_SESSION_CAMERA_FRAME = 'color_optical'
CAMERA_FRAME_VERSION = 'session_camera_frame_interpretation_v1'
REGIONS = ('outer_left', 'outer_front', 'outer_right',
           'inner_right', 'inner_front', 'inner_left')
ROBOT_JOINT_NAMES = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


@dataclass(frozen=True)
class SessionRobotObservation:
    """Capture-time robot joint observation recorded with a perception session."""

    root: Path
    joint_names: tuple[str, ...]
    joint_positions_rad: np.ndarray
    metadata: dict


def load_session_robot_observation(session_root):
    """Load and validate the capture-time joint angles in radians."""
    root = Path(session_root).resolve()
    path = root / 'robot_joint_observation.json'
    if not path.is_file():
        raise FileNotFoundError(path)
    record = json.loads(path.read_text(encoding='utf-8'))
    names = tuple(record.get('joint_names', ()))
    if names != ROBOT_JOINT_NAMES:
        raise ValueError(f'Session joint order must be {ROBOT_JOINT_NAMES}, got {names}')
    positions = np.asarray(record.get('joint_positions_rad'), dtype=float)
    if positions.shape != (len(ROBOT_JOINT_NAMES),) or not np.isfinite(positions).all():
        raise ValueError('Session joint_positions_rad must be six finite values')
    degrees = record.get('joint_positions_deg')
    if degrees is not None:
        degrees = np.asarray(degrees, dtype=float)
        if degrees.shape != positions.shape or not np.allclose(np.deg2rad(degrees), positions,
                                                               atol=1e-10, rtol=0):
            raise ValueError('Session degree and radian joint observations disagree')
    fk_path = root / 'fk_report.json'
    if fk_path.is_file():
        fk_record = json.loads(fk_path.read_text(encoding='utf-8'))
        fk_positions = np.asarray(fk_record.get('joint_positions_rad'), dtype=float)
        if fk_positions.shape != positions.shape or not np.allclose(fk_positions, positions,
                                                                      atol=1e-10, rtol=0):
            raise ValueError('Session joint observation disagrees with fk_report.json')
    return SessionRobotObservation(root, names, positions, record)


@dataclass(frozen=True)
class SessionRegion:
    """A prepared region plus its source-frame transform and provenance."""

    root: Path
    jaw: str
    region: str
    prepared: PreparedRegion
    controller_base_from_scan: np.ndarray
    base_points_m: np.ndarray
    base_normals: np.ndarray
    base_tangents: np.ndarray
    metadata: dict

    def with_world_base(self, world_from_controller_base):
        """Return a planning-ready region using the requested simulation world."""
        world_from_controller_base = validate_transform(world_from_controller_base)
        world_from_scan = validate_transform(
            world_from_controller_base @ self.controller_base_from_scan)
        return replace(self.prepared, world_from_scan=world_from_scan)


def _jaw_transform_name(jaw):
    if jaw == 'mandibular':
        return 'T_controller_base_scan_nominal_m.npy'
    if jaw == 'maxillary':
        return 'T_controller_base_scan_nominal_rigid_m.npy'
    raise ValueError(f'Unknown jaw: {jaw}')


def _project_rigid(transform):
    """Remove float round-off from a homogeneous transform rotation."""
    value = np.asarray(transform, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all() or not np.allclose(
            value[3], [0., 0., 0., 1.], atol=1e-8, rtol=0):
        raise ValueError('Expected a finite homogeneous transform')
    u, _, vt = np.linalg.svd(value[:3, :3])
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        u[:, -1] *= -1
        rotation = u @ vt
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = value[:3, 3]
    return validate_transform(result)


def _checked_sources(root, entries):
    sources = {}
    for name, entry in entries.items():
        path = manifest_path(entry['path'], root, 'manifest')
        actual = fingerprint(path)
        if actual['sha256'] != entry['sha256']:
            raise ValueError(f'Session source fingerprint changed: {name}')
        sources[name] = actual
    return sources


def load_session_scan_transform_chain(session_root, jaw, *,
                                      camera_frame=DEFAULT_SESSION_CAMERA_FRAME):
    """Recompute and audit ``controller_base <- scan`` from session artifacts.

    Always audit the published stream-frame chain first. ``color_optical``
    explicitly reinterprets the same hand-eye numbers as wrist_3 <- color optical
    and uses SDK optical-to-optical extrinsics. It is an unverified assumption.
    """
    root = Path(session_root).resolve()
    if jaw not in ('mandibular', 'maxillary'):
        raise ValueError(f'Unknown jaw: {jaw}')
    if camera_frame not in CAMERA_FRAMES:
        raise ValueError(f'Unknown session camera frame: {camera_frame}')
    manifest_path_ = root / 'planning_input_manifest.json'
    manifest = json.loads(manifest_path_.read_text(encoding='utf-8'))
    source_files = _checked_sources(root, {
        **manifest['calibration_sources'], **manifest['shared_transforms'],
        **manifest['jaws'][jaw]['shared_artifacts'],
    })
    source_files['planning_input_manifest'] = fingerprint(manifest_path_)
    paths = {
        'controller_base_from_wrist3': root / 'transforms' / 'T_controller_base_wrist3_nominal_m.npy',
        'wrist3_from_color_frame': root / 'transforms' / 'T_wrist3_color_frame_m.npy',
        'color_frame_from_depth_optical': root / 'transforms' / 'T_color_frame_depth_optical_ros_convention_m.npy',
        'depth_optical_from_scan': root / jaw / 'T_depth_optical_scan_m.npy',
    }
    matrices = {}
    for name, path in paths.items():
        if not path.is_file():
            raise FileNotFoundError(path)
        matrices[name] = np.asarray(np.load(path, allow_pickle=False), dtype=float)
        validate_transform(matrices[name])
        source_files[name] = fingerprint(path)
    raw = matrices['controller_base_from_wrist3'] @ matrices['wrist3_from_color_frame']
    raw = raw @ matrices['color_frame_from_depth_optical'] @ matrices['depth_optical_from_scan']
    computed = _project_rigid(raw)
    published_path = root / jaw / _jaw_transform_name(jaw)
    if not published_path.is_file():
        raise FileNotFoundError(published_path)
    published = validate_transform(np.load(published_path, allow_pickle=False))
    if not np.allclose(computed, published, atol=5e-7, rtol=0):
        raise ValueError(f'{jaw} calibration/registration chain differs from published transform')
    source_files['published_transform'] = fingerprint(published_path)
    effective = published
    effective_matrices = dict(matrices)
    if camera_frame == 'color_optical':
        extrinsics = json.loads(Path(source_files['camera_extrinsics.json']['path']).read_text(
            encoding='utf-8'))
        if (extrinsics.get('translation_unit') != 'm' or
                extrinsics.get('matrix_convention') != 'p_target = R @ p_source + t'):
            raise ValueError('SDK extrinsics must use metres and target-from-source matrices')
        sdk = validate_transform(extrinsics['T_color_optical_depth_optical_m'])
        sdk_raw = extrinsics['raw_sdk_extrinsics']
        raw_sdk = np.eye(4)
        raw_sdk[:3, :3] = np.asarray(sdk_raw['rotation_column_major']).reshape(3, 3, order='F')
        raw_sdk[:3, 3] = sdk_raw['translation_m']
        if not np.allclose(sdk, raw_sdk, atol=1e-12, rtol=0):
            raise ValueError('SDK optical matrix differs from raw SDK extrinsics')
        effective_matrices = {
            'controller_base_from_wrist3': matrices['controller_base_from_wrist3'],
            'wrist3_from_color_optical': matrices['wrist3_from_color_frame'],
            'color_optical_from_depth_optical': sdk,
            'depth_optical_from_scan': matrices['depth_optical_from_scan'],
        }
        effective = _project_rigid(matrices['controller_base_from_wrist3'] @
                                   matrices['wrist3_from_color_frame'] @ sdk @
                                   matrices['depth_optical_from_scan'])
    return {
        'computed_controller_base_from_scan': computed,
        'published_controller_base_from_scan': published,
        'matrices': matrices,
        'published_file': published_path,
        'effective_controller_base_from_scan': effective,
        'effective_matrices': effective_matrices,
        'camera_frame': camera_frame,
        'camera_frame_version': CAMERA_FRAME_VERSION,
        'interpretation_status': ('optical_frame_hypothesis_live_tf_unverified'
                                  if camera_frame == 'color_optical' else 'archived_declared_frames'),
        'source_files': source_files,
    }


def load_session_scan_transform(session_root, jaw, *, camera_frame=DEFAULT_SESSION_CAMERA_FRAME):
    """Load the effective scan-to-controller_base transform, in metres."""
    return load_session_scan_transform_chain(
        session_root, jaw, camera_frame=camera_frame)['effective_controller_base_from_scan']


def _read_points(path):
    points = np.loadtxt(path, delimiter=',', comments='#')
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError(f'Expected finite N x 3 points: {path}')
    return points


def load_session_region(session_root, jaw='mandibular', region='outer_front', *,
                        camera_frame=DEFAULT_SESSION_CAMERA_FRAME):
    """Load one wyh session region and validate its frame/unit contract."""
    root = Path(session_root).resolve()
    if not root.is_dir():
        raise FileNotFoundError(root)
    if region not in REGIONS:
        raise ValueError(f'Unknown region: {region}')
    manifest = root / jaw / 'regions' / 'perception_regions.json'
    if not manifest.is_file():
        raise FileNotFoundError(manifest)
    manifest_record = json.loads(manifest.read_text(encoding='utf-8'))
    if manifest_record.get('coordinate_frame') not in (None, 'controller_base'):
        raise ValueError('Session regions must use controller_base coordinates')
    if manifest_record.get('position_unit') not in (None, 'm'):
        raise ValueError('Session region positions must be in metres')
    prepared = PreparedRegion.load_from_manifest(manifest, region)
    if prepared.inputs.jaw != jaw:
        raise ValueError('Session jaw and prepared region jaw differ')

    chain = load_session_scan_transform_chain(root, jaw, camera_frame=camera_frame)
    controller_base_from_scan = chain['published_controller_base_from_scan']
    if prepared.world_from_scan is not None and not np.allclose(
            prepared.world_from_scan, controller_base_from_scan, atol=1e-9, rtol=0):
        raise ValueError('Session region transform differs from jaw transform artifact')

    region_dir = root / jaw / 'regions' / region
    base_points = _read_points(region_dir / 'points_controller_base_nominal_m.csv')
    with np.load(region_dir / 'region_controller_base_nominal.npz', allow_pickle=False) as arrays:
        required = {'points_m', 'normals', 'tangents', 'parameters_m', 'T_controller_base_scan_m'}
        missing = required - set(arrays.files)
        if missing:
            raise ValueError(f'Missing base geometry arrays: {sorted(missing)}')
        geometry_points = np.asarray(arrays['points_m'], dtype=float)
        normals = np.asarray(arrays['normals'], dtype=float)
        tangents = np.asarray(arrays['tangents'], dtype=float)
        parameters = np.asarray(arrays['parameters_m'], dtype=float)
        geometry_transform = validate_transform(arrays['T_controller_base_scan_m'])
    if (geometry_points.shape != base_points.shape or normals.shape != base_points.shape
            or tangents.shape != base_points.shape):
        raise ValueError('Session base point, normal and tangent arrays disagree')
    if not np.isfinite(normals).all() or not np.isfinite(tangents).all():
        raise ValueError('Session base normals/tangents contain non-finite values')

    metadata = json.loads((region_dir / 'base_coordinates.json').read_text(encoding='utf-8'))
    if metadata.get('frame') != 'controller_base' or metadata.get('position_unit') != 'm':
        raise ValueError('Session base coordinate metadata is inconsistent')
    if int(metadata.get('points', len(base_points))) != len(base_points):
        raise ValueError('Session base point count does not match metadata')
    if not np.allclose(base_points, geometry_points, atol=2e-9, rtol=0):
        raise ValueError('CSV and NPZ controller_base points differ')
    if not np.allclose(geometry_transform, controller_base_from_scan, atol=5e-7, rtol=0):
        raise ValueError('Base geometry transform differs from published transform')
    with np.load(region_dir / 'continuous_gingival_line.npz', allow_pickle=False) as scan:
        if (parameters.shape != (len(base_points),) or
                not np.allclose(parameters, scan['parameters_m'], atol=1e-12, rtol=0)):
            raise ValueError('Base and scan curve parameters differ')
        expected_points = (scan['points_scan_m'] @ controller_base_from_scan[:3, :3].T +
                           controller_base_from_scan[:3, 3])
        expected_normals = scan['normals_scan'] @ controller_base_from_scan[:3, :3].T
    if not np.allclose(base_points, expected_points, atol=2e-9, rtol=0):
        raise ValueError('Published base points differ from transformed scan curve')
    if not np.allclose(normals, expected_normals, atol=2e-7, rtol=0):
        raise ValueError('Published base normals differ from transformed scan normals')
    manifest_record = json.loads((root / 'planning_input_manifest.json').read_text(encoding='utf-8'))
    region_sources = _checked_sources(root, manifest_record['jaws'][jaw]['regions'][region]['files'])
    effective = chain['effective_controller_base_from_scan']
    if camera_frame != 'declared':
        delta = effective @ np.linalg.inv(controller_base_from_scan)
        base_points = base_points @ delta[:3, :3].T + delta[:3, 3]
        normals = normals @ delta[:3, :3].T
        tangents = tangents @ delta[:3, :3].T
        prepared = replace(prepared, world_from_scan=effective)
    metadata = {**metadata, 'camera_frame': camera_frame,
                'camera_frame_version': CAMERA_FRAME_VERSION,
                'interpretation_status': chain['interpretation_status'],
                'published_controller_base_from_scan': controller_base_from_scan,
                'effective_controller_base_from_scan': effective,
                'source_files': {**chain['source_files'], **region_sources}}
    return SessionRegion(root, jaw, region, prepared, effective,
                         base_points, normals, tangents, metadata)
