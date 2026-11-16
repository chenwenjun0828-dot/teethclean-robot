"""Build a MuJoCo scene from a checked-in real-camera perception session."""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from perception.session import (CAMERA_FRAME_VERSION, DEFAULT_SESSION_CAMERA_FRAME,
                                load_session_region, load_session_robot_observation,
                                load_session_scan_transform_chain)
from planning.infrastructure.io import fingerprint, validate_transform
from pipeline.simulation import (DENTAL_MODEL_PATH, apply_dental_transforms,
                                 body_transform, create_simulation)


FRAME_ADAPTER_VERSION = 'ur_base_to_controller_base_rz_pi_v1'


def _rz_pi():
    return np.diag([-1., -1., 1., 1.])


def _set_capture_joint_positions(model, data, joint_positions_rad):
    """Set the six UR joints to the session capture observation."""
    names = ('ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
             'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint')
    positions = np.asarray(joint_positions_rad, dtype=float)
    if positions.shape != (6,) or not np.isfinite(positions).all():
        raise ValueError('Capture joint positions must be six finite radians')
    joint_ids = np.array([model.joint(name).id for name in names], dtype=int)
    lower, upper = model.jnt_range[joint_ids].T
    if np.any(positions < lower) or np.any(positions > upper):
        raise ValueError('Capture joint positions are outside the MuJoCo joint limits')
    qadr = model.jnt_qposadr[joint_ids]
    data.qpos[qadr] = positions
    data.qvel[:] = 0.
    mujoco.mj_forward(model, data)


def _session_record(session_region, world_from_controller_base, world_tooth_transforms,
                    observation, scan_chains):
    root = session_region.root
    manifest = root / 'planning_input_manifest.json'
    if not manifest.is_file():
        raise FileNotFoundError(manifest)
    return {
        'source': 'wyh_session',
        'session_id': root.name,
        'session_root': root,
        'manifest_file': fingerprint(manifest),
        'jaw': session_region.jaw,
        'region': session_region.region,
        'coordinate_frame': 'controller_base',
        'position_unit': 'm',
        'frame_adapter_version': FRAME_ADAPTER_VERSION,
        'camera_frame': session_region.metadata['camera_frame'],
        'camera_frame_version': CAMERA_FRAME_VERSION,
        'camera_frame_status': session_region.metadata['interpretation_status'],
        'T_ur_base_controller_base_m': _rz_pi(),
        'T_world_controller_base_m': world_from_controller_base,
        'robot_joint_names': observation.joint_names,
        'robot_joint_positions_rad': observation.joint_positions_rad,
        'robot_joint_pose_source': observation.metadata.get('source'),
        'jaw_scan_transforms_controller_base_m': {
            jaw: scan_chains[jaw]['effective_controller_base_from_scan']
            for jaw in ('mandibular', 'maxillary')
        },
        'jaw_scan_transforms_published_controller_base_m': {
            jaw: scan_chains[jaw]['published_controller_base_from_scan']
            for jaw in ('mandibular', 'maxillary')
        },
        'jaw_scan_transform_chain_m': {
            jaw: scan_chains[jaw]['computed_controller_base_from_scan']
            for jaw in ('mandibular', 'maxillary')
        },
        'jaw_effective_transform_chain_m': {
            jaw: scan_chains[jaw]['effective_matrices']
            for jaw in ('mandibular', 'maxillary')
        },
        'jaw_published_transform_chain_m': {
            jaw: scan_chains[jaw]['matrices']
            for jaw in ('mandibular', 'maxillary')
        },
        'source_files': {
            **session_region.metadata['source_files'],
            **{f'{jaw}/{name}': value for jaw, chain in scan_chains.items()
               for name, value in chain['source_files'].items()},
        },
        'jaw_transforms_world_m': world_tooth_transforms,
        'robot_execution_allowed': False,
        'region_boundary_status': 'session_artifact',
    }


def build_session_simulation(session_root, jaw='mandibular', region='outer_front',
                             *, jaw_settings=None, robot_mount_position_m=None,
                             camera_frame=DEFAULT_SESSION_CAMERA_FRAME):
    """Load a session region and place both effective jaw poses in MuJoCo.

    The session publishes scan-to-controller_base poses.  MuJoCo ``ur_base``
    follows the ROS ``base_link`` axes, so controller_base is attached with a
    fixed Rz(pi) adapter before applying the scan poses.  The robot is placed
    at the capture-time joint observation for scene inspection and planning.
    """
    observation = load_session_robot_observation(session_root)
    scan_chains = {
        name: load_session_scan_transform_chain(session_root, name, camera_frame=camera_frame)
        for name in ('mandibular', 'maxillary')
    }
    session_region = load_session_region(session_root, jaw, region, camera_frame=camera_frame)
    model, data = create_simulation(
        model_path=DENTAL_MODEL_PATH,
        jaw_settings=jaw_settings,
        robot_mount_position_m=robot_mount_position_m,
    )
    _set_capture_joint_positions(model, data, observation.joint_positions_rad)
    world_from_ur_base = body_transform(model, data, 'ur_base')
    world_from_controller_base = validate_transform(world_from_ur_base @ _rz_pi())
    world_tooth_transforms = {
        name: validate_transform(world_from_controller_base @
                                 scan_chains[name]['effective_controller_base_from_scan'])
        for name in ('mandibular', 'maxillary')
    }
    apply_dental_transforms(model, data, world_tooth_transforms)
    mujoco.mj_forward(model, data)
    return (session_region, model, data, world_from_controller_base,
            world_tooth_transforms, _session_record(
                session_region, world_from_controller_base, world_tooth_transforms,
                observation, scan_chains))


def rebuild_session_simulation(session_info, *, jaw_settings=None,
                               robot_mount_position_m=None):
    """Rebuild a saved session scene and reject a changed controller-base anchor."""
    if session_info.get('frame_adapter_version') != FRAME_ADAPTER_VERSION:
        raise ValueError('Planning output uses an obsolete session base-frame adapter; replan it')
    version = session_info.get('camera_frame_version')
    if version is None:
        if session_info.get('camera_frame', 'declared') != 'declared':
            raise ValueError('Legacy session output cannot select a camera frame interpretation')
        camera_frame = 'declared'
    else:
        if version != CAMERA_FRAME_VERSION:
            raise ValueError('Unsupported session camera frame interpretation version')
        camera_frame = session_info['camera_frame']
    root = Path(session_info['session_root'])
    result = build_session_simulation(
        root, session_info['jaw'], session_info['region'],
        jaw_settings=jaw_settings, robot_mount_position_m=robot_mount_position_m,
        camera_frame=camera_frame)
    _, _, _, world_from_controller_base, _, rebuilt_info = result
    saved = validate_transform(session_info['T_world_controller_base_m'])
    if not np.allclose(world_from_controller_base, saved, atol=1e-10, rtol=0):
        raise ValueError('controller_base anchor changed since session planning')
    if not np.allclose(rebuilt_info['robot_joint_positions_rad'],
                       session_info.get('robot_joint_positions_rad'), atol=1e-10, rtol=0):
        raise ValueError('Capture joint observation changed since session planning')
    for key in ('jaw_scan_transforms_controller_base_m', 'jaw_transforms_world_m',
                'jaw_scan_transforms_published_controller_base_m', 'jaw_scan_transform_chain_m',
                'jaw_effective_transform_chain_m', 'jaw_published_transform_chain_m'):
        if version is None and key not in session_info:
            continue
        saved_matrices = session_info.get(key, {})
        actual_matrices = rebuilt_info[key]
        if saved_matrices.keys() != actual_matrices.keys():
            raise ValueError(f'Session transform record changed: {key}')
        for jaw, actual in actual_matrices.items():
            if isinstance(actual, dict) and (not isinstance(saved_matrices[jaw], dict) or
                                             saved_matrices[jaw].keys() != actual.keys()):
                raise ValueError(f'Session transform chain changed: {key}/{jaw}')
            pairs = ([(saved_matrices[jaw].get(name), value) for name, value in actual.items()]
                     if isinstance(actual, dict) else [(saved_matrices[jaw], actual)])
            for saved_value, actual_value in pairs:
                saved_value = np.asarray(saved_value, dtype=float)
                if saved_value.shape != (4, 4) or not np.allclose(
                        saved_value, actual_value, atol=1e-10, rtol=0):
                    raise ValueError(f'Session transform changed: {key}/{jaw}')
    if version is not None:
        if session_info.get('source_files') != rebuilt_info['source_files']:
            raise ValueError('Session source fingerprints changed since planning')
        if session_info.get('camera_frame_status') != rebuilt_info['camera_frame_status']:
            raise ValueError('Session camera frame status changed since planning')
    return result
