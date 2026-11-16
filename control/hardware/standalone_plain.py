"""Paired plain-point input; source bundle verification happens at build time."""
import copy
import hashlib
import json
from pathlib import Path

from .trajectory_contract import JointTrajectory, sha256, unique_json_object
from .ros_runner import authorization_errors, deployment_errors


PLAIN_CONTRACT = 'ur7e_plain_points_v1'
POINT_FIELDS = ('positions', 'velocities', 'accelerations', 'time_from_start')
SOURCE_HASH_FIELDS = ('trajectory_sha256', 'manifest_sha256', 'export_report_sha256')


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode('utf-8')).hexdigest()


class PlainTrajectoryInput:
    """Keep runtime binding checks distinct from original bundle hash checks."""
    def __init__(self, path, config, reference, reference_digest, script_path=None):
        if not isinstance(config, dict) or not isinstance(reference, dict):
            raise ValueError('Plain trajectory requires its paired script configuration and reference')
        self.path = Path(path).resolve()
        self.config = config
        self.reference = reference
        self.reference_digest = reference_digest
        self.script_path = Path(script_path).resolve() if script_path else None
        self.script_digest = sha256(self.script_path) if self.script_path else None
        self.digest = sha256(self.path)
        self.config_digest = canonical_sha256(config)
        self.fingerprints = {
            'trajectory_sha256': self.digest,
            'deployment_sha256': self.config_digest,
            'reference_sha256': reference_digest,
            'input_contract': PLAIN_CONTRACT,
            'source_validation': 'original_bundle_verified_at_build_time',
            'runtime_validation': 'paired_plain_trajectory_and_script_reference',
            'source_fingerprints': reference.get('source_fingerprints'),
        }
        if self.script_digest:
            self.fingerprints['script_sha256'] = self.script_digest

    def unchanged(self):
        if (sha256(self.path) != self.digest
                or canonical_sha256(self.config) != self.config_digest
                or canonical_sha256(self.reference) != self.reference_digest
                or (self.script_path and sha256(self.script_path) != self.script_digest)):
            raise ValueError('plain_input_or_paired_script_changed_before_send')

    def load(self):
        self.unchanged()
        reference = self.reference
        if (reference.get('input_contract') != PLAIN_CONTRACT
                or reference.get('plain_trajectory_sha256') != self.digest):
            raise ValueError('Plain trajectory does not match its paired script reference')
        payload = json.loads(self.path.read_bytes().decode('utf-8'),
                             object_pairs_hook=unique_json_object)
        if not isinstance(payload, dict) or set(payload) != {'joint_names', 'points'}:
            raise ValueError('Plain JSON must contain only joint_names and points')
        points = payload['points']
        if (not isinstance(points, list) or len(points) != reference['points']
                or len(points) != len(reference['tcp_poses'])
                or any(not isinstance(p, dict) or set(p) != set(POINT_FIELDS) for p in points)
                or payload['joint_names'] != reference['record']['joint_names']):
            raise ValueError('Plain point count, joint order or fields do not match reference')
        record = copy.deepcopy(reference['record'])
        record['points'] = [dict(p, tcp_pose=tcp) for p, tcp in zip(points, reference['tcp_poses'])]
        report = copy.deepcopy(reference['export_report'])
        if (record['source'] != report['source']
                or report['trajectory_sha256'] != reference['source_fingerprints']['trajectory_sha256']
                or report['manifest_sha256'] != reference['source_fingerprints']['manifest_sha256']):
            raise ValueError('Paired reference source facts differ')
        curve = JointTrajectory(record)
        self.unchanged()
        return curve, report, self.config

    def deployment_errors(self, config, curve, report, *, require_authorization=True):
        missing = deployment_errors(config, self.path, curve, report, require_authorization=False)
        # Original hashes are retained as source history, never compared with new bytes.
        authorization = config.get('authorization', {})
        for key in SOURCE_HASH_FIELDS:
            if authorization.get(key) != self.reference['source_fingerprints'][key]:
                missing.append('authorization.source_history.'+key)
        if require_authorization:
            expected = {'input_contract': PLAIN_CONTRACT, 'plain_trajectory_sha256': self.digest,
                        'reference_sha256': self.reference_digest}
            for key, value in expected.items():
                if authorization.get(key) != value:
                    missing.append('authorization.'+key)
            missing += authorization_errors(authorization)
        return missing
