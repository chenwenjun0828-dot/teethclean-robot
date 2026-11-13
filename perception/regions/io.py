"""Explicit inputs, units and reproducible exports for planning."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import os
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def dump(path, value):
    def convert(x):
        if isinstance(x, np.ndarray):
            return x.tolist()
        if isinstance(x, np.generic):
            return x.item()
        if isinstance(x, Path):
            return str(x.resolve())
        raise TypeError(type(x).__name__)
    Path(path).write_text(json.dumps(value, default=convert, indent=2,
                                    ensure_ascii=False, allow_nan=False), encoding='utf-8')


def fingerprint(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def relative_path(path, folder):
    """Persist source paths relative to the manifest, including outside-run assets."""
    return Path(os.path.relpath(Path(path).resolve(), Path(folder).resolve())).as_posix()


def manifest_path(value, folder, path_base=None):
    """New manifests use their own directory; legacy absolute paths remain supported."""
    if path_base not in (None, 'manifest'):
        raise ValueError(f'Unsupported perception path base: {path_base}')
    path = Path(value)
    if path_base == 'manifest' and not path.is_absolute():
        path = Path(folder) / path
    return path.resolve()


@dataclass
class Inputs:
    jaw: str
    scan: Path
    topology: Path
    prediction: Path

    @classmethod
    def defaults(cls, jaw):
        if jaw not in ('mandibular', 'maxillary'):
            raise ValueError('Unknown jaw')
        return cls(jaw, ROOT / f'assets/source_cad/teeth/{jaw}_scan.ply',
                   ROOT / f'assets/perception/{jaw}/gingival_line_segments.obj',
                   ROOT / f'assets/perception/tooth_segmentation/crosstooth_{jaw}')


def body_transform(model, data, jaw):
    body = model.body('dental_' + jaw).id
    transform = np.eye(4)
    transform[:3, :3] = data.xmat[body].reshape(3, 3)
    transform[:3, 3] = data.xpos[body]
    return transform


def validate_transform(value):
    value = np.asarray(value, float)
    if (value.shape != (4, 4) or not np.isfinite(value).all()
            or not np.allclose(value[3], [0, 0, 0, 1])
            or not np.allclose(value[:3, :3].T @ value[:3, :3], np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(value[:3, :3]), 1., atol=1e-8)):
        raise ValueError('Expected a rigid scan-to-world transform, translation in metres')
    return value

def unit(vector):
    vector = np.asarray(vector, float)
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError('A direction is zero or non-finite')
    return vector / length
