"""Validate a saved tooth-segmentation prediction against its source mesh."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prediction-dir', type=Path, required=True)
    parser.add_argument('--scan', type=Path, required=True)
    args = parser.parse_args(argv)

    import open3d as o3d
    import numpy as np
    from .algorithm import load_prediction

    mesh = o3d.io.read_triangle_mesh(str(args.scan))
    vertices = np.asarray(mesh.vertices)
    faces = np.asarray(mesh.triangles)
    labels, probabilities, summary = load_prediction(
        args.prediction_dir, args.scan, vertices, faces)
    result = {
        'status': 'valid',
        'faces': int(len(faces)),
        'classes': int(len(np.unique(labels[labels > 0]))),
        'probability_shape': list(probabilities.shape),
        'input_sha256': summary['input_sha256'],
    }
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
