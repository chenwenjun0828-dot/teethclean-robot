"""Stable planning assets load without using the local outputs tree."""
import hashlib
import json
from pathlib import Path
import unittest

import numpy as np

from perception.regions import PreparedRegion

ROOT = Path(__file__).resolve().parents[1]
SESSION = ROOT / 'assets/perception/sessions/20261113_220209'


class PerceptionSessionAssetTests(unittest.TestCase):
    def test_twelve_regions_and_all_manifest_files_are_stable_assets(self):
        manifest = json.loads((SESSION / 'planning_input_manifest.json').read_text())
        entries = list(manifest['calibration_sources'].values()) + list(manifest['shared_transforms'].values())
        counts = {}
        for jaw, group in manifest['jaws'].items():
            entries.extend(group.get('shared_artifacts', {}).values())
            self.assertEqual(len(group['regions']), 6)
            counts[jaw] = 0
            for name, region in group['regions'].items():
                entries.extend(region['files'].values())
                folder = (SESSION / region['files']['prepared_region']['path']).parent
                prepared = PreparedRegion.load(folder)
                self.assertFalse((folder/'teeth').exists())
                self.assertFalse((folder/'region_mesh.npz').exists())
                self.assertFalse((folder/'tooth_face_ids.npy').exists())
                self.assertEqual(prepared.region, name)
                for source in prepared.geometry.report['inputs'].values():
                    self.assertTrue(Path(source['path']).resolve().is_relative_to(ROOT / 'assets'))
                points = np.loadtxt(SESSION / region['files']['points_csv']['path'], delimiter=',')
                with np.load(SESSION / region['files']['geometry_npz']['path'], allow_pickle=False) as data:
                    np.testing.assert_allclose(points, data['points_m'], atol=1e-12)
                    self.assertEqual(points.shape, data['normals'].shape)
                    self.assertEqual(points.shape, data['tangents'].shape)
                counts[jaw] += len(points)
        self.assertEqual(counts, {'mandibular': 2711, 'maxillary': 1984})
        for entry in entries:
            path = (SESSION / entry['path']).resolve()
            self.assertTrue(path.is_relative_to(ROOT / 'assets'), str(path))
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), entry['sha256'])
        self.assertFalse(manifest['robot_execution_allowed'])


if __name__ == '__main__':
    unittest.main()
