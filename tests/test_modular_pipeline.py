"""Cross-module checks for the colored-scan gingival pipeline."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

from perception.frames import scan_to_camera_true_mm
from perception.gingival_extraction import extract_gingival_line
from perception.gingival_line import read_topology
from perception.registration.registration_math import transform_points
from pipeline.run import run_pipeline


ROOT = Path(__file__).resolve().parents[1]


class ModularPipelineTests(unittest.TestCase):
    def test_dry_run_validates_inputs_without_creating_output(self):
        result = run_pipeline(dry_run=True)
        self.assertEqual(result['status'], 'dry_run')

    def test_extraction_matches_existing_case_outputs_without_crosstooth(self):
        for jaw in ('mandibular', 'maxillary'):
            with self.subTest(jaw=jaw), tempfile.TemporaryDirectory() as tmp:
                scan = ROOT / 'assets' / 'source_cad' / 'teeth' / f'{jaw}_scan.ply'
                result = extract_gingival_line(scan, Path(tmp), jaw, surface='all', cleanup=False)
                expected = ROOT / 'assets' / 'perception' / jaw / 'gingival_line_segments.obj'
                points, edges = read_topology(result['topology_obj'])
                reference, reference_edges = read_topology(expected)
                np.testing.assert_array_equal(points, reference)
                self.assertEqual(edges, reference_edges)
                self.assertEqual(len(result['points_mm']), len(points))

    def test_default_extraction_saves_filtered_outer_topology_for_both_jaws(self):
        for jaw in ('mandibular', 'maxillary'):
            with self.subTest(jaw=jaw), tempfile.TemporaryDirectory() as tmp:
                scan = ROOT / 'assets' / 'source_cad' / 'teeth' / f'{jaw}_scan.ply'
                result = extract_gingival_line(scan, Path(tmp), jaw)
                points, edges = read_topology(result['topology_obj'])
                reference, _ = read_topology(ROOT / 'assets' / 'perception' / jaw / 'gingival_line_segments.obj')
                self.assertEqual(result['surface'], 'outer')
                self.assertGreater(len(points), len(reference)*.2)
                self.assertLess(len(points), len(reference)*.75)
                np.testing.assert_allclose(points, result['points_mm'], atol=1e-6)
                self.assertEqual(edges, result['edges'])
                report = json.loads((Path(tmp) / 'gingival_surface_filter.json').read_text())
                self.assertGreater(report['removed_segments'], 0)
                self.assertEqual(report['surface'], 'outer')
                cleanup = json.loads((Path(tmp) / 'gingival_cleanup.json').read_text())
                self.assertTrue(cleanup['enabled'])
                reasons = {region['reason'] for region in cleanup['removed_tooth_regions']}
                self.assertIn('scan_boundary_color_strip', reasons)
                self.assertIn('small_tooth_color_island', reasons)

    def test_failure_before_registration_never_starts_control(self):
        with tempfile.TemporaryDirectory() as tmp, \
                patch('perception.workflow.extract_gingival_line', side_effect=ValueError('no line')), \
                patch('pipeline.run.run_tracking') as control:
            with self.assertRaisesRegex(ValueError, 'no line'):
                run_pipeline(output_dir=Path(tmp) / 'run')
            control.assert_not_called()
            report = json.loads((Path(tmp) / 'run' / 'pipeline_report.json').read_text())
            self.assertEqual(report['status'], 'failed')
            self.assertEqual(report['stage'], 'extraction')



if __name__ == '__main__':
    unittest.main()
