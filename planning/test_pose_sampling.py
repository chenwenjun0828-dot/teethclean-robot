"""Pose density can change without changing geometry or the motion algorithm."""
from contextlib import redirect_stdout
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from planning.config import Config, SweepOptions
from planning.test_sweep import build_reference
from planning.pose_search.continuous import sample_sweep
from planning.test_sweep import straight_geometry


class PoseSamplingTests(unittest.TestCase):
    def test_denser_pose_sampling_preserves_geometry_and_part_boundaries(self):
        cfg = Config()
        coarse = replace(cfg, pose_spacing_mm=.25)
        geometry = straight_geometry()
        path = build_reference(geometry, cfg, SweepOptions(), 1.)
        other = build_reference(geometry, coarse, SweepOptions(), 1.)
        for s in np.linspace(0, path.end, 51):
            np.testing.assert_array_equal(path.evaluate(s), other.evaluate(s))
            np.testing.assert_array_equal(path.normal(s), other.normal(s))
        dense_nodes, old_nodes = sample_sweep(path, cfg), sample_sweep(other, coarse)
        self.assertGreater(len(dense_nodes), 3*len(old_nodes))
        self.assertEqual(cfg.spacing_mm, coarse.spacing_mm)
        self.assertEqual(cfg.spacing_mm, .25)
        for part in path.parts:
            self.assertIn(part['start'], dense_nodes)
            self.assertIn(part['end'], dense_nodes)
        np.testing.assert_array_equal(dense_nodes[[0, -1]], old_nodes[[0, -1]])

    def test_saved_configs_preserve_their_original_pose_density(self):
        current = asdict(Config())
        self.assertEqual(Config.from_saved(current).pose_spacing_mm, .05)
        for value in (.25, .1):
            historical = {k: v for k, v in current.items() if k != 'pose_spacing_mm'}
            historical['spacing_mm'] = value
            self.assertEqual(Config.from_saved(historical).pose_spacing_mm, value)

    def test_both_cli_entries_and_tracking_forward_independent_spacing(self):
        from planning.__main__ import main as planning_main
        from pipeline.run import main as pipeline_main, run_pipeline
        output = io.StringIO()
        with redirect_stdout(output):
            self.assertEqual(planning_main(['--dry-run', '--pose-spacing-mm', '.025']), 0)
        cfg = json.loads(output.getvalue())['config']
        self.assertEqual((cfg['pose_spacing_mm'], cfg['spacing_mm']), (.025, .25))
        with redirect_stdout(io.StringIO()), patch('pipeline.run.run_pipeline', return_value={'status': 'dry_run'}) as run:
            self.assertEqual(pipeline_main(['--dry-run', '--pose-spacing-mm', '.025']), 0)
            self.assertEqual(run.call_args.kwargs['pose_spacing_mm'], .025)
        self.assertEqual(run_pipeline(dry_run=True)['pose_spacing_mm'], .05)

    def test_invalid_spacing_is_rejected_before_pipeline_creates_scene_or_output(self):
        from pipeline.run import run_pipeline
        with tempfile.TemporaryDirectory() as temp, patch('pipeline.run.create_simulation') as scene:
            output = Path(temp)/'run'
            for value in (0, -1, float('nan'), float('inf'), True):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    run_pipeline(output_dir=output, pose_spacing_mm=value)
                self.assertFalse(output.exists())
            scene.assert_not_called()


if __name__ == '__main__':
    unittest.main()
