"""Perception owns curve construction; planning only consumes frozen regions."""
import ast
from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import asdict
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from perception.regional_paths import load_regional_paths, scan_points_to_world_m
from perception.regions import PreparedRegion
from perception.regions.config import REGION_ORDER
from perception.regions.io import Inputs, ROOT, fingerprint
from planning.config import Config, SweepOptions
from planning.pipeline import run
from planning.simple import orient_reference
from planning.test_sweep import prepared_region


class RegionalBoundaryTests(unittest.TestCase):
    def test_default_is_regional_fixed_and_historical_mode_is_preserved(self):
        from pipeline.run import run_pipeline
        self.assertEqual(Config().planning_mode, 'simple')
        self.assertEqual(run_pipeline(dry_run=True)['planning_mode'], 'simple')
        historical = asdict(Config())
        historical.pop('planning_mode')
        self.assertEqual(Config.from_saved(historical).planning_mode, 'continuous')

    def test_perception_geometry_does_not_import_planning(self):
        for path in (ROOT/'perception/regions').glob('*.py'):
            tree = ast.parse(path.read_text(encoding='utf-8'))
            for node in ast.walk(tree):
                modules = ([node.module or ''] if isinstance(node, ast.ImportFrom) else
                           [alias.name for alias in node.names] if isinstance(node, ast.Import) else [])
                self.assertFalse(any(name.startswith(('planning', 'control')) for name in modules), path.name)

    def test_planner_rejects_raw_inputs_before_writing_any_output(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'plan'
            with self.assertRaisesRegex(TypeError, 'PreparedRegion'):
                run(Inputs.defaults('mandibular'), output)
            self.assertFalse(output.exists())

    def test_planner_exports_supplied_geometry_without_any_reconstruction(self):
        prepared = prepared_region()
        with tempfile.TemporaryDirectory() as temp, redirect_stdout(io.StringIO()), \
                patch('perception.regions.frozen.build_regional_geometry', side_effect=AssertionError('refit')), \
                patch('perception.regions.frozen.continuous_regional_reference', side_effect=AssertionError('reconnect')), \
                patch('perception.regions.geometry.fit_curve', side_effect=AssertionError('refit')):
            output = Path(temp)/'plan'
            geometry, trajectory, report, scene = run(prepared, output, geometry_only=True)
            self.assertIsNone(trajectory)
            self.assertIsNone(scene)
            self.assertFalse(report['execution_ready'])
            saved = json.loads((output/'perception_reference.json').read_text(encoding='utf-8'))
            np.testing.assert_array_equal(saved['parts'][0]['coefficients_m'],
                                          prepared.reference.parts[0]['coefficients_m'])
            request = json.loads((output/'request.json').read_text(encoding='utf-8'))
            self.assertEqual(request['geometry_owner'], 'perception')
            self.assertEqual(geometry.report['config']['planning_mode'], 'simple')
            self.assertEqual(prepared.geometry.report['config']['planning_mode'], 'continuous')

    def test_orientation_policy_cannot_change_the_perception_line(self):
        prepared = prepared_region()
        before = deepcopy(prepared.reference.record)
        path = orient_reference(prepared.reference, prepared.geometry, Config(), SweepOptions())
        for station in np.linspace(0, path.end, 101):
            np.testing.assert_array_equal(path.evaluate(station), prepared.reference.evaluate(station))
            np.testing.assert_array_equal(path.normal(station), prepared.reference.normal(station))
            np.testing.assert_allclose(path.rotation(station), path.rotation(0), atol=1e-12)
        self.assertNotIn('regional_orientation', prepared.reference.record)
        self.assertEqual(before.keys(), prepared.reference.record.keys())

    def test_prepared_region_roundtrip_and_tamper_rejection(self):
        prepared = prepared_region()
        with tempfile.TemporaryDirectory() as temp:
            manifest = prepared.save(temp)
            restored = PreparedRegion.load(manifest)
            self.assertEqual(restored.region, 'outer_front')
            for station in np.linspace(0, restored.reference.end, 31):
                np.testing.assert_allclose(restored.reference.evaluate(station), prepared.reference.evaluate(station))
            reference_file = Path(temp)/'continuous_gingival_line.json'
            reference_file.write_text('{}', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'artifact changed'):
                PreparedRegion.load(manifest)

    def test_source_change_invalidates_a_prepared_region(self):
        prepared = prepared_region()
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp)/'source.txt'
            source.write_text('original', encoding='utf-8')
            prepared.geometry.report['inputs'] = {'source': fingerprint(source)}
            prepared.validate()
            source.write_text('changed', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'source changed'):
                prepared.validate()

    def test_region_mismatch_is_rejected_before_creating_output(self):
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp)/'plan'
            with self.assertRaisesRegex(ValueError, 'region differs'):
                run(prepared_region(), output, Config(region='outer_left'))
            self.assertFalse(output.exists())

    def test_frozen_dataset_contains_six_separate_directed_scan_chains(self):
        dataset = load_regional_paths()
        self.assertEqual(tuple(chain.region for chain in dataset.chains), REGION_ORDER)
        self.assertEqual(sum(len(chain.scan_points_mm) for chain in dataset.chains), 688)
        self.assertEqual(dataset.metadata['mesh_sha256'], fingerprint(Inputs.defaults(dataset.jaw).scan)['sha256'])
        world = np.eye(4)
        world[:3, 3] = [.1, .2, .3]
        for chain in dataset.chains:
            placed = scan_points_to_world_m(chain.scan_points_mm, world)
            np.testing.assert_allclose(placed, chain.scan_points_mm*.001+[.1, .2, .3])

    def test_retired_region_names_are_rejected(self):
        for region in ('left', 'middle', 'right'):
            with self.subTest(region=region), self.assertRaises(ValueError):
                Config(region=region)

    def test_missing_or_mismatched_dataset_fails_before_simulation(self):
        from pipeline.run import run_pipeline
        with patch('pipeline.run.create_simulation') as simulation:
            with self.assertRaisesRegex(FileNotFoundError, 'six-region dataset'):
                run_pipeline(jaw='maxillary', dry_run=True)
            with tempfile.TemporaryDirectory() as temp:
                with self.assertRaises(FileNotFoundError):
                    run_pipeline(prediction_dir=Path(temp), dry_run=True)
            with tempfile.TemporaryDirectory() as temp:
                scan = Path(temp)/'other.ply'
                scan.write_text('different scan', encoding='ascii')
                with self.assertRaisesRegex(ValueError, 'requested scan/jaw'):
                    run_pipeline(scan=scan, dry_run=True)
            simulation.assert_not_called()

    def test_replay_rejects_a_planning_curve_that_differs_from_perception(self):
        from perception.regions.io import dump
        from planning.motion_planning.contract import load_sweep_reference
        prepared = prepared_region()
        cfg, options = Config(), SweepOptions()
        path = orient_reference(prepared.reference, prepared.geometry, cfg, options)
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            dump(folder/'perception_reference.json', prepared.reference.record)
            dump(folder/'sweep_reference.json', path.record)
            report = {'geometry_owner': 'perception', 'planning_mode': 'simple', 'region': 'outer_front',
                      'config': asdict(cfg), 'sweep_options': asdict(options),
                      'perception_reference_file': fingerprint(folder/'perception_reference.json'),
                      'sweep_reference_file': fingerprint(folder/'sweep_reference.json')}
            load_sweep_reference(folder, report)
            record = json.loads((folder/'sweep_reference.json').read_text(encoding='utf-8'))
            record['parts'][0]['coefficients_m'][0][0] += .001
            dump(folder/'sweep_reference.json', record)
            report['sweep_reference_file'] = fingerprint(folder/'sweep_reference.json')
            with self.assertRaisesRegex(ValueError, 'differs from perception'):
                load_sweep_reference(folder, report)


class SixRegionGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from perception.regions import prepare_regions
        cls.regions = prepare_regions(Inputs.defaults('mandibular'), regional_dataset=load_regional_paths())

    def test_full_ordered_regions_keep_one_curve_across_tooth_boundaries(self):
        from scipy.interpolate import BSpline
        self.assertEqual(tuple(self.regions), REGION_ORDER)
        for name, prepared in self.regions.items():
            with self.subTest(region=name):
                path = prepared.reference
                self.assertTrue(all(part['kind'] == 'source' for part in path.parts))
                self.assertEqual(path.record['synthetic_bridge_length_mm'], 0.)
                self.assertEqual(path.record['trim_scope'], 'region_endpoints_only')
                self.assertAlmostEqual(path.record['trimmed_or_omitted_length_mm'], 2.)
                source = prepared.geometry.report['curves'][0]
                curve = BSpline(source['knots'], source['coefficients_m'], 3)
                for part in path.parts:
                    for f in np.linspace(0, 1, 5):
                        parameter = part['start']+f*(part['end']-part['start'])
                        np.testing.assert_allclose(path.evaluate(parameter), curve(part['u0']+f*(part['u1']-part['u0'])), atol=1e-12)
                for a, b in zip(path.parts[:-1], path.parts[1:]):
                    self.assertEqual(a['u1'], b['u0'])
                    np.testing.assert_allclose(a['normal1'], b['normal0'], atol=1e-12)
                    seam = a['end']
                    for derivative, tolerance in ((0, 1e-8), (1, 1e-5), (2, .2)):
                        np.testing.assert_allclose(path.evaluate(seam-1e-12, derivative),
                                                  path.evaluate(seam+1e-12, derivative), atol=tolerance)

    def test_six_region_artifact_roundtrip_retains_each_region_and_normals(self):
        with tempfile.TemporaryDirectory() as temp:
            for name, prepared in self.regions.items():
                manifest = prepared.save(Path(temp)/name)
                restored = PreparedRegion.load(manifest)
                self.assertEqual(restored.region, name)
                self.assertEqual(restored.inputs.jaw, 'mandibular')
                self.assertLessEqual(restored.geometry.report['source_surface_max_distance_mm'], .35)
                for s in np.linspace(0, restored.reference.end, 13):
                    np.testing.assert_allclose(restored.reference.normal(s), prepared.reference.normal(s))


if __name__ == '__main__':
    unittest.main()
