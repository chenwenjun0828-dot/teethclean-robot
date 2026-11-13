"""Moving a repository must not invalidate prepared region file references."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from perception.regions import PreparedRegion, prepare_region, prepare_regions
from perception.regions.io import Inputs, dump, fingerprint


class RegionPortabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.template = prepare_region(Inputs.defaults('mandibular'))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.project = Path(self.temp.name) / 'old-project'
        assets = self.project / 'assets'
        assets.mkdir(parents=True)
        for name in ('scan.ply', 'topology.obj', 'prediction.npz'):
            (assets / name).write_text(name)
        self.prepared = deepcopy(self.template)
        self.prepared.inputs = Inputs('mandibular', assets/'scan.ply', assets/'topology.obj', assets)
        self.prepared.geometry.report['inputs'] = {
            'scan': fingerprint(assets/'scan.ply'), 'prediction': fingerprint(assets/'prediction.npz')}
        self.prepared.geometry.report['source_metadata'] = {'dataset_dir': str(assets)}
        self.output = self.project / 'outputs/run/regions'

    def export(self):
        def prepare(*args, output=None, **kwargs):
            result = deepcopy(self.prepared)
            result.save(output)
            return result
        with patch('perception.regions.result.validate_region_inputs', return_value=object()), \
             patch('perception.regions.result.prepare_region', side_effect=prepare):
            prepare_regions(self.prepared.inputs, output=self.output, regions=['outer_front'])
        return self.output / 'outer_front/perception_region.json'

    def test_move_repository_and_load_from_other_working_directory(self):
        path = self.export()
        record = json.loads(path.read_text())
        report = json.loads((path.parent/'geometry.json').read_text())
        self.assertEqual(record['path_base'], 'manifest')
        self.assertFalse(Path(record['inputs']['scan']).is_absolute())
        self.assertFalse(Path(report['inputs']['scan']['path']).is_absolute())
        index = json.loads((self.output/'perception_regions.json').read_text())
        self.assertEqual(index['regions']['outer_front'], 'outer_front/perception_region.json')
        moved = self.project.with_name('new-machine-project')
        shutil.move(str(self.project), moved)
        index_path = moved/'outputs/run/regions/perception_regions.json'
        # The original project no longer exists; relative paths must resolve at the new location.
        restored = PreparedRegion.load_from_manifest(index_path, 'outer_front')
        self.assertEqual(restored.inputs.scan, moved/'assets/scan.ply')
        self.assertEqual(Path(restored.geometry.report['inputs']['scan']['path']), moved/'assets/scan.ply')
        for station in np.linspace(0, restored.reference.end, 11):
            np.testing.assert_array_equal(restored.reference.evaluate(station),
                                          self.prepared.reference.evaluate(station))
        # Exported files contain no stale project prefix and save does not mutate live provenance.
        self.assertNotIn(str(self.project), (index_path.parent/'outer_front/geometry.json').read_text())
        self.assertEqual(self.prepared.geometry.report['inputs']['scan']['path'],
                         str(self.project/'assets/scan.ply'))

    def test_source_and_artifact_checks_still_reject_tampering(self):
        path = self.export()
        self.prepared.inputs.scan.write_text('changed source')
        with self.assertRaisesRegex(ValueError, 'source changed'):
            PreparedRegion.load(path)
        self.prepared.inputs.scan.write_text('scan.ply')
        (path.parent/'continuous_gingival_line.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'artifact changed'):
            PreparedRegion.load(path)

    def test_absolute_legacy_results_remain_loadable(self):
        path = self.export()
        record = json.loads(path.read_text())
        record.pop('path_base')
        for key in ('scan', 'topology', 'prediction'):
            record['inputs'][key] = str((path.parent/record['inputs'][key]).resolve())
        report_path = path.parent/'geometry.json'
        report = json.loads(report_path.read_text())
        report.pop('path_base')
        for item in report['inputs'].values():
            item['path'] = str((path.parent/item['path']).resolve())
        report['source_metadata']['dataset_dir'] = str(self.project/'assets')
        dump(report_path, report)
        record['files']['geometry.json'] = fingerprint(report_path)['sha256']
        dump(path, record)
        restored = PreparedRegion.load(path)
        self.assertEqual(restored.inputs.scan, self.prepared.inputs.scan)

    def test_unknown_path_base_rejected(self):
        path = self.export()
        record = json.loads(path.read_text())
        record['path_base'] = 'unknown'
        dump(path, record)
        with self.assertRaisesRegex(ValueError, 'path base'):
            PreparedRegion.load(path)
