import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from perception.contracts import PerceptionResult
from planning.contracts import PlanningResult
from control.contracts import ControlReport
from pipeline.contracts import OutputPolicy, PipelineReport


class ContractTests(unittest.TestCase):
    def test_perception_round_trip_preserves_paths_and_transform(self):
        result = PerceptionResult(
            jaw="maxillary",
            registration_dir=Path("perception/registration"),
            registration_transform=np.eye(4),
            source_fingerprints={"scan": "abc"},
        )
        with tempfile.TemporaryDirectory() as directory:
            path = result.save(Path(directory) / "perception.json")
            loaded = PerceptionResult.load(path)
        self.assertEqual(loaded.jaw, "maxillary")
        self.assertEqual(loaded.registration_dir, Path("perception/registration"))
        np.testing.assert_allclose(loaded.registration_transform, np.eye(4))

    def test_planning_round_trip_rejects_unknown_schema(self):
        result = PlanningResult(jaw="mandibular", execution_ready=True)
        with tempfile.TemporaryDirectory() as directory:
            path = result.save(Path(directory) / "planning.json")
            payload = json.loads(path.read_text(encoding="utf-8"))
            payload["schema_version"] = 99
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(ValueError):
                PlanningResult.load(path)

    def test_control_rejects_unknown_fault_code(self):
        with self.assertRaises(ValueError):
            ControlReport(backend="mock", fault_reason="unknown")

    def test_output_policy_modes(self):
        self.assertTrue(OutputPolicy(no_output=True).executes_algorithms)
        self.assertFalse(OutputPolicy(no_output=True).writes_files)
        self.assertFalse(OutputPolicy(dry_run=True).executes_algorithms)
        with self.assertRaises(ValueError):
            OutputPolicy(no_output=True, dry_run=True)

    def test_pipeline_report_serializes_path(self):
        report = PipelineReport(jaw="mandibular", output_dir=Path("outputs/run"))
        self.assertEqual(report.to_dict()["output_dir"], "outputs/run")
