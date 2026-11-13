"""Offline FK and recorded jaw conversion can run from the tracked source package."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from perception.coordinate_conversion.ur7e_fk import fk, dh_fk, pose, calculate


class NominalKinematicsTests(unittest.TestCase):
    def test_urdf_and_dh_agree_for_multiple_configurations(self):
        wrist_to_tool = pose(rpy=(0, -np.pi/2, -np.pi/2)) @ pose(rpy=(np.pi/2, 0, np.pi/2))
        for q in (np.zeros(6), np.deg2rad([78.28, -46.01, 65.01, -273.56, 29.12, -19.07]),
                  np.array([.2, -.8, .7, -1.3, .6, -.3])):
            with self.subTest(q=q):
                computed = pose(rpy=(0, 0, -np.pi)) @ fk(q) @ wrist_to_tool
                np.testing.assert_allclose(computed, dh_fk(q), atol=1e-8)

    def test_invalid_joint_count_or_nonfinite_values_rejected(self):
        for q in ([0]*5, [0]*7, [0]*5+[float('nan')]):
            with self.assertRaises(ValueError):
                fk(q)

    def test_fk_reads_explicit_observation_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'joints.json'
            source.write_text(json.dumps({'joint_positions_deg': [78.28, -46.01, 65.01, -273.56, 29.12, -19.07]}))
            out = Path(tmp) / 'fk'
            with contextlib.redirect_stdout(io.StringIO()):
                calculate(source, out)
                with self.assertRaises(ValueError):
                    calculate(source, out)
            report = json.loads((out / 'fk_report.json').read_text())
            np.testing.assert_allclose(report['T_controller_base_wrist3_nominal_m'],
                np.load(out / 'T_controller_base_wrist3_nominal_m.npy'))
            self.assertFalse(report['factory_calibration_applied'])
            self.assertFalse(report['robot_execution_allowed'])


if __name__ == '__main__':
    unittest.main()
