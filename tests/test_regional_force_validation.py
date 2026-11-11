"""Regional contact preserves geometry and rejects force-only false positives."""
from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest

import fcl
import numpy as np

from control.force.compliance import ForceControlConfig
from control.force.contact_path import ContactPathConfig
from control.force.continuous_path import RegionalContactPath
from control.force.validation import audit, signal_metrics
from control.force.contact_experiment import write_csv
from planning.infrastructure.io import dump, fingerprint
from planning.infrastructure.trajectory import Trajectory
from planning.config import Config, SweepOptions
from planning.test_sweep import build_reference
from planning.test_sweep import straight_geometry


class RegionalForceTests(unittest.TestCase):
    def test_real_physics_controller_benchmark_is_not_dental_trajectory_certification(self):
        from control.force.benchmark import run
        with tempfile.TemporaryDirectory() as tmp:
            report = run(Path(tmp)/'benchmark')
        self.assertTrue(report['controller_benchmark_passed'])
        self.assertFalse(report['full_robot_contact_trajectory_verified'])
        self.assertLess(report['steady_rmse_N'], .001)
        self.assertLess(report['max_sensor_solver_difference_N'], 1e-6)

    def test_one_regional_trace_keeps_all_sources_bridges_and_joint_samples(self):
        cfg, options = Config(), SweepOptions()
        reference = build_reference(straight_geometry(), cfg, options, 1.)
        parameters = np.unique([p[k] for p in reference.parts for k in ('start', 'end')])
        count = len(parameters)+2
        plan = Trajectory(np.arange(float(count)), np.zeros((count, 6)),
            ['transfer']+['trace']*(len(parameters)-1)+['retreat'], np.ones(count-1, int), list('abcdef'))
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            dump(folder/'sweep_reference.json', reference.record)
            report = {'planning_mode': 'simple', 'config': asdict(cfg), 'sweep_options': asdict(options),
                      'sweep_reference_file': fingerprint(folder/'sweep_reference.json'),
                      'motion_segments': [{'status': 'planned', 'trace_start_waypoint': 1,
                                           'trace_end_waypoint': count-2, 'parameters': parameters.tolist()}]}
            path = RegionalContactPath(folder, plan, report, np.eye(4), ForceControlConfig(), ContactPathConfig())
            self.assertEqual(len(path.segments), 1)
            self.assertTrue(path.has_exit)
            self.assertEqual(sum(path.plan.phases == 'trace'), len(parameters)-1)
            self.assertEqual(path.reference.record['parts'], json.loads((folder/'sweep_reference.json').read_text())['parts'])
            for time in np.linspace(0., plan.duration, 101):
                for key in ('q', 'dq', 'ddq'):
                    np.testing.assert_array_equal(path.plan.sample(time)[key], plan.sample(time)[key])
            for parameter, time in zip(parameters, plan.times[1:-1]):
                self.assertAlmostEqual(path.parameter_at(time), parameter)
                path.source_time = time
                np.testing.assert_allclose(path.normal, reference.normal(parameter))

    def test_signal_audit_includes_before_and_after_command_and_keeps_all_samples(self):
        rows = [{'time_s': i*.001, 'phase': 'trace', 'compensated_force_N': 1.,
                 'applied_command_force_N': 1.} for i in range(3)]
        rows[1]['applied_command_force_N'] = 2.6
        values = signal_metrics(rows, 1., .2, .05)
        self.assertEqual(values['peak_force_N'], 2.6)
        self.assertAlmostEqual(values['trace_fraction_within_tolerance'], 2/3)
        with self.assertRaisesRegex(ValueError, 'every 1 ms'):
            signal_metrics(rows[::2], 1., .2, .05)
        rows[1]['compensated_force_N'] = float('nan')
        with self.assertRaisesRegex(ValueError, 'Nonfinite'):
            signal_metrics(rows, 1., .2, .05)

    def test_perfect_force_with_a_real_mesh_gap_does_not_pass_contact_validation(self):
        tool = fcl.CollisionObject(fcl.Sphere(.001))
        jaw = fcl.CollisionObject(fcl.Box(.1, .1, .1), fcl.Transform(np.array([0., 0., -.05])))
        data = SimpleNamespace(geom_xmat=np.array([np.eye(3).ravel()]), geom_xpos=np.zeros((1, 3)))

        def fk(q):
            data.geom_xpos[0] = [0., 0., q[0]]
            pose = np.eye(4)
            pose[:3, 3] = data.geom_xpos[0]
            return pose

        robot = SimpleNamespace(joint_names=['q'], fk=fk, data=data)
        checker = SimpleNamespace(robot=robot, model=SimpleNamespace(geom=lambda _: SimpleNamespace(name='ur_end_scaler_geom')),
                                  parts=[(0, 0, tool)], teeth=[('mandibular', jaw)])
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source, output = root/'source', root/'force'
            source.mkdir()
            output.mkdir()
            dump(source/'trajectory.npz', {'immutable_test_fixture': True})
            dump(source/'planning_report.json', {'immutable_test_fixture': True})
            report = {'source': str(source), 'source_trajectory': fingerprint(source/'trajectory.npz'),
                      'source_planning_report': fingerprint(source/'planning_report.json'),
                      'jaw': 'mandibular', 'force_control': asdict(ForceControlConfig()),
                      'path_control': asdict(ContactPathConfig()), 'force_tracking_passed': True,
                      'peak_compensated_force_N': 1., 'trace_force_rmse_N': 0.,
                      'trace_contact_loss_s': 0., 'trace_fraction_within_tolerance': 1.}
            dump(output/'force_trajectory_report.json', report)
            rows = [{'time_s': i*.001, 'phase': 'trace', 'q': .002,
                     'compensated_force_N': 1., 'applied_command_force_N': 1.} for i in range(10)]
            write_csv(output/'trajectory_force.csv', rows)
            result = audit(output, checker=checker)
            self.assertTrue(result['signals_match_report'])
            self.assertFalse(result['contact_trajectory_passed'])
            self.assertFalse(result['scanned_surface_contact_passed'])
            np.testing.assert_allclose(result['force_present_trace_scan_gap_range_mm'], [1., 1.], atol=1e-8)
            for row in rows:
                row['q'] = .00105
            write_csv(output/'trajectory_force.csv', rows)
            result = audit(output, checker=checker)
            self.assertTrue(result['scanned_surface_contact_passed'])
            self.assertTrue(result['contact_trajectory_passed'])
            for row in rows:
                row['q'] = .0005
            write_csv(output/'trajectory_force.csv', rows)
            result = audit(output, checker=checker)
            self.assertFalse(result['scanned_surface_contact_passed'])


if __name__ == '__main__':
    unittest.main()
