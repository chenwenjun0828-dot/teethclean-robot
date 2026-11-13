"""Continuous regional sources retain every tooth interval and joint motion."""
from dataclasses import asdict
from pathlib import Path
import tempfile
import unittest
import numpy as np
from control.force.sweep_source import adapt_sweep
from control.force.contact_path import ContactPath
from planning.config import Config, SweepOptions
from planning.infrastructure.io import dump, fingerprint
from planning.infrastructure.trajectory import Trajectory
from planning.test_sweep import build_reference
from planning.test_sweep import straight_geometry


class ContactSweepTests(unittest.TestCase):
    def test_saved_regional_sweep_retains_all_arcs_and_immutable_joint_motion(self):
        cfg, options = Config(planning_mode='simple'), SweepOptions()
        geometry = straight_geometry()
        path = build_reference(geometry, cfg, options, 1.)
        parameters = np.unique([p[key] for p in path.parts for key in ('start', 'end')])
        count = len(parameters)+2
        joints = np.arange(count*6).reshape(count, 6)*.001
        plan = Trajectory(np.arange(float(count)), joints,
                          ['transfer']+['trace']*(len(parameters)-1)+['retreat'],
                          np.ones(count-1, int), list('abcdef'))
        phases_before = plan.phases.copy()
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            dump(folder/'sweep_reference.json', path.record)
            report = {'path_kind': 'continuous_outer_front_sweep', 'planning_mode': 'simple',
                      'config': asdict(cfg), 'sweep_options': asdict(options),
                      'sweep_reference_file': fingerprint(folder/'sweep_reference.json'),
                      'motion_segments': [{'status': 'planned', 'trace_start_waypoint': 1,
                                           'parameters': parameters.tolist()}]}
            adapted, segments = adapt_sweep(folder, plan, report)
            self.assertEqual(len(segments), len(geometry.strokes))
            self.assertEqual(sum(adapted.phases == 'surface_transfer'), 0)
            self.assertEqual(adapted.phases[-1], 'retreat')
            np.testing.assert_array_equal(plan.phases, phases_before)
            for time in np.linspace(0., plan.duration, 101):
                for key in ('q', 'dq', 'ddq'):
                    np.testing.assert_array_equal(adapted.sample(time)[key], plan.sample(time)[key])
            normals = {k: p['normal_scan'] for k, p in geometry.planes.items()}
            scheduler = ContactPath(adapted, segments, normals)
            self.assertTrue(scheduler.has_exit)
            broken = dict(report, motion_segments=[dict(report['motion_segments'][0],
                          parameters=np.delete(parameters, 1).tolist())])
            with self.assertRaisesRegex(ValueError, 'seam'):
                adapt_sweep(folder, plan, broken)


if __name__ == '__main__':
    unittest.main()
