"""Full-path scheduling, contact recovery and predictive collision/force guards."""
import unittest
from types import SimpleNamespace

import mujoco
import numpy as np

from control.force.compliance import ForceControlConfig, SensorContactForce
from control.force.contact_path import ContactPath, ContactPathConfig, RateGovernor
from control.force.force_trajectory import PredictiveContactGuard, apply_tool_roll
from planning.infrastructure.trajectory import Trajectory


def example():
    plan = Trajectory(np.arange(5.), np.zeros((5, 6)),
                      ['transfer', 'trace', 'surface_transfer', 'trace'], [1, 1, 2, 2], list('abcdef'))
    segments = [dict(status='planned', tooth=1, trace_start_waypoint=1, trace_end_waypoint=2, length_mm=1.),
                dict(status='planned', tooth=2, trace_start_waypoint=3, trace_end_waypoint=4, length_mm=1.)]
    normals = {1: [0., 0., 1.], 2: [0., 1., 0.]}
    return plan, segments, normals


class ContactPathTests(unittest.TestCase):
    def test_segment_speed_only_retimes_selected_trace_and_keeps_slow_segment(self):
        cfg = ContactPathConfig(max_trace_speed_m_s=.0005)
        base = ContactPath(*example(), config=cfg)
        changed = ContactPath(*example(), config=cfg, segment_speeds_m_s={0: .001})
        self.assertLess(changed.durations[0][1], base.durations[0][1])
        self.assertEqual(changed.durations[0][0], base.durations[0][0])
        self.assertEqual(changed.durations[1], base.durations[1])
        self.assertEqual(changed.config.predictive_force_N, base.config.predictive_force_N)
        self.assertEqual(changed.force.force_stop_N, base.force.force_stop_N)
        for overrides in ({-1: .001}, {2: .001}, {0: 0.}, {0: float('nan')}, {0: True}):
            with self.assertRaises(ValueError): ContactPath(*example(), segment_speeds_m_s=overrides)

    def test_missing_or_mislabelled_segments_cannot_report_whole_path(self):
        plan, segments, normals = example()
        with self.assertRaises(ValueError): ContactPath(plan, segments[:1], normals)
        segments[1]['tooth'] = 1
        with self.assertRaises(ValueError): ContactPath(plan, segments, normals)

    def test_limit_configuration_rejects_above_3_N_and_bad_normals(self):
        with self.assertRaises(ValueError): ContactPathConfig().validate(ForceControlConfig(force_stop_N=3.1))
        with self.assertRaises(ValueError): ContactPathConfig(predictive_force_N=3.).validate(ForceControlConfig())
        with self.assertRaises(ValueError): ContactPathConfig(max_duration_s=float('nan')).validate(ForceControlConfig())
        plan, segments, normals = example()
        normals[1] = [0., 0., 0.]
        with self.assertRaises(ValueError): ContactPath(plan, segments, normals)

    def test_entire_two_tooth_path_visits_contact_and_release_in_order(self):
        path = ContactPath(*example())
        phases, forces, offsets = [], [], []
        offset = np.zeros(3)
        for _ in range(25000):
            force = max(0., -4000.*float(offset@path.normal)) if path.stage != 'transfer' else 0.
            state = path.update(force, 0., .001)
            offset = state['offset_world_m']
            key = state['segment_index'], state['phase']
            if not phases or phases[-1] != key: phases.append(key)
            if state['phase'] == 'trace': forces.append(force)
            offsets.append(offset.copy())
            self.assertFalse(state['fault_reason'])
            if path.completed: break
        self.assertTrue(path.completed)
        self.assertEqual(phases, [(i, p) for i in range(2) for p in ('transfer', 'acquire', 'trace', 'release')])
        self.assertLess(max(abs(np.asarray(forces)-1.)), .2)
        # Free-space offset blending is <=20 mm/s; no position jumps at phases.
        self.assertLess(np.linalg.norm(np.diff(offsets, axis=0), axis=1).max(), .00002)

    def test_contact_loss_smoothly_pauses_source_progress_and_recovers(self):
        path = ContactPath(*example())
        for _ in range(2000):
            path.update(1. if path.stage == 'acquire' else 0., 0., .001)
            if path.stage == 'trace': break
        self.assertEqual(path.stage, 'trace')
        for _ in range(100): path.update(1., 0., .001)
        before = path.source_time
        for _ in range(500): state = path.update(0., 0., .001)
        self.assertEqual(state['phase'], 'trace')
        self.assertLess(state['path_rate'], .0001)
        self.assertGreaterEqual(path.source_time, before)
        paused = path.source_time
        for _ in range(100): path.update(1., 0., .001)
        self.assertAlmostEqual(path.source_time, paused, places=6)
        for _ in range(200): path.update(1., 0., .001)
        self.assertGreater(path.source_time, paused)

    def test_soft_force_guard_unloads_and_hard_limit_latches_fault(self):
        path = ContactPath(*example())
        path.stage = 'trace'
        path.admittance.displacement = 0.
        for _ in range(100): state = path.update(2.1, 0., .001)
        self.assertEqual(state['force_target_N'], 0.)
        self.assertGreater(float(state['offset_world_m']@path.normal), 0.)
        self.assertEqual(state['path_rate'], 0.)
        self.assertIn('hard limit', path.update(3., 0., .001)['fault_reason'])

    def test_release_must_separate_before_changing_tooth(self):
        path = ContactPath(*example())
        path.stage, path.release_start = 'release', 0.
        for _ in range(2000): state = path.update(1., 0., .001)
        self.assertEqual(path.index, 0)
        self.assertEqual(path.stage, 'release')
        path.update(0., 0., .001)
        self.assertEqual(path.stage, 'transfer')
        self.assertEqual(path.index, 1)

    def test_acquisition_timeout_never_skips_failed_tooth(self):
        path = ContactPath(*example(), config=ContactPathConfig(acquisition_timeout_s=.5))
        path.stage = 'acquire'
        for _ in range(502): state = path.update(0., 0., .001)
        self.assertIn('timeout', state['fault_reason'])
        self.assertEqual(path.index, 0)
        self.assertFalse(path.completed)

    def test_completion_waits_for_original_final_retreat(self):
        original, segments, normals = example()
        plan = Trajectory(np.arange(6.), np.zeros((6, 6)),
                          [*original.phases, 'retreat'], [*original.teeth, 2], list('abcdef'))
        path = ContactPath(plan, segments, normals)
        path.index, path.stage = 1, 'release'
        path.elapsed = path.config.release_s
        path.update(0., 0., .001)
        self.assertEqual(path.stage, 'exit')
        self.assertFalse(path.completed)
        times, offsets = [], []
        for _ in range(2000):
            state = path.update(0., 0., .001)
            self.assertEqual(state['phase'], 'transfer')
            times.append(state['source_time_s'])
            offsets.append(state['offset_world_m'])
            if path.completed:
                break
        self.assertTrue(path.completed)
        self.assertEqual(times[-1], plan.duration)
        self.assertTrue(np.all(np.diff(times) >= 0.))
        fraction = (np.asarray(times)-path.exit_begin)/(plan.duration-path.exit_begin)
        np.testing.assert_allclose(np.asarray(offsets)[:, 1], .002*(1-fraction), atol=1e-14)
        np.testing.assert_allclose(offsets[-1], 0.)
        self.assertLess(np.linalg.norm(np.diff(offsets, axis=0), axis=1).max(), .00002)

    def test_rate_changes_are_bounded_and_decay_to_zero(self):
        governor = RateGovernor(.04)
        rates = [governor.step(1., .001)[0] for _ in range(500)]
        self.assertLess(max(np.diff(rates)), .01)
        self.assertGreater(rates[-1], .999)
        for _ in range(700): rate, _ = governor.step(0., .001)
        self.assertLess(rate, 1e-6)

    def test_roll_correction_preserves_tcp_position_and_shaft_direction(self):
        from scipy.spatial.transform import Rotation
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler('xyz', [.4, -.2, .7]).as_matrix()
        pose[:3, 3] = [.1, -.3, .2]
        changed = apply_tool_roll(pose, np.radians(-10.))
        np.testing.assert_allclose(changed[:3, 0], pose[:3, 0])
        np.testing.assert_allclose(changed[:3, 3], pose[:3, 3])
        self.assertAlmostEqual(np.linalg.det(changed[:3, :3]), 1.)
        self.assertGreater(np.linalg.norm(changed[:3, 1]-pose[:3, 1]), .1)
        with self.assertRaises(ValueError): ContactPath(*example(), roll_corrections_deg={2: -10.})
        with self.assertRaises(ValueError): ContactPath(*example(), roll_corrections_deg={1: -31.})

    def test_roll_correction_blends_on_transfer_and_holds_on_trace(self):
        path = ContactPath(*example(), roll_corrections_deg={1: -10.})
        path.index, path.stage, path.transfer_begin = 1, 'transfer', 2.
        path.transfer_offset = np.array([0., 0., .002])
        rolls = []
        while path.stage == 'transfer':
            rolls.append(path.update(0., 0., .001)['roll_correction_rad'])
        self.assertAlmostEqual(rolls[0], 0.)
        self.assertAlmostEqual(rolls[-1], np.radians(-10.))
        self.assertLess(np.max(abs(np.diff(rolls))), .001)
        self.assertAlmostEqual(path.update(0., 0., .001)['roll_correction_rad'], rolls[-1])


class PredictiveGuardTests(unittest.TestCase):
    def scene(self, jaw_name='dental_mandibular_visual'):
        model = mujoco.MjModel.from_xml_string(f'''<mujoco>
          <option timestep=".002" gravity="0 0 0"/>
          <worldbody>
            <body name="jaw"><geom name="{jaw_name}" type="box" pos="0 0 -.01" size=".1 .1 .01"/></body>
            <body name="ur_carrier" pos="0 0 .002"><joint type="slide" axis="0 0 1"/>
              <inertial pos="0 0 0" mass=".2" diaginertia=".001 .001 .001"/>
              <body name="ur_tool"><geom name="ur_end_scaler_geom" type="sphere" size=".001" mass=".3"/>
                <site name="ur_scaler_force_site"/><site name="ur_scaler_tcp" pos="0 0 -.001"/>
              </body></body>
          </worldbody>
          <sensor><force name="ur_scaler_force" site="ur_scaler_force_site"/>
            <torque name="ur_scaler_torque" site="ur_scaler_force_site"/></sensor></mujoco>''')
        data = mujoco.MjData(model)
        data.qvel[0] = -1.
        mujoco.mj_forward(model, data)
        robot = SimpleNamespace(site=model.site('ur_scaler_tcp').id)
        return model, data, robot

    def test_predicted_excess_force_is_rejected_without_advancing_executed_state(self):
        model, data, robot = self.scene()
        guard = PredictiveContactGuard(model, robot, 'mandibular', SensorContactForce(model), 2.5)
        before = data.qpos.copy(), data.qvel.copy(), data.time
        result = guard.check(data, np.zeros(0), 'trace')
        self.assertFalse(result['safe'])
        self.assertIn('force', result['reason'])
        self.assertGreater(result['force_N'], 2.5)
        np.testing.assert_array_equal(data.qpos, before[0])
        np.testing.assert_array_equal(data.qvel, before[1])
        self.assertEqual(data.time, before[2])
        self.assertEqual(data.ncon, 0)

    def test_opposite_jaw_cannot_be_whitelisted_as_intended_contact(self):
        model, data, robot = self.scene('dental_maxillary_visual')
        result = PredictiveContactGuard(model, robot, 'mandibular', SensorContactForce(model), 2.5).check(
            data, np.zeros(0), 'trace')
        self.assertFalse(result['safe'])
        self.assertIn('unintended collision', result['reason'])

    def test_force_at_command_application_is_checked_before_rollout(self):
        from unittest.mock import patch
        model, data, robot = self.scene()
        observer = SensorContactForce(model)
        with patch.object(observer, 'read', return_value={'force_N': 2.8}):
            result = PredictiveContactGuard(model, robot, 'mandibular', observer, 2.5).check(data, [], 'trace')
        self.assertFalse(result['safe'])
        self.assertEqual(result['rejected_at'], 'command_application')
        self.assertEqual(data.time, 0.)


if __name__ == '__main__':
    unittest.main()
