"""Normal admittance and sensor-based force compensation behavior."""
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

import mujoco
import numpy as np

from control.force.compliance import ForceControlConfig, NormalAdmittance, SensorContactForce


class AdmittanceTests(unittest.TestCase):
    def test_force_error_moves_outward_or_inward_with_bounded_speed(self):
        cfg = ForceControlConfig()
        high, low = NormalAdmittance(cfg), NormalAdmittance(cfg)
        for _ in range(100):
            high.step(2., 1., .002)
            low.step(0., 1., .002)
            self.assertLessEqual(abs(high.velocity), cfg.max_normal_speed_m_s)
            self.assertLessEqual(abs(low.velocity), cfg.max_normal_speed_m_s)
        self.assertGreater(high.displacement, 0.)
        self.assertLess(low.displacement, 0.)

    def test_compliant_wall_converges_and_recovers_after_surface_shift(self):
        control = NormalAdmittance(ForceControlConfig())
        errors = []
        for i in range(4000):
            surface = .00005 if i >= 2000 else 0.
            force = max(0., 5000.*(surface-control.displacement))
            control.step(force, 1., .002)
            if i >= 3500:
                errors.append(force-1.)
        self.assertLess(max(abs(np.asarray(errors))), .01)

    def test_travel_limit_prevents_windup_and_reverses_when_force_rises(self):
        cfg = ForceControlConfig(max_displacement_m=.0001)
        control = NormalAdmittance(cfg)
        for _ in range(1000): control.step(0., 1., .002)
        self.assertEqual(control.displacement, -cfg.max_displacement_m)
        self.assertEqual(control.velocity, 0.)
        for _ in range(100): control.step(2., 1., .002)
        self.assertGreater(control.displacement, -cfg.max_displacement_m)

    def test_invalid_force_limits_and_sampling_are_rejected(self):
        for kwargs in ({'target_force_N': 0.}, {'target_force_N': 3.}, {'force_stop_N': .5},
                       {'virtual_mass_kg': float('nan')}, {'damping_Ns_m': -1.}):
            with self.assertRaises(ValueError): ForceControlConfig(**kwargs)
        control = NormalAdmittance(ForceControlConfig())
        for sample in ((float('nan'), 1., .002), (1., 1., 0.), (1., 1., .1), (-1., 1., .002)):
            with self.assertRaises(ValueError): control.step(*sample)


class SensorCompensationTests(unittest.TestCase):
    def model(self):
        return mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="carrier" pos="0 0 1" quat=".9238795 0 .3826834 0"><freejoint/>
            <inertial mass=".2" pos="0 0 0" diaginertia=".01 .01 .01"/>
            <body name="tool" pos=".02 0 .03">
              <geom type="sphere" size=".01" mass=".3"/>
              <site name="ur_scaler_force_site" quat=".7071068 .7071068 0 0"/>
            </body></body></worldbody>
          <sensor><force name="ur_scaler_force" site="ur_scaler_force_site"/>
          <torque name="ur_scaler_torque" site="ur_scaler_force_site"/></sensor></mujoco>''')

    def test_gravity_and_inertia_are_removed_in_rotated_sensor_frame(self):
        model = self.model()
        data = mujoco.MjData(model)
        data.qvel[:] = [.2, -.1, .3, .5, -.7, .9]
        mujoco.mj_forward(model, data)
        observer = SensorContactForce(model)
        np.testing.assert_allclose(observer.read(data)['external_force_world_N'], 0., atol=1e-10)
        self.assertFalse(model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT))

    def test_sensor_feedback_recovers_applied_force_with_correct_world_sign(self):
        model = self.model()
        data = mujoco.MjData(model)
        force = np.array([.4, -.2, .9])
        data.xfrc_applied[model.body('tool').id, :3] = force
        data.qvel[:] = [.1, -.2, .1, .2, -.4, .3]
        mujoco.mj_forward(model, data)
        measured = SensorContactForce(model).read(data)
        np.testing.assert_allclose(measured['external_force_world_N'], force, atol=1e-10)
        self.assertAlmostEqual(measured['force_N'], np.linalg.norm(force))

    def test_overforce_stops_before_integration_and_preserves_sample(self):
        from pipeline.simulation import create_simulation
        from planning.infrastructure.robot import Robot
        from control.force.contact_experiment import run_probe
        model, data = create_simulation()
        robot = Robot(model, data)
        probe = {'start_q': robot.home.copy(), 'goal_q': robot.home.copy(),
                 'target_world_m': robot.fk(robot.home).copy(), 'normal_world': np.array([0., 0., 1.])}
        observation = {'force_N': 4., 'external_force_world_N': np.array([0., 0., 4.])}
        with tempfile.TemporaryDirectory() as temp, patch.object(SensorContactForce, 'read', return_value=observation):
            report, rows = run_probe(model, robot, 'mandibular', 1, probe, Path(temp),
                                    force_limit=3., force_config=ForceControlConfig())
            self.assertEqual(report['status'], 'stopped')
            self.assertIn('stop limit', report['stop_reason'])
            self.assertEqual(report['recorded_duration_s'], 0.)
            self.assertEqual(len(rows), 1)
            self.assertTrue((Path(temp)/'tooth_1_force.csv').exists())
            self.assertFalse(report['force_control']['passed'])


if __name__ == '__main__':
    unittest.main()
