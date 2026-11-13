import unittest

import numpy as np

from control.hardware import MockHardwareBackend, MujocoHardwareBackend, URRobotBackend
from pipeline.simulation import create_simulation


class HardwareBackendTests(unittest.TestCase):
    def test_mock_backend_exposes_state_and_stop_contract(self):
        backend = MockHardwareBackend()
        with self.assertRaisesRegex(RuntimeError, 'not connected'):
            backend.read_state()
        backend.connect()
        backend.command_joint_position(np.ones(6) * 0.1)
        state = backend.read_state()
        np.testing.assert_allclose(state.joint_position_rad, 0.1)
        backend.command_joint_velocity(np.zeros(6))
        backend.emergency_stop('force limit')
        with self.assertRaisesRegex(RuntimeError, 'stopped'):
            backend.read_state()

    def test_ur_backend_requires_explicit_transport(self):
        backend = URRobotBackend('192.0.2.10')
        with self.assertRaisesRegex(RuntimeError, 'transport'):
            backend.connect()

    def test_mujoco_backend_uses_scene_actuator_and_tcp_contract(self):
        model, data = create_simulation()
        backend = MujocoHardwareBackend(model, data)
        backend.connect()
        state = backend.read_state()
        self.assertEqual(state.joint_position_rad.shape, (6,))
        backend.command_joint_position(state.joint_position_rad)


if __name__ == '__main__':
    unittest.main()
