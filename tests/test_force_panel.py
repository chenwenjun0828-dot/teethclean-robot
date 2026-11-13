"""Telemetry keeps real spikes and cannot change the running physics state."""
import unittest
from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np

from control.force.compliance import ForceControlConfig
from control.force.contact_path import ContactPathConfig
from control.force.force_history import CHANNELS, ForceHistory, peak_envelope
from control.force.force_panel import ForcePanel, panel_rows, render_panel


def sample(t=0., force=1.):
    row = dict.fromkeys(CHANNELS, 0.)
    row.update(time_s=t, compensated_force_N=force, applied_command_force_N=force,
               force_target_N=1., Fx_N=-4., Fz_N=6., Ty_Nm=.02,
               phase='trace', segment=0, tooth=10)
    return row


class ForcePanelTests(unittest.TestCase):
    def test_closed_window_still_prepares_accurate_final_snapshot(self):
        panel = ForcePanel()
        panel.set_config(ForceControlConfig(), ContactPathConfig(), .001)
        panel.update(None, sample(8.), .5, '已停止 / 用户关闭窗口', force_draw=True)
        self.assertEqual(dict(panel.rows)['状态'], '已停止 / 用户关闭窗口')
        self.assertEqual(dict(panel.rows)['仿真时间'], '8.000 s')

    def test_history_rolls_and_final_sample_is_not_duplicated(self):
        history = ForceHistory(window_s=.1, sample_rate_hz=1000.)
        for i in range(501):
            history.append(sample(i*.001))
        before = len(history.samples)
        history.append(sample(.5, 2.))
        data = history.arrays()
        self.assertEqual(len(data), before)
        self.assertLessEqual(len(data), 102)
        self.assertAlmostEqual(data[0, 0], .4)
        self.assertEqual(data[-1, 1], 2.)
        with self.assertRaises(ValueError): history.append(sample(.2))
        with self.assertRaises(ValueError): history.append(sample(.6, float('nan')))

    def test_reduction_keeps_single_sample_spike_and_contact_loss(self):
        times = np.linspace(0., 10., 10001)
        force = np.ones(len(times))
        force[3456], force[3457] = 3.7, 0.
        _, low, high = peak_envelope(times, force, 0., 10., 400)
        self.assertEqual(high.max(), 3.7)
        self.assertEqual(low.min(), 0.)
        self.assertTrue(np.any((low == 0.) & (high == 3.7)))

    def test_parameters_use_actual_config_and_render_unclipped_raw_values(self):
        row = sample(force=3.7)
        original = dict(row)
        force = ForceControlConfig(target_force_N=.9, damping_Ns_m=250.)
        path = ContactPathConfig()
        rows = panel_rows(row, force, path, .001, .5, peak=3.7)
        values = dict(rows)
        self.assertEqual(values['设定目标 / 硬停止'], '0.9 / 3 N')
        self.assertIn('250', values['虚拟质量 / 阻尼'])
        self.assertEqual(values['原始 Fx / Fy / Fz [N]'], '-4.000 / 0.000 / 6.000')
        self.assertEqual(values['补偿后接触合力'], '3.700 N')
        self.assertEqual(row, original)
        history = ForceHistory()
        history.append(row)
        for width, height in [(580, 1000), (320, 600)]:
            image = render_panel(rows, width, height, history=history.arrays())
            self.assertEqual(image.shape, (height, width, 3))
            self.assertEqual(image.dtype, np.uint8)

    def test_viewer_perturbations_and_display_throttle_do_not_change_simulation(self):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <body name="dental_mandibular"><joint name="j" type="slide"/>
          <geom type="sphere" size=".01" mass="1"/></body></worldbody>
          <actuator><position joint="j"/></actuator></mujoco>''')
        data = mujoco.MjData(model)
        data.qpos[0], data.ctrl[0] = .1, .2
        mujoco.mj_forward(model, data)
        viewer = SimpleNamespace(cam=SimpleNamespace(lookat=np.zeros(3)),
                                 lock=nullcontext, is_running=lambda: True, close=lambda: None,
                                 viewport=None)
        panel = ForcePanel()
        panel.set_config(ForceControlConfig(), ContactPathConfig(), .001)
        with patch('mujoco.viewer.launch_passive', return_value=viewer):
            panel.open(model, data, 'mandibular')
        # Simulate dragging the displayed tool and changing the viewer's controls.
        def perturb():
            panel.model.opt.gravity[:] = 999.
            panel.data.ctrl[:] = 123.
            panel.data.qpos[:] = 456.
        viewer.sync = lambda **kwargs: perturb()
        try:
            with patch('control.force.force_panel.time.monotonic', return_value=1.):
                panel.update(data, sample(0.), 0.)
                panel.update(data, sample(.001, 4.), 0.)  # No second UI frame.
            self.assertEqual(panel.peak, 4.)
            self.assertEqual(len(panel.history.samples), 2)
            self.assertEqual(panel.history.arrays()[-1, 1], 4.)
            np.testing.assert_array_equal(model.opt.gravity, [0., 0., -9.81])
            self.assertEqual(data.ctrl[0], .2)
            self.assertEqual(data.qpos[0], .1)
        finally:
            panel.close()


if __name__ == '__main__':
    unittest.main()
