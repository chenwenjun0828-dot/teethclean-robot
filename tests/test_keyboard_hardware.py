"""Operator lifecycle and Linux terminal cleanup without ROS or robot connections."""
from datetime import datetime, timedelta
import io
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from control.hardware import standalone_keyboard
from control.hardware.standalone_keyboard import KeyboardControl
from control.hardware.standalone_pack import pack_standalone
from control.hardware.standalone_runtime import portable_operation
from control.hardware.trajectory_contract import read_json, sha256, write_json
from tests.test_ros1_hardware import Clock, make_bundle
from tests.test_standalone_hardware import ScaledReferenceBackend


class ScriptedKeyboard(KeyboardControl):
    def __init__(self, on_arm=None):
        super().__init__()
        self.on_arm = on_arm

    def arm(self):
        super().arm()
        if self.on_arm:
            self.on_arm(self)


class KeyboardHardwareTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='ur7e_keyboard_test_')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root/'source'
        self.source.mkdir()
        config = make_bundle(self.source, ready=False)
        record = read_json(self.source/'trajectory.json')
        record['points'][-1]['positions'] = [.05]*6
        write_json(self.source/'trajectory.json', record)
        report = read_json(self.source/'export_report.json')
        report['trajectory_sha256'] = sha256(self.source/'trajectory.json')
        write_json(self.source/'export_report.json', report)
        config['authorization'].update(trajectory_sha256=sha256(self.source/'trajectory.json'),
                                      export_report_sha256=sha256(self.source/'export_report.json'))
        config['motion_limits'] = {'joint_velocity_rad_s': [20.]*6,
                                   'joint_acceleration_rad_s2': [20.]*6}
        write_json(self.source/'deployment.json', config)
        self.delivery = self.root/'delivery'
        pack_standalone(self.source/'trajectory.json', self.source/'deployment.json',
                        self.delivery, allow_geometry_only=True)
        self.payload = self.delivery/'trajectory.json'
        self.stderr = patch('sys.stderr', new_callable=io.StringIO)
        self.stderr.start()
        self.addCleanup(self.stderr.stop)

    def invoke(self, control, *, directory='run', factor=1., on_sleep=None,
               on_snapshot=None, stop_ok=True, operation=portable_operation):
        clock = Clock()
        self.backends = []
        def factory(config, curve):
            backend = ScaledReferenceBackend(curve, stop_ok=stop_ok)
            original = backend.snapshot
            def snapshot():
                state = original()
                if on_snapshot:
                    on_snapshot(backend, state)
                return state
            backend.snapshot = snapshot
            self.backends.append(backend)
            return backend
        def sleep(duration):
            clock.sleep(duration)
            if on_sleep:
                on_sleep(clock)
        return operation('execute', self.payload, self.root/directory, operator=control,
                         speed_factor=factor, backend_factory=factory,
                         now=clock.now, sleep=sleep)

    def test_without_s_waits_and_space_cancels_without_motion_commands(self):
        control = ScriptedKeyboard()
        control.handle_key('S')
        def wait(clock):
            if control.armed:
                self.assertEqual(self.backends[0].sent, 0)
                if clock.now() > .8:
                    control.handle_key(' ')
        result = self.invoke(control, on_sleep=wait)
        self.assertEqual(result['status'], 'cancelled_before_start')
        backend = self.backends[0]
        self.assertGreater(backend.beats, 10)
        self.assertEqual((backend.sent, backend.cancelled, backend.stops), (0, 0, 0))
        self.assertFalse(result['motion_send_attempted'])
        self.assertTrue(backend.closed)

    def test_early_s_is_discarded_and_space_latches_before_backend(self):
        control = ScriptedKeyboard()
        control.handle_key('S')
        self.assertFalse(control.start_requested())
        control.handle_key(' ')
        control.handle_key('S')
        result = self.invoke(control)
        self.assertEqual(result['status'], 'cancelled_before_start')
        self.assertEqual(self.backends, [])

    def test_repeated_start_sends_exactly_one_goal(self):
        control = ScriptedKeyboard(lambda c: [c.handle_key(k) for k in 'SsSs'])
        result = self.invoke(control, on_snapshot=lambda b, s: control.handle_key('S'))
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(self.backends[0].sent, 1)
        self.assertEqual(self.backends[0].stops, 0)
        self.assertEqual(result['controller_progress_s'], 1.)

    def test_space_has_priority_over_start_in_same_batch(self):
        control = ScriptedKeyboard(lambda c: [c.handle_key(k) for k in 'S S'])
        result = self.invoke(control)
        self.assertEqual(result['status'], 'cancelled_before_start')
        self.assertEqual(self.backends[0].sent, 0)
        self.assertEqual(self.backends[0].stops, 0)

    def test_space_during_preflight_sends_no_motion_or_stop(self):
        control = ScriptedKeyboard()
        result = self.invoke(control, on_snapshot=lambda b, s: control.handle_key(' '))
        self.assertEqual(result['status'], 'cancelled_before_start')
        b = self.backends[0]
        self.assertEqual((b.sent, b.cancelled, b.stops), (0, 0, 0))

    def test_stop_after_start_request_but_before_send_sends_nothing(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        armed_snapshots = []
        def stop_before_send(backend, state):
            if control.armed:
                armed_snapshots.append(state)
                if len(armed_snapshots) == 2:
                    control.handle_key(' ')
        result = self.invoke(control, on_snapshot=stop_before_send)
        self.assertEqual(result['status'], 'cancelled_before_start')
        self.assertEqual((self.backends[0].sent, self.backends[0].stops), (0, 0))

    def test_space_during_motion_stops_confirms_and_never_resumes(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('s'))
        def stop(backend, state):
            if backend.sent:
                control.handle_key(' ')
                control.handle_key('S')
        result = self.invoke(control, factor=2., on_snapshot=stop)
        self.assertEqual(result['status'], 'operator_stopped')
        self.assertEqual(result['operator_stop_reason'], 'operator_space')
        self.assertTrue(result['stop']['confirmed'])
        b = self.backends[0]
        self.assertEqual((b.sent, b.cancelled, b.stops), (1, 1, 1))
        self.assertTrue(b.closed)

    def test_operator_stop_failure_is_not_reported_as_stopped_or_completed(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        def stop(backend, state):
            if backend.sent:
                control.handle_key(' ')
        result = self.invoke(control, on_snapshot=stop, stop_ok=False)
        self.assertEqual(result['status'], 'fault_latched')
        self.assertEqual(result['fault'], 'operator_stop_not_confirmed')
        self.assertFalse(result['stop']['confirmed'])

    def test_detected_input_failure_stops_active_motion(self):
        for reason in ('keyboard_input_eof', 'keyboard_input_error: disconnected'):
            with self.subTest(reason=reason):
                control = ScriptedKeyboard(lambda c: c.handle_key('S'))
                def lost(backend, state):
                    if backend.sent:
                        control.fail(reason)
                result = self.invoke(control, directory=reason[:18], on_snapshot=lost)
                self.assertEqual(result['status'], 'fault_latched')
                self.assertEqual(result['fault'], reason)
                self.assertTrue(result['stop']['confirmed'])

    def test_waiting_continues_state_checks_and_rejects_stale_force(self):
        control = ScriptedKeyboard()
        def stale(backend, state):
            if control.armed:
                state['ages']['wrench'] = 10.
        result = self.invoke(control, on_snapshot=stale)
        self.assertEqual(result['fault'], 'stale_wrench')
        self.assertEqual(self.backends[0].sent, 0)

    def test_waiting_checks_start_position_and_force_limit(self):
        for kind in ('start_mismatch', 'force_limit'):
            with self.subTest(kind=kind):
                control = ScriptedKeyboard()
                def wrong(backend, state):
                    if control.armed:
                        state.update(q=[1.]*6) if kind == 'start_mismatch' else state.update(wrench=[2.]*6)
                result = self.invoke(control, directory=kind, on_snapshot=wrong)
                self.assertEqual(result['fault'], kind)
                self.assertEqual(self.backends[0].sent, 0)

    def test_wait_does_not_consume_motion_wall_timeout(self):
        control = ScriptedKeyboard()
        def delayed(clock):
            if control.armed and clock.now() >= 6.:
                control.handle_key('S')
        result = self.invoke(control, on_sleep=delayed)
        self.assertEqual(result['status'], 'completed')
        self.assertGreater(result['operator_wait_s'], 5.)
        self.assertLess(result['elapsed_s'], 1.)

    def test_authorization_is_rechecked_after_wait(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        class Later(datetime):
            @classmethod
            def now(cls, tz=None):
                return datetime.now(tz)+timedelta(hours=2 if control.armed else 0)
        with patch('control.hardware.ros_runner.datetime', Later):
            result = self.invoke(control)
        self.assertIn('authorization.expired', result['fault'])
        self.assertEqual(self.backends[0].sent, 0)

    def test_changed_json_during_wait_refuses_send(self):
        def change(control):
            with self.payload.open('ab') as stream:
                stream.write(b' ')
            control.handle_key('S')
        result = self.invoke(ScriptedKeyboard(change))
        self.assertEqual(result['fault'], 'portable_input_changed_before_send')
        self.assertEqual(self.backends[0].sent, 0)

    def test_keyboard_interrupt_during_motion_still_confirms_stop(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        def interrupt(backend, state):
            if backend.sent and not backend.cancelled:
                raise KeyboardInterrupt()
        result = self.invoke(control, on_snapshot=interrupt)
        self.assertEqual(result['fault'], 'KeyboardInterrupt')
        self.assertTrue(result['stop']['confirmed'])
        self.assertTrue(self.backends[0].closed)

    def test_force_protection_during_motion_is_unchanged(self):
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        def force(backend, state):
            if backend.sent:
                state['wrench'][0] = 2.
        result = self.invoke(control, on_snapshot=force)
        self.assertEqual(result['fault'], 'force_limit')
        self.assertTrue(result['stop']['confirmed'])

    def test_keyboard_does_not_change_goal_hash_derivatives_or_json(self):
        before = self.payload.read_bytes()
        normal = self.invoke(None, factor=2., directory='automatic')
        first = self.backends[0].goal
        control = ScriptedKeyboard(lambda c: c.handle_key('S'))
        keyboard = self.invoke(control, factor=2., directory='keyboard')
        second = self.backends[0].goal
        self.assertEqual(normal['status'], keyboard['status'])
        self.assertEqual(first, second)
        self.assertEqual(normal['execution_curve_sha256'], keyboard['execution_curve_sha256'])
        self.assertEqual(normal['commanded_duration_s'], keyboard['commanded_duration_s'])
        self.assertEqual(self.payload.read_bytes(), before)

    def test_controls_are_rejected_for_read_only_modes(self):
        for command in ('inspect', 'check'):
            with self.subTest(command=command), self.assertRaises(ValueError):
                portable_operation(command, self.payload, self.root/'readonly', operator=ScriptedKeyboard())

    def test_generated_keyboard_entry_rejects_non_tty_without_connecting(self):
        completed = subprocess.run([sys.executable, '-I', '-B',
            str(self.delivery/'ur7e_saved_trajectory_executor.py'), 'execute', '--keyboard',
            '--trajectory', str(self.payload), '--out', str(self.root/'non_tty')],
            capture_output=True, text=True, stdin=subprocess.DEVNULL, cwd=self.delivery, timeout=30)
        self.assertEqual(completed.returncode, 2, completed.stdout+completed.stderr)
        self.assertIn('interactive TTY', completed.stdout)
        self.assertFalse((self.root/'non_tty').exists())

    def test_generated_keyboard_runtime_executes_with_same_operator_events(self):
        module = runpy.run_path(str(self.delivery/'ur7e_saved_trajectory_executor.py'))
        result = self.invoke(ScriptedKeyboard(lambda c: c.handle_key('S')),
                             factor=2., operation=module['portable_operation'])
        self.assertEqual(result['status'], 'completed')
        self.assertTrue(result['keyboard_control'])
        self.assertEqual(result['controller_progress_s'], .5)


class KeyboardTerminalTests(unittest.TestCase):
    def setUp(self):
        self.stderr = patch('sys.stderr', new_callable=io.StringIO)
        self.stderr.start()
        self.addCleanup(self.stderr.stop)

    def test_pending_early_s_is_discarded_but_pending_space_is_preserved(self):
        for data in (b'S', b'S '):
            with self.subTest(data=data):
                control = KeyboardControl()
                control.fd = 7
                control.select = SimpleNamespace(select=Mock(side_effect=[([7], [], []), ([], [], [])]))
                with patch.object(standalone_keyboard.os, 'read', return_value=data):
                    control.arm()
                self.assertFalse(control.start_requested())
                self.assertEqual(control.stop_reason(), 'operator_space' if b' ' in data else None)

    def test_listener_eof_and_read_error_latch_failure(self):
        for error in (None, OSError('lost terminal')):
            with self.subTest(error=error):
                control = KeyboardControl()
                control.fd = 7
                control.select = SimpleNamespace(select=lambda *_: ([7], [], []))
                with patch.object(standalone_keyboard.os, 'read', return_value=b'', side_effect=error):
                    control.listen()
                self.assertTrue(control.stop_reason().startswith('keyboard_input_'))

    def test_listener_batch_prioritizes_space_and_latches_it(self):
        control = KeyboardControl()
        control.armed = True
        control.fd = 7
        control.select = SimpleNamespace(select=lambda *_: ([7], [], []))
        with patch.object(standalone_keyboard.os, 'read', return_value=b's S'):
            control.listen()
        self.assertEqual(control.stop_reason(), 'operator_space')
        control.handle_key('S')
        self.assertEqual(control.stop_reason(), 'operator_space')

    def terminal_context(self):
        terminal = SimpleNamespace(TCSANOW=0, tcgetattr=Mock(return_value=['saved']),
                                   tcsetattr=Mock())
        tty = SimpleNamespace(setcbreak=Mock())
        fake_os = SimpleNamespace(name='posix', tcgetpgrp=lambda _: 123, getpgrp=lambda: 123)
        stream = SimpleNamespace(isatty=lambda: True, fileno=lambda: 7)
        return terminal, tty, fake_os, stream

    def test_terminal_settings_restored_on_success_exception_and_interrupt(self):
        for error in (None, RuntimeError('test'), KeyboardInterrupt()):
            with self.subTest(error=error):
                terminal, tty, fake_os, stream = self.terminal_context()
                thread = Mock(ident=1)
                with patch.dict(sys.modules, {'termios': terminal, 'tty': tty}), \
                     patch.object(standalone_keyboard, 'os', fake_os), \
                     patch.object(standalone_keyboard.threading, 'Thread', return_value=thread):
                    control = KeyboardControl(stream)
                    if error is None:
                        with control:
                            pass
                    else:
                        with self.assertRaises(type(error)), control:
                            raise error
                terminal.tcsetattr.assert_called_once_with(7, 0, ['saved'])
                tty.setcbreak.assert_called_once_with(7, when=0)
                thread.join.assert_called_once()
                self.assertTrue(control.closed.is_set())

    def test_partial_setup_failure_restores_terminal(self):
        terminal, tty, fake_os, stream = self.terminal_context()
        thread = Mock(ident=None)
        thread.start.side_effect = RuntimeError('thread unavailable')
        with patch.dict(sys.modules, {'termios': terminal, 'tty': tty}), \
             patch.object(standalone_keyboard, 'os', fake_os), \
             patch.object(standalone_keyboard.threading, 'Thread', return_value=thread):
            with self.assertRaises(RuntimeError):
                with KeyboardControl(stream):
                    self.fail('Setup must fail')
        terminal.tcsetattr.assert_called_once_with(7, 0, ['saved'])

    def test_non_tty_and_background_terminal_are_rejected(self):
        with self.assertRaises(ValueError):
            with KeyboardControl(io.StringIO('S')):
                self.fail('Non-TTY must refuse')
        terminal, tty, fake_os, stream = self.terminal_context()
        fake_os.tcgetpgrp = lambda _: 456
        with patch.dict(sys.modules, {'termios': terminal, 'tty': tty}), \
             patch.object(standalone_keyboard, 'os', fake_os):
            with self.assertRaises(ValueError):
                with KeyboardControl(stream):
                    self.fail('Background process must refuse')
        tty.setcbreak.assert_not_called()


if __name__ == '__main__':
    unittest.main()
