"""Offline script completion/cancellation checks; no native robot construction."""
import ast
import io
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
import urllib.error
import urllib.request
import json
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'backend'))
from app import Scripts
from controller import Controller


class FinishedProcess:
    def __init__(self, output='', code=0):
        self.stdout = io.StringIO(output)
        self.code = code
    def wait(self, **kwargs): return self.code
    def poll(self): return self.code


class ScriptLifecycle(unittest.TestCase):
    def setUp(self):
        self.c = Controller()
        self.c.q['left'][0] = .15
        self.c.q['right'][0] = -.1
        self.c.grip['left'] = .7
        self.c.enable()
        self.c.script_active = True
        self.c.script_sides = ['left']
        self.scripts = Scripts(self.c)
        self.process = FinishedProcess('example complete\n')
        self.scripts.process = self.process
        self.scripts.token = 'current-token'
        self.event = self.scripts.cancel_event

    def tick(self, delay):
        self.c.heartbeat = time.monotonic()
        self.c.tick(delay)
        return False

    def watch(self):
        self.scripts.watch(self.process, self.event, ('left',))

    def test_normal_completion_homes_only_selected_joints(self):
        with patch.object(self.event, 'wait', side_effect=self.tick): self.watch()
        self.assertLessEqual(abs(self.c.q['left'][0]), .02)
        self.assertEqual(self.c.q['right'][0], -.1)
        self.assertEqual(self.c.grip['left'], .7)
        self.assertFalse(self.c.enabled)
        self.assertFalse(self.c.script_active)
        self.assertIn('[回零完成]', '\n'.join(self.scripts.output))

    def test_user_stop_before_exit_does_not_home(self):
        before = self.c.q['left'].copy()
        self.scripts.stop()
        self.watch()
        np.testing.assert_array_equal(self.c.q['left'], before)
        self.assertNotIn('[正常结束]', '\n'.join(self.scripts.output))
        self.assertFalse(self.c.enabled)

    def test_stop_during_homing_revokes_further_commands(self):
        stopped_position = []
        def stop_during_wait(delay):
            self.tick(delay)
            self.scripts.stop()
            stopped_position.append(self.c.q['left'].copy())
            return True
        with patch.object(self.event, 'wait', side_effect=stop_during_wait): self.watch()
        self.c.tick(.1)
        np.testing.assert_array_equal(self.c.q['left'], stopped_position[0])
        self.assertGreater(abs(self.c.q['left'][0]), .02)
        self.assertFalse(self.c.enabled)
        self.assertEqual(self.c.raw['left'], {})

    def test_caught_sdk_error_with_zero_exit_does_not_home(self):
        self.process.stdout = io.StringIO('错误: 反馈失败\n')
        before = self.c.q['left'].copy()
        self.watch()
        np.testing.assert_array_equal(self.c.q['left'], before)
        self.assertNotIn('[正常结束]', '\n'.join(self.scripts.output))

    def test_heartbeat_loss_cannot_restart_control(self):
        self.c.heartbeat = time.monotonic()-3
        before = self.c.q['left'].copy()
        self.watch()
        np.testing.assert_array_equal(self.c.q['left'], before)
        self.assertFalse(self.c.enabled)
        self.assertIn('[回零取消]', '\n'.join(self.scripts.output))

    def test_old_watcher_cannot_stop_new_script(self):
        new_process = FinishedProcess()
        def replace_run(delay):
            self.scripts.stop()
            self.c.enable()
            self.c.script_active = True
            self.scripts.process = new_process
            self.scripts.token = 'new-token'
            self.scripts.cancel_event = threading.Event()
            return True
        with patch.object(self.event, 'wait', side_effect=replace_run): self.watch()
        self.assertTrue(self.c.enabled)
        self.assertTrue(self.c.script_active)
        self.assertEqual(self.scripts.token, 'new-token')

    def test_revoked_cleanup_is_idempotent_but_motion_aborts(self):
        # Extract rpc without executing the runner's top-level SDK script.
        source = Path(__file__).resolve().parents[1] / 'backend/script_runner.py'
        node = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'rpc')
        scope = {'urllib': __import__('urllib'), 'json': json, 'URL': 'http://localhost:1', 'TOKEN': 'old'}
        exec(compile(ast.Module(body=[node], type_ignores=[]),str(source),'exec'),scope)
        error = urllib.error.HTTPError('http://localhost:1',403,'revoked',{},io.BytesIO(b'{}'))
        with patch('urllib.request.urlopen', side_effect=error):
            self.assertEqual(scope['rpc']({'side':'left','stop':True}), {'success':True})
            with self.assertRaises(KeyboardInterrupt): scope['rpc']({'side':'left','commands':{'0':['pos_vel_MAXtqe',0,.25,10]}})


if __name__ == '__main__': unittest.main()
