"""Detector switching must not start SDK hosts or previously stopped workers."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import switch_detector as switch


class DetectorSwitchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / "detector_config.json"
        self.settings = {"active_detector": "yolo", "pixel_config": "pixel_callable_module/config.json"}
        self.config.write_text(json.dumps(self.settings))
        self.gates = tuple(self.root / path.name for path in switch.ACTIVE_GATES)
        self.states = {unit: {"ActiveState": "inactive", "MainPID": "0"}
                       for unit in (*switch.SDK_UNITS, switch.WORKER, switch.TRACKER)}
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        for name, value in (("ROOT", self.root), ("DEFAULT_CONFIG", self.config),
                            ("RUNTIME", self.root / "detector_runtime.json"), ("ACTIVE_GATES", self.gates)):
            self.stack.enter_context(patch.object(switch, name, value))
        self.stack.enter_context(patch.object(switch, "state", side_effect=lambda unit: self.states[unit].copy()))
        self.stack.enter_context(patch.object(switch, "read_selection", side_effect=lambda: json.loads(self.config.read_text())))
        self.stack.enter_context(patch("os.geteuid", return_value=0))
        self.calls = []

        def control(action, *units):
            self.calls.append((action, *units))
            for unit in units:
                self.states[unit] = {"ActiveState": "active" if action == "start" else "inactive",
                                     "MainPID": "100" if action == "start" else "0"}
        self.stack.enter_context(patch.object(switch, "control", side_effect=control))
        self.ready = self.stack.enter_context(patch.object(switch, "wait_ready", return_value={"active_detector": "yolo"}))

    def run_main(self, *args):
        with patch.object(sys, "argv", ["switch_detector.py", *args]), contextlib.redirect_stdout(io.StringIO()):
            switch.main()

    def test_each_sdk_host_blocks_switch_without_service_changes(self):
        for unit in switch.SDK_UNITS:
            for status in ("active", "activating", "deactivating", "failed"):
                with self.subTest(unit=unit, status=status):
                    self.states[unit] = {"ActiveState": status, "MainPID": "0"}
                    with self.assertRaises(SystemExit):
                        self.run_main("yolo")
                    self.assertEqual(self.calls, [])
            self.states[unit] = {"ActiveState": "inactive", "MainPID": "0"}

    def test_inactive_host_with_remaining_pid_is_rejected(self):
        self.states[switch.SDK_UNITS[1]]["MainPID"] = "100"
        with self.assertRaises(SystemExit):
            self.run_main("yolo")
        self.assertEqual(self.calls, [])

    def test_all_active_gates_and_dangling_symlinks_block_switch(self):
        for gate in self.gates:
            for symlink in (False, True):
                with self.subTest(gate=gate.name, symlink=symlink):
                    if symlink:
                        gate.symlink_to(self.root / "absent")
                    else:
                        gate.write_text("task")
                    with self.assertRaises(SystemExit):
                        self.run_main("yolo")
                    gate.unlink()
                    self.assertEqual(self.calls, [])

    def test_status_is_read_only_without_sudo_even_with_active_sdk(self):
        self.states[switch.SDK_UNITS[1]] = {"ActiveState": "active", "MainPID": "123"}
        with patch("os.geteuid", return_value=1000):
            self.run_main("--status")
        self.assertEqual(self.calls, [])
        self.assertFalse((self.root / "detector_switch.lock").exists())

    def test_previously_inactive_workers_remain_stopped(self):
        self.run_main("yolo")
        self.assertEqual(self.calls, [])
        self.ready.assert_not_called()

    def test_only_previously_active_worker_restarts(self):
        self.states[switch.WORKER] = {"ActiveState": "active", "MainPID": "1"}
        self.run_main("yolo")
        self.assertEqual(self.calls, [("stop", switch.WORKER), ("start", switch.WORKER)])

    def test_both_previously_active_vision_services_restart_in_order(self):
        for unit in (switch.WORKER, switch.TRACKER):
            self.states[unit] = {"ActiveState": "active", "MainPID": "1"}
        self.run_main("yolo")
        self.assertEqual(self.calls, [("stop", switch.TRACKER, switch.WORKER),
                                     ("start", switch.WORKER), ("start", switch.TRACKER)])

    def test_tracker_alone_or_transitional_worker_blocks_switch(self):
        self.states[switch.TRACKER] = {"ActiveState": "active", "MainPID": "1"}
        with self.assertRaises(SystemExit):
            self.run_main("yolo")
        self.states[switch.TRACKER] = {"ActiveState": "inactive", "MainPID": "0"}
        self.states[switch.WORKER] = {"ActiveState": "activating", "MainPID": "1"}
        with self.assertRaises(SystemExit):
            self.run_main("yolo")
        self.assertEqual(self.calls, [])

    def test_failure_restores_configuration_and_previously_running_worker(self):
        self.states[switch.WORKER] = {"ActiveState": "active", "MainPID": "1"}
        self.ready.side_effect = [RuntimeError("warmup failed"), {"active_detector": "yolo"}]
        with self.assertRaisesRegex(RuntimeError, "warmup failed"):
            self.run_main("yolo")
        self.assertEqual(json.loads(self.config.read_text()), self.settings)
        self.assertEqual(self.calls, [("stop", switch.WORKER), ("start", switch.WORKER),
                                     ("stop", switch.WORKER), ("start", switch.WORKER)])


if __name__ == "__main__":
    unittest.main()
