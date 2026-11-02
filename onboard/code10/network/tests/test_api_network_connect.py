import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "uav-api-network-connect"
loader = importlib.machinery.SourceFileLoader("api_network_connect_tested", str(SCRIPT))
spec = importlib.util.spec_from_loader(loader.name, loader)
helper = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = helper
loader.exec_module(helper)


class Clock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Connection:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        pass


def check_report(status=401, failed=None):
    stages = []
    for name in ("code12_tcp", "proxy_tcp", "http_connect", "tls", "api_http"):
        item = {"stage": name, "ok": name != failed}
        if name == "http_connect":
            item["http_status"] = 200 if name != failed else 502
        if name == "api_http":
            item["http_status"] = status
        stages.append(item)
        if name == failed:
            break
    return {"network_reachable": failed is None, "api_authenticated": False,
            "model_call_tested": False, "stages": stages}


class ConnectorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = helper.StateStore(Path(self.temp.name) / "state")
        self.clock = Clock()
        self.commands = []
        self.report = check_report()

    def runner(self, argv, **kwargs):
        self.commands.append((argv, kwargs))
        self.assertNotIn("shell", kwargs)
        self.assertLessEqual(kwargs["timeout"], 6.0)
        if argv[0] == helper.CHECKER:
            return subprocess.CompletedProcess(argv, 0 if self.report["network_reachable"] else 1,
                                               json.dumps(self.report), "")
        self.assertEqual(argv[0], "/usr/bin/systemctl")
        return subprocess.CompletedProcess(argv, 0, "", "")

    def connector(self, connect=None, runner=None, wall_time=None):
        return helper.Connector(self.store, runner=runner or self.runner,
                                connect=connect or (lambda *_args, **_kwargs: Connection()),
                                monotonic=self.clock.monotonic,
                                wall_time=wall_time or (lambda: 1793600000),
                                sleep=self.clock.sleep, clock_id=lambda: "monotonic:test-boot")

    def run_connector(self, connector):
        with self.store.acquire():
            result = connector.run()
        saved = self.store.read()
        self.assertEqual(saved["exit_code"], result)
        self.assertEqual(saved["clock_id"], "monotonic:test-boot")
        self.assertEqual(saved["api_authenticated"], False)
        self.assertEqual(saved["model_call_tested"], False)
        self.assertEqual((self.store.directory / "button-message.txt").read_text().strip(), saved["message"])
        self.assertLessEqual(len(saved["message"].encode("utf-8")), 96)
        return result, saved

    def test_unreachable_code12_keeps_recovery_services_started(self):
        def unavailable(*_args, **_kwargs):
            raise OSError("No route to host")
        code, saved = self.run_connector(self.connector(connect=unavailable))
        self.assertEqual(code, 10)
        self.assertEqual(saved["state"], "code12_unreachable")
        self.assertFalse(saved["network_reachable"])
        self.assertEqual([args[1:] for args, _ in self.commands], [
            ["enable", helper.TUNNEL, helper.TIMER],
            ["start", "--no-block", helper.TUNNEL, helper.TIMER],
            ["start", "--no-block", helper.TIME_SERVICE],
        ])
        all_arguments = " ".join(" ".join(args) for args, _ in self.commands)
        self.assertNotIn("restart", all_arguments)
        self.assertNotIn("uav-microsoft-agent", all_arguments)
        self.assertNotIn(".ssh", all_arguments)
        self.assertNotIn(".env", all_arguments)
        self.assertNotIn("password", all_arguments.lower())

    def test_proxy_wait_is_bounded_and_returns_11(self):
        def probe(address, **_kwargs):
            if address[0] == "127.0.0.1":
                raise ConnectionRefusedError
            return Connection()
        code, saved = self.run_connector(self.connector(connect=probe))
        self.assertEqual(code, 11)
        self.assertEqual(saved["state"], "proxy_unready")
        self.assertLessEqual(saved["duration_ms"], 25000)

    def test_invalid_clock_does_not_bypass_tls_and_returns_12(self):
        code, saved = self.run_connector(self.connector(wall_time=lambda: 700))
        self.assertEqual(code, 12)
        self.assertEqual(saved["state"], "waiting_time")
        self.assertFalse(any(command[0] == helper.CHECKER for command, _ in self.commands))
        self.assertLessEqual(saved["duration_ms"], 25000)

    def test_tls_failure_returns_13_and_never_claims_success(self):
        self.report = check_report(failed="tls")
        code, saved = self.run_connector(self.connector())
        self.assertEqual(code, 13)
        self.assertFalse(saved["network_reachable"])
        self.assertEqual(saved["state"], "tls_failed")

    def test_https_failure_and_proxy_connect_failure_return_14(self):
        for failed in ("http_connect", "api_http"):
            with self.subTest(failed=failed):
                self.report = check_report(failed=failed)
                code, saved = self.run_connector(self.connector())
                self.assertEqual(code, 14)
                self.assertFalse(saved["network_reachable"])

    def test_401_is_transport_success_with_explicitly_unverified_auth(self):
        code, saved = self.run_connector(self.connector())
        self.assertEqual(code, 0)
        self.assertTrue(saved["network_reachable"])
        self.assertEqual(saved["http_status"], 401)
        self.assertFalse(saved["api_authenticated"])
        self.assertIn("未验证", saved["message"])
        checker_command = self.commands[-1][0]
        self.assertEqual(checker_command, [helper.CHECKER, "--url", "https://api.deepseek.com", "--timeout", "3", "--json"])
        self.assertNotIn("Authorization", " ".join(checker_command))

    def test_forged_network_true_without_tls_and_http_is_rejected(self):
        self.report = {"network_reachable": True, "stages": []}
        code, saved = self.run_connector(self.connector())
        self.assertEqual(code, 1)
        self.assertFalse(saved["network_reachable"])

    def test_no_raw_checker_details_or_urls_are_copied_to_status(self):
        self.report["stages"][0]["detail"] = "secret-value-must-not-be-published"
        self.report["secret"] = "secret-value-must-not-be-published"
        _, saved = self.run_connector(self.connector())
        self.assertNotIn("secret-value", json.dumps(saved))

    def test_checker_subprocess_timeout_returns_14(self):
        def timeout_on_checker(argv, **kwargs):
            if argv[0] == helper.CHECKER:
                self.assertLessEqual(kwargs["timeout"], 6)
                raise subprocess.TimeoutExpired(argv, kwargs["timeout"])
            return self.runner(argv, **kwargs)
        code, saved = self.run_connector(self.connector(runner=timeout_on_checker))
        self.assertEqual(code, 14)
        self.assertEqual(saved["state"], "https_timeout")

    def test_service_timeout_returns_1_and_respects_total_budget(self):
        def slow_service(argv, **kwargs):
            self.assertLessEqual(kwargs["timeout"], 6)
            self.clock.sleep(kwargs["timeout"])
            raise subprocess.TimeoutExpired(argv, kwargs["timeout"])
        code, saved = self.run_connector(self.connector(runner=slow_service))
        self.assertEqual(code, 1)
        self.assertLessEqual(saved["duration_ms"], 25000)
        self.assertEqual(saved["state"], "service_timeout")

    def test_successful_slow_services_leave_bounded_end_to_end_budget(self):
        def slow_service(argv, **kwargs):
            self.clock.sleep(min(kwargs["timeout"], 5.9))
            return self.runner(argv, **kwargs)
        code, saved = self.run_connector(self.connector(runner=slow_service))
        self.assertEqual(code, 0)
        self.assertLessEqual(saved["duration_ms"], 25000)
        self.assertTrue(all(options["timeout"] <= 6 for _, options in self.commands))


class StateStoreTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = helper.StateStore(Path(self.temp.name) / "state")

    def test_concurrent_operation_cannot_overwrite_active_status(self):
        with self.store.acquire():
            self.store.publish({"state": "connecting", "message": "API联网：正在连接…"})
            before = (self.store.directory / "button-status.json").read_bytes()
            with self.assertRaises(helper.AlreadyRunning):
                with self.store.acquire():
                    self.fail("concurrent lock unexpectedly acquired")
            self.assertEqual((self.store.directory / "button-status.json").read_bytes(), before)

    def test_status_is_read_only_and_runtime_files_are_private(self):
        with self.store.acquire():
            self.store.publish({"state": "network_ready", "message": "API网络已通，密钥/额度未验证"})
        before = {p.name: (p.stat().st_mtime_ns, p.read_bytes()) for p in self.store.directory.iterdir()}
        self.assertEqual(self.store.read()["state"], "network_ready")
        after = {p.name: (p.stat().st_mtime_ns, p.read_bytes()) for p in self.store.directory.iterdir()}
        self.assertEqual(before, after)
        self.assertEqual(stat.S_IMODE(self.store.directory.stat().st_mode), 0o700)
        for p in self.store.directory.iterdir():
            self.assertEqual(stat.S_IMODE(p.stat().st_mode), 0o600)

    def test_runtime_directory_symlink_and_file_symlinks_are_rejected(self):
        other = Path(self.temp.name) / "other"
        other.mkdir(mode=0o700)
        self.store.directory.symlink_to(other, target_is_directory=True)
        with self.assertRaises(OSError):
            with self.store.acquire():
                self.fail("symlink runtime directory accepted")
        self.store.directory.unlink()
        with self.store.acquire():
            destination = other / "private"
            destination.write_text("do-not-touch")
            (self.store.directory / "button-message.txt").symlink_to(destination)
            with self.assertRaises(OSError):
                self.store.publish({"message": "new"})
            self.assertEqual(destination.read_text(), "do-not-touch")

    def test_lock_symlink_is_rejected(self):
        self.store.validate_directory(create=True)
        other = Path(self.temp.name) / "other"
        other.write_text("do-not-touch")
        (self.store.directory / "connect.lock").symlink_to(other)
        with self.assertRaises(OSError):
            with self.store.acquire():
                self.fail("symlink lock accepted")
        self.assertEqual(other.read_text(), "do-not-touch")

    def test_busy_main_returns_2_without_changing_status(self):
        with self.store.acquire():
            self.store.publish({"state": "connecting", "message": "first"})
            before = (self.store.directory / "button-status.json").read_bytes()
            busy_store = mock.Mock()
            busy_store.acquire.side_effect = helper.AlreadyRunning
            with mock.patch.object(helper.os, "geteuid", return_value=0), \
                    mock.patch.object(helper, "StateStore", return_value=busy_store), \
                    mock.patch("builtins.print"):
                self.assertEqual(helper.main([]), 2)
                busy_store.publish.assert_not_called()
            self.assertEqual((self.store.directory / "button-status.json").read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
