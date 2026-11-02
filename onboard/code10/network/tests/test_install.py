import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


SPEC = importlib.util.spec_from_file_location("deployment", Path(__file__).parents[1] / "install.py")
deployment = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(deployment)


class FakeSystemd:
    def __init__(self, agent_active=False, fail_restart=False):
        self.calls = []
        self.fail_restart = fail_restart
        self.states = {unit: {"enabled": "disabled", "active": "inactive"} for unit in deployment.UNITS}
        self.states[deployment.AGENT]["active"] = "active" if agent_active else "inactive"

    def __call__(self, command, **kwargs):
        args = command[1:]
        self.calls.append(args)
        action = args[0]
        units = [arg for arg in args[1:] if not arg.startswith("--")]
        if action == "restart" and units == [deployment.TUNNEL] and self.fail_restart:
            self.fail_restart = False
            return subprocess.CompletedProcess(command, 1, "", "simulated tunnel failure")
        output = ""
        if action in ("is-active", "is-enabled"):
            output = self.states[units[0]]["active" if action == "is-active" else "enabled"] + "\n"
        elif action in ("start", "restart", "stop", "enable", "disable"):
            for unit in units:
                if action in ("start", "restart", "stop"):
                    self.states[unit]["active"] = "inactive" if action == "stop" else "active"
                else:
                    self.states[unit]["enabled"] = "disabled" if action == "disable" else "enabled"
        return subprocess.CompletedProcess(command, 0, output, "")


class TransactionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.env = self.root / "etc/uav-code12-proxy.env"
        self.env.parent.mkdir(parents=True)
        self.env.write_text("ORIGINAL_PROXY=example\n")
        self.env.chmod(0o640)
        self.unrelated = self.root / "etc/unrelated.conf"
        self.unrelated.write_text("keep\n")

    def tearDown(self):
        self.temp.cleanup()

    def install(self, fake):
        deploy = deployment.Deployment(self.root, runner=fake)
        with contextlib.redirect_stdout(io.StringIO()):
            backup = deploy.install()
        return deploy, backup

    def test_install_and_restore_original_and_absent_files(self):
        fake = FakeSystemd(agent_active=True)
        deploy, backup = self.install(fake)
        self.assertIn("HTTP_PROXY=http://127.0.0.1:18080", self.env.read_text())
        helper = deploy.path("/usr/local/bin/uav-api-network-connect")
        self.assertEqual(helper.read_bytes(), (deployment.PACKAGE / "scripts/uav-api-network-connect").read_bytes())
        self.assertEqual(helper.stat().st_mode & 0o777, 0o755)
        self.assertIn(["start", "--no-block", deployment.TIME_SERVICE], fake.calls)
        manifest = json.loads((backup / "manifest.json").read_text())
        original = next(record for record in manifest["files"] if record["path"] == "/etc/uav-code12-proxy.env")
        self.assertEqual(original["mode"], 0o640)
        with contextlib.redirect_stdout(io.StringIO()):
            deploy.restore(backup)
        self.assertEqual(self.env.read_text(), "ORIGINAL_PROXY=example\n")
        self.assertEqual(self.env.stat().st_mode & 0o777, 0o640)
        self.assertFalse(deploy.path("/usr/local/bin/uav-api-network-check").exists())
        self.assertFalse(helper.exists())
        self.assertEqual(fake.states[deployment.TUNNEL], {"enabled": "disabled", "active": "inactive"})
        self.assertEqual(fake.states[deployment.AGENT]["active"], "active")
        self.assertEqual(self.unrelated.read_text(), "keep\n")
        self.assertFalse(any("psdk" in str(call) for call in fake.calls))

    def test_does_not_start_previously_inactive_agent(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        self.assertNotIn(["restart", deployment.AGENT], fake.calls)
        self.assertEqual(fake.states[deployment.AGENT]["active"], "inactive")

    def test_activation_failure_automatically_rolls_back(self):
        fake = FakeSystemd(fail_restart=True)
        deploy = deployment.Deployment(self.root, runner=fake)
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(deployment.DeploymentError):
                deploy.install()
        self.assertEqual(self.env.read_text(), "ORIGINAL_PROXY=example\n")
        self.assertFalse(deploy.path("/usr/local/bin/uav-api-network-check").exists())
        self.assertEqual(fake.states[deployment.TUNNEL]["active"], "inactive")
        self.assertEqual(self.unrelated.read_text(), "keep\n")

    def test_rejects_edited_backup_target_list_before_touching_services(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        manifest = json.loads((backup / "manifest.json").read_text())
        manifest["files"][0]["path"] = "/etc/unrelated.conf"
        (backup / "manifest.json").write_text(json.dumps(manifest))
        old_count = len(fake.calls)
        with self.assertRaises(deployment.DeploymentError):
            deploy.restore(backup)
        self.assertEqual(len(fake.calls), old_count)
        self.assertEqual(self.unrelated.read_text(), "keep\n")

    def test_refuses_to_overwrite_later_changes_before_touching_services(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        self.env.write_text("A_LATER_DEPLOYMENT=keep\n")
        old_count = len(fake.calls)
        with self.assertRaises(deployment.DeploymentError):
            deploy.restore(backup)
        self.assertEqual(len(fake.calls), old_count)
        self.assertEqual(self.env.read_text(), "A_LATER_DEPLOYMENT=keep\n")

    def test_normal_rollback_refuses_deleted_target(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        self.env.unlink()
        old_count = len(fake.calls)
        with self.assertRaises(deployment.DeploymentError):
            deploy.restore(backup)
        self.assertEqual(len(fake.calls), old_count)

    def test_refuses_corrupted_original_backup(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        (backup / "files/etc/uav-code12-proxy.env").write_text("CORRUPTED\n")
        old_count = len(fake.calls)
        with self.assertRaises(deployment.DeploymentError):
            deploy.restore(backup)
        self.assertEqual(len(fake.calls), old_count)

    def test_legacy_backup_does_not_remove_later_api_connect_helper(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        helper = deploy.path("/usr/local/bin/uav-api-network-connect")
        helper.write_text("later-helper\n")
        manifest = json.loads((backup / "manifest.json").read_text())
        manifest["version"] = 1
        manifest["files"] = [record for record in manifest["files"]
                             if record["path"] != "/usr/local/bin/uav-api-network-connect"]
        (backup / "manifest.json").write_text(json.dumps(manifest))
        with contextlib.redirect_stdout(io.StringIO()):
            deploy.restore(backup)
        self.assertEqual(helper.read_text(), "later-helper\n")
        self.assertEqual(self.env.read_text(), "ORIGINAL_PROXY=example\n")

    def test_current_backup_cannot_omit_api_connect_helper(self):
        fake = FakeSystemd()
        deploy, backup = self.install(fake)
        manifest = json.loads((backup / "manifest.json").read_text())
        self.assertEqual(manifest["version"], 2)
        manifest["files"] = [record for record in manifest["files"]
                             if record["path"] != "/usr/local/bin/uav-api-network-connect"]
        (backup / "manifest.json").write_text(json.dumps(manifest))
        old_count = len(fake.calls)
        with self.assertRaises(deployment.DeploymentError):
            deploy.restore(backup)
        self.assertEqual(len(fake.calls), old_count)


if __name__ == "__main__":
    unittest.main()
