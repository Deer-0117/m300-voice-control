"""Actual voice/mission functions with private files and substituted API/ASR.

No SDK, camera, production task file, network socket, or credential is used.
"""
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

RUNTIME = Path(__file__).resolve().parents[2] / "recognition/runtime"
sys.path.insert(0, str(RUNTIME))
voice = importlib.import_module("uav_voice_command")
natural = importlib.import_module("uav_natural_language")
mission = importlib.import_module("balloon_mission")

ACTIONS = [dict(capability="strike_balloon", parameters=dict(color="red")),
           dict(capability="strike_balloon", parameters=dict(color="green"))]


class ApiVoiceTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        names = ("CANDIDATE_FILE", "SESSION_STATE", "LOCK_FILE", "SESSION_FILE", "READY_FILE",
                 "MESSAGE_FILE", "MISSION_TOKEN_FILE", "MISSION_COMMIT_FILE", "PLAN_FILE", "PLAN_STATE_FILE")
        patcher = patch.multiple(voice, **{name: self.root / getattr(voice, name).name for name in names})
        patcher.start(); self.addCleanup(patcher.stop)
        patcher = patch.multiple(natural, PLAN_FILE=voice.PLAN_FILE, STATE_FILE=voice.PLAN_STATE_FILE,
                                EVENT_LOG=self.root / "events.jsonl", PLAN_LOCK_FILE=self.root / "plan.lock",
                                ARMED_FILE=self.root / "armed", TRIGGER_FILE=self.root / "trigger",
                                STOP_REQUEST_FILE=self.root / "stop")
        patcher.start(); self.addCleanup(patcher.stop)
        patcher = patch.multiple(mission, **{name: self.root / getattr(mission, name).name for name in
                                 ("TOKEN", "REQUEST", "ACK", "READY", "COMMIT", "HEALTH", "MESSAGE",
                                  "STATUS", "RECORD_REQUEST", "RECORD_ACK", "STOP", "PLAN", "STATE", "EVENTS")})
        patcher.start(); self.addCleanup(patcher.stop)
        patcher = patch.object(voice, "clock_id", return_value="mono-v1:test-boot")
        patcher.start(); self.addCleanup(patcher.stop)
        self.clock = 1000
        patcher = patch.object(voice, "now_ms", side_effect=lambda: self.clock)
        patcher.start(); self.addCleanup(patcher.stop)
        patcher = patch.dict(os.environ, {"UAV_AGENT_AUTH_TOKEN": "test-credential-never-real"})
        patcher.start(); self.addCleanup(patcher.stop)
        self.session = "recording-one"
        self.token = "mission-one"
        natural.ARMED_FILE.write_text("armed")
        voice.MISSION_TOKEN_FILE.write_text(self.token)
        self.audio = self.root / "test.pcm"
        self.audio.write_bytes(b"\0\0" * 160)
        voice.begin_session(self.session)

    def response(self, transcript="先撞红色再撞绿色"):
        return dict(ok=True, result=dict(proposal_id="a" * 32, status="proposed", command=transcript,
                                        plan=dict(version=2, actions=ACTIONS)))

    def candidate(self, defer=False):
        with patch.object(voice, "post_json", return_value=self.response()) as post:
            result = voice.create_candidate("先撞红色再撞绿色", self.session, defer)
        return result, post

    def test_normal_recognition_always_calls_real_api_adapter_not_rules(self):
        with patch.object(voice, "transcribe", return_value="先撞红色再撞绿色"), \
             patch.object(voice, "post_json", return_value=self.response()) as post, \
             patch.object(natural, "local_parse", side_effect=AssertionError("rules must not run")):
            candidate = voice.recognize(self.audio, self.session)
        post.assert_called_once()
        self.assertTrue(post.call_args.args[0].endswith("/interpret"))
        self.assertEqual(candidate["kind"], "api")
        self.assertEqual(candidate["actions"], ACTIONS)
        self.assertEqual(voice.READY_FILE.read_text().strip(), self.session)
        self.assertFalse(voice.PLAN_FILE.exists())

    def test_deferred_new_voice_calls_api_only_at_second_confirmation(self):
        candidate, post = self.candidate(defer=True)
        post.assert_not_called()
        self.assertEqual(candidate["kind"], "transcript")
        with patch.object(voice, "post_json", return_value=self.response()) as post:
            result = voice.confirm(self.session, self.token, allow_deferred=True)
        post.assert_called_once()
        self.assertTrue(post.call_args.args[0].endswith("/interpret"))
        self.assertEqual(result["plan"]["actions"], ACTIONS)
        self.assertFalse(voice.MISSION_COMMIT_FILE.exists())

    def test_existing_candidate_confirmation_stores_locally_without_remote_submit(self):
        self.candidate()
        with patch.object(voice, "post_json", side_effect=AssertionError("no submit endpoint")):
            result = voice.confirm(self.session, self.token)
        self.assertEqual(result["api_decision"]["proposal_id"], "a" * 32)
        self.assertEqual(json.loads(voice.PLAN_FILE.read_text())["source"], "api_voice_confirmed")
        self.assertFalse(voice.CANDIDATE_FILE.exists())
        self.assertFalse(voice.READY_FILE.exists())
        self.assertEqual(voice.status(self.session)["status"], "confirmed")
        with self.assertRaises(voice.VoiceError):
            voice.confirm(self.session, self.token)

    def test_c_published_token_bootstraps_state_without_recreating_missing_token(self):
        voice.cancel()
        voice.SESSION_FILE.write_text("from-c\n")
        with patch.object(voice, "transcribe", return_value="先撞红色再撞绿色"):
            result = voice.recognize(self.audio, "from-c", defer_api=True)
        self.assertEqual(result["session"], "from-c")
        voice.cancel()
        with self.assertRaisesRegex(voice.VoiceError, "取消"):
            voice.recognize(self.audio, "from-c", defer_api=True)
        self.assertFalse(voice.SESSION_FILE.exists())

    def test_stale_ready_marker_without_c_session_never_authorizes_execution(self):
        self.candidate()
        voice.SESSION_FILE.unlink()  # Synchronous C Stop/clear barrier.
        self.assertTrue(voice.READY_FILE.exists())  # Possible killed-worker residue.
        self.assertFalse(voice.status(self.session)["ready"])
        with patch.object(voice, "store_plan") as save:
            with self.assertRaisesRegex(voice.VoiceError, "取消"):
                voice.confirm(self.session, self.token)
        save.assert_not_called()
        self.assertFalse(voice.PLAN_FILE.exists())

    def test_c_stop_during_candidate_publication_cleans_late_ready(self):
        original = voice.atomic_json
        def publish(path, value):
            if path == voice.CANDIDATE_FILE:
                voice.SESSION_FILE.unlink()
            original(path, value)
        with patch.object(voice, "atomic_json", side_effect=publish):
            with self.assertRaisesRegex(voice.VoiceError, "取消"):
                voice.create_candidate("新录音", self.session, defer_api=True)
        self.assertFalse(voice.CANDIDATE_FILE.exists())
        self.assertFalse(voice.READY_FILE.exists())
        self.assertFalse(voice.SESSION_FILE.exists())

    def test_stale_worker_cannot_bootstrap_over_a_new_session(self):
        voice.SESSION_FILE.write_text("new-c-token")
        with self.assertRaisesRegex(voice.VoiceError, "变更"):
            voice.recognize(self.audio, self.session, defer_api=True)
        self.assertEqual(voice.SESSION_FILE.read_text(), "new-c-token")

    def test_cancel_during_asr_rejects_before_api(self):
        def asr(_path):
            voice.cancel(self.session)
            return "先撞红色再撞绿色"
        with patch.object(voice, "transcribe", side_effect=asr), \
             patch.object(voice, "post_json") as post:
            with self.assertRaises(voice.VoiceError):
                voice.recognize(self.audio, self.session)
        post.assert_not_called()
        self.assertFalse(voice.CANDIDATE_FILE.exists())

    def test_cancel_during_api_cannot_publish_late_candidate(self):
        def api(*_args):
            voice.cancel(self.session)
            return self.response()
        with patch.object(voice, "post_json", side_effect=api):
            with self.assertRaises(voice.VoiceError):
                voice.create_candidate("先撞红色再撞绿色", self.session)
        self.assertFalse(voice.CANDIDATE_FILE.exists())
        self.assertFalse(voice.READY_FILE.exists())

    def test_old_api_response_does_not_clear_new_session_candidate(self):
        entered = threading.Event(); release = threading.Event(); errors = []
        def api(*_args):
            entered.set()
            self.assertTrue(release.wait(2))
            return self.response()
        def old():
            try:
                voice.create_candidate("先撞红色再撞绿色", self.session)
            except voice.VoiceError as error:
                errors.append(error)
        with patch.object(voice, "post_json", side_effect=api):
            thread = threading.Thread(target=old); thread.start()
            self.assertTrue(entered.wait(2))
            voice.begin_session("new-session")
            voice.create_candidate("新录音", "new-session", defer_api=True)
            release.set(); thread.join(3)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertEqual(voice.status("new-session")["transcript"], "新录音")

    def test_candidate_ttl_future_and_boot_checks(self):
        for changed in ({"created_ms": -500000}, {"created_ms": 1001}, {"clock_id": "other-boot"}):
            with self.subTest(changed=changed):
                voice.begin_session(self.session)
                self.candidate()
                value = voice.read_object(voice.CANDIDATE_FILE); value.update(changed)
                voice.atomic_json(voice.CANDIDATE_FILE, value)
                with self.assertRaises(voice.VoiceError):
                    voice.confirm(self.session, self.token)
                self.assertFalse(voice.PLAN_FILE.exists())

    def test_old_local_candidate_cannot_execute(self):
        self.candidate()
        value = voice.read_object(voice.CANDIDATE_FILE); value["kind"] = "local"
        voice.atomic_json(voice.CANDIDATE_FILE, value)
        with self.assertRaisesRegex(voice.VoiceError, "不是API"):
            voice.confirm(self.session, self.token)
        self.assertFalse(voice.PLAN_FILE.exists())

    def test_plain_execute_rejects_deferred_new_recording(self):
        self.candidate(defer=True)
        with patch.object(voice, "post_json") as post:
            with self.assertRaisesRegex(voice.VoiceError, "新增API"):
                voice.confirm(self.session, self.token)
        post.assert_not_called()

    def test_confirm_requires_current_mission_token_and_uncommitted_gate(self):
        self.candidate()
        for value in (None, "old-token"):
            with self.assertRaises(voice.VoiceError):
                voice.confirm(self.session, value)
        voice.MISSION_COMMIT_FILE.write_text(self.token)
        with self.assertRaises(voice.VoiceError):
            voice.confirm(self.session, self.token)
        self.assertFalse(voice.PLAN_FILE.exists())

    def test_cancel_during_deferred_interpret_never_writes_plan(self):
        self.candidate(defer=True)
        def api(*_args):
            voice.SESSION_FILE.unlink()
            voice.MISSION_TOKEN_FILE.unlink()
            return self.response()
        with patch.object(voice, "post_json", side_effect=api):
            with self.assertRaises(voice.VoiceError):
                voice.confirm(self.session, self.token, allow_deferred=True)
        self.assertFalse(voice.PLAN_FILE.exists())

    def test_cancel_after_local_write_removes_only_own_plan(self):
        self.candidate()
        original = voice.store_plan
        def save(*args):
            plan = original(*args)
            voice.SESSION_FILE.unlink()
            return plan
        with patch.object(voice, "store_plan", side_effect=save):
            with self.assertRaises(voice.VoiceError):
                voice.confirm(self.session, self.token)
        self.assertFalse(voice.PLAN_FILE.exists())
        self.assertFalse(voice.PLAN_STATE_FILE.exists())
        self.assertFalse(voice.MISSION_COMMIT_FILE.exists())

    def test_conditional_cancel_cannot_delete_new_recording(self):
        voice.begin_session("next-session")
        voice.create_candidate("新录音", "next-session", defer_api=True)
        voice.cancel(self.session)
        self.assertTrue(voice.status("next-session")["ready"])

    def test_api_failure_never_uses_local_parser_or_old_candidate(self):
        self.candidate()
        with patch.object(voice, "post_json", side_effect=voice.VoiceError("API请求失败")), \
             patch.object(natural, "local_parse", side_effect=AssertionError("rules must not run")):
            with self.assertRaisesRegex(voice.VoiceError, "API请求失败"):
                voice.create_candidate("先撞红色再撞绿色", self.session)
        self.assertFalse(voice.CANDIDATE_FILE.exists())
        self.assertFalse(voice.PLAN_FILE.exists())

    def test_api_unknown_action_or_mismatched_command_fails_closed(self):
        for value in (dict(ok=True, result={}), self.response("other-command"),
                      dict(ok=True, result=dict(self.response()["result"], plan=dict(actions=[dict(capability="takeoff", parameters={})])))):
            with self.subTest(value=value):
                voice.begin_session(self.session)
                with patch.object(voice, "post_json", return_value=value):
                    with self.assertRaises(voice.VoiceError):
                        voice.create_candidate("先撞红色再撞绿色", self.session)
                self.assertFalse(voice.CANDIDATE_FILE.exists())

    def test_session_expiry_and_reboot_fail_closed(self):
        self.candidate()
        state = voice.read_object(voice.SESSION_STATE); state["clock_id"] = "old-boot"
        voice.atomic_json(voice.SESSION_STATE, state)
        self.assertFalse(voice.status(self.session)["ready"])
        with self.assertRaises(voice.VoiceError):
            voice.confirm(self.session, self.token)

    def test_display_is_utf8_bounded(self):
        voice.write_message("字" * 200)
        self.assertLessEqual(len(voice.MESSAGE_FILE.read_bytes().rstrip(b"\n")), 240)
        voice.MESSAGE_FILE.read_text(encoding="utf-8")

    def test_api_new_prepare_waits_for_exact_executor_receipt_and_never_commits(self):
        self.candidate(defer=True)
        def acknowledge(request, timeout=12):
            self.assertEqual(request["mode"], "api_new")
            self.assertEqual(request["motion_source"], "api")
            self.assertEqual(request["token"], self.token)
            plan = mission.read_json(mission.PLAN)
            self.assertEqual(plan["actions"], ACTIONS)
            self.assertEqual(plan["motion_source"], "api")
            self.assertEqual(plan["api_decision"]["source"], "agent_interpret")
        with patch.object(voice, "post_json", return_value=self.response()) as post, \
             patch.object(mission, "wait_ack", side_effect=acknowledge):
            mission.prepare("api_new", self.token, False, self.session, api_motion=True)
        post.assert_called_once()
        self.assertEqual(mission.READY.read_text().strip(), self.token)
        self.assertFalse(mission.COMMIT.exists())
        self.assertEqual(mission.read_json(mission.PLAN)["source"], "widget_api_new")

    def test_api_errors_do_not_echo_remote_response_or_credentials(self):
        import urllib.error
        error = urllib.error.HTTPError("http://local", 401, "secret_key", {}, None)
        with patch.object(voice.urllib.request, "urlopen", side_effect=error):
            with self.assertRaises(voice.VoiceError) as caught:
                voice.post_json("http://local/interpret", {}, "secret_key")
        self.assertEqual(str(caught.exception), "API请求失败：HTTP 401")


if __name__ == "__main__":
    unittest.main()
