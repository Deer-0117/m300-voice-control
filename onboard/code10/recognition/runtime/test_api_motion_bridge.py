"""Exercise the pipe and HTTPS contracts without network access or an SDK."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import api_motion_bridge as bridge


SETTINGS = (
    "UAV_MOTION_API_URL=https://motion.example/v1/chat/completions\n"
    "UAV_MOTION_API_MODEL=motion-model\n"
    "UAV_MOTION_API_KEY=fixture-secret\n"
)
PROVIDER_ID = "motion-response:1"
MEASUREMENTS = {
    "target": {"u": 0.4, "v": 0.6, "width": 0.11, "height": 0.2},
    "aircraft": {
        "altitude_agl_m": 2.5, "vel_n_mps": 0.01, "vel_e_mps": -0.02,
        "vel_u_mps": 0.0, "yaw_deg": 150.0,
    },
}
MOTION = {"duration_ms": 150, "x_mps": 0.18, "y_mps": -0.24, "z_mps": 0.12, "yaw_rate_degps": -12}


def provider_reply(content=None, provider_id=PROVIDER_ID, finish_reason="stop"):
    return json.dumps({"id": provider_id, "choices": [{
        "message": {"content": json.dumps(MOTION) if content is None else content},
        "finish_reason": finish_reason,
    }]}).encode()


def request_line(seq):
    return json.dumps({"seq": seq, **MEASUREMENTS}).encode() + b"\n"


class ApiMotionBridgeTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.config_path = Path(temporary.name) / "motion.env"
        self.write_config(SETTINGS)

    def write_config(self, text):
        self.config_path.write_text(text, encoding="ascii")
        self.config_path.chmod(0o600)

    def test_configuration_check_only_reports_success_by_exit_status(self):
        stdout, stderr = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr), \
                patch.object(bridge, "serve") as serve:
            self.assertEqual(bridge.main(["--check-config"], config_path=self.config_path), 0)
        self.assertEqual((stdout.getvalue(), stderr.getvalue()), ("", ""))
        serve.assert_not_called()

    def test_configuration_rejects_insecure_or_ambiguous_settings(self):
        for text in (
            SETTINGS.replace("https://", "http://"),
            SETTINGS.replace("motion.example", "key@motion.example"),
            SETTINGS.replace("completions", "completions?key=credential"),
            SETTINGS.replace("completions", "completions#fragment"),
            SETTINGS.replace("fixture-secret", ""),
            SETTINGS + "UAV_MOTION_API_MODEL=other\n",
            SETTINGS + "HTTPS_PROXY=http://127.0.0.1:18080/path\n",
        ):
            with self.subTest(text=text):
                self.write_config(text)
                with self.assertRaises(ValueError):
                    bridge.load_config(self.config_path)
        self.write_config(SETTINGS)
        self.config_path.chmod(0o644)
        with self.assertRaises(ValueError):
            bridge.load_config(self.config_path)
        link = self.config_path.with_name("symlink.env")
        link.symlink_to(self.config_path)
        with self.assertRaises(OSError):
            bridge.load_config(link)

    def test_https_posts_measurements_and_preserves_every_provider_axis(self):
        self.write_config(SETTINGS + "HTTPS_PROXY=http://127.0.0.1:18080\n")
        config = bridge.load_config(self.config_path)
        connection = MagicMock()
        response = connection.getresponse.return_value.__enter__.return_value
        response.status, response.read.return_value = 200, provider_reply()
        with patch.object(bridge.http.client, "HTTPSConnection", return_value=connection) as https:
            result = bridge.request_motion(config, {"seq": 9, **MEASUREMENTS})
        https.assert_called_once_with("127.0.0.1", 18080, timeout=bridge.HTTP_TIMEOUT_SECONDS)
        connection.set_tunnel.assert_called_once_with("motion.example", 443)
        method, path, raw_body, headers = connection.request.call_args.args
        self.assertEqual((method, path), ("POST", "/v1/chat/completions"))
        self.assertEqual(headers["Authorization"], "Bearer fixture-secret")
        body = json.loads(raw_body)
        self.assertEqual(body["model"], "motion-model")
        self.assertNotIn("thinking", body)
        self.assertEqual(json.loads(body["messages"][1]["content"]), MEASUREMENTS)
        self.assertEqual(result, {"seq": 9, **MOTION, "provider_id": PROVIDER_ID})
        response.read.assert_called_once_with(bridge.PROVIDER_LIMIT + 1)
        connection.close.assert_called_once_with()

    def test_deepseek_requests_disable_thinking(self):
        self.write_config(SETTINGS.replace("motion.example", "api.deepseek.com"))
        config = bridge.load_config(self.config_path)
        connection = MagicMock()
        response = connection.getresponse.return_value.__enter__.return_value
        response.status, response.read.return_value = 200, provider_reply()
        with patch.object(bridge.http.client, "HTTPSConnection", return_value=connection):
            bridge.request_motion(config, {"seq": 1, **MEASUREMENTS})
        body = json.loads(connection.request.call_args.args[2])
        self.assertEqual(body["thinking"], {"type": "disabled"})

    def test_http_error_or_oversized_reply_closes_connection(self):
        config = bridge.load_config(self.config_path)
        for status, data in ((302, b"redirect"), (401, b"secret"), (200, b"x" * (bridge.PROVIDER_LIMIT + 1))):
            with self.subTest(status=status):
                connection = MagicMock()
                response = connection.getresponse.return_value.__enter__.return_value
                response.status, response.read.return_value = status, data
                with patch.object(bridge.http.client, "HTTPSConnection", return_value=connection) as https:
                    with self.assertRaises(ValueError):
                        bridge.request_motion(config, {"seq": 1, **MEASUREMENTS})
                https.assert_called_once_with("motion.example", None, timeout=bridge.HTTP_TIMEOUT_SECONDS)
                connection.set_tunnel.assert_not_called()
                connection.request.assert_called_once()
                connection.close.assert_called_once_with()
                if status != 200:
                    response.read.assert_not_called()

    def test_provider_id_cannot_echo_the_motion_key(self):
        config = bridge.load_config(self.config_path)
        connection = MagicMock()
        response = connection.getresponse.return_value.__enter__.return_value
        response.status = 200
        response.read.return_value = provider_reply(provider_id=f"receipt-{config.key}-echo")
        output = io.BytesIO()
        with patch.object(bridge.http.client, "HTTPSConnection", return_value=connection):
            with self.assertRaisesRegex(ValueError, "^invalid provider receipt$"):
                bridge.serve(config, io.BytesIO(request_line(1)), output)
        self.assertEqual(output.getvalue(), b"")

    def test_absolute_deadline_interrupts_a_stalled_response(self):
        config = bridge.load_config(self.config_path)
        connection = MagicMock()
        response = connection.getresponse.return_value.__enter__.return_value
        response.status = 200
        response.read.side_effect = lambda _limit: time.sleep(0.2)
        with patch.object(bridge.http.client, "HTTPSConnection", return_value=connection), \
                patch.object(bridge, "HTTP_TIMEOUT_SECONDS", 0.02):
            with self.assertRaises(TimeoutError):
                bridge.request_motion(config, {"seq": 1, **MEASUREMENTS})
        connection.close.assert_called_once_with()

    def test_provider_rejects_invalid_numbers_fields_and_receipts(self):
        content = json.dumps(MOTION)
        for bad in (
            provider_reply(content.replace('"x_mps": 0.18', '"x_mps": 0.18, "x_mps": 0.2')),
            provider_reply(json.dumps({**MOTION, "duration_ms": 150.0})),
            provider_reply(json.dumps({**MOTION, "duration_ms": 200})),
            provider_reply(json.dumps({**MOTION, "x_mps": 0.301})),
            provider_reply(json.dumps({**MOTION, "x_mps": 0.30, "y_mps": 0.17})),
            provider_reply(json.dumps({**MOTION, "y_mps": True})),
            provider_reply(json.dumps({**MOTION, "z_mps": float("nan")})),
            provider_reply(content.replace("-12", "-1e999")),
            provider_reply(json.dumps({**MOTION, "seq": 1})),
            provider_reply(provider_id="unsafe ID"),
            provider_reply(finish_reason="length"),
            provider_reply().replace(b'"id": ', b'"id":"other", "id": '),
        ):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    bridge.decode_provider_reply(bad)

    def test_pipe_accepts_multiple_requests_but_rejects_replay(self):
        config = bridge.load_config(self.config_path)
        output = io.BytesIO()
        with patch.object(bridge, "request_motion", side_effect=lambda _cfg, request: {
            "seq": request["seq"], **MOTION, "provider_id": f"response-{request['seq']}",
        }) as call:
            with self.assertRaises(ValueError):
                bridge.serve(config, io.BytesIO(request_line(1) + request_line(2) + request_line(2)), output)
        self.assertEqual(call.call_count, 2)
        self.assertEqual([json.loads(line)["seq"] for line in output.getvalue().splitlines()], [1, 2])

    def test_pipe_rejects_reused_provider_ids_across_other_replies(self):
        config = bridge.load_config(self.config_path)
        output = io.BytesIO()
        replies = [{"seq": seq, **MOTION, "provider_id": ident} for seq, ident in
                   enumerate(("first", "second", "first"), start=1)]
        with patch.object(bridge, "request_motion", side_effect=replies):
            with self.assertRaisesRegex(ValueError, "repeated provider receipt"):
                bridge.serve(config, io.BytesIO(b"".join(request_line(i) for i in range(1, 4))), output)
        self.assertEqual([json.loads(line)["provider_id"] for line in output.getvalue().splitlines()],
                         ["first", "second"])

    def test_provider_receipt_history_is_bounded(self):
        config = bridge.load_config(self.config_path)
        output = io.BytesIO()
        replies = [{"seq": seq, **MOTION, "provider_id": f"response-{seq}"} for seq in (1, 2)]
        with patch.object(bridge, "MAX_PROVIDER_REPLIES", 2), \
                patch.object(bridge, "request_motion", side_effect=replies) as call:
            bridge.serve(config, io.BytesIO(b"".join(request_line(i) for i in range(1, 4))), output)
        self.assertEqual(call.call_count, 2)
        self.assertEqual(len(output.getvalue().splitlines()), 2)

    def test_bad_pipe_input_does_not_call_provider(self):
        config = bridge.load_config(self.config_path)
        for line in (
            request_line(1).rstrip(),
            b"x" * bridge.REQUEST_LIMIT + b"\n",
            request_line(1).replace(b'"seq": 1', b'"seq": 1, "seq": 2'),
            request_line(1).replace(b'"seq": 1', b'"seq": 4294967296'),
            request_line(1).replace(b'"width": 0.11', b'"width": 0'),
            request_line(1).replace(b'"u": 0.4', b'"u": 1.01'),
            request_line(1).replace(b'"yaw_deg": 150.0', b'"yaw_deg": 1e999'),
        ):
            with self.subTest(line=line):
                output = io.BytesIO()
                with patch.object(bridge, "request_motion") as call:
                    with self.assertRaises(ValueError):
                        bridge.serve(config, io.BytesIO(line), output)
                self.assertEqual(output.getvalue(), b"")
                call.assert_not_called()

    def test_failure_diagnostics_do_not_echo_private_config_or_network_errors(self):
        stderr = io.StringIO()
        stdout = io.TextIOWrapper(io.BytesIO())
        with contextlib.redirect_stderr(stderr), contextlib.redirect_stdout(stdout), \
                patch.object(bridge, "serve", side_effect=OSError("fixture-secret")):
            self.assertEqual(bridge.main([], config_path=self.config_path), 1)
        self.assertEqual(stderr.getvalue(), "Motion API bridge stopped.\n")
        self.assertEqual(stdout.buffer.getvalue(), b"")


if __name__ == "__main__":
    unittest.main()
