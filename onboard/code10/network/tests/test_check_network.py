import importlib.util
from pathlib import Path
import ssl
import sys
import unittest
from unittest.mock import MagicMock, patch

SPEC = importlib.util.spec_from_file_location("uav_check_network", Path(__file__).parents[1] / "check_network.py")
network = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = network
SPEC.loader.exec_module(network)


class ConfigurationTests(unittest.TestCase):
    def config(self, *args):
        return network.configuration(network.parser().parse_args(args))

    def test_defaults(self):
        config = self.config()
        self.assertEqual(config.target_port, 443)
        self.assertEqual(config.proxy_port, 18080)

    def test_rejects_unsafe_urls_and_invalid_ports(self):
        for value in ["http://example.com", "https://u:p@example.com", "https://example.com/#x",
                      "https://example.com:0", "https://example.com:99999", "https://example.com/a b",
                      "https://example.com/\r\nInjected"]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.config("--url", value)

    def test_proxy_rejects_authentication(self):
        with self.assertRaises(ValueError):
            self.config("--proxy", "http://u:secret@localhost:18080")

    def test_ipv6(self):
        config = self.config("--proxy", "http://[::1]:18080", "--url", "https://[::1]/")
        self.assertEqual(config.proxy_host, "::1")
        self.assertEqual(network.authority(config.target_host, config.target_port), "[::1]:443")


class LayerTests(unittest.TestCase):
    def setUp(self):
        self.config = network.configuration(network.parser().parse_args(["--url", "https://api.example.com/models?secret=not-logged"]))

    def run_check(self, http_status=401, connect_status=200, tls_error=None):
        raw, tls = MagicMock(), MagicMock()
        connect_response, api_response = MagicMock(), MagicMock()
        connect_response.status = connect_status
        api_response.status = http_status
        context = MagicMock()
        context.wrap_socket.return_value = tls
        if tls_error:
            context.wrap_socket.side_effect = tls_error
        with patch.object(network.socket, "create_connection", side_effect=[MagicMock(), raw]), \
             patch.object(network.http.client, "HTTPResponse", side_effect=[connect_response, api_response]), \
             patch.object(network.ssl, "create_default_context", return_value=context) as create_context:
            report = network.check(self.config)
        return report, raw, tls, context, create_context

    def test_unauthenticated_response_is_only_network_success(self):
        report, raw, tls, context, create_context = self.run_check()
        self.assertTrue(report["network_reachable"])
        self.assertFalse(report["api_authenticated"])
        self.assertFalse(report["model_call_tested"])
        self.assertEqual([s["stage"] for s in report["stages"]], ["code12_tcp", "proxy_tcp", "http_connect", "tls", "api_http"])
        self.assertEqual(report["stages"][-1]["http_status"], 401)
        self.assertNotIn("not-logged", str(report))
        create_context.assert_called_once_with()
        context.wrap_socket.assert_called_once_with(raw, server_hostname="api.example.com")
        self.assertNotIn(b"Authorization:", tls.sendall.call_args.args[0])
        tls.close.assert_called_once()

    def test_proxy_failure_does_not_attempt_tls(self):
        report, raw, tls, context, create_context = self.run_check(connect_status=502)
        self.assertFalse(report["network_reachable"])
        self.assertEqual(report["stages"][-1]["stage"], "http_connect")
        create_context.assert_not_called()
        raw.close.assert_called_once()

    def test_certificate_failure_is_not_suppressed(self):
        report, raw, tls, context, create_context = self.run_check(tls_error=ssl.SSLCertVerificationError("secret query should not appear"))
        self.assertFalse(report["network_reachable"])
        self.assertEqual(report["stages"][-1]["stage"], "tls")
        self.assertNotIn("secret query", str(report))

    def test_403_404_are_not_api_success(self):
        for status in (403, 404):
            report, *_ = self.run_check(http_status=status)
            self.assertTrue(report["network_reachable"])
            self.assertFalse(report["api_authenticated"])
            self.assertIn("不能据此认定 API 可调用", report["stages"][-1]["detail"])

    def test_first_hop_failure_stops_checks(self):
        with patch.object(network.socket, "create_connection", side_effect=ConnectionRefusedError):
            report = network.check(self.config)
        self.assertEqual(len(report["stages"]), 1)
        self.assertFalse(report["network_reachable"])


if __name__ == "__main__":
    unittest.main()
