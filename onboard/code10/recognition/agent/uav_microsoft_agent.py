#!/usr/bin/env python3
"""Microsoft Agent Framework gateway for declarative UAV capability plans.

The model may only propose ``strike_balloon(color)`` actions. Search is
inserted automatically by the executor between ordered strikes. A proposal is
persisted first and must be submitted in a separate authenticated request.  The
code10 capability gateway performs the final validation before execution.
"""

from __future__ import annotations

import argparse
import asyncio
import hmac
import json
import os
import threading
import time
import urllib.error
import urllib.request
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Literal

from uav_capabilities import (
    ALLOWED_COLOR_ARGUMENTS,
    CapabilityError,
    capability_catalog,
    validate_plan,
)


ColorArgument = Literal["any", "red", "orange", "yellow", "green", "blue", "purple", "pink"]
STATE_ROOT = Path(os.environ.get("UAV_AGENT_STATE_ROOT", "/var/lib/uav-microsoft-agent"))
PLAN_DIR = STATE_ROOT / "plans"
EVENT_LOG = STATE_ROOT / "events.jsonl"


class AgentError(RuntimeError):
    pass


class AgentUnavailable(AgentError):
    pass


def agent_max_tokens() -> int:
    raw = os.environ.get("UAV_AGENT_MAX_TOKENS", "2048").strip()
    try:
        value = int(raw)
    except ValueError as exc:
        raise AgentUnavailable("UAV_AGENT_MAX_TOKENS must be an integer") from exc
    if value < 512 or value > 8192:
        raise AgentUnavailable("UAV_AGENT_MAX_TOKENS must be between 512 and 8192")
    return value


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def append_event(event: str, **fields: Any) -> None:
    EVENT_LOG.parent.mkdir(parents=True, exist_ok=True)
    with EVENT_LOG.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"ts_ms": now_ms(), "event": event, **fields}, ensure_ascii=False) + "\n")


def build_strike_plan(colors: list[str]) -> dict[str, Any]:
    if not isinstance(colors, list) or not colors:
        raise AgentError("colors must be a non-empty list")
    actions = [
        {"capability": "strike_balloon", "parameters": {"color": color}}
        for color in colors
    ]
    try:
        return validate_plan({"actions": actions})
    except CapabilityError as exc:
        raise AgentError(str(exc)) from exc


async def interpret_with_agent(command: str) -> tuple[dict[str, Any], str]:
    command = command.strip()
    if not command:
        raise AgentError("empty command")
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise AgentUnavailable("OPENAI_API_KEY is not configured")
    base_url = os.environ.get("OPENAI_BASE_URL", "https://api.deepseek.com").rstrip("/")
    model = os.environ.get("OPENAI_CHAT_MODEL_ID", "deepseek-v4-flash").strip()
    if not model:
        raise AgentUnavailable("OPENAI_CHAT_MODEL_ID is not configured")

    from agent_framework.openai import OpenAIChatCompletionClient

    proposals: list[dict[str, Any]] = []

    def propose_strike_balloon_plan(colors: list[ColorArgument]) -> str:
        """Propose an ordered plan to strike balloons by color; use 'any' for any color."""
        if proposals:
            raise AgentError("only one plan proposal is allowed per request")
        plan = build_strike_plan(list(colors))
        proposals.append(plan)
        return json.dumps({"accepted": True, "plan": plan}, ensure_ascii=False)

    client = OpenAIChatCompletionClient(model=model, api_key=api_key, base_url=base_url)
    agent = client.as_agent(
        name="UavStrikePlanner",
        instructions=(
            "Translate the user's UAV balloon instruction into exactly one call to "
            "propose_strike_balloon_plan. Preserve order. Allowed colors are "
            f"{list(ALLOWED_COLOR_ARGUMENTS)}. Use 'any' only when the user requests any "
            "color. The executor automatically performs a full gimbal/body search between "
            "successive colors; do not add or omit colors to represent searching. "
            "Do not invent movement, speed, altitude, attitude, coordinates, takeoff, "
            "landing, following, orbiting, or any other action. If the request cannot be "
            "expressed only as ordered balloon strikes, do not call a tool."
        ),
        tools=[propose_strike_balloon_plan],
        default_options={"temperature": 0, "max_tokens": agent_max_tokens()},
    )
    response = await agent.run(command)
    if len(proposals) != 1:
        raise AgentError("model did not produce exactly one valid capability plan")
    return proposals[0], str(response)


def save_proposal(command: str, plan: dict[str, Any], model_response: str) -> dict[str, Any]:
    proposal_id = uuid.uuid4().hex
    payload = {
        "proposal_id": proposal_id,
        "status": "proposed",
        "created_ts_ms": now_ms(),
        "command": command,
        "plan": plan,
        "model_response": model_response,
    }
    atomic_write_json(PLAN_DIR / f"{proposal_id}.json", payload)
    append_event("proposal_created", proposal_id=proposal_id, actions=plan["actions"])
    return payload


def load_proposal(proposal_id: str) -> dict[str, Any]:
    if not proposal_id or any(character not in "0123456789abcdef" for character in proposal_id):
        raise AgentError("invalid proposal_id")
    try:
        payload = json.loads((PLAN_DIR / f"{proposal_id}.json").read_text(encoding="utf-8"))
    except (FileNotFoundError, OSError, json.JSONDecodeError) as exc:
        raise AgentError("proposal not found") from exc
    if payload.get("proposal_id") != proposal_id:
        raise AgentError("proposal identity mismatch")
    return payload


def submit_proposal(proposal_id: str) -> dict[str, Any]:
    payload = load_proposal(proposal_id)
    if payload.get("status") != "proposed":
        raise AgentError(f"proposal is not submit-ready: {payload.get('status')}")
    plan = validate_plan(payload.get("plan"))
    endpoint = os.environ.get("UAV_PLAN_ENDPOINT", "http://192.168.168.155:8765/plan")
    token = os.environ.get("UAV_PLAN_AUTH_TOKEN", "").strip()
    if not token:
        raise AgentUnavailable("UAV_PLAN_AUTH_TOKEN is not configured")
    request = urllib.request.Request(
        endpoint,
        data=json.dumps(plan, ensure_ascii=False).encode("utf-8"),
        headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
        method="POST",
    )
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(request, timeout=float(os.environ.get("UAV_PLAN_TIMEOUT_SECONDS", "5"))) as response:
            remote = json.loads(response.read().decode("utf-8"))
    except (OSError, ValueError, urllib.error.URLError) as exc:
        append_event("proposal_submit_failed", proposal_id=proposal_id, error=str(exc))
        raise AgentError(f"code10 plan submission failed: {exc}") from exc
    payload.update(status="submitted", submitted_ts_ms=now_ms(), code10_response=remote)
    atomic_write_json(PLAN_DIR / f"{proposal_id}.json", payload)
    append_event("proposal_submitted", proposal_id=proposal_id)
    return payload


class AgentHandler(BaseHTTPRequestHandler):
    server_version = "UavMicrosoftAgent/1.0"

    def _respond(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        expected = os.environ.get("UAV_AGENT_AUTH_TOKEN", "")
        if not expected:
            return self.client_address[0] in ("127.0.0.1", "::1")
        return hmac.compare_digest(self.headers.get("Authorization", ""), f"Bearer {expected}")

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 16384:
            raise AgentError("invalid request size")
        value = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(value, dict):
            raise AgentError("request body must be an object")
        return value

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._respond(HTTPStatus.OK, {
                "ok": True,
                "service": "uav-microsoft-agent",
                "model_key_configured": bool(os.environ.get("OPENAI_API_KEY", "").strip()),
                "plan_target": os.environ.get("UAV_PLAN_ENDPOINT", "http://192.168.168.155:8765/plan"),
            })
            return
        if self.path == "/capabilities":
            self._respond(HTTPStatus.OK, {"ok": True, **capability_catalog()})
            return
        self._respond(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})

    def do_POST(self) -> None:  # noqa: N802
        if self.path not in ("/interpret", "/submit"):
            self._respond(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        if not self._authorized():
            self._respond(HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "unauthorized"})
            return
        try:
            body = self._body()
            if self.path == "/interpret":
                command = body.get("command")
                if not isinstance(command, str):
                    raise AgentError("command must be a string")
                plan, model_response = asyncio.run(interpret_with_agent(command))
                result = save_proposal(command, plan, model_response)
            else:
                proposal_id = body.get("proposal_id")
                if not isinstance(proposal_id, str):
                    raise AgentError("proposal_id must be a string")
                result = submit_proposal(proposal_id)
            self._respond(HTTPStatus.OK, {"ok": True, "result": result})
        except AgentUnavailable as exc:
            append_event("request_unavailable", path=self.path, error=str(exc))
            self._respond(HTTPStatus.SERVICE_UNAVAILABLE, {"ok": False, "error": str(exc)})
        except (AgentError, CapabilityError, UnicodeDecodeError, json.JSONDecodeError) as exc:
            append_event("request_rejected", path=self.path, error=str(exc))
            self._respond(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})

    def log_message(self, format_string: str, *args: Any) -> None:
        append_event("http_access", client=self.client_address[0], message=format_string % args)


def self_test() -> None:
    expected = {
        "version": 2,
        "actions": [
            {"capability": "strike_balloon", "parameters": {"color": "pink"}},
            {"capability": "strike_balloon", "parameters": {"color": "any"}},
        ],
    }
    if build_strike_plan(["pink", "any"]) != expected:
        raise AssertionError("ordered plan mismatch")
    for rejected in ([], ["cyan"], ["pink"] * 9):
        try:
            build_strike_plan(rejected)
        except AgentError:
            continue
        raise AssertionError(f"unsafe plan accepted: {rejected!r}")
    print("self-test passed")


def mock_agent_test() -> None:
    class MockHandler(BaseHTTPRequestHandler):
        calls = 0

        def do_POST(self) -> None:  # noqa: N802
            length = int(self.headers.get("Content-Length", "0"))
            request_payload = json.loads(self.rfile.read(length).decode("utf-8"))
            MockHandler.calls += 1
            if MockHandler.calls == 1:
                tool_name = request_payload["tools"][0]["function"]["name"]
                message = {
                    "role": "assistant", "content": None,
                    "tool_calls": [{
                        "id": "call_mock_1", "type": "function",
                        "function": {"name": tool_name, "arguments": '{"colors":["pink","any"]}'},
                    }],
                }
                finish_reason = "tool_calls"
            else:
                message = {"role": "assistant", "content": "plan proposed"}
                finish_reason = "stop"
            response_payload = {
                "id": f"mock-{MockHandler.calls}", "object": "chat.completion",
                "created": int(time.time()), "model": "mock-model",
                "choices": [{"index": 0, "message": message, "finish_reason": finish_reason}],
                "usage": {"prompt_tokens": 1, "completion_tokens": 1, "total_tokens": 2},
            }
            body = json.dumps(response_payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format_string: str, *args: Any) -> None:
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), MockHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    old = {name: os.environ.get(name) for name in ("OPENAI_API_KEY", "OPENAI_BASE_URL", "OPENAI_CHAT_MODEL_ID")}
    try:
        os.environ.update({
            "OPENAI_API_KEY": "mock-key",
            "OPENAI_BASE_URL": f"http://127.0.0.1:{server.server_port}/v1",
            "OPENAI_CHAT_MODEL_ID": "mock-model",
        })
        plan, _response = asyncio.run(interpret_with_agent("先撞粉色气球，再撞任意颜色气球"))
        if plan != build_strike_plan(["pink", "any"]) or MockHandler.calls != 2:
            raise AssertionError(f"mock Agent Framework flow failed: {plan!r}, calls={MockHandler.calls}")
        print("mock-agent-test passed")
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        for name, value in old.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--serve", action="store_true")
    group.add_argument("--self-test", action="store_true")
    group.add_argument("--mock-agent-test", action="store_true")
    group.add_argument("--command")
    parser.add_argument("--bind", default=os.environ.get("UAV_AGENT_BIND", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("UAV_AGENT_PORT", "8770")))
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.mock_agent_test:
        mock_agent_test()
        return 0
    if args.command is not None:
        plan, response = asyncio.run(interpret_with_agent(args.command))
        print(json.dumps({"plan": plan, "model_response": response}, ensure_ascii=False, indent=2))
        return 0
    if args.bind not in ("127.0.0.1", "::1") and not os.environ.get("UAV_AGENT_AUTH_TOKEN", ""):
        raise SystemExit("UAV_AGENT_AUTH_TOKEN is required for a non-loopback bind")
    STATE_ROOT.mkdir(parents=True, exist_ok=True)
    server = ThreadingHTTPServer((args.bind, args.port), AgentHandler)
    append_event("service_started", bind=args.bind, port=args.port)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        append_event("service_stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
