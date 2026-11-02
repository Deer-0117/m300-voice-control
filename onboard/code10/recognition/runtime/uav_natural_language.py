#!/usr/bin/env python3
"""Local command gateway and structured entry point for the UAV capability library.

The rule parser is intentionally small.  A future LLM may submit the same
strict plan schema through /plan, but it cannot bypass capability validation.
Emergency stop is always recognized and executed locally.
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import hmac
import json
import os
import re
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from uav_capabilities import (
    CapabilityError,
    PLAN_VERSION,
    capability_catalog,
    validate_actions,
    validate_plan,
)


ROOT = Path(__file__).resolve().parent
PLAN_FILE = ROOT / "nl_task_plan.json"
STATE_FILE = ROOT / "nl_task_state.json"
EVENT_LOG = ROOT / "nl_command_events.jsonl"
STOP_REQUEST_FILE = Path("/tmp/balloon_nl_stop.request")
ARMED_FILE = Path("/tmp/balloon_strike.armed")
TRIGGER_FILE = Path("/tmp/balloon_strike.trigger")
PLAN_LOCK_FILE = ROOT / ".nl_task_plan.lock"

COLOR_ALIASES = {
    "任意颜色": "any", "任何颜色": "any", "任意": "any", "any color": "any", "any": "any",
    "红色": "red", "红": "red", "red": "red",
    "橙色": "orange", "橙": "orange", "orange": "orange",
    "黄色": "yellow", "黄": "yellow", "yellow": "yellow",
    "绿色": "green", "绿": "green", "green": "green",
    "蓝色": "blue", "蓝": "blue", "blue": "blue",
    "紫色": "purple", "紫": "purple", "purple": "purple",
    "粉红色": "pink", "粉色": "pink", "粉红": "pink", "粉": "pink", "pink": "pink",
}
STOP_PATTERN = re.compile(r"立即停止|马上停止|停下|停止|悬停|终止任务|别撞了|不要继续|\bstop\b|\babort\b|\bhover\b", re.I)
STRIKE_PATTERN = re.compile(r"撞击|撞|击破|追击|\bhit\b|\bstrike\b|\bchase\b", re.I)


class CommandRejected(CapabilityError):
    pass


def now_ms() -> int:
    return time.time_ns() // 1_000_000


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f"{path.name}.tmp.{os.getpid()}.{uuid.uuid4().hex}"
    )
    try:
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


@contextlib.contextmanager
def plan_write_lock(timeout_seconds: float = 3.0):
    """Serialize plan writers across the HTTP service and helper processes."""
    deadline = time.monotonic() + timeout_seconds
    descriptor: int | None = None
    while descriptor is None:
        try:
            descriptor = os.open(
                PLAN_LOCK_FILE,
                os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                0o600,
            )
            os.write(descriptor, f"{os.getpid()} {now_ms()}\n".encode("ascii"))
        except FileExistsError:
            try:
                age_seconds = time.time() - PLAN_LOCK_FILE.stat().st_mtime
                if age_seconds > 30:
                    PLAN_LOCK_FILE.unlink(missing_ok=True)
                    continue
            except FileNotFoundError:
                continue
            if time.monotonic() >= deadline:
                raise CommandRejected("another plan update is still in progress")
            time.sleep(0.05)
    try:
        yield
    finally:
        os.close(descriptor)
        PLAN_LOCK_FILE.unlink(missing_ok=True)


def append_event(event: str, **fields: Any) -> None:
    EVENT_LOG.parent.mkdir(parents=True, exist_ok=True)
    payload = {"ts_ms": now_ms(), "source": "uav_natural_language.py", "event": event, **fields}
    with EVENT_LOG.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")


def _color_occurrences(command: str) -> list[tuple[int, int, str]]:
    hits: list[tuple[int, int, str]] = []
    for alias, color in COLOR_ALIASES.items():
        for match in re.finditer(re.escape(alias), command, re.I):
            hits.append((match.start(), match.end(), color))
    # Prefer the longest alias when aliases overlap at the same position.
    hits.sort(key=lambda item: (item[0], -(item[1] - item[0])))
    selected: list[tuple[int, int, str]] = []
    occupied: set[int] = set()
    for start, end, color in hits:
        if any(position in occupied for position in range(start, end)):
            continue
        occupied.update(range(start, end))
        selected.append((start, end, color))
    return selected


def local_parse(command: str) -> list[dict[str, Any]]:
    text = re.sub(r"\s+", "", command.strip().lower())
    if not text:
        raise CommandRejected("empty command")
    stop_matches = list(STOP_PATTERN.finditer(text))
    colors = _color_occurrences(text)
    strike_matches = list(STRIKE_PATTERN.finditer(text))

    # A standalone stop must never depend on any other parser or service.
    if stop_matches and not colors:
        return [{"capability": "stop", "parameters": {}}]

    events: list[tuple[int, dict[str, Any]]] = []
    for start, _end, color in colors:
        preceding = [match for match in strike_matches if match.start() <= start]
        if not preceding or start - preceding[-1].end() > 12:
            raise CommandRejected("target color was provided without a nearby strike action")
        events.append((preceding[-1].start(), {
            "capability": "strike_balloon",
            "parameters": {"color": color},
        }))
    if stop_matches:
        events.append((stop_matches[-1].start(), {"capability": "stop", "parameters": {}}))
    if not events:
        raise CommandRejected("local parser cannot map the command to an installed capability")
    events.sort(key=lambda item: item[0])
    try:
        return validate_actions([item[1] for item in events])
    except CapabilityError as exc:
        raise CommandRejected(str(exc)) from exc


def execute_stop_request(plan_id: str, reason: str) -> None:
    STOP_REQUEST_FILE.write_text(f"plan_id={plan_id}\nreason={reason}\nts_ms={now_ms()}\n", encoding="utf-8")


def store_plan(actions: list[dict[str, Any]], source: str, original_command: str | None = None) -> dict[str, Any]:
    actions = validate_actions(actions)
    with plan_write_lock():
        is_stop = actions[0]["capability"] == "stop"
        if not is_stop and not ARMED_FILE.exists():
            raise CommandRejected("press Prepare before submitting a flight plan")
        if not is_stop and TRIGGER_FILE.exists():
            raise CommandRejected("flight task is running or stopping; new plans are locked")

        plan_id = uuid.uuid4().hex
        payload: dict[str, Any] = {
            "version": PLAN_VERSION,
            "plan_id": plan_id,
            "created_ts_ms": now_ms(),
            "source": source,
            "requires_operator_start": not is_stop,
            "actions": actions,
        }
        if original_command is not None:
            payload["original_command"] = original_command
        atomic_write_json(PLAN_FILE, payload)
        atomic_write_json(STATE_FILE, {
            "version": PLAN_VERSION,
            "plan_id": plan_id,
            "status": "queued",
            "current_index": 0,
            "updated_ts_ms": now_ms(),
        })
        if is_stop:
            execute_stop_request(plan_id, "local_emergency_stop")
    append_event("plan_accepted", plan_id=plan_id, source=source, actions=actions)
    return payload


def process_command(command: str) -> dict[str, Any]:
    command = command.strip()
    if not command:
        raise CommandRejected("empty command")
    return store_plan(local_parse(command), "local_rules", command)


def process_structured_plan(raw_plan: Any) -> dict[str, Any]:
    try:
        normalized = validate_plan(raw_plan)
    except CapabilityError as exc:
        raise CommandRejected(str(exc)) from exc
    return store_plan(normalized["actions"], "structured_api")


def read_status() -> dict[str, Any]:
    result: dict[str, Any] = {"ok": True, "stop_requested": STOP_REQUEST_FILE.exists()}
    for name, path in (("plan", PLAN_FILE), ("state", STATE_FILE)):
        try:
            result[name] = json.loads(path.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, json.JSONDecodeError):
            result[name] = None
    return result


class CommandHandler(BaseHTTPRequestHandler):
    server_version = "UavCapabilityGateway/2.0"

    def _json_response(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status.value)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        expected = os.environ.get("UAV_NL_AUTH_TOKEN", "")
        if not expected:
            return self.client_address[0] in ("127.0.0.1", "::1")
        supplied = self.headers.get("Authorization", "")
        return hmac.compare_digest(supplied, f"Bearer {expected}")

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/health":
            self._json_response(HTTPStatus.OK, {"ok": True, "service": "uav-capability-gateway"})
            return
        if self.path not in ("/status", "/capabilities"):
            self._json_response(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        if not self._authorized():
            self._json_response(HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "unauthorized"})
            return
        payload = read_status() if self.path == "/status" else {"ok": True, **capability_catalog()}
        self._json_response(HTTPStatus.OK, payload)

    def do_POST(self) -> None:  # noqa: N802
        if self.path not in ("/command", "/plan"):
            self._json_response(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        if not self._authorized():
            self._json_response(HTTPStatus.UNAUTHORIZED, {"ok": False, "error": "unauthorized"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 16384:
                raise CommandRejected("invalid request size")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if self.path == "/command":
                command = payload.get("command") if isinstance(payload, dict) else None
                if not isinstance(command, str):
                    raise CommandRejected("command must be a string")
                plan = process_command(command)
            else:
                plan = process_structured_plan(payload)
            self._json_response(HTTPStatus.OK, {"ok": True, "plan": plan})
        except (CommandRejected, UnicodeDecodeError, json.JSONDecodeError) as exc:
            append_event("request_rejected", path=self.path, error=str(exc))
            self._json_response(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})

    def log_message(self, format_string: str, *args: Any) -> None:
        append_event("http_access", client=self.client_address[0], message=format_string % args)


def self_test() -> None:
    cases = {
        "停下": [{"capability": "stop", "parameters": {}}],
        "先撞那个粉色气球再撞那个绿色的": [
            {"capability": "strike_balloon", "parameters": {"color": "pink"}},
            {"capability": "strike_balloon", "parameters": {"color": "green"}},
        ],
        "撞击任意颜色气球": [
            {"capability": "strike_balloon", "parameters": {"color": "any"}},
        ],
    }
    for command, expected in cases.items():
        actual = local_parse(command)
        if actual != expected:
            raise AssertionError(f"{command!r}: expected {expected!r}, got {actual!r}")
    for rejected in ("起飞到100米", "绕绿色箱子一圈", "捡起瓶子", "等待100秒"):
        try:
            local_parse(rejected)
        except CommandRejected:
            continue
        raise AssertionError(f"unsupported command was accepted: {rejected!r}")
    print("self-test passed")


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--serve", action="store_true")
    group.add_argument("--command")
    group.add_argument("--command-base64")
    group.add_argument("--plan-json-base64")
    group.add_argument("--list-capabilities", action="store_true")
    group.add_argument("--self-test", action="store_true")
    parser.add_argument("--bind", default=os.environ.get("UAV_NL_BIND", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("UAV_NL_PORT", "8765")))
    args = parser.parse_args()

    if args.self_test:
        self_test()
        return 0
    if args.list_capabilities:
        print(json.dumps(capability_catalog(), ensure_ascii=False, indent=2))
        return 0
    if args.command_base64:
        command = base64.b64decode(args.command_base64, validate=True).decode("utf-8")
        print(json.dumps(process_command(command), ensure_ascii=False, indent=2))
        return 0
    if args.plan_json_base64:
        raw = base64.b64decode(args.plan_json_base64, validate=True).decode("utf-8")
        print(json.dumps(process_structured_plan(json.loads(raw)), ensure_ascii=False, indent=2))
        return 0
    if args.command is not None:
        print(json.dumps(process_command(args.command), ensure_ascii=False, indent=2))
        return 0

    if args.bind not in ("127.0.0.1", "::1") and not os.environ.get("UAV_NL_AUTH_TOKEN", ""):
        raise SystemExit("UAV_NL_AUTH_TOKEN is required for a non-loopback bind address")
    server = ThreadingHTTPServer((args.bind, args.port), CommandHandler)
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
