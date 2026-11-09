#!/usr/bin/env python3
"""Relay measured target geometry to the configured motion model over HTTPS."""

from __future__ import annotations

import http.client
import json
import math
import os
import re
import signal
import stat
import sys
from dataclasses import dataclass, field
from pathlib import Path
from urllib.parse import SplitResult, urlsplit


CONFIG_PATH = Path("/etc/uav-api-motion.env")
CONFIG_LIMIT = 4096
REQUEST_LIMIT = 1024
PROVIDER_LIMIT = 32768
MAX_PROVIDER_REPLIES = 100
HTTP_TIMEOUT_SECONDS = 2.0
REQUIRED_SETTINGS = {"UAV_MOTION_API_URL", "UAV_MOTION_API_MODEL", "UAV_MOTION_API_KEY"}
TARGET_FIELDS = {"u", "v", "width", "height"}
AIRCRAFT_FIELDS = {"altitude_agl_m", "vel_n_mps", "vel_e_mps", "vel_u_mps", "yaw_deg"}
AXIS_LIMITS = {"x_mps": 0.30, "y_mps": 0.30, "z_mps": 0.12, "yaw_rate_degps": 12.0}
PROVIDER_ID = re.compile(r"[A-Za-z0-9_.:-]{1,128}")
MODEL_ID = re.compile(r"[A-Za-z0-9_./:-]{1,128}")
VISIBLE_ASCII = re.compile(r"[!-~]+")
MOTION_PROMPT = (
    "Given the measured bounding box of the current balloon and aircraft telemetry, "
    "return one short body-velocity segment as a JSON object with exactly these keys: "
    "duration_ms, x_mps, y_mps, z_mps, yaw_rate_degps. Duration must be 50, 100, or 150 ms. "
    "x is forward, y is right, z is up, and positive yaw turns right. "
    "The combined horizontal speed sqrt(x^2 + y^2) must be <= 0.30 m/s. "
    "|z| must be <= 0.12 m/s and |yaw| must be <= 12 deg/s. "
    "The box u and v are its center in normalized image coordinates, with origin at the top left. "
    "Width and height are normalized by the image dimensions. Aircraft velocity is north/east/up. "
    "Align with the target before moving forward. Return all four motion values as zero if uncertain. "
    "Do not output takeoff, landing, search, or any fields besides the five JSON keys."
)


@dataclass(frozen=True)
class Config:
    endpoint: SplitResult
    model: str
    key: str = field(repr=False)
    proxy: SplitResult | None = None


def parse_url(value: str, scheme: str) -> SplitResult:
    if len(value) > 512 or not VISIBLE_ASCII.fullmatch(value) or any(c in value for c in "\\?#"):
        raise ValueError("invalid URL")
    url = urlsplit(value)
    if (url.scheme != scheme or not url.hostname or url.username is not None or
            url.password is not None or url.port == 0):
        raise ValueError("invalid URL")
    return url


def load_config(path: Path) -> Config:
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    with os.fdopen(fd, "rb") as stream:
        meta = os.fstat(stream.fileno())
        if (not stat.S_ISREG(meta.st_mode) or meta.st_uid != os.geteuid() or
                meta.st_mode & 0o077 or meta.st_size > CONFIG_LIMIT):
            raise ValueError("config must be a private regular file")
        data = stream.read(CONFIG_LIMIT + 1)
    if len(data) > CONFIG_LIMIT:
        raise ValueError("config too large")
    values = {}
    for line in data.decode("ascii").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, separator, value = line.partition("=")
        if not separator or name not in REQUIRED_SETTINGS | {"HTTPS_PROXY"} or name in values:
            raise ValueError("invalid config entry")
        values[name] = value
    if not REQUIRED_SETTINGS <= values.keys():
        raise ValueError("missing config entry")
    endpoint = parse_url(values["UAV_MOTION_API_URL"], "https")
    if not endpoint.path.startswith("/") or endpoint.path == "/":
        raise ValueError("missing completion endpoint path")
    model = values["UAV_MOTION_API_MODEL"]
    key = values["UAV_MOTION_API_KEY"]
    if not MODEL_ID.fullmatch(model) or len(key) > 1024 or not VISIBLE_ASCII.fullmatch(key):
        raise ValueError("invalid model or key")
    proxy = None
    if values.get("HTTPS_PROXY"):
        proxy = parse_url(values["HTTPS_PROXY"], "http")
        if proxy.path not in ("", "/"):
            raise ValueError("invalid proxy path")
    return Config(endpoint, model, key, proxy)


def unique_fields(pairs):
    result = {}
    for name, value in pairs:
        if name in result:
            raise ValueError("duplicate JSON field")
        result[name] = value
    return result


def reject_constant(_value):
    raise ValueError("invalid JSON number")


def decode_json(text):
    return json.loads(text, object_pairs_hook=unique_fields, parse_constant=reject_constant)


def exact_fields(value, expected):
    if not isinstance(value, dict) or value.keys() != expected:
        raise ValueError("invalid object fields")


def number_in_range(value, minimum, maximum):
    return type(value) in (int, float) and minimum <= value <= maximum


def validate_request(request, last_seq):
    exact_fields(request, {"seq", "target", "aircraft"})
    seq = request["seq"]
    if type(seq) is not int or not last_seq < seq <= 0xFFFFFFFF:
        raise ValueError("invalid request sequence")
    target = request["target"]
    exact_fields(target, TARGET_FIELDS)
    if not all(number_in_range(value, 0, 1) for value in target.values()) or min(target["width"], target["height"]) <= 0:
        raise ValueError("invalid normalized target")
    aircraft = request["aircraft"]
    exact_fields(aircraft, AIRCRAFT_FIELDS)
    if not all(number_in_range(value, -sys.float_info.max, sys.float_info.max) for value in aircraft.values()):
        raise ValueError("invalid aircraft measurements")


def decode_provider_reply(data):
    envelope = decode_json(data.decode("utf-8"))
    if not isinstance(envelope, dict):
        raise ValueError("invalid provider reply")
    provider_id = envelope.get("id")
    choices = envelope.get("choices")
    if (not isinstance(provider_id, str) or not PROVIDER_ID.fullmatch(provider_id) or
            not isinstance(choices, list) or len(choices) != 1):
        raise ValueError("invalid provider receipt")
    choice = choices[0]
    if not isinstance(choice, dict) or choice.get("finish_reason") != "stop":
        raise ValueError("provider reply was not completed")
    message = choice.get("message")
    if not isinstance(message, dict) or not isinstance(message.get("content"), str) or message.get("tool_calls"):
        raise ValueError("invalid provider content")
    motion = decode_json(message["content"])
    exact_fields(motion, {"duration_ms", *AXIS_LIMITS})
    duration = motion["duration_ms"]
    if type(duration) is not int or duration not in (50, 100, 150):
        raise ValueError("invalid motion duration")
    if not all(number_in_range(motion[axis], -limit, limit) for axis, limit in AXIS_LIMITS.items()):
        raise ValueError("invalid motion axes")
    if math.hypot(motion["x_mps"], motion["y_mps"]) > 0.30:
        raise ValueError("invalid horizontal speed")
    return {**motion, "provider_id": provider_id}


def request_timed_out(_signum, _frame):
    raise TimeoutError("motion provider deadline")


def request_motion(config, request):
    measurements = {"target": request["target"], "aircraft": request["aircraft"]}
    payload = {
        "model": config.model,
        "messages": [
            {"role": "system", "content": MOTION_PROMPT},
            {"role": "user", "content": json.dumps(measurements, allow_nan=False)},
        ],
        "response_format": {"type": "json_object"},
        "max_tokens": 256,
        "stream": False,
    }
    if config.endpoint.hostname == "api.deepseek.com":
        payload["thinking"] = {"type": "disabled"}
    body = json.dumps(payload, allow_nan=False).encode("utf-8")
    peer = config.proxy or config.endpoint
    port = peer.port if config.proxy is None else (peer.port or 80)
    connection = http.client.HTTPSConnection(peer.hostname, port, timeout=HTTP_TIMEOUT_SECONDS)
    if config.proxy is not None:
        connection.set_tunnel(config.endpoint.hostname, config.endpoint.port or 443)
    old_handler = signal.signal(signal.SIGALRM, request_timed_out)
    try:
        signal.setitimer(signal.ITIMER_REAL, HTTP_TIMEOUT_SECONDS)
        connection.request("POST", config.endpoint.path, body, {
            "Authorization": f"Bearer {config.key}",
            "Content-Type": "application/json",
            "Accept": "application/json",
        })
        with connection.getresponse() as response:
            if response.status != 200:
                raise ValueError("motion provider HTTP failure")
            data = response.read(PROVIDER_LIMIT + 1)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, old_handler)
        connection.close()
    if len(data) > PROVIDER_LIMIT:
        raise ValueError("motion provider reply too large")
    motion = decode_provider_reply(data)
    if config.key in motion["provider_id"]:
        raise ValueError("invalid provider receipt")
    return {"seq": request["seq"], **motion}


def serve(config, source, sink):
    last_seq = 0
    provider_ids = set()
    for _ in range(MAX_PROVIDER_REPLIES):
        line = source.readline(REQUEST_LIMIT + 1)
        if not line:
            return
        if len(line) > REQUEST_LIMIT or not line.endswith(b"\n"):
            raise ValueError("invalid request framing")
        request = decode_json(line.decode("utf-8"))
        validate_request(request, last_seq)
        result = request_motion(config, request)
        provider_id = result["provider_id"]
        if provider_id in provider_ids:
            raise ValueError("repeated provider receipt")
        provider_ids.add(provider_id)
        sink.write(json.dumps(result, allow_nan=False, separators=(",", ":")).encode("utf-8") + b"\n")
        sink.flush()
        last_seq = request["seq"]


def main(argv=None, *, config_path=CONFIG_PATH):
    args = sys.argv[1:] if argv is None else argv
    try:
        if args not in ([], ["--check-config"]):
            raise ValueError("invalid arguments")
        config = load_config(config_path)
        if not args:
            serve(config, sys.stdin.buffer, sys.stdout.buffer)
        return 0
    except (OSError, ValueError, RecursionError, http.client.HTTPException):
        print("Motion API bridge stopped.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
