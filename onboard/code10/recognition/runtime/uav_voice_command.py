#!/usr/bin/env python3
"""Boot-bound API voice candidates; only the widget mission gate may start flight.

Interpretation calls the configured API agent. Confirmation validates that exact
proposal locally and stores a plan; it deliberately does not call the agent's
/submit endpoint, whose remote write could outlive cancellation.
"""
from __future__ import annotations

import argparse
import contextlib
import fcntl
import io
import json
import os
import re
import time
import urllib.error
import urllib.request
import uuid
import wave
from pathlib import Path
from typing import Any

from balloon_clock import clock_id, now_ms
from uav_capabilities import CapabilityError, validate_actions
from uav_natural_language import CommandRejected, plan_write_lock, store_plan

ROOT = Path(__file__).resolve().parent
CANDIDATE_FILE = ROOT / "voice_command_candidate.json"
SESSION_STATE = ROOT / "voice_command_session.json"
LOCK_FILE = ROOT / ".voice_command.lock"
SESSION_FILE = Path("/tmp/balloon_api_voice.session")
READY_FILE = Path("/tmp/balloon_api_voice.ready")
MESSAGE_FILE = Path("/tmp/balloon_voice_message.txt")
MISSION_TOKEN_FILE = Path("/tmp/balloon_mission.token")
MISSION_COMMIT_FILE = Path("/tmp/balloon_mission.commit")
PLAN_FILE = ROOT / "nl_task_plan.json"
PLAN_STATE_FILE = ROOT / "nl_task_state.json"
CANDIDATE_TTL_MS = 300_000
SESSION_TTL_MS = 600_000
TOKEN_RE = re.compile(r"[A-Za-z0-9_-]{1,95}\Z")


class VoiceError(RuntimeError):
    pass


VOICE_NORMALIZATIONS = {
    "氣球": "气球", "顔色": "颜色", "顏色": "颜色", "紅色": "红色",
    "綠色": "绿色", "藍色": "蓝色", "黃色": "黄色", "橙色": "橙色",
    "紫色": "紫色", "粉紅": "粉红", "撞擊": "撞击", "撞機": "撞击",
    "撞机": "撞击", "狀机": "撞击", "状机": "撞击", "壯機": "撞击",
    "追擊": "追击", "停止": "停下",
}


def normalize_transcript(transcript: str) -> str:
    if not isinstance(transcript, str):
        raise VoiceError("语音文字格式无效，请重新录音")
    normalized = transcript.strip().strip("。！？!?，,")
    for source, target in VOICE_NORMALIZATIONS.items():
        normalized = normalized.replace(source, target)
    if not normalized or len(normalized) > 2000:
        raise VoiceError("语音内容为空或过长，请重新录音")
    return normalized


def atomic_text(path: Path, text: str) -> None:
    temporary = path.with_name(path.name + f".{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_text(path, json.dumps(payload, ensure_ascii=False, indent=2) + "\n")


def read_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise VoiceError("语音状态不存在或损坏，请重新录音") from exc
    if not isinstance(value, dict):
        raise VoiceError("语音状态格式无效，请重新录音")
    return value


def write_message(message: str) -> None:
    # The widget has a 240-byte UTF-8 display limit. Error bodies/URLs are never
    # included, since remote failures can contain credentials or request text.
    visible = message.strip().encode("utf-8")[:240].decode("utf-8", errors="ignore")
    atomic_text(MESSAGE_FILE, visible + "\n")


@contextlib.contextmanager
def voice_lock():
    fd = os.open(LOCK_FILE, os.O_CREAT | os.O_RDWR, 0o600)
    deadline = time.monotonic() + 3.0
    try:
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                if time.monotonic() >= deadline:
                    raise VoiceError("语音状态处理中，请稍后重试")
                time.sleep(.01)
        yield
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)


def valid_token(value: Any) -> bool:
    return isinstance(value, str) and TOKEN_RE.fullmatch(value) is not None


def _session_locked(expected: str | None = None) -> dict[str, Any]:
    try:
        token = SESSION_FILE.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise VoiceError("语音会话已取消，请重新录音") from exc
    if not valid_token(token) or (expected is not None and token != expected):
        raise VoiceError("语音会话已变更，旧结果已拒绝")
    state = read_object(SESSION_STATE)
    stamp = state.get("created_ms")
    if (state.get("session") != token or state.get("clock_id") != clock_id() or
            type(stamp) is not int or not 0 <= now_ms() - stamp <= SESSION_TTL_MS):
        raise VoiceError("语音会话已过期或属于其他开机周期，请重新录音")
    return state


def _clear_candidate_locked() -> None:
    READY_FILE.unlink(missing_ok=True)
    CANDIDATE_FILE.unlink(missing_ok=True)


def begin_session(session: str) -> dict[str, Any]:
    if not valid_token(session):
        raise VoiceError("语音会话编号无效")
    with voice_lock():
        # Always invalidate the previous session before publishing a new one.
        SESSION_FILE.unlink(missing_ok=True)
        _clear_candidate_locked()
        state = dict(version=1, session=session, clock_id=clock_id(),
                     created_ms=now_ms(), status="awaiting_audio", operation=None)
        atomic_json(SESSION_STATE, state)
        atomic_text(SESSION_FILE, session + "\n")
        write_message("请按遥控器语音键重新输入指令；识别完成后返回点击执行")
        return state


def _initialize_recording_locked(session: str | None) -> None:
    """Accept a fresh token published by the C watcher, never recreate it.

    C allocates a unique token for each stable recording while holding its
    action mutex. Stop removes that token synchronously, including before a
    delayed helper gets scheduled. Only the current token authorizes bootstrap.
    """
    if session is None:
        return
    try:
        current = SESSION_FILE.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise VoiceError("语音会话已取消，请重新录音") from exc
    if not valid_token(session) or current != session:
        raise VoiceError("语音会话已变更，旧录音已拒绝")
    try:
        state = read_object(SESSION_STATE)
    except VoiceError:
        if SESSION_STATE.exists():
            raise
        state = {}
    if state.get("session") != session:
        atomic_json(SESSION_STATE, dict(version=1, session=session, clock_id=clock_id(),
                                       created_ms=now_ms(), status="awaiting_audio", operation=None))
    _session_locked(session)


def _reserve(session: str | None, status: str) -> tuple[str, str]:
    with voice_lock():
        _initialize_recording_locked(session)
        state = _session_locked(session)
        if state.get("status") in ("recognizing", "interpreting", "confirming"):
            raise VoiceError("本轮语音正在处理中，请稍候")
        if state.get("status") == "confirmed":
            raise VoiceError("本轮语音已确认，请先新增语音任务")
        _clear_candidate_locked()
        operation = uuid.uuid4().hex
        state.update(status=status, operation=operation)
        atomic_json(SESSION_STATE, state)
        return state["session"], operation


def _operation_locked(session: str, operation: str) -> dict[str, Any]:
    state = _session_locked(session)
    if state.get("operation") != operation:
        raise VoiceError("语音操作已取消，迟到结果已拒绝")
    return state


def _fail_operation(session: str, operation: str) -> None:
    with voice_lock():
        try:
            state = _operation_locked(session, operation)
        except VoiceError:
            return
        state.update(status="failed", operation=None)
        atomic_json(SESSION_STATE, state)
        _clear_candidate_locked()


def pcm_to_wav(pcm_path: Path) -> bytes:
    before = pcm_path.stat()
    if not pcm_path.is_file() or not 0 < before.st_size <= 1_920_000:
        raise VoiceError("录音长度无效，请录制不超过60秒的指令")
    pcm = pcm_path.read_bytes()
    after = pcm_path.stat()
    if (not pcm or len(pcm) % 2 or len(pcm) != before.st_size or
            (before.st_mtime_ns, before.st_size) != (after.st_mtime_ns, after.st_size)):
        raise VoiceError("录音尚未完成或数据无效，请录音结束后重试")
    output = io.BytesIO()
    with wave.open(output, "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(16000)
        stream.writeframes(pcm)
    return output.getvalue()


def post_json(url: str, payload: dict[str, Any], token: str, timeout: int = 45) -> dict[str, Any]:
    request = urllib.request.Request(url, data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                                     headers={"Content-Type": "application/json", "Authorization": f"Bearer {token}"},
                                     method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            data = response.read(262145)
            if len(data) > 262144:
                raise VoiceError("API响应过长，任务未确认")
            value = json.loads(data.decode("utf-8"))
    except urllib.error.HTTPError as exc:
        exc.close()
        raise VoiceError(f"API请求失败：HTTP {exc.code}") from exc
    except (OSError, urllib.error.URLError, UnicodeError, ValueError) as exc:
        raise VoiceError("API连接失败、超时或响应格式错误，任务未确认") from exc
    if not isinstance(value, dict) or value.get("ok") is not True:
        raise VoiceError("API未返回成功结果，任务未确认")
    return value


def transcribe(pcm_path: Path) -> str:
    formal = os.environ.get("UAV_VOICE_ASR_FALLBACK_ENDPOINT", "http://192.168.168.222:8780/transcribe")
    configured = os.environ.get("UAV_VOICE_ASR_ENDPOINT", formal)
    endpoints = list(dict.fromkeys((formal, configured) if configured.startswith("http://192.168.55.2:18780/")
                                  else (configured, formal)))
    token = os.environ.get("UAV_VOICE_ASR_TOKEN", "")
    if not token:
        raise VoiceError("语音识别令牌未配置")
    audio = pcm_to_wav(pcm_path)
    deadline = time.monotonic() + 45.0
    for endpoint in endpoints:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        request = urllib.request.Request(endpoint, data=audio,
                                         headers={"Content-Type": "audio/wav", "Authorization": f"Bearer {token}"},
                                         method="POST")
        try:
            with urllib.request.urlopen(request, timeout=remaining) as response:
                data = response.read(16385)
                if len(data) > 16384:
                    raise VoiceError("语音识别响应过长")
                value = json.loads(data.decode("utf-8"))
        except urllib.error.HTTPError as exc:
            exc.close()
            raise VoiceError(f"语音识别失败：HTTP {exc.code}") from exc
        except (OSError, urllib.error.URLError, UnicodeError, ValueError):
            continue
        if (not isinstance(value, dict) or value.get("ok") is not True or
                not isinstance(value.get("transcript"), str) or not value["transcript"].strip()):
            raise VoiceError("语音识别未返回有效文字，请重新录音")
        return normalize_transcript(value["transcript"])
    raise VoiceError("语音识别连接失败或超时，请检查ASR服务")


def _api_proposal(transcript: str) -> dict[str, Any]:
    token = os.environ.get("UAV_AGENT_AUTH_TOKEN", "").strip()
    if not token:
        raise VoiceError("API决策服务令牌未配置，任务未确认")
    endpoint = os.environ.get("UAV_VOICE_AGENT_ENDPOINT", "http://192.168.168.155:8770/interpret")
    result = post_json(endpoint, {"command": transcript}, token)
    proposal = result.get("result")
    if not isinstance(proposal, dict):
        raise VoiceError("API未返回有效候选计划")
    proposal_id = proposal.get("proposal_id")
    if (not isinstance(proposal_id, str) or re.fullmatch(r"[0-9a-f]{32}", proposal_id) is None or
            proposal.get("status") != "proposed" or proposal.get("command") != transcript):
        raise VoiceError("API候选身份或指令不匹配，任务未确认")
    try:
        actions = validate_actions(proposal["plan"]["actions"])
    except (KeyError, TypeError, CapabilityError) as exc:
        raise VoiceError("API候选包含无效动作，任务未确认") from exc
    if any(action["capability"] != "strike_balloon" for action in actions):
        raise VoiceError("API语音任务仅支持气球目标顺序；停止请使用停止测试")
    return dict(kind="api", proposal_id=proposal_id, actions=actions)


def _publish_candidate(transcript: str, session: str, operation: str, defer_api: bool) -> dict[str, Any]:
    transcript = normalize_transcript(transcript)
    payload = dict(kind="transcript") if defer_api else _api_proposal(transcript)
    with voice_lock():
        state = _operation_locked(session, operation)
        candidate = dict(version=2, session=session, clock_id=clock_id(), created_ms=now_ms(),
                         candidate_id=uuid.uuid4().hex, transcript=transcript, **payload)
        atomic_json(CANDIDATE_FILE, candidate)
        state.update(status="ready", operation=None)
        atomic_json(SESSION_STATE, state)
        atomic_text(READY_FILE, session + "\n")
        # The C Stop path can invalidate the token without waiting for Python's
        # lock. Recheck after publication; a stale ready marker is never enough
        # for confirm(), which revalidates both token and boot-bound state.
        try:
            _session_locked(session)
        except VoiceError:
            _clear_candidate_locked()
            raise
        if defer_api:
            write_message(f"新语音：{transcript}；请再次点击新增API语音任务，发送API并执行")
        else:
            write_message("API候选：" + action_summary(candidate["actions"]) + "；请点击执行API语音任务确认")
        return candidate


def create_candidate(transcript: str, session: str | None = None, defer_api: bool = False) -> dict[str, Any]:
    session, operation = _reserve(session, "interpreting")
    try:
        return _publish_candidate(transcript, session, operation, defer_api)
    except Exception:
        _fail_operation(session, operation)
        raise


def recognize(pcm_path: Path, session: str | None = None, defer_api: bool = False) -> dict[str, Any]:
    session, operation = _reserve(session, "recognizing")
    try:
        transcript = transcribe(pcm_path)
        # Reject cancellation before spending another API request.
        with voice_lock():
            _operation_locked(session, operation)
        return _publish_candidate(transcript, session, operation, defer_api)
    except Exception:
        _fail_operation(session, operation)
        raise


def action_summary(actions: list[dict[str, Any]]) -> str:
    colors = dict(red="红", orange="橙", yellow="黄", green="绿", blue="蓝", purple="紫", pink="粉", any="任意")
    return "→".join(colors[action["parameters"]["color"]] for action in actions)


def _candidate_locked(session: str | None, allow_deferred: bool) -> tuple[dict[str, Any], dict[str, Any]]:
    state = _session_locked(session)
    candidate = read_object(CANDIDATE_FILE)
    stamp = candidate.get("created_ms")
    if (state.get("status") != "ready" or candidate.get("version") != 2 or
            candidate.get("session") != state["session"] or
            candidate.get("clock_id") != clock_id() or type(stamp) is not int or
            not 0 <= now_ms() - stamp <= CANDIDATE_TTL_MS):
        raise VoiceError("候选语音已过期、已确认或不属于本轮，请重新录音")
    if candidate.get("kind") not in (("api", "transcript") if allow_deferred else ("api",)):
        raise VoiceError("当前不是API候选；新录音请点击新增API语音任务")
    normalize_transcript(candidate.get("transcript", ""))
    return state, candidate


def _mission_valid(mission_token: str | None) -> None:
    if not valid_token(mission_token):
        raise VoiceError("缺少本轮任务令牌，不能确认执行")
    try:
        current = MISSION_TOKEN_FILE.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise VoiceError("任务启动已取消") from exc
    if current != mission_token or MISSION_COMMIT_FILE.exists():
        raise VoiceError("任务启动已取消或已有任务正在执行")


def _remove_own_plan(plan_id: str) -> None:
    # Cancellation must never remove a later mission's plan.
    with plan_write_lock():
        for path in (PLAN_FILE, PLAN_STATE_FILE):
            try:
                if read_object(path).get("plan_id") == plan_id:
                    path.unlink(missing_ok=True)
            except VoiceError:
                pass


def confirm(session: str | None = None, mission_token: str | None = None,
            allow_deferred: bool = False) -> dict[str, Any]:
    with voice_lock():
        state, candidate = _candidate_locked(session, allow_deferred)
        _mission_valid(mission_token)
        session = state["session"]
        operation = uuid.uuid4().hex
        state.update(status="confirming", operation=operation)
        atomic_json(SESSION_STATE, state)
        READY_FILE.unlink(missing_ok=True)
    try:
        if candidate["kind"] == "transcript":
            candidate.update(_api_proposal(candidate["transcript"]))
        with voice_lock():
            state = _operation_locked(session, operation)
            _mission_valid(mission_token)
            if not 0 <= now_ms() - candidate["created_ms"] <= CANDIDATE_TTL_MS:
                raise VoiceError("候选语音已过期，请重新录音")
            try:
                actions = validate_actions(candidate["actions"])
            except (KeyError, CapabilityError) as exc:
                raise VoiceError("API候选动作无效") from exc
            if (re.fullmatch(r"[0-9a-f]{32}", str(candidate.get("proposal_id", ""))) is None or
                    any(action["capability"] != "strike_balloon" for action in actions)):
                raise VoiceError("API候选身份或动作无效")
            # This is the sole submission path. No remote /submit is called;
            # only locally validated API actions enter the existing plan gate.
            plan = store_plan(actions, "api_voice_confirmed", candidate["transcript"])
            try:
                _operation_locked(session, operation)
                _mission_valid(mission_token)
            except VoiceError:
                _remove_own_plan(plan["plan_id"])
                raise
            state.update(status="confirmed", operation=None)
            atomic_json(SESSION_STATE, state)
            _clear_candidate_locked()
            write_message("API决策已确认：" + action_summary(actions) + "；等待执行端确认后启动")
            return dict(plan=plan, api_decision=dict(proposal_id=candidate["proposal_id"],
                        session=session, clock_id=candidate["clock_id"],
                        candidate_created_ms=candidate["created_ms"], source="agent_interpret"))
    except Exception:
        _fail_operation(session, operation)
        raise


def cancel(session: str | None = None) -> None:
    with voice_lock():
        if session is not None:
            try:
                current = SESSION_FILE.read_text(encoding="ascii").strip()
            except (OSError, UnicodeError):
                current = None
            if current is not None and current != session:
                return  # A cancelled worker must not delete a newer recording.
        SESSION_FILE.unlink(missing_ok=True)
        _clear_candidate_locked()
        SESSION_STATE.unlink(missing_ok=True)
        write_message("候选语音任务已清除")


def status(session: str | None = None) -> dict[str, Any]:
    with voice_lock():
        try:
            state = _session_locked(session)
            result = dict(session=state["session"], clock_id=state["clock_id"], status=state["status"], ready=False)
            if state.get("status") == "ready":
                _, candidate = _candidate_locked(state["session"], True)
                result.update(ready=True, kind=candidate["kind"], transcript=candidate["transcript"],
                              created_ms=candidate["created_ms"], actions=candidate.get("actions"))
            return result
        except VoiceError:
            return dict(status="unavailable", ready=False)


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--begin-session", metavar="TOKEN")
    group.add_argument("--pcm-file", type=Path)
    group.add_argument("--confirm", action="store_true")
    group.add_argument("--cancel", action="store_true")
    group.add_argument("--status", action="store_true")
    parser.add_argument("--session")
    parser.add_argument("--mission-token")
    parser.add_argument("--defer-api", action="store_true")
    args = parser.parse_args()
    try:
        if args.begin_session is not None:
            begin_session(args.begin_session)
        elif args.pcm_file is not None:
            recognize(args.pcm_file, args.session, args.defer_api)
        elif args.confirm:
            confirm(args.session, args.mission_token, args.defer_api)
        elif args.cancel:
            cancel(args.session)
        else:
            print(json.dumps(status(args.session), ensure_ascii=False))
        return 0
    except (VoiceError, CapabilityError, OSError, ValueError) as exc:
        # Only VoiceError carries vetted user-facing text. File paths and remote
        # payloads from unexpected exceptions must not leak into the RC/logs.
        detail = str(exc) if isinstance(exc, VoiceError) else "本地状态更新失败，请重新录音"
        write_message("语音：" + detail)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
