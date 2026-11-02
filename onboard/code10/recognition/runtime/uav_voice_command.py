#!/usr/bin/env python3
"""Turn DJI Pilot PCM recordings into operator-confirmed UAV plans."""

from __future__ import annotations

import argparse
import io
import json
import os
import time
import urllib.error
import urllib.request
import wave
from pathlib import Path
from typing import Any

from uav_natural_language import CommandRejected, local_parse, store_plan


ROOT = Path(__file__).resolve().parent
CANDIDATE_FILE = ROOT / "voice_command_candidate.json"
MESSAGE_FILE = Path("/tmp/balloon_voice_message.txt")


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
    normalized = transcript.strip().strip("。！？!?，,")
    for source, target in VOICE_NORMALIZATIONS.items():
        normalized = normalized.replace(source, target)
    return normalized


def write_message(message: str) -> None:
    temporary = MESSAGE_FILE.with_suffix(".tmp")
    temporary.write_text(message.strip()[:240] + "\n", encoding="utf-8")
    temporary.replace(MESSAGE_FILE)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def pcm_to_wav(pcm_path: Path) -> bytes:
    pcm = pcm_path.read_bytes()
    if not pcm or len(pcm) % 2:
        raise VoiceError("录音数据无效")
    output = io.BytesIO()
    with wave.open(output, "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(16000)
        stream.writeframes(pcm)
    return output.getvalue()


def post_json(url: str, payload: dict[str, Any], token: str, timeout: int = 45) -> dict[str, Any]:
    request = urllib.request.Request(
        url,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {token}"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            value = json.loads(response.read().decode("utf-8"))
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        raise VoiceError(f"服务连接失败：{exc}") from exc
    if not isinstance(value, dict) or not value.get("ok"):
        raise VoiceError(str(value.get("error", "服务返回异常")) if isinstance(value, dict) else "服务返回异常")
    return value


def transcribe(pcm_path: Path) -> str:
    formal_endpoint = os.environ.get(
        "UAV_VOICE_ASR_FALLBACK_ENDPOINT",
        "http://192.168.168.222:8780/transcribe",
    )
    configured_endpoint = os.environ.get("UAV_VOICE_ASR_ENDPOINT", formal_endpoint)
    # A running PSDK process may retain the retired USB/PC bench endpoint until
    # its next safe restart. Prefer the flight endpoint when that exact stale
    # override is inherited, while preserving normal endpoint override behavior.
    if configured_endpoint.startswith("http://192.168.55.2:18780/"):
        endpoints = list(dict.fromkeys((formal_endpoint, configured_endpoint)))
    else:
        endpoints = list(dict.fromkeys((configured_endpoint, formal_endpoint)))
    token = os.environ.get("UAV_VOICE_ASR_TOKEN", "")
    if not token:
        raise VoiceError("语音识别令牌未配置")
    audio = pcm_to_wav(pcm_path)
    connection_errors: list[str] = []
    for endpoint in endpoints:
        for attempt in range(3):
            request = urllib.request.Request(
                endpoint, data=audio,
                headers={"Content-Type": "audio/wav", "Authorization": f"Bearer {token}"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(request, timeout=120) as response:
                    value = json.loads(response.read().decode("utf-8"))
            except urllib.error.HTTPError as exc:
                detail = exc.read().decode("utf-8", errors="replace")[:200]
                raise VoiceError(f"语音识别失败：HTTP {exc.code} {detail}") from exc
            except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
                connection_errors.append(f"{endpoint} attempt {attempt + 1}: {exc}")
                if attempt < 2:
                    time.sleep(1)
                continue
            transcript = value.get("transcript") if isinstance(value, dict) else None
            if not value.get("ok") or not isinstance(transcript, str) or not transcript.strip():
                raise VoiceError(str(value.get("error", "未识别到语音")))
            return transcript.strip()
    raise VoiceError(f"语音识别连接失败：{'；'.join(connection_errors)}")


def create_candidate(transcript: str) -> dict[str, Any]:
    transcript = normalize_transcript(transcript)
    try:
        actions = local_parse(transcript)
        candidate = {"kind": "local", "transcript": transcript, "actions": actions}
    except CommandRejected:
        endpoint = os.environ.get("UAV_VOICE_AGENT_ENDPOINT", "http://192.168.168.155:8770/interpret")
        token = os.environ.get("UAV_AGENT_AUTH_TOKEN", "")
        if not token:
            raise VoiceError("当前能力库无法理解，且大模型未配置")
        result = post_json(endpoint, {"command": transcript}, token)
        proposal = result.get("result")
        if not isinstance(proposal, dict) or not isinstance(proposal.get("proposal_id"), str):
            raise VoiceError("大模型未返回有效候选计划")
        candidate = {"kind": "agent", "transcript": transcript, "proposal_id": proposal["proposal_id"]}
    atomic_json(CANDIDATE_FILE, candidate)
    return candidate


def recognize(pcm_path: Path) -> None:
    transcript = transcribe(pcm_path)
    candidate = create_candidate(transcript)
    write_message(f"语音：已识别“{candidate['transcript']}”，请按执行语音任务（实飞）")


def confirm() -> None:
    try:
        candidate = json.loads(CANDIDATE_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise VoiceError("没有待确认的语音命令") from exc
    if candidate.get("kind") == "local":
        plan = store_plan(candidate.get("actions"), "voice_confirmed", candidate.get("transcript"))
        plan_id = plan["plan_id"]
    elif candidate.get("kind") == "agent":
        token = os.environ.get("UAV_AGENT_AUTH_TOKEN", "")
        endpoint = os.environ.get("UAV_VOICE_AGENT_SUBMIT_ENDPOINT", "http://192.168.168.155:8770/submit")
        result = post_json(endpoint, {"proposal_id": candidate.get("proposal_id")}, token)
        plan_id = str(result.get("result", {}).get("plan", {}).get("plan_id", "accepted"))
    else:
        raise VoiceError("候选语音命令格式无效")
    CANDIDATE_FILE.unlink(missing_ok=True)
    write_message(f"语音：计划已确认（{plan_id[:8]}），等待执行端确认后自动启动")


def cancel() -> None:
    CANDIDATE_FILE.unlink(missing_ok=True)
    write_message("语音：候选命令已取消")


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pcm-file", type=Path)
    group.add_argument("--confirm", action="store_true")
    group.add_argument("--cancel", action="store_true")
    args = parser.parse_args()
    try:
        if args.pcm_file is not None:
            recognize(args.pcm_file)
        elif args.confirm:
            confirm()
        else:
            cancel()
        return 0
    except (VoiceError, CommandRejected, OSError, ValueError) as exc:
        write_message(f"语音：处理失败（{exc}）")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
