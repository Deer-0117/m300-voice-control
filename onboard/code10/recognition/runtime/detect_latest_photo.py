#!/usr/bin/env python3
"""Run YOLO on one PSDK-triggered photo and append a compact JSONL record."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from balloon_clock import now_ms, clock_id, log_clock, split_record, frame_timestamp
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
YOLO_ENV_PYTHON = ROOT / "yolo_env" / "bin" / "python"
YOLO_MODULE = ROOT / "yolo_callable_module"
DEFAULT_LOG = ROOT / "detection_results.jsonl"
TARGET_CHASE_CLI = ROOT / "target_chase" / "target_chase_plan_cli"
CONTROL_EVENT_LOG = ROOT / "control_events.jsonl"
LATEST_TARGET_FILE = ROOT / "latest_target.tsv"

if YOLO_ENV_PYTHON.exists() and Path(sys.executable) != YOLO_ENV_PYTHON:
    os.execv(str(YOLO_ENV_PYTHON), [str(YOLO_ENV_PYTHON), *sys.argv])



def image_size(path: Path) -> tuple[int | None, int | None]:
    try:
        from PIL import Image

        with Image.open(path) as image:
            return image.size
    except Exception:
        return None, None


def prepare_detection_image(path: Path) -> tuple[Path, dict[str, Any]]:
    return path, {
        "enabled": False,
        "reason": "disabled",
        "path": str(path),
    }


def enrich_detection(record: dict[str, Any], width: int | None, height: int | None) -> dict[str, Any]:
    bbox = record.get("bbox_xyxy") or []
    if len(bbox) != 4:
        return record

    x1, y1, x2, y2 = [float(value) for value in bbox]
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    enriched = dict(record)
    enriched["center_xy"] = [cx, cy]

    if width and height and width > 0 and height > 0:
        enriched["center_norm"] = [cx / width, cy / height]
        enriched["area_ratio"] = max(0.0, (x2 - x1) * (y2 - y1)) / float(width * height)

    return enriched


def build_target_chase_plan(
    best: dict[str, Any] | None,
    width: int | None,
    height: int | None,
    target_timestamp_ms: int,
    eval_timestamp_ms: int,
) -> dict[str, Any] | None:
    if not best or not width or not height or width <= 0 or height <= 0:
        return None

    bbox = best.get("bbox_xyxy") or []
    if len(bbox) != 4:
        return None

    if not TARGET_CHASE_CLI.exists():
        return {
            "ok": False,
            "error": f"target chase cli not found: {TARGET_CHASE_CLI}",
            "no_obtain_authority": True,
            "no_execute_joystick": True,
        }

    cmd = [
        str(TARGET_CHASE_CLI),
        "--width",
        str(width),
        "--height",
        str(height),
        "--x1",
        str(float(bbox[0])),
        "--y1",
        str(float(bbox[1])),
        "--x2",
        str(float(bbox[2])),
        "--y2",
        str(float(bbox[3])),
        "--target-ts-ms",
        str(target_timestamp_ms),
        "--eval-ms",
        str(eval_timestamp_ms),
    ]

    try:
        completed = subprocess.run(cmd, check=False, text=True, capture_output=True, timeout=5)
        if completed.returncode != 0:
            return {
                "ok": False,
                "returncode": completed.returncode,
                "stdout": completed.stdout.strip(),
                "stderr": completed.stderr.strip(),
                "no_obtain_authority": True,
                "no_execute_joystick": True,
            }
        plan = json.loads(completed.stdout)
        plan["bbox_xyxy"] = [float(value) for value in bbox]
        plan["target_timestamp_ms"] = target_timestamp_ms
        plan["eval_timestamp_ms"] = eval_timestamp_ms
        plan["target_age_ms"] = max(0, eval_timestamp_ms - target_timestamp_ms)
        return plan
    except Exception as exc:
        return {
            "ok": False,
            "error": repr(exc),
            "no_obtain_authority": True,
            "no_execute_joystick": True,
        }


def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")


def append_control_event(event: str, payload: dict[str, Any] | None = None) -> None:
    record = {
        "ts_ms": now_ms(), **log_clock(),
        "source": "detect_latest_photo.py",
        "event": event,
    }
    if payload:
        record.update(payload)
    append_jsonl(CONTROL_EVENT_LOG, record)


def write_latest_target(payload: dict[str, Any], best: dict[str, Any] | None,
                        width: int | None, height: int | None) -> None:
    if not best or not width or not height:
        append_control_event(
            "target_not_written",
            {
                "seq": payload.get("seq"),
                "reason": "no_detection",
                "source_photo": payload.get("source"),
            },
        )
        return

    bbox = best.get("bbox_xyxy") or []
    if len(bbox) != 4:
        append_control_event(
            "target_not_written",
            {
                "seq": payload.get("seq"),
                "reason": "invalid_bbox",
                "source_photo": payload.get("source"),
            },
        )
        return

    target_ts_ms = int(payload.get("target_timestamp_ms") or 0)
    finished_ts_ms = int(payload.get("detection_finished_ts_ms") or 0)
    confidence = float(best.get("confidence") or 0.0)
    line = (
        f"{int(payload.get('seq') or 0)}\t{float(width):.3f}\t{float(height):.3f}\t"
        f"{float(bbox[0]):.3f}\t{float(bbox[1]):.3f}\t"
        f"{float(bbox[2]):.3f}\t{float(bbox[3]):.3f}\t"
        f"{target_ts_ms}\t{finished_ts_ms}\t{confidence:.6f}\t"
        f"{payload.get('source', '')}\t{clock_id()}\n"
    )
    tmp_path = LATEST_TARGET_FILE.with_suffix(".tsv.tmp")
    tmp_path.write_text(line, encoding="utf-8")
    tmp_path.replace(LATEST_TARGET_FILE)
    append_control_event(
        "target_written",
        {
            "seq": payload.get("seq"),
            "source_photo": payload.get("source"),
            "target_timestamp_ms": target_ts_ms,
            "detection_finished_ts_ms": finished_ts_ms,
            "target_age_ms": payload.get("target_age_ms"),
            "confidence": confidence,
            "bbox_xyxy": [float(value) for value in bbox],
        },
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect balloon targets in the latest PSDK photo.")
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--seq", type=int, default=0)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--target-label", default="balloon")
    parser.add_argument("--capture-ts-ms", type=int, default=0)
    parser.add_argument("--clock-id", default="")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    started = time.perf_counter()
    detection_started_ts_ms = now_ms()
    if args.capture_ts_ms and (args.clock_id != clock_id() or
                              not 0 < args.capture_ts_ms <= detection_started_ts_ms):
        raise ValueError("capture timestamp must carry the current monotonic boot clock id")
    target_timestamp_ms = args.capture_ts_ms if args.capture_ts_ms > 0 else detection_started_ts_ms
    source = args.source.resolve()
    log_path = args.log if args.log.is_absolute() else (ROOT / args.log).resolve()

    payload: dict[str, Any] = {
        "ok": False,
        "ts": int(time.time()),
        "ts_ms": detection_started_ts_ms, **log_clock(),
        "detection_started_ts_ms": detection_started_ts_ms,
        "capture_ts_ms": args.capture_ts_ms if args.capture_ts_ms > 0 else None,
        "target_timestamp_ms": target_timestamp_ms,
        "seq": args.seq,
        "source": str(source),
        "target_label": args.target_label,
    }

    try:
        if not source.is_file():
            raise FileNotFoundError(f"source not found: {source}")

        detect_source, enhancement_info = prepare_detection_image(source)
        sys.path.insert(0, str(YOLO_MODULE))
        import yolo_infer

        config = yolo_infer.load_config()
        width, height = image_size(detect_source)
        result = yolo_infer.run_detection(
            source=detect_source,
            weights=config["weights"].resolve(),
            imgsz=config["imgsz"],
            conf=config["conf"],
            device=config["device"],
            save=config["save"],
            project=YOLO_MODULE / "runs",
            name="latest_photo",
        )
        detections = [enrich_detection(item, width, height) for item in result.get("detections", [])]
        label = args.target_label.lower()
        candidates = [item for item in detections if label in str(item.get("class_name", "")).lower()]
        if not candidates:
            candidates = detections
        best_balloon = max(candidates, key=lambda item: float(item.get("confidence", 0.0))) if candidates else None
        detection_finished_ts_ms = now_ms()
        target_chase_plan = build_target_chase_plan(
            best_balloon,
            width,
            height,
            target_timestamp_ms,
            detection_finished_ts_ms,
        )

        payload.update(result)
        payload.update(
            {
                "ok": True,
                "ts": detection_started_ts_ms // 1000,
                "ts_ms": detection_started_ts_ms, **log_clock(),
                "detection_started_ts_ms": detection_started_ts_ms,
                "detection_finished_ts_ms": detection_finished_ts_ms,
                "target_timestamp_ms": target_timestamp_ms,
                "target_age_ms": max(0, detection_finished_ts_ms - target_timestamp_ms),
                "seq": args.seq,
                "source": str(source),
                "detect_source": str(detect_source),
                "enhancement": enhancement_info,
                "image_size": [width, height],
                "detections": detections,
                "balloon_candidates": candidates,
                "best_balloon": best_balloon,
                "target_chase_plan": target_chase_plan,
            }
        )
        write_latest_target(payload, best_balloon, width, height)
    except Exception as exc:
        payload["error"] = repr(exc)
        append_control_event("detection_error", {"seq": args.seq, "error": repr(exc), "source_photo": str(source)})
    finally:
        payload.setdefault("detection_finished_ts_ms", now_ms())
        payload.setdefault("target_age_ms", max(0, payload["detection_finished_ts_ms"] - target_timestamp_ms))
        payload["wrapper_latency_ms"] = round((time.perf_counter() - started) * 1000, 3)
        append_jsonl(log_path, payload)
        print(json.dumps(payload, ensure_ascii=False, separators=(",", ":")))

    return 0 if payload.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
