#!/usr/bin/env python3
"""Continuously track the latest YOLO balloon target on Liveview frames.

YOLO remains the source of truth and periodically reinitializes OpenCV's
TrackerMIL. Between detector results, this worker publishes fresh target boxes
for the existing C control loop. Tracking automatically expires when no new
YOLO correction arrives within the configured safety window.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import time
from balloon_clock import now_ms, clock_id, log_clock, split_record, frame_timestamp
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from hardcase_protocol import parse_frame_filename
from performance_timing import PerformanceRecorder

import cv2


ROOT = Path(__file__).resolve().parent
YOLO_TARGET_FILE = ROOT / "latest_yolo_target.tsv"
CONTROL_TARGET_FILE = ROOT / "latest_target.tsv"
LIVEVIEW_METADATA = Path("/dev/shm/balloon_liveview_latest.tsv")
LIVEVIEW_FRAME_DIR = Path("/dev/shm/balloon_liveview_frames")
TRACKER_LOG = ROOT / "tracker_events.jsonl"
FRAME_ID_PATTERN = re.compile(r"frame_(\d+)\.(?:jpg|bmp)$")
TRACKER_SEQ_BIT = 0x80000000



def append_event(path: Path, event: str, **fields: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"ts_ms": now_ms(), **log_clock(), "source": "balloon_tracker_worker.py", "event": event, **fields}
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")


@dataclass(frozen=True)
class TargetRecord:
    seq: int
    width: float
    height: float
    x1: float
    y1: float
    x2: float
    y2: float
    target_ts_ms: int
    finished_ts_ms: int
    confidence: float
    source: Path

    @property
    def key(self) -> tuple[int, int, str]:
        return self.seq, self.target_ts_ms, str(self.source)

    @property
    def bbox(self) -> tuple[float, float, float, float]:
        return self.x1, self.y1, self.x2 - self.x1, self.y2 - self.y1


@dataclass(frozen=True)
class LiveFrame:
    frame_id: int
    ready_ts_ms: int
    path: Path


def parse_target(path: Path) -> TargetRecord | None:
    try:
        parts = split_record(path.read_text(encoding="utf-8"), 11)
        if parts is None:
            return None
        target = TargetRecord(
            seq=int(parts[0]), width=float(parts[1]), height=float(parts[2]),
            x1=float(parts[3]), y1=float(parts[4]), x2=float(parts[5]), y2=float(parts[6]),
            target_ts_ms=int(parts[7]), finished_ts_ms=int(parts[8]),
            confidence=float(parts[9]), source=Path(parts[10]),
        )
    except (FileNotFoundError, OSError, ValueError):
        return None
    if not valid_box(target.bbox, target.width, target.height):
        return None
    return target


def parse_liveview_metadata(path: Path) -> LiveFrame | None:
    try:
        parts = split_record(path.read_text(encoding="utf-8"), 4)
        if parts is None:
            return None
        return LiveFrame(frame_id=int(parts[0]), ready_ts_ms=int(parts[1]), path=Path(parts[3]))
    except (FileNotFoundError, OSError, ValueError):
        return None


def frame_id(path: Path) -> int | None:
    identity = parse_frame_filename(path)
    return identity[1] if identity else None


def frame_session(path: Path) -> str | None:
    identity = parse_frame_filename(path)
    return identity[0] if identity else None


def valid_box(box: tuple[float, float, float, float], width: float, height: float) -> bool:
    x, y, w, h = box
    if width <= 1 or height <= 1 or w < 2 or h < 2 or x < 0 or y < 0:
        return False
    if x + w > width + 1 or y + h > height + 1:
        return False
    return w * h <= width * height * 0.65


def clamp_box(box: Iterable[float], width: int, height: int) -> tuple[float, float, float, float] | None:
    x, y, w, h = (float(value) for value in box)
    x = min(max(0.0, x), max(0.0, width - 2.0))
    y = min(max(0.0, y), max(0.0, height - 2.0))
    w = min(w, width - x)
    h = min(h, height - y)
    result = (x, y, w, h)
    return result if valid_box(result, float(width), float(height)) else None


def select_catchup_frames(frames: list[tuple[int, Path]], limit: int) -> list[tuple[int, Path]]:
    if limit <= 0 or len(frames) <= limit:
        return frames
    # Preserve temporal progression while always ending at the newest frame.
    indices = sorted({min(len(frames) - 1, math.ceil((i + 1) * len(frames) / limit) - 1)
                      for i in range(limit)})
    return [frames[index] for index in indices]


def candidate_frames(frame_dir: Path, after_id: int, latest_id: int,
                     limit: int, session: str = "unknown") -> list[tuple[int, Path]]:
    found: list[tuple[int, Path]] = []
    for pattern in ("frame_*.jpg", "frame_*.bmp"):
        for path in frame_dir.glob(pattern):
            identifier = frame_id(path)
            if (identifier is not None and frame_session(path) == session
                    and after_id < identifier <= latest_id):
                found.append((identifier, path))
    found.sort(key=lambda item: item[0])
    return select_catchup_frames(found, limit)


def create_tracker() -> Any:
    factory = getattr(cv2, "TrackerMIL_create", None)
    if factory is None:
        tracker_class = getattr(cv2, "TrackerMIL", None)
        factory = getattr(tracker_class, "create", None)
    if factory is None:
        raise RuntimeError("OpenCV TrackerMIL is unavailable")
    return factory()


def read_image(path: Path) -> Any | None:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    return image if image is not None and image.size > 0 else None


def atomic_write_target(path: Path, seq: int, width: int, height: int,
                        box: tuple[float, float, float, float], target_ts_ms: int,
                        confidence: float, source: Path) -> None:
    x, y, w, h = box
    line = (
        f"{seq}\t{float(width):.3f}\t{float(height):.3f}\t"
        f"{x:.3f}\t{y:.3f}\t{x + w:.3f}\t{y + h:.3f}\t"
        f"{target_ts_ms}\t{now_ms()}\t{confidence:.6f}\t{source}\t{clock_id()}\n"
    )
    tmp = path.with_suffix(path.suffix + ".tracker.tmp")
    tmp.write_text(line, encoding="utf-8")
    os.replace(tmp, path)


def tracker_sequence(identifier: int) -> int:
    return TRACKER_SEQ_BIT | (identifier & 0x7FFFFFFF)


def tracker_control_timestamp(frame_ts_ms: int) -> int:
    """Use the timestamp of the frame that MIL actually inspected."""
    return frame_ts_ms


def tracker_should_publish(seed_age_ms: int, handoff_delay_ms: int) -> bool:
    """Do not let MIL overwrite a recent YOLO correction."""
    return seed_age_ms >= max(0, handoff_delay_ms)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="YOLO-corrected OpenCV MIL balloon tracker")
    parser.add_argument("--yolo-target", type=Path, default=YOLO_TARGET_FILE)
    parser.add_argument("--control-target", type=Path, default=CONTROL_TARGET_FILE)
    parser.add_argument("--metadata", type=Path, default=LIVEVIEW_METADATA)
    parser.add_argument("--frame-dir", type=Path, default=LIVEVIEW_FRAME_DIR)
    parser.add_argument("--log", type=Path, default=TRACKER_LOG)
    parser.add_argument("--poll-ms", type=int, default=10)
    parser.add_argument("--max-tracker-age-ms", type=int, default=800)
    parser.add_argument("--handoff-delay-ms", type=int, default=450)
    parser.add_argument("--max-catchup-frames", type=int, default=4)
    return parser


def timed_tracker_update(tracker, image, performance):
    """Measure all update attempts, including ones not published to control."""
    profile_started = performance.begin()
    started = time.perf_counter()
    try:
        ok, raw_box = tracker.update(image)
    except Exception:
        finished = performance.begin()
        performance.record('mil_update_call', profile_started, finished, success=False)
        raise
    update_ms = (time.perf_counter()-started)*1000.0
    finished = performance.begin()
    performance.record('mil_update_call', profile_started, finished, success=bool(ok))
    return ok, raw_box, update_ms


def run_tracker(args, performance) -> int:
    stop = False

    def handle_stop(signum: int, frame: Any) -> None:
        nonlocal stop
        stop = True
        append_event(args.log, "tracker_stop_signal", signal=signum)

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)
    append_event(args.log, "tracker_ready", algorithm="OpenCV TrackerMIL",
                 opencv=cv2.__version__, max_tracker_age_ms=args.max_tracker_age_ms,
                 handoff_delay_ms=args.handoff_delay_ms,
                 max_catchup_frames=args.max_catchup_frames)

    tracker: Any | None = None
    seed: TargetRecord | None = None
    last_seed_key: tuple[int, int, str] | None = None
    last_frame_id = 0
    active_session: str | None = None
    handoff_announced = False

    while not stop:
        latest = parse_liveview_metadata(args.metadata)
        latest_session = frame_session(latest.path) if latest is not None else None
        if latest_session is not None and latest_session != active_session:
            active_session = latest_session
            tracker = seed = None
            last_seed_key = None
            last_frame_id = -1
            handoff_announced = False
        correction = parse_target(args.yolo_target)
        if (correction is not None and active_session is not None
                and frame_session(correction.source) != active_session):
            correction = None
        if correction is not None and correction.key != last_seed_key:
            last_seed_key = correction.key
            age_ms = now_ms() - correction.target_ts_ms
            source_id = frame_id(correction.source)
            image = read_image(correction.source) if source_id is not None else None
            if age_ms < 0 or age_ms > args.max_tracker_age_ms:
                append_event(args.log, "tracker_seed_rejected", reason="stale",
                             seq=correction.seq, age_ms=age_ms)
                tracker = None
                seed = None
            elif image is None:
                append_event(args.log, "tracker_seed_rejected", reason="source_unavailable",
                             seq=correction.seq, source=str(correction.source))
                tracker = None
                seed = None
            else:
                height, width = image.shape[:2]
                box = clamp_box(correction.bbox, width, height)
                if box is None:
                    append_event(args.log, "tracker_seed_rejected", reason="invalid_bbox",
                                 seq=correction.seq)
                    tracker = None
                    seed = None
                else:
                    try:
                        tracker = create_tracker()
                        # OpenCV 4.8 Python bindings require an integer Rect.
                        tracker_box = tuple(int(round(value)) for value in box)
                        tracker.init(image, tracker_box)
                        seed = correction
                        handoff_announced = False
                        last_frame_id = int(source_id)
                        append_event(args.log, "tracker_initialized", algorithm="OpenCV TrackerMIL",
                                     seq=seed.seq, source_frame_id=last_frame_id,
                                     confidence=seed.confidence, bbox_xywh=list(tracker_box), age_ms=age_ms)
                    except Exception as exc:
                        append_event(args.log, "tracker_init_error", seq=correction.seq,
                                     error=repr(exc))
                        tracker = None
                        seed = None

        if tracker is None or seed is None:
            time.sleep(max(10, args.poll_ms) / 1000.0)
            continue

        seed_age_ms = now_ms() - seed.target_ts_ms
        if seed_age_ms < 0 or seed_age_ms > args.max_tracker_age_ms:
            append_event(args.log, "tracker_expired", seq=seed.seq, age_ms=seed_age_ms,
                         reason="no_recent_yolo_correction")
            tracker = None
            seed = None
            continue

        if latest is None or latest.frame_id <= last_frame_id:
            time.sleep(max(10, args.poll_ms) / 1000.0)
            continue

        frames = candidate_frames(args.frame_dir, last_frame_id, latest.frame_id,
                                  max(1, args.max_catchup_frames), session=active_session or "unknown")
        if not frames:
            time.sleep(max(10, args.poll_ms) / 1000.0)
            continue

        for identifier, path in frames:
            image = read_image(path)
            if image is None:
                append_event(args.log, "tracker_frame_skipped", frame_id=identifier,
                             reason="decode_failed", source=str(path))
                last_frame_id = identifier
                continue
            try:
                ok, raw_box, update_ms = timed_tracker_update(tracker, image, performance)
            except Exception as exc:
                append_event(args.log, "tracker_update_error", frame_id=identifier,
                             error=repr(exc))
                tracker = None
                seed = None
                break
            height, width = image.shape[:2]
            box = clamp_box(raw_box, width, height) if ok else None
            last_frame_id = identifier
            if box is None:
                append_event(args.log, "tracker_lost", frame_id=identifier,
                             update_ms=round(update_ms, 3))
                tracker = None
                seed = None
                break
            try:
                frame_ts_ms = frame_timestamp(path)
                if frame_ts_ms is None:
                    append_event(args.log, "tracker_frame_skipped", frame_id=identifier,
                                 reason="missing_or_invalid_frame_clock")
                    continue
                seed_age_ms = now_ms() - seed.target_ts_ms
                # YOLO is authoritative. MIL continues updating internally,
                # but only publishes after YOLO has been quiet long enough to
                # avoid racing and overwriting a newer detector result.
                if seed_age_ms < 0 or seed_age_ms > args.max_tracker_age_ms:
                    tracker = seed = None
                    break
                if not tracker_should_publish(seed_age_ms, args.handoff_delay_ms):
                    continue
                if not handoff_announced:
                    append_event(args.log, "tracker_handoff_active",
                                 seed_seq=seed.seq,
                                 seed_age_ms=seed_age_ms,
                                 handoff_delay_ms=args.handoff_delay_ms)
                    handoff_announced = True
                # MIL has inspected this exact frame, so freshness is tied to
                # the frame rather than the older YOLO seed. The independent
                # max-tracker-age window still bounds total prediction time.
                control_target_ts_ms = tracker_control_timestamp(frame_ts_ms)
                atomic_write_target(args.control_target, tracker_sequence(identifier),
                                    width, height, box, control_target_ts_ms,
                                    seed.confidence, path)
                append_event(args.log, "tracker_target_written", frame_id=identifier,
                             seed_seq=seed.seq, frame_ts_ms=frame_ts_ms,
                             target_ts_ms=control_target_ts_ms,
                             seed_age_ms=seed_age_ms,
                             update_ms=round(update_ms, 3), bbox_xywh=[round(v, 3) for v in box])
            except OSError as exc:
                append_event(args.log, "tracker_target_write_error", frame_id=identifier,
                             error=repr(exc))

    append_event(args.log, "tracker_stopped")
    return 0


def main() -> int:
    args = build_parser().parse_args()
    performance = PerformanceRecorder(ROOT/'performance_events.jsonl', 'balloon_tracker_worker.py')
    try:
        return run_tracker(args, performance)
    finally:
        performance.close()


if __name__ == "__main__":
    raise SystemExit(main())
