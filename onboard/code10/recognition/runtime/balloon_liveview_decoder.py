#!/usr/bin/env python3
"""Decode the DJI PSDK H.264 FIFO and publish low-latency frames in tmpfs."""

from __future__ import annotations

import argparse
import json
import os
import stat
import threading
import time
import uuid
from balloon_mission import atomic_json, read_json
from timestamp_video import TimestampVideoWriter
from balloon_clock import now_ms, clock_id, log_clock, split_record, frame_timestamp
from collections import deque
from pathlib import Path

from hardcase_protocol import DEFAULT_IPC_DIR, Producer

import cv2


DEFAULT_FIFO = Path("/tmp/balloon_liveview.h264")
DEFAULT_FRAME_DIR = Path("/dev/shm/balloon_liveview_frames")
DEFAULT_METADATA = Path("/dev/shm/balloon_liveview_latest.tsv")
DEFAULT_LOG = Path("/home/uav/offline_balloon_strike_test/liveview_decoder.jsonl")
RECORD_REQUEST = Path("/tmp/balloon_prediction_record.request")
RECORD_ROOT = Path("/home/uav/offline_balloon_strike_test/prediction_recordings")


class PredictionRecorder:
    """Optional bounded writer; disk I/O never runs on the decoder read loop."""

    def __init__(self, request: Path, root: Path, fps: float, log: Path) -> None:
        self.request, self.root, self.fps, self.log = request, root, fps, log
        self.queue: deque[tuple[str, int, int, object, str]] = deque()
        self.ack = request.with_name("balloon_prediction_record_ack.json")
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.active = False
        self.stop = False
        self.dropped = 0
        self.dropped_by_token = {}
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, frame_id: int, timestamp_ms: int, frame: object, decoder_session_id: str = "") -> None:
        try:
            token = self.request.read_text(encoding="ascii").strip()
        except OSError:
            token = ""
        requested = bool(token)
        with self.lock:
            self.active = requested
            if requested:
                if len(self.queue) >= 64:
                    self.dropped += 1
                    self.dropped_by_token[token] = self.dropped_by_token.get(token, 0) + 1
                else:
                    self.queue.append((token, frame_id, timestamp_ms, frame.copy(), decoder_session_id))
            self.wake.set()

    def _run(self) -> None:
        writer = index = session = None
        session_token = ""
        last_ack = 0
        while not self.stop:
            self.wake.wait(0.1)
            self.wake.clear()
            try:
                token = self.request.read_text(encoding="ascii").strip()
            except OSError:
                token = ""
            try:
                # Drain frames already accepted before Stop into their original
                # session; never discard the end of a flight or mix two tokens.
                with self.lock:
                    if self.queue:
                        token = self.queue[0][0]
                if writer is not None and token != session_token:
                    writer.release()
                    writer = None
                    index.flush()
                    os.fsync(index.fileno())
                    index.close()
                    index = None
                    with self.lock:
                        session_dropped = self.dropped_by_token.pop(session_token, 0)
                    metadata.update(status="stopped", frames=video_frame, last_frame_ts_ms=timestamp_ms,
                                    stopped_ts_ms=now_ms(), dropped=session_dropped)
                    atomic_json(session / "session.json", metadata)
                    atomic_json(self.ack, dict(token=session_token, clock_id=clock_id(),
                                              ts_ms=now_ms(), status="stopped", directory=str(session)))
                    append_log(self.log, "prediction_record_stopped", directory=str(session), token=session_token, mission_token=session_token, dropped=session_dropped)
                    session = None
                with self.lock:
                    self.active = bool(token)
                    while self.queue and self.queue[0][0] != token:
                        self.queue.popleft()
                    item = self.queue.popleft() if self.queue else None
                if item is None or not token:
                    continue
                _, frame_id, timestamp_ms, frame, decoder_session_id = item
                if writer is None:
                    session_token = token
                    session = self.root / f"run_{timestamp_ms}_{os.getpid()}_{uuid.uuid4().hex[:8]}"
                    session.mkdir(parents=True, exist_ok=False)
                    height, width = frame.shape[:2]
                    writer = TimestampVideoWriter(session / "liveview.mkv", width, height)
                    metadata = dict(token=token, mission_token=token, clock_id=clock_id(),
                                    status="recording", first_frame_ts_ms=timestamp_ms,
                                    decoder_session_id=decoder_session_id,
                                    timestamp_meaning="decoder publish_frame entry; not sensor exposure",
                                    format="Matroska/MJPEG with actual millisecond PTS")
                    try:
                        request = read_json(self.request.with_name("balloon_mission.json"))
                        if request.get("token") == token:
                            metadata["mission"] = request
                    except (OSError, ValueError):
                        pass
                    atomic_json(session / "session.json", metadata)
                    index = (session / "frames.csv").open("w", encoding="utf-8")
                    index.write("video_frame,source_frame_id,frame_ts_ms,clock_id,decoder_session_id\n")
                    video_frame = 0
                    last_ack = 0
                    last_sync = 0
                    append_log(self.log, "prediction_record_started", directory=str(session), token=token)
                if frame.shape[1] != width or frame.shape[0] != height:
                    raise OSError("recording frame geometry changed")
                writer.write(frame, timestamp_ms)
                index.write(f"{video_frame},{frame_id},{timestamp_ms},{clock_id()},{decoder_session_id}\n")
                video_frame += 1
                index.flush()
                writer.flush()
                if last_ack == 0 or now_ms() - last_ack >= 500:
                    index.flush()
                    last_ack = now_ms()
                    if last_sync == 0 or last_ack - last_sync >= 1000:
                        writer.sync()
                        os.fsync(index.fileno())
                        last_sync = last_ack
                    atomic_json(self.ack, dict(token=token, clock_id=clock_id(), ts_ms=last_ack,
                                              status="recording", frames=video_frame, directory=str(session)))
                with self.lock:
                    if self.queue:
                        self.wake.set()
            except Exception as exc:
                if writer is not None:
                    try:
                        writer.release()
                    except OSError:
                        pass
                if index is not None:
                    try:
                        index.close()
                    except OSError:
                        pass
                writer = index = None
                with self.lock:
                    self.queue.clear()
                # Do not delete a different request that arrived during failure.
                try:
                    if self.request.read_text(encoding="ascii").strip() == token:
                        self.request.unlink(missing_ok=True)
                except OSError:
                    pass
                atomic_json(self.ack, dict(token=token, clock_id=clock_id(), ts_ms=now_ms(),
                                          status="failed", error=str(exc)))
                if session is not None:
                    atomic_json(session / "failure.json", dict(token=token, clock_id=clock_id(), error=str(exc), ts_ms=now_ms()))
                append_log(self.log, "prediction_record_failed", token=token, mission_token=token, error=str(exc))
        if writer is not None:
            writer.release()
        if index is not None:
            index.close()



def append_log(path: Path, event: str, **fields: object) -> None:
    payload = {"ts_ms": now_ms(), **log_clock(), "event": event, **fields}
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(payload, separators=(",", ":")) + "\n")


def ensure_fifo(path: Path) -> None:
    try:
        mode = path.stat().st_mode
        if stat.S_ISFIFO(mode):
            return
        path.unlink()
    except FileNotFoundError:
        pass
    os.mkfifo(path, 0o666)


def hardware_pipeline(fifo: Path) -> str:
    return (
        f"filesrc location={fifo} do-timestamp=true ! "
        "h264parse ! nvv4l2decoder disable-dpb=true enable-max-performance=true ! "
        "nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! "
        "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
    )


def software_pipeline(fifo: Path) -> str:
    return (
        f"filesrc location={fifo} do-timestamp=true ! "
        "h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def atomic_write(path: Path, data: bytes) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def resize_for_publish(frame: object, max_frame_side: int) -> object:
    height, width = frame.shape[:2]
    longest_side = max(int(width), int(height))
    if max_frame_side <= 0 or longest_side <= max_frame_side:
        return frame
    ratio = float(max_frame_side) / float(longest_side)
    target = (max(1, round(width * ratio)), max(1, round(height * ratio)))
    return cv2.resize(frame, target, interpolation=cv2.INTER_AREA)


def near_rate_hint_active(path: Path, max_age_ms: int) -> bool:
    try:
        parts = split_record(path.read_text(encoding="ascii"), 1)
        if parts is None:
            return False
        hinted_ms = int(parts[0])
    except (FileNotFoundError, OSError, ValueError):
        return False
    current_ms = now_ms()
    return (
        hinted_ms > 0
        and current_ms >= hinted_ms
        and current_ms - hinted_ms <= max(100, max_age_ms)
    )


def publish_frame(frame_dir: Path, metadata: Path, frame_id: int, frame: object,
                  jpeg_quality: int, max_frame_side: int = 0,
                  frame_format: str = "jpg", hardcase: Producer | None = None) -> tuple[Path, int, int, int, int]:
    # Timestamp the decoded image before resize/encode/write latency.
    frame_ts_ms = now_ms()
    source_mono_ns = time.monotonic_ns() if hardcase is not None else None
    wall_utc_ms = time.time_ns() // 1_000_000 if hardcase is not None else None
    snapshot = hardcase.snapshot() if hardcase is not None else None
    encode_started = time.perf_counter_ns()
    published = resize_for_publish(frame, max_frame_side)
    normalized_format = frame_format.strip().lower()
    if normalized_format not in {"jpg", "bmp"}:
        raise ValueError(f"unsupported frame format: {frame_format}")
    extension = f".{normalized_format}"
    encode_options = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality] if normalized_format == "jpg" else []
    ok, encoded = cv2.imencode(extension, published, encode_options)
    if not ok:
        raise RuntimeError("cv2.imencode returned false")
    encoded_bytes = encoded.tobytes()
    session_suffix = f"_{hardcase.session_id}" if hardcase is not None else ""
    frame_path = frame_dir / f"frame_{frame_id:012d}{session_suffix}{extension}"
    atomic_write(frame_path, encoded_bytes)
    atomic_write(frame_path.with_suffix(extension + ".tsv"),
                 f"{frame_ts_ms}\t{frame_path.name}\t{clock_id()}\n".encode("ascii"))
    encode_ms = (time.perf_counter_ns() - encode_started) // 1_000_000
    metadata_payload = f"{frame_id}\t{frame_ts_ms}\t{encode_ms}\t{frame_path}\t{clock_id()}\n".encode("utf-8")
    atomic_write(metadata, metadata_payload)
    height, width = published.shape[:2]
    if hardcase is not None:
        try:
            hardcase.frame_ready(frame_path, frame_id, int(width), int(height),
                                 source_mono_ns, wall_utc_ms, frame_ts_ms, encode_ms,
                                 snapshot=snapshot)
        except Exception:
            # Collection cannot turn a successful business publication into failure.
            pass
    return frame_path, int(encode_ms), int(width), int(height), frame_ts_ms


def decode_loop(args: argparse.Namespace) -> None:
    frame_dir: Path = args.frame_dir
    metadata: Path = args.metadata
    fifo: Path = args.fifo
    log_path: Path = args.log
    frame_dir.mkdir(parents=True, exist_ok=True)
    metadata.parent.mkdir(parents=True, exist_ok=True)
    ensure_fifo(fifo)

    for pattern in ("frame_*.jpg", "frame_*.bmp", "frame_*.jpg.tsv", "frame_*.bmp.tsv"):
        for old in frame_dir.glob(pattern):
            try:
                old.unlink()
            except OSError:
                pass
    try:
        metadata.unlink()
    except FileNotFoundError:
        pass

    normal_min_publish_ns = int(
        1_000_000_000 / max(1.0, args.publish_fps)
    )
    near_min_publish_ns = int(
        1_000_000_000 /
        max(args.publish_fps, args.near_publish_fps, 1.0)
    )
    last_publish_ns = 0
    hardcase = Producer(True, args.hardcase_ipc_dir) if args.hardcase_enable else None
    frame_id = 0 if hardcase is not None else time.monotonic_ns()
    retained: deque[Path] = deque()
    recorder = PredictionRecorder(args.record_request, args.record_root,
                                  max(args.publish_fps, args.near_publish_fps), log_path)
    # DJI M300/H20 streams may use GDR encoding. On this aircraft the hardware
    # decoder can block inside VideoCapture for the entire next stream after a
    # previous stream ends. Software-only mode avoids that cross-session stall.
    pipeline_options = {
        "software": [("software", software_pipeline(fifo))],
        "hardware": [("hardware", hardware_pipeline(fifo))],
        "auto": [
            ("software", software_pipeline(fifo)),
            ("hardware", hardware_pipeline(fifo)),
        ],
    }
    pipelines = pipeline_options[args.decoder]

    append_log(log_path, "decoder_start", fifo=str(fifo),
               publish_fps=args.publish_fps,
               near_publish_fps=args.near_publish_fps,
               near_rate_hint_max_age_ms=args.near_rate_hint_max_age_ms,
               opencv=cv2.__version__, decoder_mode=args.decoder,
               frame_format=args.frame_format, max_frame_side=args.max_frame_side)
    while True:
        for decoder_name, pipeline in pipelines:
            append_log(log_path, "pipeline_open", decoder=decoder_name, pipeline=pipeline)
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not capture.isOpened():
                append_log(log_path, "pipeline_open_failed", decoder=decoder_name)
                capture.release()
                continue

            received = 0
            while True:
                ok, frame = capture.read()
                if not ok or frame is None:
                    append_log(log_path, "stream_read_ended", decoder=decoder_name,
                               received_frames=received)
                    break
                received += 1
                current_ns = time.monotonic_ns()
                min_publish_ns = (
                    near_min_publish_ns
                    if near_rate_hint_active(
                        args.near_rate_hint, args.near_rate_hint_max_age_ms
                    )
                    else normal_min_publish_ns
                )
                if last_publish_ns and current_ns - last_publish_ns < min_publish_ns:
                    continue

                frame_id += 1
                if hardcase is not None and frame_id > 999999999999:
                    hardcase.session_id = uuid.uuid4().hex
                    frame_id = 1
                try:
                    frame_path, encode_ms, published_width, published_height, frame_ts_ms = publish_frame(
                        frame_dir, metadata, frame_id, frame, args.jpeg_quality,
                        args.max_frame_side, args.frame_format,
                        hardcase=hardcase,
                    )
                except Exception as exc:
                    append_log(log_path, "publish_failed", frame_id=frame_id, error=repr(exc))
                    continue
                last_publish_ns = current_ns
                if args.record_request.exists():
                    recorder.submit(frame_id, frame_ts_ms,
                                    resize_for_publish(frame, args.max_frame_side),
                                    decoder_session_id=hardcase.session_id if hardcase else "")
                retained.append(frame_path)
                while len(retained) > args.retain_frames:
                    stale = retained.popleft()
                    try:
                        stale.unlink()
                        stale.with_suffix(stale.suffix + ".tsv").unlink(missing_ok=True)
                    except FileNotFoundError:
                        pass
                if frame_id == 1 or frame_id % 100 == 0:
                    append_log(log_path, "frame_published", decoder=decoder_name,
                               frame_id=frame_id, encode_ms=encode_ms,
                               source_width=int(frame.shape[1]), source_height=int(frame.shape[0]),
                               width=published_width, height=published_height,
                               frame_format=args.frame_format)

            capture.release()
            time.sleep(0.2)
        time.sleep(0.5)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fifo", type=Path, default=DEFAULT_FIFO)
    parser.add_argument("--frame-dir", type=Path, default=DEFAULT_FRAME_DIR)
    parser.add_argument("--metadata", type=Path, default=DEFAULT_METADATA)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--publish-fps", type=float, default=10.0)
    parser.add_argument("--near-publish-fps", type=float, default=18.0)
    parser.add_argument(
        "--near-rate-hint",
        type=Path,
        default=Path("/dev/shm/balloon_near_rate.hint"),
    )
    parser.add_argument("--near-rate-hint-max-age-ms", type=int, default=500)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--frame-format", choices=("jpg", "bmp"), default="jpg")
    parser.add_argument(
        "--max-frame-side",
        type=int,
        default=0,
        help="resize frames so the longest side is at most this value; 0 disables resizing",
    )
    parser.add_argument("--retain-frames", type=int, default=40)
    parser.add_argument("--record-request", type=Path, default=RECORD_REQUEST)
    parser.add_argument("--record-root", type=Path, default=RECORD_ROOT)
    parser.add_argument("--decoder", choices=("software", "hardware", "auto"),
                        default="software")
    capture = parser.add_mutually_exclusive_group()
    capture.add_argument("--hardcase-enable", dest="hardcase_enable", action="store_true")
    capture.add_argument("--no-hardcase", dest="hardcase_enable", action="store_false")
    parser.set_defaults(hardcase_enable=True)
    parser.add_argument("--hardcase-ipc-dir", type=Path, default=DEFAULT_IPC_DIR)
    return parser


if __name__ == "__main__":
    decode_loop(build_parser().parse_args())
