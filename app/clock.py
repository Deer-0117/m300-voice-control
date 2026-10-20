"""One clock for the Linux decoder, detector, tracker and PSDK protocol.

Wall time is metadata only. Wire records must carry the current boot's token;
legacy records and records surviving a reboot are never inferred by magnitude.
"""
from functools import lru_cache
from pathlib import Path
import os
import sys
import time


def now_ms() -> int:
    return time.monotonic_ns() // 1_000_000


@lru_cache(maxsize=1)
def clock_id() -> str:
    if sys.platform != "linux":
        # Local Windows tests only; never a valid Jetson protocol token.
        return f"test-mono-v1:{os.getpid()}"
    boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    if len(boot) != 36 or any(c not in "0123456789abcdef-" for c in boot):
        raise RuntimeError("cannot establish the Linux boot clock identity")
    return f"mono-v1:{boot}"


def log_clock() -> dict:
    return {"clock_id": clock_id(), "wall_ts_ms": time.time_ns() // 1_000_000}


def split_record(line: str, count: int) -> list[str] | None:
    parts = line.strip().split("\t")
    if len(parts) != count + 1 or parts[-1] != clock_id():
        return None
    return parts[:-1]


def frame_timestamp(path: Path) -> int | None:
    """Read decoder sidecar; file mtime is intentionally never a fallback."""
    try:
        parts = split_record(path.with_suffix(path.suffix + ".tsv").read_text(), 2)
        if parts is None or parts[1] != path.name:
            return None
        stamp = int(parts[0])
        return stamp if 0 < stamp <= now_ms() else None
    except (OSError, ValueError):
        return None
