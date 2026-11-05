"""Local vision evidence bound to a mission, action and captured camera frame.

This channel authorizes visible-target tracking. Legacy TSV/MIL output is still
available for diagnostics, but cannot authorize managed voice flight tasks.
"""
from __future__ import annotations

import math
from pathlib import Path

from balloon_clock import now_ms
from balloon_mission import atomic_json
from uav_capabilities import balloon_matches, canonical_balloon_color

STATUS = Path('/tmp/balloon_visual_status.json')
MAX_FRAME_AGE_MS = 1400


def snapshot(sequence) -> dict | None:
    gate = getattr(sequence, 'mission_gate', None)
    return gate.visual_snapshot(sequence) if gate is not None else None


def frame_is_current(sequence, context: dict | None, payload: dict,
                     width, height, source: str) -> bool:
    """Validate before feeding the stateful task selector, not just publishing."""
    if not context or source not in ('yolo', 'pixel') or snapshot(sequence) != context:
        return False
    capture = payload.get('capture_ts_ms')
    finished = payload.get('detection_finished_ts_ms')
    seq = payload.get('seq')
    current = now_ms()
    if any(isinstance(x, bool) or not isinstance(x, int) for x in (capture, finished, seq)):
        return False
    if not (context['started_ts_ms'] <= capture <= finished <= current and
            current - capture <= MAX_FRAME_AGE_MS and 0 <= seq < 0x80000000):
        return False
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or
           not math.isfinite(x) or not 1 < x <= 32768 for x in (width, height)):
        return False
    return True


def publish(sequence, context: dict | None, payload: dict, best: dict | None,
            width, height, source: str) -> bool:
    """Reject inference results which crossed cancellation or an action change."""
    if not frame_is_current(sequence, context, payload, width, height, source):
        return False
    capture, finished, seq = (payload[k] for k in ('capture_ts_ms', 'detection_finished_ts_ms', 'seq'))
    selected = canonical_balloon_color(best.get('color_name')) if best else None
    found = bool(best and selected in ('yellow', 'pink', 'blue') and
                 balloon_matches(best, context['requested_color']))
    box = best.get('bbox_xyxy', []) if found else []
    confidence = best.get('confidence', 0) if found else 0
    if found:
        if (len(box) != 4 or any(isinstance(x, bool) or not isinstance(x, (int, float)) or
                                not math.isfinite(x) for x in [*box, confidence]) or
                not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height and
                     0 < confidence <= 1)):
            found = False
    receipt = dict(version=1, **context, capture_ts_ms=capture, ts_ms=finished,
                   seq=seq, source=source, width=width, height=height,
                   target_found=found, matches_requested=found,
                   selected_color=selected if found else None)
    if found:
        receipt.update(x1=box[0], y1=box[1], x2=box[2], y2=box[3], confidence=confidence)
    # A cancellation/phase change while checking dimensions must not relabel
    # an old frame. The C consumer also checks the current ACK around its read.
    if snapshot(sequence) != context:
        return False
    atomic_json(STATUS, receipt)
    return True
