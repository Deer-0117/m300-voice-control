#!/usr/bin/env python3
"""Persistent selectable detector worker for PSDK balloon photo requests.

The PSDK app writes only the latest downloaded photo request to
latest_detection_request.tsv. This worker keeps one detector loaded and processes
the newest request, intentionally dropping intermediate overwritten requests.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import os
import signal
import sys
import time
import uuid
from balloon_clock import now_ms, clock_id, log_clock, split_record, frame_timestamp
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hardcase_protocol import DEFAULT_IPC_DIR, WorkerProducer, restore_source_references
import balloon_visual_receipt as visual_receipt
from detector_selector import load_detector as select_detector, detector_metadata

from uav_capabilities import (
    CapabilityError,
    COLOR_GROUPS,
    DEFAULT_TEST_ACTION,
    balloon_matches,
    canonical_balloon_color,
    validate_actions,
)


ROOT = Path(__file__).resolve().parent
YOLO_ENV_PYTHON = ROOT / "yolo_env" / "bin" / "python"
YOLO_MODULE = ROOT / "yolo_callable_module"
DEFAULT_LOG = ROOT / "detection_results.jsonl"
CONTROL_EVENT_LOG = ROOT / "control_events.jsonl"
LATENCY_EVENT_LOG = ROOT / "latency_events.jsonl"
LATEST_TARGET_FILE = ROOT / "latest_target.tsv"
LATEST_YOLO_TARGET_FILE = ROOT / "latest_yolo_target.tsv"
REQUEST_FILE = ROOT / "latest_detection_request.tsv"
WARMUP_IMAGE = ROOT / "worker_warmup.jpg"
FAR_TILE_DIR = Path("/dev/shm/balloon_far_tiles")
DEFAULT_TRT_ENGINE = YOLO_MODULE / "weights" / "best_fp16.engine"
TASK_PLAN_FILE = ROOT / "nl_task_plan.json"
TASK_STATE_FILE = ROOT / "nl_task_state.json"
STOP_REQUEST_FILE = Path("/tmp/balloon_nl_stop.request")
SEARCH_REQUEST_FILE = Path("/tmp/balloon_target_search.request")
NEAR_RATE_HINT_FILE = Path("/dev/shm/balloon_near_rate.hint")
WORKER_SESSION_ID = uuid.uuid4().hex

# The service chooses the interpreter; importing this module never re-execs it.



def append_jsonl(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as file:
        file.write(json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n")


def append_control_event(event: str, payload: dict[str, Any] | None = None) -> None:
    record = {
        "ts_ms": now_ms(), **log_clock(),
        "source": "balloon_yolo_worker.py",
        "event": event,
    }
    try:
        record["mission_token"] = Path("/tmp/balloon_evidence.token").read_text(encoding="ascii").strip()
    except OSError:
        record["mission_token"] = ""
    if payload:
        record.update(payload)
    append_jsonl(CONTROL_EVENT_LOG, record)


def append_latency(stage: str, seq: int, duration_ms: float, **extra: Any) -> None:
    valid = (not isinstance(duration_ms, bool) and isinstance(duration_ms, (int, float))
             and math.isfinite(duration_ms) and duration_ms >= 0)
    record: dict[str, Any] = {
        "ts_ms": now_ms(), **log_clock(),
        "source": "balloon_yolo_worker.py",
        "stage": stage,
        "seq": seq,
        "worker_session_id": WORKER_SESSION_ID,
        "timing_valid": valid,
        "duration_ms": round(float(duration_ms), 3) if valid else None,
    }
    if not valid:
        record["timing_error"] = "nonfinite_negative_or_invalid_duration"
    record.update(extra)
    append_jsonl(LATENCY_EVENT_LOG, record)


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def build_color_masks(hue: Any, saturation: Any, value: Any, chromatic: Any) -> dict[str, Any]:
    """Return mutually exclusive masks calibrated against captured liveview frames.

    These are camera-space profiles rather than textbook HSV bands.  In the
    aircraft stream, yellow can shift towards orange, green towards cyan, and
    pale pink loses much of its saturation.
    """
    pale_pink_signature = (
        ((saturation <= 90) & (value >= 105))
        | ((value - saturation * 0.35) >= 130)
    )
    # Flight samples put the pink/blue boundary around H=123..126.  In that
    # narrow band pink is distinctly bright and weakly saturated, while the
    # purple balloon is darker and more saturated.
    pink_boundary = (
        (hue >= 123)
        & (hue <= 126)
        & (saturation <= 60)
        & (value >= 145)
    )
    pink_like = (
        (
            ((hue >= 127) & (hue <= 173) & pale_pink_signature)
            | pink_boundary
            | (((hue >= 174) | (hue <= 4)) & pale_pink_signature)
        )
        & chromatic
    )
    yellow_like = (
        (
            ((hue >= 28) & (hue <= 55))
            | (
                (hue >= 18)
                & (hue <= 27)
                & (value >= 145)
                & (saturation <= 105)
            )
        )
        & (pink_like == 0)
        & chromatic
    )
    # The recorded green balloon is bright, low-saturation and can reach H=91.
    # In the green/blue overlap, retain that distinctive camera signature.
    cyan_green_like = (
        (hue >= 90)
        & (hue <= 96)
        & (saturation <= 90)
        & (value >= 145)
        & chromatic
    )
    # Direct sun and the liveview white balance shift a saturated physical red
    # balloon toward orange-red (typically H=5..12).  The recorded orange
    # balloon also reaches H=9 but is distinctly less saturated (S~=119), so
    # retain that calibrated separation instead of widening hue alone.
    shifted_saturated_red = (
        (hue >= 5)
        & (hue <= 12)
        & (saturation >= 145)
    )
    red_like = (
        ((hue <= 4) | (hue >= 170) | shifted_saturated_red)
        & (pink_like == 0)
        & chromatic
    )
    purple_like = (
        (hue >= 123)
        & (hue <= 164)
        & (pink_like == 0)
        & (
            (hue >= 127)
            | ((value <= 140) & (saturation >= 55))
        )
        & chromatic
    )
    blue_like = (
        (hue >= 90)
        & (hue <= 126)
        & (cyan_green_like == 0)
        & (pink_like == 0)
        & (purple_like == 0)
        & chromatic
    )
    return {
        "red": red_like,
        "orange": (
            (hue >= 5)
            & (hue <= 27)
            & (red_like == 0)
            & (yellow_like == 0)
            & chromatic
        ),
        "yellow": yellow_like,
        "green": (
            (((hue >= 43) & (hue <= 89)) | cyan_green_like)
            & (yellow_like == 0)
            & chromatic
        ),
        "blue": blue_like,
        "purple": purple_like,
        "pink": pink_like,
    }


def classify_balloon_colors(image_path: Path, detections: list[dict[str, Any]]) -> None:
    """Classify each generic balloon box into yellow, pink, or blue."""
    try:
        import cv2
        import numpy as np

        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("image decode failed")
        image_height, image_width = image.shape[:2]
        for detection in detections:
            bbox = detection.get("bbox_xyxy") or []
            if len(bbox) != 4:
                detection.update(color_name="unknown", color_confidence=0.0)
                continue
            x1, y1, x2, y2 = (float(value) for value in bbox)
            margin_x = max(1.0, (x2 - x1) * 0.15)
            margin_y = max(1.0, (y2 - y1) * 0.15)
            left = max(0, min(image_width - 1, int(round(x1 + margin_x))))
            top = max(0, min(image_height - 1, int(round(y1 + margin_y))))
            right = max(left + 1, min(image_width, int(round(x2 - margin_x))))
            bottom = max(top + 1, min(image_height, int(round(y2 - margin_y))))
            crop = image[top:bottom, left:right]
            if crop.size == 0:
                detection.update(color_name="unknown", color_confidence=0.0)
                continue
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            hue, saturation, value = cv2.split(hsv)
            # Use an ellipse inside the already inset box.  YOLO boxes contain
            # corners of sky/trees/walls; balloons occupy the centre far more
            # reliably, especially when they are only a few dozen pixels wide.
            crop_height, crop_width = crop.shape[:2]
            grid_y, grid_x = np.ogrid[:crop_height, :crop_width]
            radius_x = max(1.0, crop_width * 0.50)
            radius_y = max(1.0, crop_height * 0.50)
            central = (
                ((grid_x - (crop_width - 1) * 0.5) / radius_x) ** 2
                + ((grid_y - (crop_height - 1) * 0.5) / radius_y) ** 2
            ) <= 1.0

            # Normal coloured pixels need S>=32.  Very bright pixels may use
            # S>=22, which recovers the pale pink balloon without admitting
            # neutral white/grey highlights and clouds.
            chromatic = (
                central
                & (value >= 35)
                & ((saturation >= 32) | ((saturation >= 22) & (value >= 150)))
            )
            chromatic_count = int(np.count_nonzero(chromatic))
            if chromatic_count < max(20, int(crop.shape[0] * crop.shape[1] * 0.05)):
                detection.update(color_name="unknown", color_confidence=0.0)
                continue
            fine_masks = build_color_masks(hue, saturation, value, chromatic)
            group_masks = {
                group: np.logical_or.reduce(
                    [
                        mask
                        for name, mask in fine_masks.items()
                        if COLOR_GROUPS[name] == group
                    ]
                )
                for group in ("yellow", "pink", "blue")
            }
            ratios = {
                name: float(np.count_nonzero(mask)) / chromatic_count
                for name, mask in group_masks.items()
            }
            ranked = sorted(ratios.items(), key=lambda item: item[1], reverse=True)
            color_name, confidence = ranked[0]
            runner_up_name, runner_up_confidence = ranked[1]
            margin = confidence - runner_up_confidence
            if confidence < 0.30 or margin < 0.08:
                color_name, confidence = "unknown", 0.0
            selected_values = chromatic
            detection.update(
                color_name=color_name,
                color_confidence=round(confidence, 4),
                color_runner_up=runner_up_name,
                color_margin=round(margin, 4),
                color_chromatic_ratio=round(
                    chromatic_count / max(1, crop_height * crop_width), 4
                ),
                color_hsv_median=[
                    int(np.median(hue[selected_values])),
                    int(np.median(saturation[selected_values])),
                    int(np.median(value[selected_values])),
                ],
            )
    except Exception as exc:
        for detection in detections:
            detection.update(color_name="unknown", color_confidence=0.0)
        append_control_event("color_classification_error", {"source": str(image_path), "error": repr(exc)})


class CapabilityTaskSequence:
    def __init__(self) -> None:
        self.plan_id = ""
        self.api_motion = False
        self.actions: list[dict[str, Any]] = []
        self.current_index = 0
        self.completed_indices: set[int] = set()
        self.hit_armed = False
        self.near_arm_count = 0
        self.missing_since_ms: int | None = None
        self.search_missing_since_ms: int | None = None
        self.target_search_hook_active = False
        self.observed_color_cache: dict[str, dict[str, Any]] = {}
        self.locked_center_norm: tuple[float, float] | None = None
        self.locked_last_seen_ms: int | None = None
        self.locked_area_ratio = 0.0
        self.locked_near_until_ms: int | None = None
        self.pending_switch_center_norm: tuple[float, float] | None = None
        self.pending_switch_count = 0
        self.pending_acquire_center_norm: tuple[float, float] | None = None
        self.pending_acquire_color = ""
        self.pending_acquire_count = 0
        self.pending_acquire_edge = False
        self.last_target_center_norm: tuple[float, float] | None = None
        self.last_target_seen_at_edge = False
        self.edge_hit_recovery_pending_logged = False
        self.hit_approach_evidence = False
        self.hit_min_area_ratio: float | None = None
        self.edge_arm_rejection_logged = False
        self.last_candidate_rejection_diag_ms = 0
        self.status = "idle"

    def _reset_target_lock(self) -> None:
        self.locked_center_norm = None
        self.locked_last_seen_ms = None
        self.locked_area_ratio = 0.0
        self.locked_near_until_ms = None
        self.pending_switch_center_norm = None
        self.pending_switch_count = 0
        self.pending_acquire_center_norm = None
        self.pending_acquire_color = ""
        self.pending_acquire_count = 0
        self.pending_acquire_edge = False
        self.last_target_center_norm = None
        self.last_target_seen_at_edge = False
        self.edge_hit_recovery_pending_logged = False
        self.hit_approach_evidence = False
        self.hit_min_area_ratio = None
        self.edge_arm_rejection_logged = False
        self.last_candidate_rejection_diag_ms = 0

    def _remember_target_exit_geometry(self, detection: dict[str, Any]) -> None:
        center = self._detection_center(detection)
        if center is None:
            return
        edge_margin = min(
            0.25,
            max(
                0.02,
                float(
                    os.environ.get(
                        "BALLOON_HIT_EDGE_REJECT_MARGIN_NORM", "0.10"
                    )
                ),
            ),
        )
        self.last_target_center_norm = center
        self.last_target_seen_at_edge = (
            center[0] <= edge_margin
            or center[0] >= 1.0 - edge_margin
            or center[1] <= edge_margin
            or center[1] >= 1.0 - edge_margin
        )

    @staticmethod
    def _publish_near_rate_hint(detection: dict[str, Any]) -> None:
        area_ratio = float(detection.get("area_ratio") or 0.0)
        observation_age_ms = int(detection.get("observation_age_ms") or 0)
        min_area_ratio = float(
            os.environ.get("BALLOON_NEAR_RECOVERY_MIN_AREA_RATIO", "0.006")
        )
        max_latency_ms = int(
            os.environ.get("BALLOON_NEAR_HIGH_RATE_MAX_LATENCY_MS", "160")
        )
        if (
            area_ratio < min_area_ratio
            or observation_age_ms <= 0
            or observation_age_ms > max_latency_ms
        ):
            return
        try:
            temporary = NEAR_RATE_HINT_FILE.with_suffix(".tmp")
            temporary.write_text(f"{now_ms()}\t{clock_id()}\n", encoding="ascii")
            temporary.replace(NEAR_RATE_HINT_FILE)
        except OSError:
            pass

    def requested_color(self) -> str:
        self._load_plan()
        if (
            self.status == "active"
            and 0 <= self.current_index < len(self.actions)
            and self.actions[self.current_index].get("capability") == "strike_balloon"
        ):
            return str(self.actions[self.current_index]["parameters"]["color"])
        return str(DEFAULT_TEST_ACTION["parameters"]["color"])

    def should_use_far_tiles(self, detections: list[dict[str, Any]]) -> bool:
        if os.environ.get("BALLOON_FAR_TILE_ENABLED", "1").strip().lower() in {
            "0", "false", "no", "off"
        }:
            return False
        # Give every fresh lock full-frame refresh opportunities, including small
        # targets. Resume tiled search when the lock grace expires.
        lock_grace_ms = max(
            0, int(os.environ.get("BALLOON_FAR_TILE_LOCK_GRACE_MS", "700"))
        )
        if (
            self.locked_center_norm is not None
            and self.locked_last_seen_ms is not None
            and now_ms() - self.locked_last_seen_ms <= lock_grace_ms
        ):
            return False
        color = self.requested_color()
        threshold = (
            float(os.environ.get("BALLOON_TARGET_CONTINUE_CONFIDENCE", "0.25"))
            if self.locked_center_norm is not None
            else float(os.environ.get("BALLOON_TARGET_ACQUIRE_CONFIDENCE", "0.45"))
        )
        return not any(
            balloon_matches(item, color)
            and float(item.get("confidence") or 0.0) >= threshold
            for item in detections
        )

    def _reset(self) -> None:
        self._stop_target_search_hook("sequence_reset")
        self.plan_id = ""
        self.api_motion = False
        self.actions = []
        self.current_index = 0
        self.completed_indices = set()
        self.hit_armed = False
        self.near_arm_count = 0
        self.missing_since_ms = None
        self.search_missing_since_ms = None
        self.observed_color_cache = {}
        self._reset_target_lock()
        self.status = "idle"

    def _start_target_search_hook(self, color: str) -> None:
        """Request an immediate gimbal/body search for the current stage."""
        if self.target_search_hook_active:
            return
        self.target_search_hook_active = True
        try:
            SEARCH_REQUEST_FILE.write_text(
                (
                    f"plan_id={self.plan_id}\n"
                    f"index={self.current_index}\n"
                    f"color={color}\n"
                    f"fast_pass={1 if self.current_index > 0 else 0}\n"
                    f"ts_ms={now_ms()}\n"
                ),
                encoding="utf-8",
            )
        except OSError as exc:
            self.target_search_hook_active = False
            append_control_event(
                "capability_target_search_request_failed",
                {
                    "plan_id": self.plan_id,
                    "index": self.current_index,
                    "color": color,
                    "error": repr(exc),
                },
            )
            return
        append_control_event(
            "capability_target_search_requested",
            {
                "plan_id": self.plan_id,
                "index": self.current_index,
                "color": color,
                "implemented": True,
                "requested_pattern": "yaw_sweep",
            },
        )

    def _stop_target_search_hook(self, reason: str) -> None:
        if not self.target_search_hook_active:
            return
        self.target_search_hook_active = False
        try:
            SEARCH_REQUEST_FILE.unlink(missing_ok=True)
        except OSError:
            pass
        append_control_event(
            "capability_target_search_stopped",
            {
                "plan_id": self.plan_id,
                "index": self.current_index,
                "reason": reason,
                "implemented": True,
            },
        )

    def _load_plan(self) -> None:
        try:
            payload = json.loads(TASK_PLAN_FILE.read_text(encoding="utf-8"))
            plan_id = str(payload.get("plan_id") or "")
            if not plan_id:
                return
            actions = validate_actions(payload.get("actions"))
            if plan_id == self.plan_id:
                return
            self._stop_target_search_hook("plan_replaced")
            self.plan_id = plan_id
            self.api_motion = payload.get("motion_source") == "api"
            self.actions = actions
            self.current_index = 0
            self.completed_indices = set()
            self.hit_armed = False
            self.near_arm_count = 0
            self.missing_since_ms = None
            self.search_missing_since_ms = None
            self.target_search_hook_active = False
            self.observed_color_cache = {}
            self._reset_target_lock()
            self.status = "active"
            restored = False
            try:
                saved_state = json.loads(TASK_STATE_FILE.read_text(encoding="utf-8"))
                if str(saved_state.get("plan_id") or "") == plan_id:
                    saved_status = str(saved_state.get("status") or "")
                    saved_completed = saved_state.get("completed_indices")
                    if saved_status == "completed":
                        self.completed_indices = set(range(len(actions)))
                        self.current_index = len(actions)
                        self.status = "completed"
                        restored = True
                    elif saved_status in {"active", "awaiting_confirmation"}:
                        if isinstance(saved_completed, list):
                            self.completed_indices = {
                                int(index) for index in saved_completed
                                if isinstance(index, int) and 0 <= index < len(actions)
                            }
                        else:
                            saved_index = int(saved_state.get("current_index") or 0)
                            self.completed_indices = set(range(min(saved_index, len(actions))))
                        saved_index = int(saved_state.get("current_index") or 0)
                        if 0 <= saved_index < len(actions) and saved_index not in self.completed_indices:
                            self.current_index = saved_index
                        else:
                            remaining = [
                                index for index in range(len(actions))
                                if index not in self.completed_indices
                            ]
                            self.current_index = min(remaining) if remaining else len(actions)
                        self.status = "active" if self.current_index < len(actions) else "completed"
                        if saved_status == "awaiting_confirmation" and self.status == "active":
                            self.status = "awaiting_confirmation"
                            STOP_REQUEST_FILE.write_text(
                                f"plan_id={self.plan_id}\nreason=capability_hit_unconfirmed\n",
                                encoding="utf-8")
                        restored = True
            except (FileNotFoundError, OSError, json.JSONDecodeError, TypeError, ValueError):
                restored = False
            event = "plan_restored" if restored else "plan_loaded"
            self._save_state(event)
            append_control_event(
                f"capability_{event}",
                {
                    "plan_id": plan_id,
                    "actions": actions,
                    "current_index": self.current_index,
                    "completed_indices": sorted(self.completed_indices),
                    "status": self.status,
                },
            )
        except FileNotFoundError:
            # A missing plan is the standalone "any balloon" test mode.
            # Preserve its in-memory target lock between frames. If an
            # explicit plan was removed, reset the old sequence state once.
            if self.plan_id or self.actions:
                self._reset()
            return
        except (OSError, json.JSONDecodeError, CapabilityError) as exc:
            append_control_event("capability_plan_rejected", {"error": str(exc)})
            return

    def _save_state(self, event: str) -> None:
        atomic_write_json(
            TASK_STATE_FILE,
            {
                "version": 2,
                "plan_id": self.plan_id,
                "status": self.status,
                "current_index": self.current_index,
                "completed_indices": sorted(self.completed_indices),
                "hit_armed": self.hit_armed,
                "near_arm_count": self.near_arm_count,
                "hit_approach_evidence": self.hit_approach_evidence,
                "hit_min_area_ratio": self.hit_min_area_ratio,
                "missing_since_ms": self.missing_since_ms,
                "search_missing_since_ms": self.search_missing_since_ms,
                "target_search_hook_active": self.target_search_hook_active,
                # This cache is diagnostic/search memory only.  Cached boxes
                # are never returned to the flight controller as fresh
                # coordinates.
                "observed_colors": {
                    color: observation
                    for color, observation in sorted(self.observed_color_cache.items())
                },
                "current_action": (
                    self.actions[self.current_index]
                    if 0 <= self.current_index < len(self.actions) else None
                ),
                "event": event,
                "updated_ts_ms": time.time_ns() // 1_000_000,
                "runtime_clock_id": clock_id(),
            },
        )

    def _request_stop(self, reason: str) -> None:
        self._stop_target_search_hook(reason)
        STOP_REQUEST_FILE.write_text(
            f"plan_id={self.plan_id}\nreason={reason}\nts_ms={now_ms()}\n", encoding="utf-8"
        )
        self.status = "completed"
        self._save_state(reason)
        append_control_event("capability_stop_requested", {"plan_id": self.plan_id, "reason": reason})

    def _mark_hit_unconfirmed(self, reason: str) -> None:
        """Disappearance cannot distinguish a burst, a cut string or an exit.

        End this chase attempt without marking any target complete. The operator
        can inspect the outcome, then Stop -> Prepare -> select the next plan.
        """
        self._stop_target_search_hook(reason)
        STOP_REQUEST_FILE.write_text(
            f"plan_id={self.plan_id}\nreason=capability_hit_unconfirmed\nts_ms={now_ms()}\n",
            encoding="utf-8")
        self.status = "awaiting_confirmation"
        self._save_state("hit_unconfirmed")
        append_control_event("capability_hit_unconfirmed", {
            "plan_id": self.plan_id, "index": self.current_index,
            "reason": reason, "physical_outcome": "unknown",
            "next_action": "stop_then_prepare_and_select_plan",
        })

    def _finish_strike_attempt(self, reason: str) -> None:
        """Keep the original fixed pink/yellow automatic sequence.

        Completing a sequence step is a disappearance heuristic, not proof
        that the aircraft physically burst the balloon.
        """
        pink_yellow = [
            {"capability": "strike_balloon", "parameters": {"color": color}}
            for color in ("pink", "yellow")
        ]
        if self.api_motion or self.actions != pink_yellow:
            self._mark_hit_unconfirmed(reason)
            return
        append_control_event("capability_step_completed_by_disappearance", {
            "plan_id": self.plan_id, "index": self.current_index,
            "reason": reason, "physical_outcome": "unknown",
            "next_action": "automatic_sequence",
        })
        self._advance()

    def _advance(self) -> None:
        self._stop_target_search_hook("target_completed")
        completed_index = self.current_index
        self.completed_indices.add(completed_index)
        self.hit_armed = False
        self.near_arm_count = 0
        self.missing_since_ms = None
        self.search_missing_since_ms = None
        self._reset_target_lock()
        remaining = [
            index for index in range(len(self.actions))
            if index not in self.completed_indices
        ]
        self.current_index = min(remaining) if remaining else len(self.actions)
        append_control_event(
            "capability_task_advanced",
            {"plan_id": self.plan_id, "completed_index": completed_index, "next_index": self.current_index},
        )
        if not remaining:
            self._request_stop("capability_plan_complete")
        else:
            self._save_state("advanced")

    def _defer_current(self) -> bool:
        remaining = [
            index for index in range(len(self.actions))
            if index not in self.completed_indices
        ]
        if len(remaining) <= 1:
            return False
        self._stop_target_search_hook("target_deferred")
        previous_index = self.current_index
        for offset in range(1, len(self.actions) + 1):
            candidate_index = (previous_index + offset) % len(self.actions)
            if candidate_index in self.completed_indices:
                continue
            self.current_index = candidate_index
            self.hit_armed = False
            self.near_arm_count = 0
            self.missing_since_ms = None
            self.search_missing_since_ms = None
            self._reset_target_lock()
            self._save_state("target_search_deferred")
            append_control_event(
                "capability_target_deferred",
                {
                    "plan_id": self.plan_id,
                    "deferred_index": previous_index,
                    "next_index": candidate_index,
                },
            )
            return True
        return False

    def _remember_detections(self, detections: list[dict[str, Any]]) -> None:
        current_ms = now_ms()
        cache_ms = max(
            250,
            int(os.environ.get("BALLOON_NL_COLOR_OBSERVATION_CACHE_MS", "1500")),
        )
        self.observed_color_cache = {
            color: observation
            for color, observation in self.observed_color_cache.items()
            if current_ms - int(observation.get("seen_ts_ms") or 0) <= cache_ms
        }
        for detection in detections:
            if "balloon" not in str(detection.get("class_name", "")).strip().lower():
                continue
            color = canonical_balloon_color(detection.get("color_name"))
            if color not in {"yellow", "pink", "blue"}:
                continue
            center = detection.get("center_norm")
            self.observed_color_cache[color] = {
                "seen_ts_ms": current_ms,
                "confidence": round(float(detection.get("confidence") or 0.0), 4),
                "color_confidence": round(
                    float(detection.get("color_confidence") or 0.0), 4
                ),
                "area_ratio": round(float(detection.get("area_ratio") or 0.0), 6),
                "center_norm": (
                    [round(float(value), 5) for value in center]
                    if isinstance(center, (list, tuple)) and len(center) == 2
                    else None
                ),
            }

    @staticmethod
    def _detection_center(detection: dict[str, Any]) -> tuple[float, float] | None:
        center = detection.get("center_norm")
        if not isinstance(center, (list, tuple)) or len(center) != 2:
            return None
        try:
            x, y = float(center[0]), float(center[1])
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return x, y

    @staticmethod
    def _center_distance(
        first: tuple[float, float], second: tuple[float, float]
    ) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _select_locked_candidate(
        self, candidates: list[dict[str, Any]], color: str
    ) -> dict[str, Any] | None:
        """Keep one physical balloon until a distant replacement is stable."""
        current_ms = now_ms()
        acquire_confidence = float(
            os.environ.get("BALLOON_TARGET_ACQUIRE_CONFIDENCE", "0.45")
        )
        continue_confidence = float(
            os.environ.get("BALLOON_TARGET_CONTINUE_CONFIDENCE", "0.25")
        )
        near_acquire_confidence = max(
            continue_confidence,
            float(
                os.environ.get(
                    "BALLOON_NEAR_ACQUIRE_CONFIDENCE", "0.28"
                )
            ),
        )
        max_jump = float(os.environ.get("BALLOON_TARGET_LOCK_MAX_JUMP_NORM", "0.12"))
        far_motion_max_jump = float(
            os.environ.get("BALLOON_FAR_MOTION_MAX_JUMP_NORM", "0.22")
        )
        far_motion_jump_rate = float(
            os.environ.get("BALLOON_FAR_MOTION_JUMP_RATE_NORM_PER_S", "0.30")
        )
        far_motion_area_ratio_min = float(
            os.environ.get("BALLOON_FAR_MOTION_AREA_RATIO_MIN", "0.55")
        )
        far_motion_area_ratio_max = float(
            os.environ.get("BALLOON_FAR_MOTION_AREA_RATIO_MAX", "1.80")
        )
        far_motion_max_age_ms = max(
            100,
            int(os.environ.get("BALLOON_FAR_MOTION_MAX_AGE_MS", "900")),
        )
        near_area_ratio = float(
            os.environ.get("BALLOON_NEAR_RECOVERY_MIN_AREA_RATIO", "0.006")
        )
        near_max_jump = float(
            os.environ.get("BALLOON_NEAR_RECOVERY_MAX_JUMP_NORM", "0.65")
        )
        near_latch_ms = max(
            500,
            int(os.environ.get("BALLOON_NEAR_RECOVERY_LATCH_MS", "1800")),
        )
        switch_cluster_radius = float(
            os.environ.get("BALLOON_TARGET_SWITCH_CLUSTER_RADIUS_NORM", "0.08")
        )
        switch_confirm_frames = max(
            2, int(os.environ.get("BALLOON_TARGET_SWITCH_CONFIRM_FRAMES", "3"))
        )
        lock_expire_ms = max(
            500, int(os.environ.get("BALLOON_TARGET_LOCK_EXPIRE_MS", "2200"))
        )
        near_lock_active = (
            self.locked_area_ratio >= near_area_ratio
            or (
                self.locked_near_until_ms is not None
                and current_ms <= self.locked_near_until_ms
            )
        )
        if near_lock_active:
            lock_expire_ms = max(
                lock_expire_ms,
                int(os.environ.get("BALLOON_NEAR_RECOVERY_LOCK_MS", "4500")),
            )

        candidates_with_centers = [
            (item, self._detection_center(item))
            for item in candidates
            if float(item.get("confidence") or 0.0) >= continue_confidence
        ]
        candidates_with_centers = [
            (item, center) for item, center in candidates_with_centers if center is not None
        ]

        if (
            self.locked_center_norm is not None
            and self.locked_last_seen_ms is not None
            and current_ms - self.locked_last_seen_ms > lock_expire_ms
        ):
            append_control_event(
                "capability_target_lock_expired",
                {"plan_id": self.plan_id, "index": self.current_index, "color": color},
            )
            self._reset_target_lock()

        if self.locked_center_norm is None:
            far_area_ratio = float(
                os.environ.get("BALLOON_FAR_CONFIRM_MAX_AREA_RATIO", "0.003")
            )
            far_acquire_confidence = float(
                os.environ.get("BALLOON_FAR_ACQUIRE_CONFIDENCE", "0.35")
            )
            eligible = [
                (item, center)
                for item, center in candidates_with_centers
                if float(item.get("confidence") or 0.0) >= (
                    far_acquire_confidence
                    if (
                        str(item.get("detection_pass") or "") == "far_tile"
                        or float(item.get("area_ratio") or 0.0) <= far_area_ratio
                    )
                    else near_acquire_confidence
                    if float(item.get("area_ratio") or 0.0) >= near_area_ratio
                    else acquire_confidence
                )
            ]
            if eligible:
                best, center = max(
                    eligible, key=lambda pair: float(pair[0].get("confidence") or 0.0)
                )
                is_far = (
                    str(best.get("detection_pass") or "") == "far_tile"
                    or float(best.get("area_ratio") or 0.0) <= far_area_ratio
                )
                if is_far:
                    color_confidence = float(best.get("color_confidence") or 0.0)
                    min_color_confidence = float(
                        os.environ.get("BALLOON_FAR_MIN_COLOR_CONFIDENCE", "0.55")
                    )
                    detected_color = str(best.get("color_name") or "")
                    detected_group = canonical_balloon_color(detected_color)
                    if (
                        detected_group not in {"yellow", "pink", "blue"}
                        or color_confidence < min_color_confidence
                    ):
                        self.pending_acquire_center_norm = None
                        self.pending_acquire_color = ""
                        self.pending_acquire_count = 0
                        self.pending_acquire_edge = False
                        return None
                    confirm_radius = float(
                        os.environ.get("BALLOON_FAR_CONFIRM_RADIUS_NORM", "0.08")
                    )
                    required = max(
                        2,
                        int(os.environ.get("BALLOON_FAR_CONFIRM_FRAMES", "2")),
                    )
                    edge_margin = min(
                        0.25,
                        max(
                            0.02,
                            float(
                                os.environ.get(
                                    "BALLOON_FAR_EDGE_CONFIRM_MARGIN_NORM",
                                    "0.10",
                                )
                            ),
                        ),
                    )
                    edge_candidate = (
                        center[0] <= edge_margin
                        or center[0] >= 1.0 - edge_margin
                        or center[1] <= edge_margin
                        or center[1] >= 1.0 - edge_margin
                    )
                    edge_sequence = self.pending_acquire_edge or edge_candidate
                    if edge_sequence:
                        confirm_radius = max(
                            confirm_radius,
                            float(
                                os.environ.get(
                                    "BALLOON_FAR_EDGE_CONFIRM_RADIUS_NORM",
                                    "0.14",
                                )
                            ),
                        )
                        required = max(
                            required,
                            int(
                                os.environ.get(
                                    "BALLOON_FAR_EDGE_CONFIRM_FRAMES", "3"
                                )
                            ),
                        )
                    if (
                        self.pending_acquire_center_norm is not None
                        and detected_group == self.pending_acquire_color
                        and self._center_distance(
                            center, self.pending_acquire_center_norm
                        ) <= confirm_radius
                    ):
                        previous_count = self.pending_acquire_count
                        count = previous_count + 1
                        old = self.pending_acquire_center_norm
                        self.pending_acquire_center_norm = (
                            (old[0] * previous_count + center[0]) / count,
                            (old[1] * previous_count + center[1]) / count,
                        )
                        self.pending_acquire_count = count
                        self.pending_acquire_edge = edge_sequence
                    else:
                        self.pending_acquire_center_norm = center
                        self.pending_acquire_color = detected_group
                        self.pending_acquire_count = 1
                        self.pending_acquire_edge = edge_candidate
                    if self.pending_acquire_count < required:
                        append_control_event(
                            "capability_far_target_acquire_pending",
                            {
                                "plan_id": self.plan_id,
                                "index": self.current_index,
                                "color": detected_color,
                                "color_group": detected_group,
                                "confirmations": self.pending_acquire_count,
                                "required": required,
                                "confidence": round(
                                    float(best.get("confidence") or 0.0), 4
                                ),
                                "color_confidence": round(color_confidence, 4),
                                "edge_sequence": self.pending_acquire_edge,
                            },
                        )
                        return None
                self.locked_center_norm = center
                self.locked_last_seen_ms = current_ms
                self.locked_area_ratio = float(best.get("area_ratio") or 0.0)
                if self.locked_area_ratio >= near_area_ratio:
                    self.locked_near_until_ms = current_ms + near_latch_ms
                self.pending_switch_center_norm = None
                self.pending_switch_count = 0
                self.pending_acquire_center_norm = None
                self.pending_acquire_color = ""
                self.pending_acquire_count = 0
                self.pending_acquire_edge = False
                append_control_event(
                    "capability_target_lock_acquired",
                    {
                        "plan_id": self.plan_id,
                        "index": self.current_index,
                        "color": color,
                        "center_norm": [round(center[0], 5), round(center[1], 5)],
                        "confidence": round(float(best.get("confidence") or 0.0), 4),
                    },
                )
                return best
            # Unit/integration callers may omit geometry. Preserve their
            # legacy behavior, while still enforcing acquisition confidence.
            geometry_free = [
                item
                for item in candidates
                if self._detection_center(item) is None
                and float(item.get("confidence") or 0.0) >= acquire_confidence
            ]
            if geometry_free:
                return max(
                    geometry_free,
                    key=lambda item: float(item.get("confidence") or 0.0),
                )
            self.pending_acquire_center_norm = None
            self.pending_acquire_color = ""
            self.pending_acquire_count = 0
            self.pending_acquire_edge = False
            return None

        lock_age_ms = (
            current_ms - self.locked_last_seen_ms
            if self.locked_last_seen_ms is not None
            and current_ms >= self.locked_last_seen_ms
            else 0
        )
        motion_jump_limit = min(
            max(far_motion_max_jump, max_jump),
            max_jump + far_motion_jump_rate * lock_age_ms / 1000.0,
        )

        def matches_motion_compensated_far_lock(
            item: dict[str, Any], center: tuple[float, float]
        ) -> bool:
            """Allow image motion during a fresh body/gimbal handoff.

            The far detector result can be 350-500 ms old. During that time a
            commanded yaw moves the same balloon farther than the static
            center gate. Restrict the wider gate to a fresh lock with a
            consistent angular area so another distant balloon is not accepted
            merely because it has the requested colour.
            """
            if near_lock_active or lock_age_ms > far_motion_max_age_ms:
                return False
            previous_area = float(self.locked_area_ratio or 0.0)
            candidate_area = float(item.get("area_ratio") or 0.0)
            if previous_area <= 0.0 or candidate_area <= 0.0:
                return False
            area_scale = candidate_area / previous_area
            return (
                far_motion_area_ratio_min <= area_scale <= far_motion_area_ratio_max
                and self._center_distance(center, self.locked_center_norm)
                <= motion_jump_limit
            )

        effective_max_jump = near_max_jump if near_lock_active else max_jump
        nearby = [
            (item, center)
            for item, center in candidates_with_centers
            if (
                self._center_distance(center, self.locked_center_norm)
                <= effective_max_jump
                or matches_motion_compensated_far_lock(item, center)
            )
        ]
        if nearby:
            best, center = min(
                nearby,
                key=lambda pair: (
                    self._center_distance(pair[1], self.locked_center_norm),
                    -float(pair[0].get("confidence") or 0.0),
                ),
            )
            matched_distance = self._center_distance(
                center, self.locked_center_norm
            )
            if near_lock_active and matched_distance > max_jump:
                append_control_event(
                    "capability_near_target_deadband_match",
                    {
                        "plan_id": self.plan_id,
                        "index": self.current_index,
                        "color": color,
                        "distance_norm": round(matched_distance, 5),
                        "far_limit_norm": round(max_jump, 5),
                        "near_limit_norm": round(near_max_jump, 5),
                    },
                )
            elif matched_distance > max_jump:
                append_control_event(
                    "capability_far_motion_match",
                    {
                        "plan_id": self.plan_id,
                        "index": self.current_index,
                        "color": color,
                        "distance_norm": round(matched_distance, 5),
                        "static_limit_norm": round(max_jump, 5),
                        "motion_limit_norm": round(motion_jump_limit, 5),
                        "lock_age_ms": lock_age_ms,
                    },
                )
            self.locked_center_norm = center
            self.locked_last_seen_ms = current_ms
            self.locked_area_ratio = float(best.get("area_ratio") or 0.0)
            if self.locked_area_ratio >= near_area_ratio:
                self.locked_near_until_ms = current_ms + near_latch_ms
            self.pending_switch_center_norm = None
            self.pending_switch_count = 0
            return best

        distant = [
            (item, center)
            for item, center in candidates_with_centers
            if float(item.get("confidence") or 0.0) >= acquire_confidence
        ]

        # At close range the balloon box is often clipped by one image edge.
        # A small gimbal/body correction can then make its reported centre jump
        # across most of the frame (the flight log contained 0.983 -> 0.012).
        # Preserve strict multi-frame switching at long range, but allow a
        # recent, large same-colour target to reattach to the near lock.
        if (
            (
                near_lock_active
                or any(
                    float(item.get("area_ratio") or 0.0) >= near_area_ratio
                    for item, _center in candidates_with_centers
                )
            )
            and self.locked_last_seen_ms is not None
            and current_ms - self.locked_last_seen_ms <= lock_expire_ms
        ):
            near_candidates = [
                (item, center)
                for item, center in candidates_with_centers
                if float(item.get("area_ratio") or 0.0) >= near_area_ratio
                and float(item.get("confidence") or 0.0)
                    >= near_acquire_confidence
                and (
                    self._center_distance(center, self.locked_center_norm)
                    <= near_max_jump
                    or (
                        abs(center[1] - self.locked_center_norm[1]) <= 0.25
                        and (
                            (
                                self.locked_center_norm[0] <= 0.12
                                and center[0] >= 0.88
                            )
                            or (
                                self.locked_center_norm[0] >= 0.88
                                and center[0] <= 0.12
                            )
                        )
                    )
                )
            ]
            if near_candidates:
                best, center = max(
                    near_candidates,
                    key=lambda pair: float(pair[0].get("confidence") or 0.0),
                )
                previous_center = self.locked_center_norm
                self.locked_center_norm = center
                self.locked_last_seen_ms = current_ms
                self.locked_area_ratio = float(best.get("area_ratio") or 0.0)
                if self.locked_area_ratio >= near_area_ratio:
                    self.locked_near_until_ms = current_ms + near_latch_ms
                self.pending_switch_center_norm = None
                self.pending_switch_count = 0
                append_control_event(
                    "capability_near_target_reacquired",
                    {
                        "plan_id": self.plan_id,
                        "index": self.current_index,
                        "color": color,
                        "previous_center_norm": [
                            round(previous_center[0], 5),
                            round(previous_center[1], 5),
                        ],
                        "center_norm": [round(center[0], 5), round(center[1], 5)],
                        "area_ratio": round(self.locked_area_ratio, 6),
                    },
                )
                return best

        if not distant:
            self.pending_switch_center_norm = None
            self.pending_switch_count = 0
            return None

        best, center = max(
            distant, key=lambda pair: float(pair[0].get("confidence") or 0.0)
        )
        if (
            self.pending_switch_center_norm is not None
            and self._center_distance(center, self.pending_switch_center_norm)
            <= switch_cluster_radius
        ):
            self.pending_switch_count += 1
            count = self.pending_switch_count
            old = self.pending_switch_center_norm
            self.pending_switch_center_norm = (
                (old[0] * (count - 1) + center[0]) / count,
                (old[1] * (count - 1) + center[1]) / count,
            )
        else:
            self.pending_switch_center_norm = center
            self.pending_switch_count = 1

        if self.pending_switch_count < switch_confirm_frames:
            append_control_event(
                "capability_target_jump_pending",
                {
                    "plan_id": self.plan_id,
                    "index": self.current_index,
                    "color": color,
                    "confirmations": self.pending_switch_count,
                    "required": switch_confirm_frames,
                    "distance_norm": round(
                        self._center_distance(center, self.locked_center_norm), 5
                    ),
                },
            )
            return None

        self.locked_center_norm = center
        self.locked_last_seen_ms = current_ms
        self.locked_area_ratio = float(best.get("area_ratio") or 0.0)
        if self.locked_area_ratio >= near_area_ratio:
            self.locked_near_until_ms = current_ms + near_latch_ms
        self.pending_switch_center_norm = None
        self.pending_switch_count = 0
        append_control_event(
            "capability_target_switch_confirmed",
            {
                "plan_id": self.plan_id,
                "index": self.current_index,
                "color": color,
                "center_norm": [round(center[0], 5), round(center[1], 5)],
                "confidence": round(float(best.get("confidence") or 0.0), 4),
            },
        )
        return best

    def select(self, detections: list[dict[str, Any]], *,
               expected_visual_context: dict | None = None) -> dict[str, Any] | None:
        gate = getattr(self, "mission_gate", None)
        if gate is not None and not gate.poll(self):
            return None
        if expected_visual_context is not None and (
                gate is None or gate.visual_context != expected_visual_context):
            return None
        if gate is None:
            self._load_plan()
        self._remember_detections(detections)
        if not self.actions:
            # The original balloon-strike test now uses the capability library
            # too: no explicit plan means strike_balloon(color="any").
            color = DEFAULT_TEST_ACTION["parameters"]["color"]
            candidates = [item for item in detections if balloon_matches(item, color)]
            best = self._select_locked_candidate(candidates, color)
            if best is not None:
                self._publish_near_rate_hint(best)
            return best
        if self.status in {"completed", "awaiting_confirmation"}:
            return None
        if self.current_index >= len(self.actions):
            self._request_stop("capability_plan_complete")
            return None
        action = self.actions[self.current_index]
        capability = action["capability"]
        if capability == "stop":
            self._request_stop("capability_stop_action")
            return None

        if capability != "strike_balloon":
            self._request_stop("capability_executor_missing")
            return None

        color = action["parameters"]["color"]
        candidates = [item for item in detections if balloon_matches(item, color)]
        had_recent_lock = self.locked_last_seen_ms is not None
        best = self._select_locked_candidate(candidates, color)
        if best is not None:
            self._remember_target_exit_geometry(best)
            self.edge_hit_recovery_pending_logged = False
            self._publish_near_rate_hint(best)
            self._stop_target_search_hook("target_reacquired")
            self.search_missing_since_ms = None
            self.missing_since_ms = None
            area_ratio = float(best.get("area_ratio") or 0.0)
            arm_ratio = float(
                os.environ.get("BALLOON_NL_HIT_ARM_AREA_RATIO", "0.015")
            )
            arm_confirm_frames = max(
                1, int(os.environ.get("BALLOON_NL_HIT_ARM_CONFIRM_FRAMES", "1"))
            )
            # An object first discovered as a large crop at the image edge can
            # be a balloon fragment, pole, banner or another red/orange patch.
            # Its clipped box also exaggerates apparent proximity.  Preserve
            # tracking, but do not arm a strike until this lock has either
            # appeared away from the edge or has visibly grown from a smaller
            # box during a real approach.
            if not self.last_target_seen_at_edge:
                self.hit_approach_evidence = True
            if (
                self.hit_min_area_ratio is not None
                and self.hit_min_area_ratio <= arm_ratio * 0.65
                and area_ratio >= arm_ratio
            ):
                self.hit_approach_evidence = True
            self.hit_min_area_ratio = (
                area_ratio
                if self.hit_min_area_ratio is None
                else min(self.hit_min_area_ratio, area_ratio)
            )

            if area_ratio >= arm_ratio and self.hit_approach_evidence:
                self.near_arm_count += 1
            else:
                # One noisy large box cannot arm a strike.  Decay instead of
                # dropping directly to zero so a single detector-size wobble
                # near the threshold does not erase genuine approach progress.
                self.near_arm_count = max(0, self.near_arm_count - 1)
                if (
                    area_ratio >= arm_ratio
                    and self.last_target_seen_at_edge
                    and not self.hit_approach_evidence
                    and not self.edge_arm_rejection_logged
                ):
                    self.edge_arm_rejection_logged = True
                    append_control_event(
                        "capability_edge_candidate_arm_rejected_no_approach",
                        {
                            "plan_id": self.plan_id,
                            "index": self.current_index,
                            "color": color,
                            "area_ratio": area_ratio,
                            "arm_ratio": arm_ratio,
                            "center_norm": (
                                [
                                    round(self.last_target_center_norm[0], 5),
                                    round(self.last_target_center_norm[1], 5),
                                ]
                                if self.last_target_center_norm is not None
                                else None
                            ),
                        },
                    )
            if self.near_arm_count >= arm_confirm_frames and not self.hit_armed:
                self.hit_armed = True
                self._save_state("hit_armed")
                append_control_event(
                    "capability_strike_armed",
                    {"plan_id": self.plan_id, "index": self.current_index,
                     "color": color, "area_ratio": area_ratio,
                     "confirmations": self.near_arm_count},
                )
            return best

        current_ms = now_ms()
        if (
            detections
            and (
                self.last_candidate_rejection_diag_ms == 0
                or current_ms - self.last_candidate_rejection_diag_ms >= 1000
            )
        ):
            self.last_candidate_rejection_diag_ms = current_ms
            append_control_event(
                "capability_candidates_rejected_for_requested_color",
                {
                    "plan_id": self.plan_id,
                    "index": self.current_index,
                    "requested_color": color,
                    "requested_color_group": canonical_balloon_color(color),
                    "candidates": [
                        {
                            "class_name": str(item.get("class_name") or ""),
                            "confidence": round(
                                float(item.get("confidence") or 0.0), 4
                            ),
                            "color_name": str(item.get("color_name") or ""),
                            "color_group": canonical_balloon_color(
                                item.get("color_name")
                            ),
                            "color_confidence": round(
                                float(item.get("color_confidence") or 0.0), 4
                            ),
                            "color_hsv_median": item.get("color_hsv_median"),
                            "area_ratio": round(
                                float(item.get("area_ratio") or 0.0), 6
                            ),
                        }
                        for item in sorted(
                            detections,
                            key=lambda candidate: float(
                                candidate.get("confidence") or 0.0
                            ),
                            reverse=True,
                        )[:3]
                    ],
                },
            )

        if self.hit_armed:
            current_ms = now_ms()
            if self.missing_since_ms is None:
                self.missing_since_ms = current_ms
                self._save_state("target_missing_after_near")
            missing_confirm_ms = int(os.environ.get("BALLOON_NL_HIT_MISSING_CONFIRM_MS", "1000"))
            if current_ms - self.missing_since_ms >= max(500, missing_confirm_ms):
                if self.last_target_seen_at_edge:
                    edge_recovery_confirm_ms = max(
                        max(500, missing_confirm_ms),
                        int(
                            os.environ.get(
                                "BALLOON_EDGE_HIT_RECOVERY_CONFIRM_MS", "3200"
                            )
                        ),
                    )
                    if not self.edge_hit_recovery_pending_logged:
                        self.edge_hit_recovery_pending_logged = True
                        append_control_event(
                            "capability_edge_exit_recovery_pending",
                            {
                                "plan_id": self.plan_id,
                                "index": self.current_index,
                                "color": color,
                                "last_center_norm": (
                                    [
                                        round(self.last_target_center_norm[0], 5),
                                        round(self.last_target_center_norm[1], 5),
                                    ]
                                    if self.last_target_center_norm is not None
                                    else None
                                ),
                                "confirm_after_ms": edge_recovery_confirm_ms,
                            },
                        )
                    if (
                        current_ms - self.missing_since_ms
                        >= edge_recovery_confirm_ms
                    ):
                        self._finish_strike_attempt("edge_exit_after_near")
                else:
                    self._finish_strike_attempt("missing_after_near")
        else:
            self.near_arm_count = max(0, self.near_arm_count - 1)
            if self.api_motion:
                # API control only observes the current action. An empty
                # detection never requests a local search or a new target.
                return None
            current_ms = now_ms()
            if self.search_missing_since_ms is None:
                self.search_missing_since_ms = current_ms
                self._save_state("target_search_waiting")
            near_lock_active = (
                self.locked_area_ratio >= float(
                    os.environ.get(
                        "BALLOON_NEAR_RECOVERY_MIN_AREA_RATIO", "0.006"
                    )
                )
                or (
                    self.locked_near_until_ms is not None
                    and current_ms <= self.locked_near_until_ms
                )
            )
            near_recovery_grace_ms = (
                int(os.environ.get("BALLOON_NEAR_RECOVERY_GRACE_MS", "3100"))
                if near_lock_active
                else 0
            )
            reacquire_grace_ms = (
                int(os.environ.get("BALLOON_SEARCH_START_GRACE_MS", "900"))
                if had_recent_lock
                else 0
            )
            search_start_grace_ms = max(
                0, near_recovery_grace_ms, reacquire_grace_ms
            )
            if (
                not self.target_search_hook_active
                and current_ms - self.search_missing_since_ms
                >= search_start_grace_ms
            ):
                self._start_target_search_hook(color)
            allow_defer = os.environ.get(
                "BALLOON_NL_ALLOW_TARGET_DEFER", "0"
            ).strip().lower() in {"1", "true", "yes", "on"}
            if allow_defer:
                search_timeout_ms = int(
                    os.environ.get("BALLOON_NL_TARGET_SEARCH_TIMEOUT_MS", "90000")
                )
                if current_ms - self.search_missing_since_ms >= max(1000, search_timeout_ms):
                    if self._defer_current():
                        return self.select(detections)
        return None


TASK_SEQUENCE = CapabilityTaskSequence()


def far_tile_regions(width: int, height: int) -> list[tuple[int, int, int, int]]:
    """Return six overlapping crops for higher-resolution far acquisition.

    A 48%-wide crop gives a 1344px liveview about 2.1x more target pixels at
    the fixed 640px TensorRT input than the full-frame pass.  Three horizontal
    positions and two vertical positions retain substantial seam overlap.
    This path runs only before acquisition (or after the lock grace expires),
    so the extra two tiles do not add latency to normal near-range tracking.
    """
    if width < 4 or height < 4:
        return []
    tile_width = min(width, max(320, int(math.ceil(width * 0.48))))
    tile_height = min(height, max(320, int(math.ceil(height * 0.68))))
    max_x = max(0, width - tile_width)
    x_positions = sorted({0, int(round(max_x * 0.5)), max_x})
    y_positions = sorted({0, max(0, height - tile_height)})
    return [
        (x, y, min(width, x + tile_width), min(height, y + tile_height))
        for y in y_positions
        for x in x_positions
    ]


def detection_iou(first: dict[str, Any], second: dict[str, Any]) -> float:
    a = [float(value) for value in (first.get("bbox_xyxy") or [])]
    b = [float(value) for value in (second.get("bbox_xyxy") or [])]
    if len(a) != 4 or len(b) != 4:
        return 0.0
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return intersection / max(1e-9, area_a + area_b - intersection)


def merge_detections(
    detections: list[dict[str, Any]], iou_threshold: float = 0.50
) -> list[dict[str, Any]]:
    kept: list[dict[str, Any]] = []
    for item in sorted(
        detections,
        key=lambda candidate: float(candidate.get("confidence") or 0.0),
        reverse=True,
    ):
        if any(
            str(existing.get("class_name")) == str(item.get("class_name"))
            and detection_iou(existing, item) >= iou_threshold
            for existing in kept
        ):
            continue
        kept.append(item)
    return kept


def predict_far_tiles(
    source: Path, detector: Any, width: int, height: int, hardcase: Any = None
) -> tuple[list[dict[str, Any]], float]:
    from PIL import Image

    regions = far_tile_regions(width, height)
    if not regions:
        return [], 0.0
    FAR_TILE_DIR.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    records: list[dict[str, Any]] = []
    created: list[Path] = []
    try:
        with Image.open(source) as image:
            for index, (x1, y1, x2, y2) in enumerate(regions):
                tile_path = FAR_TILE_DIR / f"tile_{os.getpid()}_{index}.jpg"
                image.crop((x1, y1, x2, y2)).save(
                    tile_path, format="JPEG", quality=92
                )
                created.append(tile_path)
                tile_records = detector.predict_records(tile_path)
                if hardcase is not None:
                    with contextlib.suppress(Exception):
                        hardcase.observe(tile_records, "tile", detector, (x1, y1, x2, y2))
                for record in tile_records:
                    mapped = dict(record)
                    bbox = [float(value) for value in (record.get("bbox_xyxy") or [])]
                    if len(bbox) != 4:
                        continue
                    mapped["bbox_xyxy"] = [
                        bbox[0] + x1,
                        bbox[1] + y1,
                        bbox[2] + x1,
                        bbox[3] + y1,
                    ]
                    mapped["detection_pass"] = "far_tile"
                    mapped["tile_index"] = index
                    mapped["tile_region_xyxy"] = [x1, y1, x2, y2]
                    records.append(mapped)
    finally:
        for tile_path in created:
            with contextlib.suppress(OSError):
                tile_path.unlink()
    return records, round((time.perf_counter() - started) * 1000, 3)


@dataclass(frozen=True)
class DetectionRequest:
    seq: int
    capture_ts_ms: int
    request_ts_ms: int
    source: Path

    @property
    def key(self) -> tuple[int, int, str]:
        return self.seq, self.capture_ts_ms, str(self.source)


def parse_request(path: Path) -> DetectionRequest | None:
    try:
        line = path.read_text(encoding="utf-8").strip()
    except FileNotFoundError:
        return None
    except Exception as exc:
        append_control_event("request_read_error", {"error": repr(exc)})
        return None

    if not line:
        return None

    parts = split_record(line, 4)
    if parts is None:
        return None
    seq_raw, capture_raw, request_raw, source_raw = parts

    try:
        stamp, requested, current = int(capture_raw), int(request_raw), now_ms()
        if not 0 < stamp <= requested <= current or current - requested > 1500:
            return None
        return DetectionRequest(
            seq=int(seq_raw),
            capture_ts_ms=stamp,
            request_ts_ms=requested,
            source=Path(source_raw).resolve(),
        )
    except Exception as exc:
        append_control_event("request_parse_error", {"line": line, "error": repr(exc)})
        return None


def ensure_source_ready(path: Path, wait_ms: int = 1500) -> bool:
    deadline = time.monotonic() + (wait_ms / 1000.0)
    while time.monotonic() <= deadline:
        if path.is_file() and path.stat().st_size > 0:
            return True
        time.sleep(0.05)
    return False


class PyTorchDetector:
    backend_name = "pytorch"

    def __init__(self, model: Any, yolo_infer: Any, config: dict[str, Any]) -> None:
        self.model = model
        self.yolo_infer = yolo_infer
        self.config = config

    def predict_records(self, source: Path) -> list[dict[str, Any]]:
        with contextlib.redirect_stdout(io.StringIO()):
            results = self.model.predict(
                source=str(source),
                imgsz=self.config["imgsz"],
                conf=self.config["conf"],
                device=self.config["device"],
                save=self.config["save"],
                project=str(YOLO_MODULE / "runs"),
                name="latest_photo_worker",
                exist_ok=True,
                verbose=False,
            )
        return [record for result in results for record in self.yolo_infer.result_to_records(result)]


class TensorRTWithFallbackDetector:
    backend_name = "tensorrt"

    def __init__(self, primary: Any, fallback_factory: Any) -> None:
        self.primary = primary
        self.fallback_factory = fallback_factory
        self.fallback: Any = None

    def predict_records(self, source: Path) -> list[dict[str, Any]]:
        if self.fallback is not None:
            return self.fallback.predict_records(source)
        try:
            return self.primary.predict_records(source)
        except Exception as exc:
            append_control_event(
                "worker_backend_runtime_fallback",
                {"source_photo": str(source), "error": repr(exc)},
            )
            with contextlib.suppress(Exception):
                self.primary.close()
            self.fallback = self.fallback_factory()
            self.backend_name = "pytorch_fallback"
            return self.fallback.predict_records(source)


def load_yolo_detector() -> tuple[Any, dict[str, Any], Any]:
    sys.path.insert(0, str(YOLO_MODULE))
    import yolo_infer  # type: ignore

    yolo_infer.add_local_ultralytics_to_path()
    from ultralytics import YOLO  # type: ignore

    config = yolo_infer.load_config()
    weights = config["weights"].resolve()
    requested_backend = os.environ.get("BALLOON_YOLO_BACKEND", "pytorch").strip().lower()
    engine = Path(os.environ.get("BALLOON_TRT_ENGINE", str(DEFAULT_TRT_ENGINE))).resolve()
    decode_divisor = int(os.environ.get("BALLOON_JPEG_DECODE_DIVISOR", "4"))
    if decode_divisor not in (1, 2, 4, 8):
        raise ValueError("BALLOON_JPEG_DECODE_DIVISOR must be one of 1, 2, 4, 8")
    started = time.perf_counter()
    detector: Any = None
    fallback_reason: str | None = None
    if requested_backend == "tensorrt":
        try:
            from balloon_tensorrt import TensorRTBalloonDetector

            if not engine.is_file():
                raise FileNotFoundError(f"TensorRT engine not found: {engine}")
            primary = TensorRTBalloonDetector(
                engine,
                int(config["imgsz"]),
                float(config["conf"]),
                max_decode_divisor=decode_divisor,
            )
            detector = TensorRTWithFallbackDetector(
                primary,
                lambda: PyTorchDetector(YOLO(str(weights)), yolo_infer, config),
            )
        except Exception as exc:
            fallback_reason = repr(exc)
            append_control_event(
                "worker_backend_fallback",
                {"requested_backend": requested_backend, "engine": str(engine), "error": fallback_reason},
            )
    elif requested_backend != "pytorch":
        fallback_reason = f"unsupported backend: {requested_backend}"
        append_control_event(
            "worker_backend_fallback",
            {"requested_backend": requested_backend, "error": fallback_reason},
        )

    if detector is None:
        model = YOLO(str(weights))
        detector = PyTorchDetector(model, yolo_infer, config)
    append_control_event(
        "worker_model_loaded",
        {
            "weights": str(weights),
            "backend": detector.backend_name,
            "requested_backend": requested_backend,
            "engine": str(engine) if requested_backend == "tensorrt" else None,
            "jpeg_decode_divisor": decode_divisor if requested_backend == "tensorrt" else None,
            "fallback_reason": fallback_reason,
            "imgsz": config["imgsz"],
            "conf": config["conf"],
            "device": config["device"],
            "load_ms": round((time.perf_counter() - started) * 1000, 3),
        },
    )
    return detector, config, weights


def load_detector() -> tuple[Any, dict[str, Any], Any]:
    detector, config, weights = select_detector(yolo_factory=load_yolo_detector)
    append_control_event("worker_detector_selected", detector_metadata(detector, config))
    return detector, config, weights


def warmup(detector: Any, config: dict[str, Any]) -> None:
    started = time.perf_counter()
    try:
        from PIL import Image

        if not WARMUP_IMAGE.exists():
            Image.new("RGB", (640, 640), color=(0, 0, 0)).save(WARMUP_IMAGE, quality=80)
        detector.predict_records(WARMUP_IMAGE)
        append_control_event(
            "worker_warmup_done",
            {"backend": detector.backend_name, "latency_ms": round((time.perf_counter() - started) * 1000, 3)},
        )
    except Exception as exc:
        append_control_event("worker_warmup_error", {"error": repr(exc)})
        raise


def write_latest_target(payload: dict[str, Any], best: dict[str, Any] | None,
                        width: int | None, height: int | None) -> None:
    if not best or not width or not height:
        append_control_event(
            "target_not_written",
            {"seq": payload.get("seq"), "reason": "no_detection", "source_photo": payload.get("source")},
        )
        return

    bbox = best.get("bbox_xyxy") or []
    if len(bbox) != 4:
        append_control_event(
            "target_not_written",
            {"seq": payload.get("seq"), "reason": "invalid_bbox", "source_photo": payload.get("source")},
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
    # The YOLO-only file is the correction/input channel for the high-rate
    # tracker. Keep writing latest_target.tsv as a direct fallback so the
    # existing control loop remains usable if the tracker service is absent.
    for target_path in (LATEST_YOLO_TARGET_FILE, LATEST_TARGET_FILE):
        tmp_path = target_path.with_suffix(target_path.suffix + ".tmp")
        tmp_path.write_text(line, encoding="utf-8")
        tmp_path.replace(target_path)
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


def process_request(req: DetectionRequest, detector: Any, config: dict[str, Any],
                    log_path: Path, target_label: str,
                    hardcase: WorkerProducer | None = None) -> None:
    visual_context = visual_receipt.snapshot(TASK_SEQUENCE)
    capture = None
    if hardcase is not None:
        with contextlib.suppress(Exception):
            capture = hardcase.begin(req.source, req.seq)
    input_source = capture.path if capture is not None else req.source
    started = time.perf_counter()
    detection_started_ts_ms = now_ms()
    target_timestamp_ms = req.capture_ts_ms if req.capture_ts_ms > 0 else detection_started_ts_ms
    payload: dict[str, Any] = {
        "ok": False,
        "worker_persistent": True,
        "worker_session_id": WORKER_SESSION_ID,
        "ts": int(time.time()),
        "ts_ms": detection_started_ts_ms, **log_clock(),
        "detection_started_ts_ms": detection_started_ts_ms,
        "capture_ts_ms": req.capture_ts_ms if req.capture_ts_ms > 0 else None,
        "request_ts_ms": req.request_ts_ms if req.request_ts_ms > 0 else None,
        "target_timestamp_ms": target_timestamp_ms,
        "seq": req.seq,
        "source": str(req.source),
        "target_label": target_label,
    }

    def record_latency(stage: str, duration_ms: float, **extra: Any) -> None:
        append_latency(stage, req.seq, duration_ms,
                       detection_started_ts_ms=detection_started_ts_ms,
                       capture_ts_ms=req.capture_ts_ms, request_ts_ms=req.request_ts_ms,
                       **extra)

    try:
        if req.request_ts_ms > 0:
            record_latency("request_queue_wait", detection_started_ts_ms - req.request_ts_ms)
        if req.capture_ts_ms > 0:
            record_latency("capture_to_detection_start", detection_started_ts_ms - req.capture_ts_ms)
        if not ensure_source_ready(input_source):
            raise FileNotFoundError(f"source not ready: {req.source}")

        import detect_latest_photo  # type: ignore

        detect_source, enhancement_info = detect_latest_photo.prepare_detection_image(input_source)
        width, height = detect_latest_photo.image_size(detect_source)
        if capture is not None:
            capture.width, capture.height = width, height
            capture.inference_start_ns = time.monotonic_ns()
        predict_started = time.perf_counter()
        detections = detector.predict_records(detect_source)
        if capture is not None:
            with contextlib.suppress(Exception):
                capture.observe(detections, "full", detector)
        latency_ms = round((time.perf_counter() - predict_started) * 1000, 3)
        primary_detector = getattr(detector, "primary", detector)
        software_auto_exposure = getattr(primary_detector, "last_auto_exposure_info", None)
        record_latency("pixel_predict" if config["active_detector"] == "pixel" else "yolo_predict",
                       latency_ms, detection_count=len(detections), backend=detector.backend_name)
        enriched = [detect_latest_photo.enrich_detection(item, width, height) for item in detections]
        if not getattr(detector, "provides_colors", False):
            classify_balloon_colors(detect_source, enriched)
        far_tile_latency_ms = 0.0
        far_tile_detection_count = 0
        if (
            getattr(detector, "supports_far_tiles", True)
            and width is not None
            and height is not None
            and TASK_SEQUENCE.should_use_far_tiles(enriched)
        ):
            tile_records, far_tile_latency_ms = predict_far_tiles(
                detect_source, detector, width, height, hardcase=capture
            )
            tiled_enriched = [
                detect_latest_photo.enrich_detection(item, width, height)
                for item in tile_records
            ]
            classify_balloon_colors(detect_source, tiled_enriched)
            far_tile_detection_count = len(tiled_enriched)
            enriched = merge_detections(enriched + tiled_enriched)
            record_latency(
                "yolo_far_tiles",
                far_tile_latency_ms,
                detection_count=far_tile_detection_count,
                tile_count=len(far_tile_regions(width, height)),
            )
        if capture is not None:
            capture.inference_end_ns = time.monotonic_ns()
        label = target_label.lower()
        candidates = [item for item in enriched if label in str(item.get("class_name", "")).lower()]
        if not candidates:
            candidates = enriched
        observation_age_ms = max(0, now_ms() - target_timestamp_ms)
        for item in candidates:
            item["observation_age_ms"] = observation_age_ms
        frame_evidence = dict(payload, detection_finished_ts_ms=now_ms())
        frame_current = visual_receipt.frame_is_current(
            TASK_SEQUENCE, visual_context, frame_evidence, width, height, config['active_detector'])
        if getattr(TASK_SEQUENCE, 'mission_gate', None) is None:
            best_balloon = TASK_SEQUENCE.select(candidates)
        elif frame_current:
            best_balloon = TASK_SEQUENCE.select(candidates, expected_visual_context=visual_context)
        else:
            # select([]) also advances missing/hit state; a frame from before
            # this action must not influence either tracking or task outcomes.
            best_balloon = None
        detection_finished_ts_ms = now_ms()
        record_latency("detection_total", detection_finished_ts_ms - detection_started_ts_ms)
        if req.capture_ts_ms > 0:
            record_latency("capture_to_detection_finish",
                           detection_finished_ts_ms - req.capture_ts_ms)
        target_chase_plan = detect_latest_photo.build_target_chase_plan(
            best_balloon,
            width,
            height,
            target_timestamp_ms,
            detection_finished_ts_ms,
        )

        payload.update(
            {
                "ok": True,
                **detector_metadata(detector, config),
                "latency_ms": latency_ms,
                "far_tile_latency_ms": far_tile_latency_ms,
                "far_tile_detection_count": far_tile_detection_count,
                "detection_count": len(enriched),
                "detection_finished_ts_ms": detection_finished_ts_ms,
                "target_age_ms": max(0, detection_finished_ts_ms - target_timestamp_ms),
                "request_to_detection_finished_ms": (
                    max(0, detection_finished_ts_ms - req.request_ts_ms) if req.request_ts_ms > 0 else None
                ),
                "capture_to_request_ms": (
                    max(0, req.request_ts_ms - req.capture_ts_ms)
                    if req.request_ts_ms > 0 and req.capture_ts_ms > 0 else None
                ),
                "detect_source": str(detect_source),
                "enhancement": enhancement_info,
                "software_auto_exposure": software_auto_exposure,
                "image_size": [width, height],
                "detections": enriched,
                "balloon_candidates": candidates,
                "best_balloon": best_balloon,
                "target_chase_plan": target_chase_plan,
            }
        )
        if capture is not None:
            restore_source_references(payload, capture.path, req.source)
        visual_receipt.publish(TASK_SEQUENCE, visual_context, payload, best_balloon,
                               width, height, config['active_detector'])
        write_latest_target(payload, best_balloon, width, height)
    except Exception as exc:
        payload["error"] = repr(exc)
        append_control_event("worker_detection_error", {"seq": req.seq, "error": repr(exc), "source_photo": str(req.source)})
    finally:
        payload.setdefault("detection_finished_ts_ms", now_ms())
        payload.setdefault("target_age_ms", max(0, payload["detection_finished_ts_ms"] - target_timestamp_ms))
        payload["wrapper_latency_ms"] = round((time.perf_counter() - started) * 1000, 3)
        try:
            append_jsonl(log_path, payload)
            if capture is not None:
                with contextlib.suppress(Exception):
                    capture.finish(payload, detector)
        finally:
            if capture is not None:
                with contextlib.suppress(Exception):
                    capture.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Persistent configurable detector for PSDK balloon photos.")
    parser.add_argument("--request-file", type=Path, default=REQUEST_FILE)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--target-label", default="balloon")
    parser.add_argument("--poll-ms", type=int, default=100)
    parser.add_argument("--no-warmup", action="store_true")
    capture = parser.add_mutually_exclusive_group()
    capture.add_argument("--hardcase-enable", dest="hardcase_enable", action="store_true")
    capture.add_argument("--no-hardcase", dest="hardcase_enable", action="store_false")
    parser.set_defaults(hardcase_enable=True)
    parser.add_argument("--hardcase-ipc-dir", type=Path, default=DEFAULT_IPC_DIR)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    # A second worker must not publish to the same request/target channels.
    import fcntl
    worker_lock = (ROOT / "detector_worker.lock").open("a")
    fcntl.flock(worker_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    stop = False

    def handle_stop(signum: int, frame: Any) -> None:
        nonlocal stop
        stop = True
        append_control_event("worker_stop_signal", {"signal": signum})

    signal.signal(signal.SIGTERM, handle_stop)
    signal.signal(signal.SIGINT, handle_stop)

    append_control_event("worker_starting", {"request_file": str(args.request_file)})
    detector, config, _weights = load_detector()
    hardcase = None
    if args.hardcase_enable:
        try:
            hardcase = WorkerProducer(True, args.hardcase_ipc_dir, config, detector)
        except Exception as exc:
            append_control_event("hardcase_initialization_unavailable", {"error": repr(exc)})
    if not args.no_warmup:
        warmup(detector, config)
    neural_imports = sorted(name for name in ("torch", "ultralytics", "tensorrt", "yolo_infer", "balloon_tensorrt")
                            if name in sys.modules)
    if config["active_detector"] == "pixel" and neural_imports:
        raise RuntimeError(f"Pixel mode unexpectedly loaded neural modules: {neural_imports}")
    atomic_write_json(ROOT / "detector_runtime.json", {
        **detector_metadata(detector, config), "pid": os.getpid(),
        "clock_id": clock_id(), "ready_ts_ms": now_ms(),
        "neural_modules_loaded": neural_imports,
        "selector_config": config["selector_settings"],
    })
    append_control_event("worker_ready", {"poll_ms": args.poll_ms, "backend": detector.backend_name})

    from balloon_mission import MissionGate
    TASK_SEQUENCE.mission_gate = MissionGate(active_detector=config["active_detector"])
    last_key: tuple[int, int, str] | None = None
    while not stop:
        # Acknowledge the exact plan even before the camera is started.
        TASK_SEQUENCE.mission_gate.poll(TASK_SEQUENCE)
        req = parse_request(args.request_file)
        if req is not None and req.key != last_key:
            append_control_event(
                "worker_request_seen",
                {
                    "seq": req.seq,
                    "capture_ts_ms": req.capture_ts_ms,
                    "request_ts_ms": req.request_ts_ms,
                    "source_photo": str(req.source),
                },
            )
            process_request(req, detector, config, args.log, args.target_label, hardcase=hardcase)
            last_key = req.key
        time.sleep(max(10, args.poll_ms) / 1000.0)

    append_control_event("worker_stopped")
    if hardcase is not None:
        hardcase.close()
    return 0


if __name__ == "__main__":
    from detector_selector import ensure_script_interpreter
    ensure_script_interpreter()
    raise SystemExit(main())
