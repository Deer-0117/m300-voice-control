#!/usr/bin/env python3
"""Paired, offline comparison of pixel and camera-motion-compensated prediction.

Ground truth comes from manual boxes. Sampled boxes stand in for detector output,
so this measures the prediction step alone, not YOLO's detection quality.
"""

from __future__ import annotations

import argparse
import csv
import json
import io
import math
from bisect import bisect_left
from pathlib import Path


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def read_rows(path: Path) -> list[dict[str, str]]:
    text = path.read_text(encoding="utf-8-sig").rstrip("\x00")
    if "\x00" in text:
        raise ValueError(f"embedded NUL in {path}; inspect recording corruption")
    return list(csv.DictReader(io.StringIO(text)))


def annotate(video: Path, output: Path) -> None:
    import cv2

    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {video}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["video_frame", "x1", "y1", "x2", "y2"])
        index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            roi = cv2.selectROI("Balloon: draw box; Enter=save; Esc=not visible", frame,
                                showCrosshair=False, fromCenter=False)
            x, y, w, h = (int(v) for v in roi)
            if w > 0 and h > 0:
                writer.writerow([index, x, y, x + w, y + h])
            index += 1
    cap.release()
    cv2.destroyAllWindows()


def pose_at(poses: list[tuple[int, float, float]], timestamp: int,
            max_age_ms: int) -> tuple[float, float] | None:
    if not poses:
        return None
    times = [p[0] for p in poses]
    index = bisect_left(times, timestamp)
    choices = poses[max(0, index - 1):min(len(poses), index + 1)]
    best = min(choices, key=lambda p: abs(p[0] - timestamp))
    if abs(best[0] - timestamp) > max_age_ms:
        return None
    return best[1], best[2]


class AlphaBeta:
    def __init__(self, alpha: float, beta: float, rate_limit: float,
                 reset_after_gap: bool = False) -> None:
        self.alpha, self.beta, self.rate_limit = alpha, beta, rate_limit
        self.reset_after_gap = reset_after_gap
        self.value: float | None = None
        self.rate = 0.0
        self.timestamp = 0

    def update(self, value: float, timestamp: int, angular: bool = False) -> None:
        if self.value is None or (self.reset_after_gap and timestamp > self.timestamp + 500):
            self.value, self.rate, self.timestamp = value, 0.0, timestamp
            return
        if timestamp <= self.timestamp:
            return
        dt = clamp((timestamp - self.timestamp) / 1000.0, 0.02, 0.5)
        predicted = self.value + self.rate * dt
        residual = wrap(value - predicted) if angular else value - predicted
        self.value = predicted + self.alpha * residual
        if angular:
            self.value = wrap(self.value)
        self.rate = clamp(self.rate + self.beta * residual / dt,
                          -self.rate_limit, self.rate_limit)
        self.timestamp = timestamp

    def predict(self, seconds: float, angular: bool = False) -> float:
        assert self.value is not None
        value = self.value + self.rate * seconds
        return wrap(value) if angular else value


def iou(a: tuple[float, float, float, float],
        b: tuple[float, float, float, float]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    overlap = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return overlap / (area_a + area_b - overlap) if area_a + area_b > overlap else 0.0


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def summarize(rows: list[dict[str, float]]) -> dict[str, float | int]:
    errors = [row["center_error_px"] for row in rows]
    angles = [row["center_error_deg"] for row in rows]
    overlaps = [row["iou"] for row in rows]
    return {
        "samples": len(rows),
        "mean_iou": round(sum(overlaps) / len(rows), 4),
        "iou_at_0_5": round(sum(v >= 0.5 for v in overlaps) / len(rows), 4),
        "median_center_error_px": round(percentile(errors, 0.5), 2),
        "p90_center_error_px": round(percentile(errors, 0.9), 2),
        "p95_center_error_px": round(percentile(errors, 0.95), 2),
        "median_center_error_deg": round(percentile(angles, 0.5), 4),
        "p90_center_error_deg": round(percentile(angles, 0.9), 4),
        "p95_center_error_deg": round(percentile(angles, 0.95), 4),
        "mean_center_error_px": round(sum(errors) / len(errors), 2),
        "aim_inside_true_box_rate": round(sum(row["aim_inside"] for row in rows) / len(rows), 4),
        "p95_normalized_center_error": round(percentile(
            [row["normalized_error"] for row in rows], 0.95), 4),
    }


def compare(args: argparse.Namespace) -> dict:
    frames = read_rows(args.session / "frames.csv")
    labels = {int(row["video_frame"]): tuple(float(row[key]) for key in
              ("x1", "y1", "x2", "y2")) for row in read_rows(args.labels)}
    pose_rows = read_rows(args.poses)
    if not frames:
        raise ValueError("frame index is empty")
    frame_clocks = {row.get("clock_id", "") for row in frames}
    if len(frame_clocks) != 1:
        raise ValueError("frame index mixes clock domains/boots")
    recording_clock = next(iter(frame_clocks))
    if recording_clock:
        pose_rows = [row for row in pose_rows if row.get("clock_id") == recording_clock]
    elif any(row.get("clock_id") for row in pose_rows):
        raise ValueError("legacy frame times cannot be paired with monotonic poses")
    poses = sorted((int(row["ts_ms"]), float(row["yaw_deg"]),
                    float(row["pitch_deg"])) for row in pose_rows)
    pose_times = [pose[0] for pose in poses]
    if len(labels) < 10:
        raise ValueError("at least 10 manually labelled frames are required")
    if not poses:
        raise ValueError("camera pose file is empty; compensated comparison is invalid")
    video = args.session / "liveview.mkv"
    if not video.exists():
        video = args.session / "liveview.avi"
    import cv2

    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise ValueError(f"cannot open {video}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    capture.release()
    fx = width / (2.0 * math.tan(math.radians(args.hfov_deg) / 2.0))
    fy = height / (2.0 * math.tan(math.radians(args.vfov_deg) / 2.0))
    cx, cy = width / 2.0, height / 2.0
    native = {}
    if getattr(args, "native_predictor", None):
        from prediction_native import NativePredictor
        if args.alpha != 0.65 or args.beta != 0.12:
            raise ValueError("native comparison uses flight gains alpha=0.65, beta=0.12")
        for method, adaptive in (("compensated", False), ("adaptive", True)):
            native[method] = NativePredictor(args.native_predictor, width, height,
                args.hfov_deg, args.vfov_deg, adaptive, args.horizon_ms / 1000.0,
                getattr(args, "residual_min_deg", 3.0), getattr(args, "residual_box_scale", 1.5),
                getattr(args, "box_small_deg", 4.0), getattr(args, "box_large_deg", 15.0))
    pixel_x = AlphaBeta(args.alpha, args.beta, 2 * width)
    pixel_y = AlphaBeta(args.alpha, args.beta, 2 * height)
    world_yaw = AlphaBeta(args.alpha, args.beta, 30.0, reset_after_gap=True)
    world_pitch = AlphaBeta(args.alpha, args.beta, 30.0, reset_after_gap=True)
    last_observation = -10**15
    observation_count = 0
    last_box: tuple[float, float, float, float] | None = None
    results: dict[str, list[dict[str, float]]] = {"pixel": [], "compensated": []}
    turning: dict[str, list[dict[str, float]]] = {"pixel": [], "compensated": []}
    if native:
        results["adaptive"], turning["adaptive"] = [], []
    proximity = {group: {method: [] for method in results} for group in ("small_box", "large_box")}
    skipped_pose = 0
    skipped_gt = 0
    pair_rows = []
    actual_horizons_ms: list[int] = []
    frame_times = [int(row["frame_ts_ms"]) for row in frames]
    if any(b <= a for a, b in zip(frame_times, frame_times[1:])):
        raise ValueError("frame timestamps must increase strictly; inspect clock/recording")
    adaptive_reasons: dict[int, int] = {}
    for index, row in enumerate(frames):
        ts = frame_times[index]
        # Runtime input only uses poses available at this simulated control tick.
        pose_end = bisect_left(pose_times, ts + 1)
        current_pose = pose_at(poses[:pose_end], ts, args.max_pose_age_ms)
        in_dropout = (args.dropout_every_ms > 0 and
                      (ts - frame_times[0]) % args.dropout_every_ms < args.dropout_duration_ms)
        # All methods receive the same delayed image and its capture-time pose.
        # The observation must already be available at this control tick.
        delay_ms = max(0, getattr(args, "observation_delay_ms", 0))
        observation_index = bisect_left(frame_times, ts - delay_ms + 1) - 1
        observation_ts = frame_times[observation_index] if observation_index >= 0 else None
        gt = labels.get(int(frames[observation_index]["video_frame"])) if observation_index >= 0 else None
        observation_pose = (pose_at(poses[:pose_end], observation_ts, args.max_pose_age_ms)
                            if observation_ts is not None else None)
        if (gt is not None and observation_pose is not None and not in_dropout and
                observation_ts - last_observation >= args.observation_ms):
            u, v = (gt[0] + gt[2]) / 2.0, (gt[1] + gt[3]) / 2.0
            pixel_x.update(u, observation_ts)
            pixel_y.update(v, observation_ts)
            world_yaw.update(wrap(observation_pose[0] + math.degrees(math.atan((u - cx) / fx))),
                             observation_ts, angular=True)
            world_pitch.update(observation_pose[1] - math.degrees(math.atan((v - cy) / fy)),
                               observation_ts)
            for predictor in native.values():
                predictor.observe(observation_ts, observation_pose, gt)
            last_observation, last_box = observation_ts, gt
            observation_count += 1
        if last_box is None or ts - last_observation > args.max_age_ms:
            continue
        if observation_count < getattr(args, 'warmup_observations', 0):
            continue
        future_ts = ts + args.horizon_ms
        future_index = bisect_left(frame_times, future_ts)
        if future_index >= len(frames):
            continue
        if future_index > index + 1 and abs(frame_times[future_index - 1] - future_ts) < abs(frame_times[future_index] - future_ts):
            future_index -= 1
        if abs(frame_times[future_index] - future_ts) > args.future_tolerance_ms:
            continue
        future_frame = int(frames[future_index]["video_frame"])
        truth = labels.get(future_frame)
        if truth is None:
            skipped_gt += 1
            continue
        future_pose = pose_at(poses, frame_times[future_index], args.max_pose_age_ms)
        if current_pose is None or future_pose is None:
            skipped_pose += 1
            continue
        actual_horizon_ms = frame_times[future_index] - ts
        dt = clamp((future_ts - last_observation) / 1000.0, 0.0, 0.5)
        old_u, old_v = clamp(pixel_x.predict(dt), 0, width), clamp(pixel_y.predict(dt), 0, height)
        old_world_yaw = wrap(current_pose[0] + math.degrees(math.atan((old_u - cx) / fx)))
        old_world_pitch = current_pose[1] - math.degrees(math.atan((old_v - cy) / fy))
        if native:
            pixel_result = native['compensated'].predict_pixel(ts)
            if pixel_result is None:
                continue
            old_world_yaw = wrap(current_pose[0] + pixel_result[0])
            old_world_pitch = current_pose[1] + pixel_result[1]
        predictions = {
            "pixel": (
                cx + fx * math.tan(math.radians(wrap(old_world_yaw - future_pose[0]))),
                cy - fy * math.tan(math.radians(old_world_pitch - future_pose[1])),
            ),
            "compensated": (
                cx + fx * math.tan(math.radians(wrap(world_yaw.predict(dt, True) - future_pose[0]))),
                cy - fy * math.tan(math.radians(world_pitch.predict(dt) - future_pose[1])),
            ),
        }
        if native:
            evaluated = {method: predictor.predict(ts) for method, predictor in native.items()}
            if any(value is None for value in evaluated.values()):
                continue
            for method, value in evaluated.items():
                predictions[method] = (
                    cx + fx * math.tan(math.radians(wrap(value[0] - future_pose[0]))),
                    cy - fy * math.tan(math.radians(value[1] - future_pose[1])),
                )
            reason = int(evaluated["adaptive"][4])
            adaptive_reasons[reason] = adaptive_reasons.get(reason, 0) + 1
        box_width, box_height = last_box[2] - last_box[0], last_box[3] - last_box[1]
        actual_horizons_ms.append(actual_horizon_ms)
        box_angle = math.degrees(math.hypot(math.atan(box_width / (2 * fx)),
                                           math.atan(box_height / (2 * fy))))
        group = "large_box" if box_angle >= getattr(args, "near_box_angle_deg", 5.0) else "small_box"
        for method, (u, v) in predictions.items():
            box = (u - box_width / 2, v - box_height / 2,
                   u + box_width / 2, v + box_height / 2)
            error = math.hypot(u - (truth[0] + truth[2]) / 2,
                               v - (truth[1] + truth[3]) / 2)
            truth_u, truth_v = (truth[0] + truth[2]) / 2, (truth[1] + truth[3]) / 2
            angular_error = math.hypot(
                math.degrees(math.atan((u - cx) / fx) - math.atan((truth_u - cx) / fx)),
                math.degrees(math.atan((v - cy) / fy) - math.atan((truth_v - cy) / fy)))
            metric = {"iou": iou(box, truth), "center_error_px": error,
                      "center_error_deg": angular_error,
                      "aim_inside": float(truth[0] <= u <= truth[2] and truth[1] <= v <= truth[3]),
                      "normalized_error": error / max(1.0, math.hypot(truth[2]-truth[0], truth[3]-truth[1]))}
            results[method].append(metric)
            proximity[group][method].append(metric)
            if abs(wrap(future_pose[0] - current_pose[0])) >= args.turn_threshold_deg:
                turning[method].append(metric)
            pair_rows.append([index, future_index, method, ts, frame_times[future_index],
                              round(u, 3), round(v, 3), round(metric["iou"], 5),
                              round(error, 3), round(angular_error, 4)])
    if not results["pixel"] or not results["compensated"]:
        raise ValueError("no paired samples: check manual labels, pose timestamps and video")
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "paired_predictions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["frame", "future_frame", "method", "frame_ts_ms", "future_ts_ms",
                         "pred_u", "pred_v", "iou", "center_error_px", "center_error_deg"])
        writer.writerows(pair_rows)
    report = {
        "settings": {"horizon_ms": args.horizon_ms,
                     "warmup_observations": getattr(args, 'warmup_observations', 0),
                     "observation_delay_ms": getattr(args, "observation_delay_ms", 0),
                     "near_box_angle_deg": getattr(args, "near_box_angle_deg", 5.0),
                     "box_small_deg": getattr(args, "box_small_deg", 4.0),
                     "box_large_deg": getattr(args, "box_large_deg", 15.0),
                     "observation_ms": args.observation_ms,
                     "max_age_ms": args.max_age_ms,
                     "hfov_deg": args.hfov_deg, "vfov_deg": args.vfov_deg},
        "paired_samples": len(results["pixel"]),
        "clock_id": recording_clock or "legacy-unverified",
        "native_flight_predictor": bool(native),
        "adaptive_reason_counts": adaptive_reasons,
        "median_actual_horizon_ms": percentile(actual_horizons_ms, 0.5),
        "evaluated_fraction_of_video_frames": round(len(results["pixel"]) / len(frames), 4),
        "skipped_missing_ground_truth": skipped_gt,
        "skipped_missing_pose": skipped_pose,
        "pixel": summarize(results["pixel"]),
        "compensated": summarize(results["compensated"]),
        "baseline_definition": "pixel = alpha-beta image position plus pixel velocity times (observation age + fixed future lead), capped at 0.5s; no camera motion compensation in the estimate",
        "by_observed_box_size": {group: {method: summarize(values) if values else {"samples": 0}
                                 for method, values in methods.items()}
                                 for group, methods in proximity.items()},
        "paired_comparison_to_pixel": {
            method: {"center_error_win_rate": round(sum(
                a["center_error_px"] < b["center_error_px"] for a, b in zip(values, results["pixel"])) / len(values), 4),
                "mean_iou_delta": round(sum(a["iou"]-b["iou"] for a, b in zip(values, results["pixel"])) / len(values), 4)}
            for method, values in results.items() if method != "pixel"
        },
        "during_camera_turn": {method: summarize(values) if values else {"samples": 0}
                               for method, values in turning.items()},
        "note": "Reference boxes sampled as observations (label provenance must be reported separately); future camera pose projects both predictions into the same evaluation frame. This is an offline prediction test with oracle future pose for scoring, not a deployable controller or flight-success test.",
    }
    if native:
        report["adaptive"] = summarize(results["adaptive"])
        for predictor in native.values():
            predictor.close()
    (args.output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    ann = sub.add_parser("annotate")
    ann.add_argument("video", type=Path)
    ann.add_argument("output", type=Path)
    cmp = sub.add_parser("compare")
    cmp.add_argument("session", type=Path)
    cmp.add_argument("labels", type=Path)
    cmp.add_argument("poses", type=Path)
    cmp.add_argument("output", type=Path)
    cmp.add_argument("--observation-ms", type=int, default=200)
    cmp.add_argument("--warmup-observations", type=int, default=0)
    cmp.add_argument("--horizon-ms", type=int, default=150)
    cmp.add_argument("--max-age-ms", type=int, default=500)
    cmp.add_argument("--future-tolerance-ms", type=int, default=60)
    cmp.add_argument("--max-pose-age-ms", type=int, default=120)
    cmp.add_argument("--turn-threshold-deg", type=float, default=1.0)
    cmp.add_argument("--dropout-every-ms", type=int, default=0)
    cmp.add_argument("--dropout-duration-ms", type=int, default=0)
    cmp.add_argument("--hfov-deg", type=float, default=73.7)
    cmp.add_argument("--vfov-deg", type=float, default=45.7)
    cmp.add_argument("--alpha", type=float, default=0.65)
    cmp.add_argument("--beta", type=float, default=0.12)
    cmp.add_argument("--native-predictor", type=Path,
                     help="built prediction_native.dll/.so; adds the exact C adaptive predictor")
    cmp.add_argument("--observation-delay-ms", type=int, default=100,
                     help="same image delivery delay for all predictors; 0 disables simulation")
    cmp.add_argument("--near-box-angle-deg", type=float, default=5.0,
                     help="half-diagonal angular-size threshold for reporting, not measured distance")
    cmp.add_argument("--box-small-deg", type=float, default=4.0)
    cmp.add_argument("--box-large-deg", type=float, default=15.0)
    cmp.add_argument("--residual-min-deg", type=float, default=3.0)
    cmp.add_argument("--residual-box-scale", type=float, default=1.5)
    args = parser.parse_args()
    if args.command == "annotate":
        annotate(args.video, args.output)
    else:
        print(json.dumps(compare(args), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
