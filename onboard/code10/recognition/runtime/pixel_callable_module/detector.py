"""Standalone, full-resolution yellow HSV balloon candidate detector.

Defaults are conservative but uncalibrated: synthetic checks do not establish
accuracy on the aircraft camera. Yellow objects with balloon-like silhouettes
can still pass. No neural model, model weights, worker import, or fallback is
used. OpenCV HSV uses H=0..179 and S,V=0..255.

``confidence`` is an explicit ranking heuristic, NOT a learned probability:
0.45 * yellow silhouette coverage + 0.25 * solidity +
0.20 * min(circularity / 0.85, 1) + 0.10 * min(box fill / 0.70, 1).
Hard shape and area filters run before this score. Clean accepted balloons score
above the existing 0.45 acquisition threshold; lowering the configurable score
threshold does not bypass those filters. ``color_confidence`` is the fraction
of the filled outer silhouette that actually matches the yellow HSV mask.
"""

from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import time
from typing import Any

import cv2
import numpy as np


_DEFAULTS: dict[str, Any] = {
    "_comment": "Conservative uncalibrated yellow-only defaults; tune using real camera images before flight.",
    "hsv_lower": [20, 90, 80],
    "hsv_upper": [38, 255, 255],
    "min_area_px": 12,
    "max_area_ratio": 0.35,
    "min_aspect_ratio": 0.35,
    "max_aspect_ratio": 2.8,
    "min_fill_ratio": 0.35,
    "max_fill_ratio": 0.92,
    "min_solidity": 0.80,
    "min_circularity": 0.45,
    "max_rectangularity": 0.92,
    "rectangle_min_area_px": 64,
    "open_kernel": 1,
    "close_kernel": 3,
    "min_confidence": 0.45,
    "max_components": 256,
    "max_candidates": 32,
}


def _normalize_config(config: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(config, dict):
        raise ValueError("color config must be a JSON object")
    unknown = set(config) - set(_DEFAULTS)
    if unknown:
        raise ValueError(f"unknown color config fields: {', '.join(sorted(map(str, unknown)))}")
    settings = deepcopy(_DEFAULTS)
    settings.update(deepcopy(config))
    if not isinstance(settings["_comment"], str):
        raise ValueError("color config _comment must be a string")

    for key in ("hsv_lower", "hsv_upper"):
        value = settings[key]
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"color config {key} must contain three integer HSV channels")
        for channel, (entry, maximum) in enumerate(zip(value, (179, 255, 255))):
            if type(entry) is not int or not 0 <= entry <= maximum:
                raise ValueError(f"color config {key}[{channel}] must be an integer in 0..{maximum}")
        settings[key] = list(value)
    if any(low > high for low, high in zip(settings["hsv_lower"], settings["hsv_upper"])):
        raise ValueError("color config hsv_lower must not exceed hsv_upper in any channel")

    integer_limits = {
        "min_area_px": (1, 1_000_000_000),
        "rectangle_min_area_px": (1, 1_000_000_000),
        "max_components": (1, 100_000),
        "max_candidates": (1, 10_000),
        "open_kernel": (1, 31),
        "close_kernel": (1, 31),
    }
    for key, (lower, upper) in integer_limits.items():
        value = settings[key]
        if type(value) is not int or not lower <= value <= upper:
            raise ValueError(f"color config {key} must be an integer in {lower}..{upper}")
    for key in ("open_kernel", "close_kernel"):
        if settings[key] % 2 != 1:
            raise ValueError(f"color config {key} must be odd; use 1 to disable")
    if settings["max_candidates"] > settings["max_components"]:
        raise ValueError("color config max_candidates must not exceed max_components")

    numeric_limits = {
        "max_area_ratio": (0.0, 1.0, False),
        "min_aspect_ratio": (0.0, 1000.0, False),
        "max_aspect_ratio": (0.0, 1000.0, False),
        "min_fill_ratio": (0.0, 1.0, True),
        "max_fill_ratio": (0.0, 1.0, False),
        "min_solidity": (0.0, 1.0, True),
        "min_circularity": (0.0, 1.0, True),
        "max_rectangularity": (0.0, 1.0, False),
        "min_confidence": (0.0, 1.0, True),
    }
    for key, (lower, upper, include_lower) in numeric_limits.items():
        value = settings[key]
        if type(value) not in (int, float) or (type(value) is float and not math.isfinite(value)):
            raise ValueError(f"color config {key} must be a finite number")
        if value > upper or value < lower or (not include_lower and value == lower):
            bracket = "[" if include_lower else "("
            raise ValueError(f"color config {key} must be in {bracket}{lower}, {upper}]")
        settings[key] = float(value)
    for lower, upper in (("min_aspect_ratio", "max_aspect_ratio"),
                         ("min_fill_ratio", "max_fill_ratio")):
        if settings[lower] > settings[upper]:
            raise ValueError(f"color config {lower} must not exceed {upper}")
    return settings


def load_color_config(path: Path | None = None) -> dict[str, Any]:
    """Read UTF-8 JSON and normalize overrides against the documented defaults.

    Missing and malformed files fail immediately, including when an explicitly
    supplied path is invalid. Unknown fields are errors rather than silent typos.
    """
    source = Path(path) if path is not None else Path(__file__).with_name("config.json")
    try:
        config = json.loads(source.read_text(encoding="utf-8-sig"))
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise ValueError(f"invalid color config JSON in {source}: {exc}") from exc
    return _normalize_config(config)


def color_runtime_config(detector: "ColorThresholdDetector") -> dict[str, Any]:
    """Adapt color settings to worker/report metadata without claiming YOLO work."""
    config = detector.config
    return {
        "weights": None,
        "imgsz": 0,
        "conf": config["min_confidence"],
        "device": "cpu",
        "save": False,
        "color_detector": config,
    }


class ColorThresholdDetector:
    """Detect yellow connected regions and retain plausible balloon silhouettes.

    ``predict_records`` decodes one original-resolution image per call;
    ``predict_frame`` accepts an already decoded BGR frame. The largest
    ``max_components`` area-qualified regions receive contour checks; at most
    ``max_candidates`` records are returned. Results sort by descending score,
    descending yellow area, then top/left coordinates. Severe crowding can thus
    hide smaller targets; no extra crops, rescaling, or neural fallback occurs.
    """

    backend_name = "color_threshold"
    provides_colors = True
    supports_far_tiles = False
    class_names = {0: "balloon"}

    def __init__(self, config: dict[str, Any] | None = None):
        self._config = load_color_config() if config is None else _normalize_config(config)
        self.last_timings_ms: dict[str, float] = {}
        self._lower = np.array(self._config["hsv_lower"], dtype=np.uint8)
        self._upper = np.array(self._config["hsv_upper"], dtype=np.uint8)
        self._kernels = {
            key: cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self._config[key], self._config[key]))
            for key in ("open_kernel", "close_kernel")
            if self._config[key] > 1
        }

    @property
    def config(self) -> dict[str, Any]:
        """Return a detached settings copy; mutation cannot invalidate the backend."""
        return deepcopy(self._config)

    def predict_records(self, source: Path) -> list[dict[str, Any]]:
        """Decode a path without EXIF rotation, then use the common frame API."""
        started = time.perf_counter()
        self.last_timings_ms = {}
        path = Path(source)
        if path.is_dir():
            raise IsADirectoryError(f"image source is a directory: {path}")
        encoded = path.read_bytes()  # Path supports Unicode filenames on Windows.
        if not encoded:
            raise ValueError(f"empty image: {path}")
        try:
            # Worker image_size() uses raw dimensions. EXIF display rotation must
            # not rotate/mirror detections into a different coordinate system.
            image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8),
                                 cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        except cv2.error as exc:
            raise ValueError(f"cannot decode image: {path}") from exc
        if image is None or image.size == 0:
            raise ValueError(f"cannot decode image: {path}")
        decoded = time.perf_counter()
        records = self.predict_frame(image, str(path))
        self.last_timings_ms["read_decode"] = (decoded - started) * 1000.0
        self.last_timings_ms["total"] = (time.perf_counter() - started) * 1000.0
        return records

    def predict_frame(self, frame: np.ndarray, source: str = "") -> list[dict[str, Any]]:
        """Detect in a nonempty uint8 HWC BGR frame, preserving pixel coordinates.

        ``source`` is an opaque label copied into each record's ``image`` field.
        No resizing, file read, or display-orientation correction is performed.
        """
        started = time.perf_counter()
        self.last_timings_ms = {}
        if not isinstance(frame, np.ndarray):
            raise TypeError("frame must be a numpy ndarray")
        if frame.dtype != np.uint8:
            raise TypeError("frame must have dtype uint8")
        if frame.ndim != 3 or frame.shape[2] != 3 or frame.size == 0:
            raise ValueError("frame must be nonempty HWC with three BGR channels")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = yellow_mask
        for key, operation in (("open_kernel", cv2.MORPH_OPEN), ("close_kernel", cv2.MORPH_CLOSE)):
            if key in self._kernels:
                mask = cv2.morphologyEx(mask, operation, self._kernels[key])
        masked = time.perf_counter()
        records = self._records_from_mask(source, hsv, yellow_mask, mask)
        finished = time.perf_counter()
        self.last_timings_ms = {
            "read_decode": 0.0,
            "mask": (masked - started) * 1000.0,
            "components": (finished - masked) * 1000.0,
            "total": (finished - started) * 1000.0,
        }
        return records

    def _records_from_mask(self, path: str | Path, hsv: np.ndarray,
                           yellow_mask: np.ndarray, mask: np.ndarray) -> list[dict[str, Any]]:
        settings = self._config
        height, width = mask.shape
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        areas = stats[:, cv2.CC_STAT_AREA]
        candidates = np.flatnonzero((areas >= settings["min_area_px"]) &
                                    (areas <= width * height * settings["max_area_ratio"]))
        candidates = candidates[candidates != 0]
        # The bounded slice precedes contour extraction and HSV statistics.
        candidates = sorted(candidates, key=lambda i: (-int(areas[i]),
                            int(stats[i, cv2.CC_STAT_TOP]), int(stats[i, cv2.CC_STAT_LEFT])))
        accepted: list[tuple[dict[str, Any], int]] = []
        for label in candidates[:settings["max_components"]]:
            x, y, box_width, box_height, area = map(int, stats[label])
            aspect = box_width / box_height
            fill = area / (box_width * box_height)
            if not settings["min_aspect_ratio"] <= aspect <= settings["max_aspect_ratio"]:
                continue
            if not settings["min_fill_ratio"] <= fill <= settings["max_fill_ratio"]:
                continue
            region = np.uint8(labels[y:y + box_height, x:x + box_width] == label) * 255
            contours, _ = cv2.findContours(region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            contour = max(contours, key=cv2.contourArea)
            contour_area = float(cv2.contourArea(contour))
            perimeter = float(cv2.arcLength(contour, True))
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            if contour_area <= 0.0 or perimeter <= 0.0 or hull_area <= 0.0:
                continue
            solidity = min(1.0, contour_area / hull_area)
            circularity = min(1.0, 4.0 * math.pi * contour_area / (perimeter * perimeter))
            if solidity < settings["min_solidity"] or circularity < settings["min_circularity"]:
                continue
            # Axis-aligned fill alone cannot reject rotated rectangular signs.
            # Tiny raster circles resemble diamonds; avoid applying this filter
            # before there are enough pixels for a reliable rectangularity test.
            if area >= settings["rectangle_min_area_px"]:
                (_, _), (rect_width, rect_height), _ = cv2.minAreaRect(contour)
                rectangle_area = rect_width * rect_height
                if rectangle_area > 0 and contour_area / rectangle_area > settings["max_rectangularity"]:
                    continue
            silhouette = np.zeros_like(region)
            cv2.drawContours(silhouette, [contour], -1, 255, cv2.FILLED)
            matched = (yellow_mask[y:y + box_height, x:x + box_width] != 0) & (silhouette != 0)
            yellow_area = int(np.count_nonzero(matched))
            if yellow_area < settings["min_area_px"]:
                continue
            coverage = yellow_area / int(np.count_nonzero(silhouette))
            confidence = min(1.0, 0.45 * coverage + 0.25 * solidity +
                             0.20 * min(circularity / 0.85, 1.0) +
                             0.10 * min(fill / 0.70, 1.0))
            if confidence < settings["min_confidence"]:
                continue
            median = np.median(hsv[y:y + box_height, x:x + box_width][matched], axis=0)
            record = {
                "image": str(path),
                "class_id": 0,
                "class_name": "balloon",
                "confidence": float(confidence),
                "bbox_xyxy": [float(x), float(y), float(x + box_width), float(y + box_height)],
                "color_name": "yellow",
                "color_confidence": float(coverage),
                "color_runner_up": "unknown",
                "color_margin": float(coverage),
                "color_chromatic_ratio": float(yellow_area / (box_width * box_height)),
                "color_hsv_median": [int(value) for value in median],
            }
            accepted.append((record, yellow_area))
        accepted.sort(key=lambda item: (-item[0]["confidence"], -item[1],
                      item[0]["bbox_xyxy"][1], item[0]["bbox_xyxy"][0]))
        return [record for record, _ in accepted[:settings["max_candidates"]]]
