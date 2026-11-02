#!/usr/bin/env python3
"""Small TensorRT runtime for the static 640x640 balloon detector engine."""

from __future__ import annotations

import ctypes
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np


CUDA_MEMCPY_HOST_TO_DEVICE = 1
CUDA_MEMCPY_DEVICE_TO_HOST = 2


class AdaptiveSoftwareExposure:
    """Low-rate brightness controller applied only to the YOLO input tensor."""

    GAMMA_STEPS = (0.70, 0.80, 0.90, 1.00, 1.10, 1.25, 1.40)

    def __init__(self) -> None:
        self.enabled = os.environ.get("BALLOON_SOFTWARE_AUTO_EXPOSURE", "0").lower() in {
            "1", "true", "on", "yes"
        }
        self.refresh_ms = min(
            10000,
            max(500, int(os.environ.get("BALLOON_SOFTWARE_AUTO_EXPOSURE_REFRESH_MS", "1000"))),
        )
        self.target_low = min(
            160.0,
            max(32.0, float(os.environ.get("BALLOON_SOFTWARE_EXPOSURE_TARGET_LOW", "88"))),
        )
        self.target_high = min(
            224.0,
            max(self.target_low + 8.0,
                float(os.environ.get("BALLOON_SOFTWARE_EXPOSURE_TARGET_HIGH", "152"))),
        )
        self.gamma = 1.0
        self.last_update_monotonic = 0.0
        self.update_id = 0
        self.last_info: dict[str, Any] = {
            "enabled": self.enabled,
            "gamma": self.gamma,
            "refresh_ms": self.refresh_ms,
            "reason": "waiting_for_frame" if self.enabled else "disabled",
        }
        self._lut_gamma: float | None = None
        self._lut: np.ndarray | None = None

    @staticmethod
    def _nearest_gamma(value: float) -> float:
        return min(AdaptiveSoftwareExposure.GAMMA_STEPS, key=lambda item: abs(item - value))

    def _evaluate(self, image: np.ndarray, now: float) -> None:
        height, width = image.shape[:2]
        y1, y2 = int(height * 0.10), max(int(height * 0.90), 1)
        x1, x2 = int(width * 0.10), max(int(width * 0.90), 1)
        crop = image[y1:y2, x1:x2]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        p05, median, p95 = [float(value) for value in np.percentile(gray, (5, 50, 95))]
        low_clip = float(np.mean(gray <= 8))
        high_clip = float(np.mean(gray >= 247))
        target_mid = (self.target_low + self.target_high) * 0.5

        if self.target_low <= median <= self.target_high and low_clip < 0.08 and high_clip < 0.04:
            requested = 1.0
            reason = "within_target"
        elif median < self.target_low or low_clip >= 0.08:
            safe_median = min(254.0, max(2.0, median))
            requested = math.log(target_mid / 255.0) / math.log(safe_median / 255.0)
            requested = min(1.0, max(0.70, requested))
            reason = "underexposed"
        else:
            safe_median = min(254.0, max(2.0, median))
            requested = math.log(target_mid / 255.0) / math.log(safe_median / 255.0)
            requested = max(1.0, min(1.40, requested))
            if high_clip >= 0.04:
                requested = max(requested, 1.25)
            reason = "overexposed"

        self.gamma = self._nearest_gamma(requested)
        self.update_id += 1
        self.last_update_monotonic = now
        self.last_info = {
            "enabled": True,
            "update_id": self.update_id,
            "refresh_ms": self.refresh_ms,
            "gamma": self.gamma,
            "reason": reason,
            "p05": round(p05, 3),
            "median": round(median, 3),
            "p95": round(p95, 3),
            "low_clip_ratio": round(low_clip, 6),
            "high_clip_ratio": round(high_clip, 6),
            "target_range": [self.target_low, self.target_high],
        }

    def apply(self, image: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return image
        self.observe(image)
        return self.transform(image)

    def observe(self, image: np.ndarray) -> dict[str, Any]:
        if not self.enabled:
            return self.last_info
        now = time.monotonic()
        if self.last_update_monotonic == 0.0 or (now - self.last_update_monotonic) * 1000 >= self.refresh_ms:
            self._evaluate(image, now)
        return self.last_info

    def transform(self, image: np.ndarray) -> np.ndarray:
        if not self.enabled:
            return image
        if self.gamma == 1.0:
            return image
        if self._lut is None or self._lut_gamma != self.gamma:
            values = np.arange(256, dtype=np.float32) / 255.0
            self._lut = np.clip(np.power(values, self.gamma) * 255.0, 0, 255).astype(np.uint8)
            self._lut_gamma = self.gamma
        return cv2.LUT(image, self._lut)


class CudaError(RuntimeError):
    pass


class CudaRuntime:
    def __init__(self) -> None:
        last_error: Exception | None = None
        for name in ("libcudart.so", "libcudart.so.12"):
            try:
                self.lib = ctypes.CDLL(name)
                break
            except OSError as exc:
                last_error = exc
        else:
            raise CudaError(f"unable to load CUDA runtime: {last_error!r}")

        self.lib.cudaGetErrorString.restype = ctypes.c_char_p
        self.lib.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self.lib.cudaFree.argtypes = [ctypes.c_void_p]
        self.lib.cudaMemcpyAsync.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_int,
            ctypes.c_void_p,
        ]
        self.lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
        self.lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]
        self.lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]

    def check(self, status: int, operation: str) -> None:
        if status != 0:
            detail = self.lib.cudaGetErrorString(status)
            message = detail.decode("utf-8", errors="replace") if detail else f"status {status}"
            raise CudaError(f"{operation} failed: {message}")

    def malloc(self, size: int) -> ctypes.c_void_p:
        pointer = ctypes.c_void_p()
        self.check(self.lib.cudaMalloc(ctypes.byref(pointer), size), "cudaMalloc")
        return pointer

    def free(self, pointer: ctypes.c_void_p) -> None:
        self.check(self.lib.cudaFree(pointer), "cudaFree")

    def stream_create(self) -> ctypes.c_void_p:
        stream = ctypes.c_void_p()
        self.check(self.lib.cudaStreamCreate(ctypes.byref(stream)), "cudaStreamCreate")
        return stream

    def stream_destroy(self, stream: ctypes.c_void_p) -> None:
        self.check(self.lib.cudaStreamDestroy(stream), "cudaStreamDestroy")

    def copy_async(
        self,
        destination: int | ctypes.c_void_p,
        source: int | ctypes.c_void_p,
        size: int,
        direction: int,
        stream: ctypes.c_void_p,
    ) -> None:
        dst = destination if isinstance(destination, ctypes.c_void_p) else ctypes.c_void_p(destination)
        src = source if isinstance(source, ctypes.c_void_p) else ctypes.c_void_p(source)
        self.check(self.lib.cudaMemcpyAsync(dst, src, size, direction, stream), "cudaMemcpyAsync")

    def stream_synchronize(self, stream: ctypes.c_void_p) -> None:
        self.check(self.lib.cudaStreamSynchronize(stream), "cudaStreamSynchronize")


def _load_tensorrt() -> Any:
    try:
        import tensorrt as trt  # type: ignore

        return trt
    except ImportError:
        system_packages = Path("/usr/lib/python3.10/dist-packages")
        if system_packages.is_dir():
            sys.path.insert(0, str(system_packages))
        import tensorrt as trt  # type: ignore

        return trt


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[float, float]]:
    height, width = image.shape[:2]
    ratio = min(size / height, size / width)
    resized_width = round(width * ratio)
    resized_height = round(height * ratio)
    pad_width = (size - resized_width) / 2
    pad_height = (size - resized_height) / 2

    if (resized_width, resized_height) != (width, height):
        image = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
    top, bottom = round(pad_height - 0.1), round(pad_height + 0.1)
    left, right = round(pad_width - 0.1), round(pad_width + 0.1)
    image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    tensor = image[:, :, ::-1].transpose(2, 0, 1)
    tensor = np.ascontiguousarray(tensor, dtype=np.float32) / 255.0
    return tensor[None], ratio, (pad_width, pad_height)


def read_for_inference(source: Path, size: int, max_divisor: int = 8) -> tuple[np.ndarray, tuple[int, int], int]:
    """Decode JPEG near the network resolution using libjpeg DCT scaling."""
    from PIL import Image

    with Image.open(source) as header:
        original_width, original_height = header.size
    divisor = 1
    if source.suffix.lower() in {".jpg", ".jpeg"}:
        longest_side = max(original_width, original_height)
        for candidate in (8, 4, 2):
            if candidate <= max_divisor and longest_side / candidate >= size:
                divisor = candidate
                break
    flags = {
        1: cv2.IMREAD_COLOR,
        2: cv2.IMREAD_REDUCED_COLOR_2,
        4: cv2.IMREAD_REDUCED_COLOR_4,
        8: cv2.IMREAD_REDUCED_COLOR_8,
    }
    image = cv2.imread(str(source), flags[divisor])
    if image is None:
        raise ValueError(f"unable to decode image: {source}")
    return image, (original_height, original_width), divisor


def box_iou_one_to_many(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    top_left = np.maximum(box[:2], boxes[:, :2])
    bottom_right = np.minimum(box[2:], boxes[:, 2:])
    intersection = np.prod(np.maximum(bottom_right - top_left, 0.0), axis=1)
    box_area = np.prod(np.maximum(box[2:] - box[:2], 0.0))
    areas = np.prod(np.maximum(boxes[:, 2:] - boxes[:, :2], 0.0), axis=1)
    return intersection / np.maximum(box_area + areas - intersection, 1e-7)


def non_max_suppression(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float, max_det: int) -> list[int]:
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size and len(keep) < max_det:
        current = int(order[0])
        keep.append(current)
        if order.size == 1:
            break
        remaining = order[1:]
        order = remaining[box_iou_one_to_many(boxes[current], boxes[remaining]) <= iou_threshold]
    return keep


def decode_yolo_output(
    output: np.ndarray,
    confidence: float,
    ratio: float,
    padding: tuple[float, float],
    original_shape: tuple[int, int],
    iou_threshold: float = 0.7,
    max_det: int = 300,
) -> list[dict[str, Any]]:
    prediction = np.squeeze(output, axis=0) if output.ndim == 3 and output.shape[0] == 1 else output
    if prediction.ndim != 2:
        raise ValueError(f"unsupported YOLO output shape: {output.shape}")
    if prediction.shape[0] < prediction.shape[1]:
        prediction = prediction.T
    if prediction.shape[1] < 5:
        raise ValueError(f"YOLO output has too few channels: {output.shape}")

    class_scores = prediction[:, 4:]
    class_ids = np.argmax(class_scores, axis=1)
    scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
    selected = scores >= confidence
    prediction, class_ids, scores = prediction[selected], class_ids[selected], scores[selected]
    if prediction.size == 0:
        return []

    xywh = prediction[:, :4]
    boxes = np.empty_like(xywh)
    boxes[:, 0] = xywh[:, 0] - xywh[:, 2] / 2
    boxes[:, 1] = xywh[:, 1] - xywh[:, 3] / 2
    boxes[:, 2] = xywh[:, 0] + xywh[:, 2] / 2
    boxes[:, 3] = xywh[:, 1] + xywh[:, 3] / 2

    kept: list[int] = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        local = non_max_suppression(boxes[indices], scores[indices], iou_threshold, max_det)
        kept.extend(indices[local].tolist())
    kept = sorted(kept, key=lambda index: float(scores[index]), reverse=True)[:max_det]

    height, width = original_shape
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - padding[0]) / ratio
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - padding[1]) / ratio
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, width)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, height)
    return [
        {
            "class_id": int(class_ids[index]),
            "class_name": "balloon" if int(class_ids[index]) == 0 else str(int(class_ids[index])),
            "confidence": float(scores[index]),
            "bbox_xyxy": [float(value) for value in boxes[index]],
        }
        for index in kept
    ]


class TensorRTBalloonDetector:
    def __init__(self, engine_path: Path, image_size: int, confidence: float, max_decode_divisor: int = 8) -> None:
        self.engine_path = engine_path.resolve()
        self.image_size = int(image_size)
        self.confidence = float(confidence)
        self.max_decode_divisor = int(max_decode_divisor)
        self.last_timings_ms: dict[str, float | int] = {}
        self.software_exposure = AdaptiveSoftwareExposure()
        self.last_auto_exposure_info: dict[str, Any] = dict(self.software_exposure.last_info)
        self.last_exposure_retry_monotonic = 0.0
        self.trt = _load_tensorrt()
        self.cuda = CudaRuntime()
        self.logger = self.trt.Logger(self.trt.Logger.WARNING)
        self.runtime = self.trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(self.engine_path.read_bytes())
        if self.engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT engine: {self.engine_path}")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError("failed to create TensorRT execution context")
        self.stream = self.cuda.stream_create()
        self.input_name = ""
        self.output_names: list[str] = []
        self.detection_output_name = ""
        self.device_buffers: dict[str, ctypes.c_void_p] = {}
        self.host_outputs: dict[str, np.ndarray] = {}

        for index in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(index)
            shape = tuple(int(value) for value in self.engine.get_tensor_shape(name))
            if any(value <= 0 for value in shape):
                raise ValueError(f"dynamic tensor is not supported: {name} {shape}")
            dtype = np.dtype(self.trt.nptype(self.engine.get_tensor_dtype(name)))
            size = int(np.prod(shape)) * dtype.itemsize
            pointer = self.cuda.malloc(size)
            self.device_buffers[name] = pointer
            self.context.set_tensor_address(name, int(pointer.value or 0))
            if self.engine.get_tensor_mode(name) == self.trt.TensorIOMode.INPUT:
                if self.input_name:
                    raise ValueError("multiple TensorRT inputs are not supported")
                self.input_name = name
                self.input_shape = shape
                self.input_dtype = dtype
            else:
                self.output_names.append(name)
                self.host_outputs[name] = np.empty(shape, dtype=dtype)
                if len(shape) == 3 and shape[0] == 1 and 5 in shape[1:]:
                    if self.detection_output_name:
                        raise ValueError(f"multiple YOLO-like outputs: {self.detection_output_name}, {name}")
                    self.detection_output_name = name
        if not self.input_name or not self.detection_output_name:
            raise ValueError(
                f"expected one input and a [1,5,N] detection output, got {self.input_name!r}, {self.output_names!r}"
            )
        if self.input_shape != (1, 3, self.image_size, self.image_size):
            raise ValueError(f"unexpected TensorRT input shape: {self.input_shape}")

    def close(self) -> None:
        for pointer in self.device_buffers.values():
            self.cuda.free(pointer)
        self.device_buffers.clear()
        if self.stream:
            self.cuda.stream_destroy(self.stream)
            self.stream = ctypes.c_void_p()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass

    def _infer_reduced_image(self, image: np.ndarray) -> tuple[list[dict[str, Any]], dict[str, float]]:
        started = time.perf_counter()
        tensor, ratio, padding = letterbox(image, self.image_size)
        tensor = np.ascontiguousarray(tensor, dtype=self.input_dtype)
        preprocessed = time.perf_counter()
        input_buffer = self.device_buffers[self.input_name]
        self.cuda.copy_async(input_buffer, tensor.ctypes.data, tensor.nbytes, CUDA_MEMCPY_HOST_TO_DEVICE, self.stream)
        if not self.context.execute_async_v3(stream_handle=int(self.stream.value or 0)):
            raise RuntimeError("TensorRT execute_async_v3 returned false")
        for name in self.output_names:
            output = self.host_outputs[name]
            self.cuda.copy_async(output.ctypes.data, self.device_buffers[name], output.nbytes, CUDA_MEMCPY_DEVICE_TO_HOST, self.stream)
        self.cuda.stream_synchronize(self.stream)
        inferred = time.perf_counter()
        records = decode_yolo_output(
            self.host_outputs[self.detection_output_name],
            self.confidence,
            ratio,
            padding,
            image.shape[:2],
        )
        finished = time.perf_counter()
        return records, {
            "preprocess": (preprocessed - started) * 1000,
            "inference_transfer": (inferred - preprocessed) * 1000,
            "postprocess": (finished - inferred) * 1000,
        }

    def predict_records(self, source: Path) -> list[dict[str, Any]]:
        started = time.perf_counter()
        image, original_shape, decode_divisor = read_for_inference(
            source, self.image_size, self.max_decode_divisor
        )
        decoded = time.perf_counter()
        self.software_exposure.observe(image)
        records, timings = self._infer_reduced_image(image)
        retry_used = False
        retry_recovered = False
        transform_ms = 0.0
        now = time.monotonic()
        if (
            not records
            and self.software_exposure.enabled
            and self.software_exposure.gamma != 1.0
            and (
                self.last_exposure_retry_monotonic == 0.0
                or (now - self.last_exposure_retry_monotonic) * 1000
                >= self.software_exposure.refresh_ms
            )
        ):
            transform_started = time.perf_counter()
            adjusted = self.software_exposure.transform(image)
            transform_ms = (time.perf_counter() - transform_started) * 1000
            retry_records, retry_timings = self._infer_reduced_image(adjusted)
            self.last_exposure_retry_monotonic = now
            retry_used = True
            retry_recovered = bool(retry_records)
            if retry_records:
                records = retry_records
            for key in timings:
                timings[key] += retry_timings[key]

        self.last_auto_exposure_info = dict(self.software_exposure.last_info)
        self.last_auto_exposure_info.update(
            {
                "applied": retry_used,
                "retry_recovered": retry_recovered,
                "policy": "raw_first_retry_on_miss",
            }
        )
        reduced_height, reduced_width = image.shape[:2]
        original_height, original_width = original_shape
        scale_x = original_width / reduced_width
        scale_y = original_height / reduced_height
        if scale_x != 1.0 or scale_y != 1.0:
            for record in records:
                x1, y1, x2, y2 = record["bbox_xyxy"]
                record["bbox_xyxy"] = [x1 * scale_x, y1 * scale_y, x2 * scale_x, y2 * scale_y]
        finished = time.perf_counter()
        self.last_timings_ms = {
            "decode_divisor": decode_divisor,
            "decode": round((decoded - started) * 1000, 3),
            "preprocess": round(timings["preprocess"], 3),
            "inference_transfer": round(timings["inference_transfer"], 3),
            "postprocess": round(timings["postprocess"], 3),
            "auto_exposure_transform": round(transform_ms, 3),
            "auto_exposure_retry": int(retry_used),
            "total": round((finished - started) * 1000, 3),
        }
        return records
