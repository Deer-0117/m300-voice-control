"""ctypes adapter for target_chase.c, built by the supplied build scripts."""
import ctypes as ct
from pathlib import Path


class NativePredictor:
    def __init__(self, library: Path, width: int, height: int, hfov: float, vfov: float,
                 adaptive: bool, horizon: float = 0.15, residual: float = 3.0,
                 box_scale: float = 1.5, small_box_deg: float = 4.0, large_box_deg: float = 15.0):
        self.lib = ct.CDLL(str(library.resolve()))
        self.lib.prediction_create.argtypes = [ct.c_float] * 4 + [ct.c_int] + [ct.c_float] * 3
        self.lib.prediction_create.restype = ct.c_void_p
        self.lib.prediction_destroy.argtypes = [ct.c_void_p]
        self.lib.prediction_destroy.restype = None
        self.lib.prediction_observe.argtypes = [ct.c_void_p, ct.c_uint64] + [ct.c_float] * 6
        self.lib.prediction_observe.restype = None
        self.lib.prediction_evaluate.argtypes = [ct.c_void_p, ct.c_uint64, ct.POINTER(ct.c_float)]
        self.lib.prediction_evaluate.restype = ct.c_int
        self.handle = self.lib.prediction_create(width, height, hfov, vfov, adaptive,
                                                 horizon, residual, box_scale)
        if not self.handle:
            raise MemoryError('native predictor allocation failed')
        if hasattr(self.lib, 'prediction_set_box_thresholds'):
            self.lib.prediction_set_box_thresholds.argtypes = [ct.c_void_p, ct.c_float, ct.c_float]
            self.lib.prediction_set_box_thresholds.restype = None
            self.lib.prediction_set_box_thresholds(self.handle, small_box_deg, large_box_deg)
        elif adaptive:
            self.close()
            raise ValueError('rebuild the native library for near-distance adaptive comparison')

    def observe(self, timestamp: int, pose: tuple, box: tuple) -> None:
        self.lib.prediction_observe(self.handle, timestamp, *pose, *box)

    def predict(self, now: int) -> list[float] | None:
        output = (ct.c_float * 6)()
        if not self.lib.prediction_evaluate(self.handle, now, output):
            return None
        return list(output)

    def predict_pixel(self, now: int) -> list[float] | None:
        self.lib.prediction_evaluate_pixel.argtypes = [ct.c_void_p, ct.c_uint64, ct.POINTER(ct.c_float)]
        self.lib.prediction_evaluate_pixel.restype = ct.c_int
        output = (ct.c_float * 3)()
        if not self.lib.prediction_evaluate_pixel(self.handle, now, output):
            return None
        return list(output)

    def close(self) -> None:
        if self.handle:
            self.lib.prediction_destroy(self.handle)
            self.handle = None

    def __del__(self):
        if getattr(self, 'handle', None):
            self.close()
