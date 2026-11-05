"""Callable yellow balloon detector using the existing OpenCV implementation."""

from .detector import ColorThresholdDetector, load_color_config

__all__ = ["ColorThresholdDetector", "load_color_config"]
