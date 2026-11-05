"""Select exactly one detector at process startup; no neural imports here."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "detector_config.json"


def ensure_script_interpreter():
    """Keep legacy CLI YOLO dependencies available, without re-exec on import."""
    import os
    import sys
    environment = ROOT / "yolo_env"
    if read_selection()["active_detector"] == "yolo" and Path(sys.prefix) != environment:
        python = environment / "bin" / "python"
        if not python.is_file():
            raise FileNotFoundError(f"YOLO Python environment not found: {python}")
        os.execv(str(python), [str(python), "-B", *sys.argv])


def read_selection(path=DEFAULT_CONFIG):
    path = Path(path).resolve()
    settings = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(settings, dict) or set(settings) != {"active_detector", "pixel_config"}:
        raise ValueError("detector config requires exactly active_detector and pixel_config")
    if settings["active_detector"] not in ("yolo", "pixel"):
        raise ValueError("active_detector must be 'yolo' or 'pixel'; automatic algorithm fallback is disabled")
    if not isinstance(settings["pixel_config"], str) or not settings["pixel_config"].strip():
        raise ValueError("pixel_config must be a nonempty path")
    return settings


def load_detector(path=DEFAULT_CONFIG, yolo_factory=None):
    path = Path(path).resolve()
    settings = read_selection(path)
    if settings["active_detector"] == "pixel":
        from pixel_callable_module import ColorThresholdDetector, load_color_config
        from pixel_callable_module.detector import color_runtime_config
        pixel_path = (path.parent / settings["pixel_config"]).resolve()
        detector = ColorThresholdDetector(load_color_config(pixel_path))
        config = color_runtime_config(detector)
        config["pixel_config_path"] = str(pixel_path)
        weights = None
    else:
        if yolo_factory is None:
            from balloon_yolo_worker import load_yolo_detector
            yolo_factory = load_yolo_detector
        detector, config, weights = yolo_factory()
    config = dict(config)
    config["active_detector"] = settings["active_detector"]
    config["selector_path"] = str(path)
    config["selector_settings"] = settings
    config["selector_sha256"] = hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()
    return detector, config, weights


def detector_metadata(detector, config):
    weights = config.get("weights")
    return {
        "active_detector": config["active_detector"],
        "backend": detector.backend_name,
        "weights": str(Path(weights).resolve()) if weights is not None else None,
        "imgsz": config.get("imgsz"), "conf": config.get("conf"),
        "device": config.get("device"), "save": config.get("save", False),
        "pixel_config_path": config.get("pixel_config_path"),
        "color_detector": config.get("color_detector"),
        "selector_sha256": config["selector_sha256"],
    }
