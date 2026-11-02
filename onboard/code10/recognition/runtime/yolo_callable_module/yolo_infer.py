"""Callable YOLOv8 inference module that returns JSON-friendly detections."""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = ROOT.parent
LOCAL_ULTRALYTICS_SOURCE = WORKSPACE_ROOT / "official_source" / "ultralytics_official"
YOLO_CONFIG_DIR = ROOT / ".ultralytics_config"
DEFAULT_CONFIG = ROOT / "config.yaml"


def add_local_ultralytics_to_path() -> None:
    if LOCAL_ULTRALYTICS_SOURCE.is_dir():
        sys.path.insert(0, str(LOCAL_ULTRALYTICS_SOURCE))
    YOLO_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(YOLO_CONFIG_DIR))


def resolve_module_path(value: str | Path | None) -> Path | None:
    if value is None:
        return None
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def load_config(config_path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as file:
        raw = yaml.safe_load(file) or {}

    return {
        "weights": resolve_module_path(raw.get("weights", "weights/best.pt")),
        "imgsz": int(raw.get("imgsz", 640)),
        "conf": float(raw.get("conf", 0.25)),
        "device": raw.get("device"),
        "save": bool(raw.get("save", False)),
    }


def tensor_to_python(value: Any) -> Any:
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)) and len(value) == 1:
        return tensor_to_python(value[0])
    return value


def result_to_records(result: Any) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    names = getattr(result, "names", {}) or {}
    image = str(getattr(result, "path", ""))

    for box in getattr(result, "boxes", []) or []:
        class_id = int(tensor_to_python(getattr(box, "cls")))
        confidence = float(tensor_to_python(getattr(box, "conf")))
        bbox = tensor_to_python(getattr(box, "xyxy"))
        if bbox and isinstance(bbox[0], list):
            bbox = bbox[0]

        records.append(
            {
                "image": image,
                "class_id": class_id,
                "class_name": str(names.get(class_id, class_id)),
                "confidence": confidence,
                "bbox_xyxy": [float(value) for value in bbox],
            }
        )
    return records


def run_detection(
    source: Path,
    weights: Path,
    imgsz: int = 640,
    conf: float = 0.25,
    device: str | None = None,
    save: bool = False,
    project: Path | None = None,
    name: str = "detect",
) -> dict[str, Any]:
    add_local_ultralytics_to_path()
    from ultralytics import YOLO

    started = time.perf_counter()
    model = YOLO(str(weights))
    results = model.predict(
        source=str(source),
        imgsz=imgsz,
        conf=conf,
        device=device,
        save=save,
        project=str(project) if project else None,
        name=name,
        exist_ok=True,
        verbose=False,
    )
    detections = [record for result in results for record in result_to_records(result)]
    elapsed_ms = round((time.perf_counter() - started) * 1000, 3)

    return {
        "ok": True,
        "source": str(source),
        "weights": str(weights),
        "imgsz": imgsz,
        "conf": conf,
        "device": device,
        "save": save,
        "latency_ms": elapsed_ms,
        "detection_count": len(detections),
        "detections": detections,
    }


def json_error(message: str) -> int:
    print(json.dumps({"ok": False, "error": message}, ensure_ascii=False, indent=2))
    return 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run YOLOv8 inference and print JSON detections.")
    parser.add_argument("--source", type=Path, required=True, help="Image, video, or folder to detect.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="Module config YAML.")
    parser.add_argument("--weights", type=Path, default=None, help="Override model weights.")
    parser.add_argument("--imgsz", type=int, default=None, help="Override inference image size.")
    parser.add_argument("--conf", type=float, default=None, help="Override confidence threshold.")
    parser.add_argument("--device", default=None, help='Override device, for example "0" or "cpu".')
    parser.add_argument("--save", action="store_true", help="Save annotated prediction images.")
    parser.add_argument("--project", type=Path, default=ROOT / "runs", help="Output project directory when --save is used.")
    parser.add_argument("--name", default="detect", help="Output run name when --save is used.")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    config = load_config(args.config)
    source = args.source if args.source.is_absolute() else Path.cwd() / args.source
    source = source.resolve()
    if not source.exists():
        return json_error(f"source not found: {source}")

    weights = resolve_module_path(args.weights) if args.weights else config["weights"]
    weights = weights.resolve()
    if not weights.is_file():
        return json_error(f"weights not found: {weights}")

    try:
        with contextlib.redirect_stdout(io.StringIO()):
            payload = run_detection(
                source=source,
                weights=weights,
                imgsz=args.imgsz or config["imgsz"],
                conf=args.conf if args.conf is not None else config["conf"],
                device=args.device if args.device is not None else config["device"],
                save=args.save or config["save"],
                project=args.project.resolve(),
                name=args.name,
            )
    except Exception as exc:
        return json_error(f"inference failed: {exc}")

    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
