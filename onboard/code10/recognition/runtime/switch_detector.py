#!/usr/bin/env python3
"""Apply detector selection only with both SDK hosts stopped and no task gates.

Usage: sudo /usr/bin/python3 switch_detector.py pixel|yolo
Or edit detector_config.json, then run without an argument to apply it.
Use --status for a read-only report without stopping services or requiring sudo.
The existing systemd unit name is retained for deployment compatibility.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

from detector_selector import DEFAULT_CONFIG, read_selection

ROOT = Path(__file__).resolve().parent
WORKER = "balloon-yolo-worker.service"
TRACKER = "balloon-tracker-worker.service"
RUNTIME = ROOT / "detector_runtime.json"
SDK_UNITS = ("psdk-balloon.service", "psdk-flight-panel-display.service")
ACTIVE_GATES = tuple(Path("/tmp") / name for name in (
    "balloon_strike.armed", "balloon_strike.trigger", "balloon_mission.token",
    "balloon_mission.commit", "balloon_mission.ready",
))


def state(unit):
    out = subprocess.check_output(["systemctl", "show", unit, "-p", "ActiveState", "-p", "MainPID"], text=True, timeout=10)
    return dict(line.split("=", 1) for line in out.splitlines() if "=" in line)


def assert_switch_idle():
    # The maintenance switch lives inside the panel process. An environment
    # value or stale lock cannot prove its current state, so both SDK hosts must
    # be stopped. Never stop either SDK host from this helper.
    for unit in SDK_UNITS:
        current = state(unit)
        if current.get("ActiveState") != "inactive" or int(current.get("MainPID", "-1")) != 0:
            raise SystemExit(f"Stop {unit} before switching detectors; no detector was changed")
    present = [str(path) for path in ACTIVE_GATES if path.exists() or path.is_symlink()]
    if present:
        raise SystemExit("Task gates still exist; wait for task cleanup: " + ", ".join(present))


def status_report():
    report = {"selection": read_selection(),
              "services": {unit: state(unit) for unit in (*SDK_UNITS, WORKER, TRACKER)},
              "active_gates": [str(path) for path in ACTIVE_GATES if path.exists() or path.is_symlink()]}
    try:
        report["runtime"] = json.loads(RUNTIME.read_text())
    except (OSError, ValueError) as exc:
        report["runtime"] = None
        report["runtime_error"] = type(exc).__name__
    return report


def control(action, *units):
    subprocess.run(["systemctl", action, *units], check=True, timeout=45)


def atomic_config(settings):
    temporary = DEFAULT_CONFIG.with_suffix(".json.switch-tmp")
    temporary.write_text(json.dumps(settings, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, DEFAULT_CONFIG)


def wait_ready(selected, timeout=75):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        current = state(WORKER)
        try:
            ready = json.loads(RUNTIME.read_text())
            if (current["ActiveState"] == "active" and int(current["MainPID"]) == ready["pid"]
                    and ready["active_detector"] == selected):
                if selected == "pixel" and ready["neural_modules_loaded"]:
                    raise RuntimeError("Pixel worker loaded neural modules")
                return ready
        except (OSError, ValueError, KeyError):
            pass
        time.sleep(0.25)
    raise RuntimeError("Detector did not become ready; inspect systemd journal")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", choices=("yolo", "pixel"))
    parser.add_argument("--status", action="store_true", help="report configuration and process state without changing them")
    args = parser.parse_args()
    if args.status:
        if args.mode is not None:
            parser.error("--status cannot be combined with a mode")
        print(json.dumps(status_report(), ensure_ascii=False, indent=2))
        return
    if os.geteuid() != 0:
        raise SystemExit("Run with sudo to manage the detector/tracker services")
    with (ROOT / "detector_switch.lock").open("a") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        apply_selection(args.mode)


def apply_selection(mode):
    assert_switch_idle()
    desired = read_selection()
    if mode is not None:
        desired["active_detector"] = mode
    if desired["active_detector"] == "pixel":
        # Validate parameter syntax without importing neural libraries or loading a model.
        from pixel_callable_module import load_color_config
        load_color_config(ROOT / desired["pixel_config"])
    previous = read_selection()
    worker_before, tracker_before = state(WORKER), state(TRACKER)
    if any(item.get("ActiveState") not in ("active", "inactive") for item in (worker_before, tracker_before)):
        raise SystemExit("Detector/tracker state is transitional or failed; no service was changed")
    if tracker_before["ActiveState"] == "active" and worker_before["ActiveState"] != "active":
        raise SystemExit("Tracker is active without the detector; resolve the inconsistent state first")
    try:
        runtime = json.loads(RUNTIME.read_text())
        if int(worker_before["MainPID"]) == runtime["pid"]:
            previous = runtime["selector_config"]
    except (OSError, ValueError, KeyError):
        pass
    archive_parent = ROOT / "detector_switch_backups"
    archive_parent.mkdir(exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix="switch_", dir=archive_parent))
    shutil.copy2(DEFAULT_CONFIG, backup / DEFAULT_CONFIG.name)
    (backup / "previous_running_config.json").write_text(json.dumps(previous, indent=2))
    print("Switch backup:", backup, flush=True)
    assert_switch_idle()
    try:
        active_units = [unit for unit, before in ((TRACKER, tracker_before), (WORKER, worker_before))
                        if before["ActiveState"] == "active"]
        if active_units:
            control("stop", *active_units)
        if any(int(state(unit)["MainPID"]) for unit in (TRACKER, WORKER)):
            raise RuntimeError("Previous worker/tracker did not exit")
        # Do not carry observations or queued requests across the algorithm boundary.
        for name in ("latest_target.tsv", "latest_yolo_target.tsv", "latest_detection_request.tsv", "detector_runtime.json"):
            path = ROOT / name
            if path.is_symlink():
                raise RuntimeError("Unexpected state-file symlink: " + str(path))
            if path.exists():
                shutil.copy2(path, backup / name)
                path.unlink()
        atomic_config(desired)
        ready = {"active_detector": desired["active_detector"], "running": False,
                 "note": "Configuration saved; previously inactive services remain stopped"}
        if worker_before["ActiveState"] == "active":
            control("start", WORKER)
            ready = wait_ready(desired["active_detector"])
        if tracker_before["ActiveState"] == "active":
            control("start", TRACKER)
            if state(TRACKER)["ActiveState"] != "active":
                raise RuntimeError("Tracker failed to restart")
        print(json.dumps(ready, ensure_ascii=False, indent=2))
    except Exception:
        if active_units:
            control("stop", *active_units)
        atomic_config(previous)
        if worker_before["ActiveState"] == "active":
            control("start", WORKER)
            wait_ready(previous["active_detector"])
        if tracker_before["ActiveState"] == "active":
            control("start", TRACKER)
        raise


if __name__ == "__main__":
    main()
