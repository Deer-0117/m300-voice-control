#!/usr/bin/env python3
"""Common service entry point. The legacy module name is kept for compatibility."""
from balloon_yolo_worker import main

if __name__ == "__main__":
    from detector_selector import ensure_script_interpreter
    ensure_script_interpreter()
    raise SystemExit(main())
