#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$TASK_ROOT"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
# No device services, SDK code, keys or flight settings are modified.
