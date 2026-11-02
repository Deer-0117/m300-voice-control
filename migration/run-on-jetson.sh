#!/usr/bin/env sh
# Run Python only. This script never starts/restarts a PSDK host process.
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$TASK_ROOT"
MIGRATION_PYTHON=${M300_PYTHON:-python3}
"$MIGRATION_PYTHON" -c 'import sys,platform; assert sys.platform=="linux" and platform.machine()=="aarch64" and sys.version_info[:2]==(3,10), "Offline dependency requires Jetson ARM64 Python 3.10"'
export PYTHONPATH="$TASK_ROOT/migration/python-deps:$TASK_ROOT/migration/text-deps${PYTHONPATH:+:$PYTHONPATH}"
export M300_ENV_FILE="${M300_ENV_FILE:-/home/uav/.config/m300-voice-control/runtime.env}"
exec "$MIGRATION_PYTHON" -m app "$@"
