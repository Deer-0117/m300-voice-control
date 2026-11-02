#!/usr/bin/env sh
# Tests with SDK replacements only; do not link to a live aircraft.
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$TASK_ROOT"
SDK_ROOT=${1:?Pass the actual SDK root}
export UAV_ENABLE_FLIGHT=0
export PYTHONPATH="$TASK_ROOT/migration/python-deps:$TASK_ROOT/migration/text-deps${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'import sys,platform; assert sys.platform=="linux" and platform.machine()=="aarch64" and sys.version_info[:2]==(3,10)'
PYTHON=python3 sh scripts/test.sh
sh scripts/test-psdk-bridge.sh "$SDK_ROOT"
printf '%s\n' 'Software/stub tests passed. This is NOT real PSDK host linking or flight validation.'
