#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$TASK_ROOT"
PYTHON=${PYTHON:-python3}
"$PYTHON" -m unittest discover -s tests -v
mkdir -p .build
cc -std=c11 -Wall -Wextra -Werror -Ipsdk tests/test_command_guard.c psdk/command_guard.c -lm -o .build/test-command-guard
.build/test-command-guard
