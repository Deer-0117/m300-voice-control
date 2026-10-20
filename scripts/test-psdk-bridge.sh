#!/usr/bin/env sh
# Link test-only replacements, not DJI binaries. No aircraft communication.
set -eu
SDK_ROOT=$(CDPATH= cd -- "${1:?Usage: test-psdk-bridge.sh path/to/PSDK}" && pwd)
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$TASK_ROOT"
mkdir -p .build/bridge-test/ipc/events
cc -std=c11 -Wall -Wextra -Werror -Ipsdk -I"$SDK_ROOT/psdk_lib/include" \
  -I"$SDK_ROOT/samples/sample_c/module_sample/utils" \
  -c psdk/m300_runtime.c -o .build/bridge-compile-check.o
cc -std=c11 -I"$SDK_ROOT/samples/sample_c/module_sample/utils" \
  -c "$SDK_ROOT/samples/sample_c/module_sample/utils/cJSON.c" -o .build/cJSON-test.o
cc -std=c11 -Wall -Wextra -Werror -Ipsdk -I"$SDK_ROOT/psdk_lib/include" \
  -I"$SDK_ROOT/samples/sample_c/module_sample/utils" \
  tests/test_psdk_bridge.c psdk/command_guard.c .build/cJSON-test.o -lm -lpthread -o .build/test-bridge
.build/test-bridge .build/bridge-test
