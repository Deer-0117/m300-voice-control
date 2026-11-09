#!/usr/bin/env bash
# Native Linux/Jetson source tests only. Never execute a real PSDK application.
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
sdk=${1:?Usage: run_native_no_aircraft.sh SDK_ROOT OUTPUT_DIRECTORY}
output=${2:?Separate private test output directory required}
mkdir -p "$output"
output=$(realpath "$output")
core="$root/psdk-overlay/samples/sample_c/module_sample/target_chase"
module="$root/psdk-overlay/samples/sample_c/module_sample"
for check in motion_comp delayed_kf timing_alignment delayed_control; do
    gcc -std=c99 -O2 -Wall -Wextra -Werror "$core/target_chase.c" \
        "$core/target_chase_${check}_test.c" -lm -o "$output/$check"
    timeout 30 "$output/$check"
done
gcc -std=gnu99 -D_GNU_SOURCE -O2 -Wall -Wextra -Werror \
    -I"$sdk/psdk_lib/include" -I"$module" \
    "$root/tests/camera_pose_capture_test.c" -lm -o "$output/camera_pose_capture"
mkdir -p "$output/camera_pose_fixture"
timeout 30 "$output/camera_pose_capture" "$output/camera_pose_fixture"
gcc -std=gnu99 -D_GNU_SOURCE -O2 -Wall -Wextra -Werror -pthread \
    -I"$sdk/psdk_lib/include" -I"$module" \
    "$root/tests/background_telemetry_test.c" "$root/tests/background_telemetry_peer.c" \
    -lm -o "$output/background_telemetry"
mkdir -p "$output/background_telemetry_fixture" "$output/background_telemetry_thread_fixture"
timeout 30 "$output/background_telemetry" "$output/background_telemetry_fixture"
timeout 30 "$output/background_telemetry" "$output/background_telemetry_thread_fixture" --runtime
for check in delayed_kf_control performance_control; do
    gcc -std=gnu99 -D_GNU_SOURCE -DCJSON_HIDE_SYMBOLS -O2 -Wall -Wextra -Werror -pthread \
        -flto -fwhole-program -ffunction-sections -fdata-sections \
        -I"$sdk/psdk_lib/include" -I"$module" -I"$sdk/samples/sample_c/module_sample" \
        "$root/tests/${check}_test.c" "$core/target_chase.c" \
        "$sdk/samples/sample_c/module_sample/utils/cJSON.c" \
        -Wl,--gc-sections -lm -o "$output/$check"
    (cd "$output"; timeout 30 "./$check" "$output/performance_fixture.jsonl")
done
gcc -std=c99 -O2 -shared -fPIC -Wall -Wextra -Werror \
    "$core/target_chase.c" "$core/prediction_native.c" -lm -o "$output/prediction_native.so"
printf '%s\n' 'Native source/stub tests passed. No DJI libraries linked or aircraft process started.'
