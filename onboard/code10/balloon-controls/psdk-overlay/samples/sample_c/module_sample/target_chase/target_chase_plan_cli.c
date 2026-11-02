#include "target_chase.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static float ArgFloat(int argc, char **argv, const char *name, float fallback)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (strcmp(argv[i], name) == 0) {
            return strtof(argv[i + 1], NULL);
        }
    }
    return fallback;
}

static unsigned long long ArgU64(int argc, char **argv, const char *name, unsigned long long fallback)
{
    for (int i = 1; i + 1 < argc; ++i) {
        if (strcmp(argv[i], name) == 0) {
            return strtoull(argv[i + 1], NULL, 10);
        }
    }
    return fallback;
}

int main(int argc, char **argv)
{
    const float width = ArgFloat(argc, argv, "--width", 5472.0f);
    const float height = ArgFloat(argc, argv, "--height", 3648.0f);
    const float x1 = ArgFloat(argc, argv, "--x1", -1.0f);
    const float y1 = ArgFloat(argc, argv, "--y1", -1.0f);
    const float x2 = ArgFloat(argc, argv, "--x2", -1.0f);
    const float y2 = ArgFloat(argc, argv, "--y2", -1.0f);
    const unsigned long long now = ArgU64(argc, argv, "--now-ms", 1000ULL);
    const unsigned long long target_ts = ArgU64(argc, argv, "--target-ts-ms", now);
    const unsigned long long eval_ts = ArgU64(argc, argv, "--eval-ms", now + 10ULL);
    T_TargetChaseConfig config = TargetChase_DefaultConfig(width, height, 73.7f, 45.7f);
    const unsigned long long target_age_ms = eval_ts >= target_ts ? eval_ts - target_ts : 0ULL;
    const bool target_fresh = eval_ts >= target_ts && target_age_ms <= (unsigned long long) config.target_timeout_ms;
    T_TargetChaseController controller;
    T_TargetChaseAircraftState state = {
        .altitude_agl_m = 5.0f,
        .velocity_neu_mps = {0.0f, 0.0f, 0.0f},
        .attitude_deg = {0.0f, 0.0f, 0.0f},
        .angular_rate_radps = {0.0f, 0.0f, 0.0f},
        .in_air = true,
        .velocity_healthy = true,
        .timestamp_ms = eval_ts,
    };
    T_TargetChaseCommand command;
    bool stepOk;

    if (x1 < 0.0f || y1 < 0.0f || x2 < 0.0f || y2 < 0.0f || width <= 1.0f || height <= 1.0f) {
        fprintf(stderr, "invalid bbox or image size\n");
        return 2;
    }

    TargetChase_Init(&controller, &config);
    TargetChase_UpdateAircraftState(&controller, &state);
    TargetChase_UpdateTargetBox(&controller, x1, y2, x2, y1, (uint64_t) target_ts);
    TargetChase_SetManualTrigger(&controller, true, (uint64_t) eval_ts);
    if (target_fresh) {
        TargetChase_SetAuthority(&controller, true, (uint64_t) eval_ts);
    }
    stepOk = TargetChase_Step(&controller, (uint64_t) eval_ts, &command);

    printf("{\"ok\":%s,\"source\":\"target_chase.c\",\"mode\":%d,"
           "\"target_timestamp_ms\":%llu,\"eval_timestamp_ms\":%llu,"
           "\"target_age_ms\":%llu,\"target_fresh\":%s,"
           "\"x_mps\":%.6f,\"y_mps\":%.6f,\"z_mps\":%.6f,\"yaw_rate_degps\":%.6f,"
           "\"target_bearing_yaw_deg\":%.6f,\"target_bearing_vertical_deg\":%.6f,"
           "\"effective_yaw_deadband_deg\":%.6f,\"effective_bearing_deadband_deg\":%.6f,"
           "\"should_send\":%s,\"no_obtain_authority\":true,\"no_execute_joystick\":true}\n",
           stepOk ? "true" : "false",
           (int) TargetChase_GetMode(&controller),
           target_ts,
           eval_ts,
           target_age_ms,
           target_fresh ? "true" : "false",
           command.x_mps,
           command.y_mps,
           command.z_mps,
           command.yaw_rate_degps,
           command.target_bearing_yaw_deg,
           command.target_bearing_vertical_deg,
           command.effective_yaw_deadband_deg,
           command.effective_bearing_deadband_deg,
           command.should_send ? "true" : "false");

    return 0;
}
