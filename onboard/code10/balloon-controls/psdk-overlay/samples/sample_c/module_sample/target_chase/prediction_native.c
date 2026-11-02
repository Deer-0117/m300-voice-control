/* Offline ABI: links the exact flight predictor; no Python approximation. */
#include "target_chase.h"
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

EXPORT void *prediction_create(float width, float height, float hfov, float vfov,
                                int adaptive, float horizon, float residual, float scale)
{
    T_TargetChaseController *c = calloc(1, sizeof(*c));
    T_TargetChaseConfig config = TargetChase_DefaultConfig(width, height, hfov, vfov);
    if (c == NULL) return NULL;
    config.prediction_adaptive = adaptive != 0;
    config.prediction_horizon_s = horizon;
    config.prediction_residual_min_deg = residual;
    config.prediction_residual_box_scale = scale;
    TargetChase_Init(c, &config);
    return c;
}

EXPORT void prediction_destroy(void *handle) { free(handle); }

/* Exact independent original-pixel state, expressed in the current camera.
 * Offline scoring only; future camera pose must never enter this function. */
EXPORT int prediction_evaluate_pixel(void *handle, uint64_t now, float *output)
{
    T_TargetChaseController *c = handle;
    float h, u, v;
    if (!c || !c->pixel_samples || now < c->target.timestamp_ms ||
        now-c->target.timestamp_ms > c->config.target_timeout_ms) return 0;
    h = TargetChase_PredictionHorizon(c, now);
    u = fminf(c->config.image_width_px, fmaxf(0, c->pixel_u+c->pixel_du*h));
    v = fminf(c->config.image_height_px, fmaxf(0, c->pixel_v+c->pixel_dv*h));
    output[0] = atanf((u-c->config.cx_px)/c->config.fx_px)*57.295779513f*c->config.pixel_x_to_yaw_sign;
    output[1] = atanf((v-c->config.cy_px)/c->config.fy_px)*57.295779513f*c->config.pixel_y_to_up_sign;
    output[2] = h;
    return 1;
}

EXPORT void prediction_set_box_thresholds(void *handle, float small, float large)
{
    T_TargetChaseController *c = handle;
    if (small > 0.0f && large > small) {
        c->config.box_angle_small_deg = small;
        c->config.box_angle_large_deg = large;
    }
}

EXPORT void prediction_observe(void *handle, uint64_t timestamp, float yaw, float pitch,
                                float x1, float y1, float x2, float y2)
{
    T_TargetChaseController *c = handle;
    TargetChase_SetObservationCameraPose(c, yaw, pitch, timestamp);
    TargetChase_UpdateTargetBox(c, x1, y1, x2, y2, timestamp);
}

/* output: yaw, pitch, effective horizon, rate scale, reason, residual */
EXPORT int prediction_evaluate(void *handle, uint64_t now, float *output)
{
    T_TargetChaseController *c = handle;
    T_TargetChasePrediction prediction;
    if (!TargetChase_PredictWorld(c, now, &prediction)) return 0;
    output[0] = prediction.world_yaw_deg;
    output[1] = prediction.world_pitch_deg;
    output[2] = prediction.horizon_s;
    output[3] = prediction.rate_scale;
    output[4] = (float) prediction.reason;
    output[5] = c->prediction_residual_deg;
    return 1;
}
