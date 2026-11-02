#include "target_chase.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define TARGET_CHASE_PI 3.14159265358979323846f
#define TARGET_CHASE_DEG_TO_RAD (TARGET_CHASE_PI / 180.0f)
#define TARGET_CHASE_RAD_TO_DEG (180.0f / TARGET_CHASE_PI)
#define TARGET_CHASE_NEAR_LOW_LATENCY_MAX_MS 260ULL
#define TARGET_CHASE_NEAR_HIGH_RATE_BOX_ANGLE_DEG 5.0f
#define TARGET_CHASE_NEAR_HIGH_RATE_YAW_GAIN 1.35f
#define TARGET_CHASE_MAX_DYNAMIC_DEADBAND_DEG 5.0f

#define TARGET_CHASE_DEFAULT_IMAGE_WIDTH_PX 5472.0f
#define TARGET_CHASE_DEFAULT_IMAGE_HEIGHT_PX 3078.0f
#define TARGET_CHASE_DEFAULT_HORIZONTAL_FOV_DEG 73.7f
#define TARGET_CHASE_DEFAULT_VERTICAL_FOV_DEG 45.7f

static float TargetChase_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static bool TargetChase_IsFinite(float value)
{
    return isfinite(value) != 0;
}

static float TargetChase_Min(float a, float b)
{
    return a < b ? a : b;
}

static float TargetChase_Max(float a, float b)
{
    return a > b ? a : b;
}

static bool TargetChase_TargetFresh(const T_TargetChaseController *controller, uint64_t now_ms)
{
    if (controller == NULL || !controller->target.valid || controller->target.timestamp_ms == 0 || controller->target.timestamp_ms > now_ms) {
        return false;
    }

    return (now_ms - controller->target.timestamp_ms) <= (uint64_t) controller->config.target_timeout_ms;
}

static void TargetChase_ClampHorizontal(float *x_mps, float *y_mps, float max_speed_mps)
{
    const float speed = sqrtf((*x_mps * *x_mps) + (*y_mps * *y_mps));

    if (speed > max_speed_mps && speed > 0.001f) {
        const float scale = max_speed_mps / speed;
        *x_mps *= scale;
        *y_mps *= scale;
    }
}

static float TargetChase_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

static void TargetChase_BodyVelocityFromNeu(const T_TargetChaseAircraftState *aircraft,
                                            float *velocity_forward_mps,
                                            float *velocity_right_mps)
{
    const float yaw_rad = aircraft->attitude_deg[2] * TARGET_CHASE_DEG_TO_RAD;

    *velocity_forward_mps = cosf(yaw_rad) * aircraft->velocity_neu_mps[0] +
                            sinf(yaw_rad) * aircraft->velocity_neu_mps[1];
    *velocity_right_mps = -sinf(yaw_rad) * aircraft->velocity_neu_mps[0] +
                          cosf(yaw_rad) * aircraft->velocity_neu_mps[1];

    if (!aircraft->velocity_healthy) {
        *velocity_forward_mps = 0.0f;
        *velocity_right_mps = 0.0f;
    }
}

static void TargetChase_SetZeroCommand(T_TargetChaseCommand *command, bool should_send)
{
    memset(command, 0, sizeof(*command));
    command->should_send = should_send;
}

static float TargetChase_TargetBoxHalfDiagonalBearingDeg(const T_TargetChaseController *controller)
{
    const T_TargetChaseConfig *config = &controller->config;
    const T_TargetChaseTargetPixel *target = &controller->target;
    float box_width_bearing_deg;
    float box_height_bearing_deg;

    if (!target->has_box || target->box_width_px <= 0.0f || target->box_height_px <= 0.0f ||
        config->fx_px <= 1.0f || config->fy_px <= 1.0f) {
        return 0.0f;
    }

    box_width_bearing_deg = 2.0f * atanf((target->box_width_px * 0.5f) / config->fx_px) *
                            TARGET_CHASE_RAD_TO_DEG;
    box_height_bearing_deg = 2.0f * atanf((target->box_height_px * 0.5f) / config->fy_px) *
                             TARGET_CHASE_RAD_TO_DEG;

    return 0.5f * sqrtf((box_width_bearing_deg * box_width_bearing_deg) +
                        (box_height_bearing_deg * box_height_bearing_deg));
}

float TargetChase_PredictionHorizon(const T_TargetChaseController *controller,
                                    uint64_t now_ms)
{
    if (controller == NULL || !TargetChase_TargetFresh(controller, now_ms)) return 0.0f;
    return TargetChase_Clamp((float) (now_ms - controller->target.timestamp_ms) / 1000.0f +
                             controller->config.prediction_horizon_s, 0.0f, 0.5f);
}

bool TargetChase_PredictWorld(const T_TargetChaseController *controller,
                              uint64_t now_ms, T_TargetChasePrediction *prediction)
{
    float age_s;
    float extra_s;
    float near_ratio = 0.0f;
    float base_horizon_s;
    if (prediction == NULL) return false;
    memset(prediction, 0, sizeof(*prediction));
    prediction->reason = TARGET_PREDICTION_INVALID_TIME;
    if (controller == NULL || !TargetChase_TargetFresh(controller, now_ms)) return false;
    prediction->reason = TARGET_PREDICTION_NO_POSE;
    if (!controller->motion_world_valid || !controller->motion_estimate_initialized ||
        controller->motion_estimate_timestamp_ms != controller->target.timestamp_ms) return false;
    prediction->rate_scale = 1.0f;
    prediction->reason = TARGET_PREDICTION_FIXED;
    age_s = (float) (now_ms - controller->target.timestamp_ms) / 1000.0f;
    extra_s = TargetChase_Max(controller->config.prediction_horizon_s, 0.0f);
    if (controller->config.prediction_adaptive || controller->config.prediction_mode == 2) {
        /* Angular size is only a proximity proxy, not a metric range estimate.
         * Preserve elapsed-time compensation; reduce only extra future lead
         * as the target grows. All extrapolation still obeys confidence limits. */
        float box_deg = TargetChase_Max(controller->filtered_box_angle_deg,
                                       TargetChase_TargetBoxHalfDiagonalBearingDeg(controller));
        float small = controller->config.box_angle_small_deg;
        float large = controller->config.box_angle_large_deg;
        if (controller->target.has_box && large > small && small > 0.0f) {
            near_ratio = TargetChase_Clamp((box_deg - small) / (large - small), 0.0f, 1.0f);
        }
        extra_s *= 1.0f - near_ratio;
        float nx = fabsf(controller->target.u_px - controller->config.cx_px) /
                   TargetChase_Max(controller->config.image_width_px * 0.5f, 1.0f);
        float ny = fabsf(controller->target.v_px - controller->config.cy_px) /
                   TargetChase_Max(controller->config.image_height_px * 0.5f, 1.0f);
        prediction->rate_scale = TargetChase_Clamp(controller->prediction_quality, 0.0f, 1.0f);
        prediction->reason = prediction->rate_scale < 1.0f
            ? TARGET_PREDICTION_RECOVERING : TARGET_PREDICTION_NORMAL;
        if (age_s > 0.26f) {
            prediction->rate_scale *= TargetChase_Clamp((0.5f - age_s) / 0.24f, 0.0f, 1.0f);
            prediction->reason = TARGET_PREDICTION_AGED;
        }
        if (nx > 0.9f || ny > 0.9f) {
            prediction->rate_scale = 0.0f;
            prediction->reason = TARGET_PREDICTION_EDGE;
        }
    }
    base_horizon_s = TargetChase_Clamp(age_s + extra_s, 0.0f, 0.5f);
    prediction->horizon_s = base_horizon_s * prediction->rate_scale;
    if ((controller->config.prediction_adaptive || controller->config.prediction_mode == 2) && near_ratio > 0.0f) {
        /* Bound the combined angular displacement, rather than independently
         * allowing large yaw and pitch leads. 2 degrees is a candidate near
         * limit requiring flight calibration, not a guaranteed contact radius. */
        float speed_degps = hypotf(controller->filtered_world_yaw_rate_degps,
                                  controller->filtered_world_pitch_rate_degps);
        float max_lead_deg = 15.0f * (1.0f - near_ratio) + 2.0f * near_ratio;
        if (speed_degps * prediction->horizon_s > max_lead_deg) {
            prediction->horizon_s = max_lead_deg / speed_degps;
            prediction->rate_scale = base_horizon_s > 0.0f
                ? prediction->horizon_s / base_horizon_s : 0.0f;
            prediction->reason = TARGET_PREDICTION_NEAR_LIMIT;
        }
    }
    prediction->age_horizon_s = age_s + extra_s > 0.0f
        ? prediction->horizon_s * age_s / (age_s + extra_s) : 0.0f;
    prediction->extra_horizon_s = prediction->horizon_s - prediction->age_horizon_s;
    prediction->world_yaw_deg = TargetChase_Wrap180(controller->filtered_world_yaw_deg +
        controller->filtered_world_yaw_rate_degps * prediction->horizon_s);
    prediction->world_pitch_deg = controller->filtered_world_pitch_deg +
        controller->filtered_world_pitch_rate_degps * prediction->horizon_s;
    return true;
}

bool TargetChase_LimitForwardForVertical(const T_TargetChaseConfig *config,
                                        T_TargetChaseCommand *command)
{
    float slope;
    float previous;
    if (config == NULL || command == NULL ||
        config->geometry != TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY ||
        command->x_mps <= 0.0f ||
        fabsf(command->target_bearing_vertical_deg) <= config->vertical_bearing_deadband_deg) return false;
    previous = command->x_mps;
    slope = tanf(TargetChase_Clamp(command->target_bearing_vertical_deg, -89.0f, 89.0f) * TARGET_CHASE_DEG_TO_RAD);
    if (!isfinite(slope) || !isfinite(command->z_mps) || fabsf(slope) < 0.001f) {
        command->x_mps = 0.0f;
    } else {
        command->x_mps = TargetChase_Min(previous, TargetChase_Max(0.0f, command->z_mps / slope));
    }
    return fabsf(previous - command->x_mps) > 0.0001f;
}

/* Final forward ceiling; never modifies yaw, so centering can continue.
 * Angular size is a proximity proxy, not a range measurement. Candidate gates
 * tighten from 75 to 25 degrees between 3 and 6 degrees box half diagonal.
 * Apply after every speed floor and again after command slew/repeat handling. */
bool TargetChase_LimitForwardForAlignment(const T_TargetChaseController *controller,
                                         T_TargetChaseCommand *command)
{
    float box, near_ratio, bearing, limit, factor, cap, old_x, rate;
    if (controller == NULL || command == NULL ||
        controller->config.geometry != TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY ||
        !controller->target.valid || command->x_mps <= 0.0f) return false;
    old_x = command->x_mps;
    box = TargetChase_Max(controller->filtered_box_angle_deg,
                         TargetChase_TargetBoxHalfDiagonalBearingDeg(controller));
    bearing = command->target_bearing_yaw_deg;
    if (!isfinite(box) || !isfinite(bearing) || !isfinite(old_x)) {
        command->x_mps = 0.0f;
        return true;
    }
    near_ratio = TargetChase_Clamp((box - 3.0f) / 3.0f, 0.0f, 1.0f);
    if (near_ratio <= 0.0f) return false;
    limit = 75.0f - 50.0f * near_ratio;
    factor = TargetChase_Clamp((limit - fabsf(bearing)) / (limit - 8.0f), 0.0f, 1.0f);
    /* A moving line of sight is not necessarily balloon translation. If it
     * keeps escaping faster than our yaw budget, reduce closing before it
     * reaches the image edge. Ignore inward motion and small centered errors. */
    rate = controller->config.prediction_mode == 0 ? controller->filtered_world_yaw_rate_degps : controller->hybrid_yaw_rate;
    if ((controller->motion_world_valid || controller->config.prediction_mode != 0) && fabsf(bearing) > 8.0f &&
        isfinite(rate) && rate * bearing > 0.0f) {
        float budget = TargetChase_Max(fabsf(controller->config.max_yaw_rate_degps), 0.1f);
        float rate_factor = TargetChase_Clamp((2.0f * budget - fabsf(rate)) / budget, 0.0f, 1.0f);
        factor = TargetChase_Min(factor, 1.0f - near_ratio * (1.0f - rate_factor));
    }
    /* A smoothed yaw command can still point away after target reversal. */
    if (fabsf(bearing) > 8.0f && command->yaw_rate_degps * bearing < -0.01f)
        factor = TargetChase_Min(factor, 1.0f - near_ratio);
    cap = TargetChase_Max(0.0f, controller->config.approach_speed_mps) * factor;
    command->x_mps = TargetChase_Min(old_x, cap);
    if (command->x_mps < old_x && isfinite(command->z_mps))
        command->z_mps *= command->x_mps / old_x;
    return command->x_mps < old_x;
}

/* Independent image-space estimator; never reuse world-derived pixel rates. */
static void TargetChase_UpdatePixelEstimate(T_TargetChaseController *c)
{
    uint64_t t = c->target.timestamp_ms;
    float dt, ru, rv, alpha, beta, pu, pv;
    if (!c->target.valid || t == 0 || t <= c->pixel_timestamp_ms) return;
    if (!c->pixel_timestamp_ms) {
        c->pixel_u = c->target.u_px; c->pixel_v = c->target.v_px;
        c->pixel_du = c->pixel_dv = 0; c->pixel_samples = 1;
    } else {
        /* Exact original pixel alpha-beta recurrence, including long gaps and
         * large residuals. Keep the historical baseline free of new gates. */
        dt = TargetChase_Clamp((float)(t-c->pixel_timestamp_ms)/1000.0f,0.02f,0.5f);
        alpha = TargetChase_Clamp(c->config.motion_filter_alpha,0.05f,1.0f);
        beta = TargetChase_Clamp(c->config.motion_filter_beta,0.001f,1.0f);
        pu = c->pixel_u+c->pixel_du*dt; pv = c->pixel_v+c->pixel_dv*dt;
        ru = c->target.u_px-pu; rv = c->target.v_px-pv;
        c->pixel_u = pu+alpha*ru; c->pixel_v = pv+alpha*rv;
        c->pixel_du = TargetChase_Clamp(c->pixel_du+beta*ru/dt,-2*c->config.image_width_px,2*c->config.image_width_px);
        c->pixel_dv = TargetChase_Clamp(c->pixel_dv+beta*rv/dt,-2*c->config.image_height_px,2*c->config.image_height_px);
        if (c->pixel_samples < 3) c->pixel_samples++;
    }
    c->pixel_timestamp_ms = t;
}

void TargetChase_ResetDelayedPrediction(T_TargetChaseController *c, E_TargetChaseKfReason reason)
{
    if (!c) return;
    TargetDelayedKf_Reset(&c->delayed_kf);
    c->kf_active = false;
    c->kf_reason = reason;
    c->kf_measurement_ms = 0;
    c->kf_variance_deg2 = 0;
}

bool TargetChase_ObserveAlignedBearing(T_TargetChaseController *c,
    float world_yaw_deg, float world_pitch_deg, uint64_t measurement_ms, uint64_t received_ms)
{
    bool accepted;
    if (!c || c->config.prediction_mode != 3 ||
        c->config.geometry != TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY ||
        measurement_ms <= c->kf_epoch_ms || received_ms > c->kf_valid_until_ms) return false;
    if (c->kf_measurement_ms && measurement_ms > c->kf_measurement_ms + 500)
        TargetChase_ResetDelayedPrediction(c, TARGET_KF_STALE);
    accepted = TargetDelayedKf_Observe(&c->delayed_kf, measurement_ms, received_ms,
                                      world_yaw_deg, world_pitch_deg);
    c->kf_reason = accepted ? TARGET_KF_WARMUP : TARGET_KF_REJECTED;
    c->kf_measurement_ms = c->delayed_kf.has_posterior ? c->delayed_kf.posterior_ms : 0;
    return accepted;
}

bool TargetChase_PredictHybrid(T_TargetChaseController *c, uint64_t now_ms,
    float camera_world_yaw, float camera_world_pitch,
    float *yaw, float *pitch, float *yaw_rate, float *pitch_rate)
{
    float age, h, px, py, weight = 0.0f, near, scale;
    T_TargetChasePrediction world;
    if (!c || !yaw || !pitch || !yaw_rate || !pitch_rate) return false;
    c->hybrid_weight = c->hybrid_horizon_s = c->hybrid_yaw_rate = 0;
    c->kf_active = false;
    if (c->config.prediction_mode == 0 || !TargetChase_TargetFresh(c, now_ms) ||
        c->config.fx_px <= 1 || c->config.fy_px <= 1) return false;
    age = (now_ms - c->target.timestamp_ms) * 0.001f;
    near = TargetChase_Clamp((TargetChase_TargetBoxHalfDiagonalBearingDeg(c)-4.0f)/4.0f,0,1);
    /* Pixel baseline and the far component both retain age + lead, max 500ms.
     * Freshness affects only admission of the NEAR world component. */
    scale = TargetChase_Clamp((0.25f-age)/0.10f,0,1);
    h = TargetChase_PredictionHorizon(c,now_ms);
    px = TargetChase_Clamp(c->pixel_u+c->pixel_du*h,0,c->config.image_width_px);
    py = TargetChase_Clamp(c->pixel_v+c->pixel_dv*h,0,c->config.image_height_px);
    *yaw = atanf((px-c->config.cx_px)/c->config.fx_px)*TARGET_CHASE_RAD_TO_DEG*c->config.pixel_x_to_yaw_sign;
    *pitch = atanf((py-c->config.cy_px)/c->config.fy_px)*TARGET_CHASE_RAD_TO_DEG*c->config.pixel_y_to_up_sign;
    *yaw_rate = c->pixel_du/c->config.fx_px*TARGET_CHASE_RAD_TO_DEG*c->config.pixel_x_to_yaw_sign;
    *pitch_rate = c->pixel_dv/c->config.fy_px*TARGET_CHASE_RAD_TO_DEG*c->config.pixel_y_to_up_sign;
    if (c->config.prediction_mode == 3) {
        T_TargetDelayedKfPrediction k;
        float kage = now_ms >= c->kf_measurement_ms ? (now_ms-c->kf_measurement_ms)*0.001f : 1.0f;
        /* A measured frame age is compensated once. 0.15s remains additional
         * actuator lead, bounded by the original total 0.5s extrapolation. */
        float lead = TargetChase_Clamp(c->config.prediction_horizon_s, 0, 0.5f);
        lead *= 1.0f - near;
        lead = TargetChase_Min(lead, TargetChase_Max(0, 0.5f-kage));
        if (now_ms > c->kf_valid_until_ms) {
            TargetChase_ResetDelayedPrediction(c, TARGET_KF_NO_CALIBRATION);
        } else if (!c->kf_camera_pose_valid || c->kf_camera_pose_ms > now_ms ||
                   now_ms-c->kf_camera_pose_ms > 100) {
            c->kf_reason = TARGET_KF_NO_POSE;
        } else if (TargetDelayedKf_Predict(&c->delayed_kf, now_ms, lead, 500, &k)) {
            c->kf_variance_deg2 = TargetChase_Max(k.yaw_variance_deg2, k.pitch_variance_deg2);
            if (k.accepted_samples < 3) c->kf_reason = TARGET_KF_WARMUP;
            else if (c->kf_variance_deg2 > 4.0f || fabsf(k.yaw_rate_degps) > 30 ||
                     fabsf(k.pitch_rate_degps) > 30) c->kf_reason = TARGET_KF_UNCERTAIN;
            else {
                *yaw = TargetChase_Wrap180(k.yaw_deg-c->kf_camera_yaw_deg);
                *pitch = k.pitch_deg-c->kf_camera_pitch_deg;
                *yaw_rate = k.yaw_rate_degps;
                *pitch_rate = k.pitch_rate_degps;
                weight = 1; h = k.age_ms*0.001f+lead;
                c->kf_active = true;
                if (c->kf_reason != TARGET_KF_REJECTED) c->kf_reason = TARGET_KF_ACTIVE;
            }
        } else if (c->kf_measurement_ms) c->kf_reason = TARGET_KF_STALE;
    }
    if (c->config.prediction_mode == 2 && age < 0.25f && c->pixel_samples >= 3 &&
        TargetChase_PredictWorld(c, now_ms, &world)) {
        float wh = TargetChase_Min(world.horizon_s,0.25f)*scale;
        float wy = TargetChase_Wrap180(c->filtered_world_yaw_deg + c->filtered_world_yaw_rate_degps*wh-camera_world_yaw);
        float wp = c->filtered_world_pitch_deg+c->filtered_world_pitch_rate_degps*wh-camera_world_pitch;
        weight = near*TargetChase_Clamp(TargetChase_Min(c->prediction_quality,world.rate_scale),0,1)*scale;
        *yaw = TargetChase_Wrap180(*yaw + weight*TargetChase_Wrap180(wy-*yaw));
        *pitch += weight*(wp-*pitch);
        *yaw_rate += weight*(c->filtered_world_yaw_rate_degps*world.rate_scale-*yaw_rate);
        *pitch_rate += weight*(c->filtered_world_pitch_rate_degps*world.rate_scale-*pitch_rate);
    }
    c->hybrid_weight = weight; c->hybrid_horizon_s = h; c->hybrid_yaw_rate = *yaw_rate;
    return true;
}

static void TargetChase_UpdateMotionEstimate(T_TargetChaseController *controller)
{
    const T_TargetChaseConfig *config = &controller->config;
    const uint64_t timestamp_ms = controller->target.timestamp_ms;
    const float measured_box_angle_deg =
        TargetChase_TargetBoxHalfDiagonalBearingDeg(controller);

    TargetChase_UpdatePixelEstimate(controller);
    if (!controller->target.valid) {
        controller->observation_pose_valid = false;
        return;
    }

    if (controller->observation_pose_valid &&
        controller->observation_pose_timestamp_ms == timestamp_ms &&
        config->geometry == TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY) {
        float measured_world_yaw_deg = TargetChase_Wrap180(
            controller->observation_camera_yaw_deg +
            atanf((controller->target.u_px - config->cx_px) /
                  TargetChase_Max(config->fx_px, 1.0f)) *
                TARGET_CHASE_RAD_TO_DEG * config->pixel_x_to_yaw_sign);
        float measured_world_pitch_deg =
            controller->observation_camera_pitch_deg +
            atanf((controller->target.v_px - config->cy_px) /
                  TargetChase_Max(config->fy_px, 1.0f)) *
                TARGET_CHASE_RAD_TO_DEG * config->pixel_y_to_up_sign;
        controller->observation_pose_valid = false;
        if (!controller->motion_estimate_initialized ||
            !controller->motion_world_valid ||
            timestamp_ms > controller->motion_estimate_timestamp_ms + 500ULL) {
            controller->filtered_world_yaw_deg = measured_world_yaw_deg;
            controller->filtered_world_pitch_deg = measured_world_pitch_deg;
            controller->filtered_world_yaw_rate_degps = 0.0f;
            controller->filtered_world_pitch_rate_degps = 0.0f;
            controller->filtered_box_angle_deg = measured_box_angle_deg;
            controller->filtered_box_angle_rate_degps = 0.0f;
            controller->prediction_quality = 0.0f;
            controller->prediction_recovery_frames = 2;
            controller->prediction_residual_deg = 0.0f;
        } else if (timestamp_ms > controller->motion_estimate_timestamp_ms) {
            float dt_s = (float) (timestamp_ms -
                                  controller->motion_estimate_timestamp_ms) / 1000.0f;
            float alpha = TargetChase_Clamp(config->motion_filter_alpha, 0.05f, 1.0f);
            float beta = TargetChase_Clamp(config->motion_filter_beta, 0.001f, 1.0f);
            float predicted_yaw_deg;
            float predicted_pitch_deg;
            float residual_yaw_deg;
            float residual_pitch_deg;
            dt_s = TargetChase_Clamp(dt_s, 0.02f, 0.5f);
            predicted_yaw_deg = TargetChase_Wrap180(
                controller->filtered_world_yaw_deg +
                controller->filtered_world_yaw_rate_degps * dt_s);
            predicted_pitch_deg = controller->filtered_world_pitch_deg +
                                  controller->filtered_world_pitch_rate_degps * dt_s;
            residual_yaw_deg = TargetChase_Wrap180(measured_world_yaw_deg -
                                                   predicted_yaw_deg);
            residual_pitch_deg = measured_world_pitch_deg - predicted_pitch_deg;
            controller->prediction_residual_deg = hypotf(residual_yaw_deg, residual_pitch_deg);
            if ((config->prediction_adaptive || config->prediction_mode == 2) && controller->prediction_residual_deg >
                TargetChase_Max(config->prediction_residual_min_deg,
                    measured_box_angle_deg * config->prediction_residual_box_scale)) {
                /* One anomalous frame corrects position but does not establish velocity. */
                controller->filtered_world_yaw_deg = measured_world_yaw_deg;
                controller->filtered_world_pitch_deg = measured_world_pitch_deg;
                controller->filtered_world_yaw_rate_degps = 0.0f;
                controller->filtered_world_pitch_rate_degps = 0.0f;
                controller->filtered_box_angle_deg = measured_box_angle_deg;
                controller->filtered_box_angle_rate_degps = 0.0f;
                controller->prediction_quality = 0.0f;
                controller->prediction_recovery_frames = 2;
            } else {
                controller->filtered_world_yaw_deg = TargetChase_Wrap180(
                    predicted_yaw_deg + alpha * residual_yaw_deg);
                controller->filtered_world_pitch_deg =
                    predicted_pitch_deg + alpha * residual_pitch_deg;
                controller->filtered_world_yaw_rate_degps = TargetChase_Clamp(
                    controller->filtered_world_yaw_rate_degps +
                        beta * residual_yaw_deg / dt_s, -30.0f, 30.0f);
                controller->filtered_world_pitch_rate_degps = TargetChase_Clamp(
                    controller->filtered_world_pitch_rate_degps +
                        beta * residual_pitch_deg / dt_s, -30.0f, 30.0f);
                if (controller->target.has_box && measured_box_angle_deg > 0.0f) {
                    float predicted_box_angle_deg =
                        controller->filtered_box_angle_deg +
                        controller->filtered_box_angle_rate_degps * dt_s;
                    float residual_box_angle_deg =
                        measured_box_angle_deg - predicted_box_angle_deg;
                    controller->filtered_box_angle_deg =
                        predicted_box_angle_deg + alpha * residual_box_angle_deg;
                    controller->filtered_box_angle_rate_degps = TargetChase_Clamp(
                        controller->filtered_box_angle_rate_degps +
                            beta * residual_box_angle_deg / dt_s, -60.0f, 60.0f);
                }
                if (controller->prediction_recovery_frames > 0) controller->prediction_recovery_frames--;
                controller->prediction_quality = 1.0f - 0.5f * controller->prediction_recovery_frames;
            }
        } else {
            return;
        }
        controller->motion_estimate_initialized = true;
        controller->motion_world_valid = true;
        controller->motion_estimate_timestamp_ms = timestamp_ms;
        controller->filtered_u_px = controller->target.u_px;
        controller->filtered_v_px = controller->target.v_px;
        /* These rates now describe inertial target motion for gimbal feed-forward. */
        controller->filtered_u_rate_pxps =
            controller->filtered_world_yaw_rate_degps * TARGET_CHASE_DEG_TO_RAD *
            config->fx_px * config->pixel_x_to_yaw_sign;
        controller->filtered_v_rate_pxps =
            controller->filtered_world_pitch_rate_degps * TARGET_CHASE_DEG_TO_RAD *
            config->fy_px * config->pixel_y_to_up_sign;
        return;
    }
    controller->observation_pose_valid = false;
    if (config->geometry == TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY) {
        /* Without a frame-time camera pose, image flow is ambiguous. */
        controller->motion_world_valid = false;
        controller->prediction_quality = 0.0f;
        controller->motion_estimate_initialized = true;
        controller->motion_estimate_timestamp_ms = timestamp_ms;
        controller->filtered_u_px = controller->target.u_px;
        controller->filtered_v_px = controller->target.v_px;
        controller->filtered_u_rate_pxps = 0.0f;
        controller->filtered_v_rate_pxps = 0.0f;
        controller->filtered_box_angle_deg = measured_box_angle_deg;
        controller->filtered_box_angle_rate_degps = 0.0f;
        return;
    }
    /* Never carry an inertial rate into a camera-relative observation. */
    if (controller->motion_world_valid) {
        controller->motion_estimate_initialized = false;
        controller->motion_world_valid = false;
        controller->filtered_u_rate_pxps = 0.0f;
        controller->filtered_v_rate_pxps = 0.0f;
    }

    if (!controller->motion_estimate_initialized) {
        controller->filtered_u_px = controller->target.u_px;
        controller->filtered_v_px = controller->target.v_px;
        controller->filtered_box_angle_deg = measured_box_angle_deg;
        controller->motion_estimate_timestamp_ms = timestamp_ms;
        controller->motion_estimate_initialized = true;
        return;
    }
    if (timestamp_ms <= controller->motion_estimate_timestamp_ms) {
        return;
    }

    {
        float dt_s = (float) (timestamp_ms - controller->motion_estimate_timestamp_ms) /
                     1000.0f;
        float alpha = TargetChase_Clamp(config->motion_filter_alpha, 0.05f, 1.0f);
        float beta = TargetChase_Clamp(config->motion_filter_beta, 0.001f, 1.0f);
        float predicted_u_px;
        float predicted_v_px;
        float residual_u_px;
        float residual_v_px;

        dt_s = TargetChase_Clamp(dt_s, 0.02f, 0.5f);
        predicted_u_px = controller->filtered_u_px +
                         controller->filtered_u_rate_pxps * dt_s;
        predicted_v_px = controller->filtered_v_px +
                         controller->filtered_v_rate_pxps * dt_s;
        residual_u_px = controller->target.u_px - predicted_u_px;
        residual_v_px = controller->target.v_px - predicted_v_px;
        controller->filtered_u_px = predicted_u_px + alpha * residual_u_px;
        controller->filtered_v_px = predicted_v_px + alpha * residual_v_px;
        controller->filtered_u_rate_pxps += beta * residual_u_px / dt_s;
        controller->filtered_v_rate_pxps += beta * residual_v_px / dt_s;
        controller->filtered_u_rate_pxps = TargetChase_Clamp(
            controller->filtered_u_rate_pxps,
            -2.0f * config->image_width_px,
            2.0f * config->image_width_px);
        controller->filtered_v_rate_pxps = TargetChase_Clamp(
            controller->filtered_v_rate_pxps,
            -2.0f * config->image_height_px,
            2.0f * config->image_height_px);

        if (controller->target.has_box && measured_box_angle_deg > 0.0f) {
            float predicted_box_angle_deg =
                controller->filtered_box_angle_deg +
                controller->filtered_box_angle_rate_degps * dt_s;
            float residual_box_angle_deg =
                measured_box_angle_deg - predicted_box_angle_deg;
            controller->filtered_box_angle_deg =
                predicted_box_angle_deg + alpha * residual_box_angle_deg;
            controller->filtered_box_angle_rate_degps +=
                beta * residual_box_angle_deg / dt_s;
            controller->filtered_box_angle_rate_degps = TargetChase_Clamp(
                controller->filtered_box_angle_rate_degps,
                -60.0f,
                60.0f);
        }
        controller->motion_estimate_timestamp_ms = timestamp_ms;
    }
}

static void TargetChase_RunAirborneBearingOnly(T_TargetChaseController *controller,
                                               float normalized_x,
                                               float normalized_y,
                                               uint64_t now_ms,
                                               T_TargetChaseCommand *command)
{
    const T_TargetChaseConfig *config = &controller->config;
    const T_TargetChaseAircraftState *aircraft = &controller->aircraft;
    float bearing_error_deg;
    float dynamic_deadband_deg;
    float vertical_bearing_for_tangent_deg;
    float corrected_boresight_yaw_deg;
    float image_yaw_deg;
    float image_yaw_rate_degps;
    float box_angle_deg;
    float desired_speed_mps;
    float current_speed_mps;
    float best_cost = INFINITY;
    float best_speed_mps = 0.0f;
    float best_yaw_rate_degps = 0.0f;
    float planning_max_yaw_rate_degps;
    float accel_candidates[3];
    float yaw_candidates[5];
    size_t accel_count = 3;
    size_t yaw_count;
    size_t accel_index;
    size_t yaw_index;

    corrected_boresight_yaw_deg = TargetChase_Wrap180(
        controller->camera_boresight_yaw_deg - config->gimbal_mount_yaw_offset_deg);
    image_yaw_deg = atanf(normalized_x) * TARGET_CHASE_RAD_TO_DEG *
                    config->pixel_x_to_yaw_sign;
    image_yaw_rate_degps =
        controller->filtered_u_rate_pxps / TargetChase_Max(config->fx_px, 1.0f) *
        TARGET_CHASE_RAD_TO_DEG * config->pixel_x_to_yaw_sign;
    image_yaw_rate_degps = TargetChase_Clamp(image_yaw_rate_degps,
                                             -30.0f,
                                             30.0f);
    command->target_bearing_yaw_deg = TargetChase_Wrap180(
        corrected_boresight_yaw_deg + image_yaw_deg);
    command->target_bearing_vertical_deg = controller->camera_boresight_vertical_deg +
                                           atanf(normalized_y) * TARGET_CHASE_RAD_TO_DEG *
                                               config->pixel_y_to_up_sign;
    if (config->prediction_mode == 0 && controller->motion_world_valid) {
        T_TargetChasePrediction prediction;
        if (TargetChase_PredictWorld(controller, now_ms, &prediction)) {
            command->target_bearing_yaw_deg = TargetChase_Wrap180(prediction.world_yaw_deg -
                    aircraft->attitude_deg[2] - config->gimbal_mount_yaw_offset_deg);
            command->target_bearing_vertical_deg = prediction.world_pitch_deg - aircraft->attitude_deg[1];
            image_yaw_deg = TargetChase_Wrap180(
                command->target_bearing_yaw_deg - corrected_boresight_yaw_deg);
            image_yaw_rate_degps = controller->filtered_world_yaw_rate_degps * prediction.rate_scale;
        }
    }

    if (config->prediction_mode != 0) {
        float hy, hp, hyr, hpr;
        if (TargetChase_PredictHybrid(controller, now_ms,
            aircraft->attitude_deg[2] + controller->camera_boresight_yaw_deg,
            aircraft->attitude_deg[1] + controller->camera_boresight_vertical_deg,
            &hy, &hp, &hyr, &hpr)) {
            image_yaw_deg = hy; image_yaw_rate_degps = TargetChase_Clamp(hyr, -30.0f, 30.0f);
            if (controller->kf_active) {
                corrected_boresight_yaw_deg = TargetChase_Wrap180(
                    controller->kf_camera_yaw_deg-aircraft->attitude_deg[2]-config->gimbal_mount_yaw_offset_deg);
            }
            command->target_bearing_yaw_deg = TargetChase_Wrap180(corrected_boresight_yaw_deg + hy);
            command->target_bearing_vertical_deg = (controller->kf_active ?
                controller->kf_camera_pitch_deg-aircraft->attitude_deg[1] :
                controller->camera_boresight_vertical_deg) + hp;
        }
    }

    bearing_error_deg = sqrtf((command->target_bearing_yaw_deg * command->target_bearing_yaw_deg) +
                              (command->target_bearing_vertical_deg *
                               command->target_bearing_vertical_deg));

    command->target_distance_m = 0.0f;
    command->target_box_width_px = controller->target.box_width_px;
    command->target_box_height_px = controller->target.box_height_px;

    dynamic_deadband_deg = TargetChase_TargetBoxHalfDiagonalBearingDeg(controller) *
                           TargetChase_Max(config->box_dynamic_deadband_scale, 0.0f);
    /*
     * Box-size deadband suppresses close-range balloon sway, but it must not
     * grow without bound. In the 2026-07-30 flight it reached 31.8 degrees,
     * allowing a 24-degree off-axis target to receive zero body yaw while the
     * aircraft continued forward. Keep a useful near deadband without
     * disabling edge recentering.
     */
    dynamic_deadband_deg = TargetChase_Min(
        dynamic_deadband_deg, TARGET_CHASE_MAX_DYNAMIC_DEADBAND_DEG);

    command->effective_yaw_deadband_deg = config->yaw_deadband_deg + dynamic_deadband_deg;
    command->effective_bearing_deadband_deg = config->bearing_deadband_deg + dynamic_deadband_deg;
    command->effective_approach_bearing_deg = config->max_approach_bearing_deg + dynamic_deadband_deg;
    command->reached = bearing_error_deg <= command->effective_bearing_deadband_deg;

    /*
     * Hysteresis gives the gimbal the fast image loop and asks the slower body
     * loop to recenter only before the gimbal approaches its travel limit.
     *
     * Use the boresight that the gimbal will have after it centres the target
     * (current boresight + image error), not the current boresight alone. A
     * search handoff can have the gimbal 36 degrees left while the target is
     * at the right image edge, making the real body-relative target bearing
     * almost zero. Following the current boresight in that case turns the
     * aircraft away from the target.
     */
    if (!controller->gimbal_recenter_active &&
        fabsf(command->target_bearing_yaw_deg) >=
            config->gimbal_recenter_enter_deg) {
        controller->gimbal_recenter_active = true;
    } else if (controller->gimbal_recenter_active &&
               fabsf(command->target_bearing_yaw_deg) <=
                   config->gimbal_recenter_exit_deg) {
        controller->gimbal_recenter_active = false;
    }

    box_angle_deg = controller->filtered_box_angle_deg > 0.01f
                        ? controller->filtered_box_angle_deg
                        : TargetChase_TargetBoxHalfDiagonalBearingDeg(controller);
    planning_max_yaw_rate_degps = config->max_yaw_rate_degps;
    if (box_angle_deg >= TARGET_CHASE_NEAR_HIGH_RATE_BOX_ANGLE_DEG &&
        now_ms >= controller->target.timestamp_ms &&
        now_ms - controller->target.timestamp_ms <=
            TARGET_CHASE_NEAR_LOW_LATENCY_MAX_MS) {
        planning_max_yaw_rate_degps = TargetChase_Min(
            config->max_yaw_rate_degps *
                TARGET_CHASE_NEAR_HIGH_RATE_YAW_GAIN,
            12.0f);
    }
    desired_speed_mps = config->approach_speed_mps;
    if (box_angle_deg > 0.01f &&
        config->box_angle_small_deg > 0.01f &&
        config->box_angle_large_deg > config->box_angle_small_deg) {
        float speed_ratio = (box_angle_deg - config->box_angle_large_deg) /
                            (config->box_angle_small_deg -
                             config->box_angle_large_deg);
        speed_ratio = TargetChase_Clamp(speed_ratio, 0.0f, 1.0f);
        desired_speed_mps = config->min_approach_speed_mps +
                            (config->approach_speed_mps -
                             config->min_approach_speed_mps) *
                                speed_ratio;
    }
    if (controller->target.timestamp_ms > 0 &&
        now_ms >= controller->target.timestamp_ms) {
        uint64_t observation_age_ms =
            now_ms - controller->target.timestamp_ms;
        if (observation_age_ms > 350ULL) {
            float stale_ratio = TargetChase_Clamp(
                (float) (observation_age_ms - 350ULL) / 350.0f,
                0.0f,
                1.0f);
            float latency_factor = 1.0f - stale_ratio * 0.35f;
            float latency_floor_mps =
                config->min_approach_speed_mps *
                (1.0f - stale_ratio * 0.30f);
            desired_speed_mps = TargetChase_Max(
                latency_floor_mps,
                desired_speed_mps * latency_factor);
        }
    }

    /*
     * Small finite-horizon constrained MPC. It enumerates bounded forward
     * accelerations and yaw rates, predicts target angular motion and box-size
     * growth, and minimizes the requested five-term objective. This keeps the
     * implementation deterministic on Manifold 2 and leaves a clean place for
     * obstacle constraints once an obstacle input is available.
     */
    current_speed_mps = TargetChase_Clamp(controller->last_planned_forward_mps,
                                          0.0f,
                                          config->max_horizontal_speed_mps);
    accel_candidates[0] = -config->mpc_max_accel_mps2;
    accel_candidates[1] = 0.0f;
    accel_candidates[2] = config->mpc_max_accel_mps2;
    if (controller->gimbal_recenter_active ||
        fabsf(command->target_bearing_yaw_deg) >
            command->effective_yaw_deadband_deg) {
        yaw_candidates[0] = -planning_max_yaw_rate_degps;
        yaw_candidates[1] = -0.5f * planning_max_yaw_rate_degps;
        yaw_candidates[2] = 0.0f;
        yaw_candidates[3] = 0.5f * planning_max_yaw_rate_degps;
        yaw_candidates[4] = planning_max_yaw_rate_degps;
        yaw_count = 5;
    } else {
        yaw_candidates[0] = 0.0f;
        yaw_count = 1;
    }

    for (accel_index = 0; accel_index < accel_count; ++accel_index) {
        for (yaw_index = 0; yaw_index < yaw_count; ++yaw_index) {
            float candidate_accel = accel_candidates[accel_index];
            float candidate_yaw_rate = yaw_candidates[yaw_index];
            float predicted_speed = current_speed_mps;
            float cost = 0.0f;
            float t_s;

            for (t_s = config->mpc_step_s;
                 t_s <= config->mpc_horizon_s + 0.001f;
                 t_s += config->mpc_step_s) {
                float predicted_image_error_deg =
                    image_yaw_deg * expf(-t_s / 0.20f);
                float predicted_gimbal_offset_deg =
                    command->target_bearing_yaw_deg +
                    image_yaw_rate_degps * t_s -
                    candidate_yaw_rate * t_s;
                float predicted_box_angle_deg;
                float distance_error_deg;

                predicted_speed = TargetChase_Clamp(
                    current_speed_mps + candidate_accel * t_s,
                    0.0f,
                    desired_speed_mps);
                predicted_box_angle_deg =
                    box_angle_deg +
                    controller->filtered_box_angle_rate_degps * t_s +
                    predicted_speed * 1.5f * t_s;
                distance_error_deg =
                    TargetChase_Max(config->desired_box_angle_deg -
                                        predicted_box_angle_deg,
                                    0.0f);
                cost += config->mpc_weight_image *
                            predicted_image_error_deg * predicted_image_error_deg +
                        config->mpc_weight_gimbal_center *
                            predicted_gimbal_offset_deg * predicted_gimbal_offset_deg +
                        config->mpc_weight_distance *
                            distance_error_deg * distance_error_deg +
                        config->mpc_weight_acceleration *
                            candidate_accel * candidate_accel +
                        config->mpc_weight_yaw_rate *
                            candidate_yaw_rate * candidate_yaw_rate;
            }

            if (cost < best_cost) {
                best_cost = cost;
                best_speed_mps = TargetChase_Clamp(
                    current_speed_mps + candidate_accel * config->mpc_step_s,
                    0.0f,
                    desired_speed_mps);
                best_yaw_rate_degps = candidate_yaw_rate;
            }
        }
    }

    /*
     * Keep translating while the body recentres the gimbal.  The former hard
     * gate set forward speed to zero as soon as the camera crossed roughly
     * 22 degrees.  Because target tracking also waited for body alignment,
     * this could deadlock with a visible target near the image edge.
     *
     * Project the approach speed onto body-forward instead. Moderate
     * off-axis targets retain a useful closing component while far away; the
     * size-adaptive gate below progressively requires front alignment near
     * impact. The continuous taper avoids a new stop/start discontinuity.
     */
    {
        float approach_yaw_abs_deg =
            fabsf(TargetChase_Wrap180(command->target_bearing_yaw_deg));
        float forward_projection;
        float closing_floor_mps;
        float near_alignment_factor;
        float near_alignment_limit_deg;

        /*
         * A large box means impact is close.  Far away, simultaneous yaw and
         * translation saves time; close to the balloon the same policy creates
         * a visibly sideways strike because the airframe cannot turn as fast
         * as it closes. Progressively tighten the allowable approach bearing
         * from 75 degrees to 30 degrees as angular size grows.
         */
        near_alignment_factor = TargetChase_Clamp(
            (box_angle_deg - 5.0f) / 7.0f, 0.0f, 1.0f);
        near_alignment_limit_deg =
            75.0f - near_alignment_factor * 45.0f;

        if (approach_yaw_abs_deg >= near_alignment_limit_deg) {
            forward_projection = 0.0f;
        } else {
            float alignment_taper = 1.0f;
            forward_projection =
                cosf(approach_yaw_abs_deg * TARGET_CHASE_DEG_TO_RAD);
            forward_projection = TargetChase_Clamp(forward_projection,
                                                    0.25f,
                                                    1.0f);
            if (near_alignment_factor > 0.0f &&
                approach_yaw_abs_deg > 10.0f) {
                alignment_taper = TargetChase_Clamp(
                    (near_alignment_limit_deg - approach_yaw_abs_deg) /
                        TargetChase_Max(near_alignment_limit_deg - 10.0f, 1.0f),
                    0.0f,
                    1.0f);
                forward_projection *=
                    (1.0f - near_alignment_factor) +
                    near_alignment_factor * alignment_taper;
            }
        }
        best_speed_mps *= forward_projection;

        /*
         * This mission is a strike, not a stand-off observation.  Once the
         * target box reaches the MPC's desired angular size, distance cost no
         * longer rewards moving closer and a controller starting from zero
         * could otherwise remain parked indefinitely.  Maintain the configured
         * minimum closing speed, but ramp it through the same acceleration
         * bound and angle projection used by the planner.
         */
        closing_floor_mps = TargetChase_Min(
            TargetChase_Min(config->min_approach_speed_mps,
                            desired_speed_mps),
            current_speed_mps +
                config->mpc_max_accel_mps2 * config->mpc_step_s);
        closing_floor_mps =
            TargetChase_Max(closing_floor_mps, 0.0f) * forward_projection;
        best_speed_mps = TargetChase_Max(best_speed_mps, closing_floor_mps);
    }
    command->x_mps = best_speed_mps;
    command->yaw_rate_degps = best_yaw_rate_degps;
    (void) TargetChase_LimitForwardForAlignment(controller, command);
    controller->last_planned_forward_mps = command->x_mps;
    controller->last_planned_yaw_rate_degps = command->yaw_rate_degps;

    if (fabsf(command->target_bearing_vertical_deg) > config->vertical_bearing_deadband_deg) {
        /*
         * Couple vertical speed to the commanded forward speed so the velocity
         * vector follows the target line of sight in the body-forward vertical
         * plane: z / x = tan(vertical bearing). Clamp the tangent input before
         * applying the independent vertical speed and height safety limits.
         */
        vertical_bearing_for_tangent_deg = TargetChase_Clamp(
            command->target_bearing_vertical_deg,
            -89.0f,
            89.0f);
        command->z_mps = command->x_mps *
                         tanf(vertical_bearing_for_tangent_deg * TARGET_CHASE_DEG_TO_RAD);

        command->z_mps = TargetChase_Clamp(command->z_mps,
                                           -config->max_vertical_speed_mps,
                                           config->max_vertical_speed_mps);

        /* If vertical speed saturates, reduce forward speed to preserve the
         * requested line-of-sight slope. The closing-speed floor must not
         * override this physical constraint. */
        (void) TargetChase_LimitForwardForVertical(config, command);

        if ((aircraft->altitude_agl_m <= config->min_height_m && command->z_mps < 0.0f) ||
            (aircraft->altitude_agl_m >= config->max_height_m && command->z_mps > 0.0f)) {
            command->z_mps = 0.0f;
            command->x_mps = 0.0f;
        }
    }
    controller->last_planned_forward_mps = command->x_mps;

    /*
     * Bearing-only pursuit has no range estimate, so it cannot choose a final
     * stopping distance. Keep moving while a fresh target exists and steer at
     * the same time. Slow down for a large horizontal bearing error, but never
     * wait for perfect centering: that old gate caused yaw-only circles when
     * detections arrived near the image edge.
     */
}

static void TargetChase_RunGroundNadir(const T_TargetChaseController *controller,
                                       float normalized_x,
                                       float normalized_y,
                                       T_TargetChaseCommand *command)
{
    const T_TargetChaseConfig *config = &controller->config;
    const T_TargetChaseAircraftState *aircraft = &controller->aircraft;
    float offset_right_m;
    float offset_forward_m;
    float distance_m;
    float velocity_forward_mps;
    float velocity_right_mps;
    float yaw_error_deg;

    offset_right_m = aircraft->altitude_agl_m * normalized_x * config->pixel_x_to_body_right_sign;
    offset_forward_m = aircraft->altitude_agl_m * normalized_y * config->pixel_y_to_body_forward_sign;
    distance_m = sqrtf((offset_forward_m * offset_forward_m) + (offset_right_m * offset_right_m));

    command->target_offset_forward_m = offset_forward_m;
    command->target_offset_right_m = offset_right_m;
    command->target_distance_m = distance_m;
    command->reached = distance_m <= config->stop_radius_m;

    TargetChase_BodyVelocityFromNeu(aircraft, &velocity_forward_mps, &velocity_right_mps);

    if (!command->reached) {
        command->x_mps = (config->kp_position * offset_forward_m) -
                         (config->kd_velocity * velocity_forward_mps);
        command->y_mps = (config->kp_position * offset_right_m) -
                         (config->kd_velocity * velocity_right_mps);
    }

    TargetChase_ClampHorizontal(&command->x_mps, &command->y_mps, config->max_horizontal_speed_mps);

    if (controller->desired_height_initialized) {
        command->z_mps = (config->kp_height * (controller->desired_height_m - aircraft->altitude_agl_m)) -
                         (config->kd_vertical_velocity * aircraft->velocity_neu_mps[2]);
        command->z_mps = TargetChase_Clamp(command->z_mps,
                                           -config->max_vertical_speed_mps,
                                           config->max_vertical_speed_mps);
    }

    if (!command->reached && distance_m > 0.05f) {
        yaw_error_deg = TargetChase_Wrap180(atan2f(offset_right_m, offset_forward_m) * TARGET_CHASE_RAD_TO_DEG);
        command->target_bearing_yaw_deg = yaw_error_deg;

        if (fabsf(yaw_error_deg) > config->yaw_deadband_deg) {
            command->yaw_rate_degps = TargetChase_Clamp(config->kp_yaw_rate * yaw_error_deg,
                                                        -config->max_yaw_rate_degps,
                                                        config->max_yaw_rate_degps);
        }
    }
}

T_TargetChaseConfig TargetChase_DefaultConfig(float image_width_px,
                                              float image_height_px,
                                              float horizontal_fov_deg,
                                              float vertical_fov_deg)
{
    T_TargetChaseConfig config;

    memset(&config, 0, sizeof(config));
    config.geometry = TARGET_CHASE_GEOMETRY_AIRBORNE_BEARING_ONLY;
    config.image_width_px = image_width_px;
    config.image_height_px = image_height_px;
    config.cx_px = image_width_px * 0.5f;
    config.cy_px = image_height_px * 0.5f;

    if (horizontal_fov_deg > 0.0f) {
        config.fx_px = image_width_px / (2.0f * tanf(horizontal_fov_deg * TARGET_CHASE_DEG_TO_RAD * 0.5f));
    }

    if (vertical_fov_deg > 0.0f) {
        config.fy_px = image_height_px / (2.0f * tanf(vertical_fov_deg * TARGET_CHASE_DEG_TO_RAD * 0.5f));
    }

    config.pixel_x_to_yaw_sign = 1.0f;           /* 图像右侧目标对应正 yaw 角速度。 */
    config.pixel_y_to_up_sign = -1.0f;           /* 图像上方目标对应正向上升速度。 */
    config.pixel_x_to_body_right_sign = 1.0f;    /* 下视地面模式：图像 x 对应机体右向。 */
    config.pixel_y_to_body_forward_sign = 1.0f;  /* 下视地面模式：图像 y 对应机体前向。 */

    config.desired_height_m = 0.0f;              /* <= 0 表示接管瞬间保持当前高度。 */
    config.min_height_m = 3.0f;                  /* 低于该高度时禁止继续下降。 */
    config.max_height_m = 80.0f;                 /* 高于该高度时禁止继续上升。 */
    config.target_timeout_ms = 800.0f;           /* Stop within 0.8 s of the last real detector observation. */
    config.stop_radius_m = 0.8f;                 /* 下视地面模式到目标的停止半径。 */
    config.yaw_deadband_deg = 10.0f;              /* yaw 水平方位误差小于该值时不转向。 */
    config.bearing_deadband_deg = 8.0f;          /* 方位误差小于该值时认为目标已基本居中。 */
    config.vertical_bearing_deadband_deg = 3.0f; /* 垂直误差较小时不升降，抑制上下抖动。 */
    config.max_approach_bearing_deg = 10.0f;      /* 目标接近光轴后才允许前向接近。 */
    config.box_dynamic_deadband_scale = 1.0f;    /* 动态跟踪框随目标框变大，抑制近距离左右抖动。 */
    config.approach_speed_mps = 1.0f;            /* Forward speed for photo-feedback chase. */
    config.min_approach_speed_mps = 0.3f;
    config.box_angle_small_deg = 3.0f;
    config.box_angle_large_deg = 15.0f;
    config.gimbal_mount_yaw_offset_deg = 0.0f;
    config.gimbal_alignment_deadband_deg = 2.0f;
    config.gimbal_recenter_enter_deg = 12.0f;
    config.gimbal_recenter_exit_deg = 4.0f;
    config.prediction_horizon_s = 0.15f;
    config.prediction_mode = 0; /* Library compatibility; application selects hybrid. */
    config.prediction_adaptive = true;
    /* Provisional tuning values, not empirical balloon thresholds. */
    config.prediction_residual_min_deg = 3.0f;
    config.prediction_residual_box_scale = 1.5f;
    config.motion_filter_alpha = 0.65f;
    config.motion_filter_beta = 0.12f;
    config.mpc_horizon_s = 1.2f;
    config.mpc_step_s = 0.2f;
    config.mpc_max_accel_mps2 = 0.8f;
    config.mpc_weight_image = 1.0f;
    config.mpc_weight_gimbal_center = 2.0f;
    config.mpc_weight_distance = 1.0f;
    config.mpc_weight_acceleration = 0.3f;
    config.mpc_weight_yaw_rate = 0.18f;
    config.desired_box_angle_deg = 25.0f;

    config.max_horizontal_speed_mps = 2.0f;      /* 限制机体 x/y 水平速度命令。 */
    config.max_vertical_speed_mps = 0.3f;        /* 首次实飞前采用保守的视觉追踪升降速度上限。 */
    config.max_vertical_excursion_m = 3.0f;      /* 接管后只允许在起始高度上下 3 m 内自动升降。 */
    config.max_yaw_rate_degps = 10.0f;           /* 限制 yaw 角速度命令。 */
    config.max_tilt_deg = 20.0f;                 /* Legacy field; no authority gate uses it. */
    config.max_angular_rate_radps = 1.0f;        /* Legacy field; no authority gate uses it. */

    config.kp_position = 0.0f;                   /* 下视地面模式位置 P 增益。 */
    config.kd_velocity = 0.0f;                   /* 下视地面模式水平速度阻尼 D 增益。 */
    config.kp_height = 0.30f;                    /* 高度保持 P 增益。 */
    config.kd_vertical_velocity = 0.25f;         /* 垂直速度阻尼 D 增益。 */
    config.kp_yaw_rate = 0.45f;                  /* yaw 方位误差到 yaw 角速度的 P 增益。 */
    config.kp_vertical_bearing = 0.060f;         /* Legacy field; airborne mode now follows z/x=tan(vertical bearing). */

    return config;
}

bool TargetChase_ComputeSearchCoverage(float horizontal_fov_deg,
                                      float vertical_fov_deg,
                                      float gimbal_yaw_limit_deg,
                                      float gimbal_pitch_min_deg,
                                      float gimbal_pitch_max_deg,
                                      float requested_overlap_deg,
                                      T_TargetChaseSearchCoverage *coverage)
{
    float sector_width_deg;
    float non_overlapped_width_deg;
    unsigned int sector_count;

    if (coverage == NULL ||
        !isfinite(horizontal_fov_deg) ||
        !isfinite(vertical_fov_deg) ||
        !isfinite(gimbal_yaw_limit_deg) ||
        !isfinite(gimbal_pitch_min_deg) ||
        !isfinite(gimbal_pitch_max_deg) ||
        !isfinite(requested_overlap_deg) ||
        horizontal_fov_deg <= 0.0f ||
        horizontal_fov_deg > 180.0f ||
        vertical_fov_deg <= 0.0f ||
        vertical_fov_deg > 180.0f ||
        gimbal_yaw_limit_deg < 0.0f ||
        gimbal_pitch_max_deg < gimbal_pitch_min_deg ||
        requested_overlap_deg < 0.0f) {
        return false;
    }

    sector_width_deg = TargetChase_Clamp(
        horizontal_fov_deg + 2.0f * gimbal_yaw_limit_deg,
        horizontal_fov_deg,
        360.0f);
    if (requested_overlap_deg >= sector_width_deg) {
        return false;
    }

    non_overlapped_width_deg = sector_width_deg - requested_overlap_deg;
    sector_count = (unsigned int) ceilf(360.0f / non_overlapped_width_deg);
    if (sector_count < 1U) {
        sector_count = 1U;
    }

    memset(coverage, 0, sizeof(*coverage));
    coverage->horizontal_fov_deg = horizontal_fov_deg;
    coverage->vertical_fov_deg = vertical_fov_deg;
    coverage->gimbal_yaw_limit_deg = gimbal_yaw_limit_deg;
    coverage->gimbal_pitch_min_deg = gimbal_pitch_min_deg;
    coverage->gimbal_pitch_max_deg = gimbal_pitch_max_deg;
    coverage->sector_width_deg = sector_width_deg;
    coverage->vertical_coverage_deg = TargetChase_Clamp(
        vertical_fov_deg + gimbal_pitch_max_deg - gimbal_pitch_min_deg,
        vertical_fov_deg,
        180.0f);
    coverage->requested_overlap_deg = requested_overlap_deg;
    coverage->sector_count = sector_count;
    coverage->body_yaw_step_deg = sector_count > 1U
                                      ? 360.0f / (float) sector_count
                                      : 0.0f;
    coverage->actual_overlap_deg = sector_count > 1U
                                       ? sector_width_deg - coverage->body_yaw_step_deg
                                       : sector_width_deg;
    return true;
}

bool TargetChase_UpdateImageGeometry(T_TargetChaseController *controller,
                                     float image_width_px,
                                     float image_height_px)
{
    float fx_per_width;
    float fy_per_height;

    if (controller == NULL ||
        !TargetChase_IsFinite(image_width_px) ||
        !TargetChase_IsFinite(image_height_px) ||
        image_width_px <= 1.0f ||
        image_height_px <= 1.0f ||
        controller->config.image_width_px <= 1.0f ||
        controller->config.image_height_px <= 1.0f) {
        return false;
    }

    if (fabsf(controller->config.image_width_px - image_width_px) < 0.5f &&
        fabsf(controller->config.image_height_px - image_height_px) < 0.5f) {
        return true;
    }

    fx_per_width = controller->config.fx_px / controller->config.image_width_px;
    fy_per_height = controller->config.fy_px / controller->config.image_height_px;
    controller->config.image_width_px = image_width_px;
    controller->config.image_height_px = image_height_px;
    controller->config.cx_px = image_width_px * 0.5f;
    controller->config.cy_px = image_height_px * 0.5f;
    controller->config.fx_px = fx_per_width * image_width_px;
    controller->config.fy_px = fy_per_height * image_height_px;
    TargetChase_ResetDelayedPrediction(controller, TARGET_KF_RESET);
    return true;
}

void TargetChase_Init(T_TargetChaseController *controller,
                      const T_TargetChaseConfig *config)
{
    if (controller == NULL) {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = (config != NULL) ? *config : TargetChase_DefaultConfig(
        TARGET_CHASE_DEFAULT_IMAGE_WIDTH_PX,
        TARGET_CHASE_DEFAULT_IMAGE_HEIGHT_PX,
        TARGET_CHASE_DEFAULT_HORIZONTAL_FOV_DEG,
        TARGET_CHASE_DEFAULT_VERTICAL_FOV_DEG);
    controller->mode = TARGET_CHASE_MODE_MANUAL;
    controller->kf_reason = controller->config.prediction_mode == 3 ?
        TARGET_KF_NO_CALIBRATION : TARGET_KF_DISABLED;
}

void TargetChase_UpdateAircraftState(T_TargetChaseController *controller,
                                     const T_TargetChaseAircraftState *state)
{
    if (controller == NULL || state == NULL) {
        return;
    }

    controller->aircraft = *state;
}

void TargetChase_SetCameraBoresight(T_TargetChaseController *controller,
                                    float yaw_deg,
                                    float vertical_deg)
{
    if (controller == NULL ||
        !TargetChase_IsFinite(yaw_deg) ||
        !TargetChase_IsFinite(vertical_deg)) {
        return;
    }

    controller->camera_boresight_yaw_deg = TargetChase_Wrap180(yaw_deg);
    controller->camera_boresight_vertical_deg = TargetChase_Clamp(vertical_deg, -90.0f, 90.0f);
}

void TargetChase_SetObservationCameraPose(T_TargetChaseController *controller,
                                          float world_yaw_deg,
                                          float world_pitch_deg,
                                          uint64_t timestamp_ms)
{
    if (controller == NULL || timestamp_ms == 0 ||
        !TargetChase_IsFinite(world_yaw_deg) ||
        !TargetChase_IsFinite(world_pitch_deg)) {
        return;
    }
    controller->observation_camera_yaw_deg = TargetChase_Wrap180(world_yaw_deg);
    controller->observation_camera_pitch_deg =
        TargetChase_Clamp(world_pitch_deg, -90.0f, 90.0f);
    controller->observation_pose_timestamp_ms = timestamp_ms;
    controller->observation_pose_valid = true;
}

void TargetChase_UpdateTargetPixel(T_TargetChaseController *controller,
                                   float u_px,
                                   float v_px,
                                   uint64_t timestamp_ms)
{
    if (controller == NULL) {
        return;
    }
    if (controller->motion_estimate_initialized &&
        timestamp_ms <= controller->motion_estimate_timestamp_ms) {
        controller->observation_pose_valid = false;
        return;
    }

    controller->target.u_px = u_px;
    controller->target.v_px = v_px;
    controller->target.box_width_px = 0.0f;
    controller->target.box_height_px = 0.0f;
    controller->target.has_box = false;
    controller->target.timestamp_ms = timestamp_ms;
    controller->target.valid = TargetChase_IsFinite(u_px) &&
                               TargetChase_IsFinite(v_px) &&
                               u_px >= 0.0f &&
                               v_px >= 0.0f &&
                               u_px < controller->config.image_width_px &&
                               v_px < controller->config.image_height_px;
    TargetChase_UpdateMotionEstimate(controller);
}

void TargetChase_UpdateTargetBox(T_TargetChaseController *controller,
                                 float left_bottom_u_px,
                                 float left_bottom_v_px,
                                 float right_top_u_px,
                                 float right_top_v_px,
                                 uint64_t timestamp_ms)
{
    float left_px;
    float right_px;
    float top_px;
    float bottom_px;

    if (controller == NULL) {
        return;
    }
    if (controller->motion_estimate_initialized &&
        timestamp_ms <= controller->motion_estimate_timestamp_ms) {
        controller->observation_pose_valid = false;
        return;
    }

    controller->target.timestamp_ms = timestamp_ms;
    controller->target.valid = false;
    controller->target.has_box = false;
    controller->target.box_width_px = 0.0f;
    controller->target.box_height_px = 0.0f;

    if (!TargetChase_IsFinite(left_bottom_u_px) ||
        !TargetChase_IsFinite(left_bottom_v_px) ||
        !TargetChase_IsFinite(right_top_u_px) ||
        !TargetChase_IsFinite(right_top_v_px)) {
        return;
    }

    left_px = TargetChase_Min(left_bottom_u_px, right_top_u_px);
    right_px = TargetChase_Max(left_bottom_u_px, right_top_u_px);
    top_px = TargetChase_Min(left_bottom_v_px, right_top_v_px);
    bottom_px = TargetChase_Max(left_bottom_v_px, right_top_v_px);

    left_px = TargetChase_Clamp(left_px, 0.0f, controller->config.image_width_px);
    right_px = TargetChase_Clamp(right_px, 0.0f, controller->config.image_width_px);
    top_px = TargetChase_Clamp(top_px, 0.0f, controller->config.image_height_px);
    bottom_px = TargetChase_Clamp(bottom_px, 0.0f, controller->config.image_height_px);

    controller->target.box_width_px = right_px - left_px;
    controller->target.box_height_px = bottom_px - top_px;
    controller->target.u_px = (left_px + right_px) * 0.5f;
    controller->target.v_px = (top_px + bottom_px) * 0.5f;

    controller->target.valid = controller->target.box_width_px > 1.0f &&
                               controller->target.box_height_px > 1.0f &&
                               controller->target.u_px >= 0.0f &&
                               controller->target.v_px >= 0.0f &&
                               controller->target.u_px < controller->config.image_width_px &&
                               controller->target.v_px < controller->config.image_height_px;
    controller->target.has_box = controller->target.valid;
    TargetChase_UpdateMotionEstimate(controller);
}

void TargetChase_ClearTarget(T_TargetChaseController *controller)
{
    if (controller == NULL) {
        return;
    }

    memset(&controller->target, 0, sizeof(controller->target));
    controller->pixel_timestamp_ms = 0;
    controller->pixel_samples = 0;
    controller->pixel_du = controller->pixel_dv = 0;
    controller->hybrid_weight = controller->hybrid_horizon_s = 0;
    controller->motion_estimate_initialized = false;
    controller->motion_world_valid = false;
    controller->observation_pose_valid = false;
    controller->gimbal_recenter_active = false;
    controller->last_planned_forward_mps = 0.0f;
    controller->last_planned_yaw_rate_degps = 0.0f;
    TargetChase_ResetDelayedPrediction(controller, TARGET_KF_RESET);
}

void TargetChase_SetManualTrigger(T_TargetChaseController *controller,
                                  bool enabled,
                                  uint64_t now_ms)
{
    (void) now_ms;

    if (controller == NULL) {
        return;
    }

    if (!enabled && controller->manual_trigger_enabled)
        TargetChase_ResetDelayedPrediction(controller, TARGET_KF_RESET);
    controller->manual_trigger_enabled = enabled;

    if (!enabled && !controller->has_authority) {
        controller->mode = TARGET_CHASE_MODE_MANUAL;
    }
}

bool TargetChase_IsManualTriggerEnabled(const T_TargetChaseController *controller)
{
    return controller != NULL && controller->manual_trigger_enabled;
}

void TargetChase_SetAuthority(T_TargetChaseController *controller,
                              bool has_authority,
                              uint64_t now_ms)
{
    (void) now_ms;

    if (controller == NULL) {
        return;
    }

    controller->has_authority = has_authority;

    if (has_authority) {
        controller->desired_height_m = controller->config.desired_height_m > 0.0f
                                           ? controller->config.desired_height_m
                                           : controller->aircraft.altitude_agl_m;
        controller->desired_height_initialized = true;
        controller->mode = TARGET_CHASE_MODE_ACTIVE;
    } else {
        controller->desired_height_initialized = false;
        controller->mode = TARGET_CHASE_MODE_MANUAL;
    }
}

bool TargetChase_ShouldObtainAuthority(const T_TargetChaseController *controller,
                                       uint64_t now_ms)
{
    if (controller == NULL || controller->has_authority) {
        return false;
    }

    return controller->manual_trigger_enabled &&
           TargetChase_TargetFresh(controller, now_ms);
}

bool TargetChase_ShouldReleaseAuthority(const T_TargetChaseController *controller,
                                        uint64_t now_ms)
{
    if (controller == NULL || !controller->has_authority) {
        return false;
    }

    return !controller->manual_trigger_enabled ||
           !TargetChase_TargetFresh(controller, now_ms);
}

bool TargetChase_Step(T_TargetChaseController *controller,
                      uint64_t now_ms,
                      T_TargetChaseCommand *command)
{
    const T_TargetChaseConfig *config;
    float normalized_x;
    float normalized_y;
    float prediction_s = 0.0f;

    if (controller == NULL || command == NULL) {
        return false;
    }

    config = &controller->config;
    TargetChase_SetZeroCommand(command, controller->has_authority);

    if (!controller->manual_trigger_enabled) {
        controller->mode = controller->has_authority ? TARGET_CHASE_MODE_HOLD : TARGET_CHASE_MODE_MANUAL;
        return controller->has_authority;
    }

    if (!TargetChase_TargetFresh(controller, now_ms)) {
        if (controller->config.prediction_mode == 3)
            TargetChase_ResetDelayedPrediction(controller, TARGET_KF_STALE);
        controller->mode = controller->has_authority ? TARGET_CHASE_MODE_TARGET_LOST : TARGET_CHASE_MODE_MANUAL;
        return controller->has_authority;
    }

    if (!controller->has_authority) {
        controller->mode = TARGET_CHASE_MODE_WAITING_AUTHORITY;
        return false;
    }

    if (config->fx_px <= 1.0f || config->fy_px <= 1.0f) {
        controller->mode = TARGET_CHASE_MODE_HOLD;
        return true;
    }

    if (controller->motion_estimate_initialized) {
        prediction_s = TargetChase_PredictionHorizon(controller, now_ms);
        float predicted_u_px = controller->filtered_u_px +
                               controller->filtered_u_rate_pxps * prediction_s;
        float predicted_v_px = controller->filtered_v_px +
                               controller->filtered_v_rate_pxps * prediction_s;
        predicted_u_px = TargetChase_Clamp(predicted_u_px,
                                            0.0f,
                                            config->image_width_px);
        predicted_v_px = TargetChase_Clamp(predicted_v_px,
                                            0.0f,
                                            config->image_height_px);
        normalized_x = (predicted_u_px - config->cx_px) / config->fx_px;
        normalized_y = (predicted_v_px - config->cy_px) / config->fy_px;
    } else {
        normalized_x = (controller->target.u_px - config->cx_px) / config->fx_px;
        normalized_y = (controller->target.v_px - config->cy_px) / config->fy_px;
    }

    if (config->geometry == TARGET_CHASE_GEOMETRY_GROUND_NADIR) {
        TargetChase_RunGroundNadir(controller, normalized_x, normalized_y, command);
    } else {
        TargetChase_RunAirborneBearingOnly(
            controller, normalized_x, normalized_y, now_ms, command);
    }

    command->should_send = true;
    controller->mode = TARGET_CHASE_MODE_ACTIVE;
    return true;
}

E_TargetChaseMode TargetChase_GetMode(const T_TargetChaseController *controller)
{
    return controller != NULL ? controller->mode : TARGET_CHASE_MODE_MANUAL;
}
