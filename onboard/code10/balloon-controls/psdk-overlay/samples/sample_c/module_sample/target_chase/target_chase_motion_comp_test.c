#include "target_chase.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG_TO_RAD 0.017453292519943295f

static float pixel_for_world_yaw(const T_TargetChaseConfig *config,
                                 float world_yaw_deg, float camera_yaw_deg)
{
    return config->cx_px + config->fx_px *
        tanf((world_yaw_deg - camera_yaw_deg) * DEG_TO_RAD) /
        config->pixel_x_to_yaw_sign;
}

static void observe(T_TargetChaseController *controller, uint64_t timestamp_ms,
                    float world_yaw_deg, float camera_yaw_deg)
{
    TargetChase_SetObservationCameraPose(controller, camera_yaw_deg, 0.0f,
                                         timestamp_ms);
    TargetChase_UpdateTargetBox(controller,
        pixel_for_world_yaw(&controller->config, world_yaw_deg, camera_yaw_deg) - 20.0f,
        controller->config.cy_px - 20.0f,
        pixel_for_world_yaw(&controller->config, world_yaw_deg, camera_yaw_deg) + 20.0f,
        controller->config.cy_px + 20.0f, timestamp_ms);
}

int main(void)
{
    T_TargetChaseConfig config = TargetChase_DefaultConfig(1920.0f, 1080.0f,
                                                            73.7f, 45.7f);
    T_TargetChaseController chase;
    T_TargetChaseAircraftState aircraft;
    T_TargetChaseCommand command;
    config.prediction_adaptive = false; /* Explicit fixed-strategy regression. */

    TargetChase_Init(&chase, &config);
    memset(&aircraft, 0, sizeof(aircraft));
    aircraft.in_air = true;
    aircraft.altitude_agl_m = 10.0f;
    aircraft.timestamp_ms = 1000;
    TargetChase_UpdateAircraftState(&chase, &aircraft);
    TargetChase_SetManualTrigger(&chase, true, 1000);
    TargetChase_SetAuthority(&chase, true, 1000);

    /* A fixed balloon does not acquire speed when the camera turns 10 deg. */
    observe(&chase, 1000, 0.0f, 0.0f);
    observe(&chase, 1100, 0.0f, 10.0f);
    assert(chase.motion_world_valid);
    assert(fabsf(chase.filtered_world_yaw_rate_degps) < 0.05f);
    TargetChase_SetCameraBoresight(&chase, 10.0f, 0.0f);
    assert(TargetChase_Step(&chase, 1100, &command));
    assert(fabsf(command.target_bearing_yaw_deg) < 0.05f);

    /* A 2 deg target movement survives the same camera rotation. */
    observe(&chase, 1200, 2.0f, 10.0f);
    assert(chase.filtered_world_yaw_rate_degps > 1.0f);
    assert(TargetChase_Step(&chase, 1200, &command));
    assert(command.target_bearing_yaw_deg > chase.filtered_world_yaw_deg);

    /* Aircraft yaw changes the body-relative bearing, not world target speed. */
    aircraft.attitude_deg[2] = 10.0f;
    aircraft.timestamp_ms = 1200;
    TargetChase_UpdateAircraftState(&chase, &aircraft);
    assert(TargetChase_Step(&chase, 1200, &command));
    assert(command.target_bearing_yaw_deg < -7.0f);

    /* A delayed older frame must not roll the estimate backwards. */
    observe(&chase, 1150, -4.0f, 0.0f);
    assert(chase.motion_estimate_timestamp_ms == 1200);
    assert(chase.target.timestamp_ms == 1200);

    /* No frame-time pose means no motion extrapolation. */
    TargetChase_UpdateTargetBox(&chase, 900.0f, 500.0f,
                                940.0f, 540.0f, 1300);
    assert(!chase.motion_world_valid);
    assert(chase.filtered_u_rate_pxps == 0.0f);

    /* The yaw residual must cross the +/-180 boundary by the short arc. */
    TargetChase_ClearTarget(&chase);
    observe(&chase, 2000, 179.0f, 179.0f);
    observe(&chase, 2100, -179.0f, -179.0f);
    assert(chase.motion_world_valid);
    assert(chase.filtered_world_yaw_rate_degps > 0.0f);

    puts("motion compensation tests passed");

    /* Adaptive mode: sudden reversal discards old velocity, then recovers. */
    config.prediction_adaptive = true;
    TargetChase_Init(&chase, &config);
    for (unsigned int i = 0; i < 15; ++i) observe(&chase, 1000 + i * 100, i * 0.5f, 0.0f);
    assert(chase.filtered_world_yaw_rate_degps > 3.0f);
    observe(&chase, 2500, -5.0f, 0.0f);
    assert(fabsf(chase.filtered_world_yaw_deg + 5.0f) < 0.01f);
    assert(chase.filtered_world_yaw_rate_degps == 0.0f);
    T_TargetChasePrediction prediction;
    assert(TargetChase_PredictWorld(&chase, 2550, &prediction));
    assert(prediction.rate_scale == 0.0f);
    assert(prediction.reason == TARGET_PREDICTION_RECOVERING);
    observe(&chase, 2600, -5.5f, 0.0f);
    assert(chase.prediction_quality == 0.5f);
    observe(&chase, 2700, -6.0f, 0.0f);
    assert(chase.prediction_quality == 1.0f);
    assert(TargetChase_PredictWorld(&chase, 2800, &prediction));
    assert(fabsf(prediction.horizon_s - 0.25f) < 0.001f);
    /* A duplicate timestamp must not replace coordinates under an old filter. */
    float previous_u = chase.target.u_px;
    observe(&chase, 2700, 10.0f, 0.0f);
    assert(chase.target.u_px == previous_u);
    observe(&chase, 2690, 12.0f, 0.0f);
    assert(chase.target.u_px == previous_u);
    assert(!TargetChase_PredictWorld(&chase, 2699, &prediction));
    assert(TargetChase_PredictWorld(&chase, 3080, &prediction));
    assert(fabsf(prediction.rate_scale - 0.5f) < 0.001f);
    assert(TargetChase_PredictWorld(&chase, 3200, &prediction));
    assert(prediction.rate_scale == 0.0f);
    assert(!TargetChase_PredictWorld(&chase, 2700 + (uint64_t) config.target_timeout_ms + 1, &prediction));
    /* Missing pose and a long gap cannot reuse the previous angular velocity. */
    TargetChase_UpdateTargetBox(&chase, 900, 500, 940, 540, 3300);
    assert(!TargetChase_PredictWorld(&chase, 3300, &prediction));
    observe(&chase, 4000, 0, 0);
    assert(chase.filtered_world_yaw_rate_degps == 0);
    /* Fixed mode preserves the configured lead and ignores quality attenuation. */
    chase.config.prediction_adaptive = false;
    assert(TargetChase_PredictWorld(&chase, 4100, &prediction));
    assert(prediction.rate_scale == 1);
    assert(fabsf(prediction.horizon_s - 0.25f) < 0.001f);
    assert(TargetChase_PredictWorld(&chase, 4500, &prediction));
    assert(prediction.horizon_s == 0.5f);
    puts("adaptive prediction tests passed");

    /* A large nearby target keeps age compensation but loses extra lead.
     * Combined yaw/pitch extrapolation is bounded, including near reversals. */
    config.prediction_adaptive = true;
    TargetChase_Init(&chase, &config);
    observe(&chase, 5000, 0, 0);
    chase.prediction_quality = 1.0f;
    chase.filtered_box_angle_deg = config.box_angle_large_deg;
    chase.filtered_world_yaw_rate_degps = 30.0f;
    chase.filtered_world_pitch_rate_degps = 20.0f;
    assert(TargetChase_PredictWorld(&chase, 5200, &prediction));
    assert(prediction.extra_horizon_s == 0.0f);
    assert(prediction.age_horizon_s > 0.0f);
    assert(hypotf(prediction.world_yaw_deg, prediction.world_pitch_deg) <= 2.001f);
    assert(prediction.reason == TARGET_PREDICTION_NEAR_LIMIT);
    chase.target.u_px = config.image_width_px * 0.98f;
    assert(TargetChase_PredictWorld(&chase, 5200, &prediction));
    assert(prediction.horizon_s == 0.0f);
    assert(prediction.reason == TARGET_PREDICTION_EDGE);

    /* Near aligned tracking must still approach. A 35-degree vertical error
     * must not retain a fast forward command after z saturates at 0.12 m/s. */
    TargetChase_Init(&chase, &config);
    memset(&aircraft, 0, sizeof(aircraft));
    aircraft.in_air = true;
    aircraft.altitude_agl_m = 10.0f;
    aircraft.timestamp_ms = 6000;
    TargetChase_UpdateAircraftState(&chase, &aircraft);
    TargetChase_SetManualTrigger(&chase, true, 6000);
    TargetChase_SetAuthority(&chase, true, 6000);
    observe(&chase, 6000, 0, 0);
    assert(TargetChase_Step(&chase, 6000, &command));
    assert(command.x_mps > 0.0f);
    chase.filtered_world_pitch_deg = 35.0f;
    chase.last_planned_forward_mps = 0.8f;
    assert(TargetChase_Step(&chase, 6000, &command));
    assert(command.x_mps <= config.max_vertical_speed_mps / tanf(35.0f * DEG_TO_RAD) + 0.001f);
    assert(fabsf(command.z_mps - command.x_mps * tanf(35.0f * DEG_TO_RAD)) < 0.001f);
    aircraft.altitude_agl_m = config.max_height_m;
    TargetChase_UpdateAircraftState(&chase, &aircraft);
    assert(TargetChase_Step(&chase, 6000, &command));
    assert(command.x_mps == 0.0f && command.z_mps == 0.0f);
    command.x_mps = 0.9f; command.z_mps = -0.02f;
    command.target_bearing_vertical_deg = 35.0f;
    assert(TargetChase_LimitForwardForVertical(&config, &command));
    assert(command.x_mps == 0.0f); /* Slew still descending while target is above. */
    puts("near lead and constrained approach tests passed");

    /* Recorded 2026-10-17 approach: 42 degrees left, 6.2 degree box,
     * yaw already at -12 deg/s, but old command still closed at 0.586 m/s.
     * Test both symmetry and final clamping after an old smoothed command. */
    aircraft.altitude_agl_m = 10.0f;
    TargetChase_UpdateAircraftState(&chase, &aircraft);
    chase.filtered_box_angle_deg = 6.2f;
    chase.filtered_world_pitch_deg = 0.0f;
    chase.filtered_world_yaw_deg = -41.95f;
    chase.filtered_world_yaw_rate_degps = 0.0f;
    chase.last_planned_forward_mps = 0.8f;
    assert(TargetChase_Step(&chase, 6000, &command));
    assert(command.x_mps == 0.0f && command.yaw_rate_degps < 0.0f);
    for (int sign = -1; sign <= 1; sign += 2) {
        command.x_mps = 0.586f; command.z_mps = -0.027f;
        command.target_bearing_yaw_deg = sign * 41.95f;
        command.yaw_rate_degps = sign * 12.0f;
        chase.filtered_world_yaw_rate_degps = sign * 26.493f;
        assert(TargetChase_LimitForwardForAlignment(&chase, &command));
        assert(command.x_mps == 0.0f && command.z_mps == 0.0f);
        assert(command.yaw_rate_degps == sign * 12.0f);
        command.x_mps = 0.8f; /* A slew limiter must not revive old forward speed. */
        assert(TargetChase_LimitForwardForAlignment(&chase, &command));
        assert(command.x_mps == 0.0f);
    }
    /* Centered near target resumes approaching; no permanent yaw-only state. */
    chase.filtered_world_yaw_deg = 0.0f;
    chase.filtered_world_yaw_rate_degps = 0.0f;
    chase.last_planned_forward_mps = 0.0f;
    assert(TargetChase_Step(&chase, 6000, &command));
    assert(command.x_mps > 0.0f);
    command.x_mps = 0.5f; command.z_mps = 0.05f;
    command.target_bearing_yaw_deg = 15.0f; command.yaw_rate_degps = 12.0f;
    chase.filtered_world_yaw_rate_degps = 26.0f;
    assert(TargetChase_LimitForwardForAlignment(&chase, &command));
    assert(command.x_mps == 0.0f); /* Escaping faster than the yaw budget. */
    chase.filtered_world_yaw_rate_degps = -26.0f;
    command.x_mps = 0.5f;
    TargetChase_LimitForwardForAlignment(&chase, &command);
    assert(command.x_mps > 0.0f); /* Inward motion is not treated as escape. */
    command.x_mps = 0.5f; command.yaw_rate_degps = -4.0f;
    assert(TargetChase_LimitForwardForAlignment(&chase, &command));
    assert(command.x_mps == 0.0f); /* Still yawing away after a reversal. */
    chase.filtered_box_angle_deg = 2.0f;
    command.x_mps = 0.5f;
    assert(!TargetChase_LimitForwardForAlignment(&chase, &command));
    assert(command.x_mps == 0.5f); /* Far tracking keeps simultaneous translation. */
    puts("flight-derived alignment, slew, reversal and recovery tests passed");

    /* Hybrid uses independent pixel rates without pose. Far tile results keep
     * the original 500ms horizon; body and gimbal share this predictor. */
    config.prediction_mode = 2;
    TargetChase_Init(&chase, &config);
    for (int i=0; i<4; ++i)
        TargetChase_UpdateTargetBox(&chase, 950+i*5, 530, 990+i*5, 570, 7000+i*100);
    {
        float y,p,yr,pr;
        assert(chase.pixel_du > 0 && !chase.motion_world_valid);
        assert(TargetChase_PredictHybrid(&chase,7350,0,0,&y,&p,&yr,&pr));
        assert(chase.hybrid_weight == 0 && chase.hybrid_horizon_s > 0);
        assert(yr*config.pixel_x_to_yaw_sign > 0);
        assert(TargetChase_PredictHybrid(&chase,7700,0,0,&y,&p,&yr,&pr));
        assert(fabsf(chase.hybrid_horizon_s-0.5f)<0.0001f && yr != 0);
        assert(!TargetChase_PredictHybrid(&chase,7200,0,0,&y,&p,&yr,&pr));
        TargetChase_UpdateTargetBox(&chase,950,530,990,570,7200);
        assert(chase.pixel_timestamp_ms == 7300);
        TargetChase_ClearTarget(&chase);
        assert(chase.pixel_samples == 0 && chase.pixel_du == 0);
        /* Stable near observations enable world blend. */
        for (int i=0; i<4; ++i) {
            TargetChase_SetObservationCameraPose(&chase,0,0,8000+i*100);
            TargetChase_UpdateTargetBox(&chase,760,340,1160,740,8000+i*100);
        }
        assert(TargetChase_PredictHybrid(&chase,8300,0,0,&y,&p,&yr,&pr));
        assert(chase.hybrid_weight > 0.9f);
        assert(fabsf(y)<0.01f && fabsf(p)<0.01f);
        chase.config.prediction_mode=1;
        assert(TargetChase_PredictHybrid(&chase,8300,0,0,&y,&p,&yr,&pr));
        assert(chase.hybrid_weight == 0);
        chase.config.prediction_mode=2;
        TargetChase_UpdateTargetBox(&chase,765,340,1165,740,8400);
        assert(TargetChase_PredictHybrid(&chase,8400,0,0,&y,&p,&yr,&pr));
        assert(chase.hybrid_weight == 0 && chase.pixel_samples == 3);
        TargetChase_UpdateTargetBox(&chase,765,340,1165,740,8800);
        assert(chase.pixel_samples == 3);
    }
    puts("hybrid pixel fallback, historical long horizon, near blend and mode tests passed");
    return 0;
}
