#include "target_chase.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_DEG_TO_RAD 0.017453292519943295f

static void close_to(float actual, float expected, float tolerance)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "expected %.9g, got %.9g (tolerance %.9g)\n",
                (double)expected, (double)actual, (double)tolerance);
        assert(false);
    }
}

static void setup(T_TargetChaseController *controller, int mode, bool calibrated)
{
    T_TargetChaseConfig config = TargetChase_DefaultConfig(1280.0f, 720.0f, 90.0f, 60.0f);
    T_TargetChaseAircraftState aircraft;
    config.prediction_mode = mode;
    config.yaw_deadband_deg = 1.0f;
    config.bearing_deadband_deg = 1.0f;
    config.box_dynamic_deadband_scale = 0.0f;
    TargetChase_Init(controller, &config);
    memset(&aircraft, 0, sizeof(aircraft));
    aircraft.in_air = true;
    aircraft.velocity_healthy = true;
    aircraft.altitude_agl_m = 10.0f;
    aircraft.attitude_deg[2] = 20.0f;
    aircraft.timestamp_ms = 1000U;
    TargetChase_UpdateAircraftState(controller, &aircraft);
    TargetChase_SetCameraBoresight(controller, 5.0f, 0.0f);
    TargetChase_SetManualTrigger(controller, true, 1000U);
    TargetChase_SetAuthority(controller, true, 1000U);
    controller->kf_epoch_ms = 0U;
    controller->kf_valid_until_ms = calibrated ? 100000U : 0U;
}

static void current_camera(T_TargetChaseController *controller, uint64_t now_ms,
                           float yaw, float pitch)
{
    controller->kf_camera_pose_valid = true;
    controller->kf_camera_pose_ms = now_ms;
    controller->kf_camera_yaw_deg = yaw;
    controller->kf_camera_pitch_deg = pitch;
}

static void image_observation(T_TargetChaseController *controller, uint64_t frame_ms,
                              float world_yaw, float camera_yaw, float half_diagonal_deg)
{
    const T_TargetChaseConfig *config = &controller->config;
    const float u = config->cx_px + config->fx_px *
                    tanf((world_yaw - camera_yaw) * TEST_DEG_TO_RAD);
    const float half_angle = half_diagonal_deg / sqrtf(2.0f) * TEST_DEG_TO_RAD;
    const float half_width = config->fx_px * tanf(half_angle);
    const float half_height = config->fy_px * tanf(half_angle);
    TargetChase_SetObservationCameraPose(controller, camera_yaw, 0.0f, frame_ms);
    TargetChase_UpdateTargetBox(controller, u - half_width, config->cy_px - half_height,
                               u + half_width, config->cy_px + half_height, frame_ms);
}

static void stable_three(T_TargetChaseController *controller)
{
    unsigned int i;
    setup(controller, 3, true);
    for (i = 0U; i < 3U; ++i) {
        const uint64_t measurement_ms = 1000U + i * 100U;
        const float world_yaw = 30.0f + (float)i * 0.5f;
        image_observation(controller, measurement_ms + 40U, world_yaw,
                          15.0f + (float)i * 5.0f, 1.0f);
        assert(TargetChase_ObserveAlignedBearing(controller, world_yaw, 0.0f,
                                                 measurement_ms, measurement_ms + 40U));
    }
    current_camera(controller, 1250U, 25.0f, 0.0f);
}

static void query(T_TargetChaseController *controller, uint64_t now_ms,
                  float *yaw, float *pitch, float *yaw_rate, float *pitch_rate)
{
    assert(TargetChase_PredictHybrid(controller, now_ms, 25.0f, 0.0f,
                                    yaw, pitch, yaw_rate, pitch_rate));
}

static void assert_same_fallback(T_TargetChaseController *controller, uint64_t now_ms)
{
    T_TargetChaseController pixel = *controller;
    T_TargetChaseCommand original;
    T_TargetChaseCommand fallback;
    pixel.config.prediction_mode = 1;
    assert(TargetChase_Step(&pixel, now_ms, &original));
    assert(TargetChase_Step(controller, now_ms, &fallback));
    assert(!controller->kf_active);
    /* Both command objects are zero-initialized by Step, including padding. */
    assert(memcmp(&original, &fallback, sizeof(original)) == 0);
}

static void test_missing_calibration_preserves_original_pixel_commands(void)
{
    T_TargetChaseController pixel;
    T_TargetChaseController delayed;
    T_TargetChaseCommand original;
    T_TargetChaseCommand fallback;
    unsigned int i;
    setup(&pixel, 1, false);
    setup(&delayed, 3, false);
    for (i = 0U; i < 8U; ++i) {
        const uint64_t timestamp_ms = 1000U + 100U * i;
        image_observation(&pixel, timestamp_ms, 30.0f + (float)i, 25.0f, 1.0f);
        image_observation(&delayed, timestamp_ms, 30.0f + (float)i, 25.0f, 1.0f);
        assert(!TargetChase_ObserveAlignedBearing(&delayed, 30.0f + (float)i,
                                                  0.0f, timestamp_ms, timestamp_ms));
        assert(TargetChase_Step(&pixel, timestamp_ms + 40U, &original));
        assert(TargetChase_Step(&delayed, timestamp_ms + 40U, &fallback));
        assert(memcmp(&original, &fallback, sizeof(original)) == 0);
        assert(!delayed.kf_active);
        assert(delayed.kf_reason == TARGET_KF_NO_CALIBRATION);
    }
    assert(TargetChase_Step(&pixel, 2350U, &original));
    assert(TargetChase_Step(&delayed, 2350U, &fallback));
    assert(memcmp(&original, &fallback, sizeof(original)) == 0);
    close_to(delayed.hybrid_horizon_s, 0.5f, 1e-6f);
}

static void test_warmup_then_real_command_path(void)
{
    T_TargetChaseController controller;
    T_TargetChaseController pixel;
    T_TargetChaseCommand filtered;
    T_TargetChaseCommand original;
    T_TargetDelayedKfPrediction expected;
    unsigned int i;
    setup(&controller, 3, true);
    for (i = 0U; i < 3U; ++i) {
        const uint64_t measurement_ms = 1000U + 100U * i;
        const float world_yaw = 30.0f + 0.5f * (float)i;
        image_observation(&controller, measurement_ms + 40U, world_yaw,
                          15.0f + (float)i * 5.0f, 1.0f);
        assert(TargetChase_ObserveAlignedBearing(&controller, world_yaw, 0.0f,
                                                 measurement_ms, measurement_ms + 40U));
        current_camera(&controller, measurement_ms + 50U, 25.0f, 0.0f);
        if (i < 2U) {
            assert_same_fallback(&controller, measurement_ms + 50U);
            assert(controller.kf_reason == TARGET_KF_WARMUP);
        }
    }
    pixel = controller;
    pixel.config.prediction_mode = 1;
    assert(TargetDelayedKf_Predict(&controller.delayed_kf, 1250U, 0.15f, 500U, &expected));
    assert(TargetChase_Step(&controller, 1250U, &filtered));
    assert(TargetChase_Step(&pixel, 1250U, &original));
    assert(controller.kf_active && controller.kf_reason == TARGET_KF_ACTIVE);
    assert(controller.delayed_kf.accepted_samples == 3U);
    close_to(controller.hybrid_weight, 1.0f, 1e-6f);
    close_to(filtered.target_bearing_yaw_deg, expected.yaw_deg - 20.0f, 1e-4f);
    assert(fabsf(filtered.target_bearing_yaw_deg - original.target_bearing_yaw_deg) > 0.5f);
    assert(filtered.should_send);
    assert(isfinite(filtered.x_mps) && isfinite(filtered.yaw_rate_degps));
}

static void test_repeat_queries_and_current_camera_coordinates(void)
{
    T_TargetChaseController controller;
    T_TargetDelayedKf before;
    T_TargetChaseCommand first_command;
    T_TargetChaseCommand other_raw_camera;
    T_TargetChaseCommand turned_aircraft;
    float yaw, pitch, rate, pitch_rate;
    float repeated_yaw, repeated_pitch, repeated_rate, repeated_pitch_rate;
    float later_yaw, later_pitch, later_rate, later_pitch_rate;
    stable_three(&controller);
    before = controller.delayed_kf;
    query(&controller, 1250U, &yaw, &pitch, &rate, &pitch_rate);
    current_camera(&controller, 1300U, 25.0f, 0.0f);
    query(&controller, 1300U, &later_yaw, &later_pitch, &later_rate, &later_pitch_rate);
    current_camera(&controller, 1250U, 25.0f, 0.0f);
    query(&controller, 1250U, &repeated_yaw, &repeated_pitch, &repeated_rate, &repeated_pitch_rate);
    assert(memcmp(&before, &controller.delayed_kf, sizeof(before)) == 0);
    close_to(repeated_yaw, yaw, 1e-6f);
    close_to(repeated_pitch, pitch, 1e-6f);
    close_to(repeated_rate, rate, 1e-6f);
    close_to(later_yaw - yaw, rate * 0.05f, 1e-5f);

    assert(TargetChase_Step(&controller, 1250U, &first_command));
    TargetChase_SetCameraBoresight(&controller, -17.0f, 8.0f);
    assert(TargetChase_Step(&controller, 1250U, &other_raw_camera));
    assert(controller.kf_active);
    close_to(other_raw_camera.target_bearing_yaw_deg, first_command.target_bearing_yaw_deg, 1e-4f);
    close_to(other_raw_camera.target_bearing_vertical_deg,
             first_command.target_bearing_vertical_deg, 1e-4f);
    controller.aircraft.attitude_deg[2] += 5.0f;
    assert(TargetChase_Step(&controller, 1250U, &turned_aircraft));
    close_to(turned_aircraft.target_bearing_yaw_deg,
             first_command.target_bearing_yaw_deg - 5.0f, 1e-4f);
}

static void test_near_lead_reduction_preserves_frame_age_compensation(void)
{
    T_TargetChaseController far;
    T_TargetChaseController near;
    float fy, fp, fr, fpr;
    float ny, np, nr, npr;
    stable_three(&far);
    near = far;
    image_observation(&far, 1241U, 31.0f, 25.0f, 1.0f);
    image_observation(&near, 1241U, 31.0f, 25.0f, 9.0f);
    current_camera(&far, 1290U, 25.0f, 0.0f);
    current_camera(&near, 1290U, 25.0f, 0.0f);
    query(&far, 1290U, &fy, &fp, &fr, &fpr);
    query(&near, 1290U, &ny, &np, &nr, &npr);
    assert(far.kf_active && near.kf_active);
    close_to(far.hybrid_horizon_s, 0.24f, 1e-5f);
    close_to(near.hybrid_horizon_s, 0.09f, 1e-5f);
    close_to(fy - ny, fr * 0.15f, 1e-4f);
    close_to(ny, (float)near.delayed_kf.yaw.angle + nr * 0.09f - 25.0f, 1e-4f);
}

static void test_uncertainty_and_missing_current_pose_fallback(void)
{
    T_TargetChaseController controller;
    unsigned int i;
    setup(&controller, 3, true);
    for (i = 0U; i < 3U; ++i) {
        const uint64_t timestamp_ms = 1000U + i;
        image_observation(&controller, timestamp_ms + 40U, 30.0f, 25.0f, 1.0f);
        assert(TargetChase_ObserveAlignedBearing(&controller, 30.0f, 0.0f,
                                                 timestamp_ms, timestamp_ms + 40U));
    }
    current_camera(&controller, 1100U, 25.0f, 0.0f);
    assert_same_fallback(&controller, 1100U);
    assert(controller.kf_reason == TARGET_KF_UNCERTAIN);
    assert(controller.kf_variance_deg2 > 4.0f);

    /* A well-observed but implausibly fast bearing also uses the baseline. */
    setup(&controller, 3, true);
    for (i = 0U; i < 3U; ++i) {
        const uint64_t timestamp_ms = 1000U + 100U * i;
        const float fast_yaw = 4.0f * (float)i;
        image_observation(&controller, timestamp_ms + 40U, fast_yaw, 0.0f, 1.0f);
        assert(TargetChase_ObserveAlignedBearing(&controller, fast_yaw, 0.0f,
                                                 timestamp_ms, timestamp_ms + 40U));
    }
    current_camera(&controller, 1250U, 0.0f, 0.0f);
    assert_same_fallback(&controller, 1250U);
    assert(controller.kf_reason == TARGET_KF_UNCERTAIN);
    assert(fabs(controller.delayed_kf.yaw.rate) > 30.0);
    assert(controller.kf_variance_deg2 <= 4.0f);

    stable_three(&controller);
    controller.kf_camera_pose_valid = false;
    assert_same_fallback(&controller, 1250U);
    assert(controller.kf_reason == TARGET_KF_NO_POSE);
    current_camera(&controller, 1149U, 25.0f, 0.0f);
    assert_same_fallback(&controller, 1250U);
    assert(controller.kf_reason == TARGET_KF_NO_POSE);
    current_camera(&controller, 1251U, 25.0f, 0.0f);
    assert_same_fallback(&controller, 1250U);
    assert(controller.kf_reason == TARGET_KF_NO_POSE);
    current_camera(&controller, 1150U, 25.0f, 0.0f);
    {
        T_TargetChaseCommand command;
        assert(TargetChase_Step(&controller, 1250U, &command));
        assert(controller.kf_active); /* The documented 100ms boundary is inclusive. */
    }
}

static void test_rejection_age_expiry_and_target_timeout(void)
{
    T_TargetChaseController controller;
    T_TargetChaseCommand command;
    float yaw, pitch, rate, pitch_rate;
    stable_three(&controller);
    assert(!TargetChase_ObserveAlignedBearing(&controller, 150.0f, 0.0f, 1300U, 1340U));
    assert(controller.kf_reason == TARGET_KF_REJECTED);
    assert(controller.kf_measurement_ms == 1200U);
    assert(controller.delayed_kf.posterior_ms == 1200U);
    assert(controller.target.timestamp_ms == 1240U);
    current_camera(&controller, 1340U, 25.0f, 0.0f);
    query(&controller, 1340U, &yaw, &pitch, &rate, &pitch_rate);
    assert(controller.kf_active);
    assert(controller.kf_reason == TARGET_KF_REJECTED);
    close_to(controller.hybrid_horizon_s, 0.29f, 1e-5f);
    current_camera(&controller, 1701U, 25.0f, 0.0f);
    assert_same_fallback(&controller, 1701U);
    assert(controller.kf_reason == TARGET_KF_STALE);
    assert(controller.delayed_kf.posterior_ms == 1200U);
    current_camera(&controller, 2041U, 25.0f, 0.0f);
    assert(TargetChase_Step(&controller, 2041U, &command));
    assert(controller.mode == TARGET_CHASE_MODE_TARGET_LOST);
    assert(!controller.delayed_kf.has_posterior);
    close_to(command.x_mps, 0.0f, 0.0f);
    close_to(command.yaw_rate_degps, 0.0f, 0.0f);
}

static void test_calibration_expiry_and_warmup_after_gap(void)
{
    T_TargetChaseController controller;
    stable_three(&controller);
    controller.kf_valid_until_ms = 1280U;
    current_camera(&controller, 1281U, 25.0f, 0.0f);
    assert_same_fallback(&controller, 1281U);
    assert(controller.kf_reason == TARGET_KF_NO_CALIBRATION);
    assert(!controller.delayed_kf.has_posterior);
    assert(!TargetChase_ObserveAlignedBearing(&controller, 31.5f, 0.0f, 1300U, 1340U));

    stable_three(&controller);
    image_observation(&controller, 1840U, 34.0f, 25.0f, 1.0f);
    assert(TargetChase_ObserveAlignedBearing(&controller, 34.0f, 0.0f, 1800U, 1840U));
    assert(controller.delayed_kf.accepted_samples == 1U);
    current_camera(&controller, 1850U, 25.0f, 0.0f);
    assert_same_fallback(&controller, 1850U);
    assert(controller.kf_reason == TARGET_KF_WARMUP);
}

static void test_reset_paths_and_old_frame_admission(void)
{
    T_TargetChaseController controller;
    T_TargetChaseTargetPixel latest_target;
    stable_three(&controller);
    latest_target = controller.target;
    image_observation(&controller, 1190U, 30.75f, 22.5f, 1.0f);
    assert(memcmp(&controller.target, &latest_target, sizeof(latest_target)) == 0);
    assert(controller.motion_estimate_timestamp_ms == 1240U);
    assert(TargetChase_ObserveAlignedBearing(&controller, 30.75f, 0.0f, 1150U, 1300U));
    assert(controller.delayed_kf.accepted_samples == 4U);
    assert(controller.kf_measurement_ms == 1200U);
    assert(controller.delayed_kf.posterior_ms == 1200U);
    assert(controller.target.timestamp_ms == 1240U);
    TargetChase_ClearTarget(&controller);
    assert(!controller.target.valid && controller.pixel_samples == 0U);
    assert(!controller.delayed_kf.has_posterior && !controller.kf_active);
    assert(controller.kf_reason == TARGET_KF_RESET);

    stable_three(&controller);
    assert(TargetChase_UpdateImageGeometry(&controller, 640.0f, 360.0f));
    assert(!controller.delayed_kf.has_posterior && !controller.kf_active);
    assert(controller.kf_reason == TARGET_KF_RESET);

    stable_three(&controller);
    TargetChase_SetManualTrigger(&controller, false, 1250U);
    assert(!controller.delayed_kf.has_posterior && !controller.kf_active);
    assert(controller.kf_reason == TARGET_KF_RESET);
    assert(!TargetChase_IsManualTriggerEnabled(&controller));
}

int main(void)
{
    test_missing_calibration_preserves_original_pixel_commands();
    test_warmup_then_real_command_path();
    test_repeat_queries_and_current_camera_coordinates();
    test_near_lead_reduction_preserves_frame_age_compensation();
    test_uncertainty_and_missing_current_pose_fallback();
    test_rejection_age_expiry_and_target_timeout();
    test_calibration_expiry_and_warmup_after_gap();
    test_reset_paths_and_old_frame_admission();
    puts("delayed KF controller integration tests passed (8 groups)");
    return 0;
}
