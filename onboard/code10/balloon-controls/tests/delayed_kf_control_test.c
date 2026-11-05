/* Host integration tests for the real DJI adapter. SDK reads and commands are
 * deterministic stubs; no hardware is connected and no flight is started.
 * Build with -flto -fwhole-program to discard unused SDK initialization code. */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
static FILE *control_test_open(const char *path, const char *mode);
static int control_test_access(const char *path, int mode);
#define fopen control_test_open
#define access control_test_access
#include "../psdk-overlay/samples/sample_c/module_sample/target_chase/dji_target_chase_task.c"
#undef fopen
#undef access
#include <assert.h>

static uint64_t test_now_ms = 10000;
static uint64_t test_clock_read_advance_ms;
static uint64_t test_sdk_read_delay_ms;
static uint64_t test_pose_delay_ms = 20;
static bool test_camera_motion_model;
static float test_camera_yaw, test_camera_pitch;
static unsigned int test_joystick_calls, test_gimbal_calls;
static bool test_pose_available = true;
static T_DjiFlightControllerJoystickCommand test_joystick;
static T_DjiGimbalManagerRotation test_rotation;
static char test_boot_path[96];
static char test_timing_path[96];
static const char test_clock_id[] = "mono-v1:11111111-1111-4111-8111-111111111111";

static FILE *control_test_open(const char *path, const char *mode)
{
    if (!strcmp(path, "/proc/sys/kernel/random/boot_id")) return fopen(test_boot_path, mode);
    if (!strcmp(path, test_timing_path)) return fopen(path, mode);
    /* Logs and unrelated production files must never be written by this test. */
    return NULL;
}

static int control_test_access(const char *path, int mode)
{
    (void)path;
    (void)mode;
    return -1;
}

int clock_gettime(clockid_t clock_id, struct timespec *value)
{
    assert(clock_id == CLOCK_MONOTONIC || clock_id == CLOCK_REALTIME);
    value->tv_sec = (time_t)(test_now_ms / 1000ULL);
    value->tv_nsec = (long)((test_now_ms % 1000ULL) * 1000000ULL);
    if (clock_id == CLOCK_MONOTONIC) test_now_ms += test_clock_read_advance_ms;
    return 0;
}

T_DjiReturnCode DjiFcSubscription_GetLatestValueOfTopic(E_DjiFcSubscriptionTopic topic,
    uint8_t *data, uint16_t size, T_DjiDataTimestamp *stamp)
{
    test_now_ms += test_sdk_read_delay_ms;
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES && !test_pose_available)
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    memset(data, 0, size);
    if (stamp) {
        memset(stamp, 0, sizeof(*stamp));
        stamp->millisecond = (uint32_t)test_now_ms;
        stamp->microsecond = (uint32_t)(test_now_ms * 1000);
    }
    switch (topic) {
    case DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS:
        assert(size == sizeof(T_DjiFcSubscriptionHomePointSetStatus));
        *(T_DjiFcSubscriptionHomePointSetStatus *)data = DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_SUCCESS;
        break;
    case DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED:
        *(T_DjiFcSubscriptionAltitudeFused *)data = 10.0f;
        break;
    case DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION:
        ((T_DjiFcSubscriptionQuaternion *)data)->q0 = 1.0f;
        break;
    case DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT:
        *(T_DjiFcSubscriptionFlightStatus *)data = DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR;
        break;
    case DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES:
        assert(size == sizeof(T_DjiFcSubscriptionGimbalAngles));
        if (test_camera_motion_model)
            test_camera_yaw = ((float)test_now_ms - (float)test_pose_delay_ms - 21000)*0.010f;
        ((T_DjiFcSubscriptionGimbalAngles *)data)->z = test_camera_yaw;
        ((T_DjiFcSubscriptionGimbalAngles *)data)->x = test_camera_pitch;
        break;
    default:
        break;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiFlightController_ExecuteJoystickAction(T_DjiFlightControllerJoystickCommand command)
{
    ++test_joystick_calls;
    test_joystick = command;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiGimbalManager_Rotate(E_DjiMountPosition position, T_DjiGimbalManagerRotation rotation)
{
    assert(position == DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
    ++test_gimbal_calls;
    test_rotation = rotation;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void DjiFlightController_SetJoystickMode(T_DjiFlightControllerJoystickMode mode) { (void)mode; }
T_DjiReturnCode DjiFlightController_ObtainJoystickCtrlAuthority(void) { return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS; }
T_DjiReturnCode DjiFlightController_ReleaseJoystickCtrlAuthority(void) { return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS; }
/* These prediction fixtures exercise the callback-backed path. They must not
 * identify a real aircraft or silently activate the M300 telemetry fallback. */
T_DjiReturnCode DjiAircraftInfo_GetBaseInfo(T_DjiAircraftInfoBaseInfo *info)
{
    (void)info;
    return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
}

static void prepare(int mode)
{
    T_TargetChaseConfig config = TargetChase_DefaultConfig(640, 480, 73.7f, 45.7f);
    T_TargetChaseAircraftState state = {0};
    config.prediction_mode = mode;
    config.prediction_horizon_s = 0.15f;
    config.yaw_deadband_deg = 1;
    config.vertical_bearing_deadband_deg = 1;
    TargetChase_Init(&s_chase, &config);
    state.timestamp_ms = test_now_ms;
    state.in_air = true;
    state.altitude_agl_m = 10;
    TargetChase_UpdateAircraftState(&s_chase, &state);
    TargetChase_SetManualTrigger(&s_chase, true, test_now_ms);
    TargetChase_SetAuthority(&s_chase, true, test_now_ms);
    s_initialized = true;
    s_profile_enabled = false;
    s_no_flight_mode = false;
    s_real_authority_obtained = true;
    s_gimbal_tracking_enabled = true;
    s_gimbal_manager_initialized = true;
    s_gimbal_angles_subscribed = true;
    s_gimbal_static_test_mode = false;
    s_gimbal_search_enabled = false;
    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_TRACKING;
    s_last_gimbal_target_seen_ms = test_now_ms;
    s_last_gimbal_command_ms = s_last_gimbal_status_log_ms = 0;
    s_last_gimbal_pitch_speed_degps = s_last_gimbal_yaw_speed_degps = 0;
    s_last_gimbal_target_near_edge = false;
    s_near_high_rate_tracking_active = false;
    s_camera_pose_count = s_camera_pose_next = 0;
    s_kf_pose_count = s_kf_pose_next = 0;
    s_kf_last_pose_receipt = 0;
    s_raw_frame_ms = 0;
    memset(&s_kf_last_sdk_stamp, 0, sizeof(s_kf_last_sdk_stamp));
    memset(&s_kf_timing, 0, sizeof(s_kf_timing));
    memset(s_kf_clock_id, 0, sizeof(s_kf_clock_id));
    s_kf_timing_reason = "missing_calibration";
    s_last_slew_mono_ms = 0;
    s_has_last_command = false;
    s_start_height_initialized = false;
    test_camera_yaw = test_camera_pitch = 0;
    test_camera_motion_model = false;
    test_sdk_read_delay_ms = 0;
    test_pose_available = true;
    test_joystick_calls = test_gimbal_calls = 0;
    memset(&test_rotation, 0, sizeof(test_rotation));
    memset(&test_joystick, 0, sizeof(test_joystick));
}

static void test_missing_calibration_matches_pixel(void)
{
    T_TargetChaseController pixel;
    T_TargetChaseCommand expected, actual;
    prepare(3);
    for (unsigned i = 0; i < 6; ++i) {
        test_now_ms += 100;
        DjiTargetChase_SetRawObservationPixel(320 + 3*i, 240, test_now_ms - 200);
        assert(DjiTargetChase_OnTargetBox(640, 480, 300 + 3*i, 260,
            340 + 3*i, 220, test_now_ms - 200) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    }
    pixel = s_chase;
    pixel.config.prediction_mode = 1;
    assert(TargetChase_Step(&pixel, test_now_ms, &expected));
    assert(TargetChase_Step(&s_chase, test_now_ms, &actual));
    assert(!s_chase.kf_active);
    assert(s_chase.kf_reason == TARGET_KF_NO_CALIBRATION);
    assert(fabsf(expected.target_bearing_yaw_deg - actual.target_bearing_yaw_deg) < 1e-6f);
    assert(fabsf(expected.target_bearing_vertical_deg - actual.target_bearing_vertical_deg) < 1e-6f);
    assert(fabsf(expected.x_mps - actual.x_mps) < 1e-6f);
    assert(fabsf(expected.yaw_rate_degps - actual.yaw_rate_degps) < 1e-6f);
    assert(DjiTargetChase_RunOnce(test_now_ms) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(test_joystick_calls > 0 && test_gimbal_calls > 0);
    assert(isfinite(test_joystick.x) && isfinite(test_rotation.yaw));
}

static void write_timing_fixture(const char *verified, const char *clock_id,
    const char *extra, uint64_t valid_until)
{
    FILE *fixture = fopen(test_timing_path, "wb");
    assert(fixture != NULL);
    assert(fprintf(fixture,
        "{\"schema\":\"host_receipt_delays_v1\",\"verified\":%s,"
        "\"clock_id\":\"%s\",\"calibration_id\":\"synthetic-host-test-only\","
        "\"video_delay_ms\":120,\"pose_delay_ms\":%llu,\"uncertainty_ms\":5,"
        "\"valid_from_ms\":1000,\"valid_until_ms\":%llu%s}",
        verified, clock_id, (unsigned long long)test_pose_delay_ms,
        (unsigned long long)valid_until, extra) > 0);
    assert(fclose(fixture) == 0);
#ifdef _WIN32
    assert(_putenv_s("BALLOON_PREDICTION_TIMING_FILE", test_timing_path) == 0);
#else
    assert(setenv("BALLOON_PREDICTION_TIMING_FILE", test_timing_path, 1) == 0);
#endif
}

static void test_timing_file_validation(void)
{
    prepare(3);
    write_timing_fixture("false", test_clock_id, "", 1000000);
    assert(!DjiTargetChase_LoadKfTiming(test_now_ms));
    write_timing_fixture("true", "mono-v1:22222222-2222-4222-8222-222222222222", "", 1000000);
    assert(!DjiTargetChase_LoadKfTiming(test_now_ms));
    write_timing_fixture("true", test_clock_id, ",\"pose_delay_ms\":0", 1000000);
    assert(!DjiTargetChase_LoadKfTiming(test_now_ms));
    write_timing_fixture("true", test_clock_id, "", test_now_ms - 1);
    assert(!DjiTargetChase_LoadKfTiming(test_now_ms));
    write_timing_fixture("true", test_clock_id, "", 1000000);
    assert(DjiTargetChase_LoadKfTiming(test_now_ms));
    assert(s_kf_timing.enabled && s_kf_timing.verified);
    assert(s_chase.kf_valid_until_ms == 1000000);
}

static void feed_tracking_with_current_pose(bool expect_active)
{
    /* The camera follows a line of sight moving at 10 degrees/s, so the target
     * stays centered in each delayed image. A pixel-only estimate sees zero
     * motion; aligned bearing observations recover the moving world bearing. */
    uint64_t next_pose_time = 20950;
    test_now_ms = 20000;
    prepare(3);
    test_camera_motion_model = true;
    write_timing_fixture("true", test_clock_id, "", 1000000);
    assert(DjiTargetChase_LoadKfTiming(test_now_ms));
    for (unsigned i = 0; i < 8; ++i) {
        uint64_t measurement = 21000 + 200*i;
        uint64_t arrival = measurement + 220;
        for (; next_pose_time <= arrival - test_pose_delay_ms; next_pose_time += 50) {
            test_now_ms = next_pose_time + test_pose_delay_ms;
            test_camera_yaw = ((float)next_pose_time - 21000)*0.010f;
            assert(DjiTargetChase_UpdateGimbalBoresight(test_now_ms, NULL));
        }
        test_now_ms = arrival;
        DjiTargetChase_SetRawObservationPixel(320, 240, measurement + 120);
        assert(DjiTargetChase_OnTargetBox(640, 480, 300, 260, 340, 220,
            measurement + 120) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        assert(s_chase.kf_measurement_ms == measurement);
        assert(s_chase.target.timestamp_ms == measurement + 120);
        assert(s_raw_frame_ms == 0);
    }
    assert(s_chase.delayed_kf.accepted_samples == 8);
    assert(s_chase.kf_active == expect_active);
    assert(s_chase.kf_reason == (expect_active ? TARGET_KF_ACTIVE : TARGET_KF_NO_POSE));
}

static void feed_aligned_tracking(void) { feed_tracking_with_current_pose(true); }

static void test_adapter_drives_body_gimbal_and_coast(void)
{
    T_TargetChaseController pixel;
    T_TargetChaseCommand expected, actual;
    uint64_t last_target;
    unsigned int body_before;
    feed_aligned_tracking();
    pixel = s_chase;
    pixel.config.prediction_mode = 1;
    assert(TargetChase_Step(&pixel, test_now_ms, &expected));
    assert(TargetChase_Step(&s_chase, test_now_ms, &actual));
    assert(actual.target_bearing_yaw_deg > expected.target_bearing_yaw_deg + 0.4f);
    assert(fabsf(s_chase.hybrid_yaw_rate - 10.0f) < 0.2f);
    assert(test_gimbal_calls > 0 && test_rotation.yaw > 0.1f);
    assert(DjiTargetChase_RunOnce(test_now_ms) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(test_joystick_calls > 0 && s_chase.kf_active);
    assert(fabsf(s_last_command.target_bearing_yaw_deg - actual.target_bearing_yaw_deg) < 1e-5f);
    last_target = s_chase.target.timestamp_ms;
    body_before = test_joystick_calls;
    test_now_ms += 50;
    assert(DjiTargetChase_RepeatLastCommand(test_now_ms) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(test_joystick_calls > body_before && s_chase.kf_active);
    assert(s_chase.target.timestamp_ms == last_target);
    assert(s_last_command.target_bearing_yaw_deg > actual.target_bearing_yaw_deg + 0.1f);

    /* A delayed duplicate cannot refresh latest-target freshness or add a KF
     * observation. It also consumes staged raw data, preventing accidental reuse. */
    DjiTargetChase_SetRawObservationPixel(100, 100, last_target);
    assert(DjiTargetChase_OnTargetBox(640, 480, 80, 120, 120, 80, last_target)
        == DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER);
    assert(s_chase.target.timestamp_ms == last_target && s_raw_frame_ms == 0);
    assert(s_chase.delayed_kf.accepted_samples == 8);

    /* Absence of the raw matched observation must not feed an EMA-smoothed
     * controller box into the timing-aligned estimator. */
    test_now_ms += 100;
    assert(DjiTargetChase_OnTargetBox(640, 480, 300, 260, 340, 220, last_target + 100)
        == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(!s_chase.kf_active && s_chase.kf_reason == TARGET_KF_NO_POSE);
    assert(s_chase.delayed_kf.accepted_samples == 0);
}

static void test_pose_loss_and_stage_reset(void)
{
    uint64_t before;
    feed_aligned_tracking();
    before = s_chase.kf_measurement_ms;
    test_pose_available = false;
    test_now_ms += 200;
    DjiTargetChase_SetRawObservationPixel(320, 240, test_now_ms - 100);
    assert(DjiTargetChase_OnTargetBox(640, 480, 300, 260, 340, 220,
        test_now_ms - 100) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(!s_chase.kf_active && s_chase.kf_reason == TARGET_KF_NO_POSE);
    assert(s_chase.kf_measurement_ms == 0 && before > 0);
    feed_aligned_tracking();
    assert(DjiTargetChase_RequestSearch(test_now_ms, false) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(!s_chase.target.valid && !s_chase.kf_active);
    assert(s_chase.delayed_kf.accepted_samples == 0);
    assert(!TargetChase_ObserveAlignedBearing(&s_chase, 0, 0,
        test_now_ms - 100, test_now_ms));
}

static void test_real_control_clock_progress(void)
{
    uint32_t samples;
    feed_aligned_tracking();
    samples = s_chase.delayed_kf.accepted_samples;
    /* SDK wrapper clocks can cross a millisecond boundary between reading a
     * pose and storing it. This must not look like a backwards pose receipt. */
    test_clock_read_advance_ms = 1;
    assert(DjiTargetChase_UpdateGimbalBoresight(test_now_ms, NULL));
    test_clock_read_advance_ms = 0;
    assert(s_chase.delayed_kf.accepted_samples == samples);
    assert(s_chase.kf_active);
    /* The caller's loop timestamp precedes OnTargetBox and its observation
     * receipt. Both body entry points must predict at their current clock,
     * rather than silently falling back because loop_start < receipt. */
    assert(DjiTargetChase_RunOnce(test_now_ms - 10) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(s_chase.kf_active);
    test_now_ms += 50;
    assert(DjiTargetChase_RepeatLastCommand(test_now_ms - 10) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(s_chase.kf_active);
    test_sdk_read_delay_ms = 2;
    assert(DjiTargetChase_RunOnce(test_now_ms - 10) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(s_chase.kf_active);
    assert(DjiTargetChase_RepeatLastCommand(test_now_ms - 10) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    assert(s_chase.kf_active);
    test_sdk_read_delay_ms = 0;
}

static void test_current_camera_pose_compensation(void)
{
    float y, p, yr, pr;
    T_TargetChaseCommand command;
    test_pose_delay_ms = 50;
    feed_aligned_tracking();
    assert(s_chase.kf_camera_pose_valid);
    assert(fabsf(s_chase.kf_camera_yaw_deg - test_camera_yaw - 0.5f) < 0.02f);
    assert(TargetChase_PredictHybrid(&s_chase, test_now_ms, test_camera_yaw, 0,
        &y, &p, &yr, &pr));
    /* Both camera and target turn at 10 degrees/s. Correcting the camera's
     * 50ms receipt lag leaves only the 0.15s actuator lead, i.e. 1.5 degrees. */
    assert(s_chase.kf_active && fabsf(y - 1.5f) < 0.15f);
    assert(TargetChase_Step(&s_chase, test_now_ms, &command));
    assert(fabsf(command.target_bearing_yaw_deg - (test_camera_yaw + 2.0f)) < 0.15f);

    test_pose_available = false;
    test_now_ms += 101;
    assert(!DjiTargetChase_UpdateGimbalBoresight(test_now_ms, NULL));
    assert(TargetChase_Step(&s_chase, test_now_ms, &command));
    assert(!s_chase.kf_active && s_chase.kf_reason == TARGET_KF_NO_POSE);
    test_pose_delay_ms = 150;
    feed_tracking_with_current_pose(false);
    assert(!s_chase.kf_camera_pose_valid);
    assert(TargetChase_Step(&s_chase, test_now_ms, &command));
    assert(!s_chase.kf_active && s_chase.kf_reason == TARGET_KF_NO_POSE);
    assert(fabsf(command.target_bearing_yaw_deg - test_camera_yaw) < 1e-5f);
    test_pose_delay_ms = 20;
}

int main(void)
{
    FILE *boot;
    snprintf(test_boot_path, sizeof(test_boot_path), "delayed_kf_boot_%lu.txt", (unsigned long)getpid());
    snprintf(test_timing_path, sizeof(test_timing_path), "delayed_kf_timing_%lu.json", (unsigned long)getpid());
    boot = fopen(test_boot_path, "wb");
    assert(boot != NULL);
    assert(fputs(test_clock_id + 8, boot) >= 0);
    assert(fclose(boot) == 0);
    test_missing_calibration_matches_pixel();
    test_timing_file_validation();
    test_adapter_drives_body_gimbal_and_coast();
    test_pose_loss_and_stage_reset();
    test_real_control_clock_progress();
    test_current_camera_pose_compensation();
    assert(remove(test_boot_path) == 0);
    assert(remove(test_timing_path) == 0);
    puts("delayed KF adapter: calibration validation, raw measurements, body/gimbal/coast, pixel fallback and reset passed (synthetic SDK)");
    return 0;
}
