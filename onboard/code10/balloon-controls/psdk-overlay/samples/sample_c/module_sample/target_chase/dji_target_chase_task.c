#include "../balloon_widget/balloon_clock.h"
#include "../balloon_widget/balloon_evidence.h"
#include "../balloon_widget/balloon_performance.h"
#include "dji_target_chase_task.h"
#include "target_chase_timing_alignment.h"

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "dji_error.h"
#include "dji_aircraft_info.h"
#include "dji_fc_subscription.h"
#include "dji_flight_controller.h"
#include "dji_gimbal_manager.h"
#include "dji_platform.h"
#include "utils/cJSON.h"
#include "../balloon_widget/balloon_background_telemetry_impl.h"

#define DJI_TARGET_CHASE_RAD_TO_DEG 57.29577951308232f
#define DJI_TARGET_CHASE_DEG_TO_RAD 0.017453292519943295769f
#define DJI_TARGET_CHASE_PI 3.1415926535897932384626433832795
#define DJI_TARGET_CHASE_RID_CONFIG_FILE "/home/uav/offline_balloon_strike_test/rid_info.json"
#define DJI_TARGET_CHASE_RID_CONFIG_MAX_BYTES 4096
#define DJI_TARGET_CHASE_LEGACY_TEST_RID_LATITUDE 22.542812
#define DJI_TARGET_CHASE_LEGACY_TEST_RID_LONGITUDE 113.958902
#define DJI_TARGET_CHASE_LEGACY_TEST_RID_ALTITUDE 0

#ifdef DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT
#define DJI_TARGET_CHASE_ERROR_NONSUPPORT DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT
#else
#define DJI_TARGET_CHASE_ERROR_NONSUPPORT ((T_DjiReturnCode) 0x000000E0)
#endif
#define DJI_TARGET_CHASE_ERROR_JOYSTICK_AUTH_SUCCESS ((T_DjiReturnCode) 0x1B00000000ULL)
#define DJI_TARGET_CHASE_API_MAX_DURATION_MS 150ULL
#define DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS 600ULL
#define DJI_TARGET_CHASE_API_MAX_HORIZONTAL_MPS 0.30f
#define DJI_TARGET_CHASE_API_MAX_VERTICAL_MPS 0.12f
#define DJI_TARGET_CHASE_API_MAX_YAW_DEGPS 12.0f
#define DJI_TARGET_CHASE_API_MIN_HEIGHT_M 2.0f
#define DJI_TARGET_CHASE_API_MAX_HEIGHT_M 10.0f
#define DJI_TARGET_CHASE_CONTROL_EVENT_LOG "/home/uav/offline_balloon_strike_test/control_events.jsonl"
#define DJI_TARGET_CHASE_NEAR_HORIZONTAL_ACCEL_MPS2 0.40f
#define DJI_TARGET_CHASE_FAR_HORIZONTAL_ACCEL_MPS2 0.75f
#define DJI_TARGET_CHASE_HORIZONTAL_DECEL_MPS2 0.35f
#define DJI_TARGET_CHASE_VERTICAL_ACCEL_MPS2 0.24f
#define DJI_TARGET_CHASE_NEAR_YAW_ACCEL_DEGPS2 10.0f
#define DJI_TARGET_CHASE_FAR_YAW_ACCEL_DEGPS2 18.0f
#define DJI_TARGET_CHASE_YAW_DECEL_DEGPS2 8.0f
#define DJI_TARGET_CHASE_YAW_REVERSAL_DEGPS2 30.0f
#define DJI_TARGET_CHASE_NEAR_OFF_AXIS_BOX_ANGLE_DEG 8.0f
#define DJI_TARGET_CHASE_NEAR_OFF_AXIS_BEARING_DEG 15.0f
#define DJI_TARGET_CHASE_NEAR_OFF_AXIS_DECEL_MPS2 1.2f
#define DJI_TARGET_CHASE_SLEW_DEFAULT_DT_S 0.05f
#define DJI_TARGET_CHASE_SLEW_MIN_DT_S 0.01f
#define DJI_TARGET_CHASE_SLEW_MAX_DT_S 0.20f
#define DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1
#define DJI_TARGET_CHASE_GIMBAL_COMMAND_INTERVAL_MS 50ULL
#define DJI_TARGET_CHASE_GIMBAL_STATUS_LOG_INTERVAL_MS 1000ULL
#define DJI_TARGET_CHASE_GIMBAL_SPEED_COMMAND_TIME_S 0.1
#define DJI_TARGET_CHASE_GIMBAL_DEADBAND_DEG 1.0f
#define DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_DEG 3.0f
#define DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_START_DEG 5.0f
#define DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_FULL_DEG 12.0f
#define DJI_TARGET_CHASE_GIMBAL_CENTER_CONFIRMATIONS 3U
#define DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_ZERO 1.5f
#define DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_INFINITY 0.7f
#define DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_SLOPE_ZERO 30.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_MAX_SPEED_DEGPS 40.0f
#define DJI_TARGET_CHASE_GIMBAL_RATE_FEEDFORWARD_GAIN 0.35f
#define DJI_TARGET_CHASE_GIMBAL_NEAR_RATE_FEEDFORWARD_GAIN 0.55f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_ERROR_DEG 2.5f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_MID_ERROR_DEG 8.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_LIMIT_DEGPS 4.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_MID_LIMIT_DEGPS 12.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_FAR_LIMIT_DEGPS 24.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_FILTER_ALPHA 0.25f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_ACTIVE_FILTER_ALPHA 0.45f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_FAR_FILTER_ALPHA 0.65f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_ACCEL_DEGPS2 100.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_ACCEL_DEGPS2 80.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_YAW_ACCEL_DEGPS2 160.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_PITCH_ACCEL_DEGPS2 130.0f
#define DJI_TARGET_CHASE_GIMBAL_CENTER_MOTION_HOLD_DEGPS 0.8f
#define DJI_TARGET_CHASE_NEAR_LOW_LATENCY_MAX_MS 160ULL
#define DJI_TARGET_CHASE_NEAR_HIGH_RATE_BOX_ANGLE_DEG 5.0f
#define DJI_TARGET_CHASE_NEAR_HIGH_RATE_EXIT_BOX_ANGLE_DEG 4.0f
#define DJI_TARGET_CHASE_NEAR_HIGH_RATE_EXIT_LATENCY_MS 280ULL
#define DJI_TARGET_CHASE_NEAR_HIGH_RATE_MIN_HOLD_MS 800ULL
#define DJI_TARGET_CHASE_NEAR_HIGH_RATE_GAIN 1.60f
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_SPEED_DEGPS 30.0f
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_LIMIT_DEG 35.0f
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MIN_DEG (-20.0f)
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MAX_DEG 20.0f
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_SPEED_DEGPS 12.0f
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_POSITION_GAIN 3.0f
#define DJI_TARGET_CHASE_GIMBAL_SEARCH_POSITION_TOLERANCE_DEG 2.0f
#define DJI_TARGET_CHASE_BODY_SEARCH_REQUESTED_OVERLAP_DEG 15.0f
#define DJI_TARGET_CHASE_BODY_SEARCH_YAW_RATE_DEGPS 10.0f
#define DJI_TARGET_CHASE_BODY_SEARCH_YAW_TOLERANCE_DEG 3.0f
#define DJI_TARGET_CHASE_BODY_SEARCH_TURN_TIMEOUT_MS 22000ULL
#define DJI_TARGET_CHASE_BODY_SEARCH_GIMBAL_CENTER_GAIN 1.5f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_SOFT_LIMIT_DEG 60.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_SOFT_LIMIT_UP_DEG 15.0f
#define DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_SOFT_LIMIT_DOWN_DEG (-90.0f)
#define DJI_TARGET_CHASE_GIMBAL_TARGET_LOST_TIMEOUT_MS 1600ULL
#define DJI_TARGET_CHASE_GIMBAL_EDGE_TARGET_LOST_TIMEOUT_MS 2400ULL
#define DJI_TARGET_CHASE_GIMBAL_NEAR_TARGET_LOST_TIMEOUT_MS 3200ULL
#define DJI_TARGET_CHASE_GIMBAL_NEAR_BOX_ANGLE_DEG 5.0f
#define DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_START_MS 450ULL
#define DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_FORWARD_MS 750ULL
#define DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_TOTAL_MS 2400ULL
#define DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_YAW_SPEED_DEGPS 18.0f
#define DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_PITCH_SPEED_DEGPS 10.0f
#define DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_DURATION_MS 900ULL
#define DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_LATERAL_MPS 0.15f
#define DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_YAW_DEGPS 5.0f
#define DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED 0.15f
#define DJI_TARGET_CHASE_GIMBAL_RESET_SETTLE_MS 1500ULL
#define DJI_TARGET_CHASE_GIMBAL_RESET_TOLERANCE_DEG 2.0f
#define DJI_TARGET_CHASE_GIMBAL_SMOOTH_FACTOR 25U
#define DJI_TARGET_CHASE_GIMBAL_MAX_SPEED_PERCENT 80U

typedef enum {
    DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT = 0,
    DJI_TARGET_CHASE_SEARCH_SWEEP_LOWER_RIGHT,
    DJI_TARGET_CHASE_SEARCH_SEEK_UPPER_RIGHT,
    DJI_TARGET_CHASE_SEARCH_SWEEP_UPPER_LEFT,
    DJI_TARGET_CHASE_SEARCH_WAIT_BODY_TURN,
} E_DjiTargetChaseSearchPhase;

static T_TargetChaseController s_chase;
/* One atomic word keeps provider, reason, dry-run flag and timestamp coherent
 * for the UI thread. 48-bit milliseconds wraps only after thousands of years;
 * masked subtraction handles the wrap without sharing non-atomic state. */
static uint64_t s_prediction_status_snapshot;
#define DJI_TARGET_CHASE_STATUS_TIME_MASK UINT64_C(0xFFFFFFFFFFFF)
enum {
    PREDICTION_STATUS_PIXEL = 1, PREDICTION_STATUS_WORLD,
    PREDICTION_STATUS_KF, PREDICTION_STATUS_HYBRID, PREDICTION_STATUS_GROUND,
    PREDICTION_STATUS_WORLD_MISSING = 32, PREDICTION_STATUS_BLEND_ZERO
};

static void DjiTargetChase_ClearPredictionStatus(void)
{
    __atomic_store_n(&s_prediction_status_snapshot, UINT64_C(0), __ATOMIC_RELEASE);
}
#define DJI_TARGET_CHASE_POSE_HISTORY_COUNT 96U
#define DJI_TARGET_CHASE_POSE_MAX_GAP_MS 100ULL
#define DJI_TARGET_CHASE_POSE_NEAREST_MAX_MS 25ULL
typedef struct {
    uint64_t timestamp_ms;
    float world_yaw_deg;
    float world_pitch_deg;
} T_DjiTargetChaseCameraPose;
static T_DjiTargetChaseCameraPose
    s_camera_pose_history[DJI_TARGET_CHASE_POSE_HISTORY_COUNT];
/* Raw SDK stamp retained separately: its epoch/field semantics must be
 * verified before it can replace host-read time in the active controller. */
static T_DjiDataTimestamp s_camera_sdk_stamp;
static uint64_t s_camera_read_begin_ms;
static uint64_t s_camera_read_end_ms;
static unsigned int s_camera_pose_next = 0;
static unsigned int s_camera_pose_count = 0;
static T_TargetTimingAlignment s_kf_timing;
static char s_kf_clock_id[80];
static const char *s_kf_timing_reason = "not_loaded";
static T_TargetTimingPose s_kf_poses[DJI_TARGET_CHASE_POSE_HISTORY_COUNT];
static unsigned int s_kf_pose_count, s_kf_pose_next;
static T_DjiDataTimestamp s_kf_last_sdk_stamp;
static uint64_t s_kf_last_pose_receipt;
static float s_raw_u, s_raw_v;
static uint64_t s_raw_frame_ms;
static bool s_initialized = false;
static bool s_api_motion_only;
enum {
    API_HOME_STATUS, API_HOME_ALTITUDE, API_ALTITUDE, API_VELOCITY,
    API_QUATERNION, API_ANGULAR_RATE, API_FLIGHT_STATUS, API_STATE_TOPIC_COUNT
};
static T_BalloonTelemetryProgress s_api_state_progress[API_STATE_TOPIC_COUNT];
static const uint64_t s_api_state_max_age_ms[API_STATE_TOPIC_COUNT] = {
    [API_HOME_STATUS] = 1500, [API_HOME_ALTITUDE] = 1500,
    [API_ALTITUDE] = 200, [API_VELOCITY] = 200, [API_QUATERNION] = 200,
    [API_ANGULAR_RATE] = 200, [API_FLIGHT_STATUS] = 250,
};
static uint64_t s_last_now_ms = 0;
static T_TargetChaseCommand s_last_command;
static bool s_has_last_command = false;
static bool s_no_flight_mode = false;
/* -1 keeps the legacy environment contract for non-panel callers. A managed
 * panel explicitly selects maintenance at startup; changes are process-local
 * and atomic, never mutations of the multithreaded process environment. */
static int s_panel_maintenance_mode = -1;
static bool s_real_authority_obtained = false;
/* SDK callbacks may run outside the detector thread. The atomic latch closes
 * every command/obtain path before ordinary controller state is touched. */
static bool s_control_inhibited = false;
static bool s_manual_takeover = false;
static bool s_ever_had_authority = false;
static bool s_intentional_release = false;
static T_DjiReturnCode s_last_stop_result = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
/* M300/M350 firmware may not implement the native authority-event API.
 * CONTROL_DEVICE is only decoded for those two models. SDK timestamps have
 * an aircraft epoch; freshness uses local receipt time, never epoch subtraction.
 * Only advancing SDK stamps refresh receipt time. Two samples establish a
 * live subscription before it is accepted as a takeover monitor. */
#define DJI_TARGET_CHASE_CONTROL_DEVICE_FRESH_MS 150ULL
#define DJI_TARGET_CHASE_CONTROL_DEVICE_WAIT_MS 500ULL
typedef struct {
    bool enabled, subscribed, ready, valid, have_stamp;
    bool request_issued, awaiting_grant, confirmed;
    uint8_t device;
    bool open;
    T_DjiDataTimestamp stamp;
    uint64_t receipt_ms, generation, request_generation;
    unsigned int samples;
    const char *pending_event;
} T_DjiTargetChaseControlDeviceMonitor;
static T_DjiTargetChaseControlDeviceMonitor s_control_device;
static pthread_mutex_t s_control_device_mutex = PTHREAD_MUTEX_INITIALIZER;
static bool DjiTargetChase_ControlDeviceHealthy(bool require_owner);

void DjiTargetChase_RequestStop(void)
{
    /* Called synchronously by the operator-stop path, including while an
     * SDK request is waiting for control-device confirmation. */
    __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
    DjiTargetChase_ClearPredictionStatus();
}

bool DjiTargetChase_IsControlInhibited(void)
{
    (void)DjiTargetChase_ControlDeviceHealthy(false);
    return __atomic_load_n(&s_control_inhibited, __ATOMIC_ACQUIRE);
}

bool DjiTargetChase_IsReleasePending(void)
{
    return DjiTargetChase_IsControlInhibited() &&
           __atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE);
}

T_DjiReturnCode DjiTargetChase_GetLastStopResult(void)
{
    return __atomic_load_n(&s_last_stop_result, __ATOMIC_ACQUIRE);
}
static uint64_t s_last_repeat_command_log_ms = 0;
static uint64_t s_last_slew_mono_ms = 0;
static bool s_start_height_initialized = false;
static float s_start_height_m = 0.0f;
static bool s_gimbal_manager_initialized = false;
static bool s_gimbal_angles_subscribed = false;
static bool s_gimbal_tracking_enabled = false;
static bool s_gimbal_static_test_mode = false;
static bool s_gimbal_search_enabled = false;
static uint64_t s_last_gimbal_command_ms = 0;
static uint64_t s_last_gimbal_status_log_ms = 0;
static uint64_t s_last_gimbal_target_seen_ms = 0;
static bool s_last_gimbal_target_near_edge = false;
static uint64_t s_gimbal_edge_recovery_started_mono_ms = 0;
static float s_gimbal_edge_recovery_yaw_direction = 0.0f;
static float s_gimbal_edge_recovery_pitch_direction = 0.0f;
static uint64_t s_gimbal_reset_started_ms = 0;
static float s_last_gimbal_pitch_speed_degps = 0.0f;
static float s_last_gimbal_yaw_speed_degps = 0.0f;
static bool s_near_high_rate_tracking_active = false;
static uint64_t s_near_high_rate_changed_mono_ms = 0;
static float s_gimbal_search_center_yaw_deg = 0.0f;
static float s_gimbal_search_center_pitch_deg = 0.0f;
static float s_gimbal_search_yaw_speed_degps =
    DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_SPEED_DEGPS;
static float s_gimbal_search_pitch_speed_degps =
    DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_SPEED_DEGPS;
static float s_body_search_yaw_rate_degps =
    DJI_TARGET_CHASE_BODY_SEARCH_YAW_RATE_DEGPS;
static unsigned int s_gimbal_centered_observations = 0;
static E_DjiTargetChaseGimbalState s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_DISABLED;
static E_DjiTargetChaseSearchPhase s_search_phase =
    DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT;
static T_TargetChaseSearchCoverage s_search_coverage;
static bool s_body_search_enabled = false;
static bool s_body_search_turn_active = false;
static bool s_body_search_turn_heading_valid = false;
static float s_body_search_turn_start_yaw_deg = 0.0f;
static uint64_t s_body_search_turn_start_mono_ms = 0;
static uint64_t s_last_body_search_log_mono_ms = 0;
static unsigned int s_search_sector_index = 0;
static bool s_search_exhausted = false;

static T_DjiReturnCode DjiTargetChase_ReadState(T_TargetChaseAircraftState *state);
static uint64_t DjiTargetChase_MonotonicMs(void);
static uint64_t DjiTargetChase_NowMs(void);
static void DjiTargetChase_LogEvent(const char *event,
                                    uint64_t now_ms,
                                    T_DjiReturnCode code,
                                    const T_TargetChaseCommand *command);

static float DjiTargetChase_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/*
 * ViSP-style adaptive visual-servo gain:
 * lambda(x) = (lambda(0)-lambda(inf))*exp(-b*x) + lambda(inf),
 * b = lambda'(0)/(lambda(0)-lambda(inf)). The parameters follow the
 * mature ViSP ROS Bebop visual-servo example; x is angular image error in rad.
 * Source: https://docs.ros.org/en/noetic/api/visp_ros/html/visual__servo__nodelet_8cpp_source.html
 */
static float DjiTargetChase_GimbalAdaptiveGain(float error_norm_rad)
{
    const float gainRange = DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_ZERO -
                            DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_INFINITY;
    const float exponentScale = DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_SLOPE_ZERO / gainRange;

    return gainRange * expf(-exponentScale * fabsf(error_norm_rad)) +
           DJI_TARGET_CHASE_GIMBAL_ADAPTIVE_GAIN_INFINITY;
}

static float DjiTargetChase_GimbalTrackingSpeedLimit(float error_deg,
                                                      float target_rate_degps,
                                                      bool near_low_latency)
{
    float limit_degps;
    float feedforward_limit_degps;

    if (error_deg <= DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_ERROR_DEG) {
        limit_degps = DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_LIMIT_DEGPS;
    } else if (error_deg <= DJI_TARGET_CHASE_GIMBAL_TRACK_MID_ERROR_DEG) {
        limit_degps = DJI_TARGET_CHASE_GIMBAL_TRACK_MID_LIMIT_DEGPS;
    } else {
        limit_degps = DJI_TARGET_CHASE_GIMBAL_TRACK_FAR_LIMIT_DEGPS;
    }

    /*
     * A moving target near the image center still needs feed-forward speed.
     * Raise the cap only when measured image motion demands it.
     */
    feedforward_limit_degps = fabsf(target_rate_degps) * 1.25f + 2.0f;
    if (feedforward_limit_degps > limit_degps) {
        limit_degps = feedforward_limit_degps;
    }
    if (near_low_latency) {
        limit_degps *= DJI_TARGET_CHASE_NEAR_HIGH_RATE_GAIN;
    }
    return DjiTargetChase_Clamp(limit_degps,
                                DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_LIMIT_DEGPS,
                                DJI_TARGET_CHASE_GIMBAL_TRACK_MAX_SPEED_DEGPS);
}

static float DjiTargetChase_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void DjiTargetChase_RecordCameraPose(uint64_t timestamp_ms,
                                            float world_yaw_deg,
                                            float world_pitch_deg)
{
    T_DjiTargetChaseCameraPose *pose;
    static uint64_t last_record_ms = 0;
    if (timestamp_ms == 0 || !isfinite(world_yaw_deg) ||
        !isfinite(world_pitch_deg)) {
        return;
    }
    if (s_chase.config.prediction_mode == 3) {
        uint64_t measurement_ms;
        E_TargetTimingReason why = TargetTiming_MapPose(&s_kf_timing, s_kf_clock_id,
            timestamp_ms, timestamp_ms, timestamp_ms, &measurement_ms);
        s_kf_timing_reason = TargetTiming_ReasonName(why);
        /* The raw SDK stamp is an identity/progress check only. Its unknown
         * unit/epoch is never guessed or substituted for a measurement time. */
        if (why == TARGET_TIMING_OK &&
            (s_camera_sdk_stamp.millisecond || s_camera_sdk_stamp.microsecond) &&
            (s_camera_sdk_stamp.millisecond != s_kf_last_sdk_stamp.millisecond ||
             s_camera_sdk_stamp.microsecond != s_kf_last_sdk_stamp.microsecond) &&
            world_pitch_deg >= -90 && world_pitch_deg <= 90 &&
            world_yaw_deg >= -180 && world_yaw_deg <= 180) {
            T_TargetTimingPose *mapped = &s_kf_poses[s_kf_pose_next];
            mapped->timestamp_ms = measurement_ms;
            mapped->received_ms = timestamp_ms;
            mapped->yaw_deg = world_yaw_deg;
            mapped->pitch_deg = world_pitch_deg;
            s_kf_last_sdk_stamp = s_camera_sdk_stamp;
            s_kf_last_pose_receipt = timestamp_ms;
            s_kf_pose_next = (s_kf_pose_next+1U)%DJI_TARGET_CHASE_POSE_HISTORY_COUNT;
            if (s_kf_pose_count < DJI_TARGET_CHASE_POSE_HISTORY_COUNT) ++s_kf_pose_count;
        }
    }
    pose = &s_camera_pose_history[s_camera_pose_next];
    pose->timestamp_ms = timestamp_ms;
    pose->world_yaw_deg = DjiTargetChase_Wrap180(world_yaw_deg);
    pose->world_pitch_deg = world_pitch_deg;
    s_camera_pose_next =
        (s_camera_pose_next + 1U) % DJI_TARGET_CHASE_POSE_HISTORY_COUNT;
    if (s_camera_pose_count < DJI_TARGET_CHASE_POSE_HISTORY_COUNT) {
        s_camera_pose_count++;
    }
    /* Optional experiment telemetry. Keep it out of the normal control path. */
    if (timestamp_ms >= last_record_ms + 100 &&
        access("/tmp/balloon_prediction_record.request", F_OK) == 0) {
        char clock_id[80];
        FILE *record = BalloonClock_Id(clock_id)
            ? fopen("/home/uav/offline_balloon_strike_test/prediction_camera_pose_v2.csv", "a") : NULL;
        if (record != NULL) {
            (void) fseek(record, 0, SEEK_END);
            if (ftell(record) == 0) {
                fprintf(record, "ts_ms,yaw_deg,pitch_deg,clock_id\n");
            }
            fprintf(record, "%llu,%.6f,%.6f,%s\n",
                    (unsigned long long) timestamp_ms,
                    pose->world_yaw_deg, pose->world_pitch_deg, clock_id);
            fclose(record);
            /* Versioned raw timing evidence; never reinterpret it as exposure. */
            record = fopen("/home/uav/offline_balloon_strike_test/prediction_camera_timing_v3.csv", "a");
            if (record != NULL) {
                (void) fseek(record, 0, SEEK_END);
                if (ftell(record) == 0) {
                    fprintf(record, "host_read_begin_ms,host_read_end_ms,sdk_millisecond_raw,sdk_microsecond_raw,yaw_deg,pitch_deg,clock_id\n");
                }
                fprintf(record, "%llu,%llu,%u,%u,%.6f,%.6f,%s\n",
                    (unsigned long long)s_camera_read_begin_ms,
                    (unsigned long long)s_camera_read_end_ms,
                    (unsigned)s_camera_sdk_stamp.millisecond,
                    (unsigned)s_camera_sdk_stamp.microsecond,
                    pose->world_yaw_deg, pose->world_pitch_deg, clock_id);
                fclose(record);
            }
            last_record_ms = timestamp_ms;
        }
    }
}

static bool DjiTargetChase_CameraPoseAt(uint64_t timestamp_ms,
                                        float *world_yaw_deg,
                                        float *world_pitch_deg)
{
    const T_DjiTargetChaseCameraPose *before = NULL;
    const T_DjiTargetChaseCameraPose *after = NULL;
    unsigned int i;
    if (timestamp_ms == 0 || world_yaw_deg == NULL ||
        world_pitch_deg == NULL) {
        return false;
    }
    for (i = 0; i < s_camera_pose_count; ++i) {
        const T_DjiTargetChaseCameraPose *pose = &s_camera_pose_history[i];
        if (pose->timestamp_ms <= timestamp_ms &&
            (before == NULL || pose->timestamp_ms > before->timestamp_ms)) {
            before = pose;
        }
        if (pose->timestamp_ms >= timestamp_ms &&
            (after == NULL || pose->timestamp_ms < after->timestamp_ms)) {
            after = pose;
        }
    }
    if (before != NULL && after != NULL &&
        timestamp_ms - before->timestamp_ms <= DJI_TARGET_CHASE_POSE_MAX_GAP_MS &&
        after->timestamp_ms - timestamp_ms <= DJI_TARGET_CHASE_POSE_MAX_GAP_MS) {
        if (after->timestamp_ms > before->timestamp_ms) {
            float fraction = (float) (timestamp_ms - before->timestamp_ms) /
                             (float) (after->timestamp_ms - before->timestamp_ms);
            *world_yaw_deg = DjiTargetChase_Wrap180(
                before->world_yaw_deg + fraction * DjiTargetChase_Wrap180(
                    after->world_yaw_deg - before->world_yaw_deg));
            *world_pitch_deg = before->world_pitch_deg + fraction *
                (after->world_pitch_deg - before->world_pitch_deg);
        } else {
            *world_yaw_deg = before->world_yaw_deg;
            *world_pitch_deg = before->world_pitch_deg;
        }
        return true;
    }
    if (before != NULL &&
        timestamp_ms - before->timestamp_ms <=
            DJI_TARGET_CHASE_POSE_NEAREST_MAX_MS) {
        *world_yaw_deg = before->world_yaw_deg;
        *world_pitch_deg = before->world_pitch_deg;
        return true;
    }
    if (after != NULL &&
        after->timestamp_ms - timestamp_ms <=
            DJI_TARGET_CHASE_POSE_NEAREST_MAX_MS) {
        *world_yaw_deg = after->world_yaw_deg;
        *world_pitch_deg = after->world_pitch_deg;
        return true;
    }
    return false;
}

/* Read once per controller initialization, never perform file IO per frame. */
static bool DjiTargetChase_LoadKfTiming(uint64_t now_ms)
{
    const char *path = getenv("BALLOON_PREDICTION_TIMING_FILE");
    char text[4097];
    FILE *fp;
    size_t n;
    cJSON *root, *item;
    bool ok = false;
    TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_NO_CALIBRATION);
    s_chase.kf_valid_until_ms = 0;
    s_chase.kf_camera_pose_valid = false;
    s_kf_pose_count = s_kf_pose_next = 0;
    s_kf_last_pose_receipt = 0;
    memset(&s_kf_last_sdk_stamp, 0, sizeof(s_kf_last_sdk_stamp));
    memset(&s_kf_timing, 0, sizeof(s_kf_timing));
    memset(s_kf_clock_id, 0, sizeof(s_kf_clock_id));
    s_kf_timing_reason = "missing_calibration";
    if (s_chase.config.prediction_mode != 3 || !path || !*path ||
        !BalloonClock_Id(s_kf_clock_id)) return false;
    fp = fopen(path, "rb");
    if (!fp) return false;
    n = fread(text, 1, sizeof(text)-1, fp);
    if (ferror(fp) || n == sizeof(text)-1) { fclose(fp); return false; }
    fclose(fp); text[n] = '\0';
    s_kf_timing_reason = "invalid_calibration_file";
    root = cJSON_ParseWithOpts(text, NULL, 1);
    if (!root || !cJSON_IsObject(root)) { cJSON_Delete(root); return false; }
    /* Do not accept ambiguous duplicate keys. */
    for (item = root->child; item; item = item->next) {
        cJSON *other;
        for (other = item->next; other; other = other->next)
            if (item->string && other->string && !strcmp(item->string, other->string)) goto done;
    }
    item = cJSON_GetObjectItemCaseSensitive(root, "schema");
    if (!cJSON_IsString(item) || strcmp(item->valuestring, "host_receipt_delays_v1")) goto done;
    item = cJSON_GetObjectItemCaseSensitive(root, "verified");
    if (!cJSON_IsTrue(item)) goto done;
    item = cJSON_GetObjectItemCaseSensitive(root, "clock_id");
    if (!cJSON_IsString(item) || strlen(item->valuestring) >= sizeof(s_kf_timing.clock_id)) goto done;
    strcpy(s_kf_timing.clock_id, item->valuestring);
    item = cJSON_GetObjectItemCaseSensitive(root, "calibration_id");
    if (!cJSON_IsString(item) || !*item->valuestring || strlen(item->valuestring) >= 80 ||
        strspn(item->valuestring, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-") != strlen(item->valuestring)) goto done;
    strcpy(s_kf_timing.calibration_id, item->valuestring);
#define READ_TIMING_DOUBLE(name) \
    item = cJSON_GetObjectItemCaseSensitive(root, #name); \
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) goto done; \
    s_kf_timing.name = item->valuedouble
    READ_TIMING_DOUBLE(video_delay_ms);
    READ_TIMING_DOUBLE(pose_delay_ms);
    READ_TIMING_DOUBLE(uncertainty_ms);
#undef READ_TIMING_DOUBLE
#define READ_TIMING_MS(name) \
    item = cJSON_GetObjectItemCaseSensitive(root, #name); \
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble) || \
        item->valuedouble < 1 || item->valuedouble > 9007199254740991.0 || \
        floor(item->valuedouble) != item->valuedouble) goto done; \
    s_kf_timing.name = (uint64_t)item->valuedouble
    READ_TIMING_MS(valid_from_ms);
    READ_TIMING_MS(valid_until_ms);
#undef READ_TIMING_MS
    s_kf_timing.enabled = s_kf_timing.verified = true;
    {
        E_TargetTimingReason why = TargetTiming_Validate(&s_kf_timing, s_kf_clock_id, now_ms);
        s_kf_timing_reason = TargetTiming_ReasonName(why);
        ok = why == TARGET_TIMING_OK;
    }
done:
    cJSON_Delete(root);
    if (!ok) { s_kf_timing.enabled = false; return false; }
    s_chase.kf_valid_until_ms = s_kf_timing.valid_until_ms;
    return true;
}

void DjiTargetChase_SetRawObservationPixel(float u, float v, uint64_t frame_ms)
{
    s_raw_u = u; s_raw_v = v; s_raw_frame_ms = frame_ms;
}

static bool DjiTargetChase_TargetAfterKfEpoch(uint64_t frame_ms)
{
    uint64_t measurement_ms, now_ms;
    if (s_chase.config.prediction_mode != 3 || !s_chase.kf_epoch_ms) return true;
    now_ms = DjiTargetChase_NowMs();
    if (TargetTiming_MapVideo(&s_kf_timing, s_kf_clock_id, frame_ms, now_ms,
                             now_ms, &measurement_ms) == TARGET_TIMING_OK)
        return measurement_ms > s_chase.kf_epoch_ms;
    return frame_ms > s_chase.kf_epoch_ms;
}

static void DjiTargetChase_ObserveKf(float u, float v, uint64_t frame_ms)
{
    uint64_t measurement_ms, received_ms = DjiTargetChase_NowMs();
    float yaw, pitch;
    E_TargetTimingReason why;
    if (s_chase.config.prediction_mode != 3) return;
    why = TargetTiming_MapVideo(&s_kf_timing, s_kf_clock_id, frame_ms,
                                received_ms, received_ms, &measurement_ms);
    s_kf_timing_reason = TargetTiming_ReasonName(why);
    if (why != TARGET_TIMING_OK) {
        TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_NO_CALIBRATION);
        return;
    }
    if (!isfinite(u) || !isfinite(v) || u < 0 || v < 0 ||
        s_chase.config.fx_px <= 1 || s_chase.config.fy_px <= 1 ||
        u >= s_chase.config.image_width_px || v >= s_chase.config.image_height_px ||
        !s_kf_last_pose_receipt || received_ms < s_kf_last_pose_receipt ||
        received_ms-s_kf_last_pose_receipt > 100 ||
        !TargetTiming_PoseAt(s_kf_poses, s_kf_pose_count, measurement_ms,
                            received_ms, &yaw, &pitch)) {
        TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_NO_POSE);
        return;
    }
    yaw += atanf((u-s_chase.config.cx_px)/s_chase.config.fx_px)*
        DJI_TARGET_CHASE_RAD_TO_DEG*s_chase.config.pixel_x_to_yaw_sign;
    pitch += atanf((v-s_chase.config.cy_px)/s_chase.config.fy_px)*
        DJI_TARGET_CHASE_RAD_TO_DEG*s_chase.config.pixel_y_to_up_sign;
    (void)TargetChase_ObserveAlignedBearing(&s_chase,
        DjiTargetChase_Wrap180(yaw), pitch, measurement_ms, received_ms);
}

static float DjiTargetChase_MoveToward(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

static float DjiTargetChase_SelectSlewStep(float current,
                                           float target,
                                           float acceleration_step,
                                           float deceleration_step)
{
    if (current * target < 0.0f || fabsf(target) <= fabsf(current)) {
        return deceleration_step;
    }
    return acceleration_step;
}

static void DjiTargetChase_ApplyCommandSlew(T_TargetChaseCommand *command)
{
    uint64_t mono_ms = DjiTargetChase_MonotonicMs();
    float dt_s = DJI_TARGET_CHASE_SLEW_DEFAULT_DT_S;
    float horizontal_accel_mps2 = DJI_TARGET_CHASE_NEAR_HORIZONTAL_ACCEL_MPS2;
    float yaw_accel_degps2 = DJI_TARGET_CHASE_NEAR_YAW_ACCEL_DEGPS2;
    float horizontal_accel_step_mps;
    float horizontal_decel_step_mps;
    float vertical_step_mps;
    float yaw_accel_step_degps;
    float yaw_decel_step_degps;
    bool far_or_searching;
    bool yaw_direction_reversal;
    bool near_off_axis;

    if (command == NULL || !s_has_last_command || !s_last_command.should_send) {
        s_last_slew_mono_ms = mono_ms;
        return;
    }

    if (s_last_slew_mono_ms != 0 && mono_ms >= s_last_slew_mono_ms) {
        dt_s = DjiTargetChase_Clamp(
            (float) (mono_ms - s_last_slew_mono_ms) / 1000.0f,
            DJI_TARGET_CHASE_SLEW_MIN_DT_S,
            DJI_TARGET_CHASE_SLEW_MAX_DT_S);
    }
    s_last_slew_mono_ms = mono_ms;

    /*
     * Long-range pursuit and body search have several seconds of reaction
     * margin. Let them reach their bounded command sooner, while retaining
     * the original gentler slew once the balloon is large in the image.
     */
    far_or_searching =
        s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_SEARCHING ||
        s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_BODY_TURNING ||
        (s_chase.filtered_box_angle_deg > 0.01f &&
         s_chase.filtered_box_angle_deg <
             DJI_TARGET_CHASE_GIMBAL_NEAR_BOX_ANGLE_DEG);
    if (far_or_searching) {
        horizontal_accel_mps2 = DJI_TARGET_CHASE_FAR_HORIZONTAL_ACCEL_MPS2;
        yaw_accel_degps2 = DJI_TARGET_CHASE_FAR_YAW_ACCEL_DEGPS2;
    }
    /*
     * Search can finish while the body still carries a +/-14 deg/s yaw
     * command. If tracking immediately asks for the opposite direction, the
     * ordinary 8 deg/s^2 deceleration preserves the old turn for seconds and
     * sweeps the target through the frame. Cross zero faster, but keep a
     * bounded slew rather than injecting a discontinuous stop.
     */
    yaw_direction_reversal =
        s_last_command.yaw_rate_degps * command->yaw_rate_degps < -0.25f;
    near_off_axis =
        s_chase.filtered_box_angle_deg >=
            DJI_TARGET_CHASE_NEAR_OFF_AXIS_BOX_ANGLE_DEG &&
        fabsf(command->target_bearing_yaw_deg) >=
            DJI_TARGET_CHASE_NEAR_OFF_AXIS_BEARING_DEG;
    horizontal_accel_step_mps = horizontal_accel_mps2 * dt_s;
    horizontal_decel_step_mps =
        (near_off_axis
             ? DJI_TARGET_CHASE_NEAR_OFF_AXIS_DECEL_MPS2
             : DJI_TARGET_CHASE_HORIZONTAL_DECEL_MPS2) *
        dt_s;
    vertical_step_mps = DJI_TARGET_CHASE_VERTICAL_ACCEL_MPS2 * dt_s;
    yaw_accel_step_degps = yaw_accel_degps2 * dt_s;
    yaw_decel_step_degps =
        (yaw_direction_reversal
             ? DJI_TARGET_CHASE_YAW_REVERSAL_DEGPS2
             : DJI_TARGET_CHASE_YAW_DECEL_DEGPS2) *
        dt_s;

    command->x_mps = DjiTargetChase_MoveToward(s_last_command.x_mps,
                                               command->x_mps,
                                               DjiTargetChase_SelectSlewStep(
                                                   s_last_command.x_mps,
                                                   command->x_mps,
                                                   horizontal_accel_step_mps,
                                                   horizontal_decel_step_mps));
    command->y_mps = DjiTargetChase_MoveToward(s_last_command.y_mps,
                                               command->y_mps,
                                               DjiTargetChase_SelectSlewStep(
                                                   s_last_command.y_mps,
                                                   command->y_mps,
                                                   horizontal_accel_step_mps,
                                                   horizontal_decel_step_mps));
    command->z_mps = DjiTargetChase_MoveToward(s_last_command.z_mps,
                                               command->z_mps,
                                               vertical_step_mps);
    command->yaw_rate_degps = DjiTargetChase_MoveToward(s_last_command.yaw_rate_degps,
                                                        command->yaw_rate_degps,
                                                        DjiTargetChase_SelectSlewStep(
                                                            s_last_command.yaw_rate_degps,
                                                            command->yaw_rate_degps,
                                                            yaw_accel_step_degps,
                                                            yaw_decel_step_degps));
}

static bool DjiTargetChase_ApplyVerticalSafety(T_TargetChaseCommand *command,
                                               const T_TargetChaseAircraftState *state)
{
    float original_z_mps;
    float speed_limit_mps;
    float excursion_m;
    float lower_height_m;
    float upper_height_m;

    if (command == NULL) {
        return false;
    }

    original_z_mps = command->z_mps;
    bool alignment_limited = TargetChase_LimitForwardForAlignment(&s_chase, command);
    if (alignment_limited) {
        static uint64_t last_alignment_log_ms = 0;
        if (s_last_now_ms < last_alignment_log_ms || s_last_now_ms - last_alignment_log_ms >= 500) {
            DjiTargetChase_LogEvent("approach_alignment_limited", s_last_now_ms,
                                   DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, command);
            last_alignment_log_ms = s_last_now_ms;
        }
    }
    speed_limit_mps = fabsf(s_chase.config.max_vertical_speed_mps);
    if (!isfinite(command->z_mps) || !isfinite(speed_limit_mps) || speed_limit_mps <= 0.0f) {
        command->z_mps = 0.0f;
    } else {
        command->z_mps = DjiTargetChase_Clamp(command->z_mps,
                                              -speed_limit_mps,
                                              speed_limit_mps);
    }

    /* Never repeat a vertical command when current altitude cannot be read. */
    if (state == NULL || !isfinite(state->altitude_agl_m)) {
        bool forward_limited;
        command->z_mps = 0.0f;
        forward_limited = TargetChase_LimitForwardForVertical(&s_chase.config, command);
        return alignment_limited || forward_limited || fabsf(command->z_mps - original_z_mps) > 0.0001f;
    }

    if (!s_start_height_initialized && state->in_air && state->altitude_agl_m > 0.1f) {
        s_start_height_m = state->altitude_agl_m;
        s_start_height_initialized = true;
        DjiTargetChase_LogEvent("vertical_velocity_start_height_latched",
                                s_last_now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                command);
    }

    if ((state->altitude_agl_m <= s_chase.config.min_height_m && command->z_mps < 0.0f) ||
        (state->altitude_agl_m >= s_chase.config.max_height_m && command->z_mps > 0.0f)) {
        command->z_mps = 0.0f;
    }

    excursion_m = fabsf(s_chase.config.max_vertical_excursion_m);
    if (s_start_height_initialized && isfinite(excursion_m) && excursion_m > 0.0f) {
        lower_height_m = fmaxf(s_chase.config.min_height_m,
                               s_start_height_m - excursion_m);
        upper_height_m = fminf(s_chase.config.max_height_m,
                               s_start_height_m + excursion_m);
        if ((state->altitude_agl_m <= lower_height_m && command->z_mps < 0.0f) ||
            (state->altitude_agl_m >= upper_height_m && command->z_mps > 0.0f)) {
            command->z_mps = 0.0f;
        }
    }

    bool forward_limited = TargetChase_LimitForwardForVertical(&s_chase.config, command);
    return alignment_limited || forward_limited || fabsf(command->z_mps - original_z_mps) > 0.0001f;
}

static bool DjiTargetChase_EnvFlagEnabled(const char *name)
{
    const char *value = getenv(name);

    if (value == NULL || value[0] == '\0') {
        return false;
    }

    return strcmp(value, "0") != 0 &&
           strcmp(value, "false") != 0 &&
           strcmp(value, "FALSE") != 0 &&
           strcmp(value, "off") != 0 &&
           strcmp(value, "OFF") != 0;
}

static bool DjiTargetChase_EnvFlagEnabledByDefault(const char *name)
{
    const char *value = getenv(name);

    if (value == NULL || value[0] == '\0') {
        return true;
    }

    return strcmp(value, "0") != 0 &&
           strcmp(value, "false") != 0 &&
           strcmp(value, "FALSE") != 0 &&
           strcmp(value, "off") != 0 &&
           strcmp(value, "OFF") != 0;
}

T_DjiReturnCode DjiTargetChase_SetPanelMaintenance(bool enabled)
{
    /* The panel serializes this with task creation and waits for all workers
     * to exit. These checks also protect callers from changing a live or
     * incompletely released controller. A rejection changes nothing. */
    if (s_initialized || DjiTargetChase_IsReleasePending() ||
        __atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    __atomic_store_n(&s_panel_maintenance_mode, enabled ? 1 : 0, __ATOMIC_RELEASE);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool DjiTargetChase_RequestedNoFlight(void)
{
    int mode = __atomic_load_n(&s_panel_maintenance_mode, __ATOMIC_ACQUIRE);
    return mode >= 0 ? mode == 1 : DjiTargetChase_EnvFlagEnabled("BALLOON_NO_FLIGHT");
}

bool DjiTargetChase_RequestedGimbalTracking(void)
{
    int mode = __atomic_load_n(&s_panel_maintenance_mode, __ATOMIC_ACQUIRE);
    return mode >= 0 ? mode == 0 : DjiTargetChase_EnvFlagEnabledByDefault("BALLOON_GIMBAL_TRACKING");
}

static bool DjiTargetChase_VisibleTargetOnly(void)
{
    /* Panel tasks preserve the operator's camera view and may move the
     * gimbal only to center a fresh observed target. Legacy callers retain
     * their environment-selected reset/search behavior. */
    return __atomic_load_n(&s_panel_maintenance_mode, __ATOMIC_ACQUIRE) >= 0;
}

static float DjiTargetChase_ReadFloatEnvBounded(const char *name,
                                                float fallback,
                                                float minimum,
                                                float maximum)
{
    const char *value = getenv(name);
    char *end = NULL;
    float parsed;

    if (value == NULL || value[0] == '\0') {
        return fallback;
    }

    parsed = strtof(value, &end);
    if (end == value ||
        *end != '\0' ||
        !isfinite(parsed) ||
        parsed < minimum ||
        parsed > maximum) {
        return fallback;
    }
    return parsed;
}

static uint64_t DjiTargetChase_NowMs(void)
{
    return BalloonClock_NowMs();
}

static uint64_t DjiTargetChase_MonotonicMs(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0;
    }

    return ((uint64_t) ts.tv_sec * 1000ULL) + ((uint64_t) ts.tv_nsec / 1000000ULL);
}

/* Optional timing runs on the same owner thread as the control API. SDK
 * callbacks and LogEvent do not touch this accumulator. It adds no SDK calls.
 * Summary IO is best-effort synchronous, outside measured algorithms, at most
 * once per five seconds plus DeInit. Enable only for an explicit profiling run. */
#define DJI_TARGET_CHASE_PERFORMANCE_LOG "/home/uav/offline_balloon_strike_test/control_performance.jsonl"
#define DJI_TARGET_CHASE_PROFILE_WINDOW_NS 5000000000ULL
enum {
    PROFILE_UPDATE_PIXEL, PROFILE_UPDATE_BOX, PROFILE_STEP,
    PROFILE_WORLD, PROFILE_PIXEL, PROFILE_HYBRID, PROFILE_JOYSTICK, PROFILE_KF, PROFILE_KF_UPDATE, PROFILE_STAGE_COUNT
};
static const char *s_profile_stage_names[PROFILE_STAGE_COUNT] = {
    "target_update_pixel_core", "target_update_box_core", "control_step_core",
    "prediction_world_gimbal", "prediction_pixel_gimbal", "prediction_hybrid_gimbal",
    "joystick_api_return", "prediction_delayed_kf_gimbal", "delayed_kf_observe"
};
static const char *s_profile_state_names[3] = {"unknown", "ground", "air"};
static T_BalloonPerformanceStats s_profile_stats[PROFILE_STAGE_COUNT][3];
static bool s_profile_enabled = false;
static uint64_t s_profile_window_start_ns, s_profile_window_id, s_profile_dropped_windows;
static uint64_t s_profile_previous_flush_ns;
static char s_profile_clock_id[80], s_profile_id[64], s_profile_mission_token[96];

static uint64_t DjiTargetChase_ProfileClockNs(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0 || ts.tv_sec < 0 ||
        ts.tv_nsec < 0 || ts.tv_nsec >= 1000000000L) return 0;
    return (uint64_t) ts.tv_sec * 1000000000ULL + (uint64_t) ts.tv_nsec;
}

static void DjiTargetChase_ProfileInit(void)
{
    const char *requested = getenv("BALLOON_PROFILE_TIMING");
    s_profile_enabled = requested != NULL && strcmp(requested, "1") == 0;
    memset(s_profile_stats, 0, sizeof(s_profile_stats));
    s_profile_window_id = s_profile_dropped_windows = s_profile_previous_flush_ns = 0;
    if (!s_profile_enabled) return;
    s_profile_window_start_ns = DjiTargetChase_ProfileClockNs();
    if (!s_profile_window_start_ns || !BalloonClock_Id(s_profile_clock_id)) {
        s_profile_enabled = false;
        return;
    }
    snprintf(s_profile_id, sizeof(s_profile_id), "%lu-%llu", (unsigned long) getpid(),
             (unsigned long long) s_profile_window_start_ns);
    BalloonEvidence_Token(s_profile_mission_token);
}

static uint64_t DjiTargetChase_ProfileBegin(void)
{
    return s_profile_enabled ? DjiTargetChase_ProfileClockNs() : 0;
}

static void DjiTargetChase_ProfileEnd(unsigned int stage, uint64_t begin_ns, bool success)
{
    uint64_t end_ns, stamp, now_ms;
    unsigned int state = 0;
    if (!s_profile_enabled || stage >= PROFILE_STAGE_COUNT) return;
    end_ns = DjiTargetChase_ProfileClockNs();
    now_ms = end_ns / 1000000ULL;
    stamp = s_chase.aircraft.timestamp_ms;
    /* A no-flight synthetic state or stale/invalid read cannot prove airborne. */
    if (!s_no_flight_mode && stamp && now_ms >= stamp && now_ms - stamp <= 500ULL)
        state = s_chase.aircraft.in_air ? 2U : 1U;
    (void) BalloonPerformance_Add(&s_profile_stats[stage][state], begin_ns, end_ns, success);
}

static void DjiTargetChase_ProfileFlush(bool force)
{
    uint64_t end_ns, flush_begin_ns;
    unsigned int stage, state, bin;
    bool has_data = false, failed;
    FILE *fp;
    if (!s_profile_enabled) return;
    end_ns = DjiTargetChase_ProfileClockNs();
    if (!end_ns || end_ns < s_profile_window_start_ns ||
        (!force && end_ns - s_profile_window_start_ns < DJI_TARGET_CHASE_PROFILE_WINDOW_NS)) return;
    for (stage = 0; stage < PROFILE_STAGE_COUNT; ++stage)
        for (state = 0; state < 3; ++state)
            if (s_profile_stats[stage][state].count || s_profile_stats[stage][state].invalid_count) has_data = true;
    if (!has_data) {
        s_profile_window_start_ns = end_ns;
        ++s_profile_window_id;
        return;
    }
    flush_begin_ns = end_ns;
    fp = fopen(DJI_TARGET_CHASE_PERFORMANCE_LOG, "a");
    failed = fp == NULL;
    if (fp != NULL) {
        for (stage = 0; stage < PROFILE_STAGE_COUNT; ++stage) {
            for (state = 0; state < 3; ++state) {
                const T_BalloonPerformanceStats *stats = &s_profile_stats[stage][state];
                uint64_t p95;
                if (!stats->count && !stats->invalid_count) continue;
                fprintf(fp, "{\"schema_version\":1,\"event\":\"performance_summary\","
                    "\"source\":\"dji_target_chase_task\",\"clock_id\":\"%s\",\"profile_id\":\"%s\","
                    "\"mission_token\":\"%s\",\"window_id\":%llu,\"window_start_ns\":%llu,"
                    "\"window_end_ns\":%llu,\"ts_ms\":%llu,\"prediction_mode\":%d,\"no_flight\":%s,"
                    "\"aircraft_state\":\"%s\",\"stage\":\"%s\",\"count\":%llu,\"success_count\":%llu,"
                    "\"sum_ns\":%llu,\"min_ns\":%llu,\"max_ns\":%llu,\"invalid_count\":%llu,"
                    "\"dropped_windows_total\":%llu,\"previous_flush_ns\":%llu,\"quantile_method\":\"nearest_rank_histogram_upper_bound\","
                    "\"histogram_upper_bounds_ns\":[",
                    s_profile_clock_id, s_profile_id, s_profile_mission_token,
                    (unsigned long long) s_profile_window_id, (unsigned long long) s_profile_window_start_ns,
                    (unsigned long long) end_ns, (unsigned long long) (end_ns / 1000000ULL),
                    s_chase.config.prediction_mode, s_no_flight_mode ? "true" : "false",
                    s_profile_state_names[state], s_profile_stage_names[stage],
                    (unsigned long long) stats->count, (unsigned long long) stats->success_count,
                    (unsigned long long) stats->sum_ns, (unsigned long long) stats->min_ns,
                    (unsigned long long) stats->max_ns, (unsigned long long) stats->invalid_count,
                    (unsigned long long) s_profile_dropped_windows, (unsigned long long) s_profile_previous_flush_ns);
                for (bin = 0; bin < BALLOON_PERFORMANCE_BINS - 1U; ++bin)
                    fprintf(fp, "%llu,", (unsigned long long) BalloonPerformance_Bound(bin));
                fprintf(fp, "null],\"histogram_counts\":[");
                for (bin = 0; bin < BALLOON_PERFORMANCE_BINS; ++bin)
                    fprintf(fp, "%s%llu", bin ? "," : "", (unsigned long long) stats->histogram[bin]);
                fprintf(fp, "],\"p95_upper_bound_ns\":");
                if (BalloonPerformance_P95Upper(stats, &p95)) fprintf(fp, "%llu", (unsigned long long) p95);
                else fprintf(fp, "null");
                fprintf(fp, "}\n");
            }
        }
        failed = ferror(fp) != 0;
        if (fclose(fp) != 0) failed = true;
    }
    /* Never retry a partially emitted window (which would duplicate samples). */
    if (failed && s_profile_dropped_windows != UINT64_MAX) ++s_profile_dropped_windows;
    memset(s_profile_stats, 0, sizeof(s_profile_stats));
    ++s_profile_window_id;
    s_profile_window_start_ns = end_ns;
    end_ns = DjiTargetChase_ProfileClockNs();
    s_profile_previous_flush_ns = end_ns >= flush_begin_ns ? end_ns - flush_begin_ns : 0;
}

static void DjiTargetChase_PublishPredictionStatus(uint64_t now_ms, bool step_ok,
                                                 const T_TargetChaseCommand *command)
{
    unsigned int provider = PREDICTION_STATUS_PIXEL, reason = 0;
    uint64_t packed;
    /* ACTIVE is assigned only after the core passed target freshness,
     * authority, trigger and geometry checks and calculated the command. */
    if (!step_ok || !command || !command->should_send ||
        __atomic_load_n(&s_control_inhibited, __ATOMIC_ACQUIRE) || s_chase.mode != TARGET_CHASE_MODE_ACTIVE) {
        DjiTargetChase_ClearPredictionStatus();
        return;
    }
    if (s_chase.config.geometry == TARGET_CHASE_GEOMETRY_GROUND_NADIR) {
        provider = PREDICTION_STATUS_GROUND;
    } else if (s_chase.config.prediction_mode == 3) {
        provider = s_chase.kf_active ? PREDICTION_STATUS_KF : PREDICTION_STATUS_PIXEL;
        reason = (unsigned int)s_chase.kf_reason;
    } else if (s_chase.config.prediction_mode == 0) {
        /* These are exactly the remaining PredictWorld admission checks after
         * Step has established a fresh target. No second prediction is run. */
        if (s_chase.motion_world_valid && s_chase.motion_estimate_initialized &&
            s_chase.motion_estimate_timestamp_ms == s_chase.target.timestamp_ms)
            provider = PREDICTION_STATUS_WORLD;
        else
            reason = PREDICTION_STATUS_WORLD_MISSING;
    } else if (s_chase.config.prediction_mode == 2) {
        if (s_chase.hybrid_weight > 0.0f) provider = PREDICTION_STATUS_HYBRID;
        else reason = PREDICTION_STATUS_BLEND_ZERO;
    }
    packed = ((now_ms & DJI_TARGET_CHASE_STATUS_TIME_MASK) << 16) |
             (s_no_flight_mode ? UINT64_C(0x8000) : 0) |
             ((uint64_t)(reason & 0x7fU) << 8) | provider;
    __atomic_store_n(&s_prediction_status_snapshot, packed, __ATOMIC_RELEASE);
}

bool DjiTargetChase_PredictionStatus(char *out, size_t cap)
{
    const uint64_t packed = __atomic_load_n(&s_prediction_status_snapshot, __ATOMIC_ACQUIRE);
    const char *mode = "像素预测", *reason = "";
    const unsigned int provider = (unsigned int)(packed & 0xffU);
    const unsigned int code = (unsigned int)((packed >> 8) & 0x7fU);
    uint64_t age;
    if (!out || cap == 0) return false;
    if (!packed || __atomic_load_n(&s_control_inhibited, __ATOMIC_ACQUIRE)) {
        snprintf(out, cap, "等待新的预测控制结果");
        return false;
    }
    age = (DjiTargetChase_NowMs() - (packed >> 16)) & DJI_TARGET_CHASE_STATUS_TIME_MASK;
    if (age > 2000ULL) {
        snprintf(out, cap, "预测状态已过期，等待控制更新");
        return false;
    }
    switch (provider) {
    case PREDICTION_STATUS_WORLD: mode = "角速度补偿"; break;
    case PREDICTION_STATUS_KF: mode = "延迟卡尔曼"; break;
    case PREDICTION_STATUS_HYBRID: mode = "像素与角速度混合"; break;
    case PREDICTION_STATUS_GROUND: mode = "像素地面投影"; break;
    default: break;
    }
    if (provider == PREDICTION_STATUS_PIXEL) {
        switch (code) {
        case TARGET_KF_NO_CALIBRATION: reason = "；KF回退：缺有效时间校准"; break;
        case TARGET_KF_NO_POSE: reason = "；KF回退：缺对应时刻姿态"; break;
        case TARGET_KF_WARMUP: reason = "；KF回退：有效样本不足"; break;
        case TARGET_KF_REJECTED: reason = "；KF回退：观测被拒绝"; break;
        case TARGET_KF_STALE: reason = "；KF回退：滤波观测过期"; break;
        case TARGET_KF_UNCERTAIN: reason = "；KF回退：预测不稳定"; break;
        case TARGET_KF_RESET: reason = "；KF回退：等待重新建轨"; break;
        case PREDICTION_STATUS_WORLD_MISSING: reason = "；补偿回退：缺匹配姿态或角度轨迹"; break;
        case PREDICTION_STATUS_BLEND_ZERO: reason = "；角度分量当前未采用"; break;
        default: break;
        }
    } else if (provider == PREDICTION_STATUS_KF && code == TARGET_KF_REJECTED) {
        reason = "；当前观测被拒，沿用有效滤波状态";
    }
    snprintf(out, cap, "实际计算：%s%s%s", mode, reason,
             (packed & UINT64_C(0x8000)) ? "；禁飞，仅计算" : "");
    return true;
}

static bool DjiTargetChase_ProfileStep(uint64_t now_ms, T_TargetChaseCommand *command)
{
    uint64_t begin = DjiTargetChase_ProfileBegin();
    bool result = TargetChase_Step(&s_chase, now_ms, command);
    DjiTargetChase_ProfileEnd(PROFILE_STEP, begin, result);
    DjiTargetChase_PublishPredictionStatus(DjiTargetChase_NowMs(), result, command);
    return result;
}

static bool DjiTargetChase_ProfileWorld(uint64_t now_ms, T_TargetChasePrediction *prediction)
{
    uint64_t begin = DjiTargetChase_ProfileBegin();
    bool result = TargetChase_PredictWorld(&s_chase, now_ms, prediction);
    DjiTargetChase_ProfileEnd(PROFILE_WORLD, begin, result);
    return result;
}

static bool DjiTargetChase_ProfileHybrid(uint64_t now_ms, float yaw, float pitch,
                                        float *y, float *p, float *yr, float *pr)
{
    uint64_t begin = DjiTargetChase_ProfileBegin();
    bool result = TargetChase_PredictHybrid(&s_chase, now_ms, yaw, pitch, y, p, yr, pr);
    DjiTargetChase_ProfileEnd(s_chase.config.prediction_mode == 3 ? PROFILE_KF :
                             (s_chase.config.prediction_mode == 1 ? PROFILE_PIXEL : PROFILE_HYBRID), begin, result);
    return result;
}

static void DjiTargetChase_LogEvent(const char *event,
                                    uint64_t now_ms,
                                    T_DjiReturnCode code,
                                    const T_TargetChaseCommand *command)
{
    FILE *fp = fopen(DJI_TARGET_CHASE_CONTROL_EVENT_LOG, "a");
    uint64_t event_ms = now_ms != 0 ? now_ms : DjiTargetChase_NowMs();
    char clock_id[80] = {0}, mission_token[96];
    BalloonEvidence_Token(mission_token);
    T_TargetChasePrediction prediction;
    T_DjiTargetChaseControlDeviceMonitor monitor;
    pthread_mutex_lock(&s_control_device_mutex);
    monitor = s_control_device;
    pthread_mutex_unlock(&s_control_device_mutex);
    uint64_t logged_ms = DjiTargetChase_NowMs();
    (void) BalloonClock_Id(clock_id);
    memset(&prediction, 0, sizeof(prediction));
    if (s_chase.config.prediction_mode == 0)
        (void) TargetChase_PredictWorld(&s_chase, event_ms, &prediction);

    if (fp == NULL) {
        return;
    }

    fprintf(fp,
            "{\"clock\":\"monotonic-v1\",\"ts_ms\":%llu,\"source\":\"dji_target_chase_task\","
            "\"clock_id\":\"%s\",\"mission_token\":\"%s\",\"wall_ts_ms\":%llu,\"logged_ts_ms\":%llu,"
            "\"prediction_mode\":%d,\"hybrid_weight\":%.3f,\"pixel_lead_s\":%.4f,\"target_ts_ms\":%llu,"
            "\"prediction_provider\":\"%s\",\"kf_active\":%s,\"kf_reason\":%d,"
            "\"kf_measurement_ms\":%llu,\"kf_variance_deg2\":%.5f,"
            "\"kf_calibration_id\":\"%s\",\"kf_timing_reason\":\"%s\","
            "\"prediction_adaptive\":%s,\"prediction_horizon_s\":%.4f,"
            "\"prediction_age_horizon_s\":%.4f,\"prediction_extra_horizon_s\":%.4f,"
            "\"prediction_rate_scale\":%.3f,\"prediction_reason\":%d,"
            "\"prediction_residual_deg\":%.4f,"
            "\"event\":\"%s\",\"code\":\"0x%llX\",\"mode\":%d,"
            "\"x_mps\":%.3f,\"y_mps\":%.3f,\"z_mps\":%.3f,"
            "\"yaw_rate_degps\":%.3f,\"should_send\":%s,"
            "\"target_bearing_yaw_deg\":%.3f,"
            "\"target_bearing_vertical_deg\":%.3f,"
            "\"filtered_u_px\":%.3f,\"filtered_v_px\":%.3f,"
            "\"filtered_u_rate_pxps\":%.3f,\"filtered_v_rate_pxps\":%.3f,"
            "\"filtered_box_angle_deg\":%.3f,"
            "\"filtered_box_angle_rate_degps\":%.3f,"
            "\"motion_world_valid\":%s,\"world_yaw_deg\":%.3f,"
            "\"world_pitch_deg\":%.3f,\"world_yaw_rate_degps\":%.3f,"
            "\"world_pitch_rate_degps\":%.3f,"
            "\"gimbal_recenter_active\":%s,"
            "\"has_authority\":%s,\"real_authority\":%s,\"no_flight\":%s,"
            "\"authority_monitor\":\"%s\",\"control_device_valid\":%s,"
            "\"control_device_owner\":%u,\"control_device_open\":%s,"
            "\"control_device_age_ms\":%llu,\"control_device_generation\":%llu,"
            "\"control_device_confirmed\":%s,"
            "\"vertical_mode\":\"velocity\",\"height_reference\":\"takeoff_relative\","
            "\"start_height_m\":%.3f,"
            "\"altitude_agl_m\":%.3f,\"vel_n_mps\":%.3f,"
            "\"vel_e_mps\":%.3f,\"vel_u_mps\":%.3f,"
            "\"roll_deg\":%.3f,\"pitch_deg\":%.3f,\"yaw_deg\":%.3f,"
            "\"in_air\":%s}\n",
            (unsigned long long) event_ms,
            clock_id, mission_token, (unsigned long long) BalloonClock_WallMs(),
            (unsigned long long) logged_ms,
            s_chase.config.prediction_mode, s_chase.hybrid_weight, s_chase.hybrid_horizon_s,
            (unsigned long long) s_chase.target.timestamp_ms,
            s_chase.config.prediction_mode == 0 ? "world_alpha_beta" :
                (s_chase.config.prediction_mode == 3 && s_chase.kf_active ? "delayed_kf" :
                 (s_chase.config.prediction_mode == 2 && s_chase.hybrid_weight > 0 ? "near_hybrid" : "original_pixel")),
            s_chase.kf_active ? "true" : "false", (int)s_chase.kf_reason,
            (unsigned long long)s_chase.kf_measurement_ms, s_chase.kf_variance_deg2,
            s_kf_timing.calibration_id, s_kf_timing_reason,
            s_chase.config.prediction_adaptive ? "true" : "false",
            prediction.horizon_s, prediction.age_horizon_s, prediction.extra_horizon_s,
            prediction.rate_scale, (int) prediction.reason,
            s_chase.prediction_residual_deg,
            event == NULL ? "" : event,
            (unsigned long long) code,
            (int) TargetChase_GetMode(&s_chase),
            command == NULL ? 0.0f : command->x_mps,
            command == NULL ? 0.0f : command->y_mps,
            command == NULL ? 0.0f : command->z_mps,
            command == NULL ? 0.0f : command->yaw_rate_degps,
            command != NULL && command->should_send ? "true" : "false",
            command == NULL ? 0.0f : command->target_bearing_yaw_deg,
            command == NULL ? 0.0f : command->target_bearing_vertical_deg,
            s_chase.filtered_u_px,
            s_chase.filtered_v_px,
            s_chase.filtered_u_rate_pxps,
            s_chase.filtered_v_rate_pxps,
            s_chase.filtered_box_angle_deg,
            s_chase.filtered_box_angle_rate_degps,
            s_chase.motion_world_valid ? "true" : "false",
            s_chase.filtered_world_yaw_deg,
            s_chase.filtered_world_pitch_deg,
            s_chase.filtered_world_yaw_rate_degps,
            s_chase.filtered_world_pitch_rate_degps,
            s_chase.gimbal_recenter_active ? "true" : "false",
            s_chase.has_authority ? "true" : "false",
            __atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE) ? "true" : "false",
            s_no_flight_mode ? "true" : "false",
            monitor.enabled ? "control_device" : "native_callback",
            monitor.valid ? "true" : "false", (unsigned int)monitor.device,
            monitor.open ? "true" : "false",
            (unsigned long long)(monitor.have_stamp && logged_ms >= monitor.receipt_ms ?
                                 logged_ms - monitor.receipt_ms : 0),
            (unsigned long long)monitor.generation,
            monitor.confirmed ? "true" : "false",
            s_start_height_initialized ? s_start_height_m : 0.0f,
            s_chase.aircraft.altitude_agl_m,
            s_chase.aircraft.velocity_neu_mps[0],
            s_chase.aircraft.velocity_neu_mps[1],
            s_chase.aircraft.velocity_neu_mps[2],
            s_chase.aircraft.attitude_deg[0],
            s_chase.aircraft.attitude_deg[1],
            s_chase.aircraft.attitude_deg[2],
            s_chase.aircraft.in_air ? "true" : "false");
    fclose(fp);
}

static bool DjiTargetChase_IsSuccess(T_DjiReturnCode return_code)
{
    if (return_code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return true;
    }

    if (return_code == DJI_ERROR_FC_MODULE_CODE_OBTAIN_CONTROL_SUCCESS) {
        return true;
    }

    if (return_code == DJI_ERROR_FC_MODULE_CODE_RELEASE_CONTROL_SUCCESS) {
        return true;
    }

    if (return_code == DJI_ERROR_FC_JOYSTICK_MODULE_OBTAIN_RELEASE_JOYSTICK_AUTH_SUCCESS) {
        return true;
    }

    if (return_code == DJI_TARGET_CHASE_ERROR_JOYSTICK_AUTH_SUCCESS) {
        return true;
    }

    return false;
}

static bool DjiTargetChase_IsOptionalAuthorityCallbackFailure(T_DjiReturnCode return_code)
{
    return return_code == DJI_TARGET_CHASE_ERROR_NONSUPPORT;
}

/* Call only while holding s_control_device_mutex. A missing/invalid sample
 * cannot establish a new owner, so possible SDK ownership remains pending
 * until Release succeeds or a fresh sample identifies RC/app ownership. */
static void DjiTargetChase_ControlDeviceFault(const char *event)
{
    if (s_control_device.valid || !__atomic_load_n(&s_control_inhibited, __ATOMIC_ACQUIRE))
        s_control_device.pending_event = event;
    s_control_device.valid = false;
    __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
}

static bool DjiTargetChase_ControlDeviceHealthy(bool require_owner)
{
    bool healthy = true;
    const char *event;
    pthread_mutex_lock(&s_control_device_mutex);
    if (!s_control_device.enabled) {
        pthread_mutex_unlock(&s_control_device_mutex);
        return true;
    }
    uint64_t now = DjiTargetChase_MonotonicMs();
    if (s_control_device.enabled) {
        if (s_control_device.ready && s_control_device.valid &&
            (now < s_control_device.receipt_ms ||
             now - s_control_device.receipt_ms > DJI_TARGET_CHASE_CONTROL_DEVICE_FRESH_MS))
            DjiTargetChase_ControlDeviceFault("control_device_samples_stale");
        healthy = s_control_device.ready && s_control_device.valid &&
                  now >= s_control_device.receipt_ms &&
                  now - s_control_device.receipt_ms <= DJI_TARGET_CHASE_CONTROL_DEVICE_FRESH_MS;
        if (require_owner)
            healthy = healthy && s_control_device.confirmed &&
                      s_control_device.device == 4 && s_control_device.open;
    }
    event = s_control_device.pending_event;
    s_control_device.pending_event = NULL;
    pthread_mutex_unlock(&s_control_device_mutex);
    /* The API owner records its stop after zero/release. Keep synchronous
     * diagnostics out of the compatibility-monitor fault path during a lease. */
    if (event != NULL && !s_api_motion_only)
        DjiTargetChase_LogEvent(event, now, DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE, NULL);
    return healthy;
}

static T_DjiReturnCode DjiTargetChase_ControlDeviceCallback(
    const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *stamp)
{
    T_DjiFcSubscriptionControlDevice sample;
    uint64_t now = DjiTargetChase_MonotonicMs();
    pthread_mutex_lock(&s_control_device_mutex);
    if (!s_control_device.enabled) goto done;
    if (data == NULL || stamp == NULL || size != sizeof(sample)) {
        DjiTargetChase_ControlDeviceFault("control_device_invalid_sample");
        goto done;
    }
    memcpy(&sample, data, sizeof(sample));
    if (sample.deviceStatus != 0 && sample.deviceStatus != 1 && sample.deviceStatus != 4) {
        DjiTargetChase_ControlDeviceFault("control_device_unknown_owner");
        goto done;
    }
    if (s_control_device.have_stamp) {
        bool same = stamp->millisecond == s_control_device.stamp.millisecond &&
                    stamp->microsecond == s_control_device.stamp.microsecond;
        bool regressed = stamp->millisecond < s_control_device.stamp.millisecond ||
            (stamp->millisecond == s_control_device.stamp.millisecond &&
             stamp->microsecond < s_control_device.stamp.microsecond);
        if (regressed || (same && (sample.deviceStatus != s_control_device.device ||
                                  (sample.flightStatus != 0) != s_control_device.open))) {
            DjiTargetChase_ControlDeviceFault("control_device_stamp_invalid");
            goto done;
        }
        if (same) goto done; /* Cached samples cannot keep the monitor alive. */
        if (s_control_device.ready && now - s_control_device.receipt_ms >
            DJI_TARGET_CHASE_CONTROL_DEVICE_FRESH_MS)
            DjiTargetChase_ControlDeviceFault("control_device_samples_stale");
    }
    s_control_device.stamp = *stamp;
    s_control_device.have_stamp = true;
    s_control_device.receipt_ms = now;
    ++s_control_device.generation;
    if (s_control_device.samples < 2) ++s_control_device.samples;
    s_control_device.device = sample.deviceStatus;
    s_control_device.open = sample.flightStatus != 0;
    /* Once faulted, newer samples identify ownership for cleanup only; they
     * never reset the stop latch or silently revive a running mission. */
    if (!__atomic_load_n(&s_control_inhibited, __ATOMIC_ACQUIRE))
        s_control_device.valid = true;
    bool owned = sample.deviceStatus == 4 && sample.flightStatus == 1;
    if (owned && s_control_device.request_issued &&
        s_control_device.generation > s_control_device.request_generation) {
        s_control_device.confirmed = true;
        s_control_device.awaiting_grant = false;
        __atomic_store_n(&s_real_authority_obtained, true, __ATOMIC_RELEASE);
        __atomic_store_n(&s_ever_had_authority, true, __ATOMIC_RELEASE);
        /* A late grant remains inhibited, but must be released by Abort. */
        __atomic_store_n(&s_manual_takeover, false, __ATOMIC_RELEASE);
    } else if (!owned && s_control_device.confirmed) {
        s_control_device.confirmed = false;
        __atomic_store_n(&s_real_authority_obtained, false, __ATOMIC_RELEASE);
        if (!__atomic_load_n(&s_intentional_release, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
            __atomic_store_n(&s_manual_takeover, true, __ATOMIC_RELEASE);
            s_control_device.pending_event = "control_device_operator_takeover";
        }
    }
done:
    pthread_mutex_unlock(&s_control_device_mutex);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void DjiTargetChase_ControlDeviceStop(void)
{
    pthread_mutex_lock(&s_control_device_mutex);
    bool subscribed = s_control_device.subscribed;
    s_control_device.enabled = false;
    pthread_mutex_unlock(&s_control_device_mutex);
    if (subscribed) (void)BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE);
    pthread_mutex_lock(&s_control_device_mutex);
    memset(&s_control_device, 0, sizeof(s_control_device));
    pthread_mutex_unlock(&s_control_device_mutex);
}

static T_DjiReturnCode DjiTargetChase_ControlDeviceStart(void)
{
    T_DjiAircraftInfoBaseInfo aircraft = {0};
    T_DjiReturnCode code = DjiAircraftInfo_GetBaseInfo(&aircraft);
    if (!DjiTargetChase_IsSuccess(code) ||
        (aircraft.aircraftType != DJI_AIRCRAFT_TYPE_M300_RTK &&
         aircraft.aircraftType != DJI_AIRCRAFT_TYPE_M350_RTK)) {
        DjiTargetChase_LogEvent("control_device_aircraft_unsupported", DjiTargetChase_NowMs(),
                                DJI_TARGET_CHASE_ERROR_NONSUPPORT, NULL);
        return DJI_TARGET_CHASE_ERROR_NONSUPPORT;
    }
    pthread_mutex_lock(&s_control_device_mutex);
    memset(&s_control_device, 0, sizeof(s_control_device));
    s_control_device.enabled = true;
    pthread_mutex_unlock(&s_control_device_mutex);
    code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, DjiTargetChase_ControlDeviceCallback);
    if (!DjiTargetChase_IsSuccess(code)) {
        DjiTargetChase_ControlDeviceStop();
        return code;
    }
    pthread_mutex_lock(&s_control_device_mutex);
    s_control_device.subscribed = true;
    pthread_mutex_unlock(&s_control_device_mutex);
    uint64_t start = DjiTargetChase_MonotonicMs();
    while (DjiTargetChase_MonotonicMs() - start < DJI_TARGET_CHASE_CONTROL_DEVICE_WAIT_MS) {
        pthread_mutex_lock(&s_control_device_mutex);
        bool live = s_control_device.valid && s_control_device.samples >= 2;
        if (live) s_control_device.ready = true;
        pthread_mutex_unlock(&s_control_device_mutex);
        if (__atomic_load_n(&s_control_inhibited, __ATOMIC_ACQUIRE)) break;
        if (live && DjiTargetChase_ControlDeviceHealthy(false)) {
            DjiTargetChase_LogEvent("control_device_monitor_ready", DjiTargetChase_NowMs(), 0, NULL);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        usleep(10000);
    }
    __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
    DjiTargetChase_LogEvent("control_device_monitor_unavailable", DjiTargetChase_NowMs(),
                            DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT, NULL);
    return DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
}

static bool DjiTargetChase_JoystickIsZero(const T_DjiFlightControllerJoystickCommand *command)
{
    return command->x == 0.0f && command->y == 0.0f &&
           command->z == 0.0f && command->yaw == 0.0f;
}

static bool DjiTargetChase_ApiDeadlineValid(uint64_t deadline_ms, uint64_t max_remaining_ms)
{
    uint64_t now_ms = DjiTargetChase_NowMs();
    return now_ms != 0 && deadline_ms > now_ms &&
           deadline_ms - now_ms <= max_remaining_ms;
}

static bool DjiTargetChase_ApiStateFresh(uint64_t now_ms)
{
    if (now_ms == 0) return false;
    for (unsigned int i = 0; i < API_STATE_TOPIC_COUNT; ++i) {
        const T_BalloonTelemetryProgress *progress = &s_api_state_progress[i];
        if (!progress->have_progress || progress->last_progress_ms == 0 ||
            now_ms < progress->last_progress_ms ||
            now_ms - progress->last_progress_ms > s_api_state_max_age_ms[i])
            return false;
    }
    return true;
}

static T_DjiReturnCode DjiTargetChase_WriteJoystick(
    T_DjiFlightControllerJoystickCommand command, uint64_t api_deadline_ms)
{
    uint64_t begin = DjiTargetChase_ProfileBegin();
    bool moving = !DjiTargetChase_JoystickIsZero(&command);
    /* This also guards zero/hover/repeated/search/abort commands. Unknown,
     * stale, or unconfirmed ownership never authorizes a joystick write. */
    if (!DjiTargetChase_ControlDeviceHealthy(true) ||
        __atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE) ||
        (moving && DjiTargetChase_IsControlInhibited()) ||
        (moving && s_api_motion_only &&
         (!__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE) ||
          !DjiTargetChase_ApiStateFresh(DjiTargetChase_NowMs()) ||
          !DjiTargetChase_ApiDeadlineValid(api_deadline_ms, DJI_TARGET_CHASE_API_MAX_DURATION_MS))))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    T_DjiReturnCode result = DjiFlightController_ExecuteJoystickAction(command);
    DjiTargetChase_ProfileEnd(PROFILE_JOYSTICK, begin, DjiTargetChase_IsSuccess(result));
    return result;
}

static T_DjiReturnCode DjiTargetChase_ProfileJoystick(T_DjiFlightControllerJoystickCommand command)
{
    return DjiTargetChase_WriteJoystick(command, 0);
}

static void DjiTargetChase_LogGimbalEvent(const char *event,
                                          uint64_t now_ms,
                                          T_DjiReturnCode code,
                                          const T_DjiFcSubscriptionGimbalAngles *angles,
                                          float pitch_speed_command_degps,
                                          float yaw_speed_command_degps)
{
    FILE *fp = fopen(DJI_TARGET_CHASE_CONTROL_EVENT_LOG, "a");

    if (fp == NULL) {
        return;
    }

    fprintf(fp,
            "{\"clock\":\"monotonic-v1\",\"ts_ms\":%llu,\"source\":\"dji_target_chase_task\","
            "\"event\":\"%s\",\"code\":\"0x%llX\","
            "\"gimbal_tracking\":%s,\"no_flight\":%s,"
            "\"gimbal_state\":%d,\"target_detected\":%s,"
            "\"search_phase\":%d,\"search_sector_index\":%u,"
            "\"search_sector_count\":%u,\"search_sector_width_deg\":%.3f,"
            "\"search_body_step_deg\":%.3f,"
            "\"search_yaw_speed_degps\":%.3f,"
            "\"search_pitch_speed_degps\":%.3f,"
            "\"body_search_yaw_rate_degps\":%.3f,"
            "\"body_turn_active\":%s,"
            "\"gimbal_pitch_deg\":%.3f,\"gimbal_roll_deg\":%.3f,"
            "\"gimbal_yaw_deg\":%.3f,\"aircraft_pitch_deg\":%.3f,"
            "\"aircraft_yaw_deg\":%.3f,\"camera_body_pitch_deg\":%.3f,"
            "\"camera_body_yaw_deg\":%.3f,\"pitch_speed_command_degps\":%.3f,"
            "\"yaw_speed_command_degps\":%.3f}\n",
            (unsigned long long) (now_ms != 0 ? now_ms : DjiTargetChase_NowMs()),
            event == NULL ? "" : event,
            (unsigned long long) code,
            s_gimbal_tracking_enabled ? "true" : "false",
            s_no_flight_mode ? "true" : "false",
            (int) s_gimbal_state,
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING ? "true" : "false",
            (int) s_search_phase,
            s_search_sector_index,
            s_search_coverage.sector_count,
            s_search_coverage.sector_width_deg,
            s_search_coverage.body_yaw_step_deg,
            s_gimbal_search_yaw_speed_degps,
            s_gimbal_search_pitch_speed_degps,
            s_body_search_yaw_rate_degps,
            s_body_search_turn_active ? "true" : "false",
            angles == NULL ? 0.0f : angles->x,
            angles == NULL ? 0.0f : angles->y,
            angles == NULL ? 0.0f : angles->z,
            s_chase.aircraft.attitude_deg[1],
            s_chase.aircraft.attitude_deg[2],
            s_chase.camera_boresight_vertical_deg,
            s_chase.camera_boresight_yaw_deg,
            pitch_speed_command_degps,
            yaw_speed_command_degps);
    fclose(fp);
}

static bool DjiTargetChase_ShouldLogGimbalStatus(uint64_t now_ms)
{
    if (s_last_gimbal_status_log_ms == 0 ||
        now_ms < s_last_gimbal_status_log_ms ||
        now_ms - s_last_gimbal_status_log_ms >= DJI_TARGET_CHASE_GIMBAL_STATUS_LOG_INTERVAL_MS) {
        s_last_gimbal_status_log_ms = now_ms;
        return true;
    }

    return false;
}

static bool DjiTargetChase_ReadGimbalAngles(T_DjiFcSubscriptionGimbalAngles *angles)
{
    T_DjiDataTimestamp timestamp = {0};
    T_DjiReturnCode return_code;

    if (!s_gimbal_tracking_enabled || !s_gimbal_angles_subscribed || angles == NULL) {
        return false;
    }

    memset(angles, 0, sizeof(*angles));
    s_camera_read_begin_ms = DjiTargetChase_NowMs();
    return_code = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
        (uint8_t *) angles,
        sizeof(*angles),
        &timestamp);
    s_camera_read_end_ms = DjiTargetChase_NowMs();
    if (DjiTargetChase_IsSuccess(return_code)) s_camera_sdk_stamp = timestamp;
    return DjiTargetChase_IsSuccess(return_code);
}

static bool DjiTargetChase_UpdateGimbalBoresight(uint64_t now_ms,
                                                 T_DjiFcSubscriptionGimbalAngles *angles_out)
{
    T_DjiFcSubscriptionGimbalAngles angles;
    float body_yaw_deg;
    float body_pitch_deg;

    if (!DjiTargetChase_ReadGimbalAngles(&angles)) {
        if (s_chase.config.prediction_mode == 3) {
            s_chase.kf_camera_pose_valid = false;
            TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_NO_POSE);
        }
        return false;
    }

    body_yaw_deg = DjiTargetChase_Wrap180(angles.z - s_chase.aircraft.attitude_deg[2]);
    body_pitch_deg = DjiTargetChase_Wrap180(angles.x - s_chase.aircraft.attitude_deg[1]);
    TargetChase_SetCameraBoresight(&s_chase, body_yaw_deg, body_pitch_deg);
    DjiTargetChase_RecordCameraPose(s_chase.config.prediction_mode == 3 ?
                                    s_camera_read_end_ms : DjiTargetChase_NowMs(), angles.z, angles.x);
    if (s_chase.config.prediction_mode == 3) {
        s_chase.kf_camera_pose_ms = DjiTargetChase_NowMs();
        s_chase.kf_camera_pose_valid = TargetTiming_CurrentPose(s_kf_poses, s_kf_pose_count,
            s_chase.kf_camera_pose_ms, &s_chase.kf_camera_yaw_deg, &s_chase.kf_camera_pitch_deg);
    }
    if (s_chase.config.prediction_mode == 3 && s_kf_timing.enabled &&
        (s_camera_read_end_ms < s_kf_last_pose_receipt ||
         s_camera_read_end_ms-s_kf_last_pose_receipt > 100))
        TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_NO_POSE);

    if (angles_out != NULL) {
        *angles_out = angles;
    }

    (void) now_ms;
    return true;
}

static T_DjiReturnCode DjiTargetChase_SendGimbalSpeed(float pitch_speed_degps,
                                                       float yaw_speed_degps,
                                                       uint64_t now_ms,
                                                       const char *event,
                                                       bool bypass_interval)
{
    if (__atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE) ||
        (DjiTargetChase_IsControlInhibited() &&
         (pitch_speed_degps != 0.0f || yaw_speed_degps != 0.0f)))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;

    T_DjiGimbalManagerRotation rotation = {0};
    T_DjiFcSubscriptionGimbalAngles angles;
    T_DjiReturnCode return_code;
    uint64_t command_now_ms = DjiTargetChase_NowMs();

    if (!s_gimbal_tracking_enabled || !s_gimbal_manager_initialized) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    if (!bypass_interval &&
        s_last_gimbal_command_ms != 0 &&
        command_now_ms >= s_last_gimbal_command_ms &&
        command_now_ms - s_last_gimbal_command_ms < DJI_TARGET_CHASE_GIMBAL_COMMAND_INTERVAL_MS) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_SPEED;
    rotation.pitch = pitch_speed_degps;
    rotation.roll = 0.0f;
    rotation.yaw = yaw_speed_degps;
    rotation.time = DJI_TARGET_CHASE_GIMBAL_SPEED_COMMAND_TIME_S;
    return_code = DjiGimbalManager_Rotate(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION, rotation);
    if (DjiTargetChase_IsSuccess(return_code)) {
        s_last_gimbal_command_ms = command_now_ms;
        s_last_gimbal_pitch_speed_degps = pitch_speed_degps;
        s_last_gimbal_yaw_speed_degps = yaw_speed_degps;
    }

    memset(&angles, 0, sizeof(angles));
    (void) DjiTargetChase_UpdateGimbalBoresight(now_ms, &angles);
    if (!DjiTargetChase_IsSuccess(return_code) ||
        bypass_interval ||
        DjiTargetChase_ShouldLogGimbalStatus(command_now_ms)) {
        DjiTargetChase_LogGimbalEvent(event,
                                      now_ms,
                                      return_code,
                                      &angles,
                                      pitch_speed_degps,
                                      yaw_speed_degps);
    }
    return return_code;
}

static void DjiTargetChase_StopGimbal(uint64_t now_ms, const char *event, bool force)
{
    if (!force &&
        fabsf(s_last_gimbal_pitch_speed_degps) < 0.01f &&
        fabsf(s_last_gimbal_yaw_speed_degps) < 0.01f) {
        return;
    }

    (void) DjiTargetChase_SendGimbalSpeed(0.0f, 0.0f, now_ms, event, true);
}

static T_DjiReturnCode DjiTargetChase_BeginGimbalReset(uint64_t now_ms,
                                                       const char *requested_event,
                                                       const char *failed_event)
{
    T_DjiReturnCode return_code;

    if (DjiTargetChase_IsControlInhibited())
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    DjiTargetChase_StopGimbal(now_ms, "gimbal_stop_before_reset", true);
    if (DjiTargetChase_IsControlInhibited())
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    return_code = DjiGimbalManager_Reset(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                         DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_ERROR;
        DjiTargetChase_LogGimbalEvent(failed_event,
                                      now_ms,
                                      return_code,
                                      NULL,
                                      0.0f,
                                      0.0f);
        return return_code;
    }

    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_RESETTING;
    s_gimbal_reset_started_ms = DjiTargetChase_NowMs();
    s_last_gimbal_target_seen_ms = 0;
    s_last_gimbal_target_near_edge = false;
    s_gimbal_edge_recovery_started_mono_ms = 0;
    s_gimbal_edge_recovery_yaw_direction = 0.0f;
    s_gimbal_edge_recovery_pitch_direction = 0.0f;
    DjiTargetChase_LogGimbalEvent(requested_event,
                                  now_ms,
                                  DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                  NULL,
                                  0.0f,
                                  0.0f);
    return return_code;
}

static bool DjiTargetChase_TrackGimbalToPixel(float u_px,
                                              float v_px,
                                              uint64_t now_ms)
{
    T_DjiFcSubscriptionGimbalAngles angles;
    float normalized_x;
    float normalized_y;
    float yaw_error_deg;
    float pitch_error_deg;
    float yaw_speed_degps;
    float pitch_speed_degps;
    float adaptive_gain;
    float error_norm_rad;
    float image_yaw_rate_degps;
    float image_pitch_rate_degps;
    float yaw_speed_limit_degps;
    float pitch_speed_limit_degps;
    float gimbal_deadband_deg;
    float filter_alpha;
    float command_dt_s;
    float yaw_step_degps;
    float pitch_step_degps;
    float feedforward_gain;
    float yaw_accel_degps2;
    float pitch_accel_degps2;
    T_TargetChasePrediction prediction;
    float prediction_rate_scale = 0.0f;
    uint64_t command_now_ms = DjiTargetChase_NowMs();
    uint64_t near_mode_mono_ms;
    uint64_t observation_age_ms = UINT64_MAX;
    bool target_just_acquired;
    bool near_low_latency;
    bool previous_near_high_rate;

    if (!s_gimbal_tracking_enabled ||
        s_gimbal_static_test_mode ||
        s_chase.config.fx_px <= 0.0f ||
        s_chase.config.fy_px <= 0.0f ||
        !isfinite(u_px) ||
        !isfinite(v_px)) {
        return false;
    }

    target_just_acquired =
        s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_SEARCHING ||
        s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_BODY_TURNING ||
        s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING ||
        (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY &&
         s_last_gimbal_target_seen_ms == 0);
    if (target_just_acquired) {
        s_body_search_turn_active = false;
        s_body_search_turn_heading_valid = false;
        s_body_search_turn_start_mono_ms = 0;
        s_gimbal_edge_recovery_started_mono_ms = 0;
        s_gimbal_edge_recovery_yaw_direction = 0.0f;
        s_gimbal_edge_recovery_pitch_direction = 0.0f;
        /*
         * Do not inject a zero-speed command between search and tracking.
         * The tracking filter and acceleration limiter below can transform
         * the final search velocity into the first tracking velocity without
         * a visible stop-and-restart at acquisition.
         */
        DjiTargetChase_LogGimbalEvent(
            "gimbal_search_to_tracking_continuous",
            now_ms,
            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
            NULL,
            s_last_gimbal_pitch_speed_degps,
            s_last_gimbal_yaw_speed_degps);
        s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_TRACKING;
        s_gimbal_centered_observations = 0;
        DjiTargetChase_LogGimbalEvent("gimbal_target_acquired",
                                      now_ms,
                                      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                      NULL,
                                      0.0f,
                                      0.0f);
    }
    s_last_gimbal_target_seen_ms = command_now_ms;
    s_last_gimbal_target_near_edge =
        u_px <= s_chase.config.image_width_px *
                    DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED ||
        u_px >= s_chase.config.image_width_px *
                    (1.0f - DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED) ||
        v_px <= s_chase.config.image_height_px *
                    DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED ||
        v_px >= s_chase.config.image_height_px *
                    (1.0f - DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED);

    if (!DjiTargetChase_UpdateGimbalBoresight(now_ms, &angles)) {
        if (DjiTargetChase_ShouldLogGimbalStatus(command_now_ms)) {
            DjiTargetChase_LogGimbalEvent("gimbal_angles_unavailable",
                                          now_ms,
                                          DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN,
                                          NULL,
                                          0.0f,
                                          0.0f);
        }
        return false;
    }

    normalized_x = (u_px - s_chase.config.cx_px) / s_chase.config.fx_px;
    if (s_chase.config.prediction_mode == 3) command_now_ms = DjiTargetChase_NowMs();
    normalized_y = (v_px - s_chase.config.cy_px) / s_chase.config.fy_px;
    yaw_error_deg = atanf(normalized_x) * DJI_TARGET_CHASE_RAD_TO_DEG *
                    s_chase.config.pixel_x_to_yaw_sign;
    pitch_error_deg = atanf(normalized_y) * DJI_TARGET_CHASE_RAD_TO_DEG *
                      s_chase.config.pixel_y_to_up_sign;
    if (s_chase.config.prediction_mode == 0 && DjiTargetChase_ProfileWorld(command_now_ms, &prediction)) {
        prediction_rate_scale = prediction.rate_scale;
        yaw_error_deg = DjiTargetChase_Wrap180(
            prediction.world_yaw_deg - angles.z);
        pitch_error_deg = prediction.world_pitch_deg - angles.x;
    }
    image_pitch_rate_degps =
        s_chase.filtered_v_rate_pxps / s_chase.config.fy_px *
        DJI_TARGET_CHASE_RAD_TO_DEG *
        s_chase.config.pixel_y_to_up_sign;
    image_yaw_rate_degps =
        s_chase.filtered_u_rate_pxps / s_chase.config.fx_px *
        DJI_TARGET_CHASE_RAD_TO_DEG *
        s_chase.config.pixel_x_to_yaw_sign;
    image_pitch_rate_degps *= prediction_rate_scale;
    image_yaw_rate_degps *= prediction_rate_scale;
    if (s_chase.config.prediction_mode != 0) {
        if (!DjiTargetChase_ProfileHybrid(command_now_ms, angles.z, angles.x,
            &yaw_error_deg, &pitch_error_deg, &image_yaw_rate_degps, &image_pitch_rate_degps)) {
            image_yaw_rate_degps = image_pitch_rate_degps = 0;
        }
    }
    if (command_now_ms >= now_ms) {
        observation_age_ms = command_now_ms - now_ms;
    }
    near_mode_mono_ms = DjiTargetChase_MonotonicMs();
    previous_near_high_rate = s_near_high_rate_tracking_active;
    if (!s_near_high_rate_tracking_active) {
        if (s_chase.filtered_box_angle_deg >=
                DJI_TARGET_CHASE_NEAR_HIGH_RATE_BOX_ANGLE_DEG &&
            observation_age_ms <=
                DJI_TARGET_CHASE_NEAR_LOW_LATENCY_MAX_MS) {
            s_near_high_rate_tracking_active = true;
            s_near_high_rate_changed_mono_ms = near_mode_mono_ms;
        }
    } else {
        bool minimum_hold_elapsed =
            s_near_high_rate_changed_mono_ms == 0 ||
            near_mode_mono_ms < s_near_high_rate_changed_mono_ms ||
            near_mode_mono_ms - s_near_high_rate_changed_mono_ms >=
                DJI_TARGET_CHASE_NEAR_HIGH_RATE_MIN_HOLD_MS;
        bool exit_requested =
            s_chase.filtered_box_angle_deg <
                DJI_TARGET_CHASE_NEAR_HIGH_RATE_EXIT_BOX_ANGLE_DEG ||
            observation_age_ms >
                DJI_TARGET_CHASE_NEAR_HIGH_RATE_EXIT_LATENCY_MS;

        if (minimum_hold_elapsed && exit_requested) {
            s_near_high_rate_tracking_active = false;
            s_near_high_rate_changed_mono_ms = near_mode_mono_ms;
        }
    }
    near_low_latency = s_near_high_rate_tracking_active;
    if (near_low_latency != previous_near_high_rate) {
        DjiTargetChase_LogGimbalEvent(
            near_low_latency
                ? "gimbal_near_high_rate_enabled"
                : "gimbal_near_high_rate_disabled",
            now_ms,
            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
            &angles,
            0.0f,
            0.0f);
    }

    /*
     * A close balloon occupies a large part of the image and its reported
     * centre moves sharply as the fabric swings or clips an edge. Expand only
     * the close-range hold region so the gimbal ignores that jitter; distant
     * targets keep the original one-degree centring precision.
     */
    gimbal_deadband_deg =
        DJI_TARGET_CHASE_GIMBAL_DEADBAND_DEG +
        (DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_DEG -
         DJI_TARGET_CHASE_GIMBAL_DEADBAND_DEG) *
            DjiTargetChase_Clamp(
                (s_chase.filtered_box_angle_deg -
                 DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_START_DEG) /
                    (DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_FULL_DEG -
                     DJI_TARGET_CHASE_GIMBAL_NEAR_DEADBAND_START_DEG),
                0.0f,
                1.0f);

    if (fabsf(yaw_error_deg) <= gimbal_deadband_deg) {
        yaw_error_deg = 0.0f;
    }
    if (fabsf(pitch_error_deg) <= gimbal_deadband_deg) {
        pitch_error_deg = 0.0f;
    }
    /*
     * Position-only centring used to stop the gimbal even while a close
     * balloon was visibly crossing the image. Keep velocity feed-forward
     * alive inside the enlarged near deadband; stop only when both position
     * and measured image motion are quiet.
     */
    if (yaw_error_deg == 0.0f && pitch_error_deg == 0.0f &&
        (!near_low_latency ||
         (fabsf(image_yaw_rate_degps) <=
              DJI_TARGET_CHASE_GIMBAL_CENTER_MOTION_HOLD_DEGPS &&
          fabsf(image_pitch_rate_degps) <=
              DJI_TARGET_CHASE_GIMBAL_CENTER_MOTION_HOLD_DEGPS))) {
        DjiTargetChase_StopGimbal(now_ms, "gimbal_target_centered_stop", false);
        if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING) {
            s_gimbal_centered_observations++;
        }
        if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY ||
            s_gimbal_centered_observations >= DJI_TARGET_CHASE_GIMBAL_CENTER_CONFIRMATIONS) {
            s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_READY;
            DjiTargetChase_LogGimbalEvent("gimbal_target_centered_hold_for_body_alignment",
                                         now_ms,
                                         DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                         &angles,
                                         0.0f,
                                         0.0f);
            return true;
        }
        return false;
    }
    s_gimbal_centered_observations = 0;

    error_norm_rad = sqrtf((yaw_error_deg * DJI_TARGET_CHASE_DEG_TO_RAD) *
                               (yaw_error_deg * DJI_TARGET_CHASE_DEG_TO_RAD) +
                           (pitch_error_deg * DJI_TARGET_CHASE_DEG_TO_RAD) *
                               (pitch_error_deg * DJI_TARGET_CHASE_DEG_TO_RAD));
    adaptive_gain = DjiTargetChase_GimbalAdaptiveGain(error_norm_rad);
    pitch_speed_limit_degps = DjiTargetChase_GimbalTrackingSpeedLimit(
        fabsf(pitch_error_deg), image_pitch_rate_degps, near_low_latency);
    yaw_speed_limit_degps = DjiTargetChase_GimbalTrackingSpeedLimit(
        fabsf(yaw_error_deg), image_yaw_rate_degps, near_low_latency);
    feedforward_gain = near_low_latency
        ? DJI_TARGET_CHASE_GIMBAL_NEAR_RATE_FEEDFORWARD_GAIN
        : DJI_TARGET_CHASE_GIMBAL_RATE_FEEDFORWARD_GAIN;
    pitch_speed_degps = DjiTargetChase_Clamp(
        pitch_error_deg * adaptive_gain +
            feedforward_gain * image_pitch_rate_degps,
        -pitch_speed_limit_degps,
        pitch_speed_limit_degps);
    yaw_speed_degps = DjiTargetChase_Clamp(
        yaw_error_deg * adaptive_gain +
            feedforward_gain * image_yaw_rate_degps,
        -yaw_speed_limit_degps,
        yaw_speed_limit_degps);

    /*
     * Near center, strongly filter detector jitter and one-frame direction
     * reversals. Far from center, keep a larger alpha so dynamic targets can
     * still be caught quickly.
     */
    filter_alpha = near_low_latency
        ? DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_ACTIVE_FILTER_ALPHA
        : (fmaxf(fabsf(yaw_error_deg), fabsf(pitch_error_deg)) <=
                   DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_ERROR_DEG
               ? DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_FILTER_ALPHA
               : DJI_TARGET_CHASE_GIMBAL_TRACK_FAR_FILTER_ALPHA);
    yaw_speed_degps = s_last_gimbal_yaw_speed_degps +
                      filter_alpha *
                          (yaw_speed_degps - s_last_gimbal_yaw_speed_degps);
    pitch_speed_degps = s_last_gimbal_pitch_speed_degps +
                        filter_alpha *
                            (pitch_speed_degps - s_last_gimbal_pitch_speed_degps);

    command_dt_s = 0.05f;
    if (s_last_gimbal_command_ms != 0 &&
        command_now_ms >= s_last_gimbal_command_ms) {
        command_dt_s = DjiTargetChase_Clamp(
            (float) (command_now_ms - s_last_gimbal_command_ms) / 1000.0f,
            0.05f,
            0.25f);
    }
    yaw_accel_degps2 = near_low_latency
        ? DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_YAW_ACCEL_DEGPS2
        : DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_ACCEL_DEGPS2;
    pitch_accel_degps2 = near_low_latency
        ? DJI_TARGET_CHASE_GIMBAL_TRACK_NEAR_PITCH_ACCEL_DEGPS2
        : DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_ACCEL_DEGPS2;
    yaw_step_degps = yaw_accel_degps2 * command_dt_s;
    pitch_step_degps = pitch_accel_degps2 * command_dt_s;
    yaw_speed_degps = DjiTargetChase_MoveToward(
        s_last_gimbal_yaw_speed_degps, yaw_speed_degps, yaw_step_degps);
    pitch_speed_degps = DjiTargetChase_MoveToward(
        s_last_gimbal_pitch_speed_degps, pitch_speed_degps, pitch_step_degps);

    /* Do not drive farther into a mechanical boundary while centering. */
    if ((angles.z > DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_SOFT_LIMIT_DEG && yaw_speed_degps > 0.0f) ||
        (angles.z < -DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_SOFT_LIMIT_DEG && yaw_speed_degps < 0.0f)) {
        yaw_speed_degps = 0.0f;
    }
    if ((angles.x > DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_SOFT_LIMIT_UP_DEG && pitch_speed_degps > 0.0f) ||
        (angles.x < DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_SOFT_LIMIT_DOWN_DEG && pitch_speed_degps < 0.0f)) {
        pitch_speed_degps = 0.0f;
    }
    (void) DjiTargetChase_SendGimbalSpeed(pitch_speed_degps,
                                          yaw_speed_degps,
                                          now_ms,
                                          "gimbal_track_speed_command",
                                          target_just_acquired);
    return false;
}

static float DjiTargetChase_SearchAxisSpeed(float error_deg, float max_speed_degps)
{
    float speed_degps;

    if (fabsf(error_deg) <= DJI_TARGET_CHASE_GIMBAL_SEARCH_POSITION_TOLERANCE_DEG) {
        return 0.0f;
    }

    speed_degps = error_deg * DJI_TARGET_CHASE_GIMBAL_SEARCH_POSITION_GAIN;
    return DjiTargetChase_Clamp(speed_degps, -max_speed_degps, max_speed_degps);
}

static void DjiTargetChase_BeginGimbalSectorSearch(uint64_t now_ms,
                                                   const char *event,
                                                   const T_DjiFcSubscriptionGimbalAngles *angles)
{
    TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_RESET);
    s_chase.kf_epoch_ms = DjiTargetChase_NowMs();
    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_SEARCHING;
    s_search_phase = DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT;
    s_gimbal_search_center_yaw_deg = s_chase.config.gimbal_mount_yaw_offset_deg;
    s_gimbal_search_center_pitch_deg = 0.0f;
    s_body_search_turn_active = false;
    s_body_search_turn_heading_valid = false;
    s_body_search_turn_start_mono_ms = 0;
    s_gimbal_edge_recovery_started_mono_ms = 0;
    s_gimbal_edge_recovery_yaw_direction = 0.0f;
    s_gimbal_edge_recovery_pitch_direction = 0.0f;
    s_search_exhausted = false;
    DjiTargetChase_LogGimbalEvent(event,
                                  now_ms,
                                  DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                  angles,
                                  0.0f,
                                  0.0f);
}

static void DjiTargetChase_BeginNearEdgeRecovery(
    uint64_t now_ms,
    const T_DjiFcSubscriptionGimbalAngles *angles)
{
    float horizontal_error_px =
        s_chase.filtered_u_px - s_chase.config.cx_px;
    float vertical_error_px =
        s_chase.filtered_v_px - s_chase.config.cy_px;
    bool horizontal_edge =
        s_chase.filtered_u_px <=
            s_chase.config.image_width_px *
                DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED ||
        s_chase.filtered_u_px >=
            s_chase.config.image_width_px *
                (1.0f - DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED);
    bool vertical_edge =
        s_chase.filtered_v_px <=
            s_chase.config.image_height_px *
                DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED ||
        s_chase.filtered_v_px >=
            s_chase.config.image_height_px *
                (1.0f - DJI_TARGET_CHASE_GIMBAL_EDGE_MARGIN_NORMALIZED);

    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING;
    s_gimbal_edge_recovery_started_mono_ms = DjiTargetChase_MonotonicMs();
    s_near_high_rate_tracking_active = false;
    s_gimbal_edge_recovery_yaw_direction = horizontal_edge
        ? (horizontal_error_px > 0.0f ? s_chase.config.pixel_x_to_yaw_sign
                                     : -s_chase.config.pixel_x_to_yaw_sign)
        : 0.0f;
    s_gimbal_edge_recovery_pitch_direction = vertical_edge
        ? (vertical_error_px > 0.0f ? s_chase.config.pixel_y_to_up_sign
                                   : -s_chase.config.pixel_y_to_up_sign)
        : 0.0f;

    /*
     * If the last tracking command already contains a clear direction, use
     * it. This retains measured target motion at a corner instead of relying
     * only on which half of the image contained the final box.
     */
    if (horizontal_edge &&
        fabsf(s_last_gimbal_yaw_speed_degps) >= 0.5f) {
        s_gimbal_edge_recovery_yaw_direction =
            s_last_gimbal_yaw_speed_degps > 0.0f ? 1.0f : -1.0f;
    }
    if (vertical_edge &&
        fabsf(s_last_gimbal_pitch_speed_degps) >= 0.5f) {
        s_gimbal_edge_recovery_pitch_direction =
            s_last_gimbal_pitch_speed_degps > 0.0f ? 1.0f : -1.0f;
    }
    DjiTargetChase_LogGimbalEvent(
        "gimbal_near_edge_recovery_started",
        now_ms,
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
        angles,
        s_gimbal_edge_recovery_pitch_direction *
            DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_PITCH_SPEED_DEGPS,
        s_gimbal_edge_recovery_yaw_direction *
            DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_YAW_SPEED_DEGPS);
}

static void DjiTargetChase_BeginBodySearchTurn(uint64_t now_ms,
                                              const T_DjiFcSubscriptionGimbalAngles *angles)
{
    if (!s_body_search_enabled ||
        s_search_coverage.sector_count <= 1U ||
        s_search_coverage.body_yaw_step_deg <= 0.0f) {
        DjiTargetChase_BeginGimbalSectorSearch(now_ms,
                                               "gimbal_sector_rescan_without_body_turn",
                                               angles);
        return;
    }

    DjiTargetChase_StopGimbal(now_ms, "gimbal_sector_scan_complete_stop", true);
    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_BODY_TURNING;
    s_search_phase = DJI_TARGET_CHASE_SEARCH_WAIT_BODY_TURN;
    s_body_search_turn_active = true;
    s_body_search_turn_heading_valid = false;
    s_body_search_turn_start_mono_ms = DjiTargetChase_MonotonicMs();
    DjiTargetChase_LogGimbalEvent("gimbal_sector_scan_complete_request_body_turn",
                                  now_ms,
                                  DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                  angles,
                                  0.0f,
                                  0.0f);
}

static void DjiTargetChase_BeginFastStageSearchTurn(uint64_t now_ms)
{
    if (!s_body_search_enabled ||
        s_search_coverage.sector_count <= 1U ||
        s_search_coverage.body_yaw_step_deg <= 0.0f) {
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms, "stage_fast_search_fallback_full_raster", NULL);
        return;
    }

    DjiTargetChase_StopGimbal(
        now_ms, "stage_fast_search_cancel_previous_gimbal_motion", true);
    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_BODY_TURNING;
    s_search_phase = DJI_TARGET_CHASE_SEARCH_WAIT_BODY_TURN;
    s_body_search_turn_active = true;
    s_body_search_turn_heading_valid = false;
    s_body_search_turn_start_mono_ms = DjiTargetChase_MonotonicMs();
    DjiTargetChase_LogGimbalEvent(
        "stage_fast_search_begin_body_turn",
        now_ms,
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
        NULL,
        0.0f,
        0.0f);
}

void DjiTargetChase_GimbalTick(uint64_t now_ms)
{
    T_DjiFcSubscriptionGimbalAngles angles;
    T_TargetChaseAircraftState aircraft_state;
    uint64_t current_ms = DjiTargetChase_NowMs();
    bool angles_valid;
    bool command_due;

    if (!s_initialized || DjiTargetChase_IsControlInhibited() || !s_gimbal_tracking_enabled) {
        return;
    }

    command_due = s_last_gimbal_command_ms == 0 ||
                  current_ms < s_last_gimbal_command_ms ||
                  current_ms - s_last_gimbal_command_ms >= DJI_TARGET_CHASE_GIMBAL_COMMAND_INTERVAL_MS;
    if (command_due && DjiTargetChase_IsSuccess(DjiTargetChase_ReadState(&aircraft_state))) {
        TargetChase_UpdateAircraftState(&s_chase, &aircraft_state);
    }
    angles_valid = DjiTargetChase_UpdateGimbalBoresight(now_ms, &angles);

    if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_RESETTING) {
        bool centered = angles_valid &&
                        fabsf(s_chase.camera_boresight_yaw_deg) <= DJI_TARGET_CHASE_GIMBAL_RESET_TOLERANCE_DEG &&
                        fabsf(s_chase.camera_boresight_vertical_deg) <= DJI_TARGET_CHASE_GIMBAL_RESET_TOLERANCE_DEG;
        bool settled = s_gimbal_reset_started_ms != 0 &&
                       current_ms >= s_gimbal_reset_started_ms &&
                       current_ms - s_gimbal_reset_started_ms >= DJI_TARGET_CHASE_GIMBAL_RESET_SETTLE_MS;

        if (!centered && !settled) {
            return;
        }

        if (s_gimbal_static_test_mode) {
            DjiTargetChase_StopGimbal(now_ms, "gimbal_static_test_stop", true);
            s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_STATIC_TEST;
            DjiTargetChase_LogGimbalEvent("gimbal_static_test_hold",
                                          now_ms,
                                          DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                          angles_valid ? &angles : NULL,
                                          0.0f,
                                          0.0f);
            return;
        }

        if (!s_gimbal_search_enabled) {
            DjiTargetChase_StopGimbal(now_ms, "gimbal_track_only_ready_stop", true);
            s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_READY;
            DjiTargetChase_LogGimbalEvent("gimbal_reset_complete_wait_target",
                                          now_ms,
                                          DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                          angles_valid ? &angles : NULL,
                                          0.0f,
                                          0.0f);
            return;
        }

        /*
         * DJI Reset defines the camera's body-forward home angle. Preserve
         * its sign: taking fabs() here would mis-calibrate a negative mount
         * offset and double the alignment error.
         */
        if (angles_valid && getenv("BALLOON_GIMBAL_MOUNT_YAW_OFFSET_DEG") == NULL) {
            s_chase.config.gimbal_mount_yaw_offset_deg =
                s_chase.camera_boresight_yaw_deg;
            DjiTargetChase_LogGimbalEvent("gimbal_mount_yaw_auto_calibrated",
                                          now_ms,
                                          DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                          &angles,
                                          0.0f,
                                          0.0f);
        }
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms,
            "gimbal_reset_complete_enter_sector_search",
            angles_valid ? &angles : NULL);
    }

    if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING ||
        s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY) {
        uint64_t target_missing_ms =
            s_last_gimbal_target_seen_ms != 0 &&
                    current_ms >= s_last_gimbal_target_seen_ms
                ? current_ms - s_last_gimbal_target_seen_ms
                : 0;
        uint64_t target_lost_timeout_ms =
            s_last_gimbal_target_near_edge
                ? DJI_TARGET_CHASE_GIMBAL_EDGE_TARGET_LOST_TIMEOUT_MS
                : DJI_TARGET_CHASE_GIMBAL_TARGET_LOST_TIMEOUT_MS;
        if (s_gimbal_search_enabled &&
            s_last_gimbal_target_near_edge &&
            s_chase.filtered_box_angle_deg >=
                DJI_TARGET_CHASE_GIMBAL_NEAR_BOX_ANGLE_DEG &&
            target_missing_ms >=
                DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_START_MS) {
            DjiTargetChase_BeginNearEdgeRecovery(
                now_ms, angles_valid ? &angles : NULL);
            return;
        }
        if (s_chase.filtered_box_angle_deg >=
            DJI_TARGET_CHASE_GIMBAL_NEAR_BOX_ANGLE_DEG) {
            target_lost_timeout_ms =
                DJI_TARGET_CHASE_GIMBAL_NEAR_TARGET_LOST_TIMEOUT_MS;
        }
        if (s_last_gimbal_target_seen_ms == 0 ||
            current_ms < s_last_gimbal_target_seen_ms ||
            current_ms - s_last_gimbal_target_seen_ms <=
                target_lost_timeout_ms) {
            return;
        }

        DjiTargetChase_StopGimbal(now_ms, "gimbal_target_lost_stop", true);
        s_gimbal_centered_observations = 0;
        s_last_gimbal_target_seen_ms = 0;
        s_last_gimbal_target_near_edge = false;
        if (!s_gimbal_search_enabled) {
            s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_READY;
            return;
        }
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms,
            "gimbal_target_lost_resume_sector_search",
            angles_valid ? &angles : NULL);
        return;
    }

    if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_STATIC_TEST) {
        return;
    }

    if (s_gimbal_state ==
        DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING) {
        uint64_t recovery_now_ms = DjiTargetChase_MonotonicMs();
        uint64_t recovery_elapsed_ms;
        bool continuing_outward;
        float yaw_direction;
        float pitch_direction;
        float yaw_speed_degps;
        float pitch_speed_degps;
        float relative_yaw_deg;

        if (!angles_valid ||
            s_gimbal_edge_recovery_started_mono_ms == 0 ||
            recovery_now_ms < s_gimbal_edge_recovery_started_mono_ms) {
            DjiTargetChase_StopGimbal(
                now_ms, "gimbal_near_edge_recovery_angles_unavailable", false);
            DjiTargetChase_BeginGimbalSectorSearch(
                now_ms,
                "gimbal_near_edge_recovery_fallback_search",
                angles_valid ? &angles : NULL);
            return;
        }

        recovery_elapsed_ms =
            recovery_now_ms - s_gimbal_edge_recovery_started_mono_ms;
        if (recovery_elapsed_ms >=
            DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_TOTAL_MS) {
            DjiTargetChase_StopGimbal(
                now_ms, "gimbal_near_edge_recovery_complete_stop", true);
            s_last_gimbal_target_seen_ms = 0;
            s_last_gimbal_target_near_edge = false;
            DjiTargetChase_BeginGimbalSectorSearch(
                now_ms,
                "gimbal_near_edge_recovery_escalate_sector_search",
                &angles);
            return;
        }

        continuing_outward =
            recovery_elapsed_ms <
            DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_FORWARD_MS;
        yaw_direction = continuing_outward
                            ? s_gimbal_edge_recovery_yaw_direction
                            : -s_gimbal_edge_recovery_yaw_direction;
        pitch_direction = continuing_outward
                              ? s_gimbal_edge_recovery_pitch_direction
                              : -0.5f *
                                    s_gimbal_edge_recovery_pitch_direction;
        yaw_speed_degps =
            yaw_direction *
            DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_YAW_SPEED_DEGPS;
        pitch_speed_degps =
            pitch_direction *
            DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_PITCH_SPEED_DEGPS;

        relative_yaw_deg = DjiTargetChase_Wrap180(
            s_chase.camera_boresight_yaw_deg -
            s_chase.config.gimbal_mount_yaw_offset_deg);
        if ((relative_yaw_deg >=
                 DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_SOFT_LIMIT_DEG &&
             yaw_speed_degps > 0.0f) ||
            (relative_yaw_deg <=
                 -DJI_TARGET_CHASE_GIMBAL_TRACK_YAW_SOFT_LIMIT_DEG &&
             yaw_speed_degps < 0.0f)) {
            yaw_speed_degps = -yaw_speed_degps;
        }
        if ((s_chase.camera_boresight_vertical_deg >=
                 DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_SOFT_LIMIT_UP_DEG &&
             pitch_speed_degps > 0.0f) ||
            (s_chase.camera_boresight_vertical_deg <=
                 DJI_TARGET_CHASE_GIMBAL_TRACK_PITCH_SOFT_LIMIT_DOWN_DEG &&
             pitch_speed_degps < 0.0f)) {
            pitch_speed_degps = -pitch_speed_degps;
        }
        (void) DjiTargetChase_SendGimbalSpeed(
            pitch_speed_degps,
            yaw_speed_degps,
            now_ms,
            continuing_outward
                ? "gimbal_near_edge_recovery_follow_exit"
                : "gimbal_near_edge_recovery_reverse_sweep",
            false);
        return;
    }

    if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_BODY_TURNING) {
        float relative_yaw_deg;
        float relative_pitch_deg;
        float pitch_speed_degps;
        float yaw_speed_degps;

        if (!angles_valid) {
            DjiTargetChase_StopGimbal(now_ms,
                                      "gimbal_body_turn_angles_unavailable_stop",
                                      false);
            return;
        }

        relative_yaw_deg = DjiTargetChase_Wrap180(
            s_chase.camera_boresight_yaw_deg -
            s_chase.config.gimbal_mount_yaw_offset_deg);
        relative_pitch_deg = s_chase.camera_boresight_vertical_deg;
        pitch_speed_degps = DjiTargetChase_Clamp(
            -relative_pitch_deg * DJI_TARGET_CHASE_BODY_SEARCH_GIMBAL_CENTER_GAIN,
            -s_gimbal_search_pitch_speed_degps,
            s_gimbal_search_pitch_speed_degps);
        /*
         * The gimbal runs in YAW_FOLLOW mode, so zero yaw speed follows the
         * aircraft naturally. Only correct measured body-relative offset here.
         * Never feed the planned body rate into the gimbal: in NO_FLIGHT the
         * body does not actually turn, and that feed-forward would make the
         * camera rotate by itself.
         */
        yaw_speed_degps = DjiTargetChase_Clamp(
            -relative_yaw_deg * DJI_TARGET_CHASE_BODY_SEARCH_GIMBAL_CENTER_GAIN,
            -s_gimbal_search_yaw_speed_degps,
            s_gimbal_search_yaw_speed_degps);
        (void) DjiTargetChase_SendGimbalSpeed(
            pitch_speed_degps,
            yaw_speed_degps,
            now_ms,
            "gimbal_hold_forward_during_body_turn",
            false);
        return;
    }

    if (s_gimbal_state != DJI_TARGET_CHASE_GIMBAL_SEARCHING ||
        s_search_exhausted) {
        return;
    }

    if (angles_valid) {
        float rel_yaw_deg;
        float rel_pitch_deg;
        float desired_yaw_deg;
        float desired_pitch_deg;
        float yaw_error_deg;
        float pitch_error_deg;
        float yaw_speed_degps;
        float pitch_speed_degps;

        rel_yaw_deg = DjiTargetChase_Wrap180(
            s_chase.camera_boresight_yaw_deg -
            s_gimbal_search_center_yaw_deg);
        rel_pitch_deg = s_chase.camera_boresight_vertical_deg -
                        s_gimbal_search_center_pitch_deg;

        switch (s_search_phase) {
            case DJI_TARGET_CHASE_SEARCH_SWEEP_LOWER_RIGHT:
                desired_yaw_deg = DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_LIMIT_DEG;
                desired_pitch_deg = DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MIN_DEG;
                break;
            case DJI_TARGET_CHASE_SEARCH_SEEK_UPPER_RIGHT:
                desired_yaw_deg = DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_LIMIT_DEG;
                desired_pitch_deg = DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MAX_DEG;
                break;
            case DJI_TARGET_CHASE_SEARCH_SWEEP_UPPER_LEFT:
                desired_yaw_deg = -DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_LIMIT_DEG;
                desired_pitch_deg = DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MAX_DEG;
                break;
            case DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT:
            default:
                desired_yaw_deg = -DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_LIMIT_DEG;
                desired_pitch_deg = DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MIN_DEG;
                break;
        }

        yaw_error_deg = desired_yaw_deg - rel_yaw_deg;
        pitch_error_deg = desired_pitch_deg - rel_pitch_deg;
        if (fabsf(yaw_error_deg) <=
                DJI_TARGET_CHASE_GIMBAL_SEARCH_POSITION_TOLERANCE_DEG &&
            fabsf(pitch_error_deg) <=
                DJI_TARGET_CHASE_GIMBAL_SEARCH_POSITION_TOLERANCE_DEG) {
            switch (s_search_phase) {
                case DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT:
                    s_search_phase = DJI_TARGET_CHASE_SEARCH_SWEEP_LOWER_RIGHT;
                    break;
                case DJI_TARGET_CHASE_SEARCH_SWEEP_LOWER_RIGHT:
                    s_search_phase = DJI_TARGET_CHASE_SEARCH_SEEK_UPPER_RIGHT;
                    break;
                case DJI_TARGET_CHASE_SEARCH_SEEK_UPPER_RIGHT:
                    s_search_phase = DJI_TARGET_CHASE_SEARCH_SWEEP_UPPER_LEFT;
                    break;
                case DJI_TARGET_CHASE_SEARCH_SWEEP_UPPER_LEFT:
                    DjiTargetChase_BeginBodySearchTurn(now_ms, &angles);
                    return;
                case DJI_TARGET_CHASE_SEARCH_WAIT_BODY_TURN:
                default:
                    return;
            }
            DjiTargetChase_LogGimbalEvent("gimbal_search_raster_phase_complete",
                                          now_ms,
                                          DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                          &angles,
                                          0.0f,
                                          0.0f);
            return;
        }

        yaw_speed_degps = DjiTargetChase_SearchAxisSpeed(
            yaw_error_deg,
            s_gimbal_search_yaw_speed_degps);
        pitch_speed_degps = DjiTargetChase_SearchAxisSpeed(
            pitch_error_deg,
            s_gimbal_search_pitch_speed_degps);
        (void) DjiTargetChase_SendGimbalSpeed(
            pitch_speed_degps,
            yaw_speed_degps,
            now_ms,
            "gimbal_search_raster_command",
            false);
        return;
    }

    DjiTargetChase_StopGimbal(now_ms, "gimbal_search_angles_unavailable_stop", false);
}

static void DjiTargetChase_InitGimbalTracking(uint64_t now_ms)
{
    T_DjiReturnCode return_code;
    const bool visible_target_only = DjiTargetChase_VisibleTargetOnly();

    s_gimbal_tracking_enabled = false;
    s_gimbal_static_test_mode = !visible_target_only &&
        DjiTargetChase_EnvFlagEnabled("BALLOON_GIMBAL_STATIC_TEST");
    s_gimbal_search_enabled = !visible_target_only &&
        DjiTargetChase_EnvFlagEnabledByDefault("BALLOON_GIMBAL_SEARCH");
    s_body_search_enabled =
        s_gimbal_search_enabled &&
        DjiTargetChase_EnvFlagEnabled("BALLOON_BODY_SEARCH");
    s_gimbal_angles_subscribed = false;
    s_gimbal_manager_initialized = false;
    s_last_gimbal_command_ms = 0;
    s_last_gimbal_status_log_ms = 0;
    s_last_gimbal_target_seen_ms = 0;
    s_last_gimbal_target_near_edge = false;
    s_gimbal_edge_recovery_started_mono_ms = 0;
    s_gimbal_edge_recovery_yaw_direction = 0.0f;
    s_gimbal_edge_recovery_pitch_direction = 0.0f;
    s_gimbal_reset_started_ms = 0;
    s_last_gimbal_pitch_speed_degps = 0.0f;
    s_last_gimbal_yaw_speed_degps = 0.0f;
    s_near_high_rate_tracking_active = false;
    s_near_high_rate_changed_mono_ms = 0;
    s_gimbal_search_center_yaw_deg = 0.0f;
    s_gimbal_search_center_pitch_deg = 0.0f;
    s_gimbal_search_yaw_speed_degps = DjiTargetChase_ReadFloatEnvBounded(
        "BALLOON_GIMBAL_SEARCH_YAW_SPEED_DEGPS",
        DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_SPEED_DEGPS,
        10.0f,
        90.0f);
    s_gimbal_search_pitch_speed_degps = DjiTargetChase_ReadFloatEnvBounded(
        "BALLOON_GIMBAL_SEARCH_PITCH_SPEED_DEGPS",
        DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_SPEED_DEGPS,
        5.0f,
        45.0f);
    s_body_search_yaw_rate_degps = DjiTargetChase_ReadFloatEnvBounded(
        "BALLOON_BODY_SEARCH_YAW_RATE_DEGPS",
        DJI_TARGET_CHASE_BODY_SEARCH_YAW_RATE_DEGPS,
        5.0f,
        20.0f);
    s_gimbal_centered_observations = 0;
    s_search_phase = DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT;
    memset(&s_search_coverage, 0, sizeof(s_search_coverage));
    s_body_search_turn_active = false;
    s_body_search_turn_heading_valid = false;
    s_body_search_turn_start_yaw_deg = 0.0f;
    s_body_search_turn_start_mono_ms = 0;
    s_last_body_search_log_mono_ms = 0;
    s_search_sector_index = 0;
    s_search_exhausted = false;
    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_DISABLED;

    if (s_gimbal_search_enabled) {
        float horizontal_fov_deg =
            2.0f * atanf(s_chase.config.image_width_px /
                         (2.0f * s_chase.config.fx_px)) *
            DJI_TARGET_CHASE_RAD_TO_DEG;
        float vertical_fov_deg =
            2.0f * atanf(s_chase.config.image_height_px /
                         (2.0f * s_chase.config.fy_px)) *
            DJI_TARGET_CHASE_RAD_TO_DEG;

        if (!TargetChase_ComputeSearchCoverage(
                horizontal_fov_deg,
                vertical_fov_deg,
                DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_LIMIT_DEG,
                DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MIN_DEG,
                DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_MAX_DEG,
                DJI_TARGET_CHASE_BODY_SEARCH_REQUESTED_OVERLAP_DEG,
                &s_search_coverage)) {
            s_body_search_enabled = false;
        }
    }

    if (!DjiTargetChase_RequestedGimbalTracking()) {
        DjiTargetChase_LogGimbalEvent("gimbal_tracking_disabled_by_configuration",
                                      now_ms,
                                      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                      NULL,
                                      0.0f,
                                      0.0f);
        return;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        DjiTargetChase_LogGimbalEvent("gimbal_angles_subscribe_failed",
                                      now_ms,
                                      return_code,
                                      NULL,
                                      0.0f,
                                      0.0f);
        return;
    }
    s_gimbal_angles_subscribed = true;

    return_code = DjiGimbalManager_Init();
    if (!DjiTargetChase_IsSuccess(return_code)) {
        DjiTargetChase_LogGimbalEvent("gimbal_manager_init_failed",
                                      now_ms,
                                      return_code,
                                      NULL,
                                      0.0f,
                                      0.0f);
        (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
        s_gimbal_angles_subscribed = false;
        return;
    }
    s_gimbal_manager_initialized = true;

    if (DjiTargetChase_IsControlInhibited()) return;
    if (!visible_target_only) {
        return_code = DjiGimbalManager_SetMode(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                           DJI_GIMBAL_MODE_YAW_FOLLOW);
        if (!DjiTargetChase_IsSuccess(return_code)) {
            DjiTargetChase_LogGimbalEvent("gimbal_yaw_follow_mode_failed",
                                      now_ms,
                                      return_code,
                                      NULL,
                                      0.0f,
                                      0.0f);
            (void) DjiGimbalManager_Deinit();
            (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
            s_gimbal_manager_initialized = false;
            s_gimbal_angles_subscribed = false;
            return;
        }
    }

    if (DjiTargetChase_IsControlInhibited()) return;
    s_gimbal_tracking_enabled = true;
    DjiTargetChase_LogGimbalEvent(
        visible_target_only ? "gimbal_visible_target_only_configured" :
        s_body_search_enabled
            ? "gimbal_body_search_coverage_configured"
            : "gimbal_only_search_configured",
        now_ms,
        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
        NULL,
        0.0f,
        0.0f);
    (void) DjiGimbalManager_SetControllerSmoothFactor(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                                       DJI_GIMBAL_AXIS_PITCH,
                                                       DJI_TARGET_CHASE_GIMBAL_SMOOTH_FACTOR);
    (void) DjiGimbalManager_SetControllerSmoothFactor(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                                       DJI_GIMBAL_AXIS_YAW,
                                                       DJI_TARGET_CHASE_GIMBAL_SMOOTH_FACTOR);
    (void) DjiGimbalManager_SetControllerMaxSpeedPercentage(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                                            DJI_GIMBAL_AXIS_PITCH,
                                                            DJI_TARGET_CHASE_GIMBAL_MAX_SPEED_PERCENT);
    (void) DjiGimbalManager_SetControllerMaxSpeedPercentage(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                                            DJI_GIMBAL_AXIS_YAW,
                                                            DJI_TARGET_CHASE_GIMBAL_MAX_SPEED_PERCENT);
    if (visible_target_only) {
        if (DjiTargetChase_IsControlInhibited()) return;
        /* READY with no observation waits without motion. The first valid
         * OnTarget call transitions to TRACKING and centers that target. */
        s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_READY;
        (void) DjiTargetChase_UpdateGimbalBoresight(now_ms, NULL);
        DjiTargetChase_LogGimbalEvent("gimbal_wait_visible_target_preserve_pose",
                                      now_ms,
                                      DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                      NULL, 0.0f, 0.0f);
        return;
    }
    return_code = DjiTargetChase_BeginGimbalReset(now_ms,
                                                  "gimbal_start_reset_requested",
                                                  "gimbal_start_reset_failed");
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return;
    }
    (void) DjiTargetChase_UpdateGimbalBoresight(now_ms, NULL);
}

static void DjiTargetChase_DeInitGimbalTracking(uint64_t now_ms)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();

    if (s_gimbal_manager_initialized) {
        if (!__atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE))
            DjiTargetChase_StopGimbal(now_ms, "gimbal_shutdown_stop", true);
        /* A stop/takeover must not move the camera back to its reset pose. */
        if (!DjiTargetChase_IsControlInhibited() &&
            !DjiTargetChase_VisibleTargetOnly()) {
            s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_RESETTING;
            DjiTargetChase_LogGimbalEvent("gimbal_shutdown_reset_requested", now_ms,
                DjiGimbalManager_Reset(DJI_TARGET_CHASE_GIMBAL_MOUNT_POSITION,
                                      DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW), NULL, 0.0f, 0.0f);
            if (osal != NULL) osal->TaskSleepMs(DJI_TARGET_CHASE_GIMBAL_RESET_SETTLE_MS);
        }
        (void) DjiGimbalManager_Deinit();
    }
    if (s_gimbal_angles_subscribed) {
        (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    }

    s_gimbal_tracking_enabled = false;
    s_gimbal_static_test_mode = false;
    s_gimbal_search_enabled = false;
    s_body_search_enabled = false;
    s_gimbal_manager_initialized = false;
    s_gimbal_angles_subscribed = false;
    s_last_gimbal_command_ms = 0;
    s_last_gimbal_status_log_ms = 0;
    s_last_gimbal_target_seen_ms = 0;
    s_last_gimbal_target_near_edge = false;
    s_gimbal_edge_recovery_started_mono_ms = 0;
    s_gimbal_edge_recovery_yaw_direction = 0.0f;
    s_gimbal_edge_recovery_pitch_direction = 0.0f;
    s_gimbal_reset_started_ms = 0;
    s_last_gimbal_pitch_speed_degps = 0.0f;
    s_last_gimbal_yaw_speed_degps = 0.0f;
    s_near_high_rate_tracking_active = false;
    s_near_high_rate_changed_mono_ms = 0;
    s_gimbal_search_yaw_speed_degps =
        DJI_TARGET_CHASE_GIMBAL_SEARCH_YAW_SPEED_DEGPS;
    s_gimbal_search_pitch_speed_degps =
        DJI_TARGET_CHASE_GIMBAL_SEARCH_PITCH_SPEED_DEGPS;
    s_body_search_yaw_rate_degps =
        DJI_TARGET_CHASE_BODY_SEARCH_YAW_RATE_DEGPS;
    s_gimbal_centered_observations = 0;
    s_search_phase = DJI_TARGET_CHASE_SEARCH_SEEK_LOWER_LEFT;
    memset(&s_search_coverage, 0, sizeof(s_search_coverage));
    s_body_search_turn_active = false;
    s_body_search_turn_heading_valid = false;
    s_body_search_turn_start_yaw_deg = 0.0f;
    s_body_search_turn_start_mono_ms = 0;
    s_last_body_search_log_mono_ms = 0;
    s_search_sector_index = 0;
    s_search_exhausted = false;
    s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_DISABLED;
    TargetChase_SetCameraBoresight(&s_chase, 0.0f, 0.0f);
    DjiTargetChase_LogGimbalEvent("gimbal_tracking_deinitialized",
                                  now_ms,
                                  DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                  NULL,
                                  0.0f,
                                  0.0f);
}

static bool DjiTargetChase_ShouldLogRepeatCommand(uint64_t now_ms, T_DjiReturnCode return_code)
{
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return true;
    }

    if (s_last_repeat_command_log_ms == 0 ||
        now_ms < s_last_repeat_command_log_ms ||
        now_ms - s_last_repeat_command_log_ms >= 1000ULL) {
        s_last_repeat_command_log_ms = now_ms;
        return true;
    }

    return false;
}

static bool DjiTargetChase_ReadJsonNumber(const cJSON *root, const char *name, double *value)
{
    cJSON *item;

    if (root == NULL || name == NULL || value == NULL) {
        return false;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, name);
    if (!cJSON_IsNumber(item)) {
        return false;
    }

    *value = item->valuedouble;
    return true;
}

static bool DjiTargetChase_ReadRidConfig(T_DjiFlightControllerRidInfo *rid_info)
{
    FILE *fp;
    char buffer[DJI_TARGET_CHASE_RID_CONFIG_MAX_BYTES];
    size_t length;
    cJSON *root;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    bool has_rad;
    bool has_deg;
    bool has_altitude;

    if (rid_info == NULL) {
        return false;
    }

    fp = fopen(DJI_TARGET_CHASE_RID_CONFIG_FILE, "rb");
    if (fp == NULL) {
        DjiTargetChase_LogEvent("rid_config_missing",
                                s_last_now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER,
                                NULL);
        return false;
    }

    length = fread(buffer, 1, sizeof(buffer) - 1, fp);
    if (ferror(fp) != 0 || (!feof(fp) && length == sizeof(buffer) - 1)) {
        fclose(fp);
        DjiTargetChase_LogEvent("rid_config_read_failed",
                                s_last_now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER,
                                NULL);
        return false;
    }
    fclose(fp);
    buffer[length] = '\0';

    root = cJSON_Parse(buffer);
    if (root == NULL) {
        DjiTargetChase_LogEvent("rid_config_json_invalid",
                                s_last_now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER,
                                NULL);
        return false;
    }

    has_rad = DjiTargetChase_ReadJsonNumber(root, "latitude_rad", &latitude_rad) &&
              DjiTargetChase_ReadJsonNumber(root, "longitude_rad", &longitude_rad);
    has_deg = DjiTargetChase_ReadJsonNumber(root, "latitude_deg", &latitude_deg) &&
              DjiTargetChase_ReadJsonNumber(root, "longitude_deg", &longitude_deg);
    has_altitude = DjiTargetChase_ReadJsonNumber(root, "altitude_m", &altitude_m);

    if (!has_rad && has_deg) {
        latitude_rad = latitude_deg * DJI_TARGET_CHASE_DEG_TO_RAD;
        longitude_rad = longitude_deg * DJI_TARGET_CHASE_DEG_TO_RAD;
        has_rad = true;
    }

    if (!has_rad || !has_altitude ||
        !isfinite(latitude_rad) || !isfinite(longitude_rad) || !isfinite(altitude_m) ||
        latitude_rad < (-DJI_TARGET_CHASE_PI / 2.0) ||
        latitude_rad > (DJI_TARGET_CHASE_PI / 2.0) ||
        longitude_rad < -DJI_TARGET_CHASE_PI ||
        longitude_rad > DJI_TARGET_CHASE_PI ||
        altitude_m < 0.0 || altitude_m > 65535.0) {
        cJSON_Delete(root);
        DjiTargetChase_LogEvent("rid_config_invalid",
                                s_last_now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER,
                                NULL);
        return false;
    }

    rid_info->latitude = latitude_rad;
    rid_info->longitude = longitude_rad;
    rid_info->altitude = (uint16_t) altitude_m;
    cJSON_Delete(root);
    DjiTargetChase_LogEvent("rid_config_loaded",
                            s_last_now_ms,
                            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                            NULL);
    return true;
}

static void DjiTargetChase_UseLegacyTestRid(T_DjiFlightControllerRidInfo *rid_info)
{
    if (rid_info == NULL) {
        return;
    }

    rid_info->latitude = DJI_TARGET_CHASE_LEGACY_TEST_RID_LATITUDE;
    rid_info->longitude = DJI_TARGET_CHASE_LEGACY_TEST_RID_LONGITUDE;
    rid_info->altitude = DJI_TARGET_CHASE_LEGACY_TEST_RID_ALTITUDE;
    DjiTargetChase_LogEvent("rid_legacy_test_values_used",
                            s_last_now_ms,
                            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                            NULL);
}


static void DjiTargetChase_SetJoystickMode(void)
{
    T_DjiFlightControllerJoystickMode joystick_mode = {
        .horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        .verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        .yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        .horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        .stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    DjiFlightController_SetJoystickMode(joystick_mode);
}

static void DjiTargetChase_QuaternionToEulerDeg(const T_DjiFcSubscriptionQuaternion *q,
                                                float attitude_deg[3])
{
    const float sinr_cosp = 2.0f * ((q->q0 * q->q1) + (q->q2 * q->q3));
    const float cosr_cosp = 1.0f - (2.0f * ((q->q1 * q->q1) + (q->q2 * q->q2)));
    const float sinp = 2.0f * ((q->q0 * q->q2) - (q->q3 * q->q1));
    const float siny_cosp = 2.0f * ((q->q0 * q->q3) + (q->q1 * q->q2));
    const float cosy_cosp = 1.0f - (2.0f * ((q->q2 * q->q2) + (q->q3 * q->q3)));

    attitude_deg[0] = atan2f(sinr_cosp, cosr_cosp) * DJI_TARGET_CHASE_RAD_TO_DEG;

    if (fabsf(sinp) >= 1.0f) {
        attitude_deg[1] = copysignf(90.0f, sinp);
    } else {
        attitude_deg[1] = asinf(sinp) * DJI_TARGET_CHASE_RAD_TO_DEG;
    }

    attitude_deg[2] = atan2f(siny_cosp, cosy_cosp) * DJI_TARGET_CHASE_RAD_TO_DEG;
}

static T_DjiReturnCode DjiTargetChase_AuthorityEventCallback(
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo event_data)
{
    pthread_mutex_lock(&s_control_device_mutex);
    bool compatibility_monitor = s_control_device.enabled;
    pthread_mutex_unlock(&s_control_device_mutex);
    if (compatibility_monitor) return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    const bool has_authority =
        event_data.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK;

    if (has_authority) {
        /* A late grant after takeover does not reopen motion. It does mean
         * Abort must release that newly reported SDK ownership. */
        __atomic_store_n(&s_manual_takeover, false, __ATOMIC_RELEASE);
        __atomic_store_n(&s_ever_had_authority, true, __ATOMIC_RELEASE);
        __atomic_store_n(&s_real_authority_obtained, true, __ATOMIC_RELEASE);
    } else {
        if (__atomic_load_n(&s_ever_had_authority, __ATOMIC_ACQUIRE) &&
            !__atomic_load_n(&s_intentional_release, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
            __atomic_store_n(&s_manual_takeover, true, __ATOMIC_RELEASE);
            TargetChase_SetManualTrigger(&s_chase, false, s_last_now_ms);
            s_has_last_command = false;
        }
        /* An authoritative RC/other-owner callback confirms this task no
         * longer owns control. Abort must not send zero/release to that owner. */
        __atomic_store_n(&s_real_authority_obtained, false, __ATOMIC_RELEASE);
    }
    TargetChase_SetAuthority(&s_chase, has_authority, s_last_now_ms);
    DjiTargetChase_LogEvent(has_authority ? "authority_callback_osdk" : "authority_callback_other",
                            s_last_now_ms,
                            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                            NULL);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTargetChase_ObtainAuthorityUntil(
    uint64_t now_ms, const char *event, uint64_t grant_deadline_ms)
{
    if (!DjiTargetChase_ControlDeviceHealthy(false) || DjiTargetChase_IsControlInhibited() ||
        (grant_deadline_ms && !DjiTargetChase_ApiDeadlineValid(
            grant_deadline_ms, DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS)))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    pthread_mutex_lock(&s_control_device_mutex);
    bool compatibility_monitor = s_control_device.enabled;
    if (compatibility_monitor) {
        /* Do not adopt a grant belonging to an earlier/external SDK session. */
        if (s_control_device.device == 4 && s_control_device.open) {
            DjiTargetChase_ControlDeviceFault("control_device_unrequested_sdk_owner");
            pthread_mutex_unlock(&s_control_device_mutex);
            return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
        }
        s_control_device.request_issued = true;
        s_control_device.awaiting_grant = true;
        s_control_device.confirmed = false;
        s_control_device.request_generation = s_control_device.generation;
    }
    pthread_mutex_unlock(&s_control_device_mutex);
    DjiTargetChase_SetJoystickMode();
    if (DjiTargetChase_IsControlInhibited() ||
        (grant_deadline_ms && !DjiTargetChase_ApiDeadlineValid(
            grant_deadline_ms, DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS))) {
        if (compatibility_monitor) {
            pthread_mutex_lock(&s_control_device_mutex);
            s_control_device.request_issued = false;
            s_control_device.awaiting_grant = false;
            pthread_mutex_unlock(&s_control_device_mutex);
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    /* An ACK timeout does not prove that the aircraft rejected the request.
     * Preserve possible ownership even on failure, until explicit release or
     * a confirmed SDK->RC/app transition resolves it. */
    if (compatibility_monitor)
        __atomic_store_n(&s_real_authority_obtained, true, __ATOMIC_RELEASE);
    T_DjiReturnCode code = DjiFlightController_ObtainJoystickCtrlAuthority();
    DjiTargetChase_LogEvent(event, now_ms, code, NULL);
    if (!DjiTargetChase_IsSuccess(code)) {
        if (compatibility_monitor) {
            pthread_mutex_lock(&s_control_device_mutex);
            s_control_device.awaiting_grant = false;
            __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
            pthread_mutex_unlock(&s_control_device_mutex);
        }
        return code;
    }
    if (compatibility_monitor) {
        /* API success alone is only possible ownership. No command, including
         * zero, is allowed until a newer subscription sample confirms it. */
        pthread_mutex_lock(&s_control_device_mutex);
        if (!__atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE))
            __atomic_store_n(&s_real_authority_obtained, true, __ATOMIC_RELEASE);
        pthread_mutex_unlock(&s_control_device_mutex);
        uint64_t start = DjiTargetChase_MonotonicMs();
        while (DjiTargetChase_MonotonicMs() - start < DJI_TARGET_CHASE_CONTROL_DEVICE_WAIT_MS) {
            if (DjiTargetChase_IsControlInhibited() ||
                (grant_deadline_ms && !DjiTargetChase_ApiDeadlineValid(
                    grant_deadline_ms, DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS))) break;
            if (DjiTargetChase_ControlDeviceHealthy(true)) {
                TargetChase_SetAuthority(&s_chase, true, now_ms);
                DjiTargetChase_LogEvent("control_device_authority_confirmed", DjiTargetChase_NowMs(), 0, NULL);
                return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            usleep(10000);
        }
        pthread_mutex_lock(&s_control_device_mutex);
        s_control_device.awaiting_grant = false;
        __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
        pthread_mutex_unlock(&s_control_device_mutex);
        DjiTargetChase_LogEvent("control_device_authority_confirmation_failed", DjiTargetChase_NowMs(),
                                DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE, NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (__atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    __atomic_store_n(&s_ever_had_authority, true, __ATOMIC_RELEASE);
    __atomic_store_n(&s_real_authority_obtained, true, __ATOMIC_RELEASE);
    if (DjiTargetChase_IsControlInhibited()) return DjiTargetChase_Abort();
    TargetChase_SetAuthority(&s_chase, true, now_ms);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTargetChase_ObtainAuthority(uint64_t now_ms, const char *event)
{
    return DjiTargetChase_ObtainAuthorityUntil(now_ms, event, 0);
}

static bool DjiTargetChase_ApiHeightLimits(float *lower, float *upper)
{
    if (!isfinite(s_chase.config.min_height_m) || !isfinite(s_chase.config.max_height_m) ||
        !isfinite(s_chase.config.max_vertical_excursion_m))
        return false;
    *lower = fmaxf(DJI_TARGET_CHASE_API_MIN_HEIGHT_M, s_chase.config.min_height_m);
    *upper = fminf(DJI_TARGET_CHASE_API_MAX_HEIGHT_M, s_chase.config.max_height_m);
    if (s_start_height_initialized && s_chase.config.max_vertical_excursion_m > 0.0f) {
        *lower = fmaxf(*lower, s_start_height_m - s_chase.config.max_vertical_excursion_m);
        *upper = fminf(*upper, s_start_height_m + s_chase.config.max_vertical_excursion_m);
    }
    return *lower <= *upper;
}

static T_DjiReturnCode DjiTargetChase_ReadStateTopic(
    E_DjiFcSubscriptionTopic topic, void *value, uint16_t size,
    unsigned int index)
{
    T_BalloonTelemetryMeta meta = {0};
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
        topic, value, size, s_api_motion_only ? &meta.stamp : NULL);
    if (s_api_motion_only) {
        T_BalloonTelemetryProgress *progress = &s_api_state_progress[index];
        if (!DjiTargetChase_IsSuccess(code)) {
            memset(progress, 0, sizeof(*progress));
        } else {
            meta.code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            meta.valid = true;
            meta.end_ms = DjiTargetChase_NowMs();
            BalloonTelemetry_Progress(&meta, progress, s_api_state_max_age_ms[index]);
        }
    }
    return code;
}

static T_DjiReturnCode DjiTargetChase_ReadState(T_TargetChaseAircraftState *state)
{
    T_DjiReturnCode return_code;
    T_DjiFcSubscriptionHomePointSetStatus homepoint_status =
        DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_FAILED;
    T_DjiFcSubscriptionAltitudeFused altitude_fused = 0.0f;
    T_DjiFcSubscriptionAltitudeOfHomePoint homepoint_altitude = 0.0f;
    T_DjiFcSubscriptionVelocity velocity;
    T_DjiFcSubscriptionQuaternion quaternion;
    T_DjiFcSubscriptionAngularRateFusioned angular_rate;
    T_DjiFcSubscriptionFlightStatus flight_status = DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED;

    memset(state, 0, sizeof(*state));
    memset(&velocity, 0, sizeof(velocity));
    memset(&quaternion, 0, sizeof(quaternion));
    memset(&angular_rate, 0, sizeof(angular_rate));
    if (s_api_motion_only) {
        altitude_fused = homepoint_altitude = NAN;
        velocity.data.x = velocity.data.y = velocity.data.z = NAN;
        velocity.health = DJI_FC_SUBSCRIPTION_DATA_HEALTH;
        angular_rate.x = angular_rate.y = angular_rate.z = NAN;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS, &homepoint_status,
        sizeof(homepoint_status), API_HOME_STATUS);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }
    if (homepoint_status != DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_SUCCESS) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, &homepoint_altitude,
        sizeof(homepoint_altitude), API_HOME_ALTITUDE);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, &altitude_fused,
        sizeof(altitude_fused), API_ALTITUDE);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, &velocity,
        sizeof(velocity), API_VELOCITY);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, &quaternion,
        sizeof(quaternion), API_QUATERNION);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, &angular_rate,
        sizeof(angular_rate), API_ANGULAR_RATE);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = DjiTargetChase_ReadStateTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, &flight_status,
        sizeof(flight_status), API_FLIGHT_STATUS);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    state->altitude_agl_m = altitude_fused - homepoint_altitude;
    state->velocity_neu_mps[0] = velocity.data.x;
    state->velocity_neu_mps[1] = velocity.data.y;
    state->velocity_neu_mps[2] = velocity.data.z;
    /*
     * DJI's header names this value "NOT_HEALTH", but its comment marks
     * value 0 as healthy and usable.
     */
    state->velocity_healthy = velocity.health == DJI_FC_SUBSCRIPTION_DATA_NOT_HEALTH;
    state->angular_rate_radps[0] = angular_rate.x;
    state->angular_rate_radps[1] = angular_rate.y;
    state->angular_rate_radps[2] = angular_rate.z;
    state->in_air = flight_status == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR;
    state->timestamp_ms = s_api_motion_only ? DjiTargetChase_NowMs() : s_last_now_ms;
    if (s_api_motion_only) {
        float lower_height, upper_height;
        float quaternion_norm_sq = quaternion.q0 * quaternion.q0 + quaternion.q1 * quaternion.q1 +
                                   quaternion.q2 * quaternion.q2 + quaternion.q3 * quaternion.q3;
        if (!DjiTargetChase_ApiStateFresh(state->timestamp_ms) ||
            !state->in_air || !state->velocity_healthy ||
            !isfinite(state->altitude_agl_m) ||
            !DjiTargetChase_ApiHeightLimits(&lower_height, &upper_height) ||
            state->altitude_agl_m < lower_height || state->altitude_agl_m > upper_height ||
            !isfinite(quaternion_norm_sq) || quaternion_norm_sq < 0.81f || quaternion_norm_sq > 1.21f ||
            !isfinite(velocity.data.x) || !isfinite(velocity.data.y) || !isfinite(velocity.data.z) ||
            !isfinite(angular_rate.x) || !isfinite(angular_rate.y) || !isfinite(angular_rate.z))
            return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    DjiTargetChase_QuaternionToEulerDeg(&quaternion, state->attitude_deg);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTargetChase_SendZeroCommand(void)
{
    T_DjiFlightControllerJoystickCommand command = {
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .yaw = 0.0f,
    };

    return DjiTargetChase_ProfileJoystick(command);
}

T_DjiReturnCode DjiTargetChase_ReadApiState(T_TargetChaseAircraftState *state)
{
    if (!state || !s_initialized || !s_api_motion_only || DjiTargetChase_IsControlInhibited())
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    return DjiTargetChase_ReadState(state);
}

T_DjiReturnCode DjiTargetChase_ApiEnsureAuthority(uint64_t grant_deadline_ms)
{
    T_TargetChaseAircraftState state;
    if (s_no_flight_mode || !DjiTargetChase_ApiDeadlineValid(
            grant_deadline_ms, DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    T_DjiReturnCode code = DjiTargetChase_ReadApiState(&state);
    if (!DjiTargetChase_IsSuccess(code)) return code;
    if (!DjiTargetChase_ApiDeadlineValid(grant_deadline_ms, DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS) ||
        DjiTargetChase_IsControlInhibited())
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;

    bool acquiring = !__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE);
    if (acquiring) {
        code = DjiTargetChase_ObtainAuthorityUntil(
            state.timestamp_ms, "api_motion_obtain_authority", grant_deadline_ms);
        if (DjiTargetChase_IsSuccess(code)) {
            /* The SDK may block while granting authority. Do not use the earlier state. */
            code = DjiTargetChase_ReadApiState(&state);
        }
        if (!DjiTargetChase_IsSuccess(code)) {
            DjiTargetChase_RequestStop();
            return code;
        }
    }
    if (DjiTargetChase_IsControlInhibited() || !DjiTargetChase_ControlDeviceHealthy(true) ||
        !__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE) ||
        !DjiTargetChase_ApiDeadlineValid(grant_deadline_ms, DJI_TARGET_CHASE_API_MAX_GRANT_WAIT_MS)) {
        if (acquiring) DjiTargetChase_RequestStop();
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (!s_start_height_initialized) {
        s_start_height_m = state.altitude_agl_m;
        s_start_height_initialized = true;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTargetChase_ApiSend(const T_DjiFlightControllerJoystickCommand *command,
                                    uint64_t deadline_ms)
{
    if (!command || !s_initialized || !s_api_motion_only || s_no_flight_mode)
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    if (DjiTargetChase_JoystickIsZero(command)) {
        return __atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE)
            ? DjiTargetChase_SendZeroCommand() : DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (!DjiTargetChase_ApiDeadlineValid(deadline_ms, DJI_TARGET_CHASE_API_MAX_DURATION_MS) ||
        !__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE) ||
        !isfinite(command->x) || !isfinite(command->y) ||
        hypotf(command->x, command->y) > DJI_TARGET_CHASE_API_MAX_HORIZONTAL_MPS ||
        !isfinite(command->z) || fabsf(command->z) > DJI_TARGET_CHASE_API_MAX_VERTICAL_MPS ||
        !isfinite(command->yaw) || fabsf(command->yaw) > DJI_TARGET_CHASE_API_MAX_YAW_DEGPS)
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;

    T_TargetChaseAircraftState state;
    T_DjiReturnCode code = DjiTargetChase_ReadApiState(&state);
    if (!DjiTargetChase_IsSuccess(code)) return code;
    if (!DjiTargetChase_ApiDeadlineValid(deadline_ms, DJI_TARGET_CHASE_API_MAX_DURATION_MS))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;

    /* Veto outward vertical motion near either height limit. Do not
     * synthesize a replacement for a rejected API vector. */
    float lower, upper;
    if (!DjiTargetChase_ApiHeightLimits(&lower, &upper))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    float projected_height = state.altitude_agl_m + command->z *
                             (DJI_TARGET_CHASE_API_MAX_DURATION_MS / 1000.0f);
    if ((command->z < 0.0f && projected_height < lower) ||
        (command->z > 0.0f && projected_height > upper))
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;

    s_last_now_ms = state.timestamp_ms;
    TargetChase_UpdateAircraftState(&s_chase, &state);
    return DjiTargetChase_WriteJoystick(*command, deadline_ms);
}

static T_DjiReturnCode DjiTargetChase_SendCachedCommand(
    uint64_t now_ms,
    const char *event,
    const T_TargetChaseAircraftState *state)
{
    T_DjiReturnCode return_code;
    T_DjiFlightControllerJoystickCommand joystick_command;
    T_TargetChaseCommand safe_command;

    if (!s_has_last_command || !s_chase.has_authority || !s_last_command.should_send) {
        DjiTargetChase_LogEvent(event == NULL ? "repeat_last_command_unavailable" : event,
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE,
                                NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    safe_command = s_last_command;
    if (DjiTargetChase_ApplyVerticalSafety(&safe_command, state)) {
        DjiTargetChase_LogEvent("repeat_approach_safety_clamped",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                &safe_command);
        s_last_command = safe_command;
    }

    joystick_command.x = safe_command.x_mps;
    joystick_command.y = safe_command.y_mps;
    joystick_command.z = safe_command.z_mps;
    joystick_command.yaw = safe_command.yaw_rate_degps;

    if (s_no_flight_mode) {
        if (DjiTargetChase_ShouldLogRepeatCommand(now_ms,
                                                  DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)) {
            DjiTargetChase_LogEvent("dry_run_repeat_last_joystick_action",
                                    now_ms,
                                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    &safe_command);
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return_code = DjiTargetChase_ProfileJoystick(joystick_command);
    if (DjiTargetChase_ShouldLogRepeatCommand(now_ms, return_code)) {
        DjiTargetChase_LogEvent(event == NULL ? "repeat_last_joystick_action" : event,
                                now_ms,
                                return_code,
                                &safe_command);
    }
    return return_code;
}

static T_DjiReturnCode DjiTargetChase_SubscribeTopics(void)
{
    T_DjiReturnCode return_code;

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS,
        DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
        DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return_code = BalloonSubscription_Acquire(BALLOON_SUB_CHASE,
        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
        NULL);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTargetChase_InitMode(const T_TargetChaseConfig *config, bool api_motion_only)
{
    T_DjiReturnCode return_code;
    T_DjiFlightControllerRidInfo ridInfo = {0};

    /* A managed maintenance panel must not even initialize flight/gimbal
     * services. Unmanaged NO_FLIGHT deployments retain their legacy dry-run
     * behavior, including the existing SDK-substitute test fixtures. */
    if (__atomic_load_n(&s_panel_maintenance_mode, __ATOMIC_ACQUIRE) == 1)
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    if (s_initialized) {
        return s_api_motion_only != api_motion_only || DjiTargetChase_IsControlInhibited()
            ? DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE
            : DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (DjiTargetChase_IsReleasePending())
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    s_api_motion_only = api_motion_only;
    memset(s_api_state_progress, 0, sizeof(s_api_state_progress));
    __atomic_store_n(&s_control_inhibited, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_manual_takeover, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_ever_had_authority, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_intentional_release, false, __ATOMIC_RELEASE);
    __atomic_store_n(&s_last_stop_result, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, __ATOMIC_RELEASE);

    DjiTargetChase_ClearPredictionStatus();
    s_last_now_ms = DjiTargetChase_NowMs();
    s_no_flight_mode = DjiTargetChase_RequestedNoFlight();
    __atomic_store_n(&s_real_authority_obtained, false, __ATOMIC_RELEASE);
    memset(&s_last_command, 0, sizeof(s_last_command));
    s_has_last_command = false;
    s_last_repeat_command_log_ms = 0;
    s_last_slew_mono_ms = 0;
    s_start_height_initialized = false;
    s_start_height_m = 0.0f;
    s_camera_pose_next = 0;
    s_camera_pose_count = 0;
    TargetChase_Init(&s_chase, config);
    s_kf_pose_count = s_kf_pose_next = 0;
    s_kf_last_pose_receipt = s_raw_frame_ms = 0;
    memset(&s_kf_last_sdk_stamp, 0, sizeof(s_kf_last_sdk_stamp));
    if (!s_api_motion_only) (void)DjiTargetChase_LoadKfTiming(s_last_now_ms);
    DjiTargetChase_ProfileInit();
    DjiTargetChase_LogEvent(s_no_flight_mode ? "no_flight_mode_enabled" : "no_flight_mode_disabled",
                            s_last_now_ms,
                            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                            NULL);

    if (!DjiTargetChase_ReadRidConfig(&ridInfo)) {
        DjiTargetChase_UseLegacyTestRid(&ridInfo);
    }

    return_code = DjiFlightController_Init(ridInfo);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        DjiTargetChase_LogEvent("flight_controller_init_failed", s_last_now_ms, return_code, NULL);
        return return_code;
    }
    DjiTargetChase_LogEvent("flight_controller_initialized", s_last_now_ms, return_code, NULL);

    return_code = BalloonSubscription_Init();
    if (!DjiTargetChase_IsSuccess(return_code)) {
        DjiTargetChase_LogEvent("fc_subscription_init_failed", s_last_now_ms, return_code, NULL);
        return return_code;
    }

    return_code = DjiTargetChase_SubscribeTopics();
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    {
        T_TargetChaseAircraftState initial_state;

        return_code = DjiTargetChase_ReadState(&initial_state);
        if (DjiTargetChase_IsSuccess(return_code)) {
            TargetChase_UpdateAircraftState(&s_chase, &initial_state);
            if (initial_state.altitude_agl_m > 0.0f) {
                s_start_height_m = initial_state.altitude_agl_m;
                s_start_height_initialized = true;
                DjiTargetChase_LogEvent("vertical_velocity_start_height_set",
                                        s_last_now_ms,
                                        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                        NULL);
            } else {
                DjiTargetChase_LogEvent("vertical_velocity_start_height_unset",
                                        s_last_now_ms,
                                        return_code,
                                        NULL);
            }
        } else {
            DjiTargetChase_LogEvent("vertical_velocity_start_height_unset",
                                    s_last_now_ms,
                                    return_code,
                                    NULL);
        }
    }

    DjiTargetChase_SetJoystickMode();

    return_code = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        DjiTargetChase_AuthorityEventCallback);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        /* An unsupported native callback may use the documented M300/M350
         * control-device topic, but only after live monitoring is proved. */
        if (!s_no_flight_mode && DjiTargetChase_IsOptionalAuthorityCallbackFailure(return_code))
            return_code = DjiTargetChase_ControlDeviceStart();
        if (!s_no_flight_mode || !DjiTargetChase_IsOptionalAuthorityCallbackFailure(return_code)) {
            if (!DjiTargetChase_IsSuccess(return_code)) {
                __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
                DjiTargetChase_LogEvent("authority_callback_required_for_flight", s_last_now_ms, return_code, NULL);
                /* FC/subscriptions were initialized; unwind the rejected
                 * session, including any compatibility subscription. */
                s_initialized = true;
                (void) DjiTargetChase_DeInit();
                return return_code;
            }
        } else {
            DjiTargetChase_LogEvent("authority_callback_not_supported_no_flight", s_last_now_ms, return_code, NULL);
        }
    }

    if (DjiTargetChase_IsControlInhibited()) goto init_cancelled;
    if (!s_api_motion_only) DjiTargetChase_InitGimbalTracking(s_last_now_ms);
    if (DjiTargetChase_IsControlInhibited()) goto init_cancelled;
    s_initialized = true;
    s_last_repeat_command_log_ms = 0;
    DjiTargetChase_LogEvent("target_chase_initialized", s_last_now_ms, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, NULL);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
init_cancelled:
    s_initialized = true;
    (void)DjiTargetChase_DeInit();
    return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
}

T_DjiReturnCode DjiTargetChase_Init(const T_TargetChaseConfig *config)
{
    return DjiTargetChase_InitMode(config, false);
}

T_DjiReturnCode DjiTargetChase_InitApiMotion(const T_TargetChaseConfig *config)
{
    return DjiTargetChase_InitMode(config, true);
}

T_DjiReturnCode DjiTargetChase_DeInit(void)
{
    T_DjiReturnCode return_code;

    if (!s_initialized) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    T_DjiReturnCode abort_code = DjiTargetChase_Abort();
    if (!s_api_motion_only) DjiTargetChase_DeInitGimbalTracking(s_last_now_ms);
    if (DjiTargetChase_IsReleasePending()) return abort_code;
    DjiTargetChase_ControlDeviceStop();
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS);
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT);
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
    (void) BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
    return_code = DjiFlightController_DeInit();
    DjiTargetChase_LogEvent("flight_controller_deinit", s_last_now_ms, return_code, NULL);
    DjiTargetChase_ProfileFlush(true);
    s_profile_enabled = false;
    s_initialized = false;
    s_api_motion_only = false;
    memset(&s_last_command, 0, sizeof(s_last_command));
    s_has_last_command = false;
    s_last_slew_mono_ms = 0;
    s_start_height_initialized = false;
    s_start_height_m = 0.0f;
    if (!DjiTargetChase_IsSuccess(abort_code)) return_code = abort_code;
    __atomic_store_n(&s_last_stop_result, return_code, __ATOMIC_RELEASE);
    return return_code;
}

T_DjiReturnCode DjiTargetChase_OnTargetPixel(float u_px, float v_px, uint64_t now_ms)
{
    float world_yaw_deg;
    float world_pitch_deg;
    if (!s_initialized || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (now_ms == 0 || now_ms > DjiTargetChase_NowMs() || !DjiTargetChase_TargetAfterKfEpoch(now_ms) ||
        (s_chase.target.valid && now_ms <= s_chase.target.timestamp_ms)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }


    (void) DjiTargetChase_UpdateGimbalBoresight(DjiTargetChase_NowMs(), NULL);
    if (DjiTargetChase_CameraPoseAt(now_ms, &world_yaw_deg, &world_pitch_deg)) {
        TargetChase_SetObservationCameraPose(&s_chase, world_yaw_deg,
                                              world_pitch_deg, now_ms);
    }
    uint64_t profile_begin_ns = DjiTargetChase_ProfileBegin();
    TargetChase_UpdateTargetPixel(&s_chase, u_px, v_px, now_ms);
    DjiTargetChase_ProfileEnd(PROFILE_UPDATE_PIXEL, profile_begin_ns,
                             s_chase.target.valid && s_chase.target.timestamp_ms == now_ms);
    if (s_chase.target.valid) {
        profile_begin_ns = DjiTargetChase_ProfileBegin();
        DjiTargetChase_ObserveKf(u_px, v_px, now_ms);
        if (s_chase.config.prediction_mode == 3)
            DjiTargetChase_ProfileEnd(PROFILE_KF_UPDATE, profile_begin_ns,
                                      s_chase.delayed_kf.status == TARGET_DELAYED_KF_ACCEPTED);
        s_search_exhausted = false;
        if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_SEARCHING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_BODY_TURNING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY) {
            (void) DjiTargetChase_TrackGimbalToPixel(
                s_chase.target.u_px, s_chase.target.v_px, now_ms);
        } else {
            s_last_gimbal_target_seen_ms = DjiTargetChase_NowMs();
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTargetChase_OnTargetBox(float image_width_px,
                                           float image_height_px,
                                           float left_bottom_u_px,
                                           float left_bottom_v_px,
                                           float right_top_u_px,
                                           float right_top_v_px,
                                           uint64_t now_ms)
{
    float world_yaw_deg;
    float world_pitch_deg;
    const float raw_u = s_raw_u, raw_v = s_raw_v;
    const bool has_raw = s_raw_frame_ms == now_ms && now_ms != 0;
    s_raw_frame_ms = 0;
    if (!s_initialized || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (now_ms == 0 || now_ms > DjiTargetChase_NowMs() || !DjiTargetChase_TargetAfterKfEpoch(now_ms) ||
        (s_chase.target.valid && now_ms <= s_chase.target.timestamp_ms)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }


    if (!TargetChase_UpdateImageGeometry(&s_chase, image_width_px, image_height_px)) {
        DjiTargetChase_LogEvent("image_geometry_update_failed",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER,
                                NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    (void) DjiTargetChase_UpdateGimbalBoresight(DjiTargetChase_NowMs(), NULL);
    if (DjiTargetChase_CameraPoseAt(now_ms, &world_yaw_deg, &world_pitch_deg)) {
        TargetChase_SetObservationCameraPose(&s_chase, world_yaw_deg,
                                              world_pitch_deg, now_ms);
    }
    uint64_t profile_begin_ns = DjiTargetChase_ProfileBegin();
    TargetChase_UpdateTargetBox(&s_chase,
                                left_bottom_u_px,
                                left_bottom_v_px,
                                right_top_u_px,
                                right_top_v_px,
                                now_ms);
    DjiTargetChase_ProfileEnd(PROFILE_UPDATE_BOX, profile_begin_ns,
                             s_chase.target.valid && s_chase.target.timestamp_ms == now_ms);
    if (s_chase.target.valid) {
        profile_begin_ns = DjiTargetChase_ProfileBegin();
        if (has_raw) DjiTargetChase_ObserveKf(raw_u, raw_v, now_ms);
        else if (s_chase.config.prediction_mode == 3)
            TargetChase_ResetDelayedPrediction(&s_chase, TARGET_KF_NO_POSE);
        if (s_chase.config.prediction_mode == 3)
            DjiTargetChase_ProfileEnd(PROFILE_KF_UPDATE, profile_begin_ns,
                                      s_chase.delayed_kf.status == TARGET_DELAYED_KF_ACCEPTED);
        s_search_exhausted = false;
        if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_SEARCHING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_BODY_TURNING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING ||
            s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY) {
            (void) DjiTargetChase_TrackGimbalToPixel(
                s_chase.target.u_px, s_chase.target.v_px, now_ms);
        } else {
            s_last_gimbal_target_seen_ms = DjiTargetChase_NowMs();
        }
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTargetChase_SetManualTrigger(bool enabled, uint64_t now_ms)
{
    if (!s_initialized || s_api_motion_only || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    s_last_now_ms = now_ms;
    TargetChase_SetManualTrigger(&s_chase, enabled, now_ms);
    if (!enabled) DjiTargetChase_ClearPredictionStatus();
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool DjiTargetChase_IsManualTriggerEnabled(void)
{
    return !DjiTargetChase_IsControlInhibited() && TargetChase_IsManualTriggerEnabled(&s_chase);
}

T_DjiReturnCode DjiTargetChase_RunOnce(uint64_t now_ms)
{
    T_DjiReturnCode return_code;
    T_TargetChaseAircraftState state;
    T_TargetChaseCommand command;

    if (!s_initialized || s_api_motion_only || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    DjiTargetChase_ProfileFlush(false);
    s_last_now_ms = now_ms;

    return_code = DjiTargetChase_ReadState(&state);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        DjiTargetChase_LogEvent("read_state_failed", now_ms, return_code, NULL);
        if (!s_no_flight_mode) {
            return return_code;
        }
        memset(&state, 0, sizeof(state));
        state.altitude_agl_m = 10.0f;
        state.in_air = true;
        state.timestamp_ms = now_ms;
    }

    TargetChase_UpdateAircraftState(&s_chase, &state);
    (void) DjiTargetChase_UpdateGimbalBoresight(now_ms, NULL);
    /* The caller's loop-start time predates both the observation and SDK reads. */
    if (s_chase.config.prediction_mode == 3) now_ms = DjiTargetChase_NowMs();

    if (TargetChase_ShouldReleaseAuthority(&s_chase, now_ms)) {
        DjiTargetChase_LogEvent("auto_release_suppressed_hover",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                NULL);
        return DjiTargetChase_Hover(now_ms);
    }

    if (TargetChase_ShouldObtainAuthority(&s_chase, now_ms)) {
        if (s_no_flight_mode) {
            TargetChase_SetAuthority(&s_chase, true, now_ms);
            __atomic_store_n(&s_real_authority_obtained, false, __ATOMIC_RELEASE);
            DjiTargetChase_LogEvent("obtain_authority_skipped_no_flight",
                                    now_ms,
                                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    NULL);
        } else {
            return_code = DjiTargetChase_ObtainAuthority(now_ms, "obtain_authority");
            if (!DjiTargetChase_IsSuccess(return_code)) return return_code;
        }
    }

    if (DjiTargetChase_ProfileStep(now_ms, &command) && command.should_send) {
        /*
         * The image loop and body loop run together.  TargetChase_Step()
         * already accounts for measured gimbal boresight, image error and
         * recenter hysteresis.  Freezing every body axis while the gimbal was
         * TRACKING prevented both forward closure and the planned body yaw,
         * leaving an edge target permanently stuck off-axis.
         */
        if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING) {
            DjiTargetChase_LogEvent("gimbal_tracking_body_motion_enabled",
                                    now_ms,
                                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    &command);
        }
        DjiTargetChase_ApplyCommandSlew(&command);
        if (DjiTargetChase_ApplyVerticalSafety(&command, &state)) {
            DjiTargetChase_LogEvent("approach_safety_clamped",
                                    now_ms,
                                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    &command);
        }
        T_DjiFlightControllerJoystickCommand joystick_command = {
            .x = command.x_mps,
            .y = command.y_mps,
            .z = command.z_mps,
            .yaw = command.yaw_rate_degps,
        };

        if (s_no_flight_mode) {
            return_code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            DjiTargetChase_LogEvent("dry_run_joystick_action", now_ms, return_code, &command);
        } else {
            return_code = DjiTargetChase_ProfileJoystick(joystick_command);
            DjiTargetChase_LogEvent("execute_joystick_action", now_ms, return_code, &command);
            if (!DjiTargetChase_IsSuccess(return_code)) {
                return return_code;
            }
        }

        s_last_command = command;
        s_has_last_command = true;
        s_last_repeat_command_log_ms = 0;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode DjiTargetChase_EnsureSearchAuthority(uint64_t now_ms)
{
    if (s_api_motion_only || DjiTargetChase_IsControlInhibited())
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;

    T_DjiReturnCode return_code;

    if (s_chase.has_authority) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    if (s_no_flight_mode) {
        TargetChase_SetAuthority(&s_chase, true, now_ms);
        __atomic_store_n(&s_real_authority_obtained, false, __ATOMIC_RELEASE);
        DjiTargetChase_LogEvent("body_search_authority_skipped_no_flight",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    return_code = DjiTargetChase_ObtainAuthority(now_ms, "body_search_obtain_authority");
    return return_code;
}

static T_DjiReturnCode DjiTargetChase_SendBodySearchCommand(float forward_mps,
                                                            float right_mps,
                                                            float yaw_rate_degps,
                                                            uint64_t now_ms,
                                                            const char *event,
                                                            bool force_log)
{
    T_DjiReturnCode return_code;
    T_TargetChaseCommand command;
    T_DjiFlightControllerJoystickCommand joystick_command;
    uint64_t mono_ms = DjiTargetChase_MonotonicMs();
    bool should_log = force_log ||
                      s_last_body_search_log_mono_ms == 0 ||
                      mono_ms < s_last_body_search_log_mono_ms ||
                      mono_ms - s_last_body_search_log_mono_ms >= 1000ULL;

    memset(&command, 0, sizeof(command));
    command.x_mps = forward_mps;
    command.y_mps = right_mps;
    command.yaw_rate_degps = yaw_rate_degps;
    command.should_send = true;
    DjiTargetChase_ApplyCommandSlew(&command);
    joystick_command.x = command.x_mps;
    joystick_command.y = command.y_mps;
    joystick_command.z = command.z_mps;
    joystick_command.yaw = command.yaw_rate_degps;

    if (s_no_flight_mode) {
        return_code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    } else if (!__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE)) {
        return_code = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    } else {
        return_code = DjiTargetChase_ProfileJoystick(joystick_command);
    }

    if (should_log || !DjiTargetChase_IsSuccess(return_code)) {
        DjiTargetChase_LogEvent(event, now_ms, return_code, &command);
        s_last_body_search_log_mono_ms = mono_ms;
    }
    if (DjiTargetChase_IsSuccess(return_code)) {
        s_last_command = command;
        s_has_last_command = true;
    }
    return return_code;
}

T_DjiReturnCode DjiTargetChase_SearchTick(uint64_t now_ms)
{
    T_DjiReturnCode return_code;
    T_TargetChaseAircraftState state;
    bool state_valid;
    uint64_t mono_ms;
    uint64_t turn_elapsed_ms;
    float turned_deg;

    if (!s_initialized || DjiTargetChase_IsControlInhibited() ||
        !s_body_search_enabled ||
        !s_gimbal_search_enabled ||
        !s_chase.manual_trigger_enabled) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    DjiTargetChase_ProfileFlush(false);
    s_last_now_ms = now_ms;

    /*
     * A close balloon can be blown through the narrow camera FOV between two
     * frames. During the gimbal's outward edge recovery, move a short distance
     * toward the last horizontal exit and yaw the body the same way. The
     * bounded 0.9 s / 0.15 m/s command covers only about 0.14 m before the
     * existing reverse sweep and full-sector search take over.
     */
    if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING &&
        s_chase.has_authority) {
        uint64_t edge_now_ms = DjiTargetChase_MonotonicMs();
        uint64_t edge_elapsed_ms =
            s_gimbal_edge_recovery_started_mono_ms != 0 &&
                    edge_now_ms >= s_gimbal_edge_recovery_started_mono_ms
                ? edge_now_ms - s_gimbal_edge_recovery_started_mono_ms
                : DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_DURATION_MS;
        float edge_direction =
            s_gimbal_edge_recovery_yaw_direction > 0.0f
                ? 1.0f
                : (s_gimbal_edge_recovery_yaw_direction < 0.0f ? -1.0f
                                                               : 0.0f);

        if ((s_no_flight_mode || s_chase.aircraft.in_air) &&
            edge_direction != 0.0f &&
            edge_elapsed_ms <
                DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_DURATION_MS) {
            return DjiTargetChase_SendBodySearchCommand(
                0.0f,
                edge_direction *
                    DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_LATERAL_MPS,
                edge_direction *
                    DJI_TARGET_CHASE_BODY_EDGE_RECOVERY_YAW_DEGPS,
                now_ms,
                s_no_flight_mode
                    ? "dry_run_near_edge_body_recovery_move"
                    : "near_edge_body_recovery_move",
                false);
        }
        return DjiTargetChase_SendBodySearchCommand(
            0.0f,
            0.0f,
            0.0f,
            now_ms,
            s_no_flight_mode
                ? "dry_run_near_edge_body_recovery_hold"
                : "near_edge_body_recovery_hold",
            false);
    }

    /*
     * Once search has obtained authority, keep an explicit zero-translation
     * command alive during each gimbal raster. This prevents the previous yaw
     * command from surviving while the camera scans the new sector.
     */
    if (!s_body_search_turn_active ||
        s_gimbal_state != DJI_TARGET_CHASE_GIMBAL_BODY_TURNING) {
        if (s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_SEARCHING &&
            s_chase.has_authority) {
            return DjiTargetChase_SendBodySearchCommand(
                0.0f,
                0.0f,
                0.0f,
                now_ms,
                s_no_flight_mode
                    ? "dry_run_body_search_sector_hold"
                    : "body_search_sector_hold",
                false);
        }
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    mono_ms = DjiTargetChase_MonotonicMs();
    turn_elapsed_ms =
        s_body_search_turn_start_mono_ms != 0 &&
                mono_ms >= s_body_search_turn_start_mono_ms
            ? mono_ms - s_body_search_turn_start_mono_ms
            : 0;

    memset(&state, 0, sizeof(state));
    return_code = DjiTargetChase_ReadState(&state);
    state_valid = DjiTargetChase_IsSuccess(return_code);
    if (!state_valid) {
        if (!s_no_flight_mode) {
            if (s_chase.has_authority) {
                (void) DjiTargetChase_SendBodySearchCommand(
                    0.0f,
                    0.0f,
                    0.0f,
                    now_ms,
                    "body_search_state_failed_zero",
                    true);
            }
            s_body_search_turn_active = false;
            s_body_search_turn_heading_valid = false;
            DjiTargetChase_BeginGimbalSectorSearch(
                now_ms,
                "body_search_state_failed_resume_gimbal_only",
                NULL);
            return return_code;
        }
        state.in_air = true;
        /*
         * Dry-run progress is simulated below, but it must never overwrite
         * the measured aircraft attitude used to convert ground-frame gimbal
         * angles into body-relative angles. Doing so shifts the next scan
         * centre by the virtual 120-degree body turn even though the aircraft
         * has not moved.
         */
    } else {
        TargetChase_UpdateAircraftState(&s_chase, &state);
    }

    if (!s_no_flight_mode && !state.in_air) {
        if (s_chase.has_authority) {
            (void) DjiTargetChase_SendBodySearchCommand(
                0.0f,
                0.0f,
                0.0f,
                now_ms,
                "body_search_ground_guard_zero",
                true);
        }
        s_body_search_turn_active = false;
        s_body_search_turn_heading_valid = false;
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms,
            "body_search_ground_guard_resume_gimbal_only",
            NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    if (!s_body_search_turn_heading_valid && state_valid) {
        s_body_search_turn_start_yaw_deg = state.attitude_deg[2];
        s_body_search_turn_heading_valid = true;
    }

    if (s_no_flight_mode) {
        turned_deg = s_body_search_yaw_rate_degps *
                     (float) turn_elapsed_ms / 1000.0f;
    } else {
        turned_deg = fabsf(DjiTargetChase_Wrap180(
            state.attitude_deg[2] - s_body_search_turn_start_yaw_deg));
    }

    if (turned_deg >=
        s_search_coverage.body_yaw_step_deg -
            DJI_TARGET_CHASE_BODY_SEARCH_YAW_TOLERANCE_DEG) {
        return_code = DjiTargetChase_EnsureSearchAuthority(now_ms);
        if (DjiTargetChase_IsSuccess(return_code)) {
            return_code = DjiTargetChase_SendBodySearchCommand(
                0.0f,
                0.0f,
                0.0f,
                now_ms,
                s_no_flight_mode
                    ? "dry_run_body_search_turn_complete"
                    : "body_search_turn_complete",
                true);
        }
        s_body_search_turn_active = false;
        s_body_search_turn_heading_valid = false;
        s_body_search_turn_start_mono_ms = 0;
        if (s_search_coverage.sector_count > 0U) {
            s_search_sector_index =
                (s_search_sector_index + 1U) %
                s_search_coverage.sector_count;
        }
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms,
            "body_search_turn_complete_enter_next_sector",
            NULL);
        return return_code;
    }

    if (turn_elapsed_ms >= DJI_TARGET_CHASE_BODY_SEARCH_TURN_TIMEOUT_MS) {
        if (s_chase.has_authority) {
            (void) DjiTargetChase_SendBodySearchCommand(
                0.0f,
                0.0f,
                0.0f,
                now_ms,
                "body_search_turn_timeout_zero",
                true);
        }
        s_body_search_turn_active = false;
        s_body_search_turn_heading_valid = false;
        s_body_search_turn_start_mono_ms = 0;
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms,
            "body_search_turn_timeout_resume_sector_scan",
            NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    return_code = DjiTargetChase_EnsureSearchAuthority(now_ms);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }
    return DjiTargetChase_SendBodySearchCommand(
        0.0f,
        0.0f,
        s_body_search_yaw_rate_degps,
        now_ms,
        s_no_flight_mode
            ? "dry_run_body_search_turn"
            : "body_search_turn",
        false);
}

T_DjiReturnCode DjiTargetChase_RequestSearch(uint64_t now_ms,
                                             bool fast_pass)
{
    if (!s_initialized || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    s_last_now_ms = now_ms;
    DjiTargetChase_ClearPredictionStatus();
    TargetChase_ClearTarget(&s_chase);
    s_chase.kf_epoch_ms = DjiTargetChase_NowMs();
    s_raw_frame_ms = 0;
    s_last_gimbal_target_seen_ms = 0;
    s_last_gimbal_target_near_edge = false;
    s_gimbal_edge_recovery_started_mono_ms = 0;
    s_gimbal_edge_recovery_yaw_direction = 0.0f;
    s_gimbal_edge_recovery_pitch_direction = 0.0f;
    s_gimbal_centered_observations = 0;
    s_search_sector_index = 0;
    s_search_exhausted = false;

    if (DjiTargetChase_VisibleTargetOnly()) {
        s_body_search_turn_active = false;
        s_body_search_turn_heading_valid = false;
        s_body_search_turn_start_mono_ms = 0;
        if (s_gimbal_tracking_enabled) {
            DjiTargetChase_StopGimbal(now_ms, "stage_visible_target_wait_stop", true);
            s_gimbal_state = DJI_TARGET_CHASE_GIMBAL_READY;
        }
        DjiTargetChase_LogEvent("stage_visible_target_wait", now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    if (!s_gimbal_tracking_enabled || !s_gimbal_search_enabled) {
        DjiTargetChase_LogEvent("stage_search_requested_without_gimbal_search",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    if (fast_pass) {
        DjiTargetChase_BeginFastStageSearchTurn(now_ms);
    } else {
        DjiTargetChase_StopGimbal(
            now_ms, "stage_search_cancel_previous_gimbal_motion", true);
        DjiTargetChase_BeginGimbalSectorSearch(
            now_ms, "stage_search_begin_gimbal_sector", NULL);
    }
    DjiTargetChase_LogEvent(
                            fast_pass ? "stage_fast_search_requested"
                                      : "stage_search_requested",
                            now_ms,
                            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                            NULL);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool DjiTargetChase_IsNoFlightMode(void)
{
    return s_no_flight_mode;
}

E_DjiTargetChaseGimbalState DjiTargetChase_GetGimbalState(void)
{
    return s_gimbal_state;
}

const char *DjiTargetChase_GetGimbalStateText(void)
{
    switch (s_gimbal_state) {
        case DJI_TARGET_CHASE_GIMBAL_RESETTING:
            return "resetting";
        case DJI_TARGET_CHASE_GIMBAL_READY:
            return "target_centered_gimbal_hold";
        case DJI_TARGET_CHASE_GIMBAL_SEARCHING:
            return "searching";
        case DJI_TARGET_CHASE_GIMBAL_BODY_TURNING:
            return "body_turning_between_search_sectors";
        case DJI_TARGET_CHASE_GIMBAL_TRACKING:
            return "centering_target";
        case DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING:
            return "near_edge_recovering";
        case DJI_TARGET_CHASE_GIMBAL_STATIC_TEST:
            return "static_test";
        case DJI_TARGET_CHASE_GIMBAL_ERROR:
            return "error";
        case DJI_TARGET_CHASE_GIMBAL_DISABLED:
        default:
            return "disabled";
    }
}

bool DjiTargetChase_IsSearchExhausted(void)
{
    return s_search_exhausted;
}

bool DjiTargetChase_HasRepeatableCommand(void)
{
    return s_initialized && !DjiTargetChase_IsControlInhibited() &&
           s_has_last_command &&
           s_chase.has_authority &&
           s_last_command.should_send;
}

bool DjiTargetChase_GetLastCommand(T_TargetChaseCommand *command)
{
    if (command == NULL || !s_initialized || !s_has_last_command) {
        return false;
    }

    *command = s_last_command;
    return true;
}

T_DjiReturnCode DjiTargetChase_Hover(uint64_t now_ms)
{
    T_DjiReturnCode return_code;
    T_TargetChaseCommand command;

    if (!s_initialized || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    DjiTargetChase_ClearPredictionStatus();
    s_last_now_ms = now_ms;
    memset(&command, 0, sizeof(command));
    command.should_send = true;
    if (!s_chase.has_authority) {
        s_has_last_command = false;
        DjiTargetChase_LogEvent("hover_no_authority",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE,
                                NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    /*
     * A lost detection is common during live inference. Decelerate every axis
     * toward zero using the same limiter as normal commands instead of
     * replacing a nonzero command with an abrupt hover command.
     */
    DjiTargetChase_ApplyCommandSlew(&command);

    if (s_no_flight_mode) {
        s_last_command = command;
        s_has_last_command = true;
        s_chase.mode = TARGET_CHASE_MODE_HOLD;
        DjiTargetChase_LogEvent("dry_run_hover_joystick_action",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                &s_last_command);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    if (!__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE)) {
        s_has_last_command = false;
        DjiTargetChase_LogEvent("hover_skipped_no_real_authority",
                                now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE,
                                NULL);
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    {
        T_DjiFlightControllerJoystickCommand joystick_command = {
            .x = command.x_mps,
            .y = command.y_mps,
            .z = command.z_mps,
            .yaw = command.yaw_rate_degps,
        };
        return_code = DjiTargetChase_ProfileJoystick(joystick_command);
    }
    if (DjiTargetChase_IsSuccess(return_code)) {
        s_last_command = command;
        s_has_last_command = true;
        s_last_repeat_command_log_ms = 0;
        s_chase.mode = TARGET_CHASE_MODE_HOLD;
    } else {
        s_has_last_command = false;
    }

    DjiTargetChase_LogEvent("hover_joystick_action", now_ms, return_code, &command);
    return return_code;
}

T_DjiReturnCode DjiTargetChase_RepeatLastCommand(uint64_t now_ms)
{
    T_DjiReturnCode return_code;
    T_TargetChaseAircraftState state;
    T_TargetChaseCommand predicted_command;
    const T_TargetChaseAircraftState *state_for_safety = NULL;

    if (!s_initialized || DjiTargetChase_IsControlInhibited()) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    DjiTargetChase_ProfileFlush(false);
    s_last_now_ms = now_ms;
    return_code = DjiTargetChase_ReadState(&state);
    if (DjiTargetChase_IsSuccess(return_code)) {
        TargetChase_UpdateAircraftState(&s_chase, &state);
        state_for_safety = &state;
    } else {
        DjiTargetChase_LogEvent("repeat_read_state_failed", now_ms, return_code, NULL);
    }
    if (s_chase.config.prediction_mode == 3) {
        (void)DjiTargetChase_UpdateGimbalBoresight(DjiTargetChase_NowMs(), NULL);
        now_ms = DjiTargetChase_NowMs();
    }

    /*
     * During a short detector gap, refresh the command from the alpha-beta
     * motion estimate instead of blindly replaying a stale heading. The normal
     * target timeout still ends this coast window and commands hover.
     */
    if (DjiTargetChase_ProfileStep(now_ms, &predicted_command) &&
        predicted_command.should_send) {
        DjiTargetChase_ApplyCommandSlew(&predicted_command);
        if (DjiTargetChase_ApplyVerticalSafety(&predicted_command,
                                               state_for_safety)) {
            DjiTargetChase_LogEvent("prediction_coast_approach_safety_clamped",
                                    now_ms,
                                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    &predicted_command);
        }
        s_last_command = predicted_command;
        s_has_last_command = true;
        if (DjiTargetChase_ShouldLogRepeatCommand(
                now_ms, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)) {
            DjiTargetChase_LogEvent("prediction_coast_command_refresh",
                                    now_ms,
                                    DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    &predicted_command);
        }
    }

    return_code = DjiTargetChase_SendCachedCommand(now_ms,
                                                   "repeat_last_joystick_action",
                                                   state_for_safety);
    if (!DjiTargetChase_IsSuccess(return_code)) {
        return return_code;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTargetChase_Abort(void)
{
    T_DjiReturnCode zero_code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_DjiReturnCode release_code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    bool zero_sent = false, zero_skipped = false, release_attempted = false;
    __atomic_store_n(&s_control_inhibited, true, __ATOMIC_RELEASE);
    DjiTargetChase_ClearPredictionStatus();
    TargetChase_ClearTarget(&s_chase);
    TargetChase_SetManualTrigger(&s_chase, false, s_last_now_ms);
    s_chase.kf_epoch_ms = DjiTargetChase_NowMs();
    s_raw_frame_ms = 0;
    s_has_last_command = false;
    s_last_repeat_command_log_ms = 0;

    if (__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE) &&
        !__atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE)) {
        if (DjiTargetChase_ControlDeviceHealthy(true)) {
            zero_code = DjiTargetChase_SendZeroCommand();
            zero_sent = true;
        } else {
            zero_skipped = true;
        }
        /* A callback may have transferred control during the zero command. */
        if (__atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE) &&
            !__atomic_load_n(&s_manual_takeover, __ATOMIC_ACQUIRE)) {
            __atomic_store_n(&s_intentional_release, true, __ATOMIC_RELEASE);
            release_code = DjiFlightController_ReleaseJoystickCtrlAuthority();
            __atomic_store_n(&s_intentional_release, false, __ATOMIC_RELEASE);
            release_attempted = true;
            if (DjiTargetChase_IsSuccess(release_code)) {
                pthread_mutex_lock(&s_control_device_mutex);
                s_control_device.request_issued = false;
                s_control_device.awaiting_grant = false;
                s_control_device.confirmed = false;
                __atomic_store_n(&s_real_authority_obtained, false, __ATOMIC_RELEASE);
                pthread_mutex_unlock(&s_control_device_mutex);
            }
        }
    }
    /* Persist diagnostics after the flight-control safety actions. */
    DjiTargetChase_LogEvent("abort_requested", s_last_now_ms, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, NULL);
    if (zero_sent) {
        const T_TargetChaseCommand zero_command = {.should_send = true};
        DjiTargetChase_LogEvent("abort_zero_joystick_action", s_last_now_ms, zero_code, &zero_command);
    } else if (zero_skipped) {
        DjiTargetChase_LogEvent("abort_zero_skipped_unconfirmed_owner", s_last_now_ms,
                                DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE, NULL);
    }
    if (release_attempted)
        DjiTargetChase_LogEvent("abort_release_authority", s_last_now_ms, release_code, NULL);
    /* Preserve real ownership after a failed release, so a later explicit
     * stop can retry and a new mission cannot hide the unresolved authority. */
    TargetChase_SetAuthority(&s_chase,
        __atomic_load_n(&s_real_authority_obtained, __ATOMIC_ACQUIRE), s_last_now_ms);
    T_DjiReturnCode result = !DjiTargetChase_IsSuccess(release_code) ? release_code : zero_code;
    __atomic_store_n(&s_last_stop_result, result, __ATOMIC_RELEASE);
    return result;
}

E_TargetChaseMode DjiTargetChase_GetMode(void)
{
    return TargetChase_GetMode(&s_chase);
}
