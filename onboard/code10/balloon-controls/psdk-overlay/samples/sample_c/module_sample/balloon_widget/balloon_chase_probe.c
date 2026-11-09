#include "balloon_widget/balloon_chase_probe.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "dji_error.h"
#include "balloon_background_telemetry.h"
#include "dji_flight_controller.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "target_chase/dji_target_chase_task.h"

#define BALLOON_CHASE_PROBE_LOG "/home/uav/offline_balloon_strike_test/flight_interface_probe.jsonl"
#define BALLOON_CONTROL_HANDSHAKE_TRIGGER "/tmp/balloon_control_handshake.enable"
#define BALLOON_CONTROL_LOOP_SELFTEST_TRIGGER "/tmp/balloon_control_loop_selftest.enable"
#define BALLOON_FC_JOYSTICK_AUTH_SUCCESS_RETURN ((T_DjiReturnCode) 0x1B00000000ULL)
#define BALLOON_CONTROL_LOOP_SELFTEST_COUNT 8

static bool s_probeRan = false;

static void AppendProbeEvent(const char *event, T_DjiReturnCode code, const char *detail);

static T_DjiReturnCode ProbeJoystickAuthorityCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    (void) eventData;
    AppendProbeEvent("joystick_authority_event_callback", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "callback invoked, no command sent");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static bool ProbeReturnCodeOk(T_DjiReturnCode code)
{
    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return true;
    }

    if (code == DJI_ERROR_FC_MODULE_CODE_OBTAIN_CONTROL_SUCCESS) {
        return true;
    }

    if (code == DJI_ERROR_FC_MODULE_CODE_RELEASE_CONTROL_SUCCESS) {
        return true;
    }

    if (code == DJI_ERROR_FC_JOYSTICK_MODULE_OBTAIN_RELEASE_JOYSTICK_AUTH_SUCCESS) {
        return true;
    }

    if (code == BALLOON_FC_JOYSTICK_AUTH_SUCCESS_RETURN) {
        return true;
    }

    return false;
}

#include "balloon_clock.h"

static uint64_t ProbeNowMs(void)
{
    return BalloonClock_NowMs();
}

static bool ProbeFileExists(const char *path)
{
    FILE *fp = fopen(path, "r");

    if (fp == NULL) {
        return false;
    }

    fclose(fp);
    return true;
}

static void AppendProbeEvent(const char *event, T_DjiReturnCode code, const char *detail)
{
    FILE *fp;
    time_t now = time(NULL);

    (void) system("mkdir -p /home/uav/offline_balloon_strike_test");
    fp = fopen(BALLOON_CHASE_PROBE_LOG, "a");
    if (fp == NULL) {
        USER_LOG_ERROR("Balloon chase probe failed to open log: %s", BALLOON_CHASE_PROBE_LOG);
        return;
    }

    fprintf(fp,
            "{\"ts\":%lld,\"event\":\"%s\",\"code\":\"0x%08llX\",\"ok\":%s,"
            "\"no_obtain_authority\":true,\"no_execute_joystick\":true,\"detail\":\"%s\"}\n",
            (long long) now,
            event == NULL ? "" : event,
            (unsigned long long) code,
            ProbeReturnCodeOk(code) ? "true" : "false",
            detail == NULL ? "" : detail);
    fclose(fp);
}

static void AppendControlLoopSelfTestEvent(const char *event,
                                           uint32_t iteration,
                                           T_DjiReturnCode code,
                                           const char *detail)
{
    FILE *fp;
    time_t now = time(NULL);

    (void) system("mkdir -p /home/uav/offline_balloon_strike_test");
    fp = fopen(BALLOON_CHASE_PROBE_LOG, "a");
    if (fp == NULL) {
        USER_LOG_ERROR("Balloon chase probe failed to open log: %s", BALLOON_CHASE_PROBE_LOG);
        return;
    }

    fprintf(fp,
            "{\"ts\":%lld,\"event\":\"%s\",\"iteration\":%u,\"code\":\"0x%08llX\","
            "\"ok\":%s,\"no_obtain_authority\":true,\"no_execute_joystick\":true,"
            "\"detail\":\"%s\"}\n",
            (long long) now,
            event == NULL ? "" : event,
            iteration,
            (unsigned long long) code,
            ProbeReturnCodeOk(code) ? "true" : "false",
            detail == NULL ? "" : detail);
    fclose(fp);
}

static void AppendControlHandshakeEvent(const char *event,
                                        T_DjiReturnCode code,
                                        bool obtainAuthorityAttempted,
                                        bool releaseAuthorityAttempted,
                                        const char *detail)
{
    FILE *fp;
    time_t now = time(NULL);

    (void) system("mkdir -p /home/uav/offline_balloon_strike_test");
    fp = fopen(BALLOON_CHASE_PROBE_LOG, "a");
    if (fp == NULL) {
        USER_LOG_ERROR("Balloon chase probe failed to open log: %s", BALLOON_CHASE_PROBE_LOG);
        return;
    }

    fprintf(fp,
            "{\"ts\":%lld,\"event\":\"%s\",\"code\":\"0x%08llX\",\"ok\":%s,"
            "\"obtain_authority_attempted\":%s,\"release_authority_attempted\":%s,"
            "\"execute_joystick_attempted\":false,\"detail\":\"%s\"}\n",
            (long long) now,
            event == NULL ? "" : event,
            (unsigned long long) code,
            ProbeReturnCodeOk(code) ? "true" : "false",
            obtainAuthorityAttempted ? "true" : "false",
            releaseAuthorityAttempted ? "true" : "false",
            detail == NULL ? "" : detail);
    fclose(fp);
}

static void AppendMockCommandEvent(const T_TargetChaseCommand *command, E_TargetChaseMode mode)
{
    FILE *fp;
    time_t now = time(NULL);

    fp = fopen(BALLOON_CHASE_PROBE_LOG, "a");
    if (fp == NULL || command == NULL) {
        if (fp != NULL) {
            fclose(fp);
        }
        return;
    }

    fprintf(fp,
            "{\"ts\":%lld,\"event\":\"target_chase_mock_command\",\"code\":\"0x00000000\","
            "\"ok\":true,\"no_obtain_authority\":true,\"no_execute_joystick\":true,"
            "\"mode\":%d,\"x_mps\":%.3f,\"y_mps\":%.3f,\"z_mps\":%.3f,"
            "\"yaw_rate_degps\":%.3f,\"bearing_yaw_deg\":%.3f,"
            "\"bearing_vertical_deg\":%.3f,\"should_send\":%s}\n",
            (long long) now,
            (int) mode,
            command->x_mps,
            command->y_mps,
            command->z_mps,
            command->yaw_rate_degps,
            command->target_bearing_yaw_deg,
            command->target_bearing_vertical_deg,
            command->should_send ? "true" : "false");
    fclose(fp);
}

static void RunPureTargetChaseMock(void)
{
    const uint64_t nowMs = 1000;
    T_TargetChaseConfig config = TargetChase_DefaultConfig(5472.0f, 3648.0f, 73.7f, 45.7f);
    T_TargetChaseController controller;
    T_TargetChaseAircraftState state = {
        .altitude_agl_m = 5.0f,
        .velocity_neu_mps = {0.0f, 0.0f, 0.0f},
        .attitude_deg = {0.0f, 0.0f, 0.0f},
        .angular_rate_radps = {0.0f, 0.0f, 0.0f},
        .in_air = true,
        .velocity_healthy = true,
        .timestamp_ms = nowMs,
    };
    T_TargetChaseCommand command;

    TargetChase_Init(&controller, &config);
    TargetChase_UpdateAircraftState(&controller, &state);
    TargetChase_UpdateTargetBox(&controller, 3000.0f, 2050.0f, 3600.0f, 1450.0f, nowMs);
    TargetChase_SetManualTrigger(&controller, true, nowMs);
    TargetChase_SetAuthority(&controller, true, nowMs);

    if (TargetChase_Step(&controller, nowMs + 10, &command)) {
        AppendMockCommandEvent(&command, TargetChase_GetMode(&controller));
    } else {
        AppendProbeEvent("target_chase_mock_command_failed", DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN, "pure algorithm returned false");
    }
}

static T_DjiReturnCode ProbeSubscribeTopic(E_DjiFcSubscriptionTopic topic,
                                           E_DjiDataSubscriptionTopicFreq freq,
                                           const char *name)
{
    T_DjiReturnCode code = BalloonSubscription_Acquire(BALLOON_SUB_PROBE, topic, freq, NULL);
    AppendProbeEvent(name, code, "read-only FC topic subscription");
    return code;
}

static void ProbeSetJoystickMode(void)
{
    T_DjiFlightControllerJoystickMode joystickMode = {
        .horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        .verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        .yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        .horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        .stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    DjiFlightController_SetJoystickMode(joystickMode);
    AppendProbeEvent("set_joystick_mode", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "mode configured only, no authority and no joystick command");
}

static void RunControlAuthorityHandshakeIfRequested(T_DjiOsalHandler *osal)
{
    FILE *trigger = fopen(BALLOON_CONTROL_HANDSHAKE_TRIGGER, "r");
    T_DjiReturnCode code;

    if (trigger == NULL) {
        return;
    }

    fclose(trigger);
    (void) remove(BALLOON_CONTROL_HANDSHAKE_TRIGGER);

    AppendControlHandshakeEvent("control_handshake_triggered",
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                false,
                                false,
                                "one-shot trigger consumed before authority test");

    DjiFlightController_SetJoystickMode((T_DjiFlightControllerJoystickMode) {
        .horizontalControlMode = DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        .verticalControlMode = DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        .yawControlMode = DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        .horizontalCoordinate = DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        .stableControlMode = DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    });
    AppendControlHandshakeEvent("control_handshake_set_joystick_mode",
                                DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                false,
                                false,
                                "mode configured, still no joystick command sent");

    code = DjiFlightController_ObtainJoystickCtrlAuthority();
    AppendControlHandshakeEvent("control_handshake_obtain_authority",
                                code,
                                true,
                                false,
                                "requested joystick authority, no joystick command sent");
    if (!ProbeReturnCodeOk(code)) {
        AppendControlHandshakeEvent("control_handshake_release_skipped",
                                    code,
                                    false,
                                    false,
                                    "obtain authority failed, release skipped");
        return;
    }

    if (osal != NULL) {
        osal->TaskSleepMs(300);
    }

    code = DjiFlightController_ReleaseJoystickCtrlAuthority();
    AppendControlHandshakeEvent("control_handshake_release_authority",
                                code,
                                false,
                                true,
                                "released joystick authority, no joystick command sent");
}

static void RunTargetChaseRunOnceSelfTest(T_DjiOsalHandler *osal)
{
    T_TargetChaseConfig config = TargetChase_DefaultConfig(5472.0f, 3648.0f, 73.7f, 45.7f);
    T_DjiReturnCode code;
    T_DjiFcSubscriptionFlightStatus flightStatus = DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED;
    uint32_t i;

    if (!ProbeFileExists(BALLOON_CONTROL_LOOP_SELFTEST_TRIGGER)) {
        AppendProbeEvent("control_loop_selftest_skipped",
                         DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                         "trigger file not present");
        return;
    }

    remove(BALLOON_CONTROL_LOOP_SELFTEST_TRIGGER);
    config.target_timeout_ms = 30000.0f;

    AppendProbeEvent("control_loop_selftest_started",
                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "non-flight multi RunOnce test");

    code = DjiTargetChase_Init(&config);
    AppendProbeEvent("control_loop_selftest_target_chase_init", code, "initializes FC and subscriptions");
    if (!ProbeReturnCodeOk(code)) {
        return;
    }

    if (osal != NULL) {
        osal->TaskSleepMs(800);
    }

    code = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                   (uint8_t *) &flightStatus,
                                                   sizeof(flightStatus),
                                                   NULL);
    AppendProbeEvent("control_loop_selftest_flight_status", code,
                     flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR ? "in_air_skip" : "not_in_air");
    if (!ProbeReturnCodeOk(code) || flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        (void) DjiTargetChase_Abort();
        (void) DjiTargetChase_DeInit();
        AppendProbeEvent("control_loop_selftest_aborted",
                         DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                         "aircraft is in air or flight status unavailable");
        return;
    }

    code = DjiTargetChase_SetManualTrigger(true, ProbeNowMs());
    AppendProbeEvent("control_loop_selftest_manual_trigger", code, "enabled for ground safety-gate test");

    for (i = 1; i <= BALLOON_CONTROL_LOOP_SELFTEST_COUNT; ++i) {
        uint64_t nowMs = ProbeNowMs();
        char detail[96];

        code = DjiTargetChase_OnTargetBox(5472.0f,
                                          3648.0f,
                                          737.675f,
                                          2791.901f,
                                          3690.634f,
                                          175.125f,
                                          nowMs);
        snprintf(detail, sizeof(detail), "static latest-target bbox iteration %u", i);
        AppendControlLoopSelfTestEvent("control_loop_selftest_target_box", i, code, detail);

        code = DjiTargetChase_RunOnce(nowMs);
        AppendControlLoopSelfTestEvent("control_loop_selftest_run_once", i, code,
                                       "expected no authority and no joystick action on ground");

        if (osal != NULL) {
            osal->TaskSleepMs(100);
        }
    }

    (void) DjiTargetChase_SetManualTrigger(false, ProbeNowMs());
    (void) DjiTargetChase_Abort();
    (void) DjiTargetChase_DeInit();
    AppendProbeEvent("control_loop_selftest_finished",
                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "completed non-flight multi RunOnce test");
}

T_DjiReturnCode BalloonChaseProbe_RunNoFlightProbe(void)
{
    T_DjiReturnCode code;
    T_DjiFcSubscriptionFlightStatus flightStatus = DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    bool subscriptionInitialized = false;

    if (s_probeRan) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    s_probeRan = true;

    AppendProbeEvent("probe_started", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "first-tier no-flight probe");

    AppendProbeEvent("flight_controller_init_skipped",
                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "RID must come from the operator/RC; startup probe does not report RID");

    code = BalloonSubscription_Init();
    AppendProbeEvent("fc_subscription_init", code, "initialized shared subscription module");
    subscriptionInitialized = ProbeReturnCodeOk(code);
    if (subscriptionInitialized) {
        (void) ProbeSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                   DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                   "subscribe_height_fusion");
        (void) ProbeSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                   DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                   "subscribe_velocity");
        (void) ProbeSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                   DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                   "subscribe_quaternion");
        (void) ProbeSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
                                   DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                   "subscribe_angular_rate");
        (void) ProbeSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                   DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                   "subscribe_flight_status");
    }

    AppendProbeEvent("joystick_probe_skipped",
                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "no joystick setup or authority probe without real RID");

    if (osal != NULL) {
        osal->TaskSleepMs(1000);
    }

    if (subscriptionInitialized) {
        code = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                       (uint8_t *) &flightStatus,
                                                       sizeof(flightStatus),
                                                       NULL);
        AppendProbeEvent("fc_subscription_get_flight_status", code, "read-only subscription value");
    }

    RunPureTargetChaseMock();

    if (subscriptionInitialized) {
        (void) BalloonSubscription_Release(BALLOON_SUB_PROBE, DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION);
        (void) BalloonSubscription_Release(BALLOON_SUB_PROBE, DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
        (void) BalloonSubscription_Release(BALLOON_SUB_PROBE, DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
        (void) BalloonSubscription_Release(BALLOON_SUB_PROBE, DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
        (void) BalloonSubscription_Release(BALLOON_SUB_PROBE, DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
        AppendProbeEvent("fc_subscription_release", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                         "released probe ownership; shared subscriptions retained");
    }

    RunTargetChaseRunOnceSelfTest(osal);

    AppendProbeEvent("probe_finished", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     "did not obtain authority and did not execute joystick action");

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
