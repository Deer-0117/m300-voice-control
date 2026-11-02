#ifndef BALLOON_TIMING_CAPTURE_H
#define BALLOON_TIMING_CAPTURE_H

/* Single watcher owns SDK calls. Never call under Widget's action mutex. */
#include "dji_fc_subscription.h"
#include "balloon_clock.h"
#include "balloon_photo_detector.h"
#include <unistd.h>
#include <math.h>

#define BALLOON_TIMING_REQUEST "/tmp/balloon_timing_capture.request"
#define BALLOON_TIMING_LOG "/home/uav/offline_balloon_strike_test/calibration_camera_pose.csv"

static bool s_timingOwnSubscription = false;
static bool s_timingOwnLiveview = false;
/* Ready also covers an explicitly confirmed external subscription. Reading
 * SUCCESS alone is not evidence of a subscription in this capture session. */
static bool s_timingSubscriptionReady = false;

static void BalloonTiming_PoseProblem(uint64_t now, const char *reason, T_DjiReturnCode code)
{
    static uint64_t lastLog = 0;
    if (lastLog == 0 || now < lastLog || now - lastLog >= 1000) {
        USER_LOG_WARN("Timing capture pose rejected: %s code=0x%llX", reason,
                      (unsigned long long)code);
        lastLog = now;
    }
}

static bool BalloonTiming_Requested(void)
{
    FILE *request = fopen(BALLOON_TIMING_REQUEST, "r");
    if (!request) return false;
    fclose(request);
    return true;
}

static void BalloonTiming_Tick(bool busy)
{
    static uint64_t lastAttempt = 0;
    static uint64_t lastStreamAttempt = 0;
    static T_DjiDataTimestamp lastStamp = {0};
    static bool haveStamp = false;
    T_DjiFcSubscriptionGimbalAngles angles = {0};
    T_DjiDataTimestamp stamp = {0};
    T_DjiReturnCode code;
    char token[80] = {0}, clockId[80];
    unsigned long long deadline = 0;
    FILE *request = fopen(BALLOON_TIMING_REQUEST, "r");
    bool requested = false;
    uint64_t begin, end, now = BalloonClock_NowMs();
    if (request) {
        requested = fscanf(request, "%79s %llu", token, &deadline) == 2 &&
                    now > 0 && deadline > now && deadline - now <= 65000;
        fclose(request);
    }
    if (!requested || busy) {
        if (s_timingOwnLiveview) {
            code = BalloonPhotoDetector_StopCalibrationLiveview();
            if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) s_timingOwnLiveview = false;
        }
        if (s_timingOwnSubscription) {
            code = DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
            if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) s_timingOwnSubscription = false;
            else BalloonTiming_PoseProblem(now, "unsubscribe failed; ownership retained", code);
        }
        if (!s_timingOwnSubscription) {
            s_timingSubscriptionReady = false;
            haveStamp = false;
            lastAttempt = 0;
        }
        return;
    }
    if (!s_timingOwnLiveview && (lastStreamAttempt == 0 || now >= lastStreamAttempt + 1000)) {
        lastStreamAttempt = now;
        code = BalloonPhotoDetector_StartCalibrationLiveview();
        s_timingOwnLiveview = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (!s_timingSubscriptionReady) {
        if (lastAttempt != 0 && now >= lastAttempt && now - lastAttempt < 1000) return;
        lastAttempt = now;
        /* Init may already be active; Subscribe's explicit result decides
         * ownership. Never deinitialize this shared SDK module. */
        T_DjiReturnCode initCode = DjiFcSubscription_Init();
        code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                                              DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
        s_timingOwnSubscription = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        s_timingSubscriptionReady = s_timingOwnSubscription ||
            code == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE;
        USER_LOG_INFO("Timing capture subscription: init=0x%llX subscribe=0x%llX owned=%d ready=%d",
                      (unsigned long long)initCode, (unsigned long long)code,
                      s_timingOwnSubscription, s_timingSubscriptionReady);
        if (!s_timingSubscriptionReady) {
            BalloonTiming_PoseProblem(now, "subscribe failed; no pose read attempted", code);
            return;
        }
        haveStamp = false;
    }
    angles.x = angles.y = angles.z = NAN; /* Detect successful calls that did not fill the output. */
    begin = BalloonClock_NowMs();
    code = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
        (uint8_t *)&angles, sizeof(angles), &stamp);
    end = BalloonClock_NowMs();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        BalloonTiming_PoseProblem(end, "GetLatestValue failed", code);
        if (code == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_NOT_SUBSCRIBED) {
            s_timingSubscriptionReady = s_timingOwnSubscription = false;
            haveStamp = false;
        }
        return;
    }
    /* Bounds reject corrupt data, not a mechanical-limit calibration. Allow
     * both signed yaw and 0..360 degree conventions; never wrap huge values. */
    if (!isfinite(angles.x) || !isfinite(angles.y) || !isfinite(angles.z) ||
        fabsf(angles.x) > 180.0f || fabsf(angles.y) > 180.0f || fabsf(angles.z) > 360.0f) {
        char reason[160];
        snprintf(reason, sizeof(reason), "invalid angles pitch=%g roll=%g yaw=%g",
                 (double)angles.x, (double)angles.y, (double)angles.z);
        BalloonTiming_PoseProblem(end, reason, code);
        return;
    }
    if (haveStamp && stamp.millisecond == lastStamp.millisecond &&
        stamp.microsecond == lastStamp.microsecond) {
        BalloonTiming_PoseProblem(end, "SDK timestamp unchanged; host time is not a new sample", code);
        return;
    }
    lastStamp = stamp;
    haveStamp = true;
    if (!BalloonClock_Id(clockId)) return;
    FILE *log = fopen(BALLOON_TIMING_LOG, "a");
    if (!log) return;
    (void) fseek(log, 0, SEEK_END);
    if (ftell(log) == 0)
        fprintf(log,"ts_ms,yaw_deg,pitch_deg,clock_id,host_read_begin_ms,host_read_end_ms,sdk_millisecond_raw,sdk_microsecond_raw,session_token\n");
    fprintf(log,"%llu,%.6f,%.6f,%s,%llu,%llu,%u,%u,%s\n",
        (unsigned long long)end, angles.z, angles.x, clockId,
        (unsigned long long)begin,(unsigned long long)end,
        (unsigned)stamp.millisecond,(unsigned)stamp.microsecond,token);
    fclose(log);
}
#endif
