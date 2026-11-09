/* Exercise the actual sampling header with SDK functions replaced by local
 * stubs. fopen is redirected exclusively to a private fixture directory.
 * This executable never links DJI libraries or opens aircraft devices. */
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static const char *fixture_root;
static FILE *fixture_open(const char *path, const char *mode)
{
    char target[1024];
    const char *name = strrchr(path, '/');
    assert(name != NULL);
    int count = snprintf(target, sizeof(target), "%s/%s", fixture_root, name + 1);
    assert(count > 0 && count < (int)sizeof(target));
    return fopen(target, mode);
}
static void test_log(const char *message, ...) { (void)message; }
#define USER_LOG_WARN(...) test_log(__VA_ARGS__)
#define USER_LOG_INFO(...) test_log(__VA_ARGS__)
#define fopen fixture_open
#include "../psdk-overlay/samples/sample_c/module_sample/balloon_widget/balloon_timing_capture.h"
#undef fopen

static T_DjiDataTimestamp next_stamp;
static T_DjiFcSubscriptionGimbalAngles next_angles;
static unsigned int reads, subscribes, releases;
static bool timing_owner, external_owner, untouched;
static T_DjiReturnCode subscribe_error;

T_DjiReturnCode BalloonPhotoDetector_StartCalibrationLiveview(void) { return 0; }
T_DjiReturnCode BalloonPhotoDetector_StopCalibrationLiveview(void) { return 0; }
T_DjiReturnCode BalloonSubscription_Acquire(E_BalloonSubscriptionOwner owner,
    E_DjiFcSubscriptionTopic topic, E_DjiDataSubscriptionTopicFreq frequency,
    DjiReceiveDataOfTopicCallback callback)
{
    assert(owner == BALLOON_SUB_TIMING && topic == DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    assert(frequency == DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ && !callback);
    ++subscribes;
    if (subscribe_error) return subscribe_error;
    timing_owner = true;
    return 0;
}
T_DjiReturnCode BalloonSubscription_Release(E_BalloonSubscriptionOwner owner,
    E_DjiFcSubscriptionTopic topic)
{
    assert(owner == BALLOON_SUB_TIMING && topic == DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    ++releases;
    timing_owner = false;
    return 0;
}
void BalloonSubscription_ForgetMissing(E_DjiFcSubscriptionTopic topic)
{
    assert(topic == DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
}
T_DjiReturnCode DjiFcSubscription_GetLatestValueOfTopic(E_DjiFcSubscriptionTopic topic,
    uint8_t *data, uint16_t size, T_DjiDataTimestamp *stamp)
{
    assert(timing_owner && topic == DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    assert(size == sizeof(next_angles));
    ++reads;
    if (!untouched) memcpy(data, &next_angles, sizeof(next_angles));
    *stamp = next_stamp;
    return 0;
}
static void write_fixture(const char *path, const char *text)
{
    FILE *file = fixture_open(path, "w"); assert(file);
    assert(fputs(text, file) >= 0); assert(fclose(file) == 0);
}
static void remove_fixture(const char *path)
{
    char target[1024];
    snprintf(target, sizeof(target), "%s/%s", fixture_root, strrchr(path, '/') + 1);
    remove(target);
}
static int rows(void)
{
    FILE *file = fixture_open(BALLOON_TIMING_LOG, "r");
    if (!file) return 0;
    int count = -1; char line[1024];
    while (fgets(line, sizeof(line), file)) ++count;
    fclose(file); return count < 0 ? 0 : count;
}
static void stamp(uint32_t ms, uint32_t us)
{
    next_stamp.millisecond = ms; next_stamp.microsecond = us;
}
static void begin(void)
{
    char text[160];
    snprintf(text, sizeof(text), "isolated-sampling-test %llu\n",
        (unsigned long long)(BalloonClock_NowMs() + 30000));
    write_fixture(BALLOON_TIMING_REQUEST, text);
    assert(BalloonTiming_Requested());
    BalloonTiming_Tick(false);
}
static void end(void)
{
    remove_fixture(BALLOON_TIMING_REQUEST);
    BalloonTiming_Tick(false);
    assert(!s_timingOwnSubscription && !timing_owner);
}
int main(int argc, char **argv)
{
    assert(argc == 2); fixture_root = argv[1];
    remove_fixture(BALLOON_TIMING_LOG);
    remove_fixture(BALLOON_TIMING_REQUEST);
    write_fixture("/proc/sys/kernel/random/boot_id", "11111111-1111-4111-8111-111111111111\n");

    stamp(0, 0); begin(); BalloonTiming_Tick(false); assert(rows() == 0);
    stamp(1000, 1000000); BalloonTiming_Tick(false); assert(rows() == 0);
    BalloonTiming_Tick(false); assert(rows() == 0);
    stamp(1100, 1100000); BalloonTiming_Tick(false); assert(rows() == 1);
    stamp(0, 0); BalloonTiming_Tick(false); assert(rows() == 1);
    stamp(1200, 1200000); BalloonTiming_Tick(false); assert(rows() == 1);
    stamp(1300, 1300000); BalloonTiming_Tick(false); assert(rows() == 2);
    stamp(1250, 1250000); BalloonTiming_Tick(false); assert(rows() == 2);
    stamp(1350, 1350000); BalloonTiming_Tick(false); BalloonTiming_Tick(false); assert(rows() == 2);
    stamp(1450, 1450000); BalloonTiming_Tick(false); assert(rows() == 3);
    end();
    puts("PASS: zero timestamps, first-sample warmup, unchanged and reverse/reset timestamps");

    remove_fixture(BALLOON_TIMING_LOG);
    stamp(UINT32_MAX - 5, UINT32_MAX - 10); begin(); assert(rows() == 0);
    stamp(5, 12); BalloonTiming_Tick(false); assert(rows() == 1);
    stamp(6, UINT32_MAX - 2); BalloonTiming_Tick(false); assert(rows() == 2);
    stamp(6, 3); BalloonTiming_Tick(false); assert(rows() == 3);
    stamp(6, 2); BalloonTiming_Tick(false); assert(rows() == 3); end();
    puts("PASS: uint32 wrap and same-millisecond ordering");

    remove_fixture(BALLOON_TIMING_LOG);
    next_angles.x = NAN; stamp(1000, 1000000); begin(); assert(rows() == 0);
    next_angles.x = 1.047e34f; BalloonTiming_Tick(false); assert(rows() == 0);
    next_angles.x = 0; next_angles.z = 361; BalloonTiming_Tick(false); assert(rows() == 0);
    next_angles.z = 0; untouched = true; BalloonTiming_Tick(false); assert(rows() == 0);
    untouched = false; BalloonTiming_Tick(false); assert(rows() == 0);
    stamp(1100, 1100000); BalloonTiming_Tick(false); assert(rows() == 1); end();
    puts("PASS: nonfinite/absurd/untouched SDK angles rejected before baseline");

    int before = rows(); unsigned int before_release = releases;
    external_owner = true; stamp(2000, 2000000); begin();
    assert(s_timingOwnSubscription && timing_owner && rows() == before);
    stamp(2100, 2100000); BalloonTiming_Tick(false); assert(rows() == before + 1);
    end(); assert(external_owner && releases == before_release + 1);
    unsigned int before_read = reads;
    subscribe_error = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN; begin();
    assert(!s_timingOwnSubscription && reads == before_read); end();
    assert(subscribes > 0);
    puts("PASS: borrowed subscription warms up and failed subscribe never reads SDK");
    return 0;
}
