/* Portable host/Jetson tests. Only SDK subscription stubs are provided:
 * a camera, gimbal or flight-control dependency would fail to link. */
#include <assert.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>

/* MinGW's default assert can show a modal CRT dialog when stdout is piped,
 * turning a useful failed check into a CI timeout. Fail without any UI. */
#undef assert
#define assert(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "CHECK FAILED %s:%d: %s\n", __FILE__, __LINE__, #condition); \
        fflush(stderr); _Exit(2); \
    } \
} while (0)

static const char *test_directory;
static int runtime_mode;
static int thread_creations;
static void *(*created_functions[3])(void *);
static int test_thread_create(pthread_t *thread, const pthread_attr_t *attributes,
    void *(*function)(void *), void *argument)
{
    (void)thread; (void)attributes; (void)argument;
    if (runtime_mode) return pthread_create(thread, attributes, function, argument);
    assert(thread_creations < 3);
    created_functions[thread_creations++] = function;
    return 0;
}
static int test_thread_detach(pthread_t thread) {
    if (runtime_mode) return pthread_detach(thread);
    (void)thread; return 0;
}
#define pthread_create test_thread_create
#define pthread_detach test_thread_detach
#define BALLOON_TELEMETRY_DIRECTORY test_directory
#define BALLOON_TELEMETRY_ROTATE_BYTES (runtime_mode ? 1048576U : 8192U)
#include "balloon_widget/balloon_background_telemetry_impl.h"
#undef pthread_create
#undef pthread_detach

static int init_count, subscribe_count, get_count, unsubscribe_count, deinit_count;
static T_DjiReturnCode init_result, subscribe_result, get_result;
static DjiReceiveDataOfTopicCallback sdk_control_callback;
static unsigned control_callback_calls;
static uint32_t sdk_ms = 1000;
static int missing_float, bad_quaternion;
static pthread_mutex_t runtime_stat_mutex = PTHREAD_MUTEX_INITIALIZER;
static int runtime_subscription_blocked, runtime_blocked_attempts, runtime_reads_during_block;

T_DjiReturnCode BackgroundTelemetryTest_PeerAcquire(void);
T_DjiReturnCode BackgroundTelemetryTest_PeerRelease(void);
T_DjiReturnCode DjiFcSubscription_Init(void) { ++init_count; return init_result; }
T_DjiReturnCode DjiFcSubscription_DeInit(void) { ++deinit_count; return 0; }
T_DjiReturnCode DjiFcSubscription_SubscribeTopic(E_DjiFcSubscriptionTopic topic,
    E_DjiDataSubscriptionTopicFreq frequency, DjiReceiveDataOfTopicCallback callback)
{
    ++subscribe_count;
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE) {
        assert(frequency == DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ && callback);
        if (!subscribe_result) sdk_control_callback = callback;
    } else {
        assert(callback == NULL);
    }
    if (runtime_mode && topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE) {
        pthread_mutex_lock(&runtime_stat_mutex);
        runtime_subscription_blocked = 1;
        ++runtime_blocked_attempts;
        pthread_mutex_unlock(&runtime_stat_mutex);
        usleep(1200000); /* Real SDK can block this long on an unsupported topic. */
        pthread_mutex_lock(&runtime_stat_mutex);
        runtime_subscription_blocked = 0;
        pthread_mutex_unlock(&runtime_stat_mutex);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    return subscribe_result;
}
T_DjiReturnCode DjiFcSubscription_UnSubscribeTopic(E_DjiFcSubscriptionTopic topic)
{ (void)topic; ++unsubscribe_count; return 0; }
T_DjiReturnCode DjiFcSubscription_GetLatestValueOfTopic(E_DjiFcSubscriptionTopic topic,
    uint8_t *value, uint16_t size, T_DjiDataTimestamp *stamp)
{
    ++get_count;
    if (get_result) return get_result;
    stamp->millisecond = runtime_mode ? (uint32_t)BalloonClock_NowMs() : sdk_ms;
    stamp->microsecond = stamp->millisecond * 1000U;
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY) {
        T_DjiFcSubscriptionVelocity v = {{3.0f, 4.0f, -2.0f}, 1, 0};
        assert(size == sizeof(v));
        if (!missing_float) memcpy(value, &v, size);
        if (runtime_mode) {
            pthread_mutex_lock(&runtime_stat_mutex);
            if (runtime_subscription_blocked) ++runtime_reads_during_block;
            pthread_mutex_unlock(&runtime_stat_mutex);
        }
    } else if (topic == DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION) {
        T_DjiFcSubscriptionQuaternion q = {0.70710678f, 0, 0, 0.70710678f};
        if (bad_quaternion) q.q0 = q.q3 = 0;
        assert(size == sizeof(q)); memcpy(value, &q, size);
    } else if (topic == DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED) {
        T_DjiFcSubscriptionAngularRateFusioned rate = {0.1f, -0.2f, 0.3f};
        assert(size == sizeof(rate)); memcpy(value, &rate, size);
    } else if (topic == DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT) {
        assert(size == 1); *value = 2;
    } else if (topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE) {
        T_DjiFcSubscriptionControlDevice control = {0};
        control.deviceStatus = 0; control.controlMode = 6;
        control.flightStatus = 1;
        assert(size == sizeof(control)); memcpy(value, &control, size);
    } else if (topic == DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE) {
        assert(size == 1); *value = 6;
    } else {
        assert(!"Unexpected topic: camera and gimbal must not be required");
    }
    return 0;
}

static void test_shared_lifetime(void)
{
    init_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(BalloonSubscription_Init() == init_result);
    assert(!s_balloonSubscriptionInitialized);
    init_result = 0;
    assert(BalloonSubscription_Acquire(BALLOON_SUB_BACKGROUND,
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL) == 0);
    assert(init_count == 2 && subscribe_count == 1);
    assert(BackgroundTelemetryTest_PeerAcquire() == 0);
    assert(subscribe_count == 1);
    assert(BackgroundTelemetryTest_PeerRelease() == 0);
    assert(subscribe_count == 1 && unsubscribe_count == 0 && deinit_count == 0);
    assert(BalloonSubscription_Release(BALLOON_SUB_BACKGROUND,
        DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY) == 0);
    assert(unsubscribe_count == 0); /* SDK FIFO rule: retain process-lifetime. */
    assert(BackgroundTelemetryTest_PeerAcquire() == 0);
    assert(subscribe_count == 1);
    subscribe_result = DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE;
    assert(BalloonSubscription_Acquire(BALLOON_SUB_TIMING,
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL) == 0);
    assert(BalloonSubscription_Release(BALLOON_SUB_TIMING,
        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES) == 0);
    assert(unsubscribe_count == 0 && deinit_count == 0);
    subscribe_result = 0;
    BalloonSubscription_ForgetMissing(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    assert(BackgroundTelemetryTest_PeerAcquire() == 0);
    assert(subscribe_count == 3);
}

static T_DjiReturnCode test_control_callback(
    const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *stamp)
{
    assert(data && size == sizeof(T_DjiFcSubscriptionControlDevice) && stamp);
    ++control_callback_calls;
    return 0;
}

static void test_shared_control_device_callback(void)
{
    const E_DjiFcSubscriptionTopic topic = DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE;
    const T_DjiDataTimestamp stamp = {.millisecond = 1000};
    const T_DjiFcSubscriptionControlDevice sample = {0};
    const uint8_t *data = (const uint8_t *)&sample;
    const int first = subscribe_count;

    assert(BalloonSubscription_Acquire(BALLOON_SUB_BACKGROUND, topic,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL) == 0);
    assert(sdk_control_callback(data, sizeof(sample), &stamp) == 0 && !control_callback_calls);
    assert(BalloonSubscription_Acquire(BALLOON_SUB_CHASE, topic,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, test_control_callback) == 0);
    assert(subscribe_count == first + 1);
    assert(sdk_control_callback(data, sizeof(sample), &stamp) == 0 && control_callback_calls == 1);
    assert(BalloonSubscription_Release(BALLOON_SUB_CHASE, topic) == 0);
    assert(sdk_control_callback(data, sizeof(sample), &stamp) == 0 && control_callback_calls == 1);
    assert(BalloonSubscription_Acquire(BALLOON_SUB_CHASE, topic,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, test_control_callback) == 0);
    assert(sdk_control_callback(data, sizeof(sample), &stamp) == 0 && control_callback_calls == 2);

    /* An externally subscribed topic can provide passive reads but its
     * callback cannot confirm the chase owns flight authority. */
    BalloonSubscription_ForgetMissing(topic);
    subscribe_result = DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE;
    assert(BalloonSubscription_Acquire(BALLOON_SUB_BACKGROUND, topic,
        DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL) == 0);
    assert(BalloonSubscription_Acquire(BALLOON_SUB_CHASE, topic,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, test_control_callback)
        == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE);
    assert(sdk_control_callback(data, sizeof(sample), &stamp) == 0 && control_callback_calls == 2);
    subscribe_result = 0;
}

static void test_timestamp_progress(void)
{
    T_BalloonTelemetryProgress progress = {0};
    T_BalloonTelemetryMeta m = {0};
    m.valid = true; m.stamp.millisecond = 100; m.end_ms = 1000;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(m.valid && !m.fresh && !m.new_sample);
    m.stamp.millisecond = 120; m.end_ms = 1020;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(m.fresh && m.new_sample && m.age_ms == 0);
    m.end_ms = 1040;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(m.fresh && !m.new_sample && m.age_ms == 20);
    m.end_ms = 1240;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(!m.fresh && m.age_ms == 220);
    m.stamp.millisecond = 119;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(!m.fresh && !m.new_sample); /* Clock reversal establishes a new baseline. */
    m.stamp.millisecond = 139;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(m.fresh && m.new_sample);
    m.stamp.millisecond = m.stamp.microsecond = 0;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(!m.fresh && !m.have_progress);
    memset(&progress, 0, sizeof(progress));
    m.stamp.millisecond = UINT32_MAX - 5; m.end_ms = 2000;
    BalloonTelemetry_Progress(&m, &progress, 200);
    m.stamp.millisecond = 14; m.end_ms = 2020;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(m.fresh && m.new_sample); /* uint32 SDK wrap is forward progress. */
    m.code = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    BalloonTelemetry_Progress(&m, &progress, 200);
    assert(!m.fresh && !m.new_sample);
}

static void test_no_camera_read_and_format(void)
{
    T_BalloonTelemetryProgress progress[BALLOON_TELEMETRY_TOPIC_COUNT] = {0};
    T_DjiReturnCode codes[BALLOON_TELEMETRY_TOPIC_COUNT] = {0};
    T_BalloonTelemetrySample sample = {0};
    char json[8192], path[512];
    FILE *file;
    unsigned i;
    BalloonTelemetry_Read(&sample, progress, codes);
    assert(get_count == 6);
    for (i = 0; i < 6; ++i) assert(sample.meta[i].valid && !sample.meta[i].fresh);
    assert(fabs(sample.yaw - 90) < 0.01 && fabs(sample.roll) < 0.01);
    sdk_ms += 20;
    memset(&sample, 0, sizeof(sample));
    BalloonTelemetry_Read(&sample, progress, codes);
    for (i = 0; i < 6; ++i) assert(sample.meta[i].fresh && sample.meta[i].new_sample);
    assert(BalloonTelemetry_Format(json, sizeof(json), &sample) > 0);
    assert(strstr(json, "\"frame\":\"NED\"") && strstr(json, "\"u_mps\":-2"));
    printf("serialized valid row bytes: %u\n", (unsigned)strlen(json));
    snprintf(path, sizeof(path), "%s/sample.jsonl", test_directory);
    file = fopen(path, "wb"); assert(file); fputs(json, file); fclose(file);
    missing_float = bad_quaternion = 1;
    memset(&sample, 0, sizeof(sample));
    sdk_ms += 20;
    BalloonTelemetry_Read(&sample, progress, codes);
    assert(!sample.meta[0].valid && !sample.meta[1].valid);
    assert(BalloonTelemetry_Format(json, sizeof(json), &sample) > 0);
    assert(strstr(json, "\"n_mps\":null") && strstr(json, "\"yaw_deg\":null"));
    assert(!strstr(json, "nan") && !strstr(json, "NaN"));
    missing_float = bad_quaternion = 0;
    get_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    memset(&sample, 0, sizeof(sample));
    BalloonTelemetry_Read(&sample, progress, codes);
    for (i = 0; i < 6; ++i) assert(!sample.meta[i].valid && !sample.meta[i].fresh);
    get_result = 0;
    assert(BalloonTelemetry_Format(json, 16, &sample) == -1);
}

static void test_queue_and_rotation(void)
{
    T_BalloonTelemetrySample sample = {0};
    unsigned i;
    char line[4096], path[512];
    size_t bytes = 0;
    FILE *stream = NULL;
    memset(line, 'x', sizeof(line)); line[sizeof(line)-1] = '\n';
    for (i = 0; i < BALLOON_TELEMETRY_QUEUE_CAPACITY; ++i) {
        sample.seq = i + 1;
        assert(BalloonTelemetry_Enqueue(&sample));
    }
    assert(!BalloonTelemetry_Enqueue(&sample) && s_balloonTelemetryDropped == 1);
    assert(s_balloonTelemetryCount == BALLOON_TELEMETRY_QUEUE_CAPACITY);
    s_balloonTelemetryCount = 0;
    assert(BalloonTelemetry_Enqueue(&sample));
    assert(s_balloonTelemetryQueue[0].dropped == 1);
    for (i = 0; i < 12; ++i) assert(BalloonTelemetry_Write(&stream, &bytes, line, sizeof(line)));
    assert(fflush(stream) == 0); fclose(stream);
    for (i = 0; i < 4; ++i) {
        long size;
        BalloonTelemetry_Path(path, sizeof(path), i);
        stream = fopen(path, "rb"); assert(stream);
        assert(fseek(stream, 0, SEEK_END) == 0);
        size = ftell(stream); fclose(stream);
        assert(size > 0 && (unsigned long)size <= BALLOON_TELEMETRY_ROTATE_BYTES);
    }
    BalloonTelemetry_Path(path, sizeof(path), 4);
    stream = fopen(path, "rb"); assert(stream == NULL);
    BalloonTelemetry_WriteError();
    assert(s_balloonTelemetryErrors == 1);
}

static void test_runtime(void)
{
    FILE *file;
    char path[512], line[8192];
    unsigned rows = 0, fresh_velocity_rows = 0;
    unsigned long long first_seq = 0, last_seq = 0;
    int attempts, reads;
    runtime_mode = 1;
    puts("background telemetry runtime smoke starting");
    fflush(stdout);
    assert(BalloonTelemetry_Start() == 0);
    usleep(3200000);
    pthread_mutex_lock(&runtime_stat_mutex);
    attempts = runtime_blocked_attempts;
    reads = runtime_reads_during_block;
    pthread_mutex_unlock(&runtime_stat_mutex);
    printf("runtime SDK block observations: attempts=%d, concurrent velocity reads=%d\n", attempts, reads);
    fflush(stdout);
    /* This is a thread-isolation test, not a host realtime throughput test.
     * Even a loaded Windows runner must show multiple reads during the SDK
     * block; serializing subscription on the sampler would produce zero. */
    assert(attempts >= 1 && reads >= 5);
    /* Runtime limit is 1 MiB so host Windows file sharing cannot make reading
     * the fixture race a rename; deterministic tests exercise 8 KiB rotation. */
    for (unsigned i = 0; i < 4; ++i) {
        BalloonTelemetry_Path(path, sizeof(path), i);
        file = fopen(path, "rb");
        if (!file) continue;
        assert(fseek(file, 0, SEEK_END) == 0);
        long snapshot_bytes = ftell(file);
        assert(snapshot_bytes >= 0 && fseek(file, 0, SEEK_SET) == 0);
        while (ftell(file) < snapshot_bytes && fgets(line, sizeof(line), file)) {
            char *seq, *velocity;
            unsigned long long value = 0;
            if (!strchr(line, '\n') || !strstr(line, s_balloonTelemetrySession)) continue;
            seq = strstr(line, "\"seq\":");
            if (!seq || sscanf(seq, "\"seq\":%llu", &value) != 1) continue;
            if (!first_seq || value < first_seq) first_seq = value;
            if (value > last_seq) last_seq = value;
            ++rows;
            velocity = strstr(line, "\"velocity\":{");
            if (velocity && strstr(velocity, "\"velocity\":{\"code\":0,\"valid\":true,\"fresh\":true")) ++fresh_velocity_rows;
            assert(strstr(line, "\"queue_dropped_total\":0"));
            assert(strstr(line, "\"write_errors_total\":0"));
        }
        fclose(file);
    }
    assert(rows >= 6 && fresh_velocity_rows >= 6 && last_seq >= 20);
    assert(last_seq > first_seq);
    printf("background telemetry real-thread smoke passed: blocked attempts=%d, velocity reads during SDK block=%d, retained rows=%u, final seq=%llu\n",
        attempts, reads, rows, last_seq);
}

int main(int argc, char **argv)
{
    int initial_sdk_calls;
    assert(argc == 2 || (argc == 3 && !strcmp(argv[2], "--runtime")));
    test_directory = argv[1];
    if (argc == 3) {
        test_runtime();
        fflush(stdout);
        /* The recorder intentionally lives until process exit. Do not run CRT
         * fclose-all concurrently with its writer during this smoke fixture. */
        _Exit(0);
    }
    initial_sdk_calls = init_count;
    assert(BalloonTelemetry_Start() == 0);
    assert(BalloonTelemetry_Start() == 0);
    assert(thread_creations == 3 && init_count == initial_sdk_calls);
    assert(created_functions[0] == BalloonTelemetry_Writer && created_functions[1] == BalloonTelemetry_Sampler);
    assert(created_functions[2] == BalloonTelemetry_Subscriptions);
    test_shared_lifetime();
    test_shared_control_device_callback();
    test_timestamp_progress();
    test_no_camera_read_and_format();
    test_queue_and_rotation();
    assert(!unsubscribe_count && !deinit_count);
    puts("background telemetry tests passed: camera-free, shared ownership, freshness, queue, rotation");
    return 0;
}
