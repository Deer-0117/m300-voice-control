#ifndef BALLOON_BACKGROUND_TELEMETRY_IMPL_H
#define BALLOON_BACKGROUND_TELEMETRY_IMPL_H

/* Included once by dji_target_chase_task.c: deployed CMake already compiles it.
 * Only FC subscriptions are used here, never flight authority/control APIs. */
#include "balloon_background_telemetry.h"
#include "balloon_clock.h"
#include "dji_error.h"
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef BALLOON_TELEMETRY_DIRECTORY
#define BALLOON_TELEMETRY_DIRECTORY "/home/uav/offline_balloon_strike_test"
#endif
#ifndef BALLOON_TELEMETRY_ROTATE_BYTES
#define BALLOON_TELEMETRY_ROTATE_BYTES (64UL * 1024UL * 1024UL)
#endif
#define BALLOON_TELEMETRY_QUEUE_CAPACITY 256U
#define BALLOON_TELEMETRY_TOPIC_COUNT 6U

typedef struct {
    bool used, subscribed, sdk_owned;
    E_DjiFcSubscriptionTopic topic;
    unsigned owners;
} T_BalloonSharedTopic;
static pthread_mutex_t s_balloonSubscriptionMutex = PTHREAD_MUTEX_INITIALIZER;
static bool s_balloonSubscriptionInitialized;
static T_BalloonSharedTopic s_balloonSharedTopics[24];
static DjiReceiveDataOfTopicCallback s_balloonControlDeviceCallback;

static T_DjiReturnCode BalloonSubscription_ControlDeviceCallback(
    const uint8_t *data, uint16_t size, const T_DjiDataTimestamp *stamp)
{
    DjiReceiveDataOfTopicCallback callback =
        __atomic_load_n(&s_balloonControlDeviceCallback, __ATOMIC_ACQUIRE);
    return callback ? callback(data, size, stamp) : DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode BalloonSubscription_InitLocked(void)
{
    T_DjiReturnCode code;
    if (s_balloonSubscriptionInitialized) return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    code = DjiFcSubscription_Init();
    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) s_balloonSubscriptionInitialized = true;
    return code;
}

T_DjiReturnCode BalloonSubscription_Init(void)
{
    T_DjiReturnCode code;
    pthread_mutex_lock(&s_balloonSubscriptionMutex);
    code = BalloonSubscription_InitLocked();
    pthread_mutex_unlock(&s_balloonSubscriptionMutex);
    return code;
}

T_DjiReturnCode BalloonSubscription_Acquire(E_BalloonSubscriptionOwner owner,
    E_DjiFcSubscriptionTopic topic, E_DjiDataSubscriptionTopicFreq frequency,
    DjiReceiveDataOfTopicCallback callback)
{
    size_t i;
    T_BalloonSharedTopic *slot = NULL;
    T_DjiReturnCode code;
    bool controlDevice = topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE;
    DjiReceiveDataOfTopicCallback previous = NULL;

    if (callback && (!controlDevice || owner != BALLOON_SUB_CHASE))
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    pthread_mutex_lock(&s_balloonSubscriptionMutex);
    code = BalloonSubscription_InitLocked();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) goto done;
    for (i = 0; i < sizeof(s_balloonSharedTopics) / sizeof(s_balloonSharedTopics[0]); ++i) {
        if (s_balloonSharedTopics[i].used && s_balloonSharedTopics[i].topic == topic) {
            slot = &s_balloonSharedTopics[i];
            if (slot->subscribed) {
                if (callback && !slot->sdk_owned) {
                    code = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
                    goto done;
                }
                if (callback)
                    __atomic_store_n(&s_balloonControlDeviceCallback, callback, __ATOMIC_RELEASE);
                slot->owners |= (unsigned)owner;
                code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
                goto done;
            }
            break;
        }
        if (!s_balloonSharedTopics[i].used && slot == NULL) slot = &s_balloonSharedTopics[i];
    }
    if (slot == NULL) { code = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN; goto done; }
    if (callback)
        previous = __atomic_exchange_n(&s_balloonControlDeviceCallback, callback, __ATOMIC_ACQ_REL);
    /* Attach the trampoline regardless of which owner subscribes first.
     * The callback never acquires this mutex, even if called during Subscribe. */
    code = DjiFcSubscription_SubscribeTopic(topic,
        controlDevice ? DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ : frequency,
        controlDevice ? BalloonSubscription_ControlDeviceCallback : NULL);
    if (code == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE && controlDevice)
        __atomic_store_n(&s_balloonControlDeviceCallback, NULL, __ATOMIC_RELEASE);
    else if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && callback)
        __atomic_store_n(&s_balloonControlDeviceCallback, previous, __ATOMIC_RELEASE);

    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
        code == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE) {
        slot->used = true;
        slot->subscribed = true;
        slot->sdk_owned = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        slot->topic = topic;
        /* Borrow external data for readers only. Without our trampoline,
         * the flight authority monitor must fail closed. */
        if (callback && !slot->sdk_owned) {
            code = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
            goto done;
        }
        slot->owners |= (unsigned)owner;
        code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
done:
    pthread_mutex_unlock(&s_balloonSubscriptionMutex);
    return code;
}

T_DjiReturnCode BalloonSubscription_Release(E_BalloonSubscriptionOwner owner,
    E_DjiFcSubscriptionTopic topic)
{
    size_t i;
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    pthread_mutex_lock(&s_balloonSubscriptionMutex);
    for (i = 0; i < sizeof(s_balloonSharedTopics) / sizeof(s_balloonSharedTopics[0]); ++i) {
        T_BalloonSharedTopic *slot = &s_balloonSharedTopics[i];
        if (!slot->used || slot->topic != topic || !(slot->owners & (unsigned)owner)) continue;
        /* The SDK requires FIFO unsubscribe order. Background topics are
         * process-lifetime, so releasing later topics physically is illegal.
         * Keep the bounded set of subscriptions and release the logical owner
         * only. This also avoids disrupting consumers outside this manager. */
        if (owner == BALLOON_SUB_CHASE && topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE)
            __atomic_store_n(&s_balloonControlDeviceCallback, NULL, __ATOMIC_RELEASE);
        slot->owners &= ~(unsigned)owner;
        break;
    }
    pthread_mutex_unlock(&s_balloonSubscriptionMutex);
    return code;
}

void BalloonSubscription_ForgetMissing(E_DjiFcSubscriptionTopic topic)
{
    size_t i;
    pthread_mutex_lock(&s_balloonSubscriptionMutex);
    for (i = 0; i < sizeof(s_balloonSharedTopics) / sizeof(s_balloonSharedTopics[0]); ++i) {
        if (s_balloonSharedTopics[i].used && s_balloonSharedTopics[i].topic == topic)
            s_balloonSharedTopics[i].subscribed = false;
    }
    pthread_mutex_unlock(&s_balloonSubscriptionMutex);
}

typedef struct {
    T_DjiReturnCode code;
    T_DjiDataTimestamp stamp;
    uint64_t begin_ms, end_ms, last_progress_ms, age_ms;
    bool valid, fresh, new_sample, have_progress;
} T_BalloonTelemetryMeta;
typedef struct {
    T_DjiDataTimestamp previous;
    uint64_t last_progress_ms;
    bool have_stamp, have_progress;
} T_BalloonTelemetryProgress;
typedef struct {
    uint64_t seq, begin_ms, end_ms, dropped, errors;
    T_BalloonTelemetryMeta meta[BALLOON_TELEMETRY_TOPIC_COUNT];
    T_DjiFcSubscriptionVelocity velocity;
    T_DjiFcSubscriptionQuaternion quaternion;
    T_DjiFcSubscriptionAngularRateFusioned angular_rate;
    T_DjiFcSubscriptionFlightStatus flight_status;
    T_DjiFcSubscriptionControlDevice control_device;
    T_DjiFcSubscriptionDisplaymode display_mode;
    double roll, pitch, yaw;
} T_BalloonTelemetrySample;

static const E_DjiFcSubscriptionTopic s_balloonTelemetryTopics[BALLOON_TELEMETRY_TOPIC_COUNT] = {
    DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
    DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
    DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE
};
static const char *s_balloonTelemetryNames[BALLOON_TELEMETRY_TOPIC_COUNT] = {
    "velocity", "quaternion", "angular_rate", "flight_status", "control_device", "display_mode"
};
static pthread_mutex_t s_balloonTelemetryQueueMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_balloonTelemetryQueueReady = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t s_balloonTelemetryStartMutex = PTHREAD_MUTEX_INITIALIZER;
static T_BalloonTelemetrySample s_balloonTelemetryQueue[BALLOON_TELEMETRY_QUEUE_CAPACITY];
static unsigned s_balloonTelemetryHead, s_balloonTelemetryCount;
static uint64_t s_balloonTelemetryDropped, s_balloonTelemetryErrors;
static bool s_balloonTelemetryStarted;
static bool s_balloonTelemetryWriterStarted;
static bool s_balloonTelemetrySamplerStarted;
static pthread_mutex_t s_balloonTelemetrySubscriptionMutex = PTHREAD_MUTEX_INITIALIZER;
static T_DjiReturnCode s_balloonTelemetrySubscriptionCodes[BALLOON_TELEMETRY_TOPIC_COUNT];
static bool s_balloonTelemetryMissingTopics[BALLOON_TELEMETRY_TOPIC_COUNT];
static char s_balloonTelemetryClock[80], s_balloonTelemetrySession[128];

static void BalloonTelemetry_Progress(T_BalloonTelemetryMeta *meta,
    T_BalloonTelemetryProgress *state, uint64_t fresh_ms)
{
    uint32_t delta_ms, delta_us;
    meta->new_sample = false;
    if (meta->code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        (meta->stamp.millisecond || meta->stamp.microsecond)) {
        if (!state->have_stamp) {
            state->previous = meta->stamp;
            state->have_stamp = true;
        } else {
            delta_ms = meta->stamp.millisecond - state->previous.millisecond;
            delta_us = meta->stamp.microsecond - state->previous.microsecond;
            if ((delta_ms && delta_ms >= 0x80000000U) ||
                (!delta_ms && delta_us >= 0x80000000U)) {
                state->previous = meta->stamp;
                state->have_progress = false;
            } else if (delta_ms || delta_us) {
                state->previous = meta->stamp;
                state->last_progress_ms = meta->end_ms;
                state->have_progress = true;
                meta->new_sample = true;
            }
        }
    } else if (meta->code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        state->have_stamp = state->have_progress = false;
    }
    meta->have_progress = state->have_progress;
    meta->last_progress_ms = state->have_progress ? state->last_progress_ms : 0;
    meta->age_ms = state->have_progress && meta->end_ms >= state->last_progress_ms ?
        meta->end_ms - state->last_progress_ms : 0;
    meta->fresh = meta->valid && meta->code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        (meta->stamp.millisecond || meta->stamp.microsecond) && state->have_progress &&
        meta->end_ms >= state->last_progress_ms && meta->age_ms <= fresh_ms;
}

static void BalloonTelemetry_Read(T_BalloonTelemetrySample *sample,
    T_BalloonTelemetryProgress progress[BALLOON_TELEMETRY_TOPIC_COUNT],
    const T_DjiReturnCode subscriptions[BALLOON_TELEMETRY_TOPIC_COUNT])
{
    unsigned i;
    uint8_t *values[] = {(uint8_t *)&sample->velocity, (uint8_t *)&sample->quaternion,
        (uint8_t *)&sample->angular_rate, (uint8_t *)&sample->flight_status,
        (uint8_t *)&sample->control_device, (uint8_t *)&sample->display_mode};
    const uint16_t sizes[] = {sizeof(sample->velocity), sizeof(sample->quaternion),
        sizeof(sample->angular_rate), sizeof(sample->flight_status),
        sizeof(sample->control_device), sizeof(sample->display_mode)};
    sample->begin_ms = BalloonClock_NowMs();
    /* NaNs catch SDK success responses that did not populate float output. */
    sample->velocity.data.x = sample->velocity.data.y = sample->velocity.data.z = NAN;
    sample->quaternion.q0 = sample->quaternion.q1 = sample->quaternion.q2 = sample->quaternion.q3 = NAN;
    sample->angular_rate.x = sample->angular_rate.y = sample->angular_rate.z = NAN;
    sample->flight_status = 255;
    sample->display_mode = 255;
    memset(&sample->control_device, 0xff, sizeof(sample->control_device));
    for (i = 0; i < BALLOON_TELEMETRY_TOPIC_COUNT; ++i) {
        T_BalloonTelemetryMeta *meta = &sample->meta[i];
        meta->begin_ms = BalloonClock_NowMs();
        meta->code = subscriptions[i];
        if (meta->code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
            meta->code = DjiFcSubscription_GetLatestValueOfTopic(s_balloonTelemetryTopics[i],
                values[i], sizes[i], &meta->stamp);
        meta->end_ms = BalloonClock_NowMs();
        meta->valid = meta->code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    sample->meta[0].valid &= isfinite(sample->velocity.data.x) && isfinite(sample->velocity.data.y) &&
        isfinite(sample->velocity.data.z);
    {
        double w = sample->quaternion.q0, x = sample->quaternion.q1;
        double y = sample->quaternion.q2, z = sample->quaternion.q3;
        double norm = w*w + x*x + y*y + z*z;
        sample->meta[1].valid &= isfinite(norm) && norm >= 0.81 && norm <= 1.21;
        if (sample->meta[1].valid) {
            double inv = 1.0 / sqrt(norm), sin_pitch;
            w *= inv; x *= inv; y *= inv; z *= inv;
            sin_pitch = 2.0 * (w*y - z*x);
            if (sin_pitch > 1.0) sin_pitch = 1.0;
            if (sin_pitch < -1.0) sin_pitch = -1.0;
            sample->roll = atan2(2.0*(w*x+y*z), 1.0-2.0*(x*x+y*y)) * 57.29577951308232;
            sample->pitch = asin(sin_pitch) * 57.29577951308232;
            sample->yaw = atan2(2.0*(w*z+x*y), 1.0-2.0*(y*y+z*z)) * 57.29577951308232;
        }
    }
    sample->meta[2].valid &= isfinite(sample->angular_rate.x) && isfinite(sample->angular_rate.y) &&
        isfinite(sample->angular_rate.z);
    sample->meta[3].valid &= sample->flight_status <= 2;
    sample->meta[4].valid &= sample->control_device.deviceStatus != 7;
    sample->meta[5].valid &= sample->display_mode != 255;
    for (i = 0; i < BALLOON_TELEMETRY_TOPIC_COUNT; ++i)
        BalloonTelemetry_Progress(&sample->meta[i], &progress[i], i < 3 ? 200 : 500);
    sample->end_ms = BalloonClock_NowMs();
}

static bool BalloonTelemetry_Enqueue(T_BalloonTelemetrySample *sample)
{
    bool accepted = false;
    pthread_mutex_lock(&s_balloonTelemetryQueueMutex);
    if (s_balloonTelemetryCount == BALLOON_TELEMETRY_QUEUE_CAPACITY) ++s_balloonTelemetryDropped;
    else {
        sample->dropped = s_balloonTelemetryDropped;
        sample->errors = s_balloonTelemetryErrors;
        s_balloonTelemetryQueue[(s_balloonTelemetryHead + s_balloonTelemetryCount) %
            BALLOON_TELEMETRY_QUEUE_CAPACITY] = *sample;
        ++s_balloonTelemetryCount;
        accepted = true;
        pthread_cond_signal(&s_balloonTelemetryQueueReady);
    }
    pthread_mutex_unlock(&s_balloonTelemetryQueueMutex);
    return accepted;
}

static void BalloonTelemetry_WriteError(void)
{
    uint64_t count;
    pthread_mutex_lock(&s_balloonTelemetryQueueMutex);
    count = ++s_balloonTelemetryErrors;
    pthread_mutex_unlock(&s_balloonTelemetryQueueMutex);
    /* Visible even when the log directory is unwritable; avoid stderr flooding. */
    if (count == 1 || count % 50 == 0)
        fprintf(stderr, "background telemetry write error count=%llu errno=%d\n",
            (unsigned long long)count, errno);
}

static int BalloonTelemetry_Format(char *out, size_t size, const T_BalloonTelemetrySample *sample)
{
    size_t used = 0;
    unsigned i;
    int count;
#define BT_APPEND(...) do { count = snprintf(out + used, size - used, __VA_ARGS__); \
    if (count < 0 || (size_t)count >= size - used) { return -1; } \
    used += (size_t)count; } while (0)
    BT_APPEND("{\"schema\":\"background_telemetry_v1\",\"event\":\"telemetry_sample\","
        "\"clock_id\":\"%s\",\"session_id\":\"%s\",\"seq\":%llu,\"ts_ms\":%llu,"
        "\"sample_begin_ms\":%llu,\"sample_end_ms\":%llu,\"queue_dropped_total\":%llu,"
        "\"write_errors_total\":%llu", s_balloonTelemetryClock, s_balloonTelemetrySession,
        (unsigned long long)sample->seq, (unsigned long long)sample->end_ms,
        (unsigned long long)sample->begin_ms, (unsigned long long)sample->end_ms,
        (unsigned long long)sample->dropped, (unsigned long long)sample->errors);
    for (i = 0; i < BALLOON_TELEMETRY_TOPIC_COUNT; ++i) {
        const T_BalloonTelemetryMeta *m = &sample->meta[i];
        BT_APPEND(",\"%s\":{\"code\":%llu,\"valid\":%s,\"fresh\":%s,\"new_sample\":%s,"
            "\"host_read_begin_ms\":%llu,\"host_read_end_ms\":%llu,\"last_progress_ms\":%llu,"
            "\"sample_age_ms\":", s_balloonTelemetryNames[i], (unsigned long long)m->code,
            m->valid ? "true" : "false", m->fresh ? "true" : "false", m->new_sample ? "true" : "false",
            (unsigned long long)m->begin_ms, (unsigned long long)m->end_ms,
            (unsigned long long)m->last_progress_ms);
        if (m->have_progress) BT_APPEND("%llu", (unsigned long long)m->age_ms);
        else BT_APPEND("null");
        BT_APPEND(",\"sdk_millisecond_raw\":%u,\"sdk_microsecond_raw\":%u",
            (unsigned)m->stamp.millisecond, (unsigned)m->stamp.microsecond);
        if (i == 0) {
            if (m->valid) BT_APPEND(",\"n_mps\":%.7g,\"e_mps\":%.7g,\"u_mps\":%.7g,\"health\":%u",
                sample->velocity.data.x, sample->velocity.data.y, sample->velocity.data.z,
                (unsigned)sample->velocity.health);
            else BT_APPEND(",\"n_mps\":null,\"e_mps\":null,\"u_mps\":null,\"health\":null");
        } else if (i == 1) {
            if (m->valid) BT_APPEND(",\"w\":%.8g,\"x\":%.8g,\"y\":%.8g,\"z\":%.8g,"
                "\"roll_deg\":%.7g,\"pitch_deg\":%.7g,\"yaw_deg\":%.7g",
                sample->quaternion.q0, sample->quaternion.q1, sample->quaternion.q2,
                sample->quaternion.q3, sample->roll, sample->pitch, sample->yaw);
            else BT_APPEND(",\"w\":null,\"x\":null,\"y\":null,\"z\":null,"
                "\"roll_deg\":null,\"pitch_deg\":null,\"yaw_deg\":null");
        } else if (i == 2) {
            BT_APPEND(",\"frame\":\"NED\"");
            if (m->valid) BT_APPEND(",\"x_radps\":%.7g,\"y_radps\":%.7g,\"z_radps\":%.7g",
                sample->angular_rate.x, sample->angular_rate.y, sample->angular_rate.z);
            else BT_APPEND(",\"x_radps\":null,\"y_radps\":null,\"z_radps\":null");
        } else if (i == 4) {
            if (m->valid) BT_APPEND(",\"control_mode\":%u,\"device_status\":%u,\"flight_status\":%u,\"vrc_status\":%u",
                (unsigned)sample->control_device.controlMode, (unsigned)sample->control_device.deviceStatus,
                (unsigned)sample->control_device.flightStatus, (unsigned)sample->control_device.vrcStatus);
            else BT_APPEND(",\"control_mode\":null,\"device_status\":null,\"flight_status\":null,\"vrc_status\":null");
        } else {
            if (m->valid) BT_APPEND(",\"value\":%u", (unsigned)(i == 3 ? sample->flight_status : sample->display_mode));
            else BT_APPEND(",\"value\":null");
        }
        BT_APPEND("}");
    }
    BT_APPEND("}\n");
#undef BT_APPEND
    return (int)used;
}

static void BalloonTelemetry_Path(char *path, size_t size, unsigned index)
{
    if (index) snprintf(path, size, "%s/background_telemetry.%u.jsonl", BALLOON_TELEMETRY_DIRECTORY, index);
    else snprintf(path, size, "%s/background_telemetry.jsonl", BALLOON_TELEMETRY_DIRECTORY);
}

static bool BalloonTelemetry_Rotate(void)
{
    char source[512], target[512];
    unsigned i;
    BalloonTelemetry_Path(target, sizeof(target), 3);
    if (remove(target) != 0 && errno != ENOENT) return false;
    for (i = 3; i > 0; --i) {
        BalloonTelemetry_Path(source, sizeof(source), i - 1);
        BalloonTelemetry_Path(target, sizeof(target), i);
        if (rename(source, target) != 0 && errno != ENOENT) return false;
    }
    return true;
}

static bool BalloonTelemetry_Write(FILE **stream, size_t *bytes, const char *line, size_t length)
{
    char path[512];
    if (length > BALLOON_TELEMETRY_ROTATE_BYTES) return false;
    if (*stream == NULL) {
        long position;
        BalloonTelemetry_Path(path, sizeof(path), 0);
        *stream = fopen(path, "ab+");
        if (*stream == NULL) return false;
        if (fseek(*stream, 0, SEEK_END) != 0 || (position = ftell(*stream)) < 0) goto fail;
        *bytes = (size_t)position;
    }
    if (*bytes + length > BALLOON_TELEMETRY_ROTATE_BYTES) {
        if (fclose(*stream) != 0) { *stream = NULL; return false; }
        *stream = NULL;
        if (!BalloonTelemetry_Rotate()) return false;
        BalloonTelemetry_Path(path, sizeof(path), 0);
        *stream = fopen(path, "ab");
        if (*stream == NULL) return false;
        *bytes = 0;
    }
    if (fwrite(line, 1, length, *stream) != length) goto fail;
    *bytes += length;
    return true;
fail:
    fclose(*stream);
    *stream = NULL;
    return false;
}

static void *BalloonTelemetry_Writer(void *unused)
{
    FILE *stream = NULL;
    size_t bytes = 0;
    uint64_t last_flush = 0;
    (void)unused;
    for (;;) {
        T_BalloonTelemetrySample sample;
        char line[8192];
        int length;
        pthread_mutex_lock(&s_balloonTelemetryQueueMutex);
        while (s_balloonTelemetryCount == 0)
            pthread_cond_wait(&s_balloonTelemetryQueueReady, &s_balloonTelemetryQueueMutex);
        sample = s_balloonTelemetryQueue[s_balloonTelemetryHead];
        s_balloonTelemetryHead = (s_balloonTelemetryHead + 1) % BALLOON_TELEMETRY_QUEUE_CAPACITY;
        --s_balloonTelemetryCount;
        pthread_mutex_unlock(&s_balloonTelemetryQueueMutex);
        length = BalloonTelemetry_Format(line, sizeof(line), &sample);
        if (length < 0 || !BalloonTelemetry_Write(&stream, &bytes, line, (size_t)length)) {
            BalloonTelemetry_WriteError();
            usleep(20000); /* Writer failures cannot create a busy loop. */
            continue;
        }
        if (sample.end_ms >= last_flush + 1000 || sample.end_ms < last_flush) {
            if (fflush(stream) != 0) {
                BalloonTelemetry_WriteError();
                fclose(stream);
                stream = NULL;
            }
            last_flush = sample.end_ms;
        }
    }
    return NULL;
}

static void *BalloonTelemetry_Subscriptions(void *unused)
{
    unsigned i;
    (void)unused;
    for (;;) {
        for (i = 0; i < BALLOON_TELEMETRY_TOPIC_COUNT; ++i) {
            T_DjiReturnCode code;
            bool confirmed_missing;
            pthread_mutex_lock(&s_balloonTelemetrySubscriptionMutex);
            code = s_balloonTelemetrySubscriptionCodes[i];
            confirmed_missing = s_balloonTelemetryMissingTopics[i];
            s_balloonTelemetryMissingTopics[i] = false;
            pthread_mutex_unlock(&s_balloonTelemetrySubscriptionMutex);
            if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                if (confirmed_missing)
                    BalloonSubscription_ForgetMissing(s_balloonTelemetryTopics[i]);
                code = BalloonSubscription_Acquire(BALLOON_SUB_BACKGROUND,
                    s_balloonTelemetryTopics[i], i < 3 ? DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ :
                    DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, NULL);
                pthread_mutex_lock(&s_balloonTelemetrySubscriptionMutex);
                s_balloonTelemetrySubscriptionCodes[i] = code;
                pthread_mutex_unlock(&s_balloonTelemetrySubscriptionMutex);
            }
        }
        /* Subscribe can block for 1.2 s; keep all retries off the sampler. */
        usleep(2000000);
    }
    return NULL;
}

static void *BalloonTelemetry_Sampler(void *unused)
{
    T_BalloonTelemetryProgress progress[BALLOON_TELEMETRY_TOPIC_COUNT] = {0};
    T_DjiReturnCode subscriptions[BALLOON_TELEMETRY_TOPIC_COUNT];
    uint64_t seq = 0;
    unsigned i;
    (void)unused;
    for (;;) {
        uint64_t begin = BalloonClock_NowMs(), end;
        T_BalloonTelemetrySample sample = {0};
        pthread_mutex_lock(&s_balloonTelemetrySubscriptionMutex);
        memcpy(subscriptions, s_balloonTelemetrySubscriptionCodes, sizeof(subscriptions));
        pthread_mutex_unlock(&s_balloonTelemetrySubscriptionMutex);
        sample.seq = ++seq;
        BalloonTelemetry_Read(&sample, progress, subscriptions);
        for (i = 0; i < BALLOON_TELEMETRY_TOPIC_COUNT; ++i) {
            if (subscriptions[i] == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                sample.meta[i].code == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_NOT_SUBSCRIBED) {
                /* Confirmed missing subscription: retry on the next backoff
                 * interval, without ever resetting the whole SDK module. */
                pthread_mutex_lock(&s_balloonTelemetrySubscriptionMutex);
                s_balloonTelemetrySubscriptionCodes[i] = sample.meta[i].code;
                s_balloonTelemetryMissingTopics[i] = true;
                pthread_mutex_unlock(&s_balloonTelemetrySubscriptionMutex);
                memset(&progress[i], 0, sizeof(progress[i]));
            }
        }
        (void)BalloonTelemetry_Enqueue(&sample);
        end = BalloonClock_NowMs();
        if (end >= begin && end - begin < 20) usleep((unsigned)(20 - (end - begin)) * 1000);
        else usleep(1000);
    }
    return NULL;
}

T_DjiReturnCode BalloonTelemetry_Start(void)
{
    pthread_t thread;
    unsigned i;
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    pthread_mutex_lock(&s_balloonTelemetryStartMutex);
    if (s_balloonTelemetryStarted) goto done;
    if (!s_balloonTelemetryWriterStarted) {
        /* Empty/invalid clock identity cannot be mixed into a valid boot. */
        if (!BalloonClock_Id(s_balloonTelemetryClock)) strcpy(s_balloonTelemetryClock, "unavailable");
        snprintf(s_balloonTelemetrySession, sizeof(s_balloonTelemetrySession), "%s:%ld:%llu",
            s_balloonTelemetryClock, (long)getpid(), (unsigned long long)BalloonClock_NowMs());
        if (pthread_create(&thread, NULL, BalloonTelemetry_Writer, NULL) != 0) {
            code = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
            goto done;
        }
        pthread_detach(thread);
        s_balloonTelemetryWriterStarted = true;
    }
    if (!s_balloonTelemetrySamplerStarted) {
        for (i = 0; i < BALLOON_TELEMETRY_TOPIC_COUNT; ++i)
            s_balloonTelemetrySubscriptionCodes[i] = DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_NOT_SUBSCRIBED;
        if (pthread_create(&thread, NULL, BalloonTelemetry_Sampler, NULL) != 0) {
            code = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
            goto done;
        }
        pthread_detach(thread);
        s_balloonTelemetrySamplerStarted = true;
    }
    if (pthread_create(&thread, NULL, BalloonTelemetry_Subscriptions, NULL) != 0)
        code = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    else {
        pthread_detach(thread);
        s_balloonTelemetryStarted = true;
    }
done:
    pthread_mutex_unlock(&s_balloonTelemetryStartMutex);
    return code;
}

#endif
