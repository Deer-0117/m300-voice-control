#ifndef TARGET_CHASE_TIMING_ALIGNMENT_H
#define TARGET_CHASE_TIMING_ALIGNMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

/* Calibration describes two separately verified physical measurement delays.
 * A video/pose relative phase fit alone does not establish either absolute
 * delay and must not set verified=true. All stamps below use this boot's host
 * monotonic clock. SDK raw stamp fields are deliberately not reinterpreted. */
typedef struct {
    bool enabled;
    bool verified;
    char clock_id[80];
    char calibration_id[80];
    double video_delay_ms;
    double pose_delay_ms;
    double uncertainty_ms;
    uint64_t valid_from_ms;
    uint64_t valid_until_ms;
} T_TargetTimingAlignment;

typedef enum {
    TARGET_TIMING_OK = 0,
    TARGET_TIMING_DISABLED,
    TARGET_TIMING_UNVERIFIED,
    TARGET_TIMING_INVALID_CLOCK,
    TARGET_TIMING_CLOCK_MISMATCH,
    TARGET_TIMING_INVALID_ID,
    TARGET_TIMING_INVALID_DELAY,
    TARGET_TIMING_UNCERTAIN,
    TARGET_TIMING_INVALID_WINDOW,
    TARGET_TIMING_OUTSIDE_WINDOW,
    TARGET_TIMING_INVALID_TIMESTAMP
} E_TargetTimingReason;

typedef struct {
    uint64_t timestamp_ms; /* mapped physical pose measurement time */
    uint64_t received_ms;  /* host time this sample became available */
    float yaw_deg;
    float pitch_deg;
} T_TargetTimingPose;

static inline const char *TargetTiming_ReasonName(E_TargetTimingReason reason)
{
    switch (reason) {
    case TARGET_TIMING_OK: return "aligned";
    case TARGET_TIMING_DISABLED: return "timing_disabled";
    case TARGET_TIMING_UNVERIFIED: return "timing_unverified";
    case TARGET_TIMING_INVALID_CLOCK: return "timing_invalid_clock";
    case TARGET_TIMING_CLOCK_MISMATCH: return "timing_clock_mismatch";
    case TARGET_TIMING_INVALID_ID: return "timing_invalid_calibration_id";
    case TARGET_TIMING_INVALID_DELAY: return "timing_invalid_delay";
    case TARGET_TIMING_UNCERTAIN: return "timing_uncertain";
    case TARGET_TIMING_INVALID_WINDOW: return "timing_invalid_window";
    case TARGET_TIMING_OUTSIDE_WINDOW: return "timing_outside_window";
    case TARGET_TIMING_INVALID_TIMESTAMP: return "timing_invalid_timestamp";
    default: return "timing_unknown";
    }
}

static inline bool TargetTiming_ClockValid(const char *clock_id)
{
    size_t i;
    if (clock_id == NULL) return false;
    /* mono-v1:<boot UUID>; accepting arbitrary prefixes permits stale files
     * to be mistaken for a same-boot calibration. */
    for (i = 0; i < 44U; ++i) {
        char c = clock_id[i];
        if (i < 8U) {
            if (c != "mono-v1:"[i]) return false;
        } else if (i == 16U || i == 21U || i == 26U || i == 31U) {
            if (c != '-') return false;
        } else if (!((c >= '0' && c <= '9') ||
                     (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
            return false;
        }
    }
    return clock_id[44] == '\0';
}

static inline E_TargetTimingReason TargetTiming_Validate(
    const T_TargetTimingAlignment *alignment, const char *current_clock,
    uint64_t now_ms)
{
    if (alignment == NULL || !alignment->enabled) return TARGET_TIMING_DISABLED;
    if (!alignment->verified) return TARGET_TIMING_UNVERIFIED;
    if (!TargetTiming_ClockValid(alignment->clock_id) ||
        !TargetTiming_ClockValid(current_clock)) return TARGET_TIMING_INVALID_CLOCK;
    if (strcmp(alignment->clock_id, current_clock) != 0)
        return TARGET_TIMING_CLOCK_MISMATCH;
    if (alignment->calibration_id[0] == '\0' ||
        memchr(alignment->calibration_id, '\0', sizeof(alignment->calibration_id)) == NULL)
        return TARGET_TIMING_INVALID_ID;
    if (!isfinite(alignment->video_delay_ms) || !isfinite(alignment->pose_delay_ms) ||
        alignment->video_delay_ms < 0.0 || alignment->video_delay_ms > 1500.0 ||
        alignment->pose_delay_ms < 0.0 || alignment->pose_delay_ms > 1500.0)
        return TARGET_TIMING_INVALID_DELAY;
    if (!isfinite(alignment->uncertainty_ms) || alignment->uncertainty_ms < 0.0 ||
        alignment->uncertainty_ms > 25.0) return TARGET_TIMING_UNCERTAIN;
    if (alignment->valid_from_ms == 0 ||
        alignment->valid_until_ms <= alignment->valid_from_ms)
        return TARGET_TIMING_INVALID_WINDOW;
    if (now_ms < alignment->valid_from_ms || now_ms > alignment->valid_until_ms)
        return TARGET_TIMING_OUTSIDE_WINDOW;
    return TARGET_TIMING_OK;
}

static inline E_TargetTimingReason TargetTiming_MapDelay(
    const T_TargetTimingAlignment *alignment, const char *current_clock,
    uint64_t host_stamp_ms, uint64_t received_ms, uint64_t now_ms,
    bool video, uint64_t *measurement_ms)
{
    uint64_t delay_ms, mapped_ms;
    E_TargetTimingReason reason;
    if (measurement_ms == NULL) return TARGET_TIMING_INVALID_TIMESTAMP;
    *measurement_ms = 0;
    reason = TargetTiming_Validate(alignment, current_clock, now_ms);
    if (reason != TARGET_TIMING_OK) return reason;
    if (host_stamp_ms == 0 || received_ms < host_stamp_ms || received_ms > now_ms)
        return TARGET_TIMING_INVALID_TIMESTAMP;
    /* The estimator uses integer milliseconds; round only the bounded delay,
     * never a large absolute timestamp. Quantization is at most 0.5 ms. */
    delay_ms = (uint64_t)floor((video ? alignment->video_delay_ms :
                              alignment->pose_delay_ms) + 0.5);
    if (host_stamp_ms <= delay_ms) return TARGET_TIMING_INVALID_TIMESTAMP;
    mapped_ms = host_stamp_ms - delay_ms;
    if (mapped_ms < alignment->valid_from_ms || mapped_ms > alignment->valid_until_ms)
        return TARGET_TIMING_OUTSIDE_WINDOW;
    *measurement_ms = mapped_ms;
    return TARGET_TIMING_OK;
}

static inline E_TargetTimingReason TargetTiming_MapVideo(
    const T_TargetTimingAlignment *alignment, const char *current_clock,
    uint64_t host_stamp_ms, uint64_t received_ms, uint64_t now_ms,
    uint64_t *measurement_ms)
{
    return TargetTiming_MapDelay(alignment, current_clock, host_stamp_ms,
                                received_ms, now_ms, true, measurement_ms);
}

static inline E_TargetTimingReason TargetTiming_MapPose(
    const T_TargetTimingAlignment *alignment, const char *current_clock,
    uint64_t host_stamp_ms, uint64_t received_ms, uint64_t now_ms,
    uint64_t *measurement_ms)
{
    return TargetTiming_MapDelay(alignment, current_clock, host_stamp_ms,
                                received_ms, now_ms, false, measurement_ms);
}

static inline float TargetTiming_Wrap180(float angle)
{
    float wrapped = fmodf(angle, 360.0f);
    if (wrapped > 180.0f) wrapped -= 360.0f;
    if (wrapped < -180.0f) wrapped += 360.0f;
    return wrapped;
}

/* History must be cleared when boot/calibration changes. Samples already carry
 * mapped measurement times. There is no nearest-pose fallback: only an exact
 * sample or an observed bracket within 100 ms on each side is accepted. */
static inline bool TargetTiming_PoseAt(
    const T_TargetTimingPose *poses, size_t count, uint64_t measurement_ms,
    uint64_t now_ms, float *yaw_deg, float *pitch_deg)
{
    const T_TargetTimingPose *before = NULL, *after = NULL;
    size_t i;
    float fraction;
    if (poses == NULL || yaw_deg == NULL || pitch_deg == NULL ||
        measurement_ms == 0 || measurement_ms > now_ms) return false;
    for (i = 0; i < count; ++i) {
        const T_TargetTimingPose *pose = &poses[i];
        if (pose->timestamp_ms == 0 || pose->received_ms < pose->timestamp_ms ||
            pose->received_ms > now_ms || !isfinite(pose->yaw_deg) ||
            !isfinite(pose->pitch_deg)) continue;
        if (pose->timestamp_ms <= measurement_ms &&
            (before == NULL || pose->timestamp_ms > before->timestamp_ms)) before = pose;
        if (pose->timestamp_ms >= measurement_ms &&
            (after == NULL || pose->timestamp_ms < after->timestamp_ms)) after = pose;
    }
    if (before == NULL || after == NULL ||
        measurement_ms - before->timestamp_ms > 100U ||
        after->timestamp_ms - measurement_ms > 100U) return false;
    fraction = after->timestamp_ms == before->timestamp_ms ? 0.0f :
        (float)(measurement_ms - before->timestamp_ms) /
        (float)(after->timestamp_ms - before->timestamp_ms);
    *yaw_deg = TargetTiming_Wrap180(before->yaw_deg +
        fraction * TargetTiming_Wrap180(after->yaw_deg - before->yaw_deg));
    *pitch_deg = before->pitch_deg + fraction * (after->pitch_deg - before->pitch_deg);
    return isfinite(*yaw_deg) && isfinite(*pitch_deg);
}

/* The current camera attitude is distinct from its exposure-time attitude.
 * Extrapolate only the latest short, observed interval; a frozen source or a
 * large pose delay is not a license for unlimited camera-motion prediction. */
static inline bool TargetTiming_CurrentPose(
    const T_TargetTimingPose *poses, size_t count, uint64_t now_ms,
    float *yaw_deg, float *pitch_deg)
{
    const T_TargetTimingPose *latest = NULL, *previous = NULL;
    size_t i;
    float yaw, pitch, dt_s, ahead_s, yaw_rate, pitch_rate;
    if (poses == NULL || yaw_deg == NULL || pitch_deg == NULL || now_ms == 0)
        return false;
    for (i = 0; i < count; ++i) {
        const T_TargetTimingPose *pose = &poses[i];
        if (pose->timestamp_ms == 0 || pose->timestamp_ms > now_ms ||
            pose->received_ms < pose->timestamp_ms || pose->received_ms > now_ms ||
            !isfinite(pose->yaw_deg) || !isfinite(pose->pitch_deg)) continue;
        if (latest == NULL || pose->timestamp_ms > latest->timestamp_ms) latest = pose;
    }
    if (latest == NULL || now_ms-latest->timestamp_ms > 100U ||
        now_ms-latest->received_ms > 100U ||
        latest->pitch_deg < -90.0f || latest->pitch_deg > 90.0f) return false;
    yaw = TargetTiming_Wrap180(latest->yaw_deg);
    pitch = latest->pitch_deg;
    if (latest->timestamp_ms != now_ms) {
        for (i = 0; i < count; ++i) {
            const T_TargetTimingPose *pose = &poses[i];
            if (pose->timestamp_ms == 0 || pose->timestamp_ms >= latest->timestamp_ms ||
                pose->received_ms < pose->timestamp_ms || pose->received_ms > now_ms ||
                !isfinite(pose->yaw_deg) || !isfinite(pose->pitch_deg)) continue;
            if (previous == NULL || pose->timestamp_ms > previous->timestamp_ms) previous = pose;
        }
        if (previous == NULL || latest->timestamp_ms-previous->timestamp_ms > 100U ||
            previous->pitch_deg < -90.0f || previous->pitch_deg > 90.0f) return false;
        dt_s = (float)(latest->timestamp_ms-previous->timestamp_ms)*0.001f;
        ahead_s = (float)(now_ms-latest->timestamp_ms)*0.001f;
        yaw_rate = TargetTiming_Wrap180(yaw-TargetTiming_Wrap180(previous->yaw_deg))/dt_s;
        pitch_rate = (pitch-previous->pitch_deg)/dt_s;
        if (!isfinite(yaw_rate) || !isfinite(pitch_rate) ||
            fabsf(yaw_rate) > 60.0f || fabsf(pitch_rate) > 40.0f) return false;
        yaw = TargetTiming_Wrap180(yaw+yaw_rate*ahead_s);
        pitch += pitch_rate*ahead_s;
    }
    if (!isfinite(yaw) || !isfinite(pitch) || pitch < -90.0f || pitch > 90.0f)
        return false;
    *yaw_deg = yaw;
    *pitch_deg = pitch;
    return true;
}

#endif /* TARGET_CHASE_TIMING_ALIGNMENT_H */
