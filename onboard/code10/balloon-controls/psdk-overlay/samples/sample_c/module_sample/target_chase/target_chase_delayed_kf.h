#ifndef TARGET_CHASE_DELAYED_KF_H
#define TARGET_CHASE_DELAYED_KF_H

/*
 * Delayed world-bearing Kalman filter. No SDK calls, clocks or flight commands.
 * The caller must supply calibrated measurement times and historical camera
 * poses in the same monotonic clock, and reset on target/clock/mode changes.
 *
 * Each axis is [angle, angular rate], in degrees and seconds. Measurements are
 * replayed in measurement-time order, not receipt order. A folded posterior
 * anchors the bounded history; eviction never promotes a rejected observation
 * to a fresh initialization. Queries propagate a copy of the last accepted
 * posterior, so repeated queries cannot integrate elapsed time twice.
 */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TARGET_DELAYED_KF_CAPACITY 64U
#define TARGET_DELAYED_KF_LAG_MS UINT64_C(1500)
#define TARGET_DELAYED_KF_PROCESS_NOISE 4.0
#define TARGET_DELAYED_KF_MEASUREMENT_VARIANCE 0.25
#define TARGET_DELAYED_KF_NIS_LIMIT 25.0

typedef enum {
    TARGET_DELAYED_KF_EMPTY = 0,
    TARGET_DELAYED_KF_ACCEPTED,
    TARGET_DELAYED_KF_REJECTED_NONFINITE,
    TARGET_DELAYED_KF_REJECTED_FUTURE,
    TARGET_DELAYED_KF_REJECTED_AGE,
    TARGET_DELAYED_KF_REJECTED_ARRIVAL,
    TARGET_DELAYED_KF_REJECTED_HISTORY,
    TARGET_DELAYED_KF_REJECTED_DUPLICATE,
    TARGET_DELAYED_KF_REJECTED_NIS
} E_TargetDelayedKfStatus;

typedef struct {
    double angle;
    double rate;
    double p00;
    double p01;
    double p11;
} T_TargetDelayedKfAxis;

typedef struct {
    uint64_t measurement_ms;
    float yaw_deg;
    float pitch_deg;
    bool accepted;
} T_TargetDelayedKfObservation;

typedef struct {
    float yaw_deg;
    float pitch_deg;
    float yaw_rate_degps;
    float pitch_rate_degps;
    float yaw_variance_deg2;
    float pitch_variance_deg2;
    uint64_t age_ms;
    /* Includes accepted observations folded into the anchor; saturates. */
    uint32_t accepted_samples;
} T_TargetDelayedKfPrediction;

typedef struct {
    T_TargetDelayedKfObservation history[TARGET_DELAYED_KF_CAPACITY];
    size_t history_count;
    T_TargetDelayedKfAxis yaw;
    T_TargetDelayedKfAxis pitch;
    uint64_t posterior_ms;
    uint32_t accepted_samples;
    bool has_posterior;

    T_TargetDelayedKfAxis anchor_yaw;
    T_TargetDelayedKfAxis anchor_pitch;
    uint64_t anchor_ms;
    uint32_t anchor_accepted_samples;
    bool anchor_valid;
    uint64_t closed_before_ms;
    bool history_closed;

    uint64_t last_received_ms;
    bool has_arrival;
    /* Rejected calls, not repeated counts of an outlier during replay. */
    uint32_t rejected_inputs;
    uint32_t rejected_in_window;
    E_TargetDelayedKfStatus status;
} T_TargetDelayedKf;

static inline uint32_t TargetDelayedKf_Increment(uint32_t value)
{
    return value == UINT32_MAX ? value : value + 1U;
}

static inline double TargetDelayedKf_Wrap(double degrees)
{
    double value = fmod(degrees + 180.0, 360.0);
    if (value < 0.0) {
        value += 360.0;
    }
    return value - 180.0;
}

static inline void TargetDelayedKf_Reset(T_TargetDelayedKf *filter)
{
    if (filter != NULL) {
        memset(filter, 0, sizeof(*filter));
    }
}

static inline bool TargetDelayedKf_Reject(T_TargetDelayedKf *filter,
                                        E_TargetDelayedKfStatus status)
{
    filter->status = status;
    filter->rejected_inputs = TargetDelayedKf_Increment(filter->rejected_inputs);
    return false;
}

static inline T_TargetDelayedKfAxis TargetDelayedKf_SeedAxis(double angle)
{
    T_TargetDelayedKfAxis axis;
    axis.angle = angle;
    axis.rate = 0.0;
    axis.p00 = TARGET_DELAYED_KF_MEASUREMENT_VARIANCE;
    axis.p01 = 0.0;
    axis.p11 = 100.0;
    return axis;
}

static inline void TargetDelayedKf_Propagate(T_TargetDelayedKfAxis *axis, double dt)
{
    const double q = TARGET_DELAYED_KF_PROCESS_NOISE;
    axis->angle += axis->rate * dt;
    axis->p00 += 2.0 * dt * axis->p01 + dt * dt * axis->p11 + q * dt * dt * dt / 3.0;
    axis->p01 += dt * axis->p11 + q * dt * dt / 2.0;
    axis->p11 += q * dt;
}

static inline void TargetDelayedKf_Correct(T_TargetDelayedKfAxis *axis, double residual)
{
    const double variance = TARGET_DELAYED_KF_MEASUREMENT_VARIANCE;
    const double s = axis->p00 + variance;
    const double old_p01 = axis->p01;
    axis->angle += axis->p00 / s * residual;
    axis->rate += old_p01 / s * residual;
    axis->p00 = axis->p00 * variance / s;
    axis->p01 = old_p01 * variance / s;
    axis->p11 = fmax(1e-12, axis->p11 - old_p01 * old_p01 / s);
}

/* Rejecting an update leaves both posterior axes and their time untouched. */
static inline bool TargetDelayedKf_Update(T_TargetDelayedKfAxis *yaw,
                                        T_TargetDelayedKfAxis *pitch,
                                        uint64_t posterior_ms,
                                        const T_TargetDelayedKfObservation *observation)
{
    T_TargetDelayedKfAxis next_yaw = *yaw;
    T_TargetDelayedKfAxis next_pitch = *pitch;
    const double dt = (double)(observation->measurement_ms - posterior_ms) / 1000.0;
    double yaw_residual;
    double pitch_residual;
    double nis;
    TargetDelayedKf_Propagate(&next_yaw, dt);
    TargetDelayedKf_Propagate(&next_pitch, dt);
    yaw_residual = TargetDelayedKf_Wrap((double)observation->yaw_deg - next_yaw.angle);
    pitch_residual = (double)observation->pitch_deg - next_pitch.angle;
    nis = yaw_residual * yaw_residual /
              (next_yaw.p00 + TARGET_DELAYED_KF_MEASUREMENT_VARIANCE) +
          pitch_residual * pitch_residual /
              (next_pitch.p00 + TARGET_DELAYED_KF_MEASUREMENT_VARIANCE);
    if (!isfinite(nis) || nis > TARGET_DELAYED_KF_NIS_LIMIT) {
        return false;
    }
    TargetDelayedKf_Correct(&next_yaw, yaw_residual);
    TargetDelayedKf_Correct(&next_pitch, pitch_residual);
    next_yaw.angle = TargetDelayedKf_Wrap(next_yaw.angle);
    *yaw = next_yaw;
    *pitch = next_pitch;
    return true;
}

/*
 * Returns whether this measurement is accepted after chronological replay.
 * A NIS-rejected raw measurement remains in the bounded history for replay,
 * but never refreshes posterior_ms. Arrival time advances for a valid packet
 * even when NIS rejects it. Structural rejections do not change arrival time.
 */
static inline bool TargetDelayedKf_Observe(T_TargetDelayedKf *filter,
                                         uint64_t measurement_ms,
                                         uint64_t received_ms,
                                         float yaw_deg,
                                         float pitch_deg)
{
    T_TargetDelayedKfObservation ordered[TARGET_DELAYED_KF_CAPACITY + 1U];
    T_TargetDelayedKfAxis yaw;
    T_TargetDelayedKfAxis pitch;
    uint64_t posterior_ms;
    uint32_t accepted_samples;
    size_t position = 0U;
    size_t count;
    size_t drop_count = 0U;
    size_t i;
    bool has_posterior;
    bool incoming_accepted = false;

    if (filter == NULL) {
        return false;
    }
    if (!isfinite(yaw_deg) || !isfinite(pitch_deg)) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_NONFINITE);
    }
    if (measurement_ms > received_ms) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_FUTURE);
    }
    if (received_ms - measurement_ms > TARGET_DELAYED_KF_LAG_MS) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_AGE);
    }
    if (filter->has_arrival && received_ms < filter->last_received_ms) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_ARRIVAL);
    }
    if (filter->history_closed && measurement_ms <= filter->closed_before_ms) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_HISTORY);
    }
    while (position < filter->history_count &&
           filter->history[position].measurement_ms < measurement_ms) {
        ++position;
    }
    if (position < filter->history_count &&
        filter->history[position].measurement_ms == measurement_ms) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_DUPLICATE);
    }
    count = filter->history_count + 1U;
    for (i = 0U; i < position; ++i) {
        ordered[i] = filter->history[i];
    }
    ordered[position].measurement_ms = measurement_ms;
    ordered[position].yaw_deg = yaw_deg;
    ordered[position].pitch_deg = pitch_deg;
    ordered[position].accepted = false;
    for (i = position; i < filter->history_count; ++i) {
        ordered[i + 1U] = filter->history[i];
    }
    while (drop_count < count &&
           (count - drop_count > TARGET_DELAYED_KF_CAPACITY ||
            received_ms - ordered[drop_count].measurement_ms > TARGET_DELAYED_KF_LAG_MS)) {
        ++drop_count;
    }

    yaw = filter->anchor_yaw;
    pitch = filter->anchor_pitch;
    posterior_ms = filter->anchor_ms;
    accepted_samples = filter->anchor_accepted_samples;
    has_posterior = filter->anchor_valid;
    filter->rejected_in_window = 0U;
    for (i = 0U; i < count; ++i) {
        bool accepted;
        if (!has_posterior) {
            yaw = TargetDelayedKf_SeedAxis(TargetDelayedKf_Wrap((double)ordered[i].yaw_deg));
            pitch = TargetDelayedKf_SeedAxis((double)ordered[i].pitch_deg);
            has_posterior = true;
            accepted = true;
        } else {
            accepted = TargetDelayedKf_Update(&yaw, &pitch, posterior_ms, &ordered[i]);
        }
        ordered[i].accepted = accepted;
        if (accepted) {
            posterior_ms = ordered[i].measurement_ms;
            accepted_samples = TargetDelayedKf_Increment(accepted_samples);
        }
        if (i == position) {
            incoming_accepted = accepted;
        }
        if (i < drop_count) {
            filter->anchor_yaw = yaw;
            filter->anchor_pitch = pitch;
            filter->anchor_ms = posterior_ms;
            filter->anchor_accepted_samples = accepted_samples;
            filter->anchor_valid = has_posterior;
            filter->closed_before_ms = ordered[i].measurement_ms;
            filter->history_closed = true;
        } else {
            filter->history[i - drop_count] = ordered[i];
            if (!accepted) {
                ++filter->rejected_in_window;
            }
        }
    }
    filter->history_count = count - drop_count;
    filter->yaw = yaw;
    filter->pitch = pitch;
    filter->posterior_ms = posterior_ms;
    filter->accepted_samples = accepted_samples;
    filter->has_posterior = has_posterior;
    filter->last_received_ms = received_ms;
    filter->has_arrival = true;
    if (!incoming_accepted) {
        return TargetDelayedKf_Reject(filter, TARGET_DELAYED_KF_REJECTED_NIS);
    }
    filter->status = TARGET_DELAYED_KF_ACCEPTED;
    return true;
}

static inline bool TargetDelayedKf_FitsFloat(double value)
{
    return isfinite(value) && value >= -(double)FLT_MAX && value <= (double)FLT_MAX;
}

static inline bool TargetDelayedKf_Predict(const T_TargetDelayedKf *filter,
                                         uint64_t now_ms,
                                         float lead_s,
                                         uint64_t max_age_ms,
                                         T_TargetDelayedKfPrediction *prediction)
{
    T_TargetDelayedKfAxis yaw;
    T_TargetDelayedKfAxis pitch;
    uint64_t age_ms;
    double dt;
    if (prediction == NULL) {
        return false;
    }
    memset(prediction, 0, sizeof(*prediction));
    if (filter == NULL || !filter->has_posterior || !isfinite(lead_s) ||
        lead_s < 0.0f || lead_s > 0.5f || now_ms < filter->last_received_ms ||
        now_ms < filter->posterior_ms) {
        return false;
    }
    age_ms = now_ms - filter->posterior_ms;
    if (age_ms > max_age_ms) {
        return false;
    }
    dt = (double)age_ms / 1000.0 + (double)lead_s;
    yaw = filter->yaw;
    pitch = filter->pitch;
    TargetDelayedKf_Propagate(&yaw, dt);
    TargetDelayedKf_Propagate(&pitch, dt);
    yaw.angle = TargetDelayedKf_Wrap(yaw.angle);
    if (!TargetDelayedKf_FitsFloat(yaw.angle) || !TargetDelayedKf_FitsFloat(pitch.angle) ||
        !TargetDelayedKf_FitsFloat(yaw.rate) || !TargetDelayedKf_FitsFloat(pitch.rate) ||
        !TargetDelayedKf_FitsFloat(yaw.p00) || !TargetDelayedKf_FitsFloat(pitch.p00)) {
        return false;
    }
    prediction->yaw_deg = (float)yaw.angle;
    prediction->pitch_deg = (float)pitch.angle;
    prediction->yaw_rate_degps = (float)yaw.rate;
    prediction->pitch_rate_degps = (float)pitch.rate;
    prediction->yaw_variance_deg2 = (float)yaw.p00;
    prediction->pitch_variance_deg2 = (float)pitch.p00;
    prediction->age_ms = age_ms;
    prediction->accepted_samples = filter->accepted_samples;
    return true;
}

#endif /* TARGET_CHASE_DELAYED_KF_H */
