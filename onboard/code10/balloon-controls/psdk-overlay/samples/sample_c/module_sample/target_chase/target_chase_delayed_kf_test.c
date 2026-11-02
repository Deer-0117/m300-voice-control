#include "target_chase_delayed_kf.h"

#include <assert.h>
#include <stdio.h>

static void assert_close(double actual, double expected, double tolerance)
{
    if (!isfinite(actual) || fabs(actual - expected) > tolerance) {
        fprintf(stderr, "expected %.12g, got %.12g (tolerance %.12g)\n",
                expected, actual, tolerance);
        assert(false);
    }
}

static void assert_same_prediction(const T_TargetDelayedKfPrediction *a,
                                   const T_TargetDelayedKfPrediction *b)
{
    assert_close(a->yaw_deg, b->yaw_deg, 1e-6);
    assert_close(a->pitch_deg, b->pitch_deg, 1e-6);
    assert_close(a->yaw_rate_degps, b->yaw_rate_degps, 1e-6);
    assert_close(a->pitch_rate_degps, b->pitch_rate_degps, 1e-6);
    assert_close(a->yaw_variance_deg2, b->yaw_variance_deg2, 1e-6);
    assert_close(a->pitch_variance_deg2, b->pitch_variance_deg2, 1e-6);
    assert(a->age_ms == b->age_ms);
    assert(a->accepted_samples == b->accepted_samples);
}

static void observe_line(T_TargetDelayedKf *filter, unsigned int index, uint64_t received)
{
    const uint64_t measurement = UINT64_C(1000) + UINT64_C(100) * index;
    const float dt = (float)index / 10.0f;
    assert(TargetDelayedKf_Observe(filter, measurement, received,
                                  20.0f + 8.0f * dt, -3.0f + 2.0f * dt));
}

static void test_two_sample_solution(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKfPrediction prediction;
    const double prior_angle_variance = 0.25 + 100.0 * 0.01 + 4.0 * 0.001 / 3.0;
    const double innovation_variance = prior_angle_variance + 0.25;
    const double covariance = 100.0 * 0.1 + 4.0 * 0.01 / 2.0;
    TargetDelayedKf_Reset(&filter);
    assert(TargetDelayedKf_Observe(&filter, 1000U, 1020U, 10.0f, 5.0f));
    assert(TargetDelayedKf_Observe(&filter, 1100U, 1120U, 11.0f, 4.5f));
    assert_close(filter.yaw.angle, 10.0 + prior_angle_variance / innovation_variance, 1e-12);
    assert_close(filter.yaw.rate, covariance / innovation_variance, 1e-12);
    assert_close(filter.pitch.rate, -0.5 * covariance / innovation_variance, 1e-12);
    assert_close(filter.yaw.p00, prior_angle_variance * 0.25 / innovation_variance, 1e-12);
    assert(TargetDelayedKf_Predict(&filter, 1120U, 0.15f, 500U, &prediction));
    assert_close(prediction.yaw_deg, filter.yaw.angle + filter.yaw.rate * 0.17, 2e-6);
    assert(prediction.age_ms == 20U);
    assert(prediction.accepted_samples == 2U);
}

static void test_out_of_order_matches_ordered(void)
{
    const unsigned int shuffled[] = {6U, 0U, 9U, 2U, 1U, 8U, 3U, 5U, 4U, 7U};
    T_TargetDelayedKf ordered;
    T_TargetDelayedKf delayed;
    T_TargetDelayedKfPrediction a;
    T_TargetDelayedKfPrediction b;
    unsigned int i;
    TargetDelayedKf_Reset(&ordered);
    TargetDelayedKf_Reset(&delayed);
    for (i = 0U; i < 10U; ++i) {
        observe_line(&ordered, i, 2000U);
        observe_line(&delayed, shuffled[i], 2000U);
    }
    assert(TargetDelayedKf_Predict(&ordered, 2050U, 0.15f, 500U, &a));
    assert(TargetDelayedKf_Predict(&delayed, 2050U, 0.15f, 500U, &b));
    assert_same_prediction(&a, &b);
    assert(a.yaw_rate_degps > 7.5f && a.yaw_rate_degps < 8.5f);
}

static void test_predict_does_not_mutate_or_double_integrate(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKf before;
    T_TargetDelayedKfPrediction first;
    T_TargetDelayedKfPrediction repeated;
    T_TargetDelayedKfPrediction later;
    unsigned int i;
    TargetDelayedKf_Reset(&filter);
    for (i = 0U; i < 10U; ++i) {
        observe_line(&filter, i, 1000U + 100U * i);
    }
    before = filter;
    assert(TargetDelayedKf_Predict(&filter, 2000U, 0.15f, 500U, &first));
    assert(TargetDelayedKf_Predict(&filter, 2050U, 0.15f, 500U, &later));
    assert(TargetDelayedKf_Predict(&filter, 2000U, 0.15f, 500U, &repeated));
    assert(memcmp(&filter, &before, sizeof(filter)) == 0);
    assert_same_prediction(&first, &repeated);
    assert_close(later.yaw_deg - first.yaw_deg, 0.05 * first.yaw_rate_degps, 3e-6);
    assert(later.yaw_variance_deg2 > first.yaw_variance_deg2);
}

static void test_rejection_does_not_refresh(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKfAxis yaw_before;
    T_TargetDelayedKfAxis pitch_before;
    T_TargetDelayedKfPrediction prediction;
    unsigned int i;
    TargetDelayedKf_Reset(&filter);
    for (i = 0U; i < 5U; ++i) {
        assert(TargetDelayedKf_Observe(&filter, i * 100U, i * 100U, 0.0f, 0.0f));
    }
    yaw_before = filter.yaw;
    pitch_before = filter.pitch;
    assert(!TargetDelayedKf_Observe(&filter, 1000U, 1050U, 80.0f, 0.0f));
    assert(filter.status == TARGET_DELAYED_KF_REJECTED_NIS);
    assert(filter.posterior_ms == 400U);
    assert(filter.accepted_samples == 5U);
    assert(filter.rejected_in_window == 1U);
    assert(filter.rejected_inputs == 1U);
    assert_close(filter.yaw.angle, yaw_before.angle, 1e-12);
    assert_close(filter.yaw.p00, yaw_before.p00, 1e-12);
    assert_close(filter.pitch.p00, pitch_before.p00, 1e-12);
    assert(!TargetDelayedKf_Predict(&filter, 1050U, 0.0f, 600U, &prediction));
    assert(TargetDelayedKf_Predict(&filter, 1050U, 0.0f, 700U, &prediction));
    assert(prediction.age_ms == 650U);
    /* Joint gate must reject a pitch outlier even with plausible yaw. */
    assert(!TargetDelayedKf_Observe(&filter, 1100U, 1100U, 0.0f, 80.0f));
    assert(filter.posterior_ms == 400U);
}

static void test_wrap(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKfPrediction prediction;
    TargetDelayedKf_Reset(&filter);
    assert(TargetDelayedKf_Observe(&filter, 0U, 0U, 179.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&filter, 100U, 100U, 179.5f, 0.0f));
    assert(TargetDelayedKf_Observe(&filter, 200U, 200U, -180.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&filter, 300U, 300U, -179.5f, 0.0f));
    assert(TargetDelayedKf_Predict(&filter, 300U, 0.2f, 500U, &prediction));
    assert(prediction.yaw_rate_degps > 4.0f && prediction.yaw_rate_degps < 6.0f);
    assert(prediction.yaw_deg > -179.0f && prediction.yaw_deg < -178.0f);
    TargetDelayedKf_Reset(&filter);
    assert(TargetDelayedKf_Observe(&filter, 0U, 0U, -180.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&filter, 100U, 100U, 180.0f, 0.0f));
    assert_close(filter.yaw.rate, 0.0, 1e-12);
    assert_close(filter.yaw.angle, -180.0, 1e-12);
}

static void test_joint_nis_and_delayed_outlier(void)
{
    T_TargetDelayedKf single_axis;
    T_TargetDelayedKf both_axes;
    T_TargetDelayedKf ordered;
    T_TargetDelayedKf delayed;
    T_TargetDelayedKfPrediction a;
    T_TargetDelayedKfPrediction b;
    TargetDelayedKf_Reset(&single_axis);
    TargetDelayedKf_Reset(&both_axes);
    assert(TargetDelayedKf_Observe(&single_axis, 0U, 0U, 0.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&both_axes, 0U, 0U, 0.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&single_axis, 1U, 1U, 2.6f, 0.0f));
    /* Each residual passes alone; their combined NIS exceeds 25. */
    assert(!TargetDelayedKf_Observe(&both_axes, 1U, 1U, 2.6f, 2.6f));
    assert(both_axes.posterior_ms == 0U);

    TargetDelayedKf_Reset(&ordered);
    TargetDelayedKf_Reset(&delayed);
    assert(TargetDelayedKf_Observe(&ordered, 0U, 200U, 0.0f, 0.0f));
    assert(!TargetDelayedKf_Observe(&ordered, 50U, 200U, 20.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&ordered, 100U, 200U, 0.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&delayed, 100U, 200U, 0.0f, 0.0f));
    assert(TargetDelayedKf_Observe(&delayed, 0U, 200U, 0.0f, 0.0f));
    assert(!TargetDelayedKf_Observe(&delayed, 50U, 200U, 20.0f, 0.0f));
    assert(TargetDelayedKf_Predict(&ordered, 200U, 0.15f, 500U, &a));
    assert(TargetDelayedKf_Predict(&delayed, 200U, 0.15f, 500U, &b));
    assert_same_prediction(&a, &b);
    assert(a.accepted_samples == 2U && ordered.rejected_in_window == 1U);
    assert(!TargetDelayedKf_Observe(&delayed, 50U, 201U, 0.0f, 0.0f));
    assert(delayed.status == TARGET_DELAYED_KF_REJECTED_DUPLICATE);
}

static void test_capacity_and_anchor_replay(void)
{
    T_TargetDelayedKf ordered;
    T_TargetDelayedKf delayed;
    T_TargetDelayedKfPrediction a;
    T_TargetDelayedKfPrediction b;
    unsigned int i;
    TargetDelayedKf_Reset(&ordered);
    TargetDelayedKf_Reset(&delayed);
    for (i = 0U; i < 200U; ++i) {
        const float angle = (float)i * 0.01f;
        assert(TargetDelayedKf_Observe(&ordered, i * 5U, i * 5U, angle, 0.0f));
        if (i != 197U) {
            assert(TargetDelayedKf_Observe(&delayed, i * 5U, i * 5U, angle, 0.0f));
        }
        assert(ordered.history_count <= TARGET_DELAYED_KF_CAPACITY);
        assert(delayed.history_count <= TARGET_DELAYED_KF_CAPACITY);
    }
    assert(TargetDelayedKf_Observe(&delayed, 985U, 1000U, 1.97f, 0.0f));
    assert(ordered.history_count == 64U);
    assert(ordered.anchor_valid);
    assert(ordered.anchor_ms == 675U);
    assert(ordered.closed_before_ms == 675U);
    assert(ordered.accepted_samples == 200U);
    assert(TargetDelayedKf_Predict(&ordered, 1000U, 0.15f, 500U, &a));
    assert(TargetDelayedKf_Predict(&delayed, 1000U, 0.15f, 500U, &b));
    assert_same_prediction(&a, &b);
    assert(!TargetDelayedKf_Observe(&ordered, 675U, 1000U, 1.35f, 0.0f));
    assert(ordered.status == TARGET_DELAYED_KF_REJECTED_HISTORY);
}

static void test_rejected_observation_never_becomes_seed(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKfPrediction prediction;
    unsigned int i;
    TargetDelayedKf_Reset(&filter);
    assert(TargetDelayedKf_Observe(&filter, 0U, 0U, 0.0f, 0.0f));
    for (i = 1U; i <= 160U; ++i) {
        assert(!TargetDelayedKf_Observe(&filter, i, i, 90.0f, 0.0f));
        assert(filter.posterior_ms == 0U);
        assert(filter.accepted_samples == 1U);
        assert_close(filter.yaw.angle, 0.0, 1e-12);
    }
    assert(filter.anchor_valid);
    assert(filter.history_count == 64U);
    assert(filter.rejected_in_window == 64U);
    assert(filter.rejected_inputs == 160U);
    assert(!TargetDelayedKf_Predict(&filter, 180U, 0.0f, 100U, &prediction));
    assert(TargetDelayedKf_Observe(&filter, 181U, 181U, 0.0f, 0.0f));
    assert(filter.posterior_ms == 181U);
    assert(filter.accepted_samples == 2U);

    /* Time eviction, rather than capacity eviction, must preserve the seed. */
    TargetDelayedKf_Reset(&filter);
    assert(TargetDelayedKf_Observe(&filter, 0U, 0U, 0.0f, 0.0f));
    assert(!TargetDelayedKf_Observe(&filter, 1000U, 1000U, 90.0f, 0.0f));
    assert(!TargetDelayedKf_Observe(&filter, 1600U, 1600U, 90.0f, 0.0f));
    assert(filter.history_count == 2U);
    assert(filter.anchor_valid);
    assert(filter.posterior_ms == 0U);
    assert(filter.accepted_samples == 1U);
    assert_close(filter.yaw.angle, 0.0, 1e-12);
}

static void test_lag_bound(void)
{
    T_TargetDelayedKf filter;
    unsigned int i;
    TargetDelayedKf_Reset(&filter);
    for (i = 0U; i < 100U; ++i) {
        assert(TargetDelayedKf_Observe(&filter, i * 100U, i * 100U, 7.0f, 1.0f));
        assert(filter.history_count <= 16U);
        assert(filter.last_received_ms - filter.history[0].measurement_ms <= 1500U);
    }
    assert(filter.history_count == 16U);
    assert(filter.accepted_samples == 100U);
    assert(filter.anchor_ms == 8300U);
    assert(filter.history[0].measurement_ms == 8400U);
}

static void test_invalid_inputs_and_query_bounds(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKfPrediction prediction;
    TargetDelayedKf_Reset(NULL);
    TargetDelayedKf_Reset(&filter);
    assert(!TargetDelayedKf_Observe(NULL, 0U, 0U, 0.0f, 0.0f));
    assert(!TargetDelayedKf_Predict(&filter, 0U, 0.0f, 500U, &prediction));
    assert(!TargetDelayedKf_Predict(NULL, 0U, 0.0f, 500U, &prediction));
    assert(!TargetDelayedKf_Observe(&filter, 0U, 0U, NAN, 0.0f));
    assert(filter.status == TARGET_DELAYED_KF_REJECTED_NONFINITE);
    assert(!TargetDelayedKf_Observe(&filter, 0U, 0U, 0.0f, INFINITY));
    assert(!filter.has_posterior);
    assert(!TargetDelayedKf_Observe(&filter, 1001U, 1000U, 0.0f, 0.0f));
    assert(filter.status == TARGET_DELAYED_KF_REJECTED_FUTURE);
    assert(!TargetDelayedKf_Observe(&filter, 0U, 1501U, 0.0f, 0.0f));
    assert(filter.status == TARGET_DELAYED_KF_REJECTED_AGE);
    assert(TargetDelayedKf_Observe(&filter, 1000U, 1050U, 0.0f, 0.0f));
    assert(!TargetDelayedKf_Observe(&filter, 1001U, 1049U, 0.0f, 0.0f));
    assert(filter.status == TARGET_DELAYED_KF_REJECTED_ARRIVAL);
    assert(!TargetDelayedKf_Observe(&filter, 1000U, 1051U, 0.0f, 0.0f));
    assert(filter.status == TARGET_DELAYED_KF_REJECTED_DUPLICATE);
    assert(filter.posterior_ms == 1000U && filter.last_received_ms == 1050U);
    assert(filter.accepted_samples == 1U);
    assert(!TargetDelayedKf_Predict(&filter, 1049U, 0.0f, 500U, &prediction));
    assert(!TargetDelayedKf_Predict(&filter, 1050U, -0.01f, 500U, &prediction));
    assert(!TargetDelayedKf_Predict(&filter, 1050U, 0.501f, 500U, &prediction));
    assert(!TargetDelayedKf_Predict(&filter, 1050U, NAN, 500U, &prediction));
    assert(!TargetDelayedKf_Predict(&filter, 1050U, INFINITY, 500U, &prediction));
    assert(!TargetDelayedKf_Predict(&filter, 1050U, 0.0f, 500U, NULL));
    assert(!TargetDelayedKf_Predict(&filter, 1501U, 0.0f, 500U, &prediction));
    assert(TargetDelayedKf_Predict(&filter, 1500U, 0.5f, 500U, &prediction));
    assert(prediction.age_ms == 500U);
    assert(!TargetDelayedKf_Predict(&filter, UINT64_MAX, 0.5f, UINT64_MAX, &prediction));
    TargetDelayedKf_Reset(&filter);
    assert(TargetDelayedKf_Observe(&filter, UINT64_MAX - 500U, UINT64_MAX, 1.0f, 2.0f));
    assert(TargetDelayedKf_Predict(&filter, UINT64_MAX, 0.15f, 500U, &prediction));
    assert(prediction.age_ms == 500U);
    TargetDelayedKf_Reset(&filter);
    assert(filter.history_count == 0U && !filter.has_posterior && !filter.anchor_valid);
    assert(!filter.has_arrival && !filter.history_closed && filter.rejected_inputs == 0U);
}

static void test_long_run_covariance(void)
{
    T_TargetDelayedKf filter;
    T_TargetDelayedKfPrediction prediction;
    unsigned int i;
    TargetDelayedKf_Reset(&filter);
    for (i = 0U; i < 5000U; ++i) {
        const float noise = ((float)(i % 7U) - 3.0f) * 0.01f;
        const float yaw = (float)TargetDelayedKf_Wrap(179.0 + 0.002 * (double)i) + noise;
        assert(TargetDelayedKf_Observe(&filter, i * 10U, i * 10U, yaw, noise));
        assert(filter.yaw.p00 > 0.0 && filter.yaw.p11 > 0.0);
        assert(filter.pitch.p00 > 0.0 && filter.pitch.p11 > 0.0);
        assert(filter.yaw.p00 * filter.yaw.p11 - filter.yaw.p01 * filter.yaw.p01 >= -1e-12);
        assert(filter.pitch.p00 * filter.pitch.p11 - filter.pitch.p01 * filter.pitch.p01 >= -1e-12);
    }
    assert(filter.accepted_samples == 5000U);
    assert(TargetDelayedKf_Predict(&filter, 50000U, 0.15f, 500U, &prediction));
    assert_close(prediction.yaw_rate_degps, 0.2, 0.05);
    assert(isfinite(prediction.yaw_variance_deg2) && prediction.yaw_variance_deg2 > 0.0f);
}

int main(void)
{
    test_two_sample_solution();
    test_out_of_order_matches_ordered();
    test_predict_does_not_mutate_or_double_integrate();
    test_rejection_does_not_refresh();
    test_wrap();
    test_joint_nis_and_delayed_outlier();
    test_capacity_and_anchor_replay();
    test_rejected_observation_never_becomes_seed();
    test_lag_bound();
    test_invalid_inputs_and_query_bounds();
    test_long_run_covariance();
    puts("delayed bearing KF core tests passed (11 groups)");
    return 0;
}
