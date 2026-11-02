#ifndef BALLOON_PERFORMANCE_H
#define BALLOON_PERFORMANCE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Fixed-size, allocation-free accumulator. The owning controller thread must
 * serialize access, just as it does for T_TargetChaseController. No clock or IO
 * is read here. Bins 0..20 have inclusive bounds 1000 * 2^i ns, bin 21 overflows.
 * The quantile is a nearest-rank histogram upper bound, NOT an exact percentile. */
#define BALLOON_PERFORMANCE_BINS 22U
typedef struct {
    uint64_t count, success_count, sum_ns, min_ns, max_ns, invalid_count;
    uint64_t histogram[BALLOON_PERFORMANCE_BINS];
} T_BalloonPerformanceStats;

static inline void BalloonPerformance_Reset(T_BalloonPerformanceStats *stats)
{
    memset(stats, 0, sizeof(*stats));
}

static inline uint64_t BalloonPerformance_Bound(unsigned int bin)
{
    return bin < BALLOON_PERFORMANCE_BINS - 1U ? 1000ULL << bin : 0;
}

static inline bool BalloonPerformance_Add(T_BalloonPerformanceStats *stats,
                                          uint64_t begin_ns, uint64_t end_ns,
                                          bool success)
{
    uint64_t elapsed;
    unsigned int bin = 0;
    if (begin_ns == 0 || end_ns == 0 || end_ns < begin_ns) goto invalid;
    elapsed = end_ns - begin_ns;
    if (stats->count == UINT64_MAX || UINT64_MAX - stats->sum_ns < elapsed) goto invalid;
    while (bin < BALLOON_PERFORMANCE_BINS - 1U && elapsed > BalloonPerformance_Bound(bin)) ++bin;
    if (stats->count == 0 || elapsed < stats->min_ns) stats->min_ns = elapsed;
    if (elapsed > stats->max_ns) stats->max_ns = elapsed;
    ++stats->count;
    if (success) ++stats->success_count;
    stats->sum_ns += elapsed;
    ++stats->histogram[bin];
    return true;
invalid:
    if (stats->invalid_count != UINT64_MAX) ++stats->invalid_count;
    return false;
}

static inline bool BalloonPerformance_P95Upper(const T_BalloonPerformanceStats *stats,
                                               uint64_t *upper_ns)
{
    uint64_t cumulative = 0;
    uint64_t rank = stats->count - stats->count / 20U;
    unsigned int bin;
    if (stats->count == 0) return false;
    for (bin = 0; bin < BALLOON_PERFORMANCE_BINS - 1U; ++bin) {
        cumulative += stats->histogram[bin];
        if (cumulative >= rank) { *upper_ns = BalloonPerformance_Bound(bin); return true; }
    }
    return false;
}
#endif
