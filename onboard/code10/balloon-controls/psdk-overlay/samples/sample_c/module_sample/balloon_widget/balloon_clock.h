#ifndef BALLOON_CLOCK_H
#define BALLOON_CLOCK_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* No wall-clock fallback: a failed clock must not silently change domains. */
static inline uint64_t BalloonClock_NowMs(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (uint64_t) ts.tv_sec * 1000ULL + (uint64_t) ts.tv_nsec / 1000000ULL;
}

static inline uint64_t BalloonClock_WallMs(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return 0;
    return (uint64_t) ts.tv_sec * 1000ULL + (uint64_t) ts.tv_nsec / 1000000ULL;
}

/* /proc is memory backed. Read locally to avoid shared lazy-init races. */
static inline bool BalloonClock_Id(char token[80])
{
    char boot[40] = {0};
    FILE *fp = fopen("/proc/sys/kernel/random/boot_id", "r");
    token[0] = '\0';
    if (fp == NULL) return false;
    if (fgets(boot, sizeof(boot), fp) == NULL) { fclose(fp); return false; }
    fclose(fp);
    boot[strcspn(boot, "\r\n")] = '\0';
    if (strlen(boot) != 36 || strspn(boot, "0123456789abcdef-") != 36) return false;
    snprintf(token, 80, "mono-v1:%s", boot);
    return true;
}

static inline bool BalloonClock_Matches(const char *token)
{
    char current[80];
    return token != NULL && BalloonClock_Id(current) && strcmp(current, token) == 0;
}

#endif
