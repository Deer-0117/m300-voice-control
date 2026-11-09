#ifndef BALLOON_MISSION_GATE_H
#define BALLOON_MISSION_GATE_H
#include "balloon_clock.h"
#include "balloon_guard_file.h"

#define BALLOON_MISSION_TOKEN "/tmp/balloon_mission.token"
#define BALLOON_MISSION_READY "/tmp/balloon_mission.ready"
#define BALLOON_MISSION_COMMIT "/tmp/balloon_mission.commit"
#define BALLOON_MISSION_HEALTH "/tmp/balloon_mission.health"
#define BALLOON_MISSION_STATUS "/tmp/balloon_mission_status.txt"

static inline bool BalloonMission_ReadToken(const char *path, char token[96])
{
    char value[98];
    token[0] = '\0';
    if (!BalloonGuard_ReadRegular(path, value, sizeof(value))) return false;
    size_t length = strlen(value);
    if (length && value[length - 1] == '\n') value[--length] = '\0';
    if (length && value[length - 1] == '\r') value[--length] = '\0';
    if (!length || length > 95 ||
        strspn(value, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") != length)
        return false;
    memcpy(token, value, length + 1);
    return true;
}

static inline bool BalloonMission_Matches(const char *path, const char *token)
{
    char actual[96];
    return BalloonMission_ReadToken(path, actual) && strcmp(actual, token) == 0;
}

/* A receipt from another boot, request, or a stalled worker cannot open the gate. */
static inline bool BalloonMission_Healthy(const char *token)
{
    char line[256], actual[96], clockId[80];
    unsigned long long stamp = 0;
    int consumed = 0;
    if (!BalloonGuard_ReadRegular(BALLOON_MISSION_HEALTH, line, sizeof(line))) return false;
    int fields = sscanf(line, "%95s %79s %llu %n", actual, clockId, &stamp, &consumed);
    uint64_t now = BalloonClock_NowMs();
    return fields == 3 && consumed > 0 && line[consumed] == '\0' &&
           strcmp(actual, token) == 0 && BalloonClock_Matches(clockId) &&
           stamp > 0 && now >= stamp && now - stamp <= 2000 &&
           BalloonMission_Matches(BALLOON_MISSION_TOKEN, token);
}
#endif
