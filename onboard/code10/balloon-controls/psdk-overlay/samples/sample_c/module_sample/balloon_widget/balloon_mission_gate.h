#ifndef BALLOON_MISSION_GATE_H
#define BALLOON_MISSION_GATE_H
#include "balloon_clock.h"

#define BALLOON_MISSION_TOKEN "/tmp/balloon_mission.token"
#define BALLOON_MISSION_READY "/tmp/balloon_mission.ready"
#define BALLOON_MISSION_COMMIT "/tmp/balloon_mission.commit"
#define BALLOON_MISSION_HEALTH "/tmp/balloon_mission.health"
#define BALLOON_MISSION_STATUS "/tmp/balloon_mission_status.txt"

static inline bool BalloonMission_ReadToken(const char *path, char token[96])
{
    FILE *fp = fopen(path, "r");
    token[0] = '\0';
    if (fp == NULL) return false;
    bool ok = fgets(token, 96, fp) != NULL;
    fclose(fp);
    token[strcspn(token, "\r\n")] = '\0';
    return ok && token[0] != '\0';
}

static inline bool BalloonMission_Matches(const char *path, const char *token)
{
    char actual[96];
    return BalloonMission_ReadToken(path, actual) && strcmp(actual, token) == 0;
}

/* A receipt from another boot, request, or a stalled worker cannot open the gate. */
static inline bool BalloonMission_Healthy(const char *token)
{
    char actual[96], clockId[80];
    unsigned long long stamp = 0;
    FILE *fp = fopen(BALLOON_MISSION_HEALTH, "r");
    if (fp == NULL) return false;
    int fields = fscanf(fp, "%95s %79s %llu", actual, clockId, &stamp);
    fclose(fp);
    uint64_t now = BalloonClock_NowMs();
    return fields == 3 && strcmp(actual, token) == 0 && BalloonClock_Matches(clockId) &&
           now >= stamp && now - stamp <= 2000 &&
           BalloonMission_Matches(BALLOON_MISSION_TOKEN, token);
}
#endif
