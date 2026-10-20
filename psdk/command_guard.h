#ifndef M300_COMMAND_GUARD_H
#define M300_COMMAND_GUARD_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint64_t sent_ms, deadline_ms, revision, motion_id, target_index;
    bool enabled;
    double x,y,z,yaw;
    char session[65], bridge_boot[65], clock_id[80], plan_hash[65], nonce[65];
} M300Command;
typedef struct {
    bool enabled, prepared, authority, in_air, fresh, velocity_healthy, paused;
    double altitude_m;
    uint64_t revision, blocked_motion_id, target_index;
    char session[65], bridge_boot[65], clock_id[80], plan_hash[65], nonce[65];
} M300Permission;
bool M300_CommandAllowed(const M300Command *c,const M300Permission *p,uint64_t now);
#endif
