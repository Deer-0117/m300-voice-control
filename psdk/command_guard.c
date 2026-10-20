#include "command_guard.h"
#include <math.h>
#include <string.h>
bool M300_CommandAllowed(const M300Command *c,const M300Permission *p,uint64_t now)
{
    if(!c||!p||!p->enabled||!c->enabled||!p->prepared||!p->authority||!p->in_air||!p->fresh||!p->velocity_healthy||p->paused)return false;
    if(!isfinite(p->altitude_m)||p->altitude_m<2.||p->altitude_m>10.)return false;
    if(c->sent_ms>now||now-c->sent_ms>150||c->deadline_ms<=now||c->deadline_ms-now>500)return false;
    if(c->revision!=p->revision||c->target_index!=p->target_index||c->motion_id<=p->blocked_motion_id)return false;
    if(!c->session[0]||!c->nonce[0]||!c->plan_hash[0]||strcmp(c->session,p->session)||strcmp(c->bridge_boot,p->bridge_boot)||strcmp(c->clock_id,p->clock_id)||strcmp(c->plan_hash,p->plan_hash)||strcmp(c->nonce,p->nonce))return false;
    if(!isfinite(c->x)||!isfinite(c->y)||!isfinite(c->z)||!isfinite(c->yaw))return false;
    return hypot(c->x,c->y)<=.5&&fabs(c->z)<=.2&&fabs(c->yaw)<=5.;
}
