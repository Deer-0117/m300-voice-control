#include "command_guard.h"
#include <assert.h>
#include <math.h>
#include <string.h>
int main(void){
 M300Command c={.sent_ms=1000,.deadline_ms=1400,.revision=2,.motion_id=3,.enabled=true,.x=.1};
 M300Permission p={.enabled=true,.prepared=true,.authority=true,.in_air=true,.fresh=true,.velocity_healthy=true,.altitude_m=3.,.revision=2};
 #define MATCH(f,v) strcpy(c.f,v);strcpy(p.f,v)
 MATCH(session,"session");MATCH(bridge_boot,"boot");MATCH(clock_id,"clock");MATCH(plan_hash,"hash");MATCH(nonce,"nonce");
 assert(M300_CommandAllowed(&c,&p,1050));
 p.target_index=1;assert(!M300_CommandAllowed(&c,&p,1050));p.target_index=0;
 p.paused=true;assert(!M300_CommandAllowed(&c,&p,1050));p.paused=false;
 p.blocked_motion_id=3;assert(!M300_CommandAllowed(&c,&p,1050));p.blocked_motion_id=0;
 assert(!M300_CommandAllowed(&c,&p,1200)); /* heartbeat older than 150ms */
 c.sent_ms=1350;assert(!M300_CommandAllowed(&c,&p,1400));
 c.sent_ms=1000;c.revision=1;assert(!M300_CommandAllowed(&c,&p,1050));c.revision=2;
 strcpy(c.nonce,"old");assert(!M300_CommandAllowed(&c,&p,1050));strcpy(c.nonce,p.nonce);
 c.x=NAN;assert(!M300_CommandAllowed(&c,&p,1050));c.x=.1;
 p.authority=false;assert(!M300_CommandAllowed(&c,&p,1050));p.authority=true;
 p.fresh=false;assert(!M300_CommandAllowed(&c,&p,1050));p.fresh=true;
 p.enabled=false;assert(!M300_CommandAllowed(&c,&p,1050));p.enabled=true;
 c.x=.6;assert(!M300_CommandAllowed(&c,&p,1050));c.x=.1;
 p.altitude_m=NAN;assert(!M300_CommandAllowed(&c,&p,1050));
 return 0;
}
