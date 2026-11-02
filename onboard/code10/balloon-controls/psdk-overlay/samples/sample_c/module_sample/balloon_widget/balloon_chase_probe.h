#ifndef BALLOON_CHASE_PROBE_H
#define BALLOON_CHASE_PROBE_H

#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * First-tier flight-control interface probe.
 *
 * This function verifies initialization, subscription, joystick-mode setup, and
 * target-chase command calculation without obtaining joystick authority and
 * without sending any joystick command.
 */
T_DjiReturnCode BalloonChaseProbe_RunNoFlightProbe(void);

#ifdef __cplusplus
}
#endif

#endif /* BALLOON_CHASE_PROBE_H */
