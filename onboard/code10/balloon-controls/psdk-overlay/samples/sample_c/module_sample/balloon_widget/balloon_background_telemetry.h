#ifndef BALLOON_BACKGROUND_TELEMETRY_H
#define BALLOON_BACKGROUND_TELEMETRY_H

#include "dji_fc_subscription.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BALLOON_SUB_CHASE = 1,
    BALLOON_SUB_TIMING = 2,
    BALLOON_SUB_PROBE = 4,
    BALLOON_SUB_BACKGROUND = 8
} E_BalloonSubscriptionOwner;

/* Process-lifetime subscription module. Consumers release only their own topics.
 * Never call DjiFcSubscription_DeInit while this application is running. */
T_DjiReturnCode BalloonSubscription_Init(void);
/* Only the chase owner registers a callback, for CONTROL_DEVICE. Its
 * permanent 50 Hz SDK subscription is shared with the read-only recorder. */
T_DjiReturnCode BalloonSubscription_Acquire(E_BalloonSubscriptionOwner owner,
    E_DjiFcSubscriptionTopic topic, E_DjiDataSubscriptionTopicFreq frequency,
    DjiReceiveDataOfTopicCallback callback);
T_DjiReturnCode BalloonSubscription_Release(E_BalloonSubscriptionOwner owner,
    E_DjiFcSubscriptionTopic topic);

/* Mark a topic missing only after the SDK reports TOPIC_NOT_SUBSCRIBED. */
void BalloonSubscription_ForgetMissing(E_DjiFcSubscriptionTopic topic);

/* Starts independent read-only threads; no camera, widget, mission or flight
 * controller initialization is required. Idempotent. SDK retries are async. */
T_DjiReturnCode BalloonTelemetry_Start(void);

#ifdef __cplusplus
}
#endif
#endif
