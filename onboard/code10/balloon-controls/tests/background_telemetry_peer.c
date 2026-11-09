/* A separate translation unit proves ownership is process-global. */
#include "balloon_widget/balloon_background_telemetry.h"
T_DjiReturnCode BackgroundTelemetryTest_PeerAcquire(void)
{
    return BalloonSubscription_Acquire(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
        DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, 0);
}
T_DjiReturnCode BackgroundTelemetryTest_PeerRelease(void)
{
    return BalloonSubscription_Release(BALLOON_SUB_CHASE, DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
}
