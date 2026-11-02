#include "target_chase_timing_alignment.h"

#include <assert.h>
#include <stdio.h>

static const char CLOCK[] = "mono-v1:ad828ee3-3076-44b6-83c5-a60b29cacde3";
static const char OTHER_CLOCK[] = "mono-v1:f99d7de0-dd95-4808-bfbd-8cd2dd24d41c";

static T_TargetTimingAlignment calibration(void)
{
    T_TargetTimingAlignment a;
    memset(&a, 0, sizeof(a));
    a.enabled = true;
    a.verified = true;
    strcpy(a.clock_id, CLOCK);
    strcpy(a.calibration_id, "synthetic-two-delay-test");
    a.video_delay_ms = 120.0;
    a.pose_delay_ms = 20.0;
    a.uncertainty_ms = 5.0;
    a.valid_from_ms = 1000;
    a.valid_until_ms = 10000;
    return a;
}

static void test_calibration_contract(void)
{
    T_TargetTimingAlignment a = calibration();
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_OK);
    assert(TargetTiming_Validate(NULL, CLOCK, 2000) == TARGET_TIMING_DISABLED);
    a.enabled = false;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_DISABLED);
    a = calibration();
    a.verified = false;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_UNVERIFIED);
    a = calibration();
    assert(TargetTiming_Validate(&a, OTHER_CLOCK, 2000) == TARGET_TIMING_CLOCK_MISMATCH);
    assert(TargetTiming_Validate(&a, "mono-v1:old", 2000) == TARGET_TIMING_INVALID_CLOCK);
    assert(TargetTiming_Validate(&a, NULL, 2000) == TARGET_TIMING_INVALID_CLOCK);
    memset(a.clock_id, 'a', sizeof(a.clock_id));
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_CLOCK);
    a = calibration();
    a.calibration_id[0] = '\0';
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_ID);
    memset(a.calibration_id, 'a', sizeof(a.calibration_id));
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_ID);
    a = calibration();
    a.video_delay_ms = -1.0;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_DELAY);
    a.video_delay_ms = NAN;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_DELAY);
    a = calibration();
    a.pose_delay_ms = INFINITY;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_DELAY);
    a.pose_delay_ms = 1500.1;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_DELAY);
    a = calibration();
    a.uncertainty_ms = 25.01;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_UNCERTAIN);
    a.uncertainty_ms = NAN;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_UNCERTAIN);
    a.uncertainty_ms = -0.1;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_UNCERTAIN);
    a = calibration();
    a.valid_from_ms = 0;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_WINDOW);
    a.valid_from_ms = a.valid_until_ms;
    assert(TargetTiming_Validate(&a, CLOCK, 2000) == TARGET_TIMING_INVALID_WINDOW);
    a = calibration();
    assert(TargetTiming_Validate(&a, CLOCK, 999) == TARGET_TIMING_OUTSIDE_WINDOW);
    assert(TargetTiming_Validate(&a, CLOCK, 10001) == TARGET_TIMING_OUTSIDE_WINDOW);
    assert(strcmp(TargetTiming_ReasonName(TARGET_TIMING_UNVERIFIED), "timing_unverified") == 0);
}

static void test_mapping(void)
{
    T_TargetTimingAlignment a = calibration();
    uint64_t stamp = 777;
    assert(TargetTiming_MapVideo(&a, CLOCK, 2120, 2200, 2300, &stamp) == TARGET_TIMING_OK);
    assert(stamp == 2000);
    assert(TargetTiming_MapPose(&a, CLOCK, 2020, 2022, 2300, &stamp) == TARGET_TIMING_OK);
    assert(stamp == 2000); /* same physical instant, distinct arrival delays */
    a.video_delay_ms = 120.6;
    assert(TargetTiming_MapVideo(&a, CLOCK, 2120, 2200, 2300, &stamp) == TARGET_TIMING_OK);
    assert(stamp == 1999);
    assert(TargetTiming_MapVideo(&a, CLOCK, 2400, 2200, 2300, &stamp) == TARGET_TIMING_INVALID_TIMESTAMP);
    assert(stamp == 0);
    assert(TargetTiming_MapVideo(&a, CLOCK, 2120, 2400, 2300, &stamp) == TARGET_TIMING_INVALID_TIMESTAMP);
    assert(TargetTiming_MapVideo(&a, CLOCK, 0, 2200, 2300, &stamp) == TARGET_TIMING_INVALID_TIMESTAMP);
    assert(TargetTiming_MapVideo(&a, CLOCK, 2120, 2200, 2300, NULL) == TARGET_TIMING_INVALID_TIMESTAMP);
    assert(TargetTiming_MapVideo(&a, CLOCK, 1100, 2200, 2300, &stamp) == TARGET_TIMING_OUTSIDE_WINDOW);
    a.valid_from_ms = 1;
    assert(TargetTiming_MapVideo(&a, CLOCK, 100, 200, 2300, &stamp) == TARGET_TIMING_INVALID_TIMESTAMP);
    assert(TargetTiming_MapVideo(&a, OTHER_CLOCK, 2120, 2200, 2300, &stamp) == TARGET_TIMING_CLOCK_MISMATCH);
    assert(TargetTiming_MapVideo(&a, CLOCK, 2120, 2200, 10001, &stamp) == TARGET_TIMING_OUTSIDE_WINDOW);
}

static void test_pose_interpolation(void)
{
    T_TargetTimingPose poses[] = {
        {2100, 2120, -179.0f, 4.0f},
        {1900, 1920, 179.0f, 0.0f},
        {2000, 4000, 0.0f, 80.0f}, /* unavailable at query time */
        {2000, 1900, 0.0f, 80.0f}, /* received before physical sample */
        {2000, 2020, NAN, 0.0f},
    };
    float yaw, pitch;
    assert(TargetTiming_PoseAt(poses, 5, 2000, 2200, &yaw, &pitch));
    assert(fabsf(fabsf(yaw) - 180.0f) < 1e-5f);
    assert(fabsf(pitch - 2.0f) < 1e-5f);
    assert(TargetTiming_PoseAt(poses, 5, 1900, 2200, &yaw, &pitch));
    assert(fabsf(yaw - 179.0f) < 1e-5f);
    assert(!TargetTiming_PoseAt(poses, 5, 1899, 2200, &yaw, &pitch));
    assert(!TargetTiming_PoseAt(poses, 5, 2101, 2200, &yaw, &pitch));
    assert(!TargetTiming_PoseAt(poses, 5, 0, 2200, &yaw, &pitch));
    assert(!TargetTiming_PoseAt(poses, 5, 2300, 2200, &yaw, &pitch));
    assert(!TargetTiming_PoseAt(NULL, 0, 2000, 2200, &yaw, &pitch));
    assert(!TargetTiming_PoseAt(poses, 5, 2000, 2200, NULL, &pitch));
    poses[0].timestamp_ms = 2101;
    assert(!TargetTiming_PoseAt(poses, 5, 2000, 2200, &yaw, &pitch));
}

static void test_delayed_camera_motion(void)
{
    T_TargetTimingAlignment a = calibration();
    T_TargetTimingPose poses[2] = {{0, 1920, 19.0f, 0.0f}, {0, 2120, 21.0f, 0.0f}};
    uint64_t frame_ms;
    float yaw, pitch;
    assert(TargetTiming_MapPose(&a, CLOCK, 1920, 1920, 2300, &poses[0].timestamp_ms) == TARGET_TIMING_OK);
    assert(TargetTiming_MapPose(&a, CLOCK, 2120, 2120, 2300, &poses[1].timestamp_ms) == TARGET_TIMING_OK);
    assert(TargetTiming_MapVideo(&a, CLOCK, 2120, 2300, 2300, &frame_ms) == TARGET_TIMING_OK);
    assert(TargetTiming_PoseAt(poses, 2, frame_ms, 2300, &yaw, &pitch));
    /* The frame exposed at 2000 ms saw camera yaw 20 degrees, although the
     * newest available camera pose already points at 21 degrees. */
    assert(fabsf(yaw - 20.0f) < 1e-5f);
    assert(fabsf(pitch) < 1e-5f);
    assert(fabsf((yaw + 5.0f) - 25.0f) < 1e-5f);
}

static void test_current_pose(void)
{
    T_TargetTimingPose poses[] = {
        {1900, 1950, 19.0f, 1.0f},
        {1950, 2000, 20.0f, 1.5f},
        {2000, 2050, 21.0f, 2.0f} /* not received at now=2000 */
    };
    float yaw, pitch;
    assert(TargetTiming_CurrentPose(poses, 3, 2000, &yaw, &pitch));
    assert(fabsf(yaw-21.0f) < 1e-4f && fabsf(pitch-2.0f) < 1e-4f);
    assert(!TargetTiming_CurrentPose(poses, 2, 2051, &yaw, &pitch)); /* frozen */
    assert(!TargetTiming_CurrentPose(poses, 2, 2101, &yaw, &pitch));
    assert(!TargetTiming_CurrentPose(NULL, 0, 2000, &yaw, &pitch));
    assert(!TargetTiming_CurrentPose(poses, 3, 2000, NULL, &pitch));
    assert(!TargetTiming_CurrentPose(poses, 3, 0, &yaw, &pitch));
    assert(!TargetTiming_CurrentPose(poses+1, 1, 2000, &yaw, &pitch));
    poses[0].timestamp_ms = 1849;
    assert(!TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch)); /* gap */
    poses[0].timestamp_ms = 1900;
    poses[1].received_ms = 2100;
    assert(!TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch));
    poses[1].received_ms = 2000;
    poses[1].yaw_deg = 23.0f;
    assert(!TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch)); /* >60 deg/s */
    poses[1].yaw_deg = 20.0f;
    poses[1].pitch_deg = 4.0f;
    assert(!TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch)); /* >40 deg/s */
    poses[0].pitch_deg = 88.0f;
    poses[1].pitch_deg = 89.5f;
    assert(!TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch)); /* predicted >90 */
    poses[0].pitch_deg = poses[1].pitch_deg = 0.0f;
    poses[0].yaw_deg = 179.0f;
    poses[1].yaw_deg = -179.0f;
    assert(TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch));
    assert(fabsf(yaw-(-177.0f)) < 1e-4f);
    poses[0].yaw_deg = -179.0f;
    poses[1].yaw_deg = 179.0f;
    assert(TargetTiming_CurrentPose(poses, 2, 2000, &yaw, &pitch));
    assert(fabsf(yaw-177.0f) < 1e-4f); /* reverse direction at wrap */
    poses[1].timestamp_ms = poses[1].received_ms = 2000;
    assert(TargetTiming_CurrentPose(poses+1, 1, 2000, &yaw, &pitch));
    assert(fabsf(yaw-179.0f) < 1e-4f); /* exact current sample needs no rate */
    poses[1].yaw_deg = NAN;
    assert(!TargetTiming_CurrentPose(poses+1, 1, 2000, &yaw, &pitch));
    poses[1].yaw_deg = 179.0f;
    poses[1].pitch_deg = 90.1f;
    assert(!TargetTiming_CurrentPose(poses+1, 1, 2000, &yaw, &pitch));
}

int main(void)
{
    test_calibration_contract();
    test_mapping();
    test_pose_interpolation();
    test_delayed_camera_motion();
    test_current_pose();
    puts("live timing contract and pose alignment tests passed");
    return 0;
}
