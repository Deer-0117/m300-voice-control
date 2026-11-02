/* Real adapter + deterministic SDK stubs. No DJI library, file writes or device I/O. */
#include <stdio.h>
static FILE *blocked_open(const char *path, const char *mode) { (void)path; (void)mode; return NULL; }
#define fopen blocked_open
#include "../psdk-overlay/samples/sample_c/module_sample/target_chase/dji_target_chase_task.c"
#undef fopen
#include <assert.h>

static unsigned obtains, releases, commands, rotations, resets, deinits;
static T_DjiReturnCode release_result, zero_result, deinit_result, callback_result;
static bool takeover_on_zero;
static T_DjiFlightControllerJoystickCommand last_command;
static void owner(bool sdk) {
    T_DjiFlightControllerJoystickCtrlAuthorityEventInfo info = {0};
    info.curJoystickCtrlAuthority = sdk ? DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK : DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_RC;
    assert(DjiTargetChase_AuthorityEventCallback(info) == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
}
T_DjiReturnCode DjiFlightController_Init(T_DjiFlightControllerRidInfo info) { (void)info; return 0; }
T_DjiReturnCode DjiFlightController_DeInit(void) { ++deinits; return deinit_result; }
T_DjiReturnCode DjiFlightController_RegJoystickCtrlAuthorityEventCallback(JoystickCtrlAuthorityEventCbFunc cb) { (void)cb; return callback_result; }
void DjiFlightController_SetJoystickMode(T_DjiFlightControllerJoystickMode mode) { (void)mode; }
T_DjiReturnCode DjiFlightController_ObtainJoystickCtrlAuthority(void) { ++obtains; owner(true); return 0; }
T_DjiReturnCode DjiFlightController_ReleaseJoystickCtrlAuthority(void) { ++releases; if (!release_result) owner(false); return release_result; }
T_DjiReturnCode DjiFlightController_ExecuteJoystickAction(T_DjiFlightControllerJoystickCommand command) {
    ++commands; last_command = command; if (takeover_on_zero) owner(false); return zero_result;
}
T_DjiReturnCode DjiFcSubscription_Init(void) { return 0; }
T_DjiReturnCode DjiFcSubscription_DeInit(void) { return 0; }
T_DjiReturnCode DjiFcSubscription_SubscribeTopic(E_DjiFcSubscriptionTopic topic,E_DjiDataSubscriptionTopicFreq freq,DjiReceiveDataOfTopicCallback cb) { (void)topic; (void)freq; (void)cb; return 0; }
T_DjiReturnCode DjiFcSubscription_UnSubscribeTopic(E_DjiFcSubscriptionTopic topic) { (void)topic; return 0; }
T_DjiReturnCode DjiFcSubscription_GetLatestValueOfTopic(E_DjiFcSubscriptionTopic topic,uint8_t *data,uint16_t size,T_DjiDataTimestamp *stamp) {
    memset(data,0,size); if (stamp) { memset(stamp,0,sizeof(*stamp)); stamp->millisecond = 1000; }
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION) ((T_DjiFcSubscriptionQuaternion*)data)->q0 = 1;
    return 0;
}
T_DjiOsalHandler *DjiPlatform_GetOsalHandler(void) { return NULL; }
T_DjiReturnCode DjiGimbalManager_Init(void) { return 0; }
T_DjiReturnCode DjiGimbalManager_Deinit(void) { return 0; }
T_DjiReturnCode DjiGimbalManager_SetMode(E_DjiMountPosition pos,E_DjiGimbalMode mode) { (void)pos; (void)mode; return 0; }
T_DjiReturnCode DjiGimbalManager_Reset(E_DjiMountPosition pos,E_DjiGimbalResetMode mode) { (void)pos; (void)mode; ++resets; return 0; }
T_DjiReturnCode DjiGimbalManager_Rotate(E_DjiMountPosition pos,T_DjiGimbalManagerRotation rot) { (void)pos; (void)rot; ++rotations; return 0; }
T_DjiReturnCode DjiGimbalManager_SetControllerMaxSpeedPercentage(E_DjiMountPosition pos,E_DjiGimbalAxis axis,uint8_t val) { (void)pos; (void)axis; (void)val; return 0; }
T_DjiReturnCode DjiGimbalManager_SetControllerSmoothFactor(E_DjiMountPosition pos,E_DjiGimbalAxis axis,uint8_t val) { (void)pos; (void)axis; (void)val; return 0; }

static T_TargetChaseConfig config;
static void fresh(void) {
    s_initialized = false; s_real_authority_obtained = false; s_control_inhibited = false;
    s_manual_takeover = false; s_ever_had_authority = false; s_intentional_release = false;
    s_gimbal_manager_initialized = false; s_gimbal_angles_subscribed = false;
    release_result = zero_result = deinit_result = callback_result = 0;
    takeover_on_zero = false; obtains = releases = commands = rotations = resets = deinits = 0;
    config = TargetChase_DefaultConfig(640,480,73.7f,45.7f);
    setenv("BALLOON_NO_FLIGHT","0",1); setenv("BALLOON_GIMBAL_TRACKING","0",1);
    assert(DjiTargetChase_Init(&config) == 0);
}
static void acquire(void) {
    assert(DjiTargetChase_SetManualTrigger(true,DjiTargetChase_NowMs()) == 0);
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) == 0);
    assert(obtains == 1 && s_real_authority_obtained);
}
static void test_initial_callback_and_takeover(void) {
    fresh(); owner(false); assert(!DjiTargetChase_IsControlInhibited()); acquire();
    owner(false); assert(DjiTargetChase_IsControlInhibited()); assert(!DjiTargetChase_IsReleasePending());
    assert(!DjiTargetChase_IsManualTriggerEnabled());
    assert(DjiTargetChase_SetManualTrigger(true,1000) != 0);
    assert(DjiTargetChase_EnsureSearchAuthority(1000) != 0);
    assert(DjiTargetChase_RunOnce(1000) != 0);
    assert(DjiTargetChase_OnTargetPixel(320,240,1000) != 0);
    assert(DjiTargetChase_OnTargetBox(640,480,300,260,340,220,1000) != 0);
    assert(DjiTargetChase_RequestSearch(1000,false) != 0);
    assert(DjiTargetChase_RepeatLastCommand(1000) != 0);
    assert(DjiTargetChase_Hover(1000) != 0);
    s_gimbal_tracking_enabled = s_gimbal_manager_initialized = true;
    s_body_search_enabled = s_gimbal_search_enabled = true;
    DjiTargetChase_GimbalTick(1000); (void)DjiTargetChase_SearchTick(1000);
    assert(DjiTargetChase_SendGimbalSpeed(0,0,1000,"test",true) != 0);
    assert(DjiTargetChase_DeInit() == 0);
    assert(obtains == 1 && !commands && !releases && !rotations && !resets);
    assert(DjiTargetChase_IsControlInhibited());
    assert(DjiTargetChase_Init(&config) == 0 && !DjiTargetChase_IsControlInhibited());
    assert(DjiTargetChase_SetManualTrigger(true,1000) == 0);
    assert(DjiTargetChase_DeInit() == 0);
}
static void test_late_grant_keeps_stop_latched(void) {
    fresh(); acquire(); owner(false); owner(true);
    assert(DjiTargetChase_IsControlInhibited() && DjiTargetChase_IsReleasePending());
    assert(DjiTargetChase_EnsureSearchAuthority(1000) != 0);
    assert(DjiTargetChase_DeInit() == 0);
    assert(obtains == 1 && releases == 1 && commands == 1 && !DjiTargetChase_IsReleasePending());
}
static void test_release_failure_retry(void) {
    fresh(); acquire(); release_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == release_result);
    assert(DjiTargetChase_GetLastStopResult() == release_result);
    assert(DjiTargetChase_IsReleasePending() && s_initialized && s_chase.has_authority);
    assert(DjiTargetChase_Init(&config) != 0 && !deinits);
    assert(last_command.x == 0 && last_command.y == 0 && last_command.z == 0 && last_command.yaw == 0);
    assert(DjiTargetChase_SetManualTrigger(true,1000) != 0);
    release_result = 0; assert(DjiTargetChase_DeInit() == 0);
    assert(!DjiTargetChase_IsReleasePending() && !s_initialized && releases == 2 && deinits == 1);
    assert(DjiTargetChase_Init(&config) == 0 && !DjiTargetChase_IsControlInhibited());
    assert(DjiTargetChase_DeInit() == 0);
}
static void test_zero_failure_is_reported_after_release(void) {
    fresh(); acquire(); zero_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == zero_result);
    assert(DjiTargetChase_GetLastStopResult() == zero_result);
    assert(!DjiTargetChase_IsReleasePending() && !s_initialized && releases == 1);
    zero_result = 0; assert(DjiTargetChase_Init(&config) == 0);
    assert(DjiTargetChase_DeInit() == 0);
}
static void test_takeover_during_stop(void) {
    fresh(); acquire(); takeover_on_zero = true;
    assert(DjiTargetChase_DeInit() == 0);
    assert(commands == 1 && releases == 0 && !DjiTargetChase_IsReleasePending());
}
static void test_no_flight_and_deinit_error(void) {
    fresh(); s_no_flight_mode = true;
    assert(DjiTargetChase_SetManualTrigger(true,1000) == 0);
    assert(DjiTargetChase_EnsureSearchAuthority(1000) == 0);
    assert(DjiTargetChase_DeInit() == 0 && !commands && !releases && !obtains);
    fresh(); deinit_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == deinit_result);
    assert(DjiTargetChase_GetLastStopResult() == deinit_result);
}
static void test_missing_authority_callback_refuses_real_flight(void) {
    fresh(); assert(DjiTargetChase_DeInit() == 0);
    callback_result = DJI_TARGET_CHASE_ERROR_NONSUPPORT;
    assert(DjiTargetChase_Init(&config) == callback_result);
    assert(DjiTargetChase_IsControlInhibited());
    setenv("BALLOON_NO_FLIGHT","1",1);
    assert(DjiTargetChase_Init(&config) == 0);
    assert(DjiTargetChase_DeInit() == 0);
}
int main(void) {
    test_initial_callback_and_takeover();
    test_late_grant_keeps_stop_latched();
    test_release_failure_retry();
    test_zero_failure_is_reported_after_release();
    test_takeover_during_stop();
    test_no_flight_and_deinit_error();
    test_missing_authority_callback_refuses_real_flight();
    puts("authority_stop: 7 groups passed (SDK stubs; no aircraft I/O)");
    return 0;
}
