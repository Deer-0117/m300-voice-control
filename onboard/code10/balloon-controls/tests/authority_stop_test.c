/* Real adapter + deterministic SDK stubs. No DJI library, file writes or device I/O. */
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
static uint64_t test_clock_ms = 100000;
static int test_clock_gettime(clockid_t id, struct timespec *ts) {
    (void)id; ts->tv_sec = test_clock_ms / 1000; ts->tv_nsec = (test_clock_ms % 1000) * 1000000; return 0;
}
static int test_usleep(useconds_t usec);
#define clock_gettime test_clock_gettime
#define usleep test_usleep
static FILE *blocked_open(const char *path, const char *mode) { (void)path; (void)mode; return NULL; }
#define fopen blocked_open
#include "../psdk-overlay/samples/sample_c/module_sample/target_chase/dji_target_chase_task.c"
#undef fopen
#include <assert.h>

static unsigned obtains, releases, commands, rotations, resets, deinits;
static unsigned gimbal_modes, nonzero_rotations;
static T_DjiGimbalManagerRotation last_rotation;
static T_DjiReturnCode release_result, zero_result, deinit_result, callback_result;
static T_DjiReturnCode obtain_result, subscribe_result;
static E_DjiAircraftType aircraft_type = DJI_AIRCRAFT_TYPE_UNKNOWN;
static DjiReceiveDataOfTopicCallback control_device_callback;
static unsigned control_samples_on_subscribe, grant_after_sleeps, wait_sleeps;
static unsigned stop_after_sleeps;
static uint32_t control_stamp = 1000;
static bool stream_samples, grant_immediately, duplicate_samples;
static uint8_t test_device;
static bool test_open;
static void control_sample(uint8_t device, bool open, bool advance) {
    T_DjiFcSubscriptionControlDevice sample = {0};
    sample.deviceStatus = device; sample.flightStatus = open;
    if (advance) control_stamp += 20;
    T_DjiDataTimestamp stamp = {.millisecond = control_stamp, .microsecond = 0};
    test_device = device; test_open = open;
    if (control_device_callback)
        assert(control_device_callback((const uint8_t *)&sample, sizeof(sample), &stamp) == 0);
}
static int test_usleep(useconds_t usec) {
    test_clock_ms += usec / 1000;
    ++wait_sleeps;
    if (stop_after_sleeps && wait_sleeps == stop_after_sleeps)
        DjiTargetChase_RequestStop();
    if (stream_samples && control_device_callback) {
        if (obtains && grant_after_sleeps && wait_sleeps >= grant_after_sleeps)
            control_sample(4, true, !duplicate_samples);
        else control_sample(test_device, test_open, !duplicate_samples);
    }
    return 0;
}
T_DjiReturnCode DjiAircraftInfo_GetBaseInfo(T_DjiAircraftInfoBaseInfo *info) {
    memset(info,0,sizeof(*info)); info->aircraftType = aircraft_type; return 0;
}
static bool takeover_on_zero, stop_on_gimbal_init, stop_on_gimbal_mode;
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
T_DjiReturnCode DjiFlightController_ObtainJoystickCtrlAuthority(void) {
    ++obtains; wait_sleeps = 0;
    if (s_control_device.enabled) { if (grant_immediately) control_sample(4,true,true); }
    else owner(true);
    return obtain_result;
}
T_DjiReturnCode DjiFlightController_ReleaseJoystickCtrlAuthority(void) {
    ++releases;
    if (!release_result) {
        if (s_control_device.enabled) control_sample(0,false,true);
        else owner(false);
    }
    return release_result;
}
T_DjiReturnCode DjiFlightController_ExecuteJoystickAction(T_DjiFlightControllerJoystickCommand command) {
    ++commands; last_command = command; if (takeover_on_zero) owner(false); return zero_result;
}
T_DjiReturnCode DjiFcSubscription_Init(void) { return 0; }
T_DjiReturnCode DjiFcSubscription_DeInit(void) { return 0; }
T_DjiReturnCode DjiFcSubscription_SubscribeTopic(E_DjiFcSubscriptionTopic topic,E_DjiDataSubscriptionTopicFreq freq,DjiReceiveDataOfTopicCallback cb) {
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE) {
        assert(freq == DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ);
        if (subscribe_result) return subscribe_result;
        control_device_callback = cb;
        for (unsigned i = 0; i < control_samples_on_subscribe; ++i)
            control_sample(test_device,test_open,true);
    }
    return 0;
}
T_DjiReturnCode DjiFcSubscription_UnSubscribeTopic(E_DjiFcSubscriptionTopic topic) {
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE) control_device_callback = NULL;
    return 0;
}
T_DjiReturnCode DjiFcSubscription_GetLatestValueOfTopic(E_DjiFcSubscriptionTopic topic,uint8_t *data,uint16_t size,T_DjiDataTimestamp *stamp) {
    memset(data,0,size); if (stamp) { memset(stamp,0,sizeof(*stamp)); stamp->millisecond = 1000; }
    if (topic == DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION) ((T_DjiFcSubscriptionQuaternion*)data)->q0 = 1;
    return 0;
}
T_DjiOsalHandler *DjiPlatform_GetOsalHandler(void) { return NULL; }
T_DjiReturnCode DjiGimbalManager_Init(void) { if (stop_on_gimbal_init) DjiTargetChase_RequestStop(); return 0; }
T_DjiReturnCode DjiGimbalManager_Deinit(void) { return 0; }
T_DjiReturnCode DjiGimbalManager_SetMode(E_DjiMountPosition pos,E_DjiGimbalMode mode) { (void)pos; (void)mode; ++gimbal_modes; if (stop_on_gimbal_mode) DjiTargetChase_RequestStop(); return 0; }
T_DjiReturnCode DjiGimbalManager_Reset(E_DjiMountPosition pos,E_DjiGimbalResetMode mode) { (void)pos; (void)mode; ++resets; return 0; }
T_DjiReturnCode DjiGimbalManager_Rotate(E_DjiMountPosition pos,T_DjiGimbalManagerRotation rot) { (void)pos; last_rotation = rot; ++rotations; if (rot.pitch != 0 || rot.roll != 0 || rot.yaw != 0) ++nonzero_rotations; return 0; }
T_DjiReturnCode DjiGimbalManager_SetControllerMaxSpeedPercentage(E_DjiMountPosition pos,E_DjiGimbalAxis axis,uint8_t val) { (void)pos; (void)axis; (void)val; return 0; }
T_DjiReturnCode DjiGimbalManager_SetControllerSmoothFactor(E_DjiMountPosition pos,E_DjiGimbalAxis axis,uint8_t val) { (void)pos; (void)axis; (void)val; return 0; }

static T_TargetChaseConfig config;
static void fresh(void) {
    DjiTargetChase_ControlDeviceStop();
    /* Each case simulates a new SDK process. */
    memset(s_balloonSharedTopics, 0, sizeof(s_balloonSharedTopics));
    s_balloonSubscriptionInitialized = false;
    control_device_callback = NULL;
    aircraft_type = DJI_AIRCRAFT_TYPE_UNKNOWN; obtain_result = subscribe_result = 0;
    control_samples_on_subscribe = grant_after_sleeps = wait_sleeps = stop_after_sleeps = 0;
    stream_samples = grant_immediately = duplicate_samples = false;
    test_device = 0; test_open = false; test_clock_ms += 1000;
    s_initialized = false; s_real_authority_obtained = false; s_control_inhibited = false;
    s_manual_takeover = false; s_ever_had_authority = false; s_intentional_release = false;
    s_gimbal_manager_initialized = false; s_gimbal_angles_subscribed = false;
    s_panel_maintenance_mode = -1;
    release_result = zero_result = deinit_result = callback_result = 0;
    takeover_on_zero = stop_on_gimbal_init = stop_on_gimbal_mode = false; obtains = releases = commands = rotations = resets = deinits = 0;
    gimbal_modes = nonzero_rotations = 0; memset(&last_rotation, 0, sizeof(last_rotation));
    config = TargetChase_DefaultConfig(640,480,73.7f,45.7f);
    setenv("BALLOON_NO_FLIGHT","0",1); setenv("BALLOON_GIMBAL_TRACKING","0",1);
    unsetenv("BALLOON_GIMBAL_STATIC_TEST"); unsetenv("BALLOON_GIMBAL_SEARCH"); unsetenv("BALLOON_BODY_SEARCH");
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

static void compatibility_prepare(void) {
    fresh(); assert(DjiTargetChase_DeInit() == 0);
    callback_result = DJI_TARGET_CHASE_ERROR_NONSUPPORT;
    aircraft_type = DJI_AIRCRAFT_TYPE_M300_RTK;
    control_samples_on_subscribe = 2; stream_samples = true;
}
static void compatibility_start(void) {
    compatibility_prepare(); assert(DjiTargetChase_Init(&config) == 0);
    assert(s_control_device.ready && s_control_device.subscribed);
    assert(!DjiTargetChase_IsControlInhibited() && !s_real_authority_obtained && !commands);
}
static void compatibility_acquire(void) {
    grant_immediately = true; acquire();
    assert(s_control_device.confirmed && DjiTargetChase_ControlDeviceHealthy(true));
}
static void test_compatibility_models_and_subscription_failures(void) {
    compatibility_prepare(); aircraft_type = DJI_AIRCRAFT_TYPE_UNKNOWN;
    assert(DjiTargetChase_Init(&config) == DJI_TARGET_CHASE_ERROR_NONSUPPORT);
    assert(!s_initialized && !control_device_callback && !commands && !obtains);
    compatibility_prepare(); aircraft_type = DJI_AIRCRAFT_TYPE_M350_RTK;
    assert(DjiTargetChase_Init(&config) == 0); assert(DjiTargetChase_DeInit() == 0);
    compatibility_prepare(); subscribe_result = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
    assert(DjiTargetChase_Init(&config) == subscribe_result);
    assert(!s_initialized && !control_device_callback && !commands && !obtains);
    compatibility_prepare(); callback_result = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
    assert(DjiTargetChase_Init(&config) == callback_result);
    assert(!control_device_callback && !commands && !obtains);
}
static void test_compatibility_requires_advancing_initial_samples(void) {
    compatibility_prepare(); control_samples_on_subscribe = 0; stream_samples = false;
    assert(DjiTargetChase_Init(&config) == DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT);
    assert(!s_initialized && !s_balloonControlDeviceCallback && !commands);
    compatibility_prepare(); control_samples_on_subscribe = 1; duplicate_samples = true;
    assert(DjiTargetChase_Init(&config) == DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT);
    assert(!s_initialized && !s_balloonControlDeviceCallback && !commands);
}
static void test_compatibility_delayed_confirmation_and_takeover(void) {
    compatibility_start(); grant_after_sleeps = 5;
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) == 0);
    assert(wait_sleeps == 5 && obtains == 1 && !commands && s_control_device.confirmed);
    T_DjiFlightControllerJoystickCommand cmd = {.x = .1f};
    assert(DjiTargetChase_ProfileJoystick(cmd) == 0 && commands == 1);
    control_sample(1,true,true); /* App/RC precedence even while SDK open bit stays 1. */
    assert(DjiTargetChase_IsControlInhibited() && !DjiTargetChase_IsReleasePending());
    assert(DjiTargetChase_ProfileJoystick(cmd) != 0);
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) != 0);
    assert(DjiTargetChase_DeInit() == 0 && commands == 1 && !releases && obtains == 1);
}
static void test_compatibility_stale_duplicate_and_zero_guard(void) {
    compatibility_start(); compatibility_acquire();
    uint64_t receipt = s_control_device.receipt_ms;
    test_clock_ms += DJI_TARGET_CHASE_CONTROL_DEVICE_FRESH_MS;
    control_sample(4,true,false);
    assert(s_control_device.receipt_ms == receipt && DjiTargetChase_ControlDeviceHealthy(true));
    ++test_clock_ms; control_sample(4,true,false);
    T_DjiFlightControllerJoystickCommand zero = {0};
    assert(DjiTargetChase_ProfileJoystick(zero) != 0 && !commands);
    assert(DjiTargetChase_IsControlInhibited() && DjiTargetChase_IsReleasePending());
    release_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == release_result && !commands && releases == 1);
    assert(DjiTargetChase_IsReleasePending() && s_initialized);
    release_result = 0; assert(DjiTargetChase_DeInit() == 0 && !commands && releases == 2);
    assert(!DjiTargetChase_IsReleasePending() && !s_balloonControlDeviceCallback);
}
static void test_compatibility_invalid_and_regressed_samples(void) {
    compatibility_start(); compatibility_acquire();
    T_DjiDataTimestamp stamp = {.millisecond = control_stamp + 20};
    uint8_t invalid = 0;
    assert(control_device_callback(&invalid, 1, &stamp) == 0);
    assert(DjiTargetChase_IsControlInhibited() && !DjiTargetChase_ControlDeviceHealthy(true));
    assert(DjiTargetChase_DeInit() == 0 && !commands && releases == 1);
    compatibility_start(); compatibility_acquire();
    control_stamp -= 100; control_sample(4,true,false);
    assert(DjiTargetChase_IsControlInhibited());
    assert(DjiTargetChase_DeInit() == 0 && !commands && releases == 1);
    compatibility_start(); compatibility_acquire();
    control_sample(2,true,true);
    assert(DjiTargetChase_IsControlInhibited());
    assert(DjiTargetChase_DeInit() == 0 && !commands && releases == 1);
}
static void test_compatibility_timeout_and_late_grant(void) {
    compatibility_start();
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) != 0);
    assert(obtains == 1 && !commands && DjiTargetChase_IsReleasePending());
    T_DjiFlightControllerJoystickCommand zero = {0};
    assert(DjiTargetChase_ProfileJoystick(zero) != 0 && !commands);
    release_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == release_result && DjiTargetChase_IsReleasePending());
    assert(!commands && releases == 1);
    control_sample(4,true,true);
    assert(DjiTargetChase_IsControlInhibited() && DjiTargetChase_IsReleasePending());
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) != 0 && obtains == 1);
    release_result = 0; assert(DjiTargetChase_DeInit() == 0 && releases == 2);
}
static void test_compatibility_ack_failure_retains_pending(void) {
    compatibility_start(); obtain_result = DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) == obtain_result);
    assert(DjiTargetChase_IsReleasePending() && !commands);
    /* RC samples preceding propagation do not prove the timed-out request failed. */
    control_sample(0,false,true); assert(DjiTargetChase_IsReleasePending());
    release_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == release_result && DjiTargetChase_IsReleasePending());
    control_sample(4,true,true); assert(DjiTargetChase_IsControlInhibited());
    release_result = 0; assert(DjiTargetChase_DeInit() == 0 && releases == 2 && obtains == 1);
}
static void test_compatibility_stop_interrupts_grant_wait(void) {
    compatibility_start(); grant_after_sleeps = 5; stop_after_sleeps = 2;
    assert(DjiTargetChase_EnsureSearchAuthority(DjiTargetChase_NowMs()) != 0);
    assert(wait_sleeps == 2 && !commands && DjiTargetChase_IsReleasePending());
    assert(DjiTargetChase_DeInit() == 0 && releases == 1 && !commands);
}
static void test_compatibility_takeover_late_grant_stays_stopped(void) {
    compatibility_start(); compatibility_acquire();
    control_sample(0,false,true);
    assert(DjiTargetChase_IsControlInhibited() && !s_real_authority_obtained);
    control_sample(4,true,true);
    assert(DjiTargetChase_IsControlInhibited() && DjiTargetChase_IsReleasePending());
    T_DjiFlightControllerJoystickCommand cmd = {.x = .1f};
    assert(DjiTargetChase_ProfileJoystick(cmd) != 0 && !commands);
    assert(DjiTargetChase_DeInit() == 0 && releases == 1 && obtains == 1);
}
static void test_compatibility_all_command_paths_guarded(void) {
    compatibility_start(); compatibility_acquire();
    s_last_command.should_send = true; s_last_command.x_mps = .1f; s_has_last_command = true;
    test_clock_ms += DJI_TARGET_CHASE_CONTROL_DEVICE_FRESH_MS + 1;
    assert(DjiTargetChase_Hover(DjiTargetChase_NowMs()) != 0);
    assert(DjiTargetChase_RepeatLastCommand(DjiTargetChase_NowMs()) != 0);
    assert(DjiTargetChase_SendCachedCommand(DjiTargetChase_NowMs(),"test",NULL) != 0);
    assert(DjiTargetChase_SendBodySearchCommand(.1f,0,0,DjiTargetChase_NowMs(),"test",true) != 0);
    assert(!commands); assert(DjiTargetChase_DeInit() == 0 && !commands && releases == 1);
    compatibility_start(); compatibility_acquire();
    release_result = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    assert(DjiTargetChase_DeInit() == release_result && DjiTargetChase_IsReleasePending());
    assert(commands == 1 && last_command.x == 0 && last_command.y == 0 && last_command.z == 0 && last_command.yaw == 0);
    release_result = 0; assert(DjiTargetChase_DeInit() == 0 && !DjiTargetChase_IsReleasePending());
}
static void test_compatibility_stop_during_gimbal_initialization(void) {
    compatibility_prepare(); setenv("BALLOON_GIMBAL_TRACKING","1",1); stop_on_gimbal_init = true;
    assert(DjiTargetChase_Init(&config) == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE);
    assert(!s_initialized && !s_gimbal_manager_initialized && !resets && !commands && !obtains);
    compatibility_prepare(); setenv("BALLOON_GIMBAL_TRACKING","1",1); stop_on_gimbal_mode = true;
    assert(DjiTargetChase_Init(&config) == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE);
    assert(!s_initialized && !s_gimbal_manager_initialized && !resets && !commands && !obtains);
}

static void managed_gimbal_start(void) {
    fresh(); assert(DjiTargetChase_DeInit() == 0);
    /* Service environment must not turn panel tasks into static tests or
     * make them reset/search, and must not suppress valid target tracking. */
    setenv("BALLOON_NO_FLIGHT","1",1);
    setenv("BALLOON_GIMBAL_STATIC_TEST","1",1);
    setenv("BALLOON_GIMBAL_SEARCH","1",1);
    setenv("BALLOON_BODY_SEARCH","1",1);
    assert(DjiTargetChase_SetPanelMaintenance(false) == 0);
    assert(DjiTargetChase_Init(&config) == 0);
    assert(!s_no_flight_mode && s_gimbal_tracking_enabled);
    assert(!s_gimbal_static_test_mode && !s_gimbal_search_enabled && !s_body_search_enabled);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY && s_last_gimbal_target_seen_ms == 0);
    assert(!resets && !gimbal_modes && !rotations && !commands && !obtains);
}

static void test_panel_waits_without_reset_or_search(void) {
    managed_gimbal_start();
    assert(DjiTargetChase_SetManualTrigger(true, test_clock_ms) == 0);
    for (unsigned i = 0; i < 50; ++i) {
        test_clock_ms += 100;
        DjiTargetChase_GimbalTick(test_clock_ms);
        assert(DjiTargetChase_SearchTick(test_clock_ms) == 0);
    }
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY);
    assert(!resets && !gimbal_modes && !rotations && !commands && !obtains);
    /* Exercise the shutdown helper before Abort inhibits control, too. */
    DjiTargetChase_DeInitGimbalTracking(test_clock_ms);
    assert(!resets && !gimbal_modes && !nonzero_rotations);
    assert(DjiTargetChase_DeInit() == 0);

    fresh(); assert(DjiTargetChase_DeInit() == 0);
    setenv("BALLOON_GIMBAL_TRACKING","1",1);
    setenv("BALLOON_GIMBAL_STATIC_TEST","1",1);
    setenv("BALLOON_GIMBAL_SEARCH","1",1);
    setenv("BALLOON_BODY_SEARCH","1",1);
    assert(DjiTargetChase_Init(&config) == 0);
    assert(s_gimbal_static_test_mode && s_gimbal_search_enabled && s_body_search_enabled);
    assert(resets == 1 && gimbal_modes == 1);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_RESETTING);
    DjiTargetChase_GimbalTick(test_clock_ms);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_STATIC_TEST);
    assert(DjiTargetChase_DeInit() == 0);

    setenv("BALLOON_GIMBAL_STATIC_TEST","0",1);
    assert(DjiTargetChase_Init(&config) == 0);
    assert(resets == 2 && gimbal_modes == 2);
    test_clock_ms += 100;
    DjiTargetChase_GimbalTick(test_clock_ms);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_SEARCHING);
    assert(nonzero_rotations > 0); /* Unmanaged raster search is unchanged. */
    assert(DjiTargetChase_DeInit() == 0);
}

static void test_panel_tracks_visible_target_without_edge_recovery(void) {
    managed_gimbal_start(); test_clock_ms += 100;
    assert(DjiTargetChase_OnTargetBox(640,480,540,300,640,180,test_clock_ms) == 0);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING);
    assert(nonzero_rotations == 1 && last_rotation.yaw > 0);
    assert(fabsf(last_rotation.yaw) <= DJI_TARGET_CHASE_GIMBAL_TRACK_MAX_SPEED_DEGPS);
    assert(s_last_gimbal_target_near_edge &&
           s_chase.filtered_box_angle_deg >= DJI_TARGET_CHASE_GIMBAL_NEAR_BOX_ANGLE_DEG);
    assert(DjiTargetChase_SetManualTrigger(true, test_clock_ms) == 0);
    test_clock_ms += DJI_TARGET_CHASE_GIMBAL_EDGE_RECOVERY_START_MS + 1;
    DjiTargetChase_GimbalTick(test_clock_ms);
    assert(DjiTargetChase_SearchTick(test_clock_ms) == 0);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING && !s_body_search_turn_active);
    assert(nonzero_rotations == 1 && !commands && !obtains);
    test_clock_ms += DJI_TARGET_CHASE_GIMBAL_NEAR_TARGET_LOST_TIMEOUT_MS + 1;
    DjiTargetChase_GimbalTick(test_clock_ms);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY && s_last_gimbal_target_seen_ms == 0);
    assert(last_rotation.pitch == 0 && last_rotation.yaw == 0);
    assert(nonzero_rotations == 1 && !resets && !gimbal_modes && !commands && !obtains);
    assert(DjiTargetChase_DeInit() == 0);
}

static void test_panel_stage_change_waits_for_new_visible_target(void) {
    managed_gimbal_start();
    for (unsigned fast = 0; fast < 2; ++fast) {
        test_clock_ms += 100;
        assert(DjiTargetChase_OnTargetPixel(500,240,test_clock_ms) == 0);
        assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_TRACKING);
        assert(nonzero_rotations == fast + 1);
        assert(DjiTargetChase_RequestSearch(test_clock_ms, fast != 0) == 0);
        assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY && !s_chase.target.valid);
        assert(!s_body_search_turn_active && s_last_gimbal_target_seen_ms == 0);
        assert(last_rotation.pitch == 0 && last_rotation.yaw == 0);
        test_clock_ms += 5000;
        DjiTargetChase_GimbalTick(test_clock_ms);
        assert(DjiTargetChase_SearchTick(test_clock_ms) == 0);
        assert(nonzero_rotations == fast + 1 && !resets && !gimbal_modes && !commands && !obtains);
    }
    assert(DjiTargetChase_DeInit() == 0);
}
static void test_telemetry_first_keeps_authority_monitor_live(void) {
    compatibility_prepare();
    assert(BalloonSubscription_Acquire(BALLOON_SUB_BACKGROUND,
        DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL) == 0);
    DjiReceiveDataOfTopicCallback subscriber = control_device_callback;
    assert(subscriber && !s_balloonControlDeviceCallback);
    assert(DjiTargetChase_Init(&config) == 0);
    assert(control_device_callback == subscriber);
    compatibility_acquire();
    assert(DjiTargetChase_DeInit() == 0);
    assert(control_device_callback == subscriber && !s_balloonControlDeviceCallback);
    assert(!s_control_device.enabled && !s_control_device.valid);
    control_sample(4, true, true);
    assert(!s_control_device.valid && !s_real_authority_obtained);
}

static void test_prediction_status_reports_fallback_and_stop(void) {
    char text[256];
    fresh();
    T_TargetChaseCommand command = {.should_send = true};
    s_chase.mode = TARGET_CHASE_MODE_ACTIVE;
    s_chase.config.prediction_mode = 3;
    s_chase.kf_reason = TARGET_KF_NO_CALIBRATION;
    DjiTargetChase_PublishPredictionStatus(DjiTargetChase_NowMs(), true, &command);
    assert(DjiTargetChase_PredictionStatus(text, sizeof(text)));
    assert(strstr(text, "像素预测") && strstr(text, "缺有效时间校准"));
    s_chase.kf_active = true;
    s_chase.kf_reason = 0;
    DjiTargetChase_PublishPredictionStatus(DjiTargetChase_NowMs(), true, &command);
    assert(DjiTargetChase_PredictionStatus(text, sizeof(text)) && strstr(text, "延迟卡尔曼"));
    test_clock_ms += 2001;
    assert(!DjiTargetChase_PredictionStatus(text, sizeof(text)) && strstr(text, "已过期"));
    DjiTargetChase_PublishPredictionStatus(DjiTargetChase_NowMs(), true, &command);
    DjiTargetChase_RequestStop();
    assert(!DjiTargetChase_PredictionStatus(text, sizeof(text)));
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
    test_compatibility_models_and_subscription_failures();
    test_compatibility_requires_advancing_initial_samples();
    test_compatibility_delayed_confirmation_and_takeover();
    test_compatibility_stale_duplicate_and_zero_guard();
    test_compatibility_invalid_and_regressed_samples();
    test_compatibility_timeout_and_late_grant();
    test_compatibility_ack_failure_retains_pending();
    test_compatibility_stop_interrupts_grant_wait();
    test_compatibility_takeover_late_grant_stays_stopped();
    test_compatibility_all_command_paths_guarded();
    test_compatibility_stop_during_gimbal_initialization();
    test_panel_waits_without_reset_or_search();
    test_panel_tracks_visible_target_without_edge_recovery();
    test_panel_stage_change_waits_for_new_visible_target();
    test_telemetry_first_keeps_authority_monitor_live();
    test_prediction_status_reports_fallback_and_stop();
    puts("authority_stop: 23 groups passed (SDK stubs; no aircraft I/O)");
    return 0;
}
