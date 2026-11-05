/* Host-only inclusion lets the test exercise the exact wrappers and their
 * counters; discarded SDK entry points are never linked or executed. Build
 * with -flto -fwhole-program so uncalled platform entry points are removed. */
#include <stdio.h>
static FILE *profile_test_open(const char *path, const char *mode);
static int profile_test_close(FILE *file);
/* The real flush serializer writes to a temporary sink. Only this test replaces
 * IO, so production path, ownership and error handling remain unchanged. */
#define fopen profile_test_open
#define fclose profile_test_close
#include "../psdk-overlay/samples/sample_c/module_sample/target_chase/dji_target_chase_task.c"
#undef fopen
#undef fclose
#include <assert.h>
#include <fcntl.h>
#ifndef O_BINARY
#define O_BINARY 0
#endif

static unsigned int sdk_calls;
static T_DjiReturnCode sdk_result;
static T_DjiFlightControllerJoystickCommand sdk_command;
static uint64_t test_now_ns = 1000000000ULL;
static FILE *profile_sink;
static unsigned int sink_opens, sink_closes;
static bool sink_fail_open, sink_fail_close;

static FILE *profile_test_open(const char *path, const char *mode)
{
    assert(strcmp(path, DJI_TARGET_CHASE_PERFORMANCE_LOG) == 0);
    assert(strcmp(mode, "a") == 0);
    ++sink_opens;
    return sink_fail_open ? NULL : profile_sink;
}

static int profile_test_close(FILE *file)
{
    assert(file == profile_sink);
    ++sink_closes;
    assert(fflush(file) == 0);
    /* Simulate close reporting an IO failure after data may already be written.
     * A failure window is not the same as an entirely missing window. */
    return sink_fail_close ? EOF : 0;
}

/* Deterministic clock; this test checks instrumentation transparency, not
 * timing performance. No values produced here are onboard measurements. */
int clock_gettime(clockid_t clock_id, struct timespec *value)
{
    assert(clock_id == CLOCK_MONOTONIC);
    test_now_ns += 100;
    value->tv_sec = (time_t)(test_now_ns / 1000000000ULL);
    value->tv_nsec = (long)(test_now_ns % 1000000000ULL);
    return 0;
}
T_DjiReturnCode DjiFlightController_ExecuteJoystickAction(T_DjiFlightControllerJoystickCommand command)
{
    ++sdk_calls;
    sdk_command = command;
    return sdk_result;
}
/* Performance-wrapper fixtures do not enable telemetry authority fallback;
 * any incidental SDK discovery/read must fail rather than reaching hardware. */
T_DjiReturnCode DjiAircraftInfo_GetBaseInfo(T_DjiAircraftInfoBaseInfo *info)
{
    (void)info;
    return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
}
T_DjiReturnCode DjiFcSubscription_GetLatestValueOfTopic(E_DjiFcSubscriptionTopic topic,
    uint8_t *data, uint16_t size, T_DjiDataTimestamp *stamp)
{
    (void)topic; (void)data; (void)size; (void)stamp;
    return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
}

static void prepare_controller(int mode)
{
    T_TargetChaseConfig config = TargetChase_DefaultConfig(640, 480, 73.7f, 45.7f);
    T_TargetChaseAircraftState aircraft = {0};
    config.prediction_mode = mode;
    TargetChase_Init(&s_chase, &config);
    aircraft.in_air = true;
    aircraft.altitude_agl_m = 10;
    aircraft.timestamp_ms = DjiTargetChase_ProfileClockNs() / 1000000ULL;
    TargetChase_UpdateAircraftState(&s_chase, &aircraft);
    TargetChase_SetManualTrigger(&s_chase, true, 1000);
    TargetChase_SetAuthority(&s_chase, true, 1000);
    for (unsigned int i = 0; i < 5; ++i) {
        TargetChase_SetObservationCameraPose(&s_chase, (float)i, 0, 1000+i*100);
        TargetChase_UpdateTargetBox(&s_chase, 300+i, 220, 340+i, 260, 1000+i*100);
    }
}

static uint64_t stage_count(unsigned int stage)
{
    uint64_t total = 0;
    for (unsigned int i = 0; i < 3; ++i) total += s_profile_stats[stage][i].count;
    return total;
}

static void test_flush(const char *fixture_path)
{
    char text[8192];
    char sink_path[96];
    int sink_fd;
    size_t length;
    unsigned int lines = 0;
    /* MinGW's tmpfile can try the drive root. An exclusive test-owned file in
     * the test working directory is portable and never replaces an old file. */
    snprintf(sink_path, sizeof(sink_path), "performance_control_test_%lu.tmp", (unsigned long)getpid());
    sink_fd = open(sink_path, O_CREAT | O_EXCL | O_RDWR | O_BINARY, 0600);
    assert(sink_fd >= 0);
    profile_sink = fdopen(sink_fd, "w+b");
    assert(profile_sink != NULL);
    memset(s_profile_stats, 0, sizeof(s_profile_stats));
    s_profile_enabled = false;
    DjiTargetChase_ProfileFlush(true);
    assert(sink_opens == 0);
    s_profile_enabled = true;
    s_profile_window_start_ns = test_now_ns;
    s_profile_window_id = s_profile_dropped_windows = s_profile_previous_flush_ns = 0;
    s_chase.config.prediction_mode = 1;
    s_no_flight_mode = false;
    strcpy(s_profile_clock_id, "mono-v1:11111111-1111-4111-8111-111111111111");
    strcpy(s_profile_id, "synthetic-c-wrapper-test");
    strcpy(s_profile_mission_token, "synthetic-test-only");
    assert(BalloonPerformance_Add(&s_profile_stats[PROFILE_UPDATE_BOX][1], test_now_ns+100, test_now_ns+1100, true));
    assert(BalloonPerformance_Add(&s_profile_stats[PROFILE_UPDATE_BOX][1], test_now_ns+2000, test_now_ns+4001, false));
    assert(!BalloonPerformance_Add(&s_profile_stats[PROFILE_UPDATE_BOX][1], 0, test_now_ns, false));
    test_now_ns += 1000000ULL;
    DjiTargetChase_ProfileFlush(false);
    assert(sink_opens == 0 && stage_count(PROFILE_UPDATE_BOX) == 2);
    test_now_ns += DJI_TARGET_CHASE_PROFILE_WINDOW_NS;
    DjiTargetChase_ProfileFlush(false);
    assert(sink_opens == 1 && sink_closes == 1 && s_profile_window_id == 1);
    assert(stage_count(PROFILE_UPDATE_BOX) == 0 && s_profile_previous_flush_ns > 0);

    /* An invalid-only window must be serializable without an invented timing. */
    assert(!BalloonPerformance_Add(&s_profile_stats[PROFILE_WORLD][0], 0, test_now_ns, false));
    test_now_ns += DJI_TARGET_CHASE_PROFILE_WINDOW_NS;
    DjiTargetChase_ProfileFlush(false);
    assert(s_profile_window_id == 2 && sink_closes == 2);

    assert(BalloonPerformance_Add(&s_profile_stats[PROFILE_JOYSTICK][2], test_now_ns, test_now_ns+5000, true));
    test_now_ns += 6000;
    sink_fail_close = true;
    DjiTargetChase_ProfileFlush(true);
    sink_fail_close = false;
    assert(s_profile_dropped_windows == 1 && s_profile_window_id == 3);
    assert(stage_count(PROFILE_JOYSTICK) == 0);
    assert(BalloonPerformance_Add(&s_profile_stats[PROFILE_STEP][0], test_now_ns, test_now_ns+12000, true));
    test_now_ns += 13000;
    DjiTargetChase_ProfileFlush(true);
    assert(s_profile_window_id == 4 && s_profile_dropped_windows == 1);

    assert(BalloonPerformance_Add(&s_profile_stats[PROFILE_STEP][0], test_now_ns, test_now_ns+1000, false));
    test_now_ns += 2000;
    sink_fail_open = true;
    DjiTargetChase_ProfileFlush(true);
    sink_fail_open = false;
    assert(s_profile_window_id == 5 && s_profile_dropped_windows == 2);
    assert(stage_count(PROFILE_STEP) == 0);
    assert(BalloonPerformance_Add(&s_profile_stats[PROFILE_STEP][0], test_now_ns, test_now_ns+1000, true));
    test_now_ns += 2000;
    DjiTargetChase_ProfileFlush(true);
    assert(s_profile_window_id == 6 && sink_opens == 6 && sink_closes == 5);
    DjiTargetChase_ProfileFlush(true);
    assert(s_profile_window_id == 7 && sink_opens == 6); /* empty window: no IO */
    rewind(profile_sink);
    length = fread(text, 1, sizeof(text)-1, profile_sink);
    assert(length > 0 && length < sizeof(text)-1 && !ferror(profile_sink));
    text[length] = '\0';
    for (size_t i = 0; i < length; ++i) if (text[i] == '\n') ++lines;
    assert(lines == 5);
    assert(strstr(text, "\"count\":2,\"success_count\":1,\"sum_ns\":3001") != NULL);
    assert(strstr(text, "\"p95_upper_bound_ns\":4000") != NULL);
    assert(strstr(text, "\"count\":0,\"success_count\":0,\"sum_ns\":0") != NULL);
    assert(strstr(text, "\"p95_upper_bound_ns\":null") != NULL);
    assert(strstr(text, "\"dropped_windows_total\":2") != NULL);
    assert(strstr(text, "\"window_id\":4,") == NULL); /* failed open is not replayed */
    if (fixture_path != NULL) {
        FILE *fixture = fopen(fixture_path, "wb");
        assert(fixture != NULL);
        assert(fwrite(text, 1, length, fixture) == length);
        assert(fclose(fixture) == 0);
    }
    assert(fclose(profile_sink) == 0);
    assert(remove(sink_path) == 0);
    s_profile_enabled = false;
}

int main(int argc, char **argv)
{
    T_TargetChaseController before, expected;
    T_TargetChaseCommand actual_command, expected_command;
    T_TargetChasePrediction actual_prediction, expected_prediction;
    T_DjiFlightControllerJoystickCommand input = {1, 2, 3, 4};
    float actual[4], reference[4];
    bool actual_ok, reference_ok;
    s_no_flight_mode = false;
    for (int enabled = 0; enabled <= 1; ++enabled) {
        s_profile_enabled = enabled != 0;
        memset(s_profile_stats, 0, sizeof(s_profile_stats));
        for (int mode = 0; mode <= 2; ++mode) {
            memset(&actual_command, 0, sizeof(actual_command));
            memset(&expected_command, 0, sizeof(expected_command));
            memset(&actual_prediction, 0, sizeof(actual_prediction));
            memset(&expected_prediction, 0, sizeof(expected_prediction));
            memset(actual, 0, sizeof(actual));
            memset(reference, 0, sizeof(reference));
            prepare_controller(mode);
            before = expected = s_chase;
            reference_ok = TargetChase_Step(&expected, 1450, &expected_command);
            actual_ok = DjiTargetChase_ProfileStep(1450, &actual_command);
            assert(actual_ok == reference_ok);
            assert(actual_ok && actual_command.should_send);
            assert(TargetChase_GetMode(&s_chase) == TARGET_CHASE_MODE_ACTIVE);
            assert(memcmp(&s_chase, &expected, sizeof(expected)) == 0);
            assert(memcmp(&actual_command, &expected_command, sizeof(actual_command)) == 0);
            s_chase = before;
            reference_ok = TargetChase_PredictWorld(&before, 1450, &expected_prediction);
            actual_ok = DjiTargetChase_ProfileWorld(1450, &actual_prediction);
            assert(actual_ok == reference_ok);
            assert(actual_ok);
            assert(memcmp(&actual_prediction, &expected_prediction, sizeof(actual_prediction)) == 0);
            expected = s_chase = before;
            reference_ok = TargetChase_PredictHybrid(&expected, 1450, 4, 0,
                &reference[0], &reference[1], &reference[2], &reference[3]);
            actual_ok = DjiTargetChase_ProfileHybrid(1450, 4, 0,
                &actual[0], &actual[1], &actual[2], &actual[3]);
            assert(actual_ok == reference_ok);
            assert(actual_ok == (mode != 0));
            assert(memcmp(actual, reference, sizeof(actual)) == 0);
            assert(memcmp(&s_chase, &expected, sizeof(expected)) == 0);
        }
        assert(stage_count(PROFILE_STEP) == (enabled ? 3U : 0U));
        assert(stage_count(PROFILE_WORLD) == (enabled ? 3U : 0U));
        sdk_calls = 0;
        sdk_result = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        assert(DjiTargetChase_ProfileJoystick(input) == sdk_result);
        sdk_result = DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
        assert(DjiTargetChase_ProfileJoystick(input) == sdk_result);
        assert(sdk_calls == 2 && memcmp(&input, &sdk_command, sizeof(input)) == 0);
        assert(stage_count(PROFILE_JOYSTICK) == (enabled ? 2U : 0U));
        if (enabled) {
            unsigned int successes = 0;
            for (unsigned int i = 0; i < 3; ++i) successes += (unsigned int) s_profile_stats[PROFILE_JOYSTICK][i].success_count;
            assert(successes == 1);
        }
    }
    /* Synthetic, stale and future state readings must remain unknown. */
    memset(s_profile_stats, 0, sizeof(s_profile_stats));
    s_no_flight_mode = true;
    s_chase.aircraft.timestamp_ms = DjiTargetChase_ProfileClockNs()/1000000ULL;
    DjiTargetChase_ProfileEnd(PROFILE_STEP, DjiTargetChase_ProfileBegin(), true);
    s_no_flight_mode = false;
    s_chase.aircraft.timestamp_ms = 1;
    DjiTargetChase_ProfileEnd(PROFILE_STEP, DjiTargetChase_ProfileBegin(), true);
    s_chase.aircraft.timestamp_ms = UINT64_MAX;
    DjiTargetChase_ProfileEnd(PROFILE_STEP, DjiTargetChase_ProfileBegin(), true);
    assert(s_profile_stats[PROFILE_STEP][0].count == 3);
    assert(argc <= 2);
    test_flush(argc == 2 ? argv[1] : NULL);
    puts("profile wrappers preserve controller state/output and SDK calls with timing on/off");
    puts("profile JSON flush, interval, invalid-only windows and IO-failure accounting passed (synthetic clock)");
    return 0;
}
