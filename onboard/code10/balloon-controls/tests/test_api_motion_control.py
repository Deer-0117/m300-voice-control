"""Exercise the API-only control loop with a fake clock, broker and DJI SDK."""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile
import unittest

from test_flight_panel_widget import function

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / 'psdk-overlay/samples/sample_c/module_sample'

PRELUDE = r'''
#include <assert.h>
#include "balloon_api_motion.h"

#define DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS 0
#define DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN 1
#define BALLOON_CONTROL_LOOP_INTERVAL_MS 50
typedef uint64_t T_DjiReturnCode;
typedef struct { T_BalloonVisualContext context; } T_BalloonVisualWatch;
typedef struct { void (*TaskSleepMs)(uint32_t); } T_DjiOsalHandler;
static bool s_stopRequested, s_controlStopRequested;
static uint64_t now = 1000;
static unsigned failure_notice, bridge_closes, inits, releases, requests, authority, writes, zeroes;
static uint64_t last_move_at;
static bool reply_ready;
static const char *scenario;
static void sleep_ms(uint32_t duration) { assert(duration && duration <= 50); now += duration; }
static T_DjiOsalHandler osal = {sleep_ms};
static T_DjiOsalHandler *DjiPlatform_GetOsalHandler(void) { return &osal; }
static uint64_t NowMs(void) { return now; }
static bool BalloonControl_VisualCancelled(const char *token) {
    assert(!strcmp(token, "mission"));
    return s_controlStopRequested;
}
static bool NaturalLanguageStopRequested(void) { return false; }
static bool BalloonControl_WaitForVisual(const char *token, T_BalloonVisualWatch *watch) {
    strcpy(watch->context.token, token);
    return strcmp(scenario, "no-preflight") != 0;
}
static bool BalloonControl_ReadApiVisual(const T_BalloonVisualContext *expected,
                                         T_BalloonVisualReceipt *vision) {
    assert(!strcmp(expected->token, "mission"));
    vision->captureMs = !authority ? 900 :
        writes == 3 && zeroes == 2 ? 1200 : now > 1000 ? 1025 : 1000;
    if (!strcmp(scenario, "telemetry")) vision->captureMs = now - 10;
    if (!strcmp(scenario, "context-changed") && authority) return false;
    if (!strcmp(scenario, "target-lost") && writes) return false;
    return !s_controlStopRequested && now - vision->captureMs <= 1400;
}
static T_DjiReturnCode DjiTargetChase_InitApiMotion(const T_TargetChaseConfig *cfg) {
    (void)cfg; inits++; return 0;
}
static T_DjiReturnCode DjiTargetChase_ReadApiState(T_TargetChaseAircraftState *state) {
    memset(state, 0, sizeof(*state));
    if (!strcmp(scenario, "drift") && reply_ready) state->attitude_deg[2] = 11.0f;
    return !strcmp(scenario, "telemetry") ||
        (!strcmp(scenario, "pending-telemetry") && requests) ||
        (!strcmp(scenario, "idle-telemetry") && authority && zeroes);
}
static T_DjiReturnCode DjiTargetChase_ApiEnsureAuthority(uint64_t deadline) {
    assert(deadline > now && deadline - now <= 600);
    authority++;
    if (!strcmp(scenario, "expired-during-authority")) now = deadline;
    return 0;
}
static T_DjiReturnCode DjiTargetChase_ApiSend(const T_DjiFlightControllerJoystickCommand *command,
                                              uint64_t deadline) {
    if (!command->x && !command->y && !command->z && !command->yaw) {
        assert(!deadline);
        zeroes++;
    } else {
        assert(authority && requests == 2 && deadline > now && deadline - now <= 150);
        assert(command->x == .20f && command->y == -.05f &&
               command->z == .08f && command->yaw == 2.0f);
        if (writes) assert(now - last_move_at == 50);
        last_move_at = now;
        writes++;
    }
    return 0;
}
static bool DjiTargetChase_IsReleasePending(void) { return false; }
static T_DjiReturnCode DjiTargetChase_DeInit(void) { releases++; return 0; }
static void BalloonControl_RecordInitResult(T_DjiReturnCode code, bool cancel) {
    assert(!code && !cancel);
}
static void BalloonVisual_RecordFailure(unsigned reason, const char *color) {
    assert(reason == 4 && !color);
    failure_notice++;
}
static void AppendControlEvent(const char *event, T_DjiReturnCode code, const char *detail) {
    (void)code; (void)detail;
    if (!strcmp(event, "api_motion_command_forwarded"))
        assert(releases || (writes && zeroes >= 2));
}
static bool stub_start(T_BalloonApiMotionBridge *bridge) { (void)bridge; return true; }
static void stub_close(T_BalloonApiMotionBridge *bridge) { (void)bridge; bridge_closes++; }
static bool stub_request(T_BalloonApiMotionBridge *bridge, uint32_t seq,
                         const T_BalloonVisualReceipt *vision,
                         const T_TargetChaseAircraftState *aircraft) {
    (void)bridge; (void)vision; (void)aircraft;
    return seq == ++requests;
}
static bool stub_idle(const T_BalloonApiMotionBridge *bridge) {
    (void)bridge;
    return strcmp(scenario, "bridge-died") != 0 || !writes;
}
static int stub_poll(T_BalloonApiMotionBridge *bridge, uint32_t seq, T_BalloonApiMotionCommand *out) {
    (void)bridge;
    if (!strcmp(scenario, "cancelled")) { s_controlStopRequested = true; return 0; }
    if (!strcmp(scenario, "expired") || seq > 2) return 0;
    reply_ready = seq == 2;
    char reply[200];
    snprintf(reply, sizeof(reply),
             "{\"seq\":%u,\"duration_ms\":150,\"x_mps\":0.20,\"y_mps\":-0.05,"
             "\"z_mps\":0.08,\"yaw_rate_degps\":2,\"provider_id\":\"provider-%u\"}", seq, seq);
    return BalloonApiMotion_Parse(reply, seq, out) ? 1 : -1;
}
#define BalloonApiMotion_Start stub_start
#define BalloonApiMotion_Close stub_close
#define BalloonApiMotion_Request stub_request
#define BalloonApiMotion_Idle stub_idle
#define BalloonApiMotion_Poll stub_poll
'''

EPILOGUE = r'''
static void test_parsing(void) {
    const char *format = "{\"seq\":7,\"duration_ms\":%s,\"x_mps\":%s,\"y_mps\":%s,"
                         "\"z_mps\":0,\"yaw_rate_degps\":0,\"provider_id\":\"%s\"%s}";
    char line[512];
    T_BalloonApiMotionCommand command;
    snprintf(line, sizeof(line), format, "50", "0.20", "0.20", "provider-7", "");
    assert(BalloonApiMotion_Parse(line, 7, &command));
    assert(!BalloonApiMotion_Parse(line, 8, &command));
    const char *cases[][5] = {
        {"200", "0.20", "0.20", "id", ""},
        {"100", "0.30", "0.30", "id", ""},
        {"100", "1e999", "0", "id", ""},
        {"100", "true", "0", "id", ""},
        {"100", "0", "0", "id\\u0000tail", ""},
        {"100", "0", "0", "id", ",\"seq\":7"},
        {"100", "0", "0", "id", ",\"unexpected\":7"}
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        snprintf(line, sizeof(line), format, cases[i][0], cases[i][1], cases[i][2], cases[i][3], cases[i][4]);
        assert(!BalloonApiMotion_Parse(line, 7, &command));
    }
}
int main(int argc, char **argv) {
    assert(argc == 2);
    scenario = argv[1];
    if (!strcmp(scenario, "parse")) { test_parsing(); return 0; }
    T_TargetChaseConfig config = {0};
    T_DjiReturnCode code = BalloonControl_RunApiMotion("mission", &config);
    if (!strcmp(scenario, "no-preflight")) {
        assert(!code && !inits && !releases && !authority && !requests && !writes);
    } else {
        assert(inits == 1 && releases == 1 && bridge_closes == 1);
        if (!strcmp(scenario, "expired")) {
            assert(code && requests == 1 && !authority && !writes && failure_notice);
        } else if (!strcmp(scenario, "expired-during-authority") || !strcmp(scenario, "context-changed")) {
            assert(code && requests == 1 && authority == 1 && !writes && failure_notice);
        } else if (!strcmp(scenario, "cancelled")) {
            assert(requests == 1 && !authority && !writes && !failure_notice);
        } else if (!strcmp(scenario, "target-lost") || !strcmp(scenario, "bridge-died")) {
            assert(code && authority == 1 && writes == 1 && failure_notice);
        } else if (!strcmp(scenario, "telemetry")) {
            assert(code && !authority && !requests && !writes && failure_notice);
        } else if (!strcmp(scenario, "pending-telemetry")) {
            assert(code && !authority && requests == 1 && !writes && failure_notice);
        } else if (!strcmp(scenario, "idle-telemetry")) {
            assert(code && authority == 1 && requests == 1 && !writes && failure_notice);
        } else if (!strcmp(scenario, "drift")) {
            assert(code && authority == 1 && requests == 2 && !writes && failure_notice);
        } else {
            assert(!strcmp(scenario, "success"));
            assert(code && requests == 3 && authority == 1 && writes == 3 && zeroes == 2 && failure_notice);
        }
    }
    return 0;
}
'''


class ApiMotionControlTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sdk = Path(os.environ.get('TEST_PSDK_ROOT', ROOT/'psdk-overlay'))
        source_sdk = sdk/'samples/sample_c/module_sample'
        if not (source_sdk/'utils/cJSON.c').is_file():
            raise unittest.SkipTest('public SDK source unavailable; set TEST_PSDK_ROOT')
        folder = tempfile.TemporaryDirectory(prefix='uav-api-control-')
        cls.addClassCleanup(folder.cleanup)
        folder = Path(folder.name)
        code = (MODULE/'balloon_widget/balloon_photo_detector.c').read_text()
        source = folder/'api_control.c'
        helpers = ['BalloonControl_EarlierDeadline', 'BalloonControl_SleepUntil',
                   'BalloonControl_ApiStateMatches', 'BalloonControl_RunApiMotion']
        source.write_text(PRELUDE + '\n'.join(function(code, name) for name in helpers) + EPILOGUE)
        cls.binary = folder/'api_control'
        result = subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
            '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-deprecated-declarations', '-DCJSON_HIDE_SYMBOLS',
            '-I'+str(MODULE), '-I'+str(MODULE/'balloon_widget'), '-I'+str(source_sdk),
            '-I'+str(sdk/'psdk_lib/include'), str(source), str(source_sdk/'utils/cJSON.c'),
            '-lm', '-o', str(cls.binary)], capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(result.stderr)

    def run_case(self, case):
        result = subprocess.run([str(self.binary), case], capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_command_is_forwarded_exactly_then_zeroed_before_wait(self):
        self.run_case('success')

    def test_fail_closed(self):
        for case in ('no-preflight', 'expired', 'expired-during-authority', 'context-changed',
                     'target-lost', 'bridge-died', 'cancelled', 'telemetry',
                     'pending-telemetry', 'idle-telemetry', 'drift'):
            with self.subTest(case=case):
                self.run_case(case)

    def test_strict_reply_parsing(self):
        self.run_case('parse')


if __name__ == '__main__':
    unittest.main()
