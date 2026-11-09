"""Run actual initialization-result and operator-notice code with SDK substitutes."""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
import unittest

from test_flight_panel_widget import function

WIDGET = Path(__file__).resolve().parents[1] / 'psdk-overlay/samples/sample_c/module_sample/balloon_widget'

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint64_t T_DjiReturnCode;
typedef void *T_DjiTaskHandle;
#define BALLOON_NL_STOP_FILE "stop.request"
#define DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS 0
#define DJI_ERROR_SYSTEM_MODULE_CODE_BUSY 0x102
#define DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN 0xFF
#define USER_LOG_INFO(...) ((void)0)
#define USER_LOG_ERROR(...) ((void)0)
static T_DjiReturnCode s_controlInitResult;
static unsigned s_visualFailureReason, s_visualFailureColor;
static bool s_calibrationLiveviewOwned, s_workerActive, s_controlActive;
static bool s_controlStopRequested, s_stopRequested, s_nlStopObserved, s_hitUnconfirmedStop;
static T_DjiTaskHandle s_workerTask;
static T_DjiReturnCode init_result, stop_result, control_start_result, cleanup_result;
static bool cancel_in_init, release_pending;
static unsigned init_calls, create_calls, stop_latch_calls, cleanup_calls, ready_calls, trigger_calls;
static char last_message[512];
static T_DjiReturnCode create_task(const char *name, void *(*fn)(void *),
                                  uint32_t stack, void *arg, T_DjiTaskHandle *handle) {
    (void)name; (void)fn; (void)stack; (void)arg; (void)handle;
    create_calls++; return 0;
}
typedef struct { T_DjiReturnCode (*TaskCreate)(const char *, void *(*)(void *),
                                           uint32_t, void *, T_DjiTaskHandle *); } T_DjiOsalHandler;
static T_DjiOsalHandler osal = {create_task};
static T_DjiOsalHandler *DjiPlatform_GetOsalHandler(void) { return &osal; }
static T_DjiReturnCode BalloonControl_Start(void) { return control_start_result; }
static void DjiTargetChase_RequestStop(void) { stop_latch_calls++; }
static void *PhotoDetectorTask(void *arg) { return arg; }
static void BalloonStep_Show(const char *a, const char *b) { (void)a; (void)b; }
static void BalloonStep_ShowSeq(const char *a, uint32_t b, T_DjiReturnCode c, const char *d) {
    (void)a; (void)b; (void)c; (void)d;
}
static void AppendControlEvent(const char *a, T_DjiReturnCode b, const char *c) {
    (void)a; (void)b; (void)c;
}
static void BalloonWidget_ShowMessage(const char *s) {
    snprintf(last_message,sizeof(last_message),"%s",s);
}
static bool DjiTargetChase_IsReleasePending(void) { return release_pending; }
static T_DjiReturnCode DjiTargetChase_GetLastStopResult(void) { return stop_result; }
static T_DjiReturnCode DjiTargetChase_Init(const int *config) {
    (void)config; init_calls++; s_controlStopRequested=cancel_in_init; return init_result;
}
static T_DjiReturnCode DjiTargetChase_DeInit(void) {
    cleanup_calls++; stop_result=cleanup_result; return cleanup_result;
}
'''

EPILOGUE = r'''
static void begin(void) {
    s_workerActive=s_controlActive=false;
    s_controlStopRequested=false;
    assert(BalloonPhotoDetector_Start()==0);
    assert(BalloonPhotoDetector_GetControlInitResult()==0);
}
static void test_failure_survives_cleanup(void) {
    begin(); init_result=0xE0;
    assert(actual_init_step()==0xE0 && init_calls==1);
    assert(!ready_calls && !trigger_calls);
    /* A later successful teardown is deliberately independent of init. */
    stop_result=0;
    BalloonWidget_ShowStoppedNotice();
    assert(strstr(last_message,"飞控初始化失败") && strstr(last_message,"0xE0"));
    assert(strstr(last_message,"任务未执行") && !strstr(last_message,"可选择新任务"));
    release_pending=true; BalloonWidget_ShowStoppedNotice();
    assert(strstr(last_message,"控制权释放失败"));
    release_pending=false; BalloonWidget_ShowStoppedNotice();
    assert(strstr(last_message,"0xE0"));
}
static void test_next_task_and_busy(void) {
    begin(); init_result=0xE0; actual_init_step();
    assert(BalloonPhotoDetector_Start()==DJI_ERROR_SYSTEM_MODULE_CODE_BUSY);
    assert(BalloonPhotoDetector_GetControlInitResult()==0xE0);
    begin(); init_result=0; actual_init_step(); stop_result=0;
    assert(ready_calls==1 && trigger_calls==1);
    BalloonWidget_ShowStoppedNotice();
    assert(strstr(last_message,"任务已结束") && !strstr(last_message,"初始化失败"));
}
static void test_cancellation_and_normal_stop_error(void) {
    begin();
    /* Cancellation before initialization leaves the reset result untouched. */
    BalloonWidget_ShowStoppedNotice();
    assert(!strstr(last_message,"初始化失败"));
    init_result=0xE4; cancel_in_init=true;
    assert(actual_init_step()==0 && cleanup_calls==1);
    assert(!ready_calls && !trigger_calls);
    assert(BalloonPhotoDetector_GetControlInitResult()==0);
    BalloonWidget_ShowStoppedNotice();
    assert(!strstr(last_message,"初始化失败"));
    stop_result=0xE1; BalloonWidget_ShowStoppedNotice();
    assert(strstr(last_message,"停止过程有错误"));
}
static void test_successful_init_cancelled_before_ready(void) {
    begin(); init_result=0; cancel_in_init=true;
    assert(actual_init_step()==0 && cleanup_calls==1);
    assert(!ready_calls && !trigger_calls && BalloonPhotoDetector_GetControlInitResult()==0);
    /* A release/cleanup error is preserved, while cancellation is not called
     * an initialization failure. */
    begin(); cleanup_result=0xE1;
    assert(actual_init_step()==0xE1 && cleanup_calls==2);
    BalloonWidget_ShowStoppedNotice();
    assert(!strstr(last_message,"初始化失败") && strstr(last_message,"停止过程有错误"));
}
static void test_stop_latches_immediately_and_when_already_pending(void) {
    s_controlStopRequested=false;
    BalloonControl_Stop();
    assert(s_controlStopRequested && stop_latch_calls==1);
    BalloonControl_Stop();
    assert(s_controlStopRequested && stop_latch_calls==2);
    assert(!init_calls && !create_calls);
}
static void test_visual_notice_persists_and_next_task_resets(void) {
    begin();BalloonVisual_RecordFailure(1,"purple");stop_result=0;
    BalloonWidget_ShowStoppedNotice();assert(strstr(last_message,"未识别到紫色气球"));
    assert(BalloonPhotoDetector_Start()==DJI_ERROR_SYSTEM_MODULE_CODE_BUSY);
    BalloonWidget_ShowStoppedNotice();assert(strstr(last_message,"未识别到紫色气球"));
    release_pending=true;BalloonWidget_ShowStoppedNotice();assert(strstr(last_message,"控制权释放失败"));
    release_pending=false;BalloonWidget_ShowStoppedNotice();assert(strstr(last_message,"未识别到紫色气球"));
    begin();BalloonWidget_ShowStoppedNotice();assert(!strstr(last_message,"未识别到"));
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"failure")) test_failure_survives_cleanup();
    else if(!strcmp(argv[1],"next")) test_next_task_and_busy();
    else if(!strcmp(argv[1],"cancel")) test_cancellation_and_normal_stop_error();
    else if(!strcmp(argv[1],"stop")) test_stop_latches_immediately_and_when_already_pending();
    else if(!strcmp(argv[1],"cancel_success")) test_successful_init_cancelled_before_ready();
    else if(!strcmp(argv[1],"visual")) test_visual_notice_persists_and_next_task_resets();
    else assert(false);
    return 0;
}
'''


class ControlInitNotice(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='uav-control-notice-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.folder = Path(cls.temp.name)
        detector = (WIDGET / 'balloon_photo_detector.c').read_text()
        widget = (WIDGET / 'psdk_balloon_widget.c').read_text()
        step = re.search(r'    code = DjiTargetChase_Init\(&config\);\n.*?'
                         r'(?=    noFlightMode = DjiTargetChase_IsNoFlightMode\(\);)', detector, re.DOTALL)
        if step is None:
            raise AssertionError('Control task must capture the initialization result before cleanup')
        functions = [function(detector, 'BalloonVisual_ColorText'),
                     function(detector, 'BalloonVisual_RecordFailure'),
                     function(detector, 'BalloonPhotoDetector_GetVisualFailure'),
                     function(detector, 'BalloonControl_RecordInitResult'),
                     function(detector, 'BalloonPhotoDetector_GetControlInitResult'),
                     function(detector, 'BalloonControl_Stop'),
                     function(detector, 'BalloonPhotoDetector_Start'),
                     function(widget, 'BalloonWidget_ShowStoppedNotice')]
        wrapper = ('\nstatic T_DjiReturnCode actual_init_step(void) {\n'
                   '    int config=0; T_DjiReturnCode code;\n' + step.group(0) +
                   '\n    ready_calls++; trigger_calls++;\n'
                   'gate_cancelled:\n    return code;\n}\n')
        source = cls.folder / 'notice.c'
        source.write_text(PRELUDE + '\n'.join(functions) + wrapper + EPILOGUE)
        cls.binary = cls.folder / 'notice'
        subprocess.run(shlex.split(os.environ.get('CC', 'cc')) +
                       ['-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
                        str(source), '-o', str(cls.binary)], check=True, capture_output=True, text=True)

    def run_case(self, case):
        subprocess.run([str(self.binary), case], check=True, cwd=self.folder,
                       capture_output=True, text=True, timeout=5)

    def test_original_init_failure_survives_successful_cleanup(self):
        self.run_case('failure')

    def test_busy_attempt_preserves_failure_and_next_task_resets_it(self):
        self.run_case('next')

    def test_cancel_and_normal_completion_do_not_report_init_failure(self):
        self.run_case('cancel')

    def test_stop_closes_control_latch_without_waiting_for_worker(self):
        self.run_case('stop')

    def test_visual_error_persists_until_next_valid_task(self):
        self.run_case('visual')

    def test_successful_init_cancelled_before_ready_is_cleaned_up(self):
        self.run_case('cancel_success')


if __name__ == '__main__':
    unittest.main()
