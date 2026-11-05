"""Run mission-bound visual gating with SDK substitutes and temporary receipts.

No DJI library, network connection, aircraft command, or real worker is started.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
import unittest

from test_flight_panel_widget import function

ROOT = Path(__file__).resolve().parents[1]
WIDGET = ROOT / 'psdk-overlay/samples/sample_c/module_sample/balloon_widget'

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define BALLOON_VISUAL_STATUS_FILE "visual.json"
#define BALLOON_VISUAL_ACK_FILE "ack.json"
#include "balloon_visual_guard.h"
typedef uint64_t T_DjiReturnCode;
#define DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS 0
#define DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN 0xFF
#define BALLOON_CONTROL_LOOP_INTERVAL_MS 50
#define BALLOON_MISSION_TOKEN "token"
#define BALLOON_MISSION_COMMIT "commit"
static bool s_controlStopRequested, stop_marker, gate_ok=true, live_frame, found_target;
static unsigned s_visualFailureReason, s_visualFailureColor;
static uint64_t fake_now=10000, cancel_at, lost_gate_at;
static unsigned init_calls, gimbal_calls, flight_calls, clear_calls, hover_calls;
static bool core_has_target=true, fake_visual_hold=false, fake_has_active_target=true;
static float cached_forward=1.0f;
static T_DjiReturnCode DjiTargetChase_RequestSearch(uint64_t now,bool fast) {
    (void)now;assert(!fast);clear_calls++;core_has_target=false;return 0;
}
static T_DjiReturnCode DjiTargetChase_Hover(uint64_t now) {
    (void)now;hover_calls++;cached_forward=fmaxf(0,cached_forward-0.25f);return 0;
}
static void DjiTargetChase_GimbalTick(uint64_t now) { (void)now;gimbal_calls++; }
static void write_fixture(void);
static void sleep_ms(uint32_t ms) {
    fake_now += ms;
    if(cancel_at && fake_now >= cancel_at) s_controlStopRequested=true;
    if(lost_gate_at && fake_now >= lost_gate_at) gate_ok=false;
    write_fixture();
}
typedef struct { void (*TaskSleepMs)(uint32_t); } T_DjiOsalHandler;
static T_DjiOsalHandler osal = {sleep_ms};
static T_DjiOsalHandler *DjiPlatform_GetOsalHandler(void) { return &osal; }
static uint64_t MonoMs(void) { return fake_now; }
static uint64_t NowMs(void) { return fake_now; }
static bool BalloonClock_Id(char out[80]) { strcpy(out,"clock-A"); return true; }
static bool NaturalLanguageStopRequested(void) { return stop_marker; }
static bool BalloonMission_Matches(const char *path, const char *token) {
    (void)path; return gate_ok && !strcmp(token,"mission-A");
}
static bool BalloonMission_Healthy(const char *token) { return gate_ok && !strcmp(token,"mission-A"); }
static void AppendControlEvent(const char *a, T_DjiReturnCode b, const char *c) { (void)a;(void)b;(void)c; }
typedef struct {
    uint32_t seq; float width,height,x1,y1,x2,y2,confidence;
    uint64_t targetTsMs,detectionFinishedTsMs; char source[512];
} T_BalloonTargetRecord;
'''

MAIN = r'''
static cJSON *ack_object(void) {
    char text[1024];
    snprintf(text,sizeof(text),"{\"token\":\"mission-A\",\"clock_id\":\"clock-A\",\"plan_id\":\"plan-A\","
        "\"current_index\":0,\"requested_color\":\"purple\",\"started_ts_ms\":9000,\"ts_ms\":%llu,\"status\":\"loaded\"}",
        (unsigned long long)fake_now);
    return cJSON_Parse(text);
}
static cJSON *receipt_object(void) {
    char text[2048];
    snprintf(text,sizeof(text),"{\"version\":1,\"token\":\"mission-A\",\"clock_id\":\"clock-A\",\"plan_id\":\"plan-A\","
        "\"current_index\":0,\"requested_color\":\"purple\",\"started_ts_ms\":9000,\"capture_ts_ms\":%llu,\"ts_ms\":%llu,"
        "\"seq\":27,\"source\":\"yolo\",\"target_found\":%s,\"matches_requested\":true,\"selected_color\":\"purple\","
        "\"width\":640,\"height\":480,\"x1\":110,\"y1\":120,\"x2\":170,\"y2\":190,\"confidence\":0.91}",
        (unsigned long long)(fake_now-20),(unsigned long long)fake_now,found_target?"true":"false");
    return cJSON_Parse(text);
}
static void save_json(const char *path,cJSON *obj) {
    char *text=cJSON_PrintUnformatted(obj);
    FILE *fp=fopen(path,"w"); assert(fp); fputs(text,fp); fclose(fp); free(text);
}
static void write_fixture(void) {
    cJSON *ack=ack_object(); save_json(BALLOON_VISUAL_ACK_FILE,ack);cJSON_Delete(ack);
    if(live_frame) { cJSON *receipt=receipt_object();save_json(BALLOON_VISUAL_STATUS_FILE,receipt);cJSON_Delete(receipt); }
}
static void set_string(cJSON *obj,const char *key,const char *value) {
    cJSON_ReplaceItemInObjectCaseSensitive(obj,key,cJSON_CreateString(value));
}
static void set_number(cJSON *obj,const char *key,double value) {
    cJSON_ReplaceItemInObjectCaseSensitive(obj,key,cJSON_CreateNumber(value));
}
static bool parse_receipt(cJSON *obj) {
    cJSON *ack=ack_object(); T_BalloonVisualContext context;T_BalloonVisualReceipt receipt;
    assert(BalloonVisual_ParseAck(ack,"mission-A","clock-A",fake_now,&context));cJSON_Delete(ack);
    return BalloonVisual_ParseReceipt(obj,&context,fake_now,&receipt);
}
static void test_binding(void) {
    found_target=true;
    const char *fields[]={"token","plan_id","clock_id","requested_color","source","selected_color"};
    const char *wrong[]={"other-mission","other-plan","other-clock","yellow","mil","yellow"};
    for(unsigned i=0;i<sizeof(fields)/sizeof(fields[0]);++i) {
        cJSON *obj=receipt_object();assert(parse_receipt(obj));set_string(obj,fields[i],wrong[i]);
        assert(!parse_receipt(obj));cJSON_Delete(obj);
    }
    const char *number_fields[]={"current_index","started_ts_ms","capture_ts_ms","capture_ts_ms","ts_ms","seq","x2","confidence"};
    double wrong_numbers[]={1,8000,8000,11000,11000,2147483648.0,900,2};
    for(unsigned i=0;i<sizeof(number_fields)/sizeof(number_fields[0]);++i) {
        cJSON *obj=receipt_object();set_number(obj,number_fields[i],wrong_numbers[i]);
        assert(!parse_receipt(obj));cJSON_Delete(obj);
    }
    /* Both shipped local backends are permitted; external/API or tracker
     * receipts are never visual authorization. Purple shares the blue group. */
    cJSON *pixel=receipt_object();set_string(pixel,"source","pixel");assert(parse_receipt(pixel));
    set_string(pixel,"selected_color","blue");assert(parse_receipt(pixel));
    set_string(pixel,"source","api");assert(!parse_receipt(pixel));cJSON_Delete(pixel);
    cJSON *mismatch=receipt_object();
    cJSON_ReplaceItemInObjectCaseSensitive(mismatch,"matches_requested",cJSON_CreateBool(false));
    assert(!parse_receipt(mismatch));cJSON_Delete(mismatch);
    /* Same task but a stale frame cannot initialize flight. */
    fake_now=20000;cJSON *obj=receipt_object();set_number(obj,"capture_ts_ms",18000);
    assert(!parse_receipt(obj));cJSON_Delete(obj);
    assert(!BalloonVisual_Parse("{\"token\":\"a\",\"token\":\"b\"}"));
    assert(!BalloonVisual_Parse("{\"token\":\"a\"}trailing"));
    assert(!BalloonVisual_Parse("{\"token\":"));
}
static void test_success(void) {
    T_BalloonVisualWatch watch;live_frame=found_target=true;write_fixture();
    assert(guarded_init(&watch));assert(init_calls==1 && !gimbal_calls && !flight_calls);
    assert(!s_visualFailureReason);assert(!strcmp(watch.context.color,"purple"));
}
static void test_no_target(bool image) {
    T_BalloonVisualWatch watch;live_frame=image;write_fixture();
    assert(!guarded_init(&watch));assert(!init_calls && !gimbal_calls && !flight_calls);
    assert(fake_now==15000);assert(s_visualFailureReason==(image?1U:2U));
    char message[256];assert(BalloonPhotoDetector_GetVisualFailure(message,sizeof(message)));
    assert(strstr(message,image?"未识别到紫色气球":"有效相机图像"));
}
static void test_cancel(bool before,bool health) {
    T_BalloonVisualWatch watch;live_frame=true;write_fixture();
    if(before) s_controlStopRequested=true;
    else if(health) lost_gate_at=10150;
    else cancel_at=10150;
    assert(!guarded_init(&watch));assert(!init_calls && !gimbal_calls && !flight_calls);
    assert(!s_visualFailureReason);assert(fake_now<15000);
}
static void test_watchdog(void) {
    T_BalloonVisualWatch watch;T_BalloonTargetRecord target;bool changed;
    live_frame=found_target=true;write_fixture();assert(guarded_init(&watch));
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==1 && !changed);
    assert(target.seq==27 && target.targetTsMs==9980);
    found_target=false;fake_now=12979;write_fixture();
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==0);
    fake_now=12980;write_fixture();
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==-1);
    assert(s_visualFailureReason==3);assert(init_calls==1 && !flight_calls && !gimbal_calls);
}
static void test_action_change(void) {
    T_BalloonVisualWatch watch;T_BalloonTargetRecord target;bool changed;
    live_frame=found_target=true;write_fixture();assert(guarded_init(&watch));
    cJSON *ack=ack_object();set_number(ack,"current_index",1);set_string(ack,"requested_color","yellow");
    save_json(BALLOON_VISUAL_ACK_FILE,ack);cJSON_Delete(ack);
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==0 && changed);
    assert(watch.context.index==1 && !strcmp(watch.context.color,"yellow"));
    /* An old purple receipt can never become the next action's target. */
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==0 && !changed);
    cJSON *receipt=receipt_object();set_number(receipt,"current_index",1);
    set_string(receipt,"requested_color","yellow");set_string(receipt,"selected_color","yellow");
    save_json(BALLOON_VISUAL_STATUS_FILE,receipt);cJSON_Delete(receipt);
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==1 && !changed);
}
static void test_explicit_missing_clears_and_keeps_decelerating(void) {
    T_BalloonVisualWatch watch;T_BalloonTargetRecord target;bool changed;
    live_frame=found_target=true;write_fixture();assert(guarded_init(&watch));
    found_target=false;
    for(int i=0;i<4;++i) {
        fake_now+=50;write_fixture();
        int result=BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed);
        assert(result==0);apply_actual_visual_transition(&watch,result,changed);
        assert(!core_has_target && !fake_has_active_target && fake_visual_hold);
        assert(clear_calls==1 && hover_calls==(unsigned)(i+1) && !gimbal_calls);
    }
    assert(cached_forward==0);
}
static void test_phase_clears_before_new_target(void) {
    T_BalloonVisualWatch watch;T_BalloonTargetRecord target;bool changed;
    live_frame=found_target=true;write_fixture();assert(guarded_init(&watch));
    cJSON *ack=ack_object();set_number(ack,"current_index",1);set_string(ack,"requested_color","yellow");
    save_json(BALLOON_VISUAL_ACK_FILE,ack);cJSON_Delete(ack);
    cJSON *receipt=receipt_object();set_number(receipt,"current_index",1);
    set_string(receipt,"requested_color","yellow");set_string(receipt,"selected_color","yellow");
    save_json(BALLOON_VISUAL_STATUS_FILE,receipt);cJSON_Delete(receipt);
    int result=BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed);
    assert(result==1 && changed);apply_actual_visual_transition(&watch,result,changed);
    assert(clear_calls==1 && hover_calls==1 && !gimbal_calls && !core_has_target);
}
static void test_ack_outage_does_not_extend_loss_deadline(void) {
    T_BalloonVisualWatch watch;T_BalloonTargetRecord target;bool changed;
    live_frame=found_target=true;write_fixture();assert(guarded_init(&watch));
    assert(watch.lastTargetMonoMs==9980);found_target=false;fake_now=12500;write_fixture();
    remove(BALLOON_VISUAL_ACK_FILE);
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==0 && changed);
    assert(watch.hasContext && !watch.contextValid && watch.lastTargetMonoMs==9980);
    fake_now=12900;write_fixture();
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==0 && !changed);
    assert(watch.lastTargetMonoMs==9980 && watch.contextValid);
    fake_now=12980;write_fixture();
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==-1);
}
static void test_normal_stop_priority(void) {
    T_BalloonVisualWatch watch;T_BalloonTargetRecord target;bool changed;
    live_frame=found_target=true;write_fixture();assert(guarded_init(&watch));
    found_target=false;fake_now=14000;write_fixture();stop_marker=true;
    assert(BalloonControl_ObserveVisual("mission-A",&watch,&target,&changed)==-1);
    assert(!s_visualFailureReason);
    BalloonVisual_RecordFailure(1,"purple");char message[256];
    assert(BalloonPhotoDetector_GetVisualFailure(message,sizeof(message)));
    BalloonVisual_RecordFailure(0,NULL);assert(!BalloonPhotoDetector_GetVisualFailure(message,sizeof(message)));
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(!strcmp(argv[1],"binding")) test_binding();
    else if(!strcmp(argv[1],"success")) test_success();
    else if(!strcmp(argv[1],"absent")) test_no_target(true);
    else if(!strcmp(argv[1],"noimage")) test_no_target(false);
    else if(!strcmp(argv[1],"cancel_before")) test_cancel(true,false);
    else if(!strcmp(argv[1],"cancel_wait")) test_cancel(false,false);
    else if(!strcmp(argv[1],"health_lost")) test_cancel(false,true);
    else if(!strcmp(argv[1],"lost")) test_watchdog();
    else if(!strcmp(argv[1],"action")) test_action_change();
    else if(!strcmp(argv[1],"normal_stop")) test_normal_stop_priority();
    else if(!strcmp(argv[1],"missing_hold")) test_explicit_missing_clears_and_keeps_decelerating();
    else if(!strcmp(argv[1],"phase_clear")) test_phase_clears_before_new_target();
    else if(!strcmp(argv[1],"ack_outage")) test_ack_outage_does_not_extend_loss_deadline();
    else assert(false);
    return 0;
}
'''


class VisualPreflight(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sdk = Path(os.environ['TEST_PSDK_ROOT']) if os.environ.get('TEST_PSDK_ROOT') else ROOT/'psdk-overlay'
        module = sdk/'samples/sample_c/module_sample'
        if not (module/'utils/cJSON.c').is_file():
            raise unittest.SkipTest('public SDK cJSON source unavailable; set TEST_PSDK_ROOT')
        cls.temp = tempfile.TemporaryDirectory(prefix='uav-visual-stubs-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.folder = Path(cls.temp.name)
        detector = (WIDGET/'balloon_photo_detector.c').read_text()
        watch = re.search(r'typedef struct \{\n    T_BalloonVisualContext context;.*?\} T_BalloonVisualWatch;', detector, re.S)
        names = ['BalloonVisual_ColorText', 'BalloonVisual_RecordFailure',
                 'BalloonPhotoDetector_GetVisualFailure', 'BalloonControl_VisualCancelled',
                 'BalloonControl_WaitForVisual', 'BalloonControl_ObserveVisual', 'BalloonControl_HoldForVisual']
        # Extract the actual call order before SDK Init; an accidental removal
        # of the gate must fail the test rather than only exercise an unused helper.
        guard = re.search(r'    if \(!BalloonControl_WaitForVisual\(missionToken, &visualWatch\)\).*?'
                          r'(?=    code = DjiTargetChase_Init\(&config\);)', detector, re.S)
        if guard is None or watch is None:
            raise AssertionError('visual preflight must guard the actual SDK initialization')
        wrapper = ('\nstatic bool guarded_init(T_BalloonVisualWatch *out) {\n'
                   'const char *missionToken="mission-A";T_BalloonVisualWatch visualWatch;\n' + guard.group(0) +
                   '\ninit_calls++;*out=visualWatch;return true;\ngate_cancelled:return false;\n}\n')
        reset_block = re.search(r'        bool resetVisualTarget = phaseChanged.*?'
                                r'(?=        if \(visualResult > 0\) \{)', detector, re.S)
        if reset_block is None:
            raise AssertionError('visual target reset must precede every new target submission')
        reset_wrapper = (r"""
static void apply_actual_visual_transition(T_BalloonVisualWatch *watch,int result,bool changed) {
    const char *missionToken="mission-A";uint64_t nowMs=fake_now;
    T_BalloonVisualWatch visualWatch=*watch;
    bool phaseChanged=changed,visualHold=fake_visual_hold,hasActiveTarget=fake_has_active_target;
    bool currentTargetTimedOut=false,burstHoverLocked=false,hoverKeepaliveActive=false;
    T_BalloonTargetRecord activeTarget;int targetFilter=0;T_DjiReturnCode code;
    uint64_t lastAcceptedTargetTsMs=10,lastObservedSeq=10,lastObservedTargetTsMs=10;
    uint64_t lastAcceptedYoloControlMs=10,activeCommandExpireMonoMs=10;
    uint64_t hoverStartedMonoMs=10,firstTargetLoadedMonoMs=10;
    int visualResult=result;
    do {
""" + reset_block.group(0) + r"""
    } while(0);
    fake_visual_hold=visualHold;fake_has_active_target=hasActiveTarget;
    (void)currentTargetTimedOut;(void)burstHoverLocked;(void)hoverKeepaliveActive;
    (void)lastAcceptedTargetTsMs;(void)lastObservedSeq;(void)lastObservedTargetTsMs;
    (void)lastAcceptedYoloControlMs;(void)activeCommandExpireMonoMs;
    (void)hoverStartedMonoMs;(void)firstTargetLoadedMonoMs;
}
""")
        source = cls.folder/'guard.c'
        source.write_text(PRELUDE+watch.group(0)+'\n'+'\n'.join(function(detector,n) for n in names)+wrapper+reset_wrapper+MAIN)
        cls.binary = cls.folder/'guard'
        compiled = subprocess.run(shlex.split(os.environ.get('CC','cc')) + [
            '-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-function','-DCJSON_HIDE_SYMBOLS',
            '-I'+str(WIDGET),'-I'+str(module),str(source),str(module/'utils/cJSON.c'),
            '-lm','-o',str(cls.binary)], capture_output=True,text=True)
        if compiled.returncode:
            raise AssertionError(compiled.stderr)

    def run_case(self, case):
        with tempfile.TemporaryDirectory(dir=self.folder) as work:
            subprocess.run([str(self.binary),case],cwd=work,check=True,capture_output=True,text=True,timeout=5)

    def test_binding_color_time_clock_and_corrupt_json(self): self.run_case('binding')
    def test_fresh_current_target_allows_initialization(self): self.run_case('success')
    def test_absent_color_causes_error_without_sdk_calls(self): self.run_case('absent')
    def test_missing_image_has_distinct_error_without_sdk_calls(self): self.run_case('noimage')
    def test_cancel_before_preflight_does_not_initialize(self): self.run_case('cancel_before')
    def test_cancel_during_preflight_does_not_initialize(self): self.run_case('cancel_wait')
    def test_worker_health_loss_exits_without_initialization(self): self.run_case('health_lost')
    def test_target_loss_stops_at_three_seconds(self): self.run_case('lost')
    def test_action_change_rejects_prior_target(self): self.run_case('action')
    def test_normal_stop_precedes_visual_error(self): self.run_case('normal_stop')
    def test_explicit_missing_clears_old_target_and_hover_repeats_to_zero(self): self.run_case('missing_hold')
    def test_phase_change_clears_core_before_gimbal_or_target_updates(self): self.run_case('phase_clear')
    def test_transient_ack_outage_does_not_renew_loss_deadline(self): self.run_case('ack_outage')


if __name__ == '__main__': unittest.main()
