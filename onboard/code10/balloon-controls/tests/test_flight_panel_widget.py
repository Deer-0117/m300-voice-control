"""Exercise actual panel C functions with SDK/process substitutes.

Only temporary files are touched; no threads, subprocess helpers, SDK calls or
network operations are started by the compiled harness.
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
WIDGET = ROOT / 'psdk-overlay/samples/sample_c/module_sample/balloon_widget'


def function(source, name):
    pattern = re.compile(r'^(?:static )?[^;{}\n]*?\b' + re.escape(name) +
                         r'\([^;{}]*?\)\n\{', re.MULTILINE)
    found = list(pattern.finditer(source))
    if len(found) != 1:
        raise ValueError('Expected one real definition of ' + name)
    start = found[0].start()
    end = source.index('\n}\n', found[0].end()) + 3
    return source[start:end]


PRELUDE = r'''
#include <assert.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
typedef int T_DjiReturnCode;
typedef int E_DjiWidgetType;
enum { DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS=0, DJI_ERROR_SYSTEM_MODULE_CODE_BUSY=1,
       DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN=2, DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT=3,
       DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE=4 };
enum { DJI_WIDGET_TYPE_BUTTON=1, DJI_WIDGET_TYPE_SWITCH=2, DJI_WIDGET_TYPE_LIST=3,
       DJI_WIDGET_BUTTON_STATE_PRESS_DOWN=1, DJI_WIDGET_BUTTON_STATE_RELEASE_UP=0 };
#define USER_LOG_ERROR(...) ((void)0)
#define USER_LOG_WARN(...) ((void)0)
#define USER_LOG_INFO(...) ((void)0)
#define BALLOON_TIMING_REQUEST "timing.request"
#define BALLOON_PREDICTION_RECORD_REQUEST "record.request"
#define PANEL_MESSAGE "panel.message"
#define BALLOON_OPTIONS_REQUEST "options.request"
#define BALLOON_OPTIONS_ACK "options.ack"
#define BALLOON_OPTIONS_MESSAGE "options.message"
#define BALLOON_OPTIONS_HELPER "do-not-execute-detector-helper"
#define PANEL_HELPER "do-not-execute-panel-helper"
#define BALLOON_VOICE_PYTHON "do-not-execute-python"
#define API_VOICE_SESSION "voice.session"
#define API_VOICE_READY "voice.ready"
#define API_VOICE_CANDIDATE "voice.candidate"
#define BALLOON_VOICE_MESSAGE_FILE "voice.message"
typedef struct { unsigned int epoch; int kind; } T_BalloonPanelTask;
enum { PANEL_AUDIT, PANEL_SAMPLE, PANEL_KALMAN, PANEL_RELEASE };
static pthread_mutex_t s_actionMutex=PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t s_voiceCommandMutex=PTHREAD_MUTEX_INITIALIZER;
static bool s_voiceCommandRunning, s_missionBusy, s_apiNetworkRunning, s_apiTestRunning;
static bool s_apiTestLiveviewReserved, s_apiVoiceResync, s_panelBusy, s_panelSampling;
static bool s_timingReserved, s_timingOwnLiveview, s_timingOwnSubscription;
static bool s_newVoicePending, s_newVoicePromptPending, s_apiTestCancelRequested;
static bool s_stopNoticePending, s_isArmed, s_apiTestCleanupFailed, s_recordEnabled, s_displayOnly;
static unsigned int s_actionEpoch;
static uint64_t s_sampleConfirmUntil;
static pid_t s_voicePid, s_missionPid, s_panelPid, s_apiTestPid, s_optionsPid;
static char s_voiceSession[96];
static int s_predictionMode;
static int32_t s_taskSelection, s_detectorSelection, startup_detector;
static char s_armedFilePath[256], s_triggerFilePath[256], s_stopFilePath[256];
static bool running, stopping, timing_requested, release_pending;
static uint64_t fake_now;
static int child_result, child_epoch_change, child_calls, thread_calls, fail_thread;
static int timing_ticks, timing_cleanup, stop_calls, network_calls, new_api_calls, mission_calls;
static int kill_calls, cleared_gate, reset_calls, voice_clears, exposure_calls, startup_fail_stage;
static int maintenance_calls, maintenance_result, env_failures;
static bool detector_ack;
static bool controller_maintenance;
static char last_mode[32], last_message[1024];
static void *queued_arg;
static void *(*queued_function)(void *);
static bool BalloonPhotoDetector_IsRunning(void) { return running; }
static bool BalloonPhotoDetector_IsStopping(void) { return stopping; }
static bool BalloonTiming_Requested(void) { return timing_requested; }
static bool DjiTargetChase_IsReleasePending(void) { return release_pending; }
static T_DjiReturnCode DjiTargetChase_SetPanelMaintenance(bool enabled) {
    maintenance_calls++;
    if(maintenance_result) return maintenance_result;
    controller_maintenance=enabled; return 0;
}
static bool DjiTargetChase_RequestedNoFlight(void) { return controller_maintenance; }
static int fake_setenv(const char *name,const char *value,int replace) {
    if(env_failures) { env_failures--; return -1; }
    return setenv(name,value,replace);
}
#define setenv fake_setenv
static T_DjiReturnCode DjiTargetChase_DeInit(void) { release_pending=false; return 0; }
static uint64_t BalloonClock_NowMs(void) { return fake_now; }
static void BalloonWidget_ShowMessage(const char *text) { snprintf(last_message,sizeof(last_message),"%s",text); }
static void BalloonWidget_LogButton(uint32_t i,int32_t v,const char *p,int c) { (void)i;(void)v;(void)p;(void)c; }
static void BalloonTiming_Tick(bool busy) {
    assert(!busy); timing_ticks++;
    if (timing_cleanup) { timing_requested=false; s_timingOwnLiveview=false; s_timingOwnSubscription=false; }
}
static int fake_pthread_create(pthread_t *t,const pthread_attr_t *a,void *(*fn)(void *),void *arg) {
    (void)a; thread_calls++; if(fail_thread) return 1;
    assert(!queued_arg); *t=(pthread_t)1; queued_function=fn; queued_arg=arg; return 0;
}
static int fake_pthread_detach(pthread_t t) { (void)t; return 0; }
static int fake_kill(pid_t pid,int sig) { assert(pid < -1); assert(sig==SIGTERM); kill_calls++; return 0; }
static int fake_remove(const char *path) {
    if(!strcmp(path,BALLOON_TIMING_REQUEST)) timing_requested=false;
    if(!strcmp(path,API_VOICE_SESSION)) voice_clears++;
    return unlink(path);
}
#define pthread_create fake_pthread_create
#define pthread_detach fake_pthread_detach
#define kill fake_kill
#define remove fake_remove
static int BalloonPanel_RunChild(char *const argv[],unsigned int epoch,pid_t *owner) {
    *owner=0; if(epoch!=s_actionEpoch)return -1; child_calls++;
    if (owner == &s_optionsPid) {
        assert(!strcmp(argv[0],BALLOON_VOICE_PYTHON));
        assert(!strcmp(argv[2],"--select-detector"));
        assert(!strcmp(argv[4],"--token") && !strcmp(argv[6],"--owner-pid"));
        assert(strtol(argv[7],NULL,10)==getpid());
        if (detector_ack) {
            FILE *file=fopen(BALLOON_OPTIONS_ACK,"w"); assert(file);
            fprintf(file,"%s\n",argv[5]); fclose(file);
        }
    }
    s_actionEpoch+=(unsigned int)child_epoch_change; return child_result;
}
static bool BalloonMission_Matches(const char *path,const char *token) {
    FILE *file=fopen(path,"r"); if (!file) return false;
    char data[96]; bool read=fgets(data,sizeof(data),file)!=NULL; fclose(file);
    if (!read) return false;
    data[strcspn(data,"\n")]=0; return !strcmp(data,token);
}
static int32_t BalloonPanel_ReadDetectorSelection(void) { return startup_detector; }
static T_DjiReturnCode BalloonWidget_BeginMission(const char *mode) {
    mission_calls++; snprintf(last_mode,sizeof(last_mode),"%s",mode); return 0;
}
static T_DjiReturnCode BalloonWidget_StartApiNetwork(void) { network_calls++; return 0; }
static T_DjiReturnCode BalloonPanel_NewApiTask(void) { new_api_calls++; return 0; }
static T_DjiReturnCode BalloonPhotoDetector_StopWithReason(const char *reason) {
    assert(!strcmp(reason,"widget_stop_all")); stop_calls++; return 0;
}
static void BalloonWidget_CloseMissionGate(void) { cleared_gate++; }
static void ClearTriggerFiles(void) { remove(BALLOON_PREDICTION_RECORD_REQUEST); }
static void BalloonWidget_ResetButtonValues(void) { reset_calls++; }
static void SafeCopyPath(char *dst,size_t n,const char *src) { if(src)snprintf(dst,n,"%s",src); }
static T_DjiReturnCode DjiWidget_Init(void) { return startup_fail_stage==1?2:0; }
static T_DjiReturnCode DjiTest_WidgetSpeakerStartService(void) { return startup_fail_stage==2?2:0; }
static T_DjiReturnCode RegisterWidgetUiConfig(const char *p) { (void)p;return startup_fail_stage==3?2:0; }
static T_DjiReturnCode BalloonWidget_StartVoiceWatch(void) { return startup_fail_stage==5?2:0; }
static T_DjiReturnCode BalloonPhotoDetector_StartAutoExposure(void) { exposure_calls++;return 0; }
static T_DjiReturnCode DjiWidget_RegHandlerList(const void *p,size_t n) { (void)p;assert(n==26);return startup_fail_stage==4?2:0; }
static T_DjiReturnCode BalloonWidget_SetValue(E_DjiWidgetType,uint32_t,int32_t,void *);
static T_DjiReturnCode BalloonWidget_GetValue(E_DjiWidgetType,uint32_t,int32_t *,void *);
typedef struct { uint32_t index; E_DjiWidgetType type;
    T_DjiReturnCode (*set)(E_DjiWidgetType,uint32_t,int32_t,void *);
    T_DjiReturnCode (*get)(E_DjiWidgetType,uint32_t,int32_t *,void *); void *data;
} T_DjiWidgetHandlerListItem;
'''

VOICE_ENTRY_PRELUDE = r'''
#include <sys/stat.h>
#ifdef __APPLE__
#define st_mtim st_mtimespec
#endif
#define BALLOON_VOICE_PCM_DEFAULT "voice.pcm"
static unsigned int s_voiceSequence;
static time_t s_voiceLastSeconds;
static long s_voiceLastNanoseconds;
static off_t s_voiceLastSize;
static int s_voiceStableCount, voice_start_calls;
static bool clock_fail, pcm_exists = true;
static struct stat pcm_stat;
static bool BalloonClock_Id(char *id) {
    if(clock_fail) return false;
    strcpy(id,"mono-v1:11111111-1111-4111-8111-111111111111"); return true;
}
static bool BalloonWidget_WritePlain(const char *path,const char *text) {
    FILE *f=fopen(path,"w"); if(!f)return false;
    bool ok=fprintf(f,"%s\n",text)>0; return fclose(f)==0 && ok;
}
static int fake_stat(const char *path,struct stat *data) {
    assert(!strcmp(path,BALLOON_VOICE_PCM_DEFAULT));
    if(!pcm_exists)return -1; *data=pcm_stat; return 0;
}
#define stat(path,data) fake_stat((path),(data))
#define BALLOON_VOICE_RECOGNIZE 0
static T_DjiReturnCode BalloonWidget_StartVoiceTask(int action,const char *path) {
    assert(action==BALLOON_VOICE_RECOGNIZE && !strcmp(path,BALLOON_VOICE_PCM_DEFAULT));
    assert(!s_newVoicePending); voice_start_calls++; return 0;
}
'''

EPILOGUE = r'''
static void reset(void) {
    s_voiceCommandRunning=s_missionBusy=s_apiNetworkRunning=s_apiTestRunning=false;
    s_apiTestLiveviewReserved=s_apiVoiceResync=s_panelBusy=s_panelSampling=false;
    s_timingReserved=s_timingOwnLiveview=s_timingOwnSubscription=false;
    s_newVoicePending=s_newVoicePromptPending=s_apiTestCancelRequested=false;
    s_stopNoticePending=s_isArmed=s_apiTestCleanupFailed=s_recordEnabled=s_displayOnly=false;
    running=stopping=timing_requested=release_pending=false;
    s_actionEpoch=1; s_sampleConfirmUntil=0;
    s_voicePid=s_missionPid=s_panelPid=s_apiTestPid=s_optionsPid=0;
    s_voiceSession[0]=0; s_predictionMode=1; fake_now=1000;
    s_taskSelection=s_detectorSelection=startup_detector=0; detector_ack=false;
    child_result=child_epoch_change=child_calls=thread_calls=fail_thread=0;
    timing_ticks=timing_cleanup=stop_calls=network_calls=new_api_calls=mission_calls=0;
    kill_calls=cleared_gate=reset_calls=voice_clears=exposure_calls=startup_fail_stage=0;
    maintenance_calls=maintenance_result=env_failures=0; controller_maintenance=false;
    last_message[0]=last_mode[0]=0;
    clock_fail=false; pcm_exists=true; memset(&pcm_stat,0,sizeof(pcm_stat));
    pcm_stat.st_mtim.tv_sec=10; pcm_stat.st_size=128;
    s_voiceLastSeconds=0; s_voiceLastNanoseconds=0; s_voiceLastSize=0;
    s_voiceStableCount=voice_start_calls=0;
    unlink(API_VOICE_SESSION); unlink(API_VOICE_READY); unlink(API_VOICE_CANDIDATE);
    assert(!queued_arg); queued_function=NULL;
    setenv("BALLOON_PREDICTION_MODE","1",1);
    unsetenv("BALLOON_PANEL_DISPLAY_ONLY");
    unlink(PANEL_MESSAGE); unlink(BALLOON_TIMING_REQUEST); unlink(BALLOON_PREDICTION_RECORD_REQUEST);
    unlink(BALLOON_OPTIONS_REQUEST); unlink(BALLOON_OPTIONS_ACK); unlink(BALLOON_OPTIONS_MESSAGE);
}
static void finish(void) {
    assert(queued_arg && queued_function);
    void *arg=queued_arg; void *(*fn)(void *)=queued_function;
    queued_arg=NULL; queued_function=NULL; fn(arg);
}
static void message_file(const char *text) {
    FILE *f=fopen(PANEL_MESSAGE,"w"); assert(f); fputs(text,f); fclose(f);
}
static void button(uint32_t index) {
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,index,1,NULL)==0);
}
static void test_registration_and_dispatch(void) {
    const uint32_t expected[]={50,51,52,53,54,55,56,57,58,60,61,65,66,67,68,69,
                               81,82,83,84,91,92,93,94,98,99};
    assert(sizeof(s_widgetHandlerList)/sizeof(s_widgetHandlerList[0])==sizeof(expected)/sizeof(expected[0]));
    for(size_t i=0;i<sizeof(expected)/sizeof(expected[0]);i++) {
        const T_DjiWidgetHandlerListItem *h=&s_widgetHandlerList[i];
        assert(h->index==expected[i]);
        assert(BalloonWidget_ValidControl(h->type,BalloonWidget_ActionIndex(h->index)));
    }
    for(uint32_t i=0;i<50;i++) assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,i,1,NULL)==3);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,59,1,NULL)==3);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,54,1,NULL)==3);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,50,1,NULL)==3);
    assert(!mission_calls && !thread_calls && !stop_calls);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,50,0,NULL)==0);
    assert(!mission_calls);
    button(50); assert(!mission_calls && s_voiceSession[0] && voice_clears==1 && cleared_gate==1);
    button(51); assert(network_calls==1);
    button(55); assert(mission_calls==1 && !strcmp(last_mode,"voice"));
    button(56); assert(new_api_calls==1);
    button(57); assert(voice_clears==2);
    button(58); assert(stop_calls==1);
    reset(); button(60); assert(!mission_calls && s_voiceSession[0] && voice_clears==1 && cleared_gate==1);
    reset(); button(61); assert(network_calls==1 && !mission_calls);
    reset(); button(65); assert(mission_calls==1 && !strcmp(last_mode,"voice"));
    reset(); button(66); assert(new_api_calls==1 && !mission_calls);
    reset(); button(67); assert(voice_clears==1 && !stop_calls);
    reset(); button(68); assert(stop_calls==1 && cleared_gate==1);
    for(uint32_t i=59;i<=70;i++) {
        if(i==60 || i==61 || (i>=65 && i<=68)) continue;
        assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,i,1,NULL)==3);
        int32_t value=123;
        assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_BUTTON,i,&value,NULL)==3 && value==123);
    }
    int32_t value=123;
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_BUTTON,60,&value,NULL)==0 && value==0);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,60,1,NULL)==3);
}
static void test_voice_input_new_session_never_starts_a_mission(void) {
    strcpy(s_voiceSession,"old-session");
    assert(BalloonWidget_WritePlain(API_VOICE_SESSION,"old-session"));
    assert(BalloonWidget_WritePlain(API_VOICE_READY,"old-session"));
    assert(BalloonWidget_WritePlain(API_VOICE_CANDIDATE,"stale candidate"));
    s_newVoicePending=true; s_newVoicePromptPending=true; s_sampleConfirmUntil=9999;
    unsigned old=s_actionEpoch;
    button(50);
    assert(s_actionEpoch==old+1 && strcmp(s_voiceSession,"old-session"));
    assert(access(API_VOICE_READY,F_OK)!=0 && access(API_VOICE_CANDIDATE,F_OK)!=0);
    assert(access(API_VOICE_SESSION,F_OK)==0 && !s_newVoicePending && !s_newVoicePromptPending);
    assert(!s_apiVoiceResync && !s_sampleConfirmUntil && !s_isArmed);
    assert(!mission_calls && !thread_calls && !child_calls && !stop_calls && !kill_calls);
    assert(strstr(last_message,"语音键") && strstr(last_message,"执行API语音任务") && strstr(last_message,"未启动"));
    for(unsigned i=0;i<8;i++)BalloonWidget_PollVoicePcm(false);
    assert(!voice_start_calls); /* The PCM that predates the click is consumed. */
    pcm_stat.st_mtim.tv_nsec=1; pcm_stat.st_size++;
    for(unsigned i=0;i<4;i++)BalloonWidget_PollVoicePcm(false);
    assert(voice_start_calls==1 && !mission_calls && !s_isArmed);
    for(unsigned i=0;i<8;i++)BalloonWidget_PollVoicePcm(false);
    assert(voice_start_calls==1); /* A stable old recording is not retriggered. */
    button(55); assert(mission_calls==1 && !strcmp(last_mode,"voice"));
}
static void test_voice_input_busy_and_maintenance_preserve_existing_task(void) {
    bool *flags[]={&running,&stopping,&s_missionBusy,&s_voiceCommandRunning,&s_apiNetworkRunning,
        &s_apiTestRunning,&s_apiTestLiveviewReserved,&s_panelBusy,&timing_requested,&s_timingReserved,
        &s_timingOwnLiveview,&s_timingOwnSubscription,&release_pending,&s_apiTestCleanupFailed,
        &s_isArmed,&s_stopNoticePending};
    for(unsigned i=0;i<sizeof(flags)/sizeof(flags[0]);i++) {
        reset(); *flags[i]=true; strcpy(s_voiceSession,"keep-current");
        unsigned old=s_actionEpoch;
        assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,60,1,NULL)==1);
        assert(s_actionEpoch==old && !strcmp(s_voiceSession,"keep-current"));
        assert(!cleared_gate && !voice_clears && !stop_calls && !mission_calls && !thread_calls);
    }
    reset(); s_displayOnly=true;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,50,1,NULL)==4);
    assert(!cleared_gate && !voice_clears && !mission_calls && !thread_calls);
    reset(); clock_fail=true; strcpy(s_voiceSession,"stale");
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,50,1,NULL)==2);
    assert(!s_voiceSession[0] && !mission_calls && !thread_calls && s_apiVoiceResync);
}
static void test_voice_input_consumes_busy_audio_and_rearms_fast_new_audio(void) {
    button(60);
    pcm_stat.st_mtim.tv_nsec=100;
    BalloonWidget_PollVoicePcm(true);
    for(unsigned i=0;i<8;i++)BalloonWidget_PollVoicePcm(false);
    assert(!voice_start_calls);
    button(50); /* Snapshot is synchronous, before the next watcher tick. */
    pcm_stat.st_mtim.tv_nsec=200;
    for(unsigned i=0;i<4;i++)BalloonWidget_PollVoicePcm(false);
    assert(voice_start_calls==1 && !mission_calls && !thread_calls && !s_isArmed);
    pcm_exists=false; BalloonWidget_PollVoicePcm(false);
    assert(!s_voiceStableCount);
}
static void test_sample_confirmation_and_busy(void) {
    button(52); assert(!thread_calls && s_sampleConfirmUntil==16000);
    fake_now=16001; button(52); assert(!thread_calls && s_sampleConfirmUntil==31001);
    fake_now=17000; button(52); assert(thread_calls==1 && s_panelBusy && s_panelSampling);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,53,1,NULL)==1);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,54,0,NULL)==1);
    message_file("采样完成：录像与姿态已保存；尚未生成或验证时间标定\n");
    finish(); assert(!s_panelBusy && !s_panelSampling && child_calls==1);
    assert(strstr(last_message,"采样完成"));
}
static void test_sample_release_and_spawn_failure(void) {
    button(52); release_pending=true;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,52,1,NULL)==1);
    assert(!thread_calls && !s_panelBusy);
    release_pending=false; button(52); fail_thread=1;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,52,1,NULL)==2);
    assert(!s_panelBusy && !s_panelSampling && !queued_arg);
}
static void test_timing_worker_exclusive_exception(void) {
    s_panelBusy=s_panelSampling=timing_requested=true;
    BalloonWidget_TimingStep(); assert(timing_ticks==1 && s_timingReserved);
    s_panelSampling=false; BalloonWidget_TimingStep(); assert(timing_ticks==1);
    s_panelSampling=true; s_voiceCommandRunning=true;
    BalloonWidget_TimingStep(); assert(timing_ticks==1);
    s_voiceCommandRunning=false; s_missionBusy=true;
    BalloonWidget_TimingStep(); assert(timing_ticks==1);
    s_missionBusy=false; s_timingOwnLiveview=true; timing_cleanup=1;
    BalloonWidget_TimingStep(); assert(timing_ticks==2 && !s_timingReserved);
}
static void test_kalman_requires_check_and_idle(void) {
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,54,1,NULL)==0);
    assert(s_predictionMode==1 && s_panelBusy && !s_panelSampling);
    child_result=1; message_file("Kalman未启用：标定未生效或已过期\n"); finish();
    assert(s_predictionMode==1 && !strcmp(getenv("BALLOON_PREDICTION_MODE"),"1"));
    assert(strstr(last_message,"标定"));
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,54,1,NULL)==0);
    child_result=0; finish(); assert(s_predictionMode==3);
    int32_t value=-1;
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_SWITCH,54,&value,NULL)==0 && value==1);
    running=true;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,54,0,NULL)==1 && s_predictionMode==3);
    running=false;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,54,0,NULL)==0 && s_predictionMode==1);
}
static void test_stop_invalidates_pending_success_and_sample(void) {
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,54,1,NULL)==0);
    unsigned int old=s_actionEpoch; s_panelPid=123; s_voicePid=124; s_missionPid=125;
    button(58); assert(s_actionEpoch==old+1 && kill_calls==3 && s_panelBusy);
    char stopped[1024]; snprintf(stopped,sizeof(stopped),"%s",last_message);
    child_epoch_change=0; child_result=0;
    finish(); assert(s_predictionMode==1 && !s_panelBusy);
    assert(!strcmp(last_message,stopped));
    s_apiVoiceResync=false; s_panelPid=0; s_voicePid=0; s_missionPid=0;
    button(52); button(52);
    FILE *f=fopen(BALLOON_TIMING_REQUEST,"w");assert(f);fputs("own",f);fclose(f);
    f=fopen(BALLOON_PREDICTION_RECORD_REQUEST,"w");assert(f);fputs("own",f);fclose(f);
    child_epoch_change=1;
    finish(); assert(access(BALLOON_TIMING_REQUEST,F_OK)!=0);
    assert(access(BALLOON_PREDICTION_RECORD_REQUEST,F_OK)!=0);
    assert(!s_panelBusy && !s_panelSampling);
}
static void test_audit_result_does_not_change_mode(void) {
    s_predictionMode=3; setenv("BALLOON_PREDICTION_MODE","3",1);
    button(53); child_result=1;
    message_file("部署：一致｜进程：运行版本一致｜标定：未配置时间标定\n"); finish();
    assert(strstr(last_message,"部署：一致") && strstr(last_message,"未配置时间标定"));
    assert(s_predictionMode==3 && !s_panelBusy);
}
static void test_combined_options_and_detector_transaction(void) {
    int32_t value=-1;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,91,1,NULL)==0);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_LIST,81,&value,NULL)==0 && value==1);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,83,1,NULL)==0);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_LIST,93,&value,NULL)==0 && value==1);
    assert(s_predictionMode==0 && !strcmp(getenv("BALLOON_PREDICTION_MODE"),"0"));
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,93,2,NULL)==0);
    child_result=1; finish(); assert(s_predictionMode==0);
    child_result=0;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,83,2,NULL)==0);
    finish(); assert(s_predictionMode==3);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_SWITCH,54,&value,NULL)==0 && value==1);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,98,1,NULL)==0 && s_recordEnabled);
    button(94); assert(!strcmp(last_mode,"pink"));
    button(99); assert(!strcmp(last_mode,"next"));
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,82,1,NULL)==4);
    s_displayOnly=controller_maintenance=true;
    FILE *unrelated=fopen("unrelated","w"); assert(unrelated);
    fputs("unchanged",unrelated); fclose(unrelated);
    assert(symlink("unrelated",BALLOON_OPTIONS_REQUEST)==0);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,92,1,NULL)==0 && s_panelBusy);
    assert(access(BALLOON_OPTIONS_REQUEST,F_OK)==0);
    unrelated=fopen("unrelated","r"); assert(unrelated);
    char text[16]; assert(fgets(text,sizeof(text),unrelated)); fclose(unrelated);
    assert(!strcmp(text,"unchanged")); unlink("unrelated");
    finish(); assert(s_detectorSelection==0 && access(BALLOON_OPTIONS_REQUEST,F_OK)!=0);
    detector_ack=true;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,82,1,NULL)==0);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,81,0,NULL)==1);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,0,NULL)==1);
    button(58); finish();
    assert(!s_panelBusy && s_detectorSelection==0 && access(BALLOON_OPTIONS_REQUEST,F_OK)!=0);
    s_apiVoiceResync=s_stopNoticePending=false;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_LIST,92,1,NULL)==0);
    finish(); assert(s_detectorSelection==1);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,0,NULL)==0);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_LIST,82,&value,NULL)==0 && value==1);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_LIST,93,&value,NULL)==0 && value==0);
}

static void test_startup_resets_stale_mode_and_voice(void) {
    s_predictionMode=3; setenv("BALLOON_PREDICTION_MODE","3",1);
    strcpy(s_voiceSession,"old-session");
    setenv("BALLOON_PANEL_DISPLAY_ONLY","0",1);
    setenv("BALLOON_NO_FLIGHT","0",1);
    startup_detector=1;
    assert(BalloonWidget_StartService("test-ui",NULL,NULL,NULL)==0);
    assert(s_predictionMode==1 && !strcmp(getenv("BALLOON_PREDICTION_MODE"),"1"));
    assert(!s_voiceSession[0] && s_recordEnabled && voice_clears==1 && cleared_gate==1);
    assert(s_displayOnly && controller_maintenance && maintenance_calls==1 && !exposure_calls && s_detectorSelection==1);
}
static void test_display_only_resources_are_closed(void) {
    setenv("BALLOON_PANEL_DISPLAY_ONLY","1",1);
    assert(BalloonWidget_StartService("test-ui",NULL,NULL,NULL)==0);
    assert(s_displayOnly && controller_maintenance && !exposure_calls && strstr(last_message,"维护已开启"));
    s_panelBusy=s_panelSampling=timing_requested=true;
    BalloonWidget_TimingStep(); assert(!timing_ticks && !s_timingReserved);
    s_panelBusy=s_panelSampling=timing_requested=false;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,52,1,NULL)==4);
    assert(!thread_calls && !child_calls && !s_sampleConfirmUntil);
    button(51); assert(network_calls==1);
    button(53); message_file("部署一致；仅显示模式，采样与飞行禁用"); finish();
    assert(child_calls==1 && !exposure_calls && !timing_ticks);
}
static void test_startup_failure_is_not_reported_as_ready(void) {
    setenv("BALLOON_PANEL_DISPLAY_ONLY","1",1);
    for(int stage=1;stage<=5;stage++) {
        startup_fail_stage=stage; last_message[0]=0;
        assert(BalloonWidget_StartService("test-ui",NULL,NULL,NULL)==2);
        assert(!exposure_calls && !strstr(last_message,"面板已加载"));
    }
}

static void assert_mode(bool maintenance) {
    int32_t value=-1;
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_SWITCH,69,&value,NULL)==0);
    assert(value==(maintenance?1:0) && s_displayOnly==maintenance && controller_maintenance==maintenance);
    assert(!strcmp(BalloonWidget_ModeText(),maintenance?"NO_FLIGHT":"FLIGHT"));
}
static void test_maintenance_transitions(void) {
    s_displayOnly=controller_maintenance=true;
    setenv("BALLOON_NO_FLIGHT","unchanged",1);
    setenv("BALLOON_GIMBAL_TRACKING","unchanged",1);
    setenv("BALLOON_PANEL_DISPLAY_ONLY","unchanged",1);
    unsigned old=s_actionEpoch;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,1,NULL)==0);
    assert_mode(true); assert(s_actionEpoch==old && !maintenance_calls && !cleared_gate && !voice_clears);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,2,NULL)==3);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,-1,NULL)==3);
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_BUTTON,69,0,NULL)==3);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_BUTTON,69,(int32_t *)&old,NULL)==3);
    assert(BalloonWidget_GetValue(DJI_WIDGET_TYPE_SWITCH,69,NULL,NULL)==3);
    s_predictionMode=3; setenv("BALLOON_PREDICTION_MODE","3",1);
    s_sampleConfirmUntil=5000; s_newVoicePending=true; s_newVoicePromptPending=true;
    strcpy(s_voiceSession,"stale");
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,0,NULL)==0);
    assert_mode(false);
    assert(s_actionEpoch==old+1 && s_predictionMode==1 && !s_sampleConfirmUntil);
    assert(!s_newVoicePending && !s_newVoicePromptPending && !s_voiceSession[0] && s_apiVoiceResync);
    assert(maintenance_calls==1 && cleared_gate==1 && voice_clears==1 && reset_calls==1);
    assert(!stop_calls && !thread_calls && !child_calls && !mission_calls && !exposure_calls);
    assert(strstr(last_message,"切换未启动任务"));
    assert(!strcmp(getenv("BALLOON_NO_FLIGHT"),"unchanged"));
    assert(!strcmp(getenv("BALLOON_GIMBAL_TRACKING"),"unchanged"));
    assert(!strcmp(getenv("BALLOON_PANEL_DISPLAY_ONLY"),"unchanged"));
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,0,NULL)==0);
    assert(s_actionEpoch==old+1 && maintenance_calls==1 && voice_clears==1);
    s_apiVoiceResync=false;
    assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,1,NULL)==0);
    assert_mode(true); assert(s_actionEpoch==old+2 && maintenance_calls==2);
    assert(!stop_calls && !thread_calls && !child_calls && !mission_calls && !exposure_calls);
}
static void test_maintenance_busy_rejected(void) {
    bool *flags[]={&running,&stopping,&s_missionBusy,&s_voiceCommandRunning,&s_apiNetworkRunning,
        &s_apiTestRunning,&s_apiTestLiveviewReserved,&s_panelBusy,&timing_requested,&s_timingReserved,
        &s_timingOwnLiveview,&s_timingOwnSubscription,&release_pending,&s_apiTestCleanupFailed,
        &s_isArmed,&s_stopNoticePending};
    for(unsigned mode=0;mode<2;mode++) {
        for(unsigned i=0;i<sizeof(flags)/sizeof(flags[0]);i++) {
            reset(); s_displayOnly=controller_maintenance=mode!=0; *flags[i]=true;
            unsigned old=s_actionEpoch;
            assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,mode?0:1,NULL)==1);
            assert_mode(mode!=0); assert(s_actionEpoch==old && !maintenance_calls && !cleared_gate);
            assert(!stop_calls && !thread_calls && !child_calls && !mission_calls);
            assert(strstr(last_message,"先按停止测试"));
        }
        pid_t *pids[]={&s_voicePid,&s_missionPid,&s_panelPid,&s_apiTestPid};
        for(unsigned i=0;i<4;i++) {
            reset(); s_displayOnly=controller_maintenance=mode!=0; *pids[i]=123;
            assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,mode?0:1,NULL)==1);
            assert_mode(mode!=0); assert(!maintenance_calls && !kill_calls);
        }
    }
}
static void test_maintenance_failures(void) {
    for(unsigned mode=0;mode<2;mode++) {
        reset(); s_displayOnly=controller_maintenance=mode!=0; env_failures=1;
        unsigned old=s_actionEpoch;
        assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,mode?0:1,NULL)==2);
        assert_mode(mode!=0); assert(!maintenance_calls && !cleared_gate && s_actionEpoch==old);
        maintenance_result=1;
        assert(BalloonWidget_SetValue(DJI_WIDGET_TYPE_SWITCH,69,mode?0:1,NULL)==1);
        assert_mode(mode!=0); assert(maintenance_calls==1 && !cleared_gate && s_actionEpoch==old);
        assert(!stop_calls && !thread_calls && !child_calls && !mission_calls);
    }
    reset(); env_failures=1;
    assert(BalloonWidget_StartService("test-ui",NULL,NULL,NULL)==2);
    assert(s_displayOnly && controller_maintenance && !exposure_calls && !last_message[0]);
    reset(); maintenance_result=1;
    assert(BalloonWidget_StartService("test-ui",NULL,NULL,NULL)==1);
    assert(s_displayOnly && !exposure_calls && !last_message[0]);
}

int main(int argc,char **argv) {
    assert(argc==2); reset();
    if(!strcmp(argv[1],"dispatch"))test_registration_and_dispatch();
    else if(!strcmp(argv[1],"voice_input"))test_voice_input_new_session_never_starts_a_mission();
    else if(!strcmp(argv[1],"voice_busy"))test_voice_input_busy_and_maintenance_preserve_existing_task();
    else if(!strcmp(argv[1],"voice_pcm"))test_voice_input_consumes_busy_audio_and_rearms_fast_new_audio();
    else if(!strcmp(argv[1],"sample"))test_sample_confirmation_and_busy();
    else if(!strcmp(argv[1],"spawn_failure"))test_sample_release_and_spawn_failure();
    else if(!strcmp(argv[1],"timing"))test_timing_worker_exclusive_exception();
    else if(!strcmp(argv[1],"kalman"))test_kalman_requires_check_and_idle();
    else if(!strcmp(argv[1],"cancel"))test_stop_invalidates_pending_success_and_sample();
    else if(!strcmp(argv[1],"audit"))test_audit_result_does_not_change_mode();
    else if(!strcmp(argv[1],"startup"))test_startup_resets_stale_mode_and_voice();
    else if(!strcmp(argv[1],"options"))test_combined_options_and_detector_transaction();
    else if(!strcmp(argv[1],"display"))test_display_only_resources_are_closed();
    else if(!strcmp(argv[1],"startup_failure"))test_startup_failure_is_not_reported_as_ready();
    else if(!strcmp(argv[1],"maintenance"))test_maintenance_transitions();
    else if(!strcmp(argv[1],"maintenance_busy"))test_maintenance_busy_rejected();
    else if(!strcmp(argv[1],"maintenance_failure"))test_maintenance_failures();
    else assert(false);
    assert(!queued_arg); return 0;
}
'''


class ActualPanelC(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='uav-panel-c-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.folder = Path(cls.temp.name)
        widget = (WIDGET / 'psdk_balloon_widget.c').read_text()
        panel = (WIDGET / 'balloon_flight_panel.h').read_text()
        macros = '\n'.join(re.findall(r'^#define BALLOON_WIDGET_\w+\s+\d+\s*$', widget, re.MULTILINE))
        registration = re.search(r'static const T_DjiWidgetHandlerListItem s_widgetHandlerList\[\] = \{.*?\n\};',
                                 widget, re.DOTALL).group(0)
        options = re.search(r'typedef struct \{[^}]+\} T_BalloonDetectorSelection;', panel).group(0)
        functions = [function(widget, 'BalloonWidget_ActionIndex'), function(widget, 'BalloonWidget_HelperBusy'), function(panel, 'BalloonPanel_ClearVoice'),
                     function(widget, 'BalloonWidget_ResetVoiceBaseline'), function(panel, 'BalloonPanel_NewVoiceSession'),
                     function(panel, 'BalloonPanel_Idle'), function(panel, 'BalloonPanel_BeginVoiceInput'),
                     function(widget, 'BalloonWidget_PollVoicePcm'), function(panel, 'BalloonPanel_Task'),
                     function(panel, 'BalloonPanel_RetryRelease'),
                     function(panel, 'BalloonPanel_StartTool'), function(panel, 'BalloonPanel_SetPrediction'),
                     function(panel, 'BalloonPanel_DetectorName'), function(panel, 'BalloonPanel_WriteOptionsLease'),
                     function(panel, 'BalloonPanel_RunDetectorHelper'), function(panel, 'BalloonPanel_ShowDetectorMessage'),
                     function(panel, 'BalloonPanel_DetectorTask'), function(panel, 'BalloonPanel_SelectDetector'),
                     function(widget, 'BalloonWidget_TimingStep'),
                     function(widget, 'BalloonWidget_Stop'), function(widget, 'BalloonWidget_SetMaintenance'),
                     function(widget, 'BalloonWidget_ModeText'), function(widget, 'BalloonWidget_ValidControl'), function(widget, 'BalloonWidget_SetValue'),
                     function(widget, 'BalloonWidget_GetValue'), function(widget, 'BalloonWidget_StartService')]
        source = cls.folder / 'panel_test.c'
        source.write_text(PRELUDE + VOICE_ENTRY_PRELUDE + '\n' + macros + '\n' + registration + '\n' + options + '\n' + '\n'.join(functions) + EPILOGUE)
        cls.binary = cls.folder / 'panel-test'
        command = shlex.split(os.environ.get('CC', 'cc')) + ['-std=gnu11', '-pthread', '-Wno-unused-function',
                   str(source), '-o', str(cls.binary)]
        built = subprocess.run(command, capture_output=True, text=True)
        if built.returncode:
            raise AssertionError(built.stderr)

    def test_real_widget_and_panel_state_transitions(self):
        for case in ('dispatch', 'voice_input', 'voice_busy', 'voice_pcm', 'sample', 'spawn_failure', 'timing', 'kalman', 'cancel', 'audit', 'startup', 'options', 'display', 'startup_failure', 'maintenance', 'maintenance_busy', 'maintenance_failure'):
            with self.subTest(case=case):
                result = subprocess.run([str(self.binary), case], cwd=self.folder, timeout=5,
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)


    def test_actual_mission_gate_rejects_pixel_pink_and_missing_voice_candidate(self):
        # Reuse the existing mission harness: every post-gate side effect
        # asserts if reached. Only the real BeginMission guard is exercised.
        import test_display_mode as display
        prelude, count = re.subn(
            r'static T_DjiReturnCode BalloonWidget_BeginMission\(const char \*mode\) \{.*?\n\}',
            '', PRELUDE, flags=re.DOTALL)
        self.assertEqual(count, 1)
        extra = display.MISSION_EXTRA.replace(
            'static bool BalloonPanel_VoiceReady(void) { return true; }',
            'static bool gate_ready; static bool BalloonPanel_VoiceReady(void) { return gate_ready; }')
        widget = (WIDGET / 'psdk_balloon_widget.c').read_text()
        source = self.folder / 'voice-confirm-gate.c'
        source.write_text(prelude + extra + function(widget, 'BalloonWidget_HelperBusy') +
                          function((WIDGET / 'balloon_flight_panel.h').read_text(), 'BalloonPanel_Idle') +
                          function(widget, 'BalloonWidget_BeginMission') + r"""
int main(void) {
    s_displayOnly=false;
    s_detectorSelection=1;
    assert(BalloonWidget_BeginMission("pink")==DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE);
    assert(BalloonWidget_BeginMission("voice")==DJI_ERROR_SYSTEM_MODULE_CODE_BUSY);
    assert(BalloonWidget_BeginMission("api_new")==DJI_ERROR_SYSTEM_MODULE_CODE_BUSY);
    gate_ready=true; s_newVoicePending=true;
    assert(BalloonWidget_BeginMission("voice")==DJI_ERROR_SYSTEM_MODULE_CODE_BUSY);
    assert(!thread_calls && !child_calls && !cleared_gate && !s_isArmed && !s_missionBusy);
    assert(s_actionEpoch==0); return 0;
}
""")
        binary = self.folder / 'voice-confirm-gate'
        built = subprocess.run(shlex.split(os.environ.get('CC', 'cc')) +
                               ['-std=gnu11', '-pthread', str(source), '-o', str(binary)],
                               capture_output=True, text=True)
        self.assertEqual(built.returncode, 0, built.stderr)
        result = subprocess.run([str(binary)], cwd=self.folder, timeout=5,
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
