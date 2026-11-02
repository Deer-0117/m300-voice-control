#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <stdint.h>
#ifdef __APPLE__
#define st_mtim st_mtimespec
#endif
#define DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS 0ULL
#define DJI_ERROR_SYSTEM_MODULE_CODE_BUSY 1ULL
#define DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN 2ULL
#define DJI_WIDGET_BUTTON_STATE_PRESS_DOWN 1
#define BALLOON_VOICE_RECOGNIZE 0
#define USER_LOG_INFO(...) ((void)0)
typedef uint64_t T_DjiReturnCode;
void BalloonWidget_ShowMessage(const char *message);
void BalloonWidget_LogButton(uint32_t,int32_t,const char *,T_DjiReturnCode);
bool BalloonPhotoDetector_IsRunning(void);
bool BalloonPhotoDetector_IsStopping(void);
bool BalloonTiming_Requested(void);
T_DjiReturnCode BalloonPhotoDetector_StartCalibrationLiveview(void);
T_DjiReturnCode BalloonPhotoDetector_StopCalibrationLiveview(void);
void BalloonWidget_TimingStep(void);
T_DjiReturnCode BalloonWidget_StartVoiceTask(int,const char *);

/* New IDs prevent a cached old Prepare button from starting flight. */
#define BALLOON_WIDGET_ANY_INDEX 18
#define BALLOON_WIDGET_PINK_YELLOW_INDEX 19
#define BALLOON_WIDGET_VOICE_CONFIRM_INDEX 20
#define BALLOON_WIDGET_STOP_INDEX 21
#define BALLOON_WIDGET_RECORD_INDEX 22
#define BALLOON_WIDGET_NEXT_INDEX 23
#define BALLOON_WIDGET_NETWORK_INDEX 24
#define BALLOON_WIDGET_VOICE_CANCEL_INDEX 25
#define BALLOON_WIDGET_API_NETWORK_INDEX 26
#define BALLOON_WIDGET_API_ASR_INDEX 27
#define BALLOON_WIDGET_API_VISION_INDEX 28
#define BALLOON_WIDGET_API_INTENT_INDEX 29
#define BALLOON_WIDGET_COUNT 30
#define BALLOON_WIDGET_PATH_SIZE     256
#define BALLOON_NETWORK_DEFAULT_HOST "192.168.168.222"
#define BALLOON_NETWORK_DEFAULT_PORT 22U
#define BALLOON_NETWORK_DEFAULT_TIMEOUT_MS 2000
#define BALLOON_NETWORK_MIN_TIMEOUT_MS 200
#define BALLOON_NETWORK_MAX_TIMEOUT_MS 10000
#define BALLOON_LATEST_TARGET_FILE "/home/uav/offline_balloon_strike_test/latest_target.tsv"
#define BALLOON_LATEST_PHOTO_FILE "/tmp/balloon_latest_photo.txt"
#define BALLOON_DETECTION_REQUEST_FILE "/home/uav/offline_balloon_strike_test/latest_detection_request.tsv"
#define BALLOON_DETECTION_REQUEST_TMP_FILE "/home/uav/offline_balloon_strike_test/latest_detection_request.tsv.tmp"
#define BALLOON_NL_STOP_FILE "/tmp/balloon_nl_stop.request"
#define BALLOON_NL_PLAN_FILE "/home/uav/offline_balloon_strike_test/nl_task_plan.json"
#define BALLOON_NL_STATE_FILE "/home/uav/offline_balloon_strike_test/nl_task_state.json"
#define BALLOON_VOICE_PCM_DEFAULT "/home/uav/yanshou_uav/uavs_psdk/psdk_dji_3.9.2_cpp_json/samples/sample_c++/platform/linux/manifold2/build_cpp_json_minimal_c_linux/bin/test_audio.pcm"
#define BALLOON_VOICE_HELPER "/home/uav/offline_balloon_strike_test/uav_voice_command.py"
#define BALLOON_VOICE_PYTHON "/home/uav/offline_balloon_strike_test/yolo_env/bin/python"
#define BALLOON_VOICE_MESSAGE_FILE "/tmp/balloon_voice_message.txt"
#define BALLOON_PINK_YELLOW_HELPER "/home/uav/offline_balloon_strike_test/stage_pink_yellow_test.py"
#define BALLOON_PINK_YELLOW_MESSAGE_FILE "/tmp/balloon_pink_yellow_test.log"
#define BALLOON_PREDICTION_RECORD_REQUEST "/tmp/balloon_prediction_record.request"

static char s_armedFilePath[BALLOON_WIDGET_PATH_SIZE] = "/tmp/balloon_strike.armed";
static char s_triggerFilePath[BALLOON_WIDGET_PATH_SIZE] = "/tmp/balloon_strike.trigger";
static char s_stopFilePath[BALLOON_WIDGET_PATH_SIZE] = "/tmp/balloon_strike.stop";
static int32_t s_widgetValue[BALLOON_WIDGET_COUNT] = {0};
static bool s_isArmed = false;
static bool s_networkTestRunning = false;
static pthread_mutex_t s_networkTestMutex = PTHREAD_MUTEX_INITIALIZER;
static bool s_voiceCommandRunning = false;
static pthread_mutex_t s_voiceCommandMutex = PTHREAD_MUTEX_INITIALIZER;
/* All mission transitions are serialized; subprocess waits never hold this lock. */
static pthread_mutex_t s_actionMutex = PTHREAD_MUTEX_INITIALIZER;
static bool s_missionBusy = false;
/* Read and written only while s_actionMutex is held. */
static bool s_apiNetworkRunning = false;
/* Test state and child process ownership are protected by s_actionMutex. */
static bool s_apiTestRunning = false;
static bool s_apiVoiceResync = false;
static bool s_apiTestCancelRequested = false;
static bool s_apiTestLiveviewReserved = false;
static bool s_apiTestCleanupFailed = false;
static pid_t s_apiTestPid = 0;
static uint64_t s_apiTestLastRequestMs = 0;
static bool s_recordEnabled = true;
static bool s_stopNoticePending = false;
/* Protected by s_actionMutex. Reserve resources while the watcher is inside
 * an SDK call; never keep the UI/mission mutex locked across that call. */
static bool s_timingReserved = false;
#define BALLOON_MISSION_HELPER "/home/uav/offline_balloon_strike_test/balloon_mission.py"
#define BALLOON_RECORD_SETTING "/home/uav/offline_balloon_strike_test/widget_record_enabled.txt"
static unsigned int s_actionEpoch = 0;

static bool BalloonWidget_HelperBusy(void)
{
    bool busy;
    pthread_mutex_lock(&s_voiceCommandMutex);
    busy = s_voiceCommandRunning;
    pthread_mutex_unlock(&s_voiceCommandMutex);
    busy = busy || s_missionBusy || s_apiNetworkRunning || s_apiTestRunning ||
           s_apiTestLiveviewReserved || s_apiVoiceResync;
    return busy;
}


#include <assert.h>
#undef BALLOON_VOICE_PCM_DEFAULT
#ifndef TEST_PCM_PATH
#define TEST_PCM_PATH "/tmp/uav-api-voice-watch-isolation.pcm"
#endif
#define BALLOON_VOICE_PCM_DEFAULT TEST_PCM_PATH
static int g_startCalls, g_stopCalls, g_forkCalls, g_voiceCalls;
static T_DjiReturnCode g_startCode, g_stopCode;
static char g_message[512];
pid_t TestFork(void) { ++g_forkCalls; errno=EAGAIN; return -1; }
int TestPthreadCreate(pthread_t *a,const pthread_attr_t *b,void *(*c)(void *),void *d)
{ (void)a;(void)b;(void)c;(void)d; return EAGAIN; }
#define fork TestFork
#define pthread_create TestPthreadCreate
typedef struct {
    const char *kind;
    uint32_t button;
    unsigned int epoch;
    char requestId[96];
} T_BalloonApiTestTask;

static uint64_t BalloonWidget_ApiTestMonotonicMs(void)
{
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0;
    return (uint64_t) now.tv_sec * 1000ULL + (uint64_t) now.tv_nsec / 1000000ULL;
}

/* The request ID is generated locally and is never taken from UI text. */
static bool BalloonWidget_ReadApiTestMessage(const char *requestId, char message[241])
{
    char path[256];
    struct stat info;
    size_t length;
    snprintf(path, sizeof(path), "/run/uav-api-tests/message-%s.txt", requestId);
    FILE *fp = fopen(path, "rb");
    if (fp == NULL) return false;
    if (fstat(fileno(fp), &info) != 0 || !S_ISREG(info.st_mode) ||
        info.st_size <= 0 || info.st_size > 240) {
        fclose(fp);
        return false;
    }
    length = fread(message, 1, 240, fp);
    bool ok = !ferror(fp) && length == (size_t) info.st_size;
    fclose(fp);
    if (!ok || memchr(message, '\0', length) != NULL) return false;
    while (length > 0 && (message[length - 1] == '\n' || message[length - 1] == '\r')) --length;
    message[length] = '\0';
    return strncmp(message, "仅测试", strlen("仅测试")) == 0;
}

static void *BalloonWidget_ApiTestTask(void *arg)
{
    T_BalloonApiTestTask *task = arg;
    bool vision = strcmp(task->kind, "vision") == 0;
    bool streamAttempted = false, streamFailed = false, cleanupFailed = false;
    bool cancelled;
    pid_t child = -1;
    int status = 0, exitCode = -1;
    T_DjiReturnCode streamCode = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

    pthread_mutex_lock(&s_actionMutex);
    cancelled = s_apiTestCancelRequested || task->epoch != s_actionEpoch;
    if (vision && !cancelled) {
        /* Reserve before leaving the lock, including partially failed starts. */
        s_apiTestLiveviewReserved = true;
        streamAttempted = true;
    }
    pthread_mutex_unlock(&s_actionMutex);
    if (streamAttempted) {
        streamCode = BalloonPhotoDetector_StartCalibrationLiveview();
        streamFailed = streamCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    pthread_mutex_lock(&s_actionMutex);
    cancelled = s_apiTestCancelRequested || task->epoch != s_actionEpoch;
    if (!cancelled && !streamFailed) {
        sigset_t blockedSignals, previousMask;
        sigemptyset(&blockedSignals);
        sigaddset(&blockedSignals, SIGTERM);
        sigaddset(&blockedSignals, SIGINT);
        if (pthread_sigmask(SIG_BLOCK, &blockedSignals, &previousMask) == 0) {
            child = fork();
            if (child == 0) {
                /* A pending Stop must never invoke inherited PSDK signal handlers. */
                struct sigaction defaultAction = {0};
                defaultAction.sa_handler = SIG_DFL;
                sigemptyset(&defaultAction.sa_mask);
                if (sigaction(SIGTERM, &defaultAction, NULL) != 0 ||
                    sigaction(SIGINT, &defaultAction, NULL) != 0 ||
                    sigaction(SIGCHLD, &defaultAction, NULL) != 0) _exit(126);
                if (setpgid(0, 0) != 0) _exit(126);
                sigdelset(&previousMask, SIGTERM);
                sigdelset(&previousMask, SIGINT);
                (void) sigprocmask(SIG_SETMASK, &previousMask, NULL);
                /* Only async-signal-safe calls are used after a multithreaded fork. */
                execl("/usr/bin/timeout", "timeout", "--foreground", "--kill-after=2s", "90s",
                      "/usr/local/bin/uav-api-functional-test", "--kind", task->kind,
                      "--request-id", task->requestId, (char *) NULL);
                _exit(127);
            }
            (void) pthread_sigmask(SIG_SETMASK, &previousMask, NULL);
        }
        if (child > 0) {
            /* Child also sets its group before exec; registration and Stop are atomic. */
            (void) setpgid(child, child);
            s_apiTestPid = child;
        }
    }
    pthread_mutex_unlock(&s_actionMutex);

    if (child > 0) {
        uint64_t startedMs = BalloonWidget_ApiTestMonotonicMs(), cancelMs = 0;
        while (1) {
            bool done;
            uint64_t nowMs = BalloonWidget_ApiTestMonotonicMs();
            pthread_mutex_lock(&s_actionMutex);
            pid_t waited = waitpid(child, &status, WNOHANG);
            done = waited == child || (waited < 0 && errno != EINTR);
            if (done) {
                /* Clear ownership before unlocking, so Stop never signals a reused PID. */
                s_apiTestPid = 0;
                if (waited == child) exitCode = WIFEXITED(status) ? WEXITSTATUS(status) :
                    WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1;
            } else {
                cancelled = s_apiTestCancelRequested || task->epoch != s_actionEpoch;
                if (cancelled && cancelMs == 0) {
                    cancelMs = nowMs;
                    (void) kill(-child, SIGTERM);
                }
                if ((cancelled && cancelMs != 0 && nowMs >= cancelMs + 2000ULL) ||
                    (startedMs != 0 && nowMs >= startedMs + 95000ULL)) {
                    (void) kill(-child, SIGKILL);
                }
            }
            pthread_mutex_unlock(&s_actionMutex);
            if (done) break;
            usleep(100000);
        }
    }

    /* Camera cleanup also runs after failed starts, fork failures and cancellation. */
    if (streamAttempted) {
        streamCode = BalloonPhotoDetector_StopCalibrationLiveview();
        cleanupFailed = streamCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    pthread_mutex_lock(&s_actionMutex);
    cancelled = s_apiTestCancelRequested || task->epoch != s_actionEpoch;
    s_apiVoiceResync = true;
    s_apiTestCleanupFailed = cleanupFailed;
    if (cleanupFailed) {
        /* A failed SDK release must not let a later mission reuse this resource. */
        BalloonWidget_ShowMessage("仅测试：画面流停止失败；请重启PSDK服务后重试，任务启动保持禁用");
    } else {
        s_apiTestLiveviewReserved = false;
        s_apiTestRunning = false;
        if (cancelled) {
            BalloonWidget_ShowMessage("仅测试：已取消；测试结果不会执行飞行任务");
        } else if (streamFailed) {
            BalloonWidget_ShowMessage("仅测试：画面流启动失败，未执行API识别");
        } else if (exitCode == 0 || exitCode == 1 || exitCode == 2) {
            char message[241] = {0};
            if (BalloonWidget_ReadApiTestMessage(task->requestId, message)) {
                BalloonWidget_ShowMessage(message);
            } else {
                BalloonWidget_ShowMessage("仅测试：结果文件缺失或无效，请重新测试");
                exitCode = -1;
            }
        } else if (exitCode == 124 || exitCode == 137) {
            BalloonWidget_ShowMessage("仅测试：API测试超时，未执行飞行任务");
        } else {
            BalloonWidget_ShowMessage("仅测试：测试工具启动或执行失败，请检查安装与日志");
        }
    }
    USER_LOG_INFO("API functional test kind=%s request=%s exit=%d cancelled=%d cleanup_failed=%d stream_code=0x%llX",
                  task->kind, task->requestId, exitCode, cancelled, cleanupFailed,
                  (unsigned long long) streamCode);
    BalloonWidget_LogButton(task->button, DJI_WIDGET_BUTTON_STATE_PRESS_DOWN, "async_complete",
                            !cancelled && !cleanupFailed && !streamFailed && exitCode == 0 ?
                            DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN);
    pthread_mutex_unlock(&s_actionMutex);
    free(task);
    return NULL;
}

/* Caller holds s_actionMutex; no production task files are touched here. */
static T_DjiReturnCode BalloonWidget_StartApiTest(const char *kind, uint32_t button)
{
    pthread_t thread;
    pthread_attr_t attributes;
    int result = -1;
    if (BalloonPhotoDetector_IsRunning() || BalloonPhotoDetector_IsStopping() ||
        BalloonWidget_HelperBusy() || BalloonTiming_Requested() || s_timingReserved) {
        BalloonWidget_ShowMessage("仅测试：请等待任务、语音、联网或校准结束后重试");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    if (access(BALLOON_PREDICTION_RECORD_REQUEST, F_OK) == 0) {
        BalloonWidget_ShowMessage("仅测试：检测到录像请求，请先按停止全部测试并等待结束");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    uint64_t nowMs = BalloonWidget_ApiTestMonotonicMs();
    if (nowMs == 0) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    if (nowMs <= s_apiTestLastRequestMs) nowMs = s_apiTestLastRequestMs + 1;
    T_BalloonApiTestTask *task = calloc(1, sizeof(*task));
    if (task == NULL) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    task->kind = kind;
    task->button = button;
    task->epoch = s_actionEpoch;
    snprintf(task->requestId, sizeof(task->requestId), "api-%lu-%llu-%u", (unsigned long) getpid(),
             (unsigned long long) nowMs, (unsigned int) button);
    s_apiTestLastRequestMs = nowMs;
    s_apiTestRunning = true;
    s_apiVoiceResync = true;
    s_apiTestCancelRequested = false;
    s_apiTestCleanupFailed = false;
    if (pthread_attr_init(&attributes) == 0) {
        if (pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED) == 0) {
            result = pthread_create(&thread, &attributes, BalloonWidget_ApiTestTask, task);
        }
        (void) pthread_attr_destroy(&attributes);
    }
    if (result != 0) {
        free(task);
        s_apiTestRunning = false;
        s_apiVoiceResync = true;
        BalloonWidget_ShowMessage("仅测试：测试线程启动失败，请重试");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    BalloonWidget_ShowMessage(strcmp(kind, "asr") == 0 ?
        "仅测试：请按遥控器语音键说话，正在等待下一段录音；不会执行任务" :
        strcmp(kind, "vision") == 0 ? "仅测试：正在获取新画面并调用识别API，不录像、不飞行" :
        "仅测试：正在解析任务意图，不创建或执行任务");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


#undef fork
#undef pthread_create
static void *BalloonWidget_VoiceWatchTask(void *arg)
{
    struct stat current;
    time_t lastSeconds = 0;
    long lastNanoseconds = 0;
    off_t lastSize = 0;
    int stableCount = 0;

    (void) arg;
    if (stat(BALLOON_VOICE_PCM_DEFAULT, &current) == 0) {
        lastSeconds = current.st_mtim.tv_sec;
        lastNanoseconds = current.st_mtim.tv_nsec;
        lastSize = current.st_size;
    }
    while (1) {
        usleep(100000);
        BalloonWidget_TimingStep();
        pthread_mutex_lock(&s_actionMutex);
        if (s_apiTestRunning || s_apiVoiceResync) {
            /* Consume all test-period recordings, including a completion-boundary update.
             * The check, baseline update and normal dispatch share the action lock. */
            if (stat(BALLOON_VOICE_PCM_DEFAULT, &current) == 0) {
                lastSeconds = current.st_mtim.tv_sec;
                lastNanoseconds = current.st_mtim.tv_nsec;
                lastSize = current.st_size;
            } else {
                lastSeconds = 0;
                lastNanoseconds = 0;
                lastSize = 0;
            }
            stableCount = 0;
            s_apiVoiceResync = false;
            pthread_mutex_unlock(&s_actionMutex);
            continue;
        }
        if (BalloonPhotoDetector_IsRunning()) s_stopNoticePending = true;
        if (s_stopNoticePending && !BalloonPhotoDetector_IsRunning() && !BalloonWidget_HelperBusy()) {
            char stopReason[256] = {0};
            FILE *marker = fopen(BALLOON_NL_STOP_FILE, "r");
            if (marker) { (void) fread(stopReason, 1, sizeof(stopReason) - 1, marker); fclose(marker); }
            BalloonWidget_ShowMessage(strstr(stopReason, "capability_hit_unconfirmed")
                ? "追击已停止：当前目标结果未确认。可跳过并继续下一目标；最后一个目标请选择停止全部测试"
                : (strstr(stopReason, "capability_plan_complete")
                    ? "任务顺序已执行完毕，追击停止并保存录像；是否撞破以现场核实为准"
                    : "追击已停止，可直接选择任务；已接收录像帧正在完成保存"));
            s_stopNoticePending = false;
        }
        if (stat(BALLOON_VOICE_PCM_DEFAULT, &current) != 0 || current.st_size <= 0) {
            stableCount = 0;
            pthread_mutex_unlock(&s_actionMutex);
            continue;
        }
        if (current.st_mtim.tv_sec == lastSeconds &&
            current.st_mtim.tv_nsec == lastNanoseconds && current.st_size == lastSize) {
            if (stableCount > 0 && ++stableCount >= 4) {
                stableCount = 0;
                (void) BalloonWidget_StartVoiceTask(BALLOON_VOICE_RECOGNIZE,
                                                     BALLOON_VOICE_PCM_DEFAULT);
            }
            pthread_mutex_unlock(&s_actionMutex);
            continue;
        }
        lastSeconds = current.st_mtim.tv_sec;
        lastNanoseconds = current.st_mtim.tv_nsec;
        lastSize = current.st_size;
        stableCount = 1;
        pthread_mutex_unlock(&s_actionMutex);
    }
    return NULL;
}


#undef fork
#undef pthread_create
void BalloonWidget_ShowMessage(const char *message) { snprintf(g_message,sizeof(g_message),"%s",message); }
void BalloonWidget_LogButton(uint32_t a,int32_t b,const char *c,T_DjiReturnCode d)
{ (void)a;(void)b;(void)c;(void)d; }
bool BalloonPhotoDetector_IsRunning(void) { return false; }
bool BalloonPhotoDetector_IsStopping(void) { return false; }
bool BalloonTiming_Requested(void) { return false; }
T_DjiReturnCode BalloonPhotoDetector_StartCalibrationLiveview(void) { ++g_startCalls; return g_startCode; }
T_DjiReturnCode BalloonPhotoDetector_StopCalibrationLiveview(void) { ++g_stopCalls; return g_stopCode; }
void BalloonWidget_TimingStep(void) {}
T_DjiReturnCode BalloonWidget_StartVoiceTask(int a,const char *b)
{ (void)a;(void)b; ++g_voiceCalls; return 0; }
static T_BalloonApiTestTask *Task(const char *kind)
{
    T_BalloonApiTestTask *task=calloc(1,sizeof(*task));
    assert(task); task->kind=kind;task->button=28;task->epoch=s_actionEpoch;
    snprintf(task->requestId,sizeof(task->requestId),"isolated-mock"); return task;
}
static void Reset(void)
{
    s_apiTestRunning=true;s_apiVoiceResync=false;s_apiTestCancelRequested=false;
    s_apiTestLiveviewReserved=false;s_apiTestCleanupFailed=false;s_apiTestPid=0;
    g_startCalls=g_stopCalls=g_forkCalls=0;g_startCode=g_stopCode=0;
}
static void PCM(const char *value)
{
    FILE *fp=fopen(BALLOON_VOICE_PCM_DEFAULT,"wb");assert(fp);
    assert(fwrite(value,1,strlen(value),fp)==strlen(value));assert(fclose(fp)==0);
}
int main(void)
{
    Reset();g_stopCode=1;
    BalloonWidget_ApiTestTask(Task("vision"));
    assert(g_startCalls==1&&g_stopCalls==1&&g_forkCalls==1);
    assert(s_apiTestRunning&&s_apiTestLiveviewReserved&&s_apiTestCleanupFailed);
    assert(BalloonWidget_HelperBusy()&&strstr(g_message,"重启PSDK"));
    puts("PASS: camera release failure latches busy and reservation");
    Reset();s_apiTestCancelRequested=true;
    BalloonWidget_ApiTestTask(Task("vision"));
    assert(g_startCalls==0&&g_stopCalls==0&&g_forkCalls==0);
    assert(!s_apiTestRunning&&s_apiVoiceResync&&strstr(g_message,"已取消"));
    puts("PASS: early cancellation starts neither stream nor helper");
    Reset();g_startCode=1;
    BalloonWidget_ApiTestTask(Task("vision"));
    assert(g_startCalls==1&&g_stopCalls==1&&g_forkCalls==0);
    assert(!s_apiTestRunning&&!s_apiTestLiveviewReserved&&s_apiVoiceResync);
    puts("PASS: failed camera start still cleans the stream");
    Reset();s_apiTestRunning=false;
    pthread_mutex_lock(&s_actionMutex);
    T_DjiReturnCode code=BalloonWidget_StartApiTest("asr",27);
    pthread_mutex_unlock(&s_actionMutex);
    assert(code==DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN&&!s_apiTestRunning&&s_apiVoiceResync);
    puts("PASS: thread creation failure releases busy and requests voice resync");
    Reset();BalloonWidget_ApiTestTask(Task("asr"));
    assert(g_forkCalls==1&&!s_apiTestRunning&&s_apiVoiceResync&&s_apiTestPid==0);
    puts("PASS: fork failure releases busy and requests voice resync");
    Reset();PCM("baseline");
    pthread_t watcher;assert(pthread_create(&watcher,NULL,BalloonWidget_VoiceWatchTask,NULL)==0);
    usleep(220000);PCM("test-only recording");usleep(650000);
    pthread_mutex_lock(&s_actionMutex);assert(g_voiceCalls==0);
    s_apiTestRunning=false;s_apiVoiceResync=true;pthread_mutex_unlock(&s_actionMutex);
    usleep(650000);
    pthread_mutex_lock(&s_actionMutex);assert(g_voiceCalls==0&&!s_apiVoiceResync);pthread_mutex_unlock(&s_actionMutex);
    PCM("new production recording");usleep(650000);
    pthread_mutex_lock(&s_actionMutex);assert(g_voiceCalls==1);pthread_mutex_unlock(&s_actionMutex);
    assert(pthread_cancel(watcher)==0);assert(pthread_join(watcher,NULL)==0);
    remove(BALLOON_VOICE_PCM_DEFAULT);
    puts("PASS: test PCM never dispatches production voice; new later production PCM still works");
    return 0;
}
