/*
 * DJI Pilot Widget bridge for offline balloon strike tests.
 *
 * Integration point:
 *   Call BalloonWidget_StartService(...) after PSDK platform/application
 *   initialization. The function registers mission controls and an optional
 *   UI config directory.
 *
 * This module does not call flight-control APIs. The flight-test buttons write
 * local state files. The network buttons perform a bounded asynchronous TCP
 * probe or start and verify the configured API proxy connection.
 */

#include "psdk_balloon_widget.h"

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

#include "dji_error.h"
#include "dji_logger.h"
#include "dji_widget.h"
#include "balloon_widget/balloon_photo_detector.h"
#include "balloon_clock.h"
#include "balloon_timing_capture.h"
#include "balloon_mission_gate.h"
#include "balloon_evidence.h"
#include "widget/test_widget_speaker.h"

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

static void BalloonWidget_LogButton(uint32_t index, int32_t value, const char *phase, T_DjiReturnCode code)
{
    char clockId[80] = {0}, token[96];
    BalloonEvidence_Token(token);
    FILE *fp = fopen("/home/uav/offline_balloon_strike_test/widget_events.jsonl", "a");
    if (fp == NULL) return;
    (void) BalloonClock_Id(clockId);
    fprintf(fp, "{\"event\":\"button_result\",\"ts_ms\":%llu,\"clock_id\":\"%s\","
                "\"mission_token\":\"%s\",\"phase\":\"%s\",\"value\":%d,\"button\":%u,\"code\":\"0x%llX\",\"armed\":%s,\"running_or_stopping\":%s,"
                "\"helper_busy\":%s,\"epoch\":%u}\n",
            (unsigned long long) BalloonClock_NowMs(), clockId, token, phase, (int)value, (unsigned int) index,
            (unsigned long long) code, s_isArmed ? "true" : "false",
            BalloonPhotoDetector_IsRunning() ? "true" : "false",
            BalloonWidget_HelperBusy() ? "true" : "false", s_actionEpoch);
    fclose(fp);
}

static T_DjiReturnCode BalloonWidget_SetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData);
static T_DjiReturnCode BalloonWidget_GetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData);
static void SafeCopyPath(char *dst, size_t dstSize, const char *src);

static void BalloonWidget_ResetButtonValues(void)
{
    uint32_t index;

    for (index = 0; index < BALLOON_WIDGET_COUNT; ++index) {
        s_widgetValue[index] = DJI_WIDGET_BUTTON_STATE_RELEASE_UP;
    }
}

static const T_DjiWidgetHandlerListItem s_widgetHandlerList[] = {
    {BALLOON_WIDGET_ANY_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_PINK_YELLOW_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_VOICE_CONFIRM_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_STOP_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_RECORD_INDEX, DJI_WIDGET_TYPE_SWITCH, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_NEXT_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_NETWORK_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_VOICE_CANCEL_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_API_NETWORK_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_API_ASR_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_API_VISION_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_API_INTENT_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
};

typedef enum {
    BALLOON_VOICE_RECOGNIZE = 0,
    BALLOON_VOICE_CONFIRM,
    BALLOON_VOICE_CANCEL,
} E_BalloonVoiceAction;

typedef struct {
    E_BalloonVoiceAction action;
    unsigned int epoch;
    char pcmPath[BALLOON_WIDGET_PATH_SIZE];
} T_BalloonVoiceTask;

typedef enum {
    BALLOON_NETWORK_RESULT_SUCCESS = 0,
    BALLOON_NETWORK_RESULT_FAILED,
    BALLOON_NETWORK_RESULT_EXCEPTION,
} E_BalloonNetworkResult;

static void BalloonWidget_ShowMessage(const char *message)
{
    char visible[DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN + 1];
    size_t length = strlen(message);
    USER_LOG_INFO("Balloon UI: %s", message);
    char escaped[2048], token[96], clockId[80] = {0};
    BalloonEvidence_Escape(message, escaped, sizeof(escaped));
    BalloonEvidence_Token(token);
    (void) BalloonClock_Id(clockId);
    FILE *fp = fopen("/home/uav/offline_balloon_strike_test/widget_events.jsonl", "a");
    if (fp) {
        fprintf(fp, "{\"event\":\"operator_message\",\"ts_ms\":%llu,\"clock_id\":\"%s\",\"mission_token\":\"%s\",\"message\":\"%s\"}\n",
                (unsigned long long)BalloonClock_NowMs(), clockId, token, escaped);
        fclose(fp);
    }
    if (length > DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN) {
        length = DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN;
        while (length > 0 && ((unsigned char) message[length] & 0xC0) == 0x80) --length;
    }
    memcpy(visible, message, length);
    visible[length] = '\0';
    (void) DjiWidgetFloatingWindow_ShowMessage(visible);
}

static const char *BalloonWidget_ModeText(void)
{
    const char *value = getenv("BALLOON_NO_FLIGHT");

    if (value == NULL || value[0] == '\0' ||
        strcmp(value, "0") == 0 ||
        strcmp(value, "false") == 0 ||
        strcmp(value, "FALSE") == 0 ||
        strcmp(value, "off") == 0 ||
        strcmp(value, "OFF") == 0) {
        return "FLIGHT";
    }

    return "NO_FLIGHT";
}

static bool BalloonWidget_UsesLiveview(void)
{
    const char *value = getenv("BALLOON_INPUT_MODE");
    return value != NULL &&
           (strcmp(value, "liveview") == 0 || strcmp(value, "LIVEVIEW") == 0 ||
            strcmp(value, "stream") == 0 || strcmp(value, "STREAM") == 0);
}

static void BalloonWidget_ShowStep(const char *step, const char *detail)
{
    char message[256];

    snprintf(message, sizeof(message),
             "Balloon step: %s | mode=%s | %s",
             step == NULL ? "unknown" : step,
             BalloonWidget_ModeText(),
             detail == NULL ? "" : detail);
    BalloonWidget_ShowMessage(message);
}

static void BalloonWidget_ShowNetworkResult(E_BalloonNetworkResult result)
{
    switch (result) {
        case BALLOON_NETWORK_RESULT_SUCCESS:
            BalloonWidget_ShowMessage("联网：连接成功");
            break;
        case BALLOON_NETWORK_RESULT_FAILED:
            BalloonWidget_ShowMessage("联网：连接失败");
            break;
        default:
            BalloonWidget_ShowMessage("联网：连接异常");
            break;
    }
}

static uint16_t BalloonWidget_NetworkPort(void)
{
    const char *value = getenv("BALLOON_NETWORK_TEST_PORT");
    char *end = NULL;
    unsigned long port;

    if (value == NULL || value[0] == '\0') {
        return BALLOON_NETWORK_DEFAULT_PORT;
    }
    errno = 0;
    port = strtoul(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || port == 0 || port > 65535UL) {
        return 0;
    }
    return (uint16_t) port;
}

static int BalloonWidget_NetworkTimeoutMs(void)
{
    const char *value = getenv("BALLOON_NETWORK_TEST_TIMEOUT_MS");
    char *end = NULL;
    long timeoutMs;

    if (value == NULL || value[0] == '\0') {
        return BALLOON_NETWORK_DEFAULT_TIMEOUT_MS;
    }
    errno = 0;
    timeoutMs = strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' ||
        timeoutMs < BALLOON_NETWORK_MIN_TIMEOUT_MS ||
        timeoutMs > BALLOON_NETWORK_MAX_TIMEOUT_MS) {
        return -1;
    }
    return (int) timeoutMs;
}

static bool BalloonWidget_IsExpectedConnectFailure(int errorCode)
{
    return errorCode == ECONNREFUSED ||
           errorCode == ETIMEDOUT ||
           errorCode == ENETUNREACH ||
           errorCode == EHOSTUNREACH ||
           errorCode == ENETDOWN ||
           errorCode == EHOSTDOWN;
}

static E_BalloonNetworkResult BalloonWidget_ProbeNetwork(const char *host,
                                                         uint16_t port,
                                                         int timeoutMs,
                                                         int *detailError)
{
    struct sockaddr_in address;
    struct pollfd pollFd;
    int socketFd = -1;
    int flags;
    int pollResult;
    int socketError = 0;
    socklen_t socketErrorSize = sizeof(socketError);
    E_BalloonNetworkResult result = BALLOON_NETWORK_RESULT_EXCEPTION;

    *detailError = 0;
    if (host == NULL || host[0] == '\0' || port == 0 || timeoutMs < 0) {
        return BALLOON_NETWORK_RESULT_EXCEPTION;
    }

    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &address.sin_addr) != 1) {
        return BALLOON_NETWORK_RESULT_EXCEPTION;
    }

    socketFd = socket(AF_INET, SOCK_STREAM, 0);
    if (socketFd < 0) {
        *detailError = errno;
        return BALLOON_NETWORK_RESULT_EXCEPTION;
    }
    flags = fcntl(socketFd, F_GETFL, 0);
    if (flags < 0 || fcntl(socketFd, F_SETFL, flags | O_NONBLOCK) < 0) {
        *detailError = errno;
        close(socketFd);
        return BALLOON_NETWORK_RESULT_EXCEPTION;
    }

    if (connect(socketFd, (struct sockaddr *) &address, sizeof(address)) == 0) {
        result = BALLOON_NETWORK_RESULT_SUCCESS;
        goto out;
    }
    if (errno != EINPROGRESS) {
        *detailError = errno;
        result = BalloonWidget_IsExpectedConnectFailure(errno) ?
                 BALLOON_NETWORK_RESULT_FAILED : BALLOON_NETWORK_RESULT_EXCEPTION;
        goto out;
    }

    pollFd.fd = socketFd;
    pollFd.events = POLLOUT;
    pollFd.revents = 0;
    do {
        pollResult = poll(&pollFd, 1, timeoutMs);
    } while (pollResult < 0 && errno == EINTR);
    if (pollResult == 0) {
        *detailError = ETIMEDOUT;
        result = BALLOON_NETWORK_RESULT_FAILED;
        goto out;
    }
    if (pollResult < 0 ||
        getsockopt(socketFd, SOL_SOCKET, SO_ERROR, &socketError, &socketErrorSize) < 0) {
        *detailError = errno;
        result = BALLOON_NETWORK_RESULT_EXCEPTION;
        goto out;
    }
    if (socketError == 0) {
        result = BALLOON_NETWORK_RESULT_SUCCESS;
    } else {
        *detailError = socketError;
        result = BalloonWidget_IsExpectedConnectFailure(socketError) ?
                 BALLOON_NETWORK_RESULT_FAILED : BALLOON_NETWORK_RESULT_EXCEPTION;
    }

out:
    close(socketFd);
    return result;
}

static void *BalloonWidget_NetworkTestTask(void *arg)
{
    const char *configuredHost = getenv("BALLOON_NETWORK_TEST_HOST");
    const char *host = configuredHost == NULL || configuredHost[0] == '\0' ?
                       BALLOON_NETWORK_DEFAULT_HOST : configuredHost;
    uint16_t port = BalloonWidget_NetworkPort();
    int timeoutMs = BalloonWidget_NetworkTimeoutMs();
    int detailError = 0;
    E_BalloonNetworkResult result;

    (void) arg;
    result = BalloonWidget_ProbeNetwork(host, port, timeoutMs, &detailError);
    USER_LOG_INFO("Balloon network probe host=%s port=%u timeout_ms=%d result=%d errno=%d",
                  host, port, timeoutMs, result, detailError);
    BalloonWidget_ShowNetworkResult(result);

    pthread_mutex_lock(&s_networkTestMutex);
    s_networkTestRunning = false;
    pthread_mutex_unlock(&s_networkTestMutex);
    return NULL;
}

static T_DjiReturnCode BalloonWidget_StartNetworkTest(void)
{
    pthread_t thread;
    pthread_attr_t attributes;
    int threadResult;

    pthread_mutex_lock(&s_networkTestMutex);
    if (s_networkTestRunning) {
        pthread_mutex_unlock(&s_networkTestMutex);
        USER_LOG_WARN("Balloon network probe rejected: a probe is already running.");
        BalloonWidget_ShowNetworkResult(BALLOON_NETWORK_RESULT_EXCEPTION);
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    s_networkTestRunning = true;
    pthread_mutex_unlock(&s_networkTestMutex);

    if (pthread_attr_init(&attributes) != 0) {
        threadResult = -1;
    } else {
        (void) pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
        threadResult = pthread_create(&thread, &attributes, BalloonWidget_NetworkTestTask, NULL);
        (void) pthread_attr_destroy(&attributes);
    }
    if (threadResult != 0) {
        pthread_mutex_lock(&s_networkTestMutex);
        s_networkTestRunning = false;
        pthread_mutex_unlock(&s_networkTestMutex);
        USER_LOG_ERROR("Balloon network probe thread creation failed: %d", threadResult);
        BalloonWidget_ShowNetworkResult(BALLOON_NETWORK_RESULT_EXCEPTION);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    BalloonWidget_ShowMessage("联网：正在连接");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static const char *BalloonWidget_ApiNetworkMessage(int exitCode)
{
    switch (exitCode) {
        case 0: return "API网络已通；密钥和额度未验证";
        case 2: return "API联网正在处理中，请稍后重试";
        case 10: return "API联网失败：code12不可达";
        case 11: return "API联网失败：SSH代理未就绪";
        case 12: return "API正在校时，请稍后重试";
        case 13: return "API联网失败：TLS证书或时间异常";
        case 14: return "API联网失败：HTTPS请求未完成";
        case 124:
        case 137: return "API联网超时，请稍后重试";
        case 126:
        case 127: return "API联网工具不可用，请检查安装";
        default: return "API联网失败，请检查网络日志";
    }
}

static void *BalloonWidget_ApiNetworkTask(void *arg)
{
    int result, exitCode;
    (void) arg;
    /* No UI/mission lock is held during service or network operations. */
    result = system("/usr/bin/timeout --kill-after=2s 38s /usr/local/bin/uav-api-network-connect");
    exitCode = result >= 0 && WIFEXITED(result) ? WEXITSTATUS(result) :
               result >= 0 && WIFSIGNALED(result) ? 128 + WTERMSIG(result) : -1;
    USER_LOG_INFO("API network helper result=%d exit_code=%d", result, exitCode);
    pthread_mutex_lock(&s_actionMutex);
    BalloonWidget_ShowMessage(BalloonWidget_ApiNetworkMessage(exitCode));
    s_apiNetworkRunning = false;
    BalloonWidget_LogButton(BALLOON_WIDGET_API_NETWORK_INDEX, DJI_WIDGET_BUTTON_STATE_PRESS_DOWN,
                           "async_complete", exitCode == 0 ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS :
                           DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN);
    pthread_mutex_unlock(&s_actionMutex);
    return NULL;
}

/* Called with s_actionMutex held, like the other mission/menu handlers. */
static T_DjiReturnCode BalloonWidget_StartApiNetwork(void)
{
    pthread_t thread;
    pthread_attr_t attributes;
    int threadResult = -1;
    if (s_apiNetworkRunning) {
        BalloonWidget_ShowMessage("API联网正在处理中，请稍候");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    if (BalloonPhotoDetector_IsRunning() || BalloonPhotoDetector_IsStopping() ||
        BalloonWidget_HelperBusy() || BalloonTiming_Requested() || s_timingReserved) {
        BalloonWidget_ShowMessage("请等待任务或校准结束，再进行API联网");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    s_apiNetworkRunning = true;
    if (pthread_attr_init(&attributes) == 0) {
        if (pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED) == 0) {
            threadResult = pthread_create(&thread, &attributes, BalloonWidget_ApiNetworkTask, NULL);
        }
        (void) pthread_attr_destroy(&attributes);
    }
    if (threadResult != 0) {
        s_apiNetworkRunning = false;
        BalloonWidget_ShowMessage("API联网线程启动失败，请重试");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    BalloonWidget_ShowMessage("API联网：正在建立代理并检查HTTPS");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

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

static void BalloonWidget_ShowVoiceMessage(const char *fallback)
{
    FILE *stream;
    char message[512] = {0};
    size_t length;

    stream = fopen(BALLOON_VOICE_MESSAGE_FILE, "rb");
    if (stream != NULL) {
        length = fread(message, 1, sizeof(message) - 1, stream);
        fclose(stream);
        while (length > 0 && (message[length - 1] == '\n' || message[length - 1] == '\r')) {
            message[--length] = '\0';
        }
    }
    BalloonWidget_ShowMessage(message[0] == '\0' ? fallback : message);
}

static void *BalloonWidget_VoiceCommandTask(void *arg)
{
    T_BalloonVoiceTask *task = (T_BalloonVoiceTask *) arg;
    char command[768];
    int result;

    if (task->action == BALLOON_VOICE_RECOGNIZE) {
        snprintf(command, sizeof(command), "timeout --kill-after=2s 60s %s %s --pcm-file %s",
                 BALLOON_VOICE_PYTHON, BALLOON_VOICE_HELPER, task->pcmPath);
    } else if (task->action == BALLOON_VOICE_CONFIRM) {
        snprintf(command, sizeof(command), "timeout --kill-after=2s 15s %s %s --confirm",
                 BALLOON_VOICE_PYTHON, BALLOON_VOICE_HELPER);
    } else {
        snprintf(command, sizeof(command), "timeout --kill-after=2s 15s %s %s --cancel",
                 BALLOON_VOICE_PYTHON, BALLOON_VOICE_HELPER);
    }
    USER_LOG_INFO("Balloon voice task started: action=%d", task->action);
    result = system(command);
    USER_LOG_INFO("Balloon voice task finished: action=%d result=%d", task->action, result);
    pthread_mutex_lock(&s_actionMutex);
    if (task->epoch != s_actionEpoch) {
        if (task->action == BALLOON_VOICE_RECOGNIZE) {
            remove("/home/uav/offline_balloon_strike_test/voice_command_candidate.json");
        }
        if (task->action == BALLOON_VOICE_CONFIRM) {
            remove(BALLOON_NL_PLAN_FILE);
            remove(BALLOON_NL_STATE_FILE);
        }
        BalloonWidget_ShowMessage("语音操作已取消；等待停止完成后重新选择任务");
    } else {
        BalloonWidget_ShowVoiceMessage(result == 0 ? "语音：处理完成" : "语音：失败，请重试或按停止重置");
    }
    free(task);

    pthread_mutex_lock(&s_voiceCommandMutex);
    s_voiceCommandRunning = false;
    pthread_mutex_unlock(&s_voiceCommandMutex);
    pthread_mutex_unlock(&s_actionMutex);
    return NULL;
}

static T_DjiReturnCode BalloonWidget_StartVoiceTask(E_BalloonVoiceAction action, const char *pcmPath)
{
    pthread_t thread;
    pthread_attr_t attributes;
    T_BalloonVoiceTask *task;
    int threadResult;

    if (BalloonWidget_HelperBusy()) {
        BalloonWidget_ShowMessage("计划/语音处理中，请等待；需要取消请按停止");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    pthread_mutex_lock(&s_voiceCommandMutex);
    if (s_voiceCommandRunning) {
        pthread_mutex_unlock(&s_voiceCommandMutex);
        BalloonWidget_ShowMessage("语音：上一条命令仍在处理中");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    s_voiceCommandRunning = true;
    pthread_mutex_unlock(&s_voiceCommandMutex);

    task = calloc(1, sizeof(*task));
    if (task == NULL) {
        threadResult = -1;
    } else {
        task->action = action;
        task->epoch = s_actionEpoch;
        if (pcmPath != NULL) {
            SafeCopyPath(task->pcmPath, sizeof(task->pcmPath), pcmPath);
        }
        if (pthread_attr_init(&attributes) != 0) {
            threadResult = -1;
        } else {
            (void) pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
            threadResult = pthread_create(&thread, &attributes, BalloonWidget_VoiceCommandTask, task);
            (void) pthread_attr_destroy(&attributes);
        }
    }
    if (threadResult != 0) {
        free(task);
        pthread_mutex_lock(&s_voiceCommandMutex);
        s_voiceCommandRunning = false;
        pthread_mutex_unlock(&s_voiceCommandMutex);
        BalloonWidget_ShowMessage("语音：处理线程启动失败");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    BalloonWidget_ShowMessage(action == BALLOON_VOICE_RECOGNIZE ?
                                        "语音：正在识别" : "语音：正在处理");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void BalloonWidget_TimingStep(void)
{
    bool run;
    pthread_mutex_lock(&s_actionMutex);
    run = !BalloonPhotoDetector_IsRunning() && !BalloonWidget_HelperBusy() &&
          (s_timingReserved || BalloonTiming_Requested());
    if (run) s_timingReserved = true;
    pthread_mutex_unlock(&s_actionMutex);
    if (!run) return;

    /* Single watcher owns Timing_Tick and its SDK state. Stop only cancels
     * the request; it must not synchronously join or call the SDK. */
    BalloonTiming_Tick(false);

    pthread_mutex_lock(&s_actionMutex);
    if (!BalloonTiming_Requested() && !s_timingOwnLiveview && !s_timingOwnSubscription)
        s_timingReserved = false;
    pthread_mutex_unlock(&s_actionMutex);
}

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

static T_DjiReturnCode BalloonWidget_StartVoiceWatch(void)
{
    pthread_t thread;
    pthread_attr_t attributes;
    int result;

    if (pthread_attr_init(&attributes) != 0) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    (void) pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
    result = pthread_create(&thread, &attributes, BalloonWidget_VoiceWatchTask, NULL);
    (void) pthread_attr_destroy(&attributes);
    if (result != 0) {
        USER_LOG_ERROR("Balloon voice watcher creation failed: %d", result);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    USER_LOG_INFO("Balloon voice watcher started: %s", BALLOON_VOICE_PCM_DEFAULT);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void SafeCopyPath(char *dst, size_t dstSize, const char *src)
{
    if (src == NULL || src[0] == '\0') {
        return;
    }
    memset(dst, 0, dstSize);
    strncpy(dst, src, dstSize - 1);
}

static bool WriteStateFile(const char *path, const char *state)
{
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        USER_LOG_ERROR("Balloon widget failed to write file: %s", path);
        return false;
    }
    fprintf(fp, "state=%s\n", state);
    fclose(fp);
    USER_LOG_INFO("Balloon widget wrote %s: %s", state, path);
    return true;
}

static void RemoveFileIfExists(const char *path)
{
    remove(path);
}

static void ClearTriggerFiles(void)
{
    RemoveFileIfExists(BALLOON_PREDICTION_RECORD_REQUEST);
    RemoveFileIfExists(s_triggerFilePath);
    RemoveFileIfExists(s_armedFilePath);
    RemoveFileIfExists(s_stopFilePath);
    RemoveFileIfExists(BALLOON_LATEST_TARGET_FILE);
    RemoveFileIfExists(BALLOON_LATEST_PHOTO_FILE);
    RemoveFileIfExists(BALLOON_DETECTION_REQUEST_FILE);
    RemoveFileIfExists(BALLOON_DETECTION_REQUEST_TMP_FILE);
    RemoveFileIfExists(BALLOON_NL_STOP_FILE);
    RemoveFileIfExists(BALLOON_NL_PLAN_FILE);
    RemoveFileIfExists(BALLOON_NL_STATE_FILE);
    USER_LOG_INFO("Balloon widget cleared trigger files.");
}

static bool BuildSiblingLanguagePath(const char *configDirPath,
                                     const char *fromDirName,
                                     const char *toDirName,
                                     char *outPath,
                                     size_t outPathSize)
{
    size_t configPathLen;
    size_t fromDirLen;
    size_t baseLen;

    if (configDirPath == NULL || fromDirName == NULL || toDirName == NULL || outPath == NULL) {
        return false;
    }

    configPathLen = strlen(configDirPath);
    fromDirLen = strlen(fromDirName);
    if (configPathLen < fromDirLen ||
        strcmp(configDirPath + configPathLen - fromDirLen, fromDirName) != 0) {
        return false;
    }

    baseLen = configPathLen - fromDirLen;
    if (baseLen + strlen(toDirName) + 1 > outPathSize) {
        return false;
    }

    memset(outPath, 0, outPathSize);
    strncpy(outPath, configDirPath, baseLen);
    strncat(outPath, toDirName, outPathSize - strlen(outPath) - 1);
    return true;
}

static T_DjiReturnCode RegisterWidgetUiConfig(const char *widgetConfigDirPath)
{
    T_DjiReturnCode code;
    char cnPath[BALLOON_WIDGET_PATH_SIZE] = {0};
    char enPath[BALLOON_WIDGET_PATH_SIZE] = {0};
    bool hasCnPath = false;
    bool hasEnPath = false;

    if (widgetConfigDirPath == NULL || widgetConfigDirPath[0] == '\0') {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    code = DjiWidget_RegDefaultUiConfigByDirPath(widgetConfigDirPath);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWidget_RegDefaultUiConfigByDirPath failed: 0x%08llX", code);
        return code;
    }

    if (BuildSiblingLanguagePath(widgetConfigDirPath, "cn_big_screen", "cn_big_screen",
                                 cnPath, sizeof(cnPath))) {
        hasCnPath = true;
        BuildSiblingLanguagePath(widgetConfigDirPath, "cn_big_screen", "en_big_screen",
                                 enPath, sizeof(enPath));
        hasEnPath = true;
    } else if (BuildSiblingLanguagePath(widgetConfigDirPath, "en_big_screen", "cn_big_screen",
                                        cnPath, sizeof(cnPath))) {
        hasCnPath = true;
        BuildSiblingLanguagePath(widgetConfigDirPath, "en_big_screen", "en_big_screen",
                                 enPath, sizeof(enPath));
        hasEnPath = true;
    }

    if (hasCnPath) {
        code = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_CHINESE,
                                              DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                              cnPath);
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("DjiWidget_RegUiConfigByDirPath Chinese failed: 0x%08llX", code);
            return code;
        }
        USER_LOG_INFO("Balloon widget Chinese UI path: %s", cnPath);
    }

    if (hasEnPath) {
        code = DjiWidget_RegUiConfigByDirPath(DJI_MOBILE_APP_LANGUAGE_ENGLISH,
                                              DJI_MOBILE_APP_SCREEN_TYPE_BIG_SCREEN,
                                              enPath);
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("DjiWidget_RegUiConfigByDirPath English failed: 0x%08llX", code);
            return code;
        }
        USER_LOG_INFO("Balloon widget English UI path: %s", enPath);
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static bool BalloonWidget_WritePlain(const char *path, const char *text)
{
    FILE *fp = fopen(path, "w");
    if (!fp) return false;
    bool ok = fprintf(fp, "%s\n", text) > 0;
    if (fclose(fp) != 0) ok = false;
    return ok;
}

static void BalloonWidget_CloseMissionGate(void)
{
    RemoveFileIfExists(BALLOON_MISSION_TOKEN);
    RemoveFileIfExists(BALLOON_MISSION_COMMIT);
    RemoveFileIfExists(BALLOON_MISSION_READY);
    RemoveFileIfExists(BALLOON_MISSION_HEALTH);
    RemoveFileIfExists(BALLOON_PREDICTION_RECORD_REQUEST);
}

typedef struct {
    unsigned int epoch;
    bool record;
    const char *mode;
    char token[96];
} T_BalloonMissionTask;

static void BalloonWidget_MissionFailure(void)
{
    char reason[320] = {0}, message[512];
    FILE *fp = fopen("/tmp/balloon_mission_message.txt", "r");
    if (fp) { (void) fread(reason, 1, sizeof(reason) - 1, fp); fclose(fp); }
    snprintf(message, sizeof(message), "任务未启动/已停止：%s。等待停止完成后重试",
             reason[0] ? reason : "计划、录像或控制启动未通过检查");
    BalloonWidget_ShowMessage(message);
}

static void *BalloonWidget_MissionTask(void *arg)
{
    T_BalloonMissionTask *task = arg;
    char command[768];
    int result;
    snprintf(command, sizeof(command), "timeout --kill-after=2s 25s %s %s --mode %s --token %s --record %d",
             BALLOON_VOICE_PYTHON, BALLOON_MISSION_HELPER, task->mode, task->token, task->record ? 1 : 0);
    result = system(command);
    pthread_mutex_lock(&s_actionMutex);
    if (task->epoch != s_actionEpoch) goto cancelled;
    if (result != 0 || !BalloonMission_Matches(BALLOON_MISSION_READY, task->token) ||
        !BalloonMission_Healthy(task->token)) goto failed;
    if (task->record && !BalloonWidget_WritePlain(BALLOON_PREDICTION_RECORD_REQUEST, task->token)) goto failed;
    if (!WriteStateFile(s_triggerFilePath, "triggered")) goto failed;
    /* The camera can start now, but the control thread waits for COMMIT. */
    if (BalloonPhotoDetector_Start() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) goto failed;
    BalloonWidget_ShowMessage(task->record ? "计划已确认加载，正在等待录像启动；尚未放行追击" : "计划已确认加载，正在启动任务");
    pthread_mutex_unlock(&s_actionMutex);
    if (task->record) {
        snprintf(command, sizeof(command), "timeout --kill-after=2s 15s %s %s --wait-record --token %s",
                 BALLOON_VOICE_PYTHON, BALLOON_MISSION_HELPER, task->token);
        result = system(command);
    }
    pthread_mutex_lock(&s_actionMutex);
    if (task->epoch != s_actionEpoch) goto cancelled;
    if (result != 0 || !BalloonPhotoDetector_IsRunning() || BalloonPhotoDetector_IsStopping() ||
        !BalloonMission_Healthy(task->token) ||
        !BalloonWidget_WritePlain(BALLOON_MISSION_COMMIT, task->token)) goto failed;
    BalloonWidget_ShowMessage("任务已放行：正在搜索/追击；停止请按停止全部测试");
    goto done;
failed:
    result = -1;
    BalloonWidget_CloseMissionGate();
    (void) BalloonPhotoDetector_StopWithReason("mission_start_failed_or_cancelled");
    RemoveFileIfExists(s_triggerFilePath);
    RemoveFileIfExists(s_armedFilePath);
    s_isArmed = false;
    BalloonWidget_MissionFailure();
    goto done;
cancelled:
    result = -1;
    /* A late subprocess can write a plan, but can never reopen the flight gate.
     * New starts remain blocked by s_missionBusy until this cleanup finishes. */
    BalloonWidget_CloseMissionGate();
    RemoveFileIfExists(BALLOON_NL_PLAN_FILE);
    RemoveFileIfExists(BALLOON_NL_STATE_FILE);
    BalloonWidget_ShowMessage("启动已取消，迟到结果已丢弃");
done:
    s_missionBusy = false;
    uint32_t button = strcmp(task->mode, "pink") == 0 ? BALLOON_WIDGET_PINK_YELLOW_INDEX :
                      strcmp(task->mode, "voice") == 0 ? BALLOON_WIDGET_VOICE_CONFIRM_INDEX :
                      strcmp(task->mode, "next") == 0 ? BALLOON_WIDGET_NEXT_INDEX : BALLOON_WIDGET_ANY_INDEX;
    BalloonWidget_LogButton(button, 1, "async_complete",
                           result == 0 ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN);
    pthread_mutex_unlock(&s_actionMutex);
    free(task);
    return NULL;
}

static T_DjiReturnCode BalloonWidget_BeginMission(const char *mode)
{
    if (BalloonTiming_Requested() || s_timingReserved) {
        BalloonWidget_ShowMessage("地面时间校准采样中，请先结束校准或按停止全部测试，再启动任务");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    pthread_t thread;
    pthread_attr_t attributes;
    int result = -1;
    if (BalloonPhotoDetector_IsRunning() || BalloonWidget_HelperBusy()) {
        BalloonWidget_ShowMessage("任务运行、启动或停止中；请先停止并等待结束，再选择任务");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    if (s_recordEnabled && !BalloonWidget_UsesLiveview()) {
        BalloonWidget_ShowMessage("录像需要liveview；请启用liveview或关闭录像开关后重试");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    T_BalloonMissionTask *task = calloc(1, sizeof(*task));
    if (!task) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    task->epoch = ++s_actionEpoch;
    task->record = s_recordEnabled;
    task->mode = mode;
    char bootClock[80];
    if (!BalloonClock_Id(bootClock)) { free(task); return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN; }
    snprintf(task->token, sizeof(task->token), "mission-%.36s-%lu-%llu-%u", bootClock + 8, (unsigned long) getpid(),
             (unsigned long long) BalloonClock_NowMs(), task->epoch);
    BalloonWidget_CloseMissionGate();
    RemoveFileIfExists("/tmp/balloon_mission_message.txt");
    RemoveFileIfExists("/tmp/balloon_mission_ack.json");
    RemoveFileIfExists(BALLOON_MISSION_STATUS);
    RemoveFileIfExists(s_triggerFilePath);
    RemoveFileIfExists(s_stopFilePath);
    RemoveFileIfExists(BALLOON_NL_STOP_FILE);
    RemoveFileIfExists(BALLOON_LATEST_TARGET_FILE);
    RemoveFileIfExists(BALLOON_DETECTION_REQUEST_FILE);
    if (!WriteStateFile(s_armedFilePath, "armed") || !BalloonWidget_WritePlain(BALLOON_MISSION_TOKEN, task->token)) {
        free(task);
        BalloonWidget_CloseMissionGate();
        RemoveFileIfExists(s_armedFilePath);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    if (!BalloonWidget_WritePlain(BALLOON_EVIDENCE_TOKEN, task->token)) {
        free(task); BalloonWidget_CloseMissionGate(); RemoveFileIfExists(s_armedFilePath);
        BalloonWidget_ShowMessage("无法保存任务记录编号，任务未启动");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    s_isArmed = true;
    s_missionBusy = true;
    if (pthread_attr_init(&attributes) == 0) {
        (void) pthread_attr_setdetachstate(&attributes, PTHREAD_CREATE_DETACHED);
        result = pthread_create(&thread, &attributes, BalloonWidget_MissionTask, task);
        (void) pthread_attr_destroy(&attributes);
    }
    if (result != 0) {
        free(task); s_missionBusy = false; s_isArmed = false;
        BalloonWidget_CloseMissionGate();
        RemoveFileIfExists(s_armedFilePath);
        BalloonWidget_ShowMessage("启动线程失败，请重新选择任务");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    BalloonWidget_ShowStep("preparing", "正在准备任务并等待执行端确认计划；可按停止全部测试取消");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode BalloonWidget_Stop(void)
{
    (void) remove(BALLOON_TIMING_REQUEST);
    ++s_actionEpoch;
    if (s_apiTestRunning) {
        s_apiTestCancelRequested = true;
        s_apiVoiceResync = true;
        /* This PID is owned by the test worker and remains unreaped while registered. */
        if (s_apiTestPid > 1) (void) kill(-s_apiTestPid, SIGTERM);
    }
    s_stopNoticePending = !s_apiTestRunning;
    /* Persist the human stop before closing the gate, so gate-loss does not
     * obscure its origin in the asynchronous control thread. */
    (void) BalloonPhotoDetector_StopWithReason("widget_stop_all");
    BalloonWidget_CloseMissionGate();
    ClearTriggerFiles();
    s_isArmed = false;
    BalloonWidget_ResetButtonValues();
    BalloonWidget_ShowMessage(s_apiTestCleanupFailed ?
        "仅测试：画面资源未释放，请重启PSDK服务后重试；任务启动保持禁用" :
        s_apiTestRunning ? "仅测试：正在取消API测试并释放画面流；请等待完成" :
        "正在停止追击和录像；资源释放完成后可直接选择任务");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode BalloonWidget_SetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData)
{
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    (void) userData;
    pthread_mutex_lock(&s_actionMutex);
    BalloonWidget_LogButton(index, value, "received", code);
    if (index < BALLOON_WIDGET_ANY_INDEX || index >= BALLOON_WIDGET_COUNT ||
        widgetType != (index == BALLOON_WIDGET_RECORD_INDEX ? DJI_WIDGET_TYPE_SWITCH : DJI_WIDGET_TYPE_BUTTON)) {
        BalloonWidget_ShowMessage("旧按钮编号或控件类型不匹配，请刷新PSDK页面");
        goto out;
    }
    if (index == BALLOON_WIDGET_RECORD_INDEX) {
        if (value != 0 && value != 1) goto out;
        if (BalloonPhotoDetector_IsRunning() || BalloonWidget_HelperBusy()) {
            BalloonWidget_ShowMessage("录像设置在任务结束后才能修改");
            code = DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
        } else if (BalloonWidget_WritePlain(BALLOON_RECORD_SETTING, value ? "1" : "0")) {
            s_recordEnabled = value != 0;
            code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            BalloonWidget_ShowMessage(value ? "录像已开启，下次任务随启动自动录制" : "录像已关闭，下次任务不录制");
        } else {
            BalloonWidget_ShowMessage("录像设置保存失败，原设置保持不变");
            code = DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
        }
        goto out;
    }
    if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) {
        code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        goto out;
    }
    switch (index) {
        case BALLOON_WIDGET_ANY_INDEX: code = BalloonWidget_BeginMission("any"); break;
        case BALLOON_WIDGET_PINK_YELLOW_INDEX: code = BalloonWidget_BeginMission("pink"); break;
        case BALLOON_WIDGET_VOICE_CONFIRM_INDEX: code = BalloonWidget_BeginMission("voice"); break;
        case BALLOON_WIDGET_NEXT_INDEX: code = BalloonWidget_BeginMission("next"); break;
        case BALLOON_WIDGET_STOP_INDEX: code = BalloonWidget_Stop(); break;
        case BALLOON_WIDGET_NETWORK_INDEX: code = BalloonWidget_StartNetworkTest(); break;
        case BALLOON_WIDGET_API_NETWORK_INDEX: code = BalloonWidget_StartApiNetwork(); break;
        case BALLOON_WIDGET_API_ASR_INDEX: code = BalloonWidget_StartApiTest("asr", index); break;
        case BALLOON_WIDGET_API_VISION_INDEX: code = BalloonWidget_StartApiTest("vision", index); break;
        case BALLOON_WIDGET_API_INTENT_INDEX: code = BalloonWidget_StartApiTest("intent", index); break;
        case BALLOON_WIDGET_VOICE_CANCEL_INDEX: code = BalloonWidget_StartVoiceTask(BALLOON_VOICE_CANCEL, NULL); break;
        default: break;
    }
out:
    BalloonWidget_LogButton(index, value, "dispatch_result", code);
    pthread_mutex_unlock(&s_actionMutex);
    return code;
}

static T_DjiReturnCode BalloonWidget_GetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData)
{
    (void) widgetType;
    (void) userData;
    if (value == NULL || index >= BALLOON_WIDGET_COUNT) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }
    pthread_mutex_lock(&s_actionMutex);
    *value = index == BALLOON_WIDGET_RECORD_INDEX ? (s_recordEnabled ? 1 : 0) : 0;
    pthread_mutex_unlock(&s_actionMutex);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode BalloonWidget_StartService(const char *widgetConfigDirPath,
                                           const char *armedFilePath,
                                           const char *triggerFilePath,
                                           const char *stopFilePath)
{
    T_DjiReturnCode code;

    SafeCopyPath(s_armedFilePath, sizeof(s_armedFilePath), armedFilePath);
    SafeCopyPath(s_triggerFilePath, sizeof(s_triggerFilePath), triggerFilePath);
    SafeCopyPath(s_stopFilePath, sizeof(s_stopFilePath), stopFilePath);
    BalloonWidget_CloseMissionGate();
    char savedRecord[96];
    if (BalloonMission_ReadToken(BALLOON_RECORD_SETTING, savedRecord)) s_recordEnabled = strcmp(savedRecord, "0") != 0;
    BalloonWidget_ResetButtonValues();

    code = DjiWidget_Init();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWidget_Init failed: 0x%08llX", code);
        return code;
    }

    code = DjiTest_WidgetSpeakerStartService();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiTest_WidgetSpeakerStartService failed: 0x%08llX", code);
        return code;
    }

    code = RegisterWidgetUiConfig(widgetConfigDirPath);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return code;
    }

    code = DjiWidget_RegHandlerList(s_widgetHandlerList, sizeof(s_widgetHandlerList) / sizeof(s_widgetHandlerList[0]));
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiWidget_RegHandlerList failed: 0x%08llX", code);
        return code;
    }

    code = BalloonWidget_StartVoiceWatch();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("Voice command watcher did not start: 0x%08llX", code);
    }

    code = BalloonPhotoDetector_StartAutoExposure();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("Automatic exposure watchdog did not start: 0x%08llX", code);
    }

    USER_LOG_INFO("Balloon widget started. armed=%s trigger=%s stop=%s",
                  s_armedFilePath, s_triggerFilePath, s_stopFilePath);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
