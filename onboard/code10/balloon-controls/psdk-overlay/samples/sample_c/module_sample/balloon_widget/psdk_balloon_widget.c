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
#include <stdatomic.h>
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
#include "target_chase/dji_target_chase_task.h"
#include "balloon_widget/balloon_photo_detector.h"
#include "balloon_clock.h"
#include "balloon_timing_capture.h"
#include "balloon_mission_gate.h"
#include "balloon_evidence.h"
#include "widget/test_widget_speaker.h"

/* New IDs prevent a cached old Prepare button from starting flight. */
#define BALLOON_WIDGET_VOICE_INPUT_INDEX 50
#define BALLOON_WIDGET_PINK_YELLOW_INDEX 19
#define BALLOON_WIDGET_VOICE_CONFIRM_INDEX 55
#define BALLOON_WIDGET_STOP_INDEX 58
#define BALLOON_WIDGET_RECORD_INDEX 22
#define BALLOON_WIDGET_NEXT_INDEX 23
#define BALLOON_WIDGET_NETWORK_INDEX 24
#define BALLOON_WIDGET_VOICE_CANCEL_INDEX 57
#define BALLOON_WIDGET_API_NETWORK_INDEX 51
#define BALLOON_WIDGET_API_ASR_INDEX 27
#define BALLOON_WIDGET_API_VISION_INDEX 28
#define BALLOON_WIDGET_API_INTENT_INDEX 29
#define BALLOON_WIDGET_SAMPLE_INDEX 52
#define BALLOON_WIDGET_AUDIT_INDEX 53
#define BALLOON_WIDGET_KALMAN_INDEX 54
#define BALLOON_WIDGET_NEW_API_INDEX 56
#define BALLOON_WIDGET_CONFIG_VOICE_INPUT_INDEX 60
#define BALLOON_WIDGET_CONFIG_API_NETWORK_INDEX 61
#define BALLOON_WIDGET_CONFIG_VOICE_CONFIRM_INDEX 65
#define BALLOON_WIDGET_CONFIG_NEW_API_INDEX 66
#define BALLOON_WIDGET_CONFIG_VOICE_CANCEL_INDEX 67
#define BALLOON_WIDGET_CONFIG_STOP_INDEX 68
#define BALLOON_WIDGET_MAINTENANCE_INDEX 69
#define BALLOON_WIDGET_COUNT 70
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
static bool s_panelBusy = false, s_panelSampling = false;
static bool s_newVoicePending = false, s_newVoicePromptPending = false;
static int s_predictionMode = 1;
/* Messages and the timing watcher read this outside s_actionMutex. */
static _Atomic bool s_displayOnly = true;
static pid_t s_voicePid = 0, s_missionPid = 0, s_panelPid = 0;
static char s_voiceSession[96] = {0};
static unsigned int s_voiceSequence = 0;
static uint64_t s_sampleConfirmUntil = 0;
/* The watcher and button callback use these only under s_actionMutex. A new
 * input button snapshots the old PCM before prompting, so a fast new recording
 * cannot be consumed by a later asynchronous resync. */
static time_t s_voiceLastSeconds = 0;
static long s_voiceLastNanoseconds = 0;
static off_t s_voiceLastSize = 0;
static int s_voiceStableCount = 0;

static void BalloonWidget_ResetVoiceBaseline(void)
{
    struct stat current;
    bool exists = stat(BALLOON_VOICE_PCM_DEFAULT, &current) == 0 && current.st_size > 0;
    s_voiceLastSeconds = exists ? current.st_mtim.tv_sec : 0;
    s_voiceLastNanoseconds = exists ? current.st_mtim.tv_nsec : 0;
    s_voiceLastSize = exists ? current.st_size : 0;
    s_voiceStableCount = 0;
    s_apiVoiceResync = false;
}

static bool BalloonWidget_HelperBusy(void)
{
    bool busy;
    pthread_mutex_lock(&s_voiceCommandMutex);
    busy = s_voiceCommandRunning;
    pthread_mutex_unlock(&s_voiceCommandMutex);
    busy = busy || s_missionBusy || s_apiNetworkRunning || s_apiTestRunning ||
           s_apiTestLiveviewReserved || s_apiVoiceResync || s_panelBusy;
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
    {BALLOON_WIDGET_VOICE_INPUT_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_API_NETWORK_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_SAMPLE_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_AUDIT_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_KALMAN_INDEX, DJI_WIDGET_TYPE_SWITCH, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_VOICE_CONFIRM_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_NEW_API_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_VOICE_CANCEL_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_STOP_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_CONFIG_VOICE_INPUT_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_CONFIG_API_NETWORK_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_CONFIG_VOICE_CONFIRM_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_CONFIG_NEW_API_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_CONFIG_VOICE_CANCEL_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_CONFIG_STOP_INDEX, DJI_WIDGET_TYPE_BUTTON, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
    {BALLOON_WIDGET_MAINTENANCE_INDEX, DJI_WIDGET_TYPE_SWITCH, BalloonWidget_SetValue, BalloonWidget_GetValue, NULL},
};

/* Distinct SDK indices for the settings page, sharing the same task gates. */
static uint32_t BalloonWidget_ActionIndex(uint32_t index)
{
    switch (index) {
        case BALLOON_WIDGET_CONFIG_VOICE_INPUT_INDEX: return BALLOON_WIDGET_VOICE_INPUT_INDEX;
        case BALLOON_WIDGET_CONFIG_API_NETWORK_INDEX: return BALLOON_WIDGET_API_NETWORK_INDEX;
        case BALLOON_WIDGET_CONFIG_VOICE_CONFIRM_INDEX: return BALLOON_WIDGET_VOICE_CONFIRM_INDEX;
        case BALLOON_WIDGET_CONFIG_NEW_API_INDEX: return BALLOON_WIDGET_NEW_API_INDEX;
        case BALLOON_WIDGET_CONFIG_VOICE_CANCEL_INDEX: return BALLOON_WIDGET_VOICE_CANCEL_INDEX;
        case BALLOON_WIDGET_CONFIG_STOP_INDEX: return BALLOON_WIDGET_STOP_INDEX;
        default: return index;
    }
}

typedef enum {
    BALLOON_VOICE_RECOGNIZE = 0,
    BALLOON_VOICE_CONFIRM,
    BALLOON_VOICE_CANCEL,
} E_BalloonVoiceAction;

typedef struct {
    E_BalloonVoiceAction action;
    unsigned int epoch;
    char pcmPath[BALLOON_WIDGET_PATH_SIZE];
    char session[96];
    bool deferApi;
} T_BalloonVoiceTask;

typedef enum {
    BALLOON_NETWORK_RESULT_SUCCESS = 0,
    BALLOON_NETWORK_RESULT_FAILED,
    BALLOON_NETWORK_RESULT_EXCEPTION,
} E_BalloonNetworkResult;

static void BalloonWidget_ShowMessage(const char *message)
{
    char maintenance[1024];
    if (s_displayOnly) {
        snprintf(maintenance, sizeof(maintenance), "仅显示/维护｜%s", message);
        message = maintenance;
    }
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

#include "balloon_flight_panel.h"

static const char *BalloonWidget_ModeText(void)
{
    return DjiTargetChase_RequestedNoFlight() ? "NO_FLIGHT" : "FLIGHT";
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
    if (s_newVoicePending) {
        BalloonWidget_ShowMessage("新增语音录入中；请先清除候选语音任务，再进行API联网");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
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
        BalloonWidget_ShowMessage("仅测试：检测到录像请求，请先按停止测试并等待结束");
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
    T_BalloonVoiceTask *task = arg;
    char *args[] = {"/usr/bin/timeout", "--kill-after=2s", "100s", BALLOON_VOICE_PYTHON,
                   BALLOON_VOICE_HELPER, "--pcm-file", task->pcmPath, "--session", task->session,
                   task->deferApi ? "--defer-api" : NULL, NULL};
    int result = BalloonPanel_RunChild(args, task->epoch, &s_voicePid);
    pthread_mutex_lock(&s_actionMutex);
    if (task->epoch != s_actionEpoch) {
        BalloonPanel_ClearVoice();
        BalloonWidget_ShowMessage("语音操作已取消，迟到结果已丢弃");
    } else if (result != 0 && result != 1) {
        BalloonPanel_ClearVoice();
        BalloonWidget_ShowMessage(result == 124 || result == 137 ?
            "语音处理超时，候选已清除；请检查ASR/API网络后重新录入" :
            "语音程序异常，候选已清除；请检查部署后重新录入");
    } else {
        if (result != 0) {
            (void)remove(API_VOICE_READY);
            (void)remove(API_VOICE_CANDIDATE);
        }
        BalloonWidget_ShowVoiceMessage(result == 0 ? "语音处理完成" : "语音失败，请重新录入");
    }
    pthread_mutex_lock(&s_voiceCommandMutex);
    s_voiceCommandRunning = false;
    pthread_mutex_unlock(&s_voiceCommandMutex);
    pthread_mutex_unlock(&s_actionMutex);
    free(task);
    return NULL;
}

static T_DjiReturnCode BalloonWidget_StartVoiceTask(E_BalloonVoiceAction action, const char *pcmPath)
{
    pthread_t thread;
    pthread_attr_t attributes;
    T_BalloonVoiceTask *task;
    int threadResult;

    if (action != BALLOON_VOICE_RECOGNIZE || BalloonPhotoDetector_IsRunning() ||
        BalloonTiming_Requested() || s_timingReserved || BalloonWidget_HelperBusy()) {
        BalloonWidget_ShowMessage("任务/语音处理中，请等待；需要取消请按停止测试");
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
    if (!BalloonPanel_NewVoiceSession()) {
        pthread_mutex_lock(&s_voiceCommandMutex);
        s_voiceCommandRunning = false;
        pthread_mutex_unlock(&s_voiceCommandMutex);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    task = calloc(1, sizeof(*task));
    if (task == NULL) {
        threadResult = -1;
    } else {
        task->action = action;
        task->epoch = s_actionEpoch;
        task->deferApi = s_newVoicePending;
        snprintf(task->session, sizeof(task->session), "%s", s_voiceSession);
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
    if (s_displayOnly) return;
    bool run;
    pthread_mutex_lock(&s_actionMutex);
    run = !BalloonPhotoDetector_IsRunning() && !s_missionBusy && !s_voiceCommandRunning &&
          !s_apiNetworkRunning && !s_apiTestRunning && !s_apiTestLiveviewReserved &&
          (!s_panelBusy || s_panelSampling) &&
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

static void BalloonWidget_ShowStoppedNotice(void)
{
    char visualMessage[256];
    const T_DjiReturnCode initResult = BalloonPhotoDetector_GetControlInitResult();
    if (DjiTargetChase_IsReleasePending()) {
        BalloonWidget_ShowMessage("停止未完成：控制权释放失败。请用遥控器原生操作接管，再按停止测试重试");
    } else if (initResult != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        char message[256];
        snprintf(message, sizeof(message), "飞控初始化失败（0x%llX），任务未执行；请查看飞控日志",
                 (unsigned long long) initResult);
        BalloonWidget_ShowMessage(message);
    } else if (BalloonPhotoDetector_GetVisualFailure(visualMessage, sizeof(visualMessage))) {
        BalloonWidget_ShowMessage(visualMessage);
    } else if (DjiTargetChase_GetLastStopResult() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        BalloonWidget_ShowMessage("任务已结束；停止过程有错误，请查看日志并核实飞机状态");
    } else {
        BalloonWidget_ShowMessage("任务已结束，控制线程已退出；录像正在完成保存。可选择新任务");
    }
}

/* Called only with s_actionMutex held; records observed while busy are
 * consumed and never queued for a later mission. */
static void BalloonWidget_PollVoicePcm(bool consume)
{
    struct stat current;
    if (stat(BALLOON_VOICE_PCM_DEFAULT, &current) != 0 || current.st_size <= 0) {
        s_voiceStableCount = 0;
        s_apiVoiceResync = false;
        return;
    }
    if (consume) {
        s_voiceLastSeconds = current.st_mtim.tv_sec;
        s_voiceLastNanoseconds = current.st_mtim.tv_nsec;
        s_voiceLastSize = current.st_size;
        s_voiceStableCount = 0;
        s_apiVoiceResync = false;
    } else if (current.st_mtim.tv_sec == s_voiceLastSeconds &&
               current.st_mtim.tv_nsec == s_voiceLastNanoseconds && current.st_size == s_voiceLastSize) {
        if (s_voiceStableCount > 0 && ++s_voiceStableCount >= 4) {
            s_voiceStableCount = 0;
            (void)BalloonWidget_StartVoiceTask(BALLOON_VOICE_RECOGNIZE, BALLOON_VOICE_PCM_DEFAULT);
        }
    } else {
        s_voiceLastSeconds = current.st_mtim.tv_sec;
        s_voiceLastNanoseconds = current.st_mtim.tv_nsec;
        s_voiceLastSize = current.st_size;
        s_voiceStableCount = 1;
    }
}

static void *BalloonWidget_VoiceWatchTask(void *arg)
{
    (void)arg;
    while (1) {
        usleep(100000);
        BalloonWidget_TimingStep();
        pthread_mutex_lock(&s_actionMutex);
        if (BalloonPhotoDetector_IsRunning()) s_stopNoticePending = true;
        if (s_stopNoticePending && !BalloonPhotoDetector_IsRunning() && !BalloonWidget_HelperBusy()) {
            BalloonWidget_ShowStoppedNotice();
            s_stopNoticePending = false;
        }
        if (s_newVoicePromptPending && BalloonPanel_Idle() && !DjiTargetChase_IsReleasePending()) {
            s_newVoicePromptPending = false;
            s_apiVoiceResync = true;
            BalloonWidget_ShowMessage("请重新按遥控器语音键输入指令；识别完成后回来点击新增API语音任务");
        }
        bool consume = s_apiVoiceResync || BalloonWidget_HelperBusy() ||
                       BalloonPhotoDetector_IsRunning() || BalloonTiming_Requested() ||
                       s_timingReserved || s_newVoicePromptPending;
        BalloonWidget_PollVoicePcm(consume);
        pthread_mutex_unlock(&s_actionMutex);
    }
    return NULL;
}

static T_DjiReturnCode BalloonWidget_StartVoiceWatch(void)
{
    pthread_t thread;
    pthread_attr_t attributes;
    int result;

    /* Set startup resync before launching the watcher. A delayed thread must
     * not override a voice-input button's newer synchronous PCM baseline. */
    pthread_mutex_lock(&s_actionMutex);
    s_apiVoiceResync = true;
    pthread_mutex_unlock(&s_actionMutex);

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
    char voiceSession[96];
    bool kalman;
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
    char *prepare[] = {"/usr/bin/timeout", "--kill-after=2s", "75s", BALLOON_VOICE_PYTHON,
        BALLOON_MISSION_HELPER, "--mode", (char *)task->mode, "--token", task->token,
        "--record", task->record ? "1" : "0", "--voice-session", task->voiceSession, NULL};
    char *check[] = {"/usr/bin/timeout", "--kill-after=2s", "10s", BALLOON_VOICE_PYTHON,
        PANEL_HELPER, "--check-kalman", NULL};
    int result = task->kalman ? BalloonPanel_RunChild(check, task->epoch, &s_missionPid) : 0;
    bool calibrationFailed = result != 0;
    if (result == 0) result = BalloonPanel_RunChild(prepare, task->epoch, &s_missionPid);
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
        char *record[] = {"/usr/bin/timeout", "--kill-after=2s", "15s", BALLOON_VOICE_PYTHON,
            BALLOON_MISSION_HELPER, "--wait-record", "--token", task->token, NULL};
        result = BalloonPanel_RunChild(record, task->epoch, &s_missionPid);
    }
    if (result == 0 && task->kalman) {
        result = BalloonPanel_RunChild(check, task->epoch, &s_missionPid);
        calibrationFailed = result != 0;
    }
    pthread_mutex_lock(&s_actionMutex);
    if (task->epoch != s_actionEpoch) goto cancelled;
    if (result != 0 || !BalloonPhotoDetector_IsRunning() || BalloonPhotoDetector_IsStopping() ||
        !BalloonMission_Healthy(task->token) ||
        !BalloonWidget_WritePlain(BALLOON_MISSION_COMMIT, task->token)) goto failed;
    BalloonWidget_ShowMessage("计划已放行，正在启动飞控；停止请按停止测试");
    goto done;
failed:
    result = -1;
    BalloonWidget_CloseMissionGate();
    (void) BalloonPhotoDetector_StopWithReason("mission_start_failed_or_cancelled");
    RemoveFileIfExists(s_triggerFilePath);
    RemoveFileIfExists(s_armedFilePath);
    s_isArmed = false;
    if (calibrationFailed) {
        char message[512] = {0};
        FILE *fp = fopen(PANEL_MESSAGE, "r");
        if (fp) { (void)fread(message, 1, sizeof(message)-1, fp); fclose(fp); }
        BalloonWidget_ShowMessage(message[0] ? message : "Kalman标定检查失败，任务未放行");
    } else BalloonWidget_MissionFailure();
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
    uint32_t button = strcmp(task->mode, "voice") == 0 ? BALLOON_WIDGET_VOICE_CONFIRM_INDEX :
                      strcmp(task->mode, "api_new") == 0 ? BALLOON_WIDGET_NEW_API_INDEX : BALLOON_WIDGET_VOICE_INPUT_INDEX;
    BalloonWidget_LogButton(button, 1, "async_complete",
                           result == 0 ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN);
    pthread_mutex_unlock(&s_actionMutex);
    free(task);
    return NULL;
}

static T_DjiReturnCode BalloonWidget_BeginMission(const char *mode)
{
    if (s_displayOnly) {
        BalloonWidget_ShowMessage("仅显示模式：维护锁生效，飞行任务未启动；需明确切换到实机测试后使用");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (DjiTargetChase_IsReleasePending()) {
        BalloonWidget_ShowMessage("控制权释放尚未确认，禁止启动新任务；请先停止测试并核实飞机状态");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    bool voice = strcmp(mode, "voice") == 0 || strcmp(mode, "api_new") == 0;
    if (!voice) {
        BalloonWidget_ShowMessage("请先点击语音输入任务；录音生成API候选后再确认执行");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (voice && (!BalloonPanel_VoiceReady() ||
        (strcmp(mode, "voice") == 0 && s_newVoicePending))) {
        BalloonWidget_ShowMessage(s_newVoicePending ? "新录音请点击新增API语音任务执行" : "没有本次录入的有效API候选，请先重新录音");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    if (BalloonTiming_Requested() || s_timingReserved) {
        BalloonWidget_ShowMessage("地面时间校准采样中，请先结束校准或按停止测试，再启动任务");
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
    task->kalman = s_predictionMode == 3;
    snprintf(task->voiceSession, sizeof(task->voiceSession), "%s", voice ? s_voiceSession : "none");
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
    s_newVoicePending = false;
    s_sampleConfirmUntil = 0;
    BalloonWidget_ShowStep("preparing", "正在准备任务并等待执行端确认计划；可按停止测试取消");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode BalloonWidget_Stop(void)
{
    (void) remove(BALLOON_TIMING_REQUEST);
    ++s_actionEpoch;
    s_sampleConfirmUntil = 0;
    s_newVoicePending = false; s_newVoicePromptPending = false;
    s_apiVoiceResync = true;
    BalloonPanel_ClearVoice();
    if (s_voicePid > 1) (void)kill(-s_voicePid, SIGTERM);
    if (s_missionPid > 1) (void)kill(-s_missionPid, SIGTERM);
    if (s_panelPid > 1) (void)kill(-s_panelPid, SIGTERM);
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
    BalloonPanel_RetryRelease();
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

/* Called with s_actionMutex held. The controller setter only changes the
 * requested mode; it cannot initialize, take authority or send commands. */
static T_DjiReturnCode BalloonWidget_SetMaintenance(bool maintenance)
{
    if (s_displayOnly == maintenance) return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    if (!BalloonPanel_Idle() || DjiTargetChase_IsReleasePending() || s_apiTestCleanupFailed ||
        s_isArmed || s_timingOwnLiveview || s_timingOwnSubscription || s_stopNoticePending ||
        s_voicePid || s_missionPid || s_panelPid || s_apiTestPid) {
        BalloonWidget_ShowMessage("请先按停止测试，等待任务、采样和控制权释放完成后，再切换仅显示/维护");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    /* Reset prediction before allowing real operation. A failed write never
     * disables maintenance, and the reported switch retains the actual mode. */
    if (setenv("BALLOON_PREDICTION_MODE", "1", 1) != 0) {
        BalloonWidget_ShowMessage("模式切换失败：预测设置未重置，原模式保持不变");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    s_predictionMode = 1;
    T_DjiReturnCode code = DjiTargetChase_SetPanelMaintenance(maintenance);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        BalloonWidget_ShowMessage("模式切换失败：控制状态未就绪，原模式保持不变；请先停止测试");
        return code;
    }
    s_displayOnly = maintenance;
    ++s_actionEpoch;
    s_sampleConfirmUntil = 0;
    s_newVoicePending = false;
    s_newVoicePromptPending = false;
    s_apiVoiceResync = true;
    BalloonPanel_ClearVoice();
    BalloonWidget_CloseMissionGate();
    ClearTriggerFiles();
    BalloonWidget_ResetButtonValues();
    BalloonWidget_ShowMessage(maintenance ?
        "仅显示/维护已开启：飞行任务和相机采样禁用；候选任务已清除" :
        "实机测试待命：切换未启动任务；请点击语音输入任务，录音后确认执行API语音任务");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode BalloonWidget_SetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t value, void *userData)
{
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    (void)userData;
    const uint32_t rawIndex = index;
    index = BalloonWidget_ActionIndex(index);
    pthread_mutex_lock(&s_actionMutex);
    BalloonWidget_LogButton(rawIndex, value, "received", code);
    if ((index < 50 || index > 58) && index != BALLOON_WIDGET_MAINTENANCE_INDEX) {
        BalloonWidget_ShowMessage("按钮版本不匹配，请刷新PSDK页面");
        goto out;
    }
    if (widgetType != ((index == BALLOON_WIDGET_KALMAN_INDEX || index == BALLOON_WIDGET_MAINTENANCE_INDEX) ?
                       DJI_WIDGET_TYPE_SWITCH : DJI_WIDGET_TYPE_BUTTON)) {
        BalloonWidget_ShowMessage("按钮版本不匹配，请刷新PSDK页面");
        goto out;
    }
    if (index == BALLOON_WIDGET_MAINTENANCE_INDEX) {
        if (value == 0 || value == 1) code = BalloonWidget_SetMaintenance(value != 0);
        goto out;
    }
    if (index == BALLOON_WIDGET_KALMAN_INDEX) {
        if (value != 0 && value != 1) goto out;
        if (value == 1) code = BalloonPanel_StartTool(PANEL_KALMAN);
        else if (!BalloonPanel_Idle()) {
            BalloonWidget_ShowMessage("请先停止当前任务，再切换预测模式");
            code = DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
        } else if (setenv("BALLOON_PREDICTION_MODE", "1", 1) == 0) {
            s_predictionMode = 1;
            BalloonWidget_ShowMessage("已选择基线预测模式1，下次任务生效");
            code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        goto out;
    }
    if (value != DJI_WIDGET_BUTTON_STATE_PRESS_DOWN) { code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS; goto out; }
    switch (index) {
        case BALLOON_WIDGET_VOICE_INPUT_INDEX: code = BalloonPanel_BeginVoiceInput(); break;
        case BALLOON_WIDGET_API_NETWORK_INDEX: code = BalloonWidget_StartApiNetwork(); break;
        case BALLOON_WIDGET_SAMPLE_INDEX: code = BalloonPanel_StartTool(PANEL_SAMPLE); break;
        case BALLOON_WIDGET_AUDIT_INDEX: code = BalloonPanel_StartTool(PANEL_AUDIT); break;
        case BALLOON_WIDGET_VOICE_CONFIRM_INDEX: code = BalloonWidget_BeginMission("voice"); break;
        case BALLOON_WIDGET_NEW_API_INDEX: code = BalloonPanel_NewApiTask(); break;
        case BALLOON_WIDGET_VOICE_CANCEL_INDEX:
            if (s_missionBusy || s_panelBusy || s_apiNetworkRunning) {
                BalloonWidget_ShowMessage("任务正在准备或检查中；终止请按停止测试");
                code = DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
            } else {
                ++s_actionEpoch;
                if (s_voicePid > 1) (void)kill(-s_voicePid, SIGTERM);
                BalloonPanel_ClearVoice();
                s_newVoicePending = false; s_newVoicePromptPending = false;
                s_apiVoiceResync = true;
                BalloonWidget_ShowMessage("候选语音已清除；运行中的飞行任务未改变");
                code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
            }
            break;
        case BALLOON_WIDGET_STOP_INDEX: code = BalloonWidget_Stop(); break;
        default: break;
    }
out:
    BalloonWidget_LogButton(rawIndex, value, "dispatch_result", code);
    pthread_mutex_unlock(&s_actionMutex);
    return code;
}

static T_DjiReturnCode BalloonWidget_GetValue(E_DjiWidgetType widgetType, uint32_t index, int32_t *value, void *userData)
{
    (void) widgetType;
    (void) userData;
    index = BalloonWidget_ActionIndex(index);
    if (value == NULL || ((index < BALLOON_WIDGET_VOICE_INPUT_INDEX || index > BALLOON_WIDGET_STOP_INDEX) &&
        index != BALLOON_WIDGET_MAINTENANCE_INDEX) ||
        widgetType != ((index == BALLOON_WIDGET_KALMAN_INDEX || index == BALLOON_WIDGET_MAINTENANCE_INDEX) ?
                       DJI_WIDGET_TYPE_SWITCH : DJI_WIDGET_TYPE_BUTTON)) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }
    pthread_mutex_lock(&s_actionMutex);
    *value = index == BALLOON_WIDGET_MAINTENANCE_INDEX ? (s_displayOnly ? 1 : 0) :
             index == BALLOON_WIDGET_KALMAN_INDEX ? (s_predictionMode == 3 ? 1 : 0) : 0;
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
    /* Every process start returns to maintenance, regardless of inherited
     * environment or a previous remote-control switch position. */
    s_displayOnly = true;
    code = DjiTargetChase_SetPanelMaintenance(true);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return code;
    /* Flight evidence is always recorded for this panel. No hidden old UI
     * setting may silently disable recording. Restart resets prediction to 1. */
    s_recordEnabled = true;
    s_predictionMode = 1;
    if (setenv("BALLOON_PREDICTION_MODE", "1", 1) != 0)
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    BalloonPanel_ClearVoice();
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
        return code;
    }

    BalloonWidget_ShowMessage("面板已加载｜仅显示/维护已开启，飞行和相机采样禁用");

    USER_LOG_INFO("Balloon widget started. armed=%s trigger=%s stop=%s",
                  s_armedFilePath, s_triggerFilePath, s_stopFilePath);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}
