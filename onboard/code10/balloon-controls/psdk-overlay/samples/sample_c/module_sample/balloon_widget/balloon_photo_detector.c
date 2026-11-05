#include "../balloon_widget/balloon_clock.h"
#include "balloon_mission_gate.h"
#include "balloon_visual_guard.h"
#include "balloon_evidence.h"
#include "balloon_widget/balloon_photo_detector.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "dji_camera_manager.h"
#include "dji_error.h"
#include "dji_logger.h"
#include "dji_liveview.h"
#include "dji_platform.h"
#include "dji_widget.h"
#include "target_chase/dji_target_chase_task.h"

#define BALLOON_CAPTURE_DIR "/home/uav/offline_balloon_strike_test/captures"
#define BALLOON_LATEST_PHOTO_FILE "/tmp/balloon_latest_photo.txt"
#define BALLOON_CAPTURE_EVENT_LOG "/home/uav/offline_balloon_strike_test/capture_events.jsonl"
#define BALLOON_CAMERA_DIAG_LOG "/home/uav/offline_balloon_strike_test/camera_diag.jsonl"
#define BALLOON_DETECTION_LOG "/home/uav/offline_balloon_strike_test/detection_results.jsonl"
#define BALLOON_DETECT_SCRIPT "/home/uav/offline_balloon_strike_test/detect_latest_photo.py"
#define BALLOON_DETECTION_REQUEST_FILE "/home/uav/offline_balloon_strike_test/latest_detection_request.tsv"
#define BALLOON_DETECTION_REQUEST_TMP_FILE "/home/uav/offline_balloon_strike_test/latest_detection_request.tsv.tmp"
#define BALLOON_CONTROL_EVENT_LOG "/home/uav/offline_balloon_strike_test/control_events.jsonl"
#define BALLOON_LATENCY_EVENT_LOG "/home/uav/offline_balloon_strike_test/latency_events.jsonl"
#define BALLOON_LATEST_TARGET_FILE "/home/uav/offline_balloon_strike_test/latest_target.tsv"
#define BALLOON_NL_STOP_FILE "/tmp/balloon_nl_stop.request"
#define BALLOON_TARGET_SEARCH_REQUEST_FILE "/tmp/balloon_target_search.request"
#define BALLOON_LIVEVIEW_FIFO "/tmp/balloon_liveview.h264"
#define BALLOON_LIVEVIEW_METADATA "/dev/shm/balloon_liveview_latest.tsv"
#define BALLOON_PREDICTION_RECORD_REQUEST "/tmp/balloon_prediction_record.request"
#define BALLOON_PATH_SIZE 512
#define BALLOON_PHOTO_INTERVAL_MS 2500
#define BALLOON_MAX_CONSECUTIVE_DOWNLOAD_FAILURES 3
#define BALLOON_CONTROL_LOOP_INTERVAL_MS 50
#define BALLOON_DEFAULT_TARGET_TIMEOUT_MS 800ULL
#define BALLOON_DEFAULT_FAR_TARGET_TIMEOUT_MS 1400ULL
#define BALLOON_FAR_TARGET_MAX_AREA_RATIO 0.003f
#define BALLOON_DEFAULT_MIL_MAX_AGE_MS 250ULL
#define BALLOON_DEFAULT_MIL_SUPPRESS_AFTER_YOLO_MS 500ULL
#define BALLOON_DEFAULT_REAL_CONTROL_MAX_AFTER_TARGET_MS 300000ULL
#define BALLOON_DEFAULT_PHOTO_LOOP_MAX_DURATION_MS 300000ULL
#define BALLOON_RUN_ONCE_LOG_INTERVAL_MS 1000ULL
#define BALLOON_HOVER_WAIT_FOR_DETECTION_MS 100000ULL
#define BALLOON_NEW_MEDIA_TIMEOUT_MS 12000ULL
#define BALLOON_NEW_MEDIA_POLL_MS 600ULL
#define BALLOON_LIVEVIEW_REQUEST_INTERVAL_MS 40ULL
#define BALLOON_LIVEVIEW_I_FRAME_INTERVAL_MS 5000ULL
#define BALLOON_LIVEVIEW_FRAME_TIMEOUT_MS 3000ULL
#define BALLOON_OPERATOR_STATUS_INTERVAL_MS 2000ULL
#define BALLOON_DEFAULT_AUTO_EXPOSURE_REFRESH_MS 1000ULL
#define BALLOON_TRACKER_SEQ_FLAG 0x80000000U
#define BALLOON_YOLO_FILTER_ALPHA 0.70f
#define BALLOON_TRACKER_FILTER_ALPHA 0.30f
#define BALLOON_TRACKER_MAX_CENTER_JUMP_NORMALIZED 0.16f
#define BALLOON_TARGET_FILTER_RESET_GAP_MS 1500ULL

static volatile bool s_workerActive = false;
static volatile bool s_stopRequested = false;
static volatile bool s_controlActive = false;
static volatile bool s_controlStopRequested = false;
/* Keep initialization failure separate from teardown: a successful DeInit
 * must not turn a refused flight task into a normal completion notice. */
static T_DjiReturnCode s_controlInitResult = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
/* Atomics publish an enduring operator failure independently of SDK cleanup. */
static unsigned s_visualFailureReason = 0;
static unsigned s_visualFailureColor = 0;
static bool s_cameraManagerReady = false;
static bool s_downloaderRightsHeld = false;
static T_DjiTaskHandle s_workerTask = NULL;
static T_DjiTaskHandle s_controlTask = NULL;
static T_DjiTaskHandle s_autoExposureTask = NULL;
static volatile bool s_autoExposureStopRequested = false;
static FILE *s_downloadFile = NULL;
static char s_downloadTempPath[BALLOON_PATH_SIZE] = {0};
static char s_downloadFinalPath[BALLOON_PATH_SIZE] = {0};
static bool s_downloadCompleted = false;
static uint32_t s_captureSeq = 0;
static uint64_t s_lastDownloadedKey = 0;
static uint32_t s_lastDownloadedFileIndex = 0;
static int s_liveviewFifoFd = -1;
static bool s_liveviewInitialized = false;
static bool s_liveviewStreamStarted = false;
static uint64_t s_liveviewBytesReceived = 0;
static uint64_t s_liveviewBytesDropped = 0;
static bool s_nlStopObserved = false;
static bool s_hitUnconfirmedStop = false;

typedef struct {
    uint64_t frameId;
    uint64_t readyTsMs;
    uint64_t encodeMs;
    char path[BALLOON_PATH_SIZE];
} T_BalloonLiveviewFrame;

static bool ReadLatestLiveviewFrame(T_BalloonLiveviewFrame *frame);

static void AppendControlEvent(const char *event, T_DjiReturnCode code, const char *detail);

static void BalloonControl_RecordInitResult(T_DjiReturnCode code, bool cancelled)
{
    __atomic_store_n(&s_controlInitResult,
                     cancelled ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS : code,
                     __ATOMIC_RELEASE);
}

T_DjiReturnCode BalloonPhotoDetector_GetControlInitResult(void)
{
    return __atomic_load_n(&s_controlInitResult, __ATOMIC_ACQUIRE);
}

static const char *BalloonVisual_ColorText(unsigned color)
{
    static const char *names[] = {"目标", "粉色", "红色", "橙色", "黄色", "绿色", "蓝色", "紫色"};
    return color < sizeof(names) / sizeof(names[0]) ? names[color] : names[0];
}

static void BalloonVisual_RecordFailure(unsigned reason, const char *color)
{
    static const char *colors[] = {"any", "pink", "red", "orange", "yellow", "green", "blue", "purple"};
    unsigned index = 0;
    if (color) {
        for (unsigned i = 1; i < sizeof(colors) / sizeof(colors[0]); ++i)
            if (!strcmp(color, colors[i])) index = i;
    }
    __atomic_store_n(&s_visualFailureColor, index, __ATOMIC_RELAXED);
    __atomic_store_n(&s_visualFailureReason, reason, __ATOMIC_RELEASE);
}

bool BalloonPhotoDetector_GetVisualFailure(char *message, size_t size)
{
    unsigned reason = __atomic_load_n(&s_visualFailureReason, __ATOMIC_ACQUIRE);
    unsigned color = __atomic_load_n(&s_visualFailureColor, __ATOMIC_RELAXED);
    if (!message || !size || !reason) return false;
    if (reason == 1)
        snprintf(message, size, "当前画面未识别到%s气球，请对准目标后重新执行", BalloonVisual_ColorText(color));
    else if (reason == 2)
        snprintf(message, size, "未收到当前任务的有效相机图像，任务未执行；请检查图传后重试");
    else
        snprintf(message, size, "%s气球目标丢失，任务已停止；请对准目标后重新执行", BalloonVisual_ColorText(color));
    return true;
}

static bool NaturalLanguageStopRequested(void)
{
    if (access(BALLOON_NL_STOP_FILE, F_OK) != 0) {
        return false;
    }
    if (!s_nlStopObserved) {
        char reason[256] = {0};
        FILE *marker = fopen(BALLOON_NL_STOP_FILE, "r");
        if (marker != NULL) {
            (void) fread(reason, 1, sizeof(reason) - 1, marker);
            fclose(marker);
        }
        s_hitUnconfirmedStop = strstr(reason, "reason=capability_hit_unconfirmed") != NULL;
        AppendControlEvent("stop_marker_observed", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
            s_hitUnconfirmedStop ? "hit_unconfirmed" :
            (strstr(reason, "mission_plan_invalid") ? "mission_plan_invalid" :
             strstr(reason, "capability_plan_complete") ? "capability_plan_complete" :
             strstr(reason, "capability_stop_action") ? "capability_stop_action" :
             strstr(reason, "capability_executor_missing") ? "capability_executor_missing" : "other_stop_marker"));
        USER_LOG_WARN("Natural-language stop marker observed: %s", BALLOON_NL_STOP_FILE);
        s_nlStopObserved = true;
    }
    s_stopRequested = true;
    s_controlStopRequested = true;
    return true;
}

static bool UseLiveviewInput(void)
{
    const char *value = getenv("BALLOON_INPUT_MODE");

    return value != NULL &&
           (strcmp(value, "liveview") == 0 || strcmp(value, "LIVEVIEW") == 0 ||
            strcmp(value, "stream") == 0 || strcmp(value, "STREAM") == 0);
}

static const char *BalloonStep_ModeText(void)
{
    return DjiTargetChase_RequestedNoFlight() ? "NO_FLIGHT" : "FLIGHT";
}

static void BalloonStep_Show(const char *step, const char *detail)
{
    char message[256];

    snprintf(message, sizeof(message),
             "Balloon step: %s | mode=%s | %s",
             step == NULL ? "unknown" : step,
             BalloonStep_ModeText(),
             detail == NULL ? "" : detail);
    USER_LOG_INFO("%s", message);
}

static bool ReadBoolEnv(const char *name, bool defaultValue)
{
    const char *value = getenv(name);

    if (value == NULL || value[0] == '\0') {
        return defaultValue;
    }
    if (strcmp(value, "1") == 0 || strcmp(value, "true") == 0 ||
        strcmp(value, "TRUE") == 0 || strcmp(value, "on") == 0 ||
        strcmp(value, "ON") == 0) {
        return true;
    }
    if (strcmp(value, "0") == 0 || strcmp(value, "false") == 0 ||
        strcmp(value, "FALSE") == 0 || strcmp(value, "off") == 0 ||
        strcmp(value, "OFF") == 0) {
        return false;
    }

    return defaultValue;
}

static void BalloonStep_ShowSeq(const char *step,
                                uint32_t seq,
                                T_DjiReturnCode code,
                                const char *detail)
{
    char message[256];

    snprintf(message, sizeof(message),
             "Balloon step: %s | mode=%s | seq=%u | code=0x%08llX | %s",
             step == NULL ? "unknown" : step,
             BalloonStep_ModeText(),
             seq,
             (unsigned long long) code,
             detail == NULL ? "" : detail);
    USER_LOG_INFO("%s", message);
}

typedef struct {
    uint32_t seq;
    float width;
    float height;
    float x1;
    float y1;
    float x2;
    float y2;
    uint64_t targetTsMs;
    uint64_t detectionFinishedTsMs;
    float confidence;
    char source[BALLOON_PATH_SIZE];
} T_BalloonTargetRecord;

static float BalloonTargetAreaRatio(const T_BalloonTargetRecord *target)
{
    float boxWidth;
    float boxHeight;

    if (target == NULL ||
        target->width <= 0.0f ||
        target->height <= 0.0f) {
        return 0.0f;
    }
    boxWidth = fmaxf(0.0f, target->x2 - target->x1);
    boxHeight = fmaxf(0.0f, target->y2 - target->y1);
    return boxWidth * boxHeight / (target->width * target->height);
}

static uint64_t BalloonTargetAdaptiveTimeoutMs(
    const T_BalloonTargetRecord *target,
    uint64_t targetTimeoutMs,
    uint64_t farTargetTimeoutMs)
{
    float areaRatio = BalloonTargetAreaRatio(target);

    return areaRatio > 0.0f &&
                   areaRatio <= BALLOON_FAR_TARGET_MAX_AREA_RATIO
               ? farTargetTimeoutMs
               : targetTimeoutMs;
}

typedef struct {
    bool valid;
    float imageWidth;
    float imageHeight;
    float centerX;
    float centerY;
    float boxWidth;
    float boxHeight;
    uint64_t targetTsMs;
} T_BalloonTargetFilter;

static const char *BalloonTargetSourceText(uint32_t seq)
{
    return (seq & BALLOON_TRACKER_SEQ_FLAG) != 0U ? "MIL" : "YOLO";
}

static float BalloonClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool BalloonFilterTarget(T_BalloonTargetFilter *filter,
                                T_BalloonTargetRecord *target)
{
    bool trackerTarget;
    bool resetFilter;
    bool jumpClamped = false;
    float centerX;
    float centerY;
    float boxWidth;
    float boxHeight;
    float alpha;

    if (filter == NULL || target == NULL ||
        target->width <= 1.0f || target->height <= 1.0f) {
        return false;
    }

    trackerTarget = (target->seq & BALLOON_TRACKER_SEQ_FLAG) != 0U;
    centerX = (target->x1 + target->x2) * 0.5f;
    centerY = (target->y1 + target->y2) * 0.5f;
    boxWidth = target->x2 - target->x1;
    boxHeight = target->y2 - target->y1;
    resetFilter = !filter->valid ||
                  fabsf(filter->imageWidth - target->width) > 1.0f ||
                  fabsf(filter->imageHeight - target->height) > 1.0f ||
                  (target->targetTsMs > filter->targetTsMs &&
                   target->targetTsMs - filter->targetTsMs > BALLOON_TARGET_FILTER_RESET_GAP_MS);

    if (!resetFilter) {
        float deltaX = centerX - filter->centerX;
        float deltaY = centerY - filter->centerY;
        float normalizedJump = sqrtf((deltaX / target->width) * (deltaX / target->width) +
                                     (deltaY / target->height) * (deltaY / target->height));

        if (trackerTarget && normalizedJump > BALLOON_TRACKER_MAX_CENTER_JUMP_NORMALIZED) {
            float scale = BALLOON_TRACKER_MAX_CENTER_JUMP_NORMALIZED / normalizedJump;
            centerX = filter->centerX + deltaX * scale;
            centerY = filter->centerY + deltaY * scale;
            jumpClamped = true;
        }

        alpha = trackerTarget ? BALLOON_TRACKER_FILTER_ALPHA : BALLOON_YOLO_FILTER_ALPHA;
        centerX = filter->centerX + alpha * (centerX - filter->centerX);
        centerY = filter->centerY + alpha * (centerY - filter->centerY);
        boxWidth = filter->boxWidth + alpha * (boxWidth - filter->boxWidth);
        boxHeight = filter->boxHeight + alpha * (boxHeight - filter->boxHeight);
    }

    boxWidth = BalloonClampFloat(boxWidth, 2.0f, target->width);
    boxHeight = BalloonClampFloat(boxHeight, 2.0f, target->height);
    centerX = BalloonClampFloat(centerX, boxWidth * 0.5f, target->width - boxWidth * 0.5f);
    centerY = BalloonClampFloat(centerY, boxHeight * 0.5f, target->height - boxHeight * 0.5f);

    target->x1 = centerX - boxWidth * 0.5f;
    target->x2 = centerX + boxWidth * 0.5f;
    target->y1 = centerY - boxHeight * 0.5f;
    target->y2 = centerY + boxHeight * 0.5f;

    filter->valid = true;
    filter->imageWidth = target->width;
    filter->imageHeight = target->height;
    filter->centerX = centerX;
    filter->centerY = centerY;
    filter->boxWidth = boxWidth;
    filter->boxHeight = boxHeight;
    filter->targetTsMs = target->targetTsMs;
    return jumpClamped;
}

static void BalloonTargetDirectionText(const T_BalloonTargetRecord *target,
                                       char *text,
                                       size_t textSize)
{
    float centerX;
    float centerY;
    const char *horizontal;
    const char *vertical;

    if (target == NULL || text == NULL || textSize == 0U ||
        target->width <= 1.0f || target->height <= 1.0f) {
        return;
    }

    centerX = (target->x1 + target->x2) * 0.5f;
    centerY = (target->y1 + target->y2) * 0.5f;
    horizontal = centerX < target->width * 0.40f ? "LEFT" :
                 centerX > target->width * 0.60f ? "RIGHT" : "CENTER";
    vertical = centerY < target->height * 0.40f ? "UP" :
               centerY > target->height * 0.60f ? "DOWN" : "CENTER";
    snprintf(text, textSize, "%s/%s", horizontal, vertical);
}

static void BalloonCommandText(const T_TargetChaseCommand *command,
                               bool hasCommand,
                               char *text,
                               size_t textSize)
{
    const char *forward;
    const char *side;
    const char *yaw;
    const char *vertical;

    if (text == NULL || textSize == 0U) {
        return;
    }
    if (!hasCommand || command == NULL || !command->should_send) {
        snprintf(text, textSize, "HOLD");
        return;
    }

    forward = command->x_mps > 0.05f ? "FWD" :
              command->x_mps < -0.05f ? "BACK" : "";
    side = command->y_mps > 0.05f ? "RIGHT" :
           command->y_mps < -0.05f ? "LEFT" : "";
    yaw = command->yaw_rate_degps > 0.5f ? "TURN_R" :
          command->yaw_rate_degps < -0.5f ? "TURN_L" : "";
    vertical = command->z_mps > 0.05f ? "CLIMB" :
               command->z_mps < -0.05f ? "DESCEND" : "";
    if (forward[0] == '\0' && side[0] == '\0' && yaw[0] == '\0' && vertical[0] == '\0') {
        snprintf(text, textSize, "HOVER");
        return;
    }

    snprintf(text, textSize, "%s%s%s%s%s%s%s",
             forward,
             forward[0] != '\0' && side[0] != '\0' ? "+" : "", side,
             (forward[0] != '\0' || side[0] != '\0') && yaw[0] != '\0' ? "+" : "", yaw,
             (forward[0] != '\0' || side[0] != '\0' || yaw[0] != '\0') && vertical[0] != '\0' ? "+" : "",
             vertical);
}

static void BalloonShowOperatorStatus(bool noFlightMode,
                                      bool hasActiveTarget,
                                      bool targetLost,
                                      const T_BalloonTargetRecord *target,
                                      uint64_t nowMs,
                                      uint64_t taskStartMs)
{
    T_TargetChaseCommand command;
    T_BalloonLiveviewFrame liveviewFrame;
    char action[72] = {0};
    char direction[32] = "UNKNOWN";
    char message[256];
    bool hasCommand = DjiTargetChase_GetLastCommand(&command);
    bool approaching = false;
    bool decoderHealthy = true;
    uint64_t ageMs = 0;

    BalloonCommandText(&command, hasCommand, action, sizeof(action));
    approaching = !noFlightMode && hasActiveTarget && hasCommand &&
                  command.should_send && !command.reached && command.x_mps > 0.05f;
    if (UseLiveviewInput() && nowMs >= taskStartMs &&
        nowMs - taskStartMs > BALLOON_LIVEVIEW_FRAME_TIMEOUT_MS) {
        decoderHealthy = ReadLatestLiveviewFrame(&liveviewFrame) &&
                         nowMs >= liveviewFrame.readyTsMs &&
                         nowMs - liveviewFrame.readyTsMs <= BALLOON_LIVEVIEW_FRAME_TIMEOUT_MS;
    }
    if (target != NULL) {
        BalloonTargetDirectionText(target, direction, sizeof(direction));
        if (nowMs >= target->targetTsMs) {
            ageMs = nowMs - target->targetTsMs;
        }
    }

    if (hasActiveTarget && target != NULL) {
        snprintf(message, sizeof(message),
                 "STATE=%s | TARGET=YES | src=%s | %s=%s | pos=%s age=%llums | gimbal=%s",
                 approaching ? "APPROACHING" : "TARGET_FOUND",
                 BalloonTargetSourceText(target->seq),
                 noFlightMode ? "PLAN" : "ACTION",
                 action,
                 direction,
                 (unsigned long long) ageMs,
                 DjiTargetChase_GetGimbalStateText());
    } else if (!decoderHealthy) {
        snprintf(message, sizeof(message),
                 "STATE=DECODER_ERROR | TARGET=NO | frame=MISSING_OR_STALE | gimbal=%s",
                 DjiTargetChase_GetGimbalStateText());
    } else if (targetLost) {
        snprintf(message, sizeof(message),
                 "STATE=TARGET_LOST | TARGET=NO | %s=%s | waiting detection | gimbal=%s",
                 noFlightMode ? "PLAN" : "ACTION",
                 action,
                 DjiTargetChase_GetGimbalStateText());
    } else {
        snprintf(message, sizeof(message),
                 "STATE=STANDBY | TARGET=NO | flight=%s | action=HOLD | gimbal=%s",
                 noFlightMode ? "SAFE_NO_FLIGHT" : "FLIGHT",
                 DjiTargetChase_GetGimbalStateText());
    }
    char missionStatus[256] = {0}, combined[512];
    FILE *statusFile = fopen(BALLOON_MISSION_STATUS, "r");
    if (statusFile) {
        (void) fread(missionStatus, 1, sizeof(missionStatus) - 1, statusFile);
        fclose(statusFile);
    }
    snprintf(combined, sizeof(combined), "%s\n%s", missionStatus, message);
    if (strlen(combined) > DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN) {
        size_t length = DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN;
        while (length > 0 && ((unsigned char) combined[length] & 0xC0) == 0x80) --length;
        combined[length] = '\0';
    }
    DjiWidgetFloatingWindow_ShowMessage(combined);
}

typedef struct {
    bool valid;
    uint64_t key;
    T_DjiCameraManagerFileListInfo info;
} T_BalloonJpegSnapshot;

static uint64_t NowMs(void)
{
    return BalloonClock_NowMs();
}

static uint64_t MonoMs(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return NowMs();
    }

    return ((uint64_t) ts.tv_sec * 1000ULL) + ((uint64_t) ts.tv_nsec / 1000000ULL);
}

static float ReadFloatEnvBounded(const char *name, float defaultValue, float minValue, float maxValue)
{
    const char *value;
    char *end = NULL;
    float parsed;

    value = getenv(name);
    if (value == NULL || value[0] == '\0') {
        return defaultValue;
    }

    errno = 0;
    parsed = strtof(value, &end);
    if (errno != 0 || end == value || parsed < minValue || parsed > maxValue) {
        return defaultValue;
    }

    return parsed;
}

static uint64_t ReadU64EnvBounded(const char *name,
                                  uint64_t defaultValue,
                                  uint64_t minValue,
                                  uint64_t maxValue,
                                  bool allowZero)
{
    const char *value;
    char *end = NULL;
    unsigned long long parsed;

    value = getenv(name);
    if (value == NULL || value[0] == '\0') {
        return defaultValue;
    }

    errno = 0;
    parsed = strtoull(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0') {
        return defaultValue;
    }
    if (allowZero && parsed == 0ULL) {
        return 0ULL;
    }
    if (parsed < minValue || parsed > maxValue) {
        return defaultValue;
    }

    return (uint64_t) parsed;
}

static void AppendCameraDiag(const char *field,
                             E_DjiMountPosition position,
                             T_DjiReturnCode code,
                             long long value,
                             const char *text)
{
    FILE *fp = fopen(BALLOON_CAMERA_DIAG_LOG, "a");
    time_t now = time(NULL);

    if (fp == NULL) {
        USER_LOG_ERROR("Balloon photo detector failed to open camera diag log: %s", BALLOON_CAMERA_DIAG_LOG);
        return;
    }

    fprintf(fp,
            "{\"ts\":%ld,\"field\":\"%s\",\"position\":%d,\"code\":\"0x%08llX\","
            "\"ok\":%s,\"value\":%lld,\"text\":\"%s\"}\n",
            (long) now,
            field,
            (int) position,
            (unsigned long long) code,
            code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "true" : "false",
            value,
            text == NULL ? "" : text);
    fclose(fp);
}

static void AppendCameraDiagEvent(const char *event)
{
    AppendCameraDiag(event, DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, 0, "");
}

static bool IsIgnorableCameraSetupCode(T_DjiReturnCode code)
{
    return code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
           code == DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND ||
           code == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT ||
           code == DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
}

static void LogCameraReadback(E_DjiMountPosition position)
{
    T_DjiReturnCode code;
    bool connectStatus = false;
    E_DjiCameraType cameraType = 0;
    T_DjiCameraManagerFirmwareVersion firmwareVersion = {0};
    E_DjiCameraManagerExposureMode exposureMode = DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_UNKNOWN;
    E_DjiCameraManagerISO iso = 0;
    E_DjiCameraManagerAperture aperture = DJI_CAMERA_MANAGER_APERTURE_F_UNKNOWN;
    E_DjiCameraManagerShutterSpeed shutterSpeed = DJI_CAMERA_MANAGER_SHUTTER_SPEED_UNKNOWN;
    char firmwareText[64] = {0};

    AppendCameraDiagEvent("camera_readback_begin");

    code = DjiCameraManager_GetCameraConnectStatus(position, &connectStatus);
    AppendCameraDiag("connect_status", position, code, connectStatus ? 1 : 0, "");

    code = DjiCameraManager_GetCameraType(position, &cameraType);
    AppendCameraDiag("camera_type", position, code, (long long) cameraType, "");

    code = DjiCameraManager_GetFirmwareVersion(position, &firmwareVersion);
    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        snprintf(firmwareText, sizeof(firmwareText), "V%02u.%02u.%02u.%02u",
                 firmwareVersion.firmware_version[0],
                 firmwareVersion.firmware_version[1],
                 firmwareVersion.firmware_version[2],
                 firmwareVersion.firmware_version[3]);
    }
    AppendCameraDiag("firmware_version", position, code, 0, firmwareText);

    code = DjiCameraManager_GetExposureMode(position, &exposureMode);
    AppendCameraDiag("exposure_mode", position, code, (long long) exposureMode, "");

    code = DjiCameraManager_GetISO(position, &iso);
    AppendCameraDiag("iso", position, code, (long long) iso, "");

    code = DjiCameraManager_GetAperture(position, &aperture);
    AppendCameraDiag("aperture", position, code, (long long) aperture, "");

    code = DjiCameraManager_GetShutterSpeed(position, &shutterSpeed);
    AppendCameraDiag("shutter_speed", position, code, (long long) shutterSpeed,
                     shutterSpeed == DJI_CAMERA_MANAGER_SHUTTER_SPEED_1_60 ? "expected_1_60" : "");

    AppendCameraDiagEvent("camera_readback_end");
}

static T_DjiReturnCode DownloadMediaFileList(E_DjiMountPosition position, T_DjiCameraManagerFileList *fileList)
{
    T_DjiReturnCode code;
    T_DjiCameraManagerSliceConfig sliceConfig = {
        .sliceStartIndex = 0,
        .countPerSlice = DJI_CAMERA_MANAGER_FILE_LIST_COUNT_ALL_PER_SLICE,
    };

    code = DjiCameraManager_DownloadFileList(position, fileList);
    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_INFO("Downloaded camera media list with DjiCameraManager_DownloadFileList.");
        return code;
    }

    USER_LOG_WARN("DjiCameraManager_DownloadFileList failed: 0x%08llX, try DownloadFileListBySlices.",
                  (unsigned long long) code);

    code = DjiCameraManager_DownloadFileListBySlices(position, sliceConfig, fileList);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiCameraManager_DownloadFileListBySlices failed: 0x%08llX",
                       (unsigned long long) code);
    }

    return code;
}

static T_DjiReturnCode ObtainDownloaderRightsTracked(E_DjiMountPosition position)
{
    T_DjiReturnCode code = DjiCameraManager_ObtainDownloaderRights(position);

    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_downloaderRightsHeld = true;
    }

    return code;
}

static void ReleaseDownloaderRightsTracked(E_DjiMountPosition position, const char *reason)
{
    T_DjiReturnCode code;

    if (!s_downloaderRightsHeld) {
        return;
    }

    code = DjiCameraManager_ReleaseDownloaderRights(position);
    AppendCameraDiag("release_downloader_rights", position, code, 0, reason == NULL ? "" : reason);
    s_downloaderRightsHeld = false;
}

static void CloseDownloadFile(void)
{
    if (s_downloadFile != NULL) {
        fclose(s_downloadFile);
        s_downloadFile = NULL;
    }
}

static void CleanupCameraManager(E_DjiMountPosition position, const char *reason)
{
    T_DjiReturnCode code;

    ReleaseDownloaderRightsTracked(position, reason);
    CloseDownloadFile();

    if (s_downloadTempPath[0] != '\0' && !s_downloadCompleted) {
        remove(s_downloadTempPath);
    }

    if (!s_cameraManagerReady) {
        return;
    }

    code = DjiCameraManager_DeInit();
    AppendCameraDiag("camera_manager_deinit", position, code, 0, reason == NULL ? "" : reason);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("DjiCameraManager_DeInit returned: 0x%08llX", (unsigned long long) code);
    }
    s_cameraManagerReady = false;
}

static void AppendEvent(const char *event, uint32_t seq, T_DjiReturnCode code, const char *path)
{
    FILE *fp = fopen(BALLOON_CAPTURE_EVENT_LOG, "a");
    char clockId[80] = {0}, token[96];
    (void) BalloonClock_Id(clockId);
    BalloonEvidence_Token(token);
    time_t now = time(NULL);
    uint64_t nowMs = NowMs();

    if (fp == NULL) {
        USER_LOG_ERROR("Balloon photo detector failed to open event log: %s", BALLOON_CAPTURE_EVENT_LOG);
        return;
    }

    fprintf(fp,
            "{\"clock_id\":\"%s\",\"mission_token\":\"%s\",\"ts\":%lld,\"ts_ms\":%llu,\"event\":\"%s\",\"seq\":%u,"
            "\"code\":\"0x%08llX\",\"path\":\"%s\"}\n",
            clockId, token, (long long) now,
            (unsigned long long) nowMs,
            event == NULL ? "" : event,
            seq,
            (unsigned long long) code,
            path == NULL ? "" : path);
    fclose(fp);
}

static void AppendControlEvent(const char *event, T_DjiReturnCode code, const char *detail)
{
    FILE *fp = fopen(BALLOON_CONTROL_EVENT_LOG, "a");
    char clockId[80] = {0}, token[96];
    BalloonEvidence_Token(token);
    (void) BalloonClock_Id(clockId);

    if (fp == NULL) {
        return;
    }

    fprintf(fp,
            "{\"clock\":\"monotonic-v1\",\"ts_ms\":%llu,\"source\":\"balloon_photo_detector\","
            "\"clock_id\":\"%s\",\"mission_token\":\"%s\","
            "\"event\":\"%s\",\"code\":\"0x%llX\",\"detail\":\"%s\"}\n",
            (unsigned long long) NowMs(),
            clockId, token,
            event == NULL ? "" : event,
            (unsigned long long) code,
            detail == NULL ? "" : detail);
    fclose(fp);
}

static void AppendLatencyEvent(const char *stage,
                               uint32_t seq,
                               uint64_t durationMs,
                               T_DjiReturnCode code)
{
    FILE *fp = fopen(BALLOON_LATENCY_EVENT_LOG, "a");
    char clockId[80] = {0}, token[96];

    if (fp == NULL) {
        return;
    }
    (void) BalloonClock_Id(clockId);
    BalloonEvidence_Token(token);

    fprintf(fp,
            "{\"clock\":\"monotonic-v1\",\"ts_ms\":%llu,\"source\":\"balloon_photo_detector.c\","
            "\"clock_id\":\"%s\",\"mission_token\":\"%s\",\"process_id\":%lu,"
            "\"stage\":\"%s\",\"seq\":%u,\"duration_ms\":%llu,"
            "\"code\":\"0x%llX\"}\n",
            (unsigned long long) NowMs(),
            clockId, token, (unsigned long) getpid(),
            stage == NULL ? "" : stage,
            seq,
            (unsigned long long) durationMs,
            (unsigned long long) code);
    fclose(fp);
}

static void LiveviewH264Callback(E_DjiLiveViewCameraPosition position, const uint8_t *buf, uint32_t len)
{
    size_t offset = 0;

    (void) position;
    if (buf == NULL || len == 0 || !s_liveviewStreamStarted) {
        return;
    }

    if (s_liveviewFifoFd < 0) {
        s_liveviewFifoFd = open(BALLOON_LIVEVIEW_FIFO, O_WRONLY | O_NONBLOCK);
        if (s_liveviewFifoFd < 0) {
            s_liveviewBytesDropped += len;
            return;
        }
#ifdef F_SETPIPE_SZ
        (void) fcntl(s_liveviewFifoFd, F_SETPIPE_SZ, 1024 * 1024);
#endif
    }

    while (offset < len) {
        ssize_t written = write(s_liveviewFifoFd, buf + offset, len - offset);
        if (written > 0) {
            offset += (size_t) written;
            s_liveviewBytesReceived += (uint64_t) written;
            continue;
        }
        if (written < 0 && errno == EINTR) {
            continue;
        }
        s_liveviewBytesDropped += (uint64_t) (len - offset);
        if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            close(s_liveviewFifoFd);
            s_liveviewFifoFd = -1;
        }
        break;
    }
}

static T_DjiReturnCode StartLiveview(E_DjiMountPosition position)
{
    T_DjiReturnCode code;

    (void) signal(SIGPIPE, SIG_IGN);
    if (mkfifo(BALLOON_LIVEVIEW_FIFO, 0666) != 0 && errno != EEXIST) {
        USER_LOG_ERROR("Create liveview FIFO failed: %s errno=%d", BALLOON_LIVEVIEW_FIFO, errno);
        return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
    }

    s_liveviewBytesReceived = 0;
    s_liveviewBytesDropped = 0;
    AppendEvent("liveview_init_begin", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
    code = DjiLiveview_Init();
    AppendEvent("liveview_init", 0, code, BALLOON_LIVEVIEW_FIFO);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        return code;
    }
    s_liveviewInitialized = true;
    s_liveviewStreamStarted = true;

    AppendEvent("liveview_stream_start_begin", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
    code = DjiLiveview_StartH264Stream((E_DjiLiveViewCameraPosition) position,
                                      DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT,
                                      LiveviewH264Callback);
    AppendEvent("liveview_stream_start", 0, code, "payload_port_no1/default");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_liveviewStreamStarted = false;
        (void) DjiLiveview_Deinit();
        s_liveviewInitialized = false;
        return code;
    }

    (void) DjiLiveview_RequestIntraframeFrameData((E_DjiLiveViewCameraPosition) position,
                                                  DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode StopLiveview(E_DjiMountPosition position, const char *reason)
{
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    char detail[160];

    s_liveviewStreamStarted = false;
    if (s_liveviewInitialized) {
        AppendEvent("liveview_stream_stop_begin", s_captureSeq, code, reason == NULL ? "" : reason);
        code = DjiLiveview_StopH264Stream((E_DjiLiveViewCameraPosition) position,
                                         DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
        AppendEvent("liveview_stream_stop", s_captureSeq, code, reason == NULL ? "" : reason);
        AppendEvent("liveview_deinit_begin", s_captureSeq, code, "");
        code = DjiLiveview_Deinit();
        AppendEvent("liveview_deinit", s_captureSeq, code, reason == NULL ? "" : reason);
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return code;
        s_liveviewInitialized = false;
    }
    if (s_liveviewFifoFd >= 0) {
        close(s_liveviewFifoFd);
        s_liveviewFifoFd = -1;
    }
    snprintf(detail, sizeof(detail), "received=%llu dropped=%llu",
             (unsigned long long) s_liveviewBytesReceived,
             (unsigned long long) s_liveviewBytesDropped);
    AppendEvent("liveview_byte_stats", s_captureSeq, code, detail);
    return code;
}

static bool s_calibrationLiveviewOwned = false;
static void EnsureDirectories(void);

T_DjiReturnCode BalloonPhotoDetector_StartCalibrationLiveview(void)
{
    T_DjiReturnCode code;
    if (BalloonPhotoDetector_IsRunning()) return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    if (s_calibrationLiveviewOwned) return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    /* Never claim/release a stream owned by another task. */
    if (s_liveviewInitialized || s_liveviewStreamStarted)
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    EnsureDirectories();
    code = StartLiveview(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1);
    s_calibrationLiveviewOwned = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    AppendEvent("calibration_liveview_start", 0, code, "record_only_no_chase");
    return code;
}

T_DjiReturnCode BalloonPhotoDetector_StopCalibrationLiveview(void)
{
    if (s_calibrationLiveviewOwned) {
        T_DjiReturnCode code = StopLiveview(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, "calibration_capture_end");
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return code;
        s_calibrationLiveviewOwned = false;
    }
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static bool ReadLatestLiveviewFrame(T_BalloonLiveviewFrame *frame)
{
    FILE *fp;
    struct stat st;
    unsigned long long frameId = 0;
    unsigned long long readyTsMs = 0;
    unsigned long long encodeMs = 0;
    char clockId[80] = {0};
    int matched;

    if (frame == NULL) {
        return false;
    }
    memset(frame, 0, sizeof(*frame));
    fp = fopen(BALLOON_LIVEVIEW_METADATA, "r");
    if (fp == NULL) {
        return false;
    }
    matched = fscanf(fp, "%llu\t%llu\t%llu\t%511[^\t\n]\t%79s",
                     &frameId, &readyTsMs, &encodeMs, frame->path, clockId);
    fclose(fp);
    if (matched != 5 || !BalloonClock_Matches(clockId) || readyTsMs > NowMs() || frameId == 0 || readyTsMs == 0 || frame->path[0] == '\0') {
        return false;
    }
    if (stat(frame->path, &st) != 0 || st.st_size <= 0) {
        return false;
    }
    frame->frameId = (uint64_t) frameId;
    frame->readyTsMs = (uint64_t) readyTsMs;
    frame->encodeMs = (uint64_t) encodeMs;
    return true;
}

static void AppendControlTargetEvent(const char *event,
                                     const T_BalloonTargetRecord *target,
                                     T_DjiReturnCode code)
{
    FILE *fp = fopen(BALLOON_CONTROL_EVENT_LOG, "a");
    uint64_t nowMs = NowMs();
    uint64_t ageMs = 0;
    char clockId[80] = {0}, token[96];
    (void) BalloonClock_Id(clockId);
    BalloonEvidence_Token(token);

    if (fp == NULL) {
        return;
    }

    if (target != NULL && nowMs >= target->targetTsMs) {
        ageMs = nowMs - target->targetTsMs;
    }

    fprintf(fp,
            "{\"clock\":\"monotonic-v1\",\"ts_ms\":%llu,\"source\":\"balloon_photo_detector\","
            "\"clock_id\":\"%s\",\"mission_token\":\"%s\",\"event\":\"%s\",\"code\":\"0x%llX\",\"seq\":%u,"
            "\"target_timestamp_ms\":%llu,\"detection_finished_ts_ms\":%llu,"
            "\"target_age_ms\":%llu,\"confidence\":%.6f,"
            "\"bbox_xyxy\":[%.3f,%.3f,%.3f,%.3f],\"source_photo\":\"%s\"}\n",
            (unsigned long long) nowMs,
            clockId, token,
            event == NULL ? "" : event,
            (unsigned long long) code,
            target == NULL ? 0 : target->seq,
            target == NULL ? 0ULL : (unsigned long long) target->targetTsMs,
            target == NULL ? 0ULL : (unsigned long long) target->detectionFinishedTsMs,
            (unsigned long long) ageMs,
            target == NULL ? 0.0f : target->confidence,
            target == NULL ? 0.0f : target->x1,
            target == NULL ? 0.0f : target->y1,
            target == NULL ? 0.0f : target->x2,
            target == NULL ? 0.0f : target->y2,
            target == NULL ? "" : target->source);
    fclose(fp);
}

static void WriteTextFile(const char *path, const char *text)
{
    FILE *fp = fopen(path, "w");
    if (fp == NULL) {
        USER_LOG_ERROR("Balloon photo detector failed to write file: %s", path);
        return;
    }
    fprintf(fp, "%s\n", text == NULL ? "" : text);
    fclose(fp);
}

static bool NaturalLanguageStopIsPlanComplete(void)
{
    FILE *fp = fopen(BALLOON_NL_STOP_FILE, "r");
    char line[160];
    bool planComplete = false;

    if (fp == NULL) {
        return false;
    }
    while (fgets(line, sizeof(line), fp) != NULL) {
        if (strstr(line, "reason=capability_plan_complete") != NULL) {
            planComplete = true;
            break;
        }
    }
    fclose(fp);
    return planComplete;
}

static void SmoothPlanCompletionStop(void)
{
    const uint64_t maxDurationMs = 1400ULL;
    const uint64_t startMonoMs = MonoMs();
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_TargetChaseCommand command;

    AppendControlEvent("plan_complete_smooth_stop_started",
                       DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                       "decelerate before authority release");
    do {
        code = DjiTargetChase_Hover(NowMs());
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
            code != (T_DjiReturnCode) 0x1B00000000ULL) {
            break;
        }
        if (DjiTargetChase_GetLastCommand(&command) &&
            fabsf(command.x_mps) <= 0.01f &&
            fabsf(command.y_mps) <= 0.01f &&
            fabsf(command.z_mps) <= 0.01f &&
            fabsf(command.yaw_rate_degps) <= 0.25f) {
            break;
        }
        usleep(BALLOON_CONTROL_LOOP_INTERVAL_MS * 1000U);
    } while (MonoMs() - startMonoMs < maxDurationMs);

    AppendControlEvent("plan_complete_smooth_stop_finished",
                       code,
                       "zero command follows before authority release");
}

static void EnsureDirectories(void)
{
    int ret = system("mkdir -p /home/uav/offline_balloon_strike_test/captures "
                     "/home/uav/offline_balloon_strike_test/yolo_runs");
    if (ret != 0) {
        USER_LOG_ERROR("Balloon photo detector mkdir command failed: %d", ret);
    }
}

static uint64_t CreateTimeKey(T_DjiCameraManagerFileCreateTime createTime, uint32_t fileIndex)
{
    uint64_t key = 0;
    key = key * 10000ULL + createTime.year;
    key = key * 100ULL + createTime.month;
    key = key * 100ULL + createTime.day;
    key = key * 100ULL + createTime.hour;
    key = key * 100ULL + createTime.minute;
    key = key * 100ULL + createTime.second;
    key = key * 100000ULL + fileIndex;
    return key;
}

static bool SelectLatestJpeg(const T_DjiCameraManagerFileList *fileList,
                             T_DjiCameraManagerFileListInfo *selected,
                             uint64_t *selectedKey)
{
    bool found = false;
    uint64_t bestKey = 0;

    if (fileList == NULL || selected == NULL || fileList->fileListInfo == NULL) {
        return false;
    }

    for (uint16_t i = 0; i < fileList->totalCount; ++i) {
        const T_DjiCameraManagerFileListInfo *item = &fileList->fileListInfo[i];
        if (item->type != DJI_CAMERA_FILE_TYPE_JPEG) {
            continue;
        }

        uint64_t key = CreateTimeKey(item->createTime, item->fileIndex);
        if (!found || key > bestKey) {
            *selected = *item;
            bestKey = key;
            found = true;
        }
    }

    if (found && selectedKey != NULL) {
        *selectedKey = bestKey;
    }

    return found;
}

static void AppendMediaSnapshotEvent(const char *event,
                                     uint32_t seq,
                                     T_DjiReturnCode code,
                                     const T_BalloonJpegSnapshot *snapshot,
                                     const char *detail)
{
    FILE *fp = fopen(BALLOON_CAPTURE_EVENT_LOG, "a");
    uint64_t nowMs = NowMs();
    const T_DjiCameraManagerFileListInfo *info = snapshot != NULL && snapshot->valid ? &snapshot->info : NULL;

    if (fp == NULL) {
        USER_LOG_ERROR("Balloon photo detector failed to open event log: %s", BALLOON_CAPTURE_EVENT_LOG);
        return;
    }

    fprintf(fp,
            "{\"ts\":%lld,\"ts_ms\":%llu,\"event\":\"%s\",\"seq\":%u,"
            "\"code\":\"0x%08llX\",\"media_valid\":%s,"
            "\"file_index\":%u,\"file_name\":\"%s\",\"file_size\":%u,"
            "\"create_time_key\":%llu,\"create_time\":\"%04u-%02u-%02u_%02u:%02u:%02u\","
            "\"detail\":\"%s\"}\n",
            (long long) time(NULL),
            (unsigned long long) nowMs,
            event == NULL ? "" : event,
            seq,
            (unsigned long long) code,
            info == NULL ? "false" : "true",
            info == NULL ? 0U : info->fileIndex,
            info == NULL ? "" : info->fileName,
            info == NULL ? 0U : info->fileSize,
            snapshot == NULL ? 0ULL : (unsigned long long) snapshot->key,
            info == NULL ? 0U : info->createTime.year,
            info == NULL ? 0U : info->createTime.month,
            info == NULL ? 0U : info->createTime.day,
            info == NULL ? 0U : info->createTime.hour,
            info == NULL ? 0U : info->createTime.minute,
            info == NULL ? 0U : info->createTime.second,
            detail == NULL ? "" : detail);
    fclose(fp);
}

static T_DjiReturnCode DownloadFileDataCallback(T_DjiDownloadFilePacketInfo packetInfo,
                                                const uint8_t *data,
                                                uint16_t len)
{
    if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START ||
        packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START_TRANSFER_END) {
        if (s_downloadFile != NULL) {
            fclose(s_downloadFile);
            s_downloadFile = NULL;
        }
        remove(s_downloadTempPath);
        s_downloadFile = fopen(s_downloadTempPath, "wb+");
        if (s_downloadFile == NULL) {
            USER_LOG_ERROR("Open download temp file failed: %s", s_downloadTempPath);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    if (s_downloadFile != NULL && data != NULL && len > 0) {
        fwrite(data, 1, len, s_downloadFile);
    }

    if (packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_END ||
        packetInfo.downloadFileEvent == DJI_DOWNLOAD_FILE_EVENT_START_TRANSFER_END) {
        if (s_downloadFile != NULL) {
            fclose(s_downloadFile);
            s_downloadFile = NULL;
        }

        if (rename(s_downloadTempPath, s_downloadFinalPath) == 0) {
            s_downloadCompleted = true;
            USER_LOG_INFO("Balloon photo detector downloaded file: %s", s_downloadFinalPath);
        } else {
            USER_LOG_ERROR("Rename download file failed: %s -> %s, errno=%d",
                           s_downloadTempPath, s_downloadFinalPath, errno);
            remove(s_downloadTempPath);
            return DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR;
        }
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode EnsureCameraManagerReady(E_DjiMountPosition position)
{
    T_DjiReturnCode code;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();

    if (s_cameraManagerReady) {
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    code = DjiCameraManager_Init();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("DjiCameraManager_Init failed: 0x%08llX", (unsigned long long) code);
        return code;
    }
    s_cameraManagerReady = true;

    LogCameraReadback(position);

    if (osal != NULL) {
        osal->TaskSleepMs(300);
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *AutoExposureTask(void *arg)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    E_DjiMountPosition position = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
    const E_DjiMountPosition candidates[] = {
        DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
        DJI_MOUNT_POSITION_PAYLOAD_PORT_NO2,
        DJI_MOUNT_POSITION_PAYLOAD_PORT_NO3,
    };
    uint64_t refreshMs = ReadU64EnvBounded("BALLOON_AUTO_EXPOSURE_REFRESH_MS",
                                           BALLOON_DEFAULT_AUTO_EXPOSURE_REFRESH_MS,
                                           500ULL,
                                           10000ULL,
                                           false);
    uint32_t refreshCount = 0;
    T_DjiReturnCode code;
    bool supportedCameraFound = false;

    (void) arg;
    AppendCameraDiag("auto_exposure_task_started", position,
                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                     (long long) refreshMs, "program_auto");

    code = EnsureCameraManagerReady(position);
    AppendCameraDiag("auto_exposure_camera_manager_init", position, code, 0, "");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        goto out;
    }

    for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); ++i) {
        bool connected = false;
        E_DjiCameraType cameraType = 0;
        E_DjiCameraManagerExposureMode mode =
            DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_UNKNOWN;
        T_DjiReturnCode connectCode;
        T_DjiReturnCode typeCode;
        T_DjiReturnCode modeCode;

        connectCode = DjiCameraManager_GetCameraConnectStatus(candidates[i], &connected);
        AppendCameraDiag("auto_exposure_probe_connect", candidates[i], connectCode,
                         connected ? 1 : 0, "");
        typeCode = DjiCameraManager_GetCameraType(candidates[i], &cameraType);
        AppendCameraDiag("auto_exposure_probe_type", candidates[i], typeCode,
                         (long long) cameraType, "");
        modeCode = DjiCameraManager_GetExposureMode(candidates[i], &mode);
        AppendCameraDiag("auto_exposure_probe_mode", candidates[i], modeCode,
                         (long long) mode, "");

        if (connectCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && connected &&
            modeCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            position = candidates[i];
            supportedCameraFound = true;
            break;
        }
    }

    if (!supportedCameraFound) {
        AppendCameraDiag("auto_exposure_no_supported_camera", position,
                         DJI_ERROR_CAMERA_MANAGER_MODULE_CODE_UNSUPPORTED_COMMAND,
                         0, "all_mount_positions_probed");
        USER_LOG_WARN("No connected camera mount exposes PSDK exposure control");
        goto out;
    }

    code = DjiCameraManager_SetAELockEnabled(position, false);
    AppendCameraDiag("auto_exposure_unlock_ae", position, code, 0,
                     IsIgnorableCameraSetupCode(code) ? "" : "setup_error");

    code = DjiCameraManager_SetExposureMode(
        position, DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO);
    AppendCameraDiag("auto_exposure_set_program_auto", position, code,
                     DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO,
                     IsIgnorableCameraSetupCode(code) ? "" : "setup_error");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_WARN("Camera program auto exposure is unavailable: 0x%08llX",
                      (unsigned long long) code);
        goto out;
    }

    USER_LOG_INFO("Camera program auto exposure enabled; watchdog interval=%llu ms",
                  (unsigned long long) refreshMs);
    LogCameraReadback(position);

    while (!s_autoExposureStopRequested) {
        uint64_t waitedMs = 0;

        while (!s_autoExposureStopRequested && waitedMs < refreshMs) {
            uint64_t sleepMs = refreshMs - waitedMs;
            if (sleepMs > 100ULL) {
                sleepMs = 100ULL;
            }
            if (osal != NULL) {
                osal->TaskSleepMs((uint32_t) sleepMs);
            } else {
                usleep((useconds_t) sleepMs * 1000U);
            }
            waitedMs += sleepMs;
        }
        if (s_autoExposureStopRequested) {
            break;
        }

        {
            E_DjiCameraManagerExposureMode mode =
                DJI_CAMERA_MANAGER_EXPOSURE_MODE_EXPOSURE_UNKNOWN;
            code = DjiCameraManager_GetExposureMode(position, &mode);
            AppendCameraDiag("auto_exposure_watchdog", position, code,
                             (long long) mode, "");
            if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                mode != DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO) {
                code = DjiCameraManager_SetExposureMode(
                    position, DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO);
                AppendCameraDiag("auto_exposure_restore_program_auto", position, code,
                                 DJI_CAMERA_MANAGER_EXPOSURE_MODE_PROGRAM_AUTO, "");
            }
        }

        refreshCount++;
        if (refreshCount % 10U == 0U) {
            LogCameraReadback(position);
        }
    }

out:
    CleanupCameraManager(position, "auto_exposure_task_exit");
    AppendCameraDiagEvent("auto_exposure_task_stopped");
    s_autoExposureTask = NULL;
    return NULL;
}

T_DjiReturnCode BalloonPhotoDetector_StartAutoExposure(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode code;

    if (!ReadBoolEnv("BALLOON_AUTO_EXPOSURE", false)) {
        AppendCameraDiagEvent("auto_exposure_disabled");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (!UseLiveviewInput()) {
        AppendCameraDiag("auto_exposure_task_create", DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                         DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT, 0,
                         "requires_liveview_input");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT;
    }
    if (osal == NULL || s_autoExposureTask != NULL) {
        AppendCameraDiag("auto_exposure_task_create", DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                         osal == NULL ? DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR :
                         DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, 0,
                         osal == NULL ? "osal_unavailable" : "already_running");
        return osal == NULL ? DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR :
               DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    s_autoExposureStopRequested = false;
    code = osal->TaskCreate("balloon_auto_exposure", AutoExposureTask,
                            32 * 1024, NULL, &s_autoExposureTask);
    AppendCameraDiag("auto_exposure_task_create", DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1,
                     code, 0, "");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_autoExposureTask = NULL;
    }
    return code;
}

static T_DjiReturnCode CaptureSinglePhoto(E_DjiMountPosition position)
{
    T_DjiReturnCode code;
    T_DjiReturnCode shootModeCode;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    E_DjiCameraManagerWorkMode workMode = DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO;

    AppendCameraDiagEvent("legacy_single_photo_begin");

    code = DjiCameraManager_SetMode(position, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO);
    AppendCameraDiag("legacy_set_work_mode", position, code, DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO, "");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set camera shoot-photo work mode failed: 0x%08llX", (unsigned long long) code);
        return code;
    }

    if (osal != NULL) {
        osal->TaskSleepMs(1000);
    }

    code = DjiCameraManager_GetMode(position, &workMode);
    AppendCameraDiag("legacy_get_work_mode", position, code, (long long) workMode,
                     code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "" :
                     IsIgnorableCameraSetupCode(code) ? "ignored_if_unsupported" : "");
    if (!IsIgnorableCameraSetupCode(code)) {
        USER_LOG_ERROR("Get camera work mode failed hard: 0x%08llX", (unsigned long long) code);
        return code;
    }
    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && workMode != DJI_CAMERA_MANAGER_WORK_MODE_SHOOT_PHOTO) {
        AppendCameraDiag("legacy_work_mode_not_photo", position,
                         DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE,
                         (long long) workMode,
                         "abort_non_photo_mode");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }

    shootModeCode = DjiCameraManager_SetShootPhotoMode(position, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    AppendCameraDiag("legacy_set_shoot_photo_mode", position, shootModeCode,
                     DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE,
                     shootModeCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "" :
                     IsIgnorableCameraSetupCode(shootModeCode) ? "ignored_for_l1_if_unsupported" : "");
    if (!IsIgnorableCameraSetupCode(shootModeCode)) {
        USER_LOG_ERROR("Set single-photo mode failed: 0x%08llX", (unsigned long long) shootModeCode);
        return shootModeCode;
    }

    if (osal != NULL) {
        osal->TaskSleepMs(500);
    }

    code = DjiCameraManager_StartShootPhoto(position, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE);
    AppendCameraDiag("legacy_start_shoot_photo", position, code, DJI_CAMERA_MANAGER_SHOOT_PHOTO_MODE_SINGLE, "");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Balloon photo detector shoot photo failed: 0x%08llX", (unsigned long long) code);
    }

    AppendCameraDiagEvent("legacy_single_photo_end");
    return code;
}

static T_DjiReturnCode DownloadNewPhoto(E_DjiMountPosition position,
                                        uint32_t seq,
                                        uint64_t minNewerThanKey,
                                        char *outPath,
                                        size_t outPathSize)
{
    T_DjiReturnCode code;
    T_DjiCameraManagerFileList fileList = {0};
    T_BalloonJpegSnapshot selected;
    time_t now = time(NULL);
    struct tm localTime;
    char ts[32] = {0};
    uint64_t deadlineMs = MonoMs() + BALLOON_NEW_MEDIA_TIMEOUT_MS;
    uint64_t functionStartMonoMs = MonoMs();
    uint64_t stepStartMonoMs;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    const bool hasMinKey = minNewerThanKey != 0;

    stepStartMonoMs = MonoMs();
    code = DjiCameraManager_RegDownloadFileDataCallback(position, DownloadFileDataCallback);
    AppendLatencyEvent("media_register_download_callback",
                       seq,
                       MonoMs() - stepStartMonoMs,
                       code);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Register download callback failed: 0x%08llX", (unsigned long long) code);
        return code;
    }

    memset(&selected, 0, sizeof(selected));

    while (!s_stopRequested) {
        if (NaturalLanguageStopRequested()) {
            break;
        }
        stepStartMonoMs = MonoMs();
        code = ObtainDownloaderRightsTracked(position);
        AppendLatencyEvent("media_obtain_downloader_rights",
                           seq,
                           MonoMs() - stepStartMonoMs,
                           code);
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Obtain downloader rights failed: 0x%08llX", (unsigned long long) code);
        } else {
            memset(&fileList, 0, sizeof(fileList));
            stepStartMonoMs = MonoMs();
            code = DownloadMediaFileList(position, &fileList);
            AppendLatencyEvent("media_download_file_list",
                               seq,
                               MonoMs() - stepStartMonoMs,
                               code);
            if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                stepStartMonoMs = MonoMs();
                selected.valid = SelectLatestJpeg(&fileList, &selected.info, &selected.key);
                AppendLatencyEvent("media_select_latest_jpeg",
                                   seq,
                                   MonoMs() - stepStartMonoMs,
                                   selected.valid ? DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
                                                  : DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND);
                AppendMediaSnapshotEvent("media_latest_after_capture", seq, code, &selected,
                                         hasMinKey && selected.valid && selected.key <= minNewerThanKey
                                         ? "not_newer_than_last_download" : "");

                if (selected.valid && (!hasMinKey || selected.key > minNewerThanKey)) {
                    AppendLatencyEvent("media_wait_until_new_file",
                                       seq,
                                       MonoMs() - functionStartMonoMs,
                                       DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                    break;
                }
            } else {
                USER_LOG_ERROR("Download file list failed: 0x%08llX", (unsigned long long) code);
            }
        }

        stepStartMonoMs = MonoMs();
        ReleaseDownloaderRightsTracked(position, "media_poll_retry");
        AppendLatencyEvent("media_release_rights_retry",
                           seq,
                           MonoMs() - stepStartMonoMs,
                           DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);

        if (MonoMs() >= deadlineMs) {
            AppendMediaSnapshotEvent("capture_no_new_media",
                                     seq,
                                     DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT,
                                     &selected,
                                     hasMinKey ? "latest_jpeg_not_newer_than_last_download"
                                               : "no_jpeg_after_capture");
            return DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
        }

        if (osal != NULL) {
            stepStartMonoMs = MonoMs();
            osal->TaskSleepMs(BALLOON_NEW_MEDIA_POLL_MS);
            AppendLatencyEvent("media_poll_sleep",
                               seq,
                               MonoMs() - stepStartMonoMs,
                               DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        }
    }

    if (!selected.valid) {
        ReleaseDownloaderRightsTracked(position, "no_media_selected");
        AppendMediaSnapshotEvent("capture_no_new_media",
                                 seq,
                                 DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND,
                                 &selected,
                                 "no_jpeg_after_capture");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NOT_FOUND;
    }

    localtime_r(&now, &localTime);
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &localTime);
    snprintf(s_downloadFinalPath, sizeof(s_downloadFinalPath),
             BALLOON_CAPTURE_DIR "/balloon_%s_%03u.jpg", ts, seq);
    snprintf(s_downloadTempPath, sizeof(s_downloadTempPath), "%s.part", s_downloadFinalPath);
    s_downloadCompleted = false;

    AppendMediaSnapshotEvent("media_download_selected", seq, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, &selected, "");
    USER_LOG_INFO("Download new JPEG file index=%u name=%s key=%llu lastKey=%llu lastIndex=%u to %s",
                  selected.info.fileIndex,
                  selected.info.fileName,
                  (unsigned long long) selected.key,
                  (unsigned long long) minNewerThanKey,
                  s_lastDownloadedFileIndex,
                  s_downloadFinalPath);
    stepStartMonoMs = MonoMs();
    code = DjiCameraManager_DownloadFileByIndex(position, selected.info.fileIndex);
    AppendLatencyEvent("media_download_file_by_index",
                       seq,
                       MonoMs() - stepStartMonoMs,
                       code);
    stepStartMonoMs = MonoMs();
    ReleaseDownloaderRightsTracked(position, "download_file_by_index_done");
    AppendLatencyEvent("media_release_rights_after_download",
                       seq,
                       MonoMs() - stepStartMonoMs,
                       DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Download new JPEG failed: 0x%08llX", (unsigned long long) code);
        remove(s_downloadTempPath);
        return code;
    }

    if (!s_downloadCompleted) {
        USER_LOG_ERROR("Download new JPEG finished without completion callback");
        remove(s_downloadTempPath);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    snprintf(outPath, outPathSize, "%s", s_downloadFinalPath);
    stepStartMonoMs = MonoMs();
    WriteTextFile(BALLOON_LATEST_PHOTO_FILE, outPath);
    AppendLatencyEvent("latest_photo_marker_write",
                       seq,
                       MonoMs() - stepStartMonoMs,
                       DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
    s_lastDownloadedKey = selected.key;
    s_lastDownloadedFileIndex = selected.info.fileIndex;
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode InitializeMediaBaseline(E_DjiMountPosition position)
{
    T_DjiReturnCode code;
    T_DjiCameraManagerFileList fileList = {0};
    T_BalloonJpegSnapshot baseline;

    memset(&baseline, 0, sizeof(baseline));
    s_lastDownloadedKey = 0;
    s_lastDownloadedFileIndex = 0;

    code = ObtainDownloaderRightsTracked(position);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        AppendMediaSnapshotEvent("media_baseline_failed",
                                 0,
                                 code,
                                 &baseline,
                                 "obtain_downloader_rights_failed");
        return code;
    }

    code = DownloadMediaFileList(position, &fileList);
    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        baseline.valid = SelectLatestJpeg(&fileList, &baseline.info, &baseline.key);
        if (baseline.valid) {
            s_lastDownloadedKey = baseline.key;
            s_lastDownloadedFileIndex = baseline.info.fileIndex;
        }
        AppendMediaSnapshotEvent("media_baseline_before_loop",
                                 0,
                                 code,
                                 &baseline,
                                 baseline.valid ? "startup_baseline" : "no_jpeg_baseline");
    } else {
        AppendMediaSnapshotEvent("media_baseline_failed",
                                 0,
                                 code,
                                 &baseline,
                                 "download_file_list_failed");
    }

    ReleaseDownloaderRightsTracked(position, "media_baseline_done");
    return code;
}

static T_DjiReturnCode LaunchDetection(const char *photoPath, uint32_t seq, uint64_t captureTsMs)
{
    FILE *fp;
    uint64_t requestTsMs = NowMs();
    char clockId[80];
    if (!BalloonClock_Id(clockId)) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    if (photoPath == NULL || photoPath[0] == '\0') {
        return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    }

    fp = fopen(BALLOON_DETECTION_REQUEST_TMP_FILE, "w");
    if (fp == NULL) {
        USER_LOG_ERROR("Open detection request tmp file failed: %s", BALLOON_DETECTION_REQUEST_TMP_FILE);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    fprintf(fp, "%u\t%llu\t%llu\t%s\t%s\n",
            seq,
            (unsigned long long) captureTsMs,
            (unsigned long long) requestTsMs,
            photoPath, clockId);

    if (fclose(fp) != 0) {
        remove(BALLOON_DETECTION_REQUEST_TMP_FILE);
        USER_LOG_ERROR("Close detection request tmp file failed: %s", BALLOON_DETECTION_REQUEST_TMP_FILE);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    if (rename(BALLOON_DETECTION_REQUEST_TMP_FILE, BALLOON_DETECTION_REQUEST_FILE) != 0) {
        remove(BALLOON_DETECTION_REQUEST_TMP_FILE);
        USER_LOG_ERROR("Commit detection request file failed: %s", BALLOON_DETECTION_REQUEST_FILE);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

typedef struct {
    T_BalloonVisualContext context;
    uint64_t lastTargetMonoMs;
    bool hasContext;
    bool contextValid;
} T_BalloonVisualWatch;

static bool BalloonControl_VisualCancelled(const char *missionToken)
{
    return s_controlStopRequested || NaturalLanguageStopRequested() ||
        !BalloonMission_Matches(BALLOON_MISSION_TOKEN, missionToken) ||
        !BalloonMission_Matches(BALLOON_MISSION_COMMIT, missionToken) ||
        !BalloonMission_Healthy(missionToken);
}

/* The camera/YOLO workers are already running and do not wait for SDK Init.
 * This entire preflight path performs no flight-controller or gimbal calls. */
static bool BalloonControl_WaitForVisual(const char *missionToken, T_BalloonVisualWatch *watch)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    uint64_t began = MonoMs();
    bool sawFrame = false;
    char clockId[80];
    memset(watch, 0, sizeof(*watch));
    while (!BalloonControl_VisualCancelled(missionToken)) {
        uint64_t now = NowMs();
        T_BalloonVisualContext context;
        T_BalloonVisualReceipt receipt;
        if (BalloonClock_Id(clockId) && BalloonVisual_ReadContext(missionToken, clockId, now, &context)) {
            if (!watch->hasContext || !BalloonVisual_SameContext(&context, &watch->context)) sawFrame = false;
            watch->context = context;
            watch->hasContext = true;
            watch->contextValid = true;
            if (BalloonVisual_ReadReceipt(missionToken, clockId, now, &context, &receipt)) {
                sawFrame = true;
                if (receipt.targetFound && !BalloonControl_VisualCancelled(missionToken)) {
                    uint64_t mono = MonoMs(), age = now - receipt.captureMs;
                    watch->lastTargetMonoMs = mono >= age ? mono - age : 0;
                    AppendControlEvent("visual_preflight_target_ready", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, context.color);
                    return true;
                }
            }
        }
        if (MonoMs() - began >= BALLOON_VISUAL_PREFLIGHT_MS) {
            if (BalloonControl_VisualCancelled(missionToken)) return false;
            BalloonVisual_RecordFailure(sawFrame ? 1 : 2, watch->hasContext ? watch->context.color : NULL);
            AppendControlEvent(sawFrame ? "visual_preflight_target_missing" : "visual_preflight_image_missing",
                               DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN,
                               watch->hasContext ? watch->context.color : "unknown");
            return false;
        }
        if (osal) osal->TaskSleepMs(BALLOON_CONTROL_LOOP_INTERVAL_MS);
        else return false;
    }
    return false;
}

/* 1: fresh target, 0: bounded target gap, -1: target lost. No SDK calls.
 * A new action invalidates the previous box before any flight command. */
static int BalloonControl_ObserveVisual(const char *missionToken, T_BalloonVisualWatch *watch,
                                       T_BalloonTargetRecord *target, bool *phaseChanged)
{
    uint64_t now = NowMs(), mono = MonoMs();
    char clockId[80];
    T_BalloonVisualContext context;
    T_BalloonVisualReceipt receipt;
    *phaseChanged = false;
    bool contextValid = BalloonClock_Id(clockId) &&
        BalloonVisual_ReadContext(missionToken, clockId, now, &context);
    if (!contextValid && watch->contextValid) {
        /* Keep the last known action and its deadline through a transient
         * ACK outage. The same action cannot renew a lost-target watchdog. */
        watch->contextValid = false;
        *phaseChanged = true;
    }
    if (contextValid && (!watch->hasContext || !BalloonVisual_SameContext(&watch->context, &context))) {
        watch->context = context;
        watch->hasContext = true;
        watch->lastTargetMonoMs = mono;
        *phaseChanged = true;
    }
    if (contextValid) watch->contextValid = true;
    if (contextValid && BalloonVisual_ReadReceipt(missionToken, clockId, now, &context, &receipt) && receipt.targetFound) {
        uint64_t age = now - receipt.captureMs;
        watch->lastTargetMonoMs = mono >= age ? mono - age : 0;
        memset(target, 0, sizeof(*target));
        target->seq = receipt.seq;
        target->width = receipt.width; target->height = receipt.height;
        target->x1 = receipt.x1; target->y1 = receipt.y1;
        target->x2 = receipt.x2; target->y2 = receipt.y2;
        target->confidence = receipt.confidence;
        target->targetTsMs = receipt.captureMs; target->detectionFinishedTsMs = receipt.finishedMs;
        snprintf(target->source, sizeof(target->source), "mission_bound_%s", receipt.source);
        return 1;
    }
    if (BalloonVisual_TimedOut(mono, watch->lastTargetMonoMs)) {
        /* A normal completion, uncertain hit, or operator cancellation has
         * priority over the watchdog's error text. */
        if (!BalloonControl_VisualCancelled(missionToken))
            BalloonVisual_RecordFailure(3, watch->hasContext ? watch->context.color : NULL);
        return -1;
    }
    return 0;
}

/* Managed RequestSearch clears the controller target, prediction filters,
 * and moving gimbal state without scanning. Call it only on entry into a
 * visual gap/action transition: repeated resets would reject delayed frames.
 * Hover must run every wait tick because deceleration is rate limited. */
static T_DjiReturnCode BalloonControl_HoldForVisual(uint64_t nowMs, bool clearTarget)
{
    if (clearTarget) {
        T_DjiReturnCode code = DjiTargetChase_RequestSearch(nowMs, false);
        AppendControlEvent("visual_target_state_cleared", code, "managed_current_view_only");
        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) return code;
    }
    return DjiTargetChase_Hover(nowMs);
}

static void *BalloonControlTask(void *arg)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    T_TargetChaseConfig config = TargetChase_DefaultConfig(5472.0f, 3648.0f, 73.7f, 45.7f);
    T_DjiReturnCode code = DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    T_BalloonVisualWatch visualWatch;
    uint32_t lastObservedSeq = 0;
    uint64_t lastObservedTargetTsMs = 0;
    uint64_t lastAcceptedTargetTsMs = 0;
    uint64_t lastAcceptedYoloControlMs = 0;
    uint64_t firstTargetLoadedMonoMs = 0;
    uint64_t lastRunOnceLogMs = 0;
    uint64_t activeCommandExpireMonoMs = 0;
    uint64_t hoverStartedMonoMs = 0;
    uint64_t targetTimeoutMs;
    uint64_t farTargetTimeoutMs;
    uint64_t milMaxAgeMs;
    uint64_t milSuppressAfterYoloMs;
    uint64_t realControlMaxAfterTargetMs;
    T_BalloonTargetRecord activeTarget;
    T_BalloonTargetFilter targetFilter;
    bool hasActiveTarget = false;
    bool currentTargetTimedOut = false;
    bool burstHoverLocked = false;
    bool hoverKeepaliveActive = false;
    bool noFlightMode = false;
    bool visualHold = false;
    uint64_t lastOperatorStatusMonoMs = 0;
    uint64_t taskStartMs = NowMs();

    (void) arg;
    targetTimeoutMs = ReadU64EnvBounded("BALLOON_TARGET_TIMEOUT_MS",
                                        BALLOON_DEFAULT_TARGET_TIMEOUT_MS,
                                        200ULL,
                                        10000ULL,
                                        false);
    farTargetTimeoutMs = ReadU64EnvBounded(
        "BALLOON_FAR_TARGET_TIMEOUT_MS",
        BALLOON_DEFAULT_FAR_TARGET_TIMEOUT_MS,
        targetTimeoutMs,
        10000ULL,
        false);
    milMaxAgeMs = ReadU64EnvBounded("BALLOON_MIL_MAX_AGE_MS",
                                    BALLOON_DEFAULT_MIL_MAX_AGE_MS,
                                    50ULL,
                                    2000ULL,
                                    false);
    milSuppressAfterYoloMs =
        ReadU64EnvBounded("BALLOON_MIL_SUPPRESS_AFTER_YOLO_MS",
                          BALLOON_DEFAULT_MIL_SUPPRESS_AFTER_YOLO_MS,
                          0ULL,
                          5000ULL,
                          true);
    realControlMaxAfterTargetMs = ReadU64EnvBounded("BALLOON_REAL_CONTROL_MAX_AFTER_TARGET_MS",
                                                     BALLOON_DEFAULT_REAL_CONTROL_MAX_AFTER_TARGET_MS,
                                                     10000ULL,
                                                     86400000ULL,
                                                     true);
    /*
     * The controller may predict through the longer far-target interval.
     * The outer control loop still applies the shorter normal timeout as soon
     * as the target box grows beyond the far-area threshold.
     */
    config.target_timeout_ms = (float) farTargetTimeoutMs;
    config.approach_speed_mps = ReadFloatEnvBounded("BALLOON_APPROACH_SPEED_MPS",
                                                    config.approach_speed_mps,
                                                    0.0f,
                                                    config.max_horizontal_speed_mps);
    config.min_approach_speed_mps = ReadFloatEnvBounded("BALLOON_MIN_APPROACH_SPEED_MPS",
                                                         config.min_approach_speed_mps,
                                                         0.0f,
                                                         config.approach_speed_mps);
    config.box_angle_small_deg = ReadFloatEnvBounded("BALLOON_BOX_ANGLE_SMALL_DEG",
                                                      config.box_angle_small_deg,
                                                      1.0f,
                                                      10.0f);
    config.box_angle_large_deg = ReadFloatEnvBounded("BALLOON_BOX_ANGLE_LARGE_DEG",
                                                      config.box_angle_large_deg,
                                                      config.box_angle_small_deg + 1.0f,
                                                      45.0f);
    config.gimbal_mount_yaw_offset_deg =
        ReadFloatEnvBounded("BALLOON_GIMBAL_MOUNT_YAW_OFFSET_DEG",
                            config.gimbal_mount_yaw_offset_deg,
                            -180.0f,
                            180.0f);
    config.gimbal_alignment_deadband_deg =
        ReadFloatEnvBounded("BALLOON_GIMBAL_ALIGNMENT_DEADBAND_DEG",
                            config.gimbal_alignment_deadband_deg,
                            0.5f,
                            10.0f);
    config.gimbal_recenter_enter_deg =
        ReadFloatEnvBounded("BALLOON_GIMBAL_RECENTER_ENTER_DEG",
                            config.gimbal_recenter_enter_deg,
                            5.0f,
                            35.0f);
    config.gimbal_recenter_exit_deg =
        ReadFloatEnvBounded("BALLOON_GIMBAL_RECENTER_EXIT_DEG",
                            config.gimbal_recenter_exit_deg,
                            1.0f,
                            config.gimbal_recenter_enter_deg - 1.0f);
    config.prediction_horizon_s =
        ReadFloatEnvBounded("BALLOON_PREDICTION_HORIZON_S",
                            config.prediction_horizon_s,
                            0.0f,
                            0.5f);
    config.prediction_adaptive = ReadBoolEnv("BALLOON_PREDICTION_ADAPTIVE", true);
    /* Recorded replay does not establish a robust benefit from size-only
     * blending. Default to the archived pixel baseline for paired trials. */
    config.prediction_mode = (int) ReadU64EnvBounded("BALLOON_PREDICTION_MODE", 1, 0, 3, true);
    if (config.prediction_mode < 0 || config.prediction_mode > 3) config.prediction_mode = 1;
    config.prediction_residual_min_deg = ReadFloatEnvBounded(
        "BALLOON_PREDICTION_RESIDUAL_MIN_DEG", config.prediction_residual_min_deg, 0.2f, 30.0f);
    config.prediction_residual_box_scale = ReadFloatEnvBounded(
        "BALLOON_PREDICTION_RESIDUAL_BOX_SCALE", config.prediction_residual_box_scale, 0.1f, 5.0f);
    config.mpc_horizon_s =
        ReadFloatEnvBounded("BALLOON_MPC_HORIZON_S",
                            config.mpc_horizon_s,
                            0.4f,
                            3.0f);
    config.mpc_max_accel_mps2 =
        ReadFloatEnvBounded("BALLOON_MPC_MAX_ACCEL_MPS2",
                            config.mpc_max_accel_mps2,
                            0.1f,
                            2.0f);
    config.max_vertical_speed_mps = ReadFloatEnvBounded("BALLOON_MAX_VERTICAL_SPEED_MPS",
                                                        config.max_vertical_speed_mps,
                                                        0.1f,
                                                        2.0f);
    config.max_yaw_rate_degps = ReadFloatEnvBounded("BALLOON_MAX_YAW_RATE_DEGPS",
                                                     config.max_yaw_rate_degps,
                                                     1.0f,
                                                     30.0f);
    config.max_vertical_excursion_m = ReadFloatEnvBounded("BALLOON_MAX_VERTICAL_EXCURSION_M",
                                                          config.max_vertical_excursion_m,
                                                          0.5f,
                                                          20.0f);
    config.vertical_bearing_deadband_deg = ReadFloatEnvBounded("BALLOON_VERTICAL_DEADBAND_DEG",
                                                               config.vertical_bearing_deadband_deg,
                                                               0.5f,
                                                               15.0f);
    config.kp_vertical_bearing = ReadFloatEnvBounded("BALLOON_VERTICAL_KP",
                                                     config.kp_vertical_bearing,
                                                     0.005f,
                                                     0.20f);
    config.kd_vertical_velocity = ReadFloatEnvBounded("BALLOON_VERTICAL_KD",
                                                      config.kd_vertical_velocity,
                                                      0.0f,
                                                      1.0f);

    remove(BALLOON_LATEST_TARGET_FILE);
    {
        char detail[256];
        snprintf(detail, sizeof(detail),
                 "joystick path ready; approach=%.2f vertical_max=%.2f yaw_max=%.2f vertical_excursion=%.2f vertical_deadband=%.2f vertical_kp=%.3f vertical_kd=%.3f target_timeout_ms=%llu far_target_timeout_ms=%llu mil_max_age_ms=%llu max_after_target_ms=%llu",
                 config.approach_speed_mps,
                 config.max_vertical_speed_mps,
                 config.max_yaw_rate_degps,
                 config.max_vertical_excursion_m,
                 config.vertical_bearing_deadband_deg,
                 config.kp_vertical_bearing,
                 config.kd_vertical_velocity,
                 (unsigned long long) targetTimeoutMs,
                 (unsigned long long) farTargetTimeoutMs,
                 (unsigned long long) milMaxAgeMs,
                 (unsigned long long) realControlMaxAfterTargetMs);
        AppendControlEvent("control_task_starting", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, detail);
    }
    BalloonStep_Show("control init", "initializing target chase");

    char missionToken[96];
    uint64_t gateWaitStart = MonoMs();
    if (!BalloonMission_ReadToken(BALLOON_MISSION_TOKEN, missionToken)) goto gate_cancelled;
    while (!s_controlStopRequested) {
        if (NaturalLanguageStopRequested() || !BalloonMission_Matches(BALLOON_MISSION_TOKEN, missionToken)) goto gate_cancelled;
        if (BalloonMission_Matches(BALLOON_MISSION_COMMIT, missionToken) && BalloonMission_Healthy(missionToken)) break;
        if (MonoMs() - gateWaitStart >= 15000) goto gate_cancelled;
        if (osal) osal->TaskSleepMs(50);
    }
    if (s_controlStopRequested) goto gate_cancelled;
    AppendControlEvent("mission_gate_open", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, missionToken);
    if (!BalloonControl_WaitForVisual(missionToken, &visualWatch)) goto gate_cancelled;
    if (BalloonControl_VisualCancelled(missionToken)) goto gate_cancelled;
    code = DjiTargetChase_Init(&config);
    BalloonControl_RecordInitResult(code, s_controlStopRequested);
    AppendControlEvent("target_chase_init", code, "");
    if (s_controlStopRequested) {
        /* Stop may arrive while initialization waits for authority telemetry.
         * Keep cancellation distinct from an initialization failure and leave
         * cleanup on this owning control thread, before any ready/trigger. */
        BalloonControl_RecordInitResult(DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, true);
        code = DjiTargetChase_DeInit();
        AppendControlEvent("control_init_cancelled", code, "stop_during_initialization");
        goto gate_cancelled;
    }
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        BalloonStep_ShowSeq("control init failed", 0, code, "target chase init failed");
        goto gate_cancelled;
    }
    noFlightMode = DjiTargetChase_IsNoFlightMode();
    BalloonStep_Show("control ready", noFlightMode ? "NO_FLIGHT dry-run; waiting YOLO target"
                                                   : "FLIGHT mode; waiting YOLO target");

    code = DjiTargetChase_SetManualTrigger(true, NowMs());
    AppendControlEvent("manual_trigger_enabled", code, "");
    BalloonStep_ShowSeq("manual trigger", 0, code, "enabled for target chase");
    memset(&activeTarget, 0, sizeof(activeTarget));
    memset(&targetFilter, 0, sizeof(targetFilter));

    while (!s_controlStopRequested) {
        uint64_t nowMs = NowMs();
        uint64_t nowMonoMs = MonoMs();
        T_BalloonTargetRecord target;
        bool targetUpdatedThisLoop = false;

        if (DjiTargetChase_IsControlInhibited()) {
            (void)BalloonPhotoDetector_StopWithReason("operator_control_takeover");
            AppendControlEvent("operator_control_takeover", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "restart_requires_new_task");
            break;
        }
        if (NaturalLanguageStopRequested()) {
            if (NaturalLanguageStopIsPlanComplete()) {
                SmoothPlanCompletionStop();
            }
            break;
        }

        if (!BalloonMission_Matches(BALLOON_MISSION_COMMIT, missionToken) || !BalloonMission_Healthy(missionToken)) {
            AppendControlEvent("mission_gate_lost", DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN, missionToken);
            s_stopRequested = true;
            break;
        }
        /* Managed voice tasks inspect the current view. Search request files
         * are never permission to pan the gimbal or rotate the aircraft. */
        (void) unlink(BALLOON_TARGET_SEARCH_REQUEST_FILE);
        bool phaseChanged = false;
        int visualResult = BalloonControl_ObserveVisual(missionToken, &visualWatch, &target, &phaseChanged);
        if (visualResult < 0) {
            AppendControlEvent("visual_target_lost_stop", DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN,
                               visualWatch.hasContext ? visualWatch.context.color : "unknown");
            break;
        }
        bool resetVisualTarget = phaseChanged || (visualResult == 0 && !visualHold);
        if (resetVisualTarget) {
            visualHold = true;
            memset(&activeTarget, 0, sizeof(activeTarget));
            memset(&targetFilter, 0, sizeof(targetFilter));
            hasActiveTarget = false;
            lastAcceptedTargetTsMs = 0;
            lastObservedSeq = 0;
            lastObservedTargetTsMs = 0;
            lastAcceptedYoloControlMs = 0;
            activeCommandExpireMonoMs = 0;
            currentTargetTimedOut = true;
            burstHoverLocked = false;
            hoverKeepaliveActive = false;
            hoverStartedMonoMs = 0;
            firstTargetLoadedMonoMs = 0;
        }
        if (BalloonControl_VisualCancelled(missionToken)) break;
        if (visualHold) {
            code = BalloonControl_HoldForVisual(nowMs, resetVisualTarget);
            if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && code != (T_DjiReturnCode) 0x1B00000000ULL) {
                AppendControlEvent("visual_wait_hover_failed", code, visualWatch.context.color);
                break;
            }
        }
        if (visualResult > 0 && !visualHold) DjiTargetChase_GimbalTick(nowMs);

        if (visualResult > 0) {
            if (target.seq != lastObservedSeq ||
                target.targetTsMs != lastObservedTargetTsMs) {
                uint64_t targetAgeMs = nowMs >= target.targetTsMs ? nowMs - target.targetTsMs : UINT64_MAX;
                bool trackerTarget = (target.seq & BALLOON_TRACKER_SEQ_FLAG) != 0U;
                bool trackerSuppressedByYolo =
                    trackerTarget &&
                    milSuppressAfterYoloMs > 0 &&
                    lastAcceptedYoloControlMs > 0 &&
                    nowMs >= lastAcceptedYoloControlMs &&
                    nowMs - lastAcceptedYoloControlMs <= milSuppressAfterYoloMs;

                lastObservedSeq = target.seq;
                lastObservedTargetTsMs = target.targetTsMs;

                uint64_t adaptiveTargetTimeoutMs =
                    BalloonTargetAdaptiveTimeoutMs(&target,
                                                   targetTimeoutMs,
                                                   farTargetTimeoutMs);

                if (targetAgeMs == UINT64_MAX ||
                    targetAgeMs > adaptiveTargetTimeoutMs) {
                    AppendControlTargetEvent("target_rejected_stale_before_control",
                                             &target,
                                             DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                } else if (trackerTarget && targetAgeMs > milMaxAgeMs) {
                    AppendControlTargetEvent("tracker_rejected_stale_before_control",
                                             &target,
                                             DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                } else if (trackerSuppressedByYolo) {
                    AppendControlTargetEvent("tracker_rejected_recent_yolo",
                                             &target,
                                             DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                } else {
                    DjiTargetChase_SetRawObservationPixel((target.x1+target.x2)*0.5f,
                                                          (target.y1+target.y2)*0.5f,
                                                          target.targetTsMs);
                    bool targetJumpClamped = BalloonFilterTarget(&targetFilter, &target);

                    if (targetJumpClamped) {
                        AppendControlTargetEvent("tracker_jump_clamped",
                                                 &target,
                                                 DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                    }
                    code = DjiTargetChase_OnTargetBox(target.width,
                                                      target.height,
                                                      target.x1,
                                                      target.y2,
                                                      target.x2,
                                                      target.y1,
                                                      target.targetTsMs);
                    AppendControlTargetEvent(hasActiveTarget ? "target_box_refreshed" : "target_box_loaded",
                                             &target,
                                             code);
                    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        char detail[128];
                        snprintf(detail, sizeof(detail),
                                 "%s target loaded; conf=%.3f age_ms=%llu",
                                 BalloonTargetSourceText(target.seq),
                                 target.confidence,
                                 (unsigned long long) targetAgeMs);
                        BalloonStep_ShowSeq(BalloonTargetSourceText(target.seq), target.seq, code, detail);
                    } else {
                        BalloonStep_ShowSeq("YOLO target failed", target.seq, code, "target box rejected");
                    }
                    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        code = DjiTargetChase_SetManualTrigger(true, nowMs);
                        AppendControlEvent("manual_trigger_enabled_for_target", code, "");
                    }

                    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        bool wasHoverKeepaliveActive = hoverKeepaliveActive;
                        if (target.detectionFinishedTsMs > 0 && nowMs >= target.detectionFinishedTsMs) {
                            AppendLatencyEvent("detection_finish_to_control_observe",
                                               target.seq,
                                               nowMs - target.detectionFinishedTsMs,
                                               code);
                        }
                        if (target.targetTsMs > 0 && nowMs >= target.targetTsMs) {
                            AppendLatencyEvent("capture_to_control_observe",
                                               target.seq,
                                               nowMs - target.targetTsMs,
                                               code);
                        }
                        visualHold = false;
                        activeTarget = target;
                        lastAcceptedTargetTsMs = target.targetTsMs;
                        if (!trackerTarget) {
                            lastAcceptedYoloControlMs = nowMs;
                        }
                        hasActiveTarget = true;
                        targetUpdatedThisLoop = true;
                        activeCommandExpireMonoMs =
                            nowMonoMs +
                            BalloonTargetAdaptiveTimeoutMs(
                                &target,
                                targetTimeoutMs,
                                farTargetTimeoutMs);
                        lastRunOnceLogMs = 0;
                        currentTargetTimedOut = false;
                        burstHoverLocked = false;
                        hoverKeepaliveActive = false;
                        hoverStartedMonoMs = 0;
                        if (firstTargetLoadedMonoMs == 0 || wasHoverKeepaliveActive) {
                            firstTargetLoadedMonoMs = nowMonoMs;
                        }
                    } else if (code != (T_DjiReturnCode) 0x1B00000000ULL) {
                        AppendControlEvent("target_box_load_error", code, "");
                    }
                }
            }
        }

        if (visualHold) {
            /* No old command or prediction may be repeated while the newest
             * frame says the requested balloon is absent. */
            if (osal) osal->TaskSleepMs(BALLOON_CONTROL_LOOP_INTERVAL_MS);
            continue;
        }

        if (realControlMaxAfterTargetMs > 0 &&
            firstTargetLoadedMonoMs != 0 &&
            !hoverKeepaliveActive &&
            nowMonoMs >= firstTargetLoadedMonoMs &&
            nowMonoMs - firstTargetLoadedMonoMs > realControlMaxAfterTargetMs) {
            AppendControlEvent("control_max_duration_reached",
                               DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                               "abort after first target window");
            BalloonStep_Show("control abort", "max target window reached");
            break;
        }

        if (!currentTargetTimedOut &&
            !hoverKeepaliveActive &&
            lastAcceptedTargetTsMs != 0 &&
            nowMs > lastAcceptedTargetTsMs &&
            (nowMs - lastAcceptedTargetTsMs) >
                BalloonTargetAdaptiveTimeoutMs(&activeTarget,
                                               targetTimeoutMs,
                                               farTargetTimeoutMs)) {
            AppendControlEvent("target_timeout_hover_keepalive",
                               DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                               "stale YOLO observation; keep authority until stop");
            BalloonStep_Show("hover keepalive", "target stale; keep authority until stop");
            code = DjiTargetChase_Hover(nowMs);
            if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                code != (T_DjiReturnCode) 0x1B00000000ULL) {
                AppendControlEvent("target_timeout_hover_error", code, "");
            }
            hasActiveTarget = false;
            hoverKeepaliveActive = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                                   !noFlightMode &&
                                   DjiTargetChase_HasRepeatableCommand();
            hoverStartedMonoMs = hoverKeepaliveActive ? nowMonoMs : 0;
            currentTargetTimedOut = true;
        }

        if (hasActiveTarget && !currentTargetTimedOut) {
            if (activeCommandExpireMonoMs != 0 && nowMonoMs >= activeCommandExpireMonoMs) {
                AppendControlTargetEvent("detection_gap_hover", &activeTarget, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                BalloonStep_ShowSeq("hover", activeTarget.seq, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                    "2.5s without new detection; keep authority");
                code = DjiTargetChase_Hover(nowMs);
                if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                    code != (T_DjiReturnCode) 0x1B00000000ULL) {
                    AppendControlEvent("detection_gap_hover_error", code, "");
                    BalloonStep_ShowSeq("hover failed", activeTarget.seq, code, "keep authority path failed");
                }
                hasActiveTarget = false;
                activeCommandExpireMonoMs = 0;
                burstHoverLocked = false;
                hoverKeepaliveActive = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                                       !noFlightMode &&
                                       DjiTargetChase_HasRepeatableCommand();
                hoverStartedMonoMs = hoverKeepaliveActive ? nowMonoMs : 0;
                lastRunOnceLogMs = nowMs;
            } else if (burstHoverLocked) {
                if (lastRunOnceLogMs == 0 ||
                    nowMs - lastRunOnceLogMs >= BALLOON_RUN_ONCE_LOG_INTERVAL_MS) {
                    AppendControlTargetEvent("control_burst_hover_locked", &activeTarget,
                                             DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                    lastRunOnceLogMs = nowMs;
                }
            } else {
                if (targetUpdatedThisLoop) {
                    uint64_t runOnceStartMonoMs = MonoMs();
                    code = DjiTargetChase_RunOnce(nowMs);
                    AppendLatencyEvent("control_run_once_api",
                                       activeTarget.seq,
                                       MonoMs() - runOnceStartMonoMs,
                                       code);
                    AppendControlTargetEvent("run_once_after_yolo_success", &activeTarget, code);
                    BalloonStep_ShowSeq(noFlightMode ? "control dry-run" : "control update",
                                        activeTarget.seq,
                                        code,
                                        noFlightMode ? "new detection updated motion plan"
                                                     : "new detection updated joystick command");
                    lastRunOnceLogMs = nowMs;

                    if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                        E_TargetChaseMode mode = DjiTargetChase_GetMode();
                        if (noFlightMode) {
                            AppendControlTargetEvent(DjiTargetChase_HasRepeatableCommand()
                                                         ? "nonflight_control_plan_repeatable"
                                                         : "nonflight_control_plan_observed",
                                                     &activeTarget,
                                                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                        }
                        if (mode == TARGET_CHASE_MODE_HOLD || mode == TARGET_CHASE_MODE_TARGET_LOST) {
                            AppendControlTargetEvent(mode == TARGET_CHASE_MODE_HOLD
                                                         ? "control_burst_hold_locked"
                                                         : "control_burst_target_lost_locked",
                                                     &activeTarget,
                                                     DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                            code = DjiTargetChase_Hover(nowMs);
                            if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                                code != (T_DjiReturnCode) 0x1B00000000ULL) {
                                AppendControlEvent("control_burst_hold_hover_error", code, "");
                            }
                            burstHoverLocked = true;
                            hoverKeepaliveActive = code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                                                   !noFlightMode &&
                                                   DjiTargetChase_HasRepeatableCommand();
                            hoverStartedMonoMs = hoverKeepaliveActive ? nowMonoMs : 0;
                        }
                    }
                } else {
                    if (!DjiTargetChase_HasRepeatableCommand()) {
                        AppendControlTargetEvent("repeat_last_skipped_no_cached_command",
                                                 &activeTarget,
                                                 DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
                        hasActiveTarget = false;
                        activeCommandExpireMonoMs = 0;
                        burstHoverLocked = false;
                        hoverKeepaliveActive = false;
                        hoverStartedMonoMs = 0;
                        lastRunOnceLogMs = nowMs;
                    } else {
                        code = DjiTargetChase_RepeatLastCommand(nowMs);
                        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
                            lastRunOnceLogMs == 0 ||
                            nowMs - lastRunOnceLogMs >= BALLOON_RUN_ONCE_LOG_INTERVAL_MS) {
                            AppendControlTargetEvent("repeat_last_control_tick", &activeTarget, code);
                            lastRunOnceLogMs = nowMs;
                        }
                        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                            code != (T_DjiReturnCode) 0x1B00000000ULL) {
                            AppendControlEvent("repeat_last_control_error", code, "");
                        }
                    }
                }
            }
        } else if (hoverKeepaliveActive && !currentTargetTimedOut) {
            if (hoverStartedMonoMs != 0 &&
                nowMonoMs - hoverStartedMonoMs >= BALLOON_HOVER_WAIT_FOR_DETECTION_MS) {
                AppendControlEvent("hover_wait_timeout_keepalive",
                                   DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                   "100s without new detection; keep zero velocity until stop");
                BalloonStep_Show("hover keepalive", "100s no detection; keeping zero velocity");
                hoverStartedMonoMs = nowMonoMs;
                lastRunOnceLogMs = nowMs;
            } else if (!DjiTargetChase_HasRepeatableCommand()) {
                AppendControlEvent("hover_keepalive_stopped_no_cached_command",
                                   DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                                   "");
                BalloonStep_Show("hover refresh", "no cached command; resend hover");
                code = DjiTargetChase_Hover(nowMs);
                if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                    code != (T_DjiReturnCode) 0x1B00000000ULL) {
                    AppendControlEvent("hover_refresh_error", code, "");
                    hoverKeepaliveActive = false;
                    hoverStartedMonoMs = 0;
                } else {
                    hoverKeepaliveActive = !noFlightMode && DjiTargetChase_HasRepeatableCommand();
                    hoverStartedMonoMs = hoverKeepaliveActive ? nowMonoMs : 0;
                }
            } else {
                code = DjiTargetChase_RepeatLastCommand(nowMs);
                if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ||
                    lastRunOnceLogMs == 0 ||
                    nowMs - lastRunOnceLogMs >= BALLOON_RUN_ONCE_LOG_INTERVAL_MS) {
                    AppendControlEvent("hover_keepalive_tick", code, "");
                    lastRunOnceLogMs = nowMs;
                }
                if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
                    code != (T_DjiReturnCode) 0x1B00000000ULL) {
                    AppendControlEvent("hover_keepalive_error", code, "");
                }
            }
        }

        if (lastOperatorStatusMonoMs == 0 ||
            nowMonoMs - lastOperatorStatusMonoMs >= BALLOON_OPERATOR_STATUS_INTERVAL_MS) {
            BalloonShowOperatorStatus(noFlightMode,
                                      hasActiveTarget && !currentTargetTimedOut,
                                      currentTargetTimedOut || hoverKeepaliveActive,
                                      lastAcceptedTargetTsMs == 0 ? NULL : &activeTarget,
                                      nowMs,
                                      taskStartMs);
            lastOperatorStatusMonoMs = nowMonoMs;
        }

        if (osal != NULL) {
            osal->TaskSleepMs(BALLOON_CONTROL_LOOP_INTERVAL_MS);
        }
    }

    code = DjiTargetChase_DeInit();
    AppendControlEvent("target_chase_stop_result", code,
                       DjiTargetChase_IsReleasePending() ? "authority_release_pending" : "authority_not_held");
gate_cancelled:
    ;
    char visualMessage[256];
    s_stopRequested = true;
    (void) remove(BALLOON_PREDICTION_RECORD_REQUEST);
    (void) remove(BALLOON_MISSION_COMMIT);
    AppendControlEvent("control_task_stopped", code, "");
    if (DjiTargetChase_IsReleasePending()) {
        DjiWidgetFloatingWindow_ShowMessage("控制权释放未确认；请用遥控器原生操作接管，按停止测试重试");
    } else if (BalloonPhotoDetector_GetControlInitResult() != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        char message[256];
        snprintf(message, sizeof(message), "飞控初始化失败（0x%llX），任务未执行；请查看飞控日志",
                 (unsigned long long) BalloonPhotoDetector_GetControlInitResult());
        DjiWidgetFloatingWindow_ShowMessage(message);
    } else if (BalloonPhotoDetector_GetVisualFailure(visualMessage, sizeof(visualMessage))) {
        DjiWidgetFloatingWindow_ShowMessage(visualMessage);
    } else {
        DjiWidgetFloatingWindow_ShowMessage(s_hitUnconfirmedStop
            ? "目标消失，结果未确认；等待停止完成后请选择新的测试或API语音任务"
            : "追击控制线程已停止；等待相机退出并核实飞机状态");
    }
    s_controlActive = false;
    s_controlTask = NULL;
    return NULL;
}

static T_DjiReturnCode BalloonControl_Start(void)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    T_DjiReturnCode code;

    if (s_controlActive) {
        if (s_controlStopRequested) return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
        AppendControlEvent("control_already_running", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
        BalloonStep_Show("control already running", "waiting for YOLO target");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }

    if (osal == NULL) {
        AppendControlEvent("control_osal_null", DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN, "");
        BalloonStep_Show("control failed", "OSAL handler is null");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    s_controlStopRequested = false;
    s_controlActive = true;
    code = osal->TaskCreate("balloon_control", BalloonControlTask, 64 * 1024, NULL, &s_controlTask);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_controlActive = false;
        s_controlTask = NULL;
        AppendControlEvent("control_task_create_failed", code, "");
        BalloonStep_ShowSeq("control failed", 0, code, "task create failed");
        return code;
    }

    AppendControlEvent("control_task_created", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
    BalloonStep_Show("control task", "waiting for YOLO target");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void BalloonControl_Stop(void)
{
    if (s_controlStopRequested) {
        DjiTargetChase_RequestStop();
        return;
    }

    s_controlStopRequested = true;
    DjiTargetChase_RequestStop();
    AppendControlEvent("control_stop_requested", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
    /*
     * The control task owns target-chase teardown. Calling Abort here as well
     * races its DjiTargetChase_DeInit() path and can enter DJI's synchronous
     * joystick-authority release API twice, which was observed to deadlock the
     * PSDK message queue. The control loop wakes every 50 ms, then sends zero
     * velocity and releases authority exactly once from its own thread.
     */
    BalloonStep_Show("control stop", "stop requested; control task owns safe abort");
}

static T_DjiReturnCode RunLiveviewDetectorLoop(T_DjiOsalHandler *osal,
                                               E_DjiMountPosition position,
                                               uint64_t taskStartMs,
                                               uint64_t taskStartMonoMs)
{
    T_DjiReturnCode code;
    T_BalloonLiveviewFrame frame;
    uint64_t lastSubmittedFrameId = 0;
    char lastSubmittedFramePath[BALLOON_PATH_SIZE] = {0};
    uint64_t lastRequestMonoMs = 0;
    uint64_t lastIFrameMonoMs = 0;
    uint64_t waitStartedMonoMs = MonoMs();
    uint64_t photoLoopMaxDurationMs = ReadU64EnvBounded("BALLOON_PHOTO_LOOP_MAX_DURATION_MS",
                                                        BALLOON_DEFAULT_PHOTO_LOOP_MAX_DURATION_MS,
                                                        10000ULL,
                                                        86400000ULL,
                                                        true);
    bool firstFrameShown = false;
    bool waitWarningLogged = false;

    BalloonStep_Show("liveview", "initializing gimbal H264 stream");
    code = StartLiveview(position);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        BalloonStep_ShowSeq("liveview failed", 0, code, "check USB and gimbal camera source");
        (void) remove(BALLOON_PREDICTION_RECORD_REQUEST);
        return code;
    }

    AppendEvent("liveview_detector_loop_started", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                BALLOON_LIVEVIEW_METADATA);
    BalloonStep_Show("liveview started", "waiting for decoded frame");
    lastIFrameMonoMs = MonoMs();

    while (!s_stopRequested) {
        uint64_t nowMs = NowMs();
        uint64_t monoMs = MonoMs();

        if (NaturalLanguageStopRequested()) {
            break;
        }

        if (photoLoopMaxDurationMs > 0 &&
            taskStartMonoMs != 0 && monoMs >= taskStartMonoMs &&
            monoMs - taskStartMonoMs > photoLoopMaxDurationMs) {
            AppendEvent("auto_stop_max_duration", s_captureSeq,
                        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                        "liveview loop max duration reached");
            s_stopRequested = true;
            BalloonControl_Stop();
            break;
        }

        if (monoMs - lastIFrameMonoMs >= BALLOON_LIVEVIEW_I_FRAME_INTERVAL_MS) {
            code = DjiLiveview_RequestIntraframeFrameData((E_DjiLiveViewCameraPosition) position,
                                                          DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT);
            AppendEvent("liveview_request_iframe", s_captureSeq, code, "");
            lastIFrameMonoMs = monoMs;
        }

        if (ReadLatestLiveviewFrame(&frame) &&
            frame.readyTsMs >= taskStartMs &&
            nowMs >= frame.readyTsMs &&
            nowMs - frame.readyTsMs <= BALLOON_LIVEVIEW_FRAME_TIMEOUT_MS &&
            (frame.frameId != lastSubmittedFrameId ||
             strcmp(frame.path, lastSubmittedFramePath) != 0) &&
            (lastRequestMonoMs == 0 ||
             monoMs - lastRequestMonoMs >= BALLOON_LIVEVIEW_REQUEST_INTERVAL_MS)) {
            uint32_t seq = ++s_captureSeq;
            uint64_t requestStartMonoMs = MonoMs();

            code = LaunchDetection(frame.path, seq, frame.readyTsMs);
            AppendLatencyEvent("liveview_frame_ready_to_request", seq,
                               nowMs - frame.readyTsMs, code);
            AppendLatencyEvent("liveview_jpeg_encode", seq, frame.encodeMs,
                               DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
            AppendLatencyEvent("detection_request_commit", seq,
                               MonoMs() - requestStartMonoMs, code);
            AppendEvent("liveview_detection_launched", seq, code, frame.path);
            if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                lastSubmittedFrameId = frame.frameId;
                snprintf(lastSubmittedFramePath, sizeof(lastSubmittedFramePath), "%s", frame.path);
                lastRequestMonoMs = monoMs;
                if (!firstFrameShown) {
                    BalloonStep_ShowSeq("liveview frame", seq, code, "YOLO pipeline running");
                    firstFrameShown = true;
                }
            }
        } else if (!firstFrameShown && !waitWarningLogged &&
                   monoMs - waitStartedMonoMs > BALLOON_LIVEVIEW_FRAME_TIMEOUT_MS) {
            AppendEvent("liveview_waiting_for_frame", 0,
                        DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT,
                        "decoder metadata not ready");
            BalloonStep_Show("liveview waiting", "decoder has not published a frame yet");
            waitWarningLogged = true;
        }

        if (osal != NULL) {
            osal->TaskSleepMs(10);
        }
    }

    StopLiveview(position, "liveview_detector_loop_exit");
    (void) remove(BALLOON_PREDICTION_RECORD_REQUEST);
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static void *PhotoDetectorTask(void *arg)
{
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();
    E_DjiMountPosition position = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
    T_DjiReturnCode code;
    uint32_t consecutiveDownloadFailures = 0;
    uint64_t taskStartMs = NowMs();
    uint64_t taskStartMonoMs = MonoMs();
    uint64_t photoLoopMaxDurationMs = ReadU64EnvBounded("BALLOON_PHOTO_LOOP_MAX_DURATION_MS",
                                                        BALLOON_DEFAULT_PHOTO_LOOP_MAX_DURATION_MS,
                                                        10000ULL,
                                                        86400000ULL,
                                                        true);

    (void) arg;
    EnsureDirectories();
    AppendEvent("photo_task_started", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");

    if (UseLiveviewInput()) {
        AppendEvent("input_mode", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "liveview");
        code = RunLiveviewDetectorLoop(osal, position, taskStartMs, taskStartMonoMs);
        AppendEvent("photo_task_stopped", s_captureSeq, code, "liveview");
        BalloonStep_Show("liveview task stopped", "stream released");
        s_workerActive = false;
        s_workerTask = NULL;
        return NULL;
    }

    AppendEvent("input_mode", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "photo_download");
    BalloonStep_Show("photo task", "camera manager init");

    code = EnsureCameraManagerReady(position);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        AppendEvent("camera_manager_not_ready", 0, code, "");
        BalloonStep_ShowSeq("camera init failed", 0, code, "camera manager not ready");
        CleanupCameraManager(position, "camera_manager_not_ready");
        AppendEvent("photo_task_stopped", s_captureSeq, code, "camera_manager_not_ready");
        s_workerActive = false;
        s_workerTask = NULL;
        return NULL;
    }
    LogCameraReadback(position);
    AppendEvent("camera_readback_logged", 0, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, BALLOON_CAMERA_DIAG_LOG);
    BalloonStep_Show("camera ready", "building media baseline");

    code = InitializeMediaBaseline(position);
    AppendEvent("media_baseline_done", 0, code, "");
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        BalloonStep_ShowSeq("media baseline failed", 0, code, "refuse old-photo detection");
        CleanupCameraManager(position, "media_baseline_failed");
        AppendEvent("photo_task_stopped", s_captureSeq, code, "media_baseline_failed");
        s_workerActive = false;
        s_workerTask = NULL;
        return NULL;
    }
    if (osal != NULL) {
        osal->TaskSleepMs(800);
    }
    BalloonStep_Show("camera ready", "photo plus YOLO loop running");

    while (!s_stopRequested) {
        uint64_t nowMonoMs = MonoMs();
        if (NaturalLanguageStopRequested()) {
            break;
        }
        if (photoLoopMaxDurationMs > 0 &&
            taskStartMonoMs != 0 &&
            nowMonoMs >= taskStartMonoMs &&
            nowMonoMs - taskStartMonoMs > photoLoopMaxDurationMs) {
            AppendEvent("auto_stop_max_duration",
                        s_captureSeq,
                        DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS,
                        "photo loop max duration reached");
            s_stopRequested = true;
            BalloonControl_Stop();
            break;
        }

        uint32_t seq = ++s_captureSeq;
        char photoPath[BALLOON_PATH_SIZE] = {0};
        uint64_t captureTsMs = 0;
        uint64_t stageStartMonoMs = 0;
        uint64_t minNewerThanKey = s_lastDownloadedKey;

        AppendEvent("photo_only_no_precapture_media_scan", seq, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
        AppendEvent("capture_start", seq, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
        BalloonStep_ShowSeq("capture start", seq, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "shooting single photo");
        stageStartMonoMs = MonoMs();
        code = CaptureSinglePhoto(position);
        captureTsMs = NowMs();
        AppendLatencyEvent("camera_capture_api", seq, MonoMs() - stageStartMonoMs, code);
        AppendEvent("capture_done", seq, code, "");
        BalloonStep_ShowSeq(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "capture done" : "capture failed",
                            seq,
                            code,
                            "");
        if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && osal != NULL) {
            stageStartMonoMs = MonoMs();
            osal->TaskSleepMs(800);
            AppendLatencyEvent("post_capture_settle_wait",
                               seq,
                               MonoMs() - stageStartMonoMs,
                               DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS);
        }

        if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            stageStartMonoMs = MonoMs();
            code = DownloadNewPhoto(position, seq, minNewerThanKey, photoPath, sizeof(photoPath));
            AppendLatencyEvent("media_wait_and_download", seq, MonoMs() - stageStartMonoMs, code);
            AppendEvent("download_done", seq, code, photoPath);
            BalloonStep_ShowSeq(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "photo downloaded" : "download failed",
                                seq,
                                code,
                                code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "queue YOLO request" : "");
            if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                consecutiveDownloadFailures = 0;
                stageStartMonoMs = MonoMs();
                code = LaunchDetection(photoPath, seq, captureTsMs);
                AppendLatencyEvent("detection_request_commit", seq, MonoMs() - stageStartMonoMs, code);
                AppendEvent("detection_launched", seq, code, photoPath);
                BalloonStep_ShowSeq(code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "YOLO request" : "YOLO request failed",
                                    seq,
                                    code,
                                    code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? "waiting worker result" : "");
            } else {
                consecutiveDownloadFailures++;
                AppendEvent("download_failure_count", consecutiveDownloadFailures, code, "");
                if (consecutiveDownloadFailures >= BALLOON_MAX_CONSECUTIVE_DOWNLOAD_FAILURES) {
                    USER_LOG_ERROR("Stop balloon photo detector after %u consecutive download failures.",
                                   consecutiveDownloadFailures);
                    AppendEvent("auto_stop_download_failures", consecutiveDownloadFailures, code, "");
                    s_stopRequested = true;
                }
            }
        }

        if (osal != NULL) {
            osal->TaskSleepMs(BALLOON_PHOTO_INTERVAL_MS);
        }
    }

    CleanupCameraManager(position, "photo_task_exit");
    AppendEvent("photo_task_stopped", s_captureSeq, DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, "");
    BalloonStep_Show("photo task stopped", "camera manager released");
    s_workerActive = false;
    s_workerTask = NULL;
    return NULL;
}

T_DjiReturnCode BalloonPhotoDetector_Start(void)
{
    T_DjiReturnCode code;
    T_DjiOsalHandler *osal = DjiPlatform_GetOsalHandler();

    if (s_calibrationLiveviewOwned) return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;

    if (s_workerActive || s_controlActive) {
        USER_LOG_INFO("Balloon photo detector already running");
        BalloonStep_Show("photo detector", "already running");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }

    if (osal == NULL) {
        USER_LOG_ERROR("Balloon photo detector OSAL handler is null");
        BalloonStep_Show("photo detector failed", "OSAL handler is null");
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }

    BalloonControl_RecordInitResult(DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, false);
    BalloonVisual_RecordFailure(0, NULL);
    s_stopRequested = false;
    s_nlStopObserved = false;
    s_hitUnconfirmedStop = false;
    s_workerActive = true;
    code = BalloonControl_Start();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_workerActive = false;
        s_stopRequested = true;
        AppendControlEvent("detector_control_start_failed", code, "retry after stop completes");
        return code;
    }
    code = osal->TaskCreate("balloon_photo_detector", PhotoDetectorTask, 64 * 1024, NULL, &s_workerTask);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        s_workerActive = false;
        s_workerTask = NULL;
        BalloonControl_Stop();
        USER_LOG_ERROR("Create balloon photo detector task failed: 0x%08llX", (unsigned long long) code);
        BalloonStep_ShowSeq("photo detector failed", 0, code, "task create failed");
        return code;
    }

    USER_LOG_INFO("Balloon photo detector started");
    BalloonStep_Show("photo detector started", "camera init pending");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode BalloonPhotoDetector_Stop(void)
{
    return BalloonPhotoDetector_StopWithReason("unspecified_api_caller");
}

T_DjiReturnCode BalloonPhotoDetector_StopWithReason(const char *reason)
{
    /* Whitelist characters so a diagnostic identifier cannot break JSONL. */
    char safeReason[96];
    size_t i = 0;
    if (reason == NULL) reason = "unspecified_api_caller";
    for (; reason[i] && i + 1 < sizeof(safeReason); ++i) {
        char c = reason[i];
        safeReason[i] = ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                         (c >= '0' && c <= '9') || c == '_' || c == '-') ? c : '_';
    }
    safeReason[i] = 0;
    AppendControlEvent("detector_stop_requested", DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS, safeReason);
    s_stopRequested = true;
    (void) remove(BALLOON_PREDICTION_RECORD_REQUEST);
    BalloonControl_Stop();
    USER_LOG_INFO("Balloon photo detector stop requested");
    BalloonStep_Show("photo detector stop", "waiting worker exit and camera release");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool BalloonPhotoDetector_IsRunning(void)
{
    return s_workerActive || s_controlActive;
}

bool BalloonPhotoDetector_IsStopping(void)
{
    return (s_workerActive || s_controlActive) && (s_stopRequested || s_controlStopRequested);
}
