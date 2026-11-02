/* Private implementation for the nine flight-panel controls. Included once. */
#ifndef BALLOON_FLIGHT_PANEL_H
#define BALLOON_FLIGHT_PANEL_H

#define PANEL_HELPER "/home/uav/offline_balloon_strike_test/flight_panel_tools.py"
#define PANEL_MESSAGE "/tmp/balloon_panel_message.txt"
#define API_VOICE_SESSION "/tmp/balloon_api_voice.session"
#define API_VOICE_READY "/tmp/balloon_api_voice.ready"
#define API_VOICE_CANDIDATE "/home/uav/offline_balloon_strike_test/voice_command_candidate.json"

static T_DjiReturnCode BalloonWidget_BeginMission(const char *mode);
static T_DjiReturnCode BalloonWidget_Stop(void);
static bool BalloonWidget_WritePlain(const char *path, const char *text);

/* Fork registration, nonblocking reap and Stop share the action mutex. Never
 * leave a reaped PID registered: cancellation must not signal a reused PID. */
static int BalloonPanel_RunChild(char *const argv[], unsigned int epoch, pid_t *owner)
{
    pid_t child;
    int status = 0;
    uint64_t cancelledAt = 0;
    pthread_mutex_lock(&s_actionMutex);
    if (epoch != s_actionEpoch) { pthread_mutex_unlock(&s_actionMutex); return -1; }
    child = fork();
    if (child == 0) {
        struct sigaction action;
        memset(&action, 0, sizeof(action));
        action.sa_handler = SIG_DFL;
        sigemptyset(&action.sa_mask);
        sigaction(SIGTERM, &action, NULL);
        sigaction(SIGINT, &action, NULL);
        sigaction(SIGQUIT, &action, NULL);
        sigaction(SIGPIPE, &action, NULL);
        setpgid(0, 0);
        execv(argv[0], argv);
        _exit(127);
    }
    if (child < 0) { pthread_mutex_unlock(&s_actionMutex); return -1; }
    (void) setpgid(child, child);
    *owner = child;
    pthread_mutex_unlock(&s_actionMutex);
    for (;;) {
        pthread_mutex_lock(&s_actionMutex);
        pid_t result = waitpid(child, &status, WNOHANG);
        if (result == child || (result < 0 && errno != EINTR)) {
            *owner = 0;
            pthread_mutex_unlock(&s_actionMutex);
            return result == child && WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        }
        if (epoch != s_actionEpoch) {
            uint64_t now = BalloonClock_NowMs();
            if (!cancelledAt) { cancelledAt = now; (void) kill(-child, SIGTERM); }
            else if (now - cancelledAt >= 2000) (void) kill(-child, SIGKILL);
        }
        pthread_mutex_unlock(&s_actionMutex);
        usleep(20000);
    }
}

static void BalloonPanel_ClearVoice(void)
{
    /* Token deletion is the synchronous cancellation barrier. Python never
     * recreates an absent/mismatching token when a worker finishes late. */
    (void) remove(API_VOICE_SESSION);
    (void) remove(API_VOICE_READY);
    (void) remove(API_VOICE_CANDIDATE);
    (void) remove(BALLOON_VOICE_MESSAGE_FILE);
    s_voiceSession[0] = 0;
}

static bool BalloonPanel_NewVoiceSession(void)
{
    char clock[80] = {0}, temporary[128];
    if (!BalloonClock_Id(clock)) return false;
    BalloonPanel_ClearVoice();
    snprintf(s_voiceSession, sizeof(s_voiceSession), "voice-%.36s-%lu-%llu-%u",
             clock + 8, (unsigned long)getpid(),
             (unsigned long long)BalloonClock_NowMs(), ++s_voiceSequence);
    snprintf(temporary, sizeof(temporary), "%s.tmp", API_VOICE_SESSION);
    if (!BalloonWidget_WritePlain(temporary, s_voiceSession) || rename(temporary, API_VOICE_SESSION) != 0) {
        BalloonPanel_ClearVoice(); return false;
    }
    return true;
}

static bool BalloonPanel_VoiceReady(void)
{
    return s_voiceSession[0] && BalloonMission_Matches(API_VOICE_SESSION, s_voiceSession) &&
           BalloonMission_Matches(API_VOICE_READY, s_voiceSession);
}

static bool BalloonPanel_Idle(void)
{
    return !BalloonPhotoDetector_IsRunning() && !BalloonPhotoDetector_IsStopping() &&
           !BalloonWidget_HelperBusy() && !BalloonTiming_Requested() && !s_timingReserved;
}

typedef struct { unsigned int epoch; int kind; } T_BalloonPanelTask;
enum { PANEL_AUDIT, PANEL_SAMPLE, PANEL_KALMAN, PANEL_RELEASE };

static void *BalloonPanel_Task(void *argument)
{
    T_BalloonPanelTask *task = argument;
    char *audit[] = {"/usr/bin/timeout", "--kill-after=2s", "20s", BALLOON_VOICE_PYTHON,
                     PANEL_HELPER, "--audit", NULL};
    char *sample[] = {"/usr/bin/timeout", "--kill-after=2s", "50s", BALLOON_VOICE_PYTHON,
                      PANEL_HELPER, "--sample", "--seconds", "30", "--stationary-aircraft", NULL};
    char *kalman[] = {"/usr/bin/timeout", "--kill-after=2s", "10s", BALLOON_VOICE_PYTHON,
                      PANEL_HELPER, "--check-kalman", NULL};
    int result = task->kind == PANEL_RELEASE ?
                 (DjiTargetChase_DeInit() == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS ? 0 : 1) :
                 BalloonPanel_RunChild(task->kind == PANEL_SAMPLE ? sample :
                                      task->kind == PANEL_KALMAN ? kalman : audit,
                                      task->epoch, &s_panelPid);
    pthread_mutex_lock(&s_actionMutex);
    if (task->epoch != s_actionEpoch) {
        /* The child is now reaped. It cannot recreate these after cleanup. */
        if (task->kind == PANEL_SAMPLE) {
            (void) remove(BALLOON_TIMING_REQUEST);
            (void) remove(BALLOON_PREDICTION_RECORD_REQUEST);
        }
    } else if (task->kind == PANEL_RELEASE) {
        BalloonWidget_ShowMessage(DjiTargetChase_IsReleasePending() ?
            "控制权释放重试失败，请用遥控器原生操作接管并检查日志" :
            result == 0 ? "控制权释放重试成功；请核实飞机状态后选择新任务" :
            "已不再持有控制权，但停止过程有错误，请检查日志");
    } else if (task->kind == PANEL_KALMAN && result == 0) {
        if (setenv("BALLOON_PREDICTION_MODE", "3", 1) == 0) {
            s_predictionMode = 3;
            BalloonWidget_ShowMessage("Kalman模式已选择，下次任务生效；实际启用/回退以运行状态为准");
        } else BalloonWidget_ShowMessage("Kalman设置失败，原模式保持不变");
    } else if (result != 0 && result != 1 && result != 2 && result != 3 && result != 130) {
        BalloonWidget_ShowMessage(result == 124 || result == 137 ?
            "采样/核验超时，本轮未通过；请查看日志后重试" :
            "辅助程序异常退出，本轮未通过；请检查部署和日志");
    } else {
        char message[512] = {0};
        FILE *fp = fopen(PANEL_MESSAGE, "r");
        if (fp) { (void)fread(message, 1, sizeof(message)-1, fp); fclose(fp); }
        BalloonWidget_ShowMessage(message[0] ? message : "检查未完成，请检查辅助程序和日志");
    }
    s_panelBusy = false;
    s_panelSampling = false;
    pthread_mutex_unlock(&s_actionMutex);
    free(task);
    return NULL;
}

/* Only after the control thread has exited; never race its own DeInit. */
static void BalloonPanel_RetryRelease(void)
{
    pthread_t thread;
    if (BalloonPhotoDetector_IsRunning() || s_panelBusy || !DjiTargetChase_IsReleasePending()) return;
    T_BalloonPanelTask *task = calloc(1, sizeof(*task));
    if (!task) return;
    task->epoch = s_actionEpoch; task->kind = PANEL_RELEASE;
    s_panelBusy = true;
    if (pthread_create(&thread, NULL, BalloonPanel_Task, task) != 0) {
        s_panelBusy = false; free(task); return;
    }
    pthread_detach(thread);
}

static T_DjiReturnCode BalloonPanel_StartTool(int kind)
{
    pthread_t thread;
    if (kind == PANEL_SAMPLE && s_displayOnly) {
        BalloonWidget_ShowMessage("仅显示模式：维护锁生效，相机/姿态采样未启动");
        return DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE;
    }
    if (!BalloonPanel_Idle() || s_newVoicePending) {
        BalloonWidget_ShowMessage("请等待当前任务结束；新增语音录入中请先清除候选语音任务");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    if (kind == PANEL_SAMPLE) {
        uint64_t now = BalloonClock_NowMs();
        if (!s_sampleConfirmUntil || now > s_sampleConfirmUntil) {
            s_sampleConfirmUntil = now + 15000;
            BalloonWidget_ShowMessage("确认飞机在地面静止：15秒内再次点击，采集30秒相机/姿态；可手动缓慢转动云台");
            return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
        }
        s_sampleConfirmUntil = 0;
        if (DjiTargetChase_IsReleasePending()) {
            BalloonWidget_ShowMessage("控制权释放未确认，禁止开始采样");
            return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
        }
    }
    T_BalloonPanelTask *task = calloc(1, sizeof(*task));
    if (!task) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    task->epoch = s_actionEpoch; task->kind = kind;
    (void)remove(PANEL_MESSAGE);
    s_panelBusy = true; s_panelSampling = kind == PANEL_SAMPLE;
    if (pthread_create(&thread, NULL, BalloonPanel_Task, task) != 0) {
        s_panelBusy = false; s_panelSampling = false; free(task);
        return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
    }
    pthread_detach(thread);
    BalloonWidget_ShowMessage(kind == PANEL_SAMPLE ? "正在采集相机/姿态；采样完成不等于时间标定有效" :
                             kind == PANEL_KALMAN ? "正在核验本次开机时间标定" : "正在核验部署、运行版本和时间标定");
    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode BalloonPanel_NewApiTask(void)
{
    if (!s_newVoicePending) {
        (void)BalloonWidget_Stop();
        s_newVoicePending = true;
        s_newVoicePromptPending = true;
        BalloonWidget_ShowMessage("正在结束旧任务；等待提示后重新输入语音，再回来点击新增API语音任务");
        return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
    }
    if (s_newVoicePromptPending || !BalloonPanel_Idle()) {
        BalloonWidget_ShowMessage("正在停止旧任务或识别新录音，请等待提示");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    if (!BalloonPanel_VoiceReady()) {
        BalloonWidget_ShowMessage("请使用遥控器语音键录入新的指令，识别完成后再次点击本按钮");
        return DJI_ERROR_SYSTEM_MODULE_CODE_BUSY;
    }
    return BalloonWidget_BeginMission("api_new");
}

#endif
