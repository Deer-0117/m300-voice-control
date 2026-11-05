#ifndef DJI_TARGET_CHASE_TASK_H
#define DJI_TARGET_CHASE_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "dji_typedef.h"
#include "target_chase.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DJI_TARGET_CHASE_GIMBAL_DISABLED = 0,
    DJI_TARGET_CHASE_GIMBAL_RESETTING,
    DJI_TARGET_CHASE_GIMBAL_READY,
    DJI_TARGET_CHASE_GIMBAL_SEARCHING,
    DJI_TARGET_CHASE_GIMBAL_BODY_TURNING,
    DJI_TARGET_CHASE_GIMBAL_TRACKING, /* First-lock centering; body motion remains inhibited. */
    DJI_TARGET_CHASE_GIMBAL_NEAR_EDGE_RECOVERING,
    DJI_TARGET_CHASE_GIMBAL_STATIC_TEST,
    DJI_TARGET_CHASE_GIMBAL_ERROR,
} E_DjiTargetChaseGimbalState;

/*
 * Call after the DJI scheduler and the flight-control/data-subscription
 * modules are ready. When BALLOON_GIMBAL_TRACKING is enabled (the default),
 * initialization also starts the DJI gimbal manager and subscribes gimbal
 * angles. Managed panel tasks preserve the current pose and gimbal mode,
 * wait for a fresh visible target, and never reset or search automatically.
 * Legacy callers reset at startup. BALLOON_GIMBAL_STATIC_TEST=1 keeps the gimbal
 * centered after startup reset. BALLOON_GIMBAL_SEARCH=0 enables tracking
 * without idle search motion, so the camera waits centered for a fresh target.
 * Stop/takeover does not reset pitch/yaw. The task does not take off; it only takes joystick
 * authority after a fresh target pixel arrives and the RC/manual trigger has
 * been enabled.
 */
T_DjiReturnCode DjiTargetChase_Init(const T_TargetChaseConfig *config);
T_DjiReturnCode DjiTargetChase_DeInit(void);

/* Process-local panel mode; setting it never invokes an SDK operation or
 * starts a task. The caller must serialize with task creation and wait for
 * every worker to exit. Live/initialized or unreleased controllers reject
 * the change without changing the prior mode. A panel calls this with true
 * at every startup. Maintenance blocks Init entirely; false permits a later
 * explicit task to use flight control and visible-target-only gimbal tracking
 * (no reset, idle search, or near-edge recovery, regardless of environment).
 * Before the first
 * call, the two Requested getters preserve the legacy environment behavior. */
T_DjiReturnCode DjiTargetChase_SetPanelMaintenance(bool enabled);
bool DjiTargetChase_RequestedNoFlight(void);
bool DjiTargetChase_RequestedGimbalTracking(void);

/*
 * Feed detector center output here. u_px/v_px are pixel coordinates in the
 * same image geometry used by T_TargetChaseConfig. Prefer
 * DjiTargetChase_OnTargetBox() when the detector can provide a YOLO box.
 */
T_DjiReturnCode DjiTargetChase_OnTargetPixel(float u_px, float v_px, uint64_t now_ms);
/* Stage the unsmoothed detector center for the matching OnTargetBox call.
 * Does not start tracking, take authority, or send a command. */
void DjiTargetChase_SetRawObservationPixel(float u_px, float v_px, uint64_t frame_ms);

/*
 * Feed YOLO detector output here. The box is provided as left-bottom and
 * right-top pixel coordinates. The implementation normalizes min/max
 * internally, so image y-axis direction does not matter. Each fresh box also
 * stops the search motion, updates the camera boresight from gimbal attitude,
 * and commands bounded pitch/yaw speeds to keep the target centered.
 */
T_DjiReturnCode DjiTargetChase_OnTargetBox(float image_width_px,
                                           float image_height_px,
                                           float left_bottom_u_px,
                                           float left_bottom_v_px,
                                           float right_top_u_px,
                                           float right_top_v_px,
                                           uint64_t now_ms);

/*
 * Call this from your RC switch/button handler. When disabled, RunOnce will
 * send zero velocity and release joystick authority if this task owns it.
 */
T_DjiReturnCode DjiTargetChase_SetManualTrigger(bool enabled, uint64_t now_ms);
bool DjiTargetChase_IsManualTriggerEnabled(void);

/*
 * Call after a fresh detector observation has been fed with OnTargetPixel()
 * or OnTargetBox(). This function reads latest FC subscription data,
 * obtains/releases joystick authority, and sends one trajectory correction
 * command when manual trigger, a fresh target, and joystick authority allow it.
 */
T_DjiReturnCode DjiTargetChase_RunOnce(uint64_t now_ms);
bool DjiTargetChase_IsNoFlightMode(void);
bool DjiTargetChase_HasRepeatableCommand(void);
bool DjiTargetChase_GetLastCommand(T_TargetChaseCommand *command);

/*
 * Advance the non-blocking gimbal reset/search/lost-target state machine.
 * Call this from the detector control loop even when no target is available.
 */
void DjiTargetChase_GimbalTick(uint64_t now_ms);
/*
 * Execute the zero-translation aircraft yaw part of cyclic search. The gimbal
 * state machine requests one body step only after a complete two-row scan.
 * BALLOON_NO_FLIGHT=1 logs the planned command without sending it.
 */
T_DjiReturnCode DjiTargetChase_SearchTick(uint64_t now_ms);
/*
 * Drop the previous target and immediately restart the cyclic gimbal/body
 * search. Used for deterministic transitions between ordered strike stages.
 * Managed panel tasks instead stop gimbal motion and wait for the next
 * visible target in the current camera view; they do not initiate a scan.
 */
T_DjiReturnCode DjiTargetChase_RequestSearch(uint64_t now_ms,
                                             bool fast_pass);
E_DjiTargetChaseGimbalState DjiTargetChase_GetGimbalState(void);
const char *DjiTargetChase_GetGimbalStateText(void);
bool DjiTargetChase_IsSearchExhausted(void);

/*
 * Send a zero-velocity joystick command while keeping authority. This is the
 * detector-gap hover path; Abort() is still used for stop/release.
 */
T_DjiReturnCode DjiTargetChase_Hover(uint64_t now_ms);

/*
 * Resend the last command computed by DjiTargetChase_RunOnce() without
 * reading FC state or recomputing the planner. Use only as a joystick
 * keepalive while waiting for the next detector observation or timeout.
 */
T_DjiReturnCode DjiTargetChase_RepeatLastCommand(uint64_t now_ms);

/*
 * Use this from an RC pause button, app command, or detector stop event.
 */
T_DjiReturnCode DjiTargetChase_Abort(void);

/* Stop/takeover latches all motion until a successful DeInit and explicit Init.
 * Pending release must block a new mission; retry DeInit to retry release.
 * The last result includes failed zero commands even if release succeeded. */
bool DjiTargetChase_IsControlInhibited(void);
/* Synchronous stop latch only; no SDK calls or controller teardown. */
void DjiTargetChase_RequestStop(void);
bool DjiTargetChase_IsReleasePending(void);
T_DjiReturnCode DjiTargetChase_GetLastStopResult(void);

E_TargetChaseMode DjiTargetChase_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif /* DJI_TARGET_CHASE_TASK_H */
