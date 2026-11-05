#ifndef BALLOON_PHOTO_DETECTOR_H
#define BALLOON_PHOTO_DETECTOR_H

#include <stddef.h>
#include <stdbool.h>

#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Start the camera/live-view, detector orchestration, and target-chase worker
 * tasks. The DJI platform and the modules used by this component must already
 * be initialized by the caller.
 *
 * Static tests and first deployments must set BALLOON_NO_FLIGHT=1 in the
 * service's effective environment before this function is called.
 */
T_DjiReturnCode BalloonPhotoDetector_Start(void);

/*
 * Start the independent native program-auto exposure watchdog when enabled by
 * BALLOON_AUTO_EXPOSURE=1. Unsupported camera firmware is reported through
 * camera_diag.jsonl without preventing the Widget or flight-control service
 * from starting.
 */
T_DjiReturnCode BalloonPhotoDetector_StartAutoExposure(void);

/* Idle calibration stream only: no detector, chase, flight or gimbal commands.
 * Serialize with mission lifecycle through Widget's action mutex. */
T_DjiReturnCode BalloonPhotoDetector_StartCalibrationLiveview(void);
T_DjiReturnCode BalloonPhotoDetector_StopCalibrationLiveview(void);

/*
 * Request both detector and target-chase workers to stop. Worker shutdown is
 * asynchronous; use BalloonPhotoDetector_IsRunning() to observe completion.
 */
T_DjiReturnCode BalloonPhotoDetector_Stop(void);
/* reason must be a fixed caller identifier, not arbitrary external text. */
T_DjiReturnCode BalloonPhotoDetector_StopWithReason(const char *reason);

/* Return true while the detector worker is active or shutting down. */
bool BalloonPhotoDetector_IsRunning(void);
bool BalloonPhotoDetector_IsStopping(void);

/* Last accepted task's initialization failure, retained through cleanup.
 * Success also represents not attempted or explicitly cancelled initialization. */
T_DjiReturnCode BalloonPhotoDetector_GetControlInitResult(void);
/* Persistent localized visual failure; reset only when a new task starts. */
bool BalloonPhotoDetector_GetVisualFailure(char *message, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* BALLOON_PHOTO_DETECTOR_H */
