/*
 * Balloon strike Widget bridge.
 *
 * Mission buttons prepare and start any-balloon, pink/yellow or voice tasks.
 * The detector's control thread remains gated until the exact plan is loaded
 * by YOLO and optional video recording has acknowledged its first frame.
 * Stop cancels pending starts and running tasks. No automatic takeoff or RTH.
 * UI IDs 18..25 are new, so cached old Prepare controls cannot start flight.
 */

#ifndef PSDK_BALLOON_WIDGET_H
#define PSDK_BALLOON_WIDGET_H

#include "dji_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

T_DjiReturnCode BalloonWidget_StartService(const char *widgetConfigDirPath,
                                           const char *armedFilePath,
                                           const char *triggerFilePath,
                                           const char *stopFilePath);

#ifdef __cplusplus
}
#endif

#endif
