#ifndef M300_RUNTIME_H
#define M300_RUNTIME_H
#include "dji_flight_controller.h"
#include "dji_liveview.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Invoke once after platform/core initialization, in a user task.
 * Disable all other widget, joystick, FC subscription and Liveview samples.
 * Pass the installation's real RID configuration, never synthetic test values.
 * liveview_fifo may point to the existing decoder's FIFO via package inputs.
 */
T_DjiReturnCode M300Runtime_Start(const char *package_root,const char *state_directory,
    const char *liveview_fifo,const T_DjiFlightControllerRidInfo *rid,bool enable_flight,
    E_DjiLiveViewCameraPosition position,E_DjiLiveViewCameraSource source);
T_DjiReturnCode M300Runtime_StartFromEnvironment(const T_DjiFlightControllerRidInfo *rid);
void M300Runtime_Stop(void);
#ifdef __cplusplus
}
#endif
#endif
