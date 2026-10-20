#define _XOPEN_SOURCE 700
#include "m300_runtime.h"
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
T_DjiReturnCode M300Runtime_StartFromEnvironment(const T_DjiFlightControllerRidInfo *rid)
{
    const char *configured=getenv("M300_RUNTIME_ROOT");
    char package[PATH_MAX],state_dir[PATH_MAX],fifo[PATH_MAX];
    if(!configured||!realpath(configured,package))return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    int a=snprintf(state_dir,sizeof state_dir,"%s/state",package);
    int b=snprintf(fifo,sizeof fifo,"%s/inputs/liveview.h264",package);
    if(a<0||b<0||(size_t)a>=sizeof state_dir||(size_t)b>=sizeof fifo)return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;
    const char *flight=getenv("M300_ALLOW_FLIGHT");
    const char *position=getenv("M300_CAMERA_POSITION");
    const char *source=getenv("M300_CAMERA_SOURCE");
    E_DjiLiveViewCameraPosition p=DJI_LIVEVIEW_CAMERA_POSITION_NO_1;
    E_DjiLiveViewCameraSource s=DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;
    /* Only the existing SDK's documented camera enum values are accepted. */
    if(position){if(!strcmp(position,"1"))p=DJI_LIVEVIEW_CAMERA_POSITION_NO_1;else if(!strcmp(position,"2"))p=DJI_LIVEVIEW_CAMERA_POSITION_NO_2;else if(!strcmp(position,"3"))p=DJI_LIVEVIEW_CAMERA_POSITION_NO_3;else if(!strcmp(position,"7"))p=DJI_LIVEVIEW_CAMERA_POSITION_FPV;else return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;}
    if(source){if(!strcmp(source,"0"))s=DJI_LIVEVIEW_CAMERA_SOURCE_DEFAULT;else if(!strcmp(source,"1"))s=DJI_LIVEVIEW_CAMERA_SOURCE_H20_WIDE;else if(!strcmp(source,"2"))s=DJI_LIVEVIEW_CAMERA_SOURCE_H20_ZOOM;else return DJI_ERROR_SYSTEM_MODULE_CODE_INVALID_PARAMETER;}
    return M300Runtime_Start(package,state_dir,fifo,rid,flight&&!strcmp(flight,"1"),p,s);
}
