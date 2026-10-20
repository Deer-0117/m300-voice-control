/* Test-only SDK replacements. Production never links this file. */
#include "../psdk/m300_runtime.c"
#include <assert.h>
static unsigned sent_count=0,obtain_count=0;
T_DjiReturnCode DjiFlightController_Init(T_DjiFlightControllerRidInfo x){(void)x;return OK;}
T_DjiReturnCode DjiFlightController_RegJoystickCtrlAuthorityEventCallback(JoystickCtrlAuthorityEventCbFunc x){(void)x;return OK;}
void DjiFlightController_SetJoystickMode(T_DjiFlightControllerJoystickMode x){(void)x;}
T_DjiReturnCode DjiFlightController_ObtainJoystickCtrlAuthority(void){obtain_count++;return OK;}
T_DjiReturnCode DjiFlightController_ReleaseJoystickCtrlAuthority(void){return OK;}
T_DjiReturnCode DjiFlightController_ExecuteJoystickAction(T_DjiFlightControllerJoystickCommand x){(void)x;sent_count++;return OK;}
T_DjiReturnCode DjiFcSubscription_Init(void){return OK;}
T_DjiReturnCode DjiFcSubscription_SubscribeTopic(E_DjiFcSubscriptionTopic a,E_DjiDataSubscriptionTopicFreq b,DjiReceiveDataOfTopicCallback c){(void)a;(void)b;(void)c;return OK;}
T_DjiReturnCode DjiWidget_Init(void){return OK;}
T_DjiReturnCode DjiWidget_RegDefaultUiConfigByDirPath(const char *x){(void)x;return OK;}
T_DjiReturnCode DjiWidget_RegHandlerList(const T_DjiWidgetHandlerListItem *x,uint32_t n){(void)x;(void)n;return OK;}
T_DjiReturnCode DjiWidget_RegSpeakerHandler(const T_DjiWidgetSpeakerHandler *x){(void)x;return OK;}
T_DjiReturnCode DjiWidgetFloatingWindow_ShowMessage(const char *x){assert(strlen(x)<255);return OK;}
T_DjiReturnCode DjiLiveview_Init(void){return OK;}
T_DjiReturnCode DjiLiveview_StartH264Stream(E_DjiLiveViewCameraPosition p,E_DjiLiveViewCameraSource s,DjiLiveview_H264Callback cb){(void)p;(void)s;(void)cb;return OK;}
T_DjiReturnCode DjiLiveview_StopH264Stream(E_DjiLiveViewCameraPosition p,E_DjiLiveViewCameraSource s){(void)p;(void)s;return OK;}
T_DjiReturnCode DjiLiveview_RequestIntraframeFrameData(E_DjiLiveViewCameraPosition p,E_DjiLiveViewCameraSource s){(void)p;(void)s;return OK;}
static void candidate(int revision,const char *nonce){
 cJSON *o=cJSON_CreateObject();cJSON_AddStringToObject(o,"clock_id",clock_token);cJSON_AddStringToObject(o,"session","python-session");cJSON_AddNumberToObject(o,"revision",revision);cJSON_AddNumberToObject(o,"target_index",0);cJSON_AddNumberToObject(o,"time_ms",(double)mono());cJSON_AddStringToObject(o,"status","awaiting_confirmation");cJSON_AddStringToObject(o,"nonce",nonce);cJSON_AddStringToObject(o,"plan_hash","hash");cJSON_AddStringToObject(o,"summary","Plan red, then green. Confirm this version.");assert(write_json("ipc/status.json",o));cJSON_Delete(o);update_ui();
}
int main(int argc,char **argv){
 assert(argc==2);copy(state,sizeof state,argv[1]);copy(clock_token,sizeof clock_token,"test-clock");copy(boot,sizeof boot,"test-bridge");permission.paused=true;
 candidate(1,"one");widget_set(DJI_WIDGET_TYPE_BUTTON,1,1,NULL);assert(permission.revision==0); /* confirm without review blocked */
 widget_set(DJI_WIDGET_TYPE_BUTTON,0,1,NULL);widget_set(DJI_WIDGET_TYPE_BUTTON,1,1,NULL);assert(permission.revision==1&&!permission.paused);
 widget_set(DJI_WIDGET_TYPE_BUTTON,3,1,NULL);assert(permission.paused);candidate(1,"one");assert(pending==NULL); /* old UI cannot reauthorize */
 candidate(2,"two");widget_set(DJI_WIDGET_TYPE_BUTTON,0,1,NULL);candidate(3,"three");widget_set(DJI_WIDGET_TYPE_BUTTON,1,1,NULL);assert(permission.revision==0); /* candidate changed since review */
 widget_set(DJI_WIDGET_TYPE_BUTTON,0,1,NULL);widget_set(DJI_WIDGET_TYPE_BUTTON,1,1,NULL);assert(permission.revision==3);
 widget_set(DJI_WIDGET_TYPE_BUTTON,2,1,NULL);assert(permission.paused); /* reject */
 candidate(4,"four");widget_set(DJI_WIDGET_TYPE_BUTTON,0,1,NULL);widget_set(DJI_WIDGET_TYPE_BUTTON,1,1,NULL);cJSON_Delete(pending);pending=NULL;displayed_target=permission.target_index;displayed_ms=mono();command.motion_id=19;widget_set(DJI_WIDGET_TYPE_BUTTON,4,1,NULL);assert(permission.blocked_motion_id==19&&permission.target_index==1); /* complete stops the segment */
 int32_t value=99;widget_get(DJI_WIDGET_TYPE_BUTTON,0,&value,NULL);assert(value==0);
 permission.enabled=false;permission.authority=true;acquire_requested=true;tick();assert(sent_count==0&&obtain_count==0); /* observation mode never sends */
 if(pending)cJSON_Delete(pending);return 0;
}
