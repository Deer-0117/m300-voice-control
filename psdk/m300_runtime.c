#define _POSIX_C_SOURCE 200809L
#include "m300_runtime.h"
#include "command_guard.h"
#include "dji_widget.h"
#include "dji_fc_subscription.h"
#include "cJSON.h"
#include <pthread.h>
#include <stdatomic.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#define OK DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS
#define FAIL DJI_ERROR_SYSTEM_MODULE_CODE_SYSTEM_ERROR
#define PI 3.14159265358979323846
#define ROOTLEN 2048
static char root[ROOTLEN],state[ROOTLEN],fifo_path[ROOTLEN],clock_token[80],boot[65];
static pthread_mutex_t lock=PTHREAD_MUTEX_INITIALIZER;
static pthread_t thread,ui_thread;
static atomic_bool running=false;
static int fifo_fd=-1;
static M300Permission permission;
static M300Command command;
static bool acquire_requested=false,release_requested=false;
static uint64_t audio_id=0,audio_size=0;
static uint32_t bitrate=0;
static FILE *audio=NULL;
static char audio_name[80];
static T_DjiWidgetSpeakerState speaker={DJI_WIDGET_SPEAKER_STATE_IDEL,DJI_WIDGET_SPEAKER_WORK_MODE_VOICE,DJI_WIDGET_SPEAKER_PLAY_MODE_SINGLE_PLAY,50};
static cJSON *pending=NULL;
static char reviewed_nonce[65]={0};
static uint64_t displayed_ms=0,event_seq=0,displayed_target=UINT64_MAX;
static E_DjiLiveViewCameraPosition camera_position;
static E_DjiLiveViewCameraSource camera_source;
static atomic_uint live_drops=0;
static uint64_t minimum_revision=0;
typedef struct {char kind[24];uint64_t revision,audio_id,time_ms,target_index;char session[65],hash[65],nonce[65],file[80],md5[33];uint32_t bitrate;} Event;
static Event events[64];static size_t head=0,tail=0;
static T_DjiFcSubscriptionVelocity velocity;
static T_DjiFcSubscriptionQuaternion quat;
static T_DjiFcSubscriptionAngularRateFusioned angular;
static T_DjiFcSubscriptionAltitudeFused altitude;
static T_DjiFcSubscriptionAltitudeOfHomePoint home_alt;
static T_DjiFcSubscriptionFlightStatus flight;
static T_DjiFcSubscriptionHomePointSetStatus home_set;
static uint64_t vel_ms=0,quat_ms=0,angular_ms=0,alt_ms=0,home_ms=0,flight_ms=0,home_set_ms=0;
static uint64_t mono(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return(uint64_t)t.tv_sec*1000+t.tv_nsec/1000000;}
static void copy(char *dst,size_t n,const char *s){snprintf(dst,n,"%s",s?s:"");}
static const cJSON *field(const cJSON *o,const char *k){return o?cJSON_GetObjectItem(o,k):NULL;}
static const char *str(const cJSON *o,const char *k){const cJSON *n=field(o,k);return n&&n->type==cJSON_String?n->valuestring:"";}
static double num(const cJSON *o,const char *k){const cJSON *n=field(o,k);return n&&n->type==cJSON_Number?n->valuedouble:NAN;}
static bool truth(const cJSON *o,const char *k){const cJSON *n=field(o,k);return n&&n->type==cJSON_True;}
static bool integer(const cJSON *o,const char *k,uint64_t *out){double v=num(o,k);if(!isfinite(v)||v<0||v>9007199254740991.||floor(v)!=v)return false;*out=(uint64_t)v;return true;}
static bool join(char *out,size_t n,const char *a,const char *b){int r=snprintf(out,n,"%s/%s",a,b);return r>=0&&(size_t)r<n;}
static cJSON *read_json(const char *name){char file[ROOTLEN],data[8192];if(!join(file,sizeof file,state,name))return NULL;FILE *f=fopen(file,"rb");if(!f)return NULL;size_t n=fread(data,1,sizeof(data)-1,f);bool big=!feof(f);fclose(f);if(big)return NULL;data[n]=0;return cJSON_Parse(data);}
static bool write_json(const char *relative,cJSON *value){char file[ROOTLEN],tmp[ROOTLEN];if(!join(file,sizeof file,state,relative))return false;int r=snprintf(tmp,sizeof tmp,"%s.tmp",file);if(r<0||(size_t)r>=sizeof tmp)return false;char *text=cJSON_PrintUnformatted(value);if(!text)return false;FILE *f=fopen(tmp,"wb");bool ok=false;if(f){ok=fwrite(text,1,strlen(text),f)==strlen(text);if(fclose(f)!=0)ok=false;if(ok)ok=rename(tmp,file)==0;}free(text);return ok;}
static void base_json(cJSON *o){cJSON_AddStringToObject(o,"clock_id",clock_token);cJSON_AddStringToObject(o,"bridge_boot",boot);cJSON_AddNumberToObject(o,"time_ms",(double)mono());}
static void invalidate(void){permission.paused=true;permission.revision=0;permission.plan_hash[0]=permission.nonce[0]=0;reviewed_nonce[0]=0;if(command.motion_id>permission.blocked_motion_id)permission.blocked_motion_id=command.motion_id;}
static void revoke_candidate(void){uint64_t rev=permission.revision;if(pending)integer(pending,"revision",&rev);if(rev>=minimum_revision)minimum_revision=rev+1;invalidate();}
static bool enqueue(const char *kind){size_t next=(tail+1)%64;if(next==head){invalidate();return false;}Event *e=&events[tail];memset(e,0,sizeof *e);copy(e->kind,sizeof e->kind,kind);e->time_ms=mono();e->revision=permission.revision;e->target_index=permission.target_index;copy(e->session,sizeof e->session,permission.session);if(pending){integer(pending,"revision",&e->revision);copy(e->hash,sizeof e->hash,str(pending,"plan_hash"));copy(e->nonce,sizeof e->nonce,str(pending,"nonce"));copy(e->session,sizeof e->session,str(pending,"session"));}tail=next;return true;}
#define TOPIC_CB(name,type,value,stamp) static T_DjiReturnCode name(const uint8_t *d,uint16_t n,const T_DjiDataTimestamp *t){(void)t;if(n!=sizeof(type))return FAIL;pthread_mutex_lock(&lock);memcpy(&value,d,n);stamp=mono();pthread_mutex_unlock(&lock);return OK;}
TOPIC_CB(on_velocity,T_DjiFcSubscriptionVelocity,velocity,vel_ms)
TOPIC_CB(on_quat,T_DjiFcSubscriptionQuaternion,quat,quat_ms)
TOPIC_CB(on_angular,T_DjiFcSubscriptionAngularRateFusioned,angular,angular_ms)
TOPIC_CB(on_alt,T_DjiFcSubscriptionAltitudeFused,altitude,alt_ms)
TOPIC_CB(on_home,T_DjiFcSubscriptionAltitudeOfHomePoint,home_alt,home_ms)
TOPIC_CB(on_flight,T_DjiFcSubscriptionFlightStatus,flight,flight_ms)
TOPIC_CB(on_home_set,T_DjiFcSubscriptionHomePointSetStatus,home_set,home_set_ms)
static T_DjiReturnCode authority_event(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo e){pthread_mutex_lock(&lock);permission.authority=e.curJoystickCtrlAuthority==DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK;if(!permission.authority){permission.prepared=false;invalidate();enqueue("release");}pthread_mutex_unlock(&lock);return OK;}
static void liveview(E_DjiLiveViewCameraPosition p,const uint8_t *data,uint32_t size){(void)p;if(fifo_fd<0)fifo_fd=open(fifo_path,O_WRONLY|O_NONBLOCK);if(fifo_fd>=0){ssize_t n=write(fifo_fd,data,size);if(n<0||n!=(ssize_t)size){live_drops++;if(n<0&&errno==EPIPE){close(fifo_fd);fifo_fd=-1;}}}}
static T_DjiReturnCode widget_set(E_DjiWidgetType type,uint32_t index,int32_t value,void *user)
{
    (void)user;if(type!=DJI_WIDGET_TYPE_BUTTON||index>=12)return FAIL;if(value!=DJI_WIDGET_BUTTON_STATE_PRESS_DOWN)return OK;
    pthread_mutex_lock(&lock);unsigned action=index%6;uint64_t now=mono();
    if(action==0){ /* Review the displayed plan before confirming it. */
        if(pending&&now-displayed_ms<=2000){copy(reviewed_nonce,sizeof reviewed_nonce,str(pending,"nonce"));acquire_requested=permission.enabled&&truth(pending,"flight_enabled");permission.prepared=true;}
        else {enqueue("prepare");invalidate();}
    }else if(action==1){
        if(pending&&now-displayed_ms<=2000&&reviewed_nonce[0]&&strcmp(reviewed_nonce,str(pending,"nonce"))==0){
            integer(pending,"revision",&permission.revision);integer(pending,"target_index",&permission.target_index);copy(permission.plan_hash,sizeof permission.plan_hash,str(pending,"plan_hash"));copy(permission.nonce,sizeof permission.nonce,str(pending,"nonce"));copy(permission.session,sizeof permission.session,str(pending,"session"));permission.paused=false;enqueue("confirm");
        }
    }else if(action==2){enqueue("reject");revoke_candidate();}
    else if(action==3){enqueue("pause");revoke_candidate();}
    else if(action==4){if(!permission.paused&&permission.revision&&pending==NULL&&displayed_target==permission.target_index&&now-displayed_ms<=2000){enqueue("complete");permission.target_index++;if(command.motion_id>permission.blocked_motion_id)permission.blocked_motion_id=command.motion_id;}}
    else {enqueue("release");revoke_candidate();release_requested=true;permission.prepared=false;}
    pthread_mutex_unlock(&lock);return OK;
}
static T_DjiReturnCode widget_get(E_DjiWidgetType type,uint32_t index,int32_t *value,void *user){(void)user;if(type!=DJI_WIDGET_TYPE_BUTTON||index>=12||!value)return FAIL;*value=DJI_WIDGET_BUTTON_STATE_RELEASE_UP;return OK;}
#define W(i) {i,DJI_WIDGET_TYPE_BUTTON,widget_set,widget_get,NULL}
static const T_DjiWidgetHandlerListItem handlers[]={W(0),W(1),W(2),W(3),W(4),W(5),W(6),W(7),W(8),W(9),W(10),W(11)};
static T_DjiReturnCode speaker_get(T_DjiWidgetSpeakerState *out){pthread_mutex_lock(&lock);*out=speaker;pthread_mutex_unlock(&lock);return OK;}
static T_DjiReturnCode speaker_mode(E_DjiWidgetSpeakerWorkMode m){pthread_mutex_lock(&lock);speaker.workMode=m;pthread_mutex_unlock(&lock);return OK;}
static T_DjiReturnCode speaker_playmode(E_DjiWidgetSpeakerPlayMode m){pthread_mutex_lock(&lock);speaker.playMode=m;pthread_mutex_unlock(&lock);return OK;}
static T_DjiReturnCode speaker_volume(uint8_t v){pthread_mutex_lock(&lock);speaker.volume=v;pthread_mutex_unlock(&lock);return OK;}
static T_DjiReturnCode speaker_play(void){return OK;}
static T_DjiReturnCode tts(E_DjiWidgetTransmitDataEvent e,uint32_t o,uint8_t *b,uint16_t n){(void)e;(void)o;(void)b;(void)n;return FAIL;}
static T_DjiReturnCode voice(E_DjiWidgetTransmitDataEvent e,uint32_t offset,uint8_t *buf,uint16_t size)
{
    pthread_mutex_lock(&lock);T_DjiReturnCode rc=OK;
    if(e==DJI_WIDGET_TRANSMIT_DATA_EVENT_START){
        revoke_candidate();if(audio)fclose(audio);audio=NULL;audio_size=0;
        if(size<sizeof(T_DjiWidgetTransDataContent)){rc=FAIL;goto done;}
        T_DjiWidgetTransDataContent info;memcpy(&info,buf,sizeof info);bitrate=info.transDataStartContent.fileDecodeBitrate;
        audio_id=mono();snprintf(audio_name,sizeof audio_name,"voice-%llu.opus",(unsigned long long)audio_id);char relative[128],file[ROOTLEN];snprintf(relative,sizeof relative,"ipc/audio/%s",audio_name);
        if(!join(file,sizeof file,state,relative)){rc=FAIL;goto done;}audio=fopen(file,"wb+");if(!audio){rc=FAIL;goto done;}
        speaker.state=DJI_WIDGET_SPEAKER_STATE_TRANSMITTING;
        if(enqueue("voice_start")){Event *last=&events[(tail+63)%64];last->audio_id=audio_id;}
    }else if(e==DJI_WIDGET_TRANSMIT_DATA_EVENT_TRANSMIT){
        if(!audio||(uint64_t)offset+size>2*1024*1024||fseek(audio,offset,SEEK_SET)!=0||fwrite(buf,1,size,audio)!=size){rc=FAIL;goto done;}
        if((uint64_t)offset+size>audio_size)audio_size=(uint64_t)offset+size;
    }else if(e==DJI_WIDGET_TRANSMIT_DATA_EVENT_FINISH){
        if(!audio||size!=16){rc=FAIL;goto done;}if(fclose(audio)!=0){audio=NULL;rc=FAIL;goto done;}audio=NULL;speaker.state=DJI_WIDGET_SPEAKER_STATE_IDEL;
        if(enqueue("voice_finish")){Event *last=&events[(tail+63)%64];last->audio_id=audio_id;last->bitrate=bitrate;copy(last->file,sizeof last->file,audio_name);for(int i=0;i<16;i++)snprintf(last->md5+2*i,3,"%02x",buf[i]);}
    }else if(e==DJI_WIDGET_TRANSMIT_DATA_EVENT_ABORT){if(audio)fclose(audio);audio=NULL;speaker.state=DJI_WIDGET_SPEAKER_STATE_IDEL;enqueue("voice_abort");invalidate();}
 done:if(rc!=OK){if(audio)fclose(audio);audio=NULL;speaker.state=DJI_WIDGET_SPEAKER_STATE_ERROR;enqueue("voice_abort");invalidate();}pthread_mutex_unlock(&lock);return rc;
}
static const T_DjiWidgetSpeakerHandler speaker_handler={speaker_get,speaker_mode,speaker_playmode,speaker_volume,speaker_play,speaker_play,tts,voice};
static void flush_events(void)
{
    while(1){Event e;pthread_mutex_lock(&lock);if(head==tail){pthread_mutex_unlock(&lock);break;}e=events[head];head=(head+1)%64;pthread_mutex_unlock(&lock);
        cJSON *o=cJSON_CreateObject();base_json(o);cJSON_GetObjectItem(o,"time_ms")->valuedouble=(double)e.time_ms;char id[80],file[128];snprintf(id,sizeof id,"%s-%012llu",boot,(unsigned long long)++event_seq);snprintf(file,sizeof file,"ipc/events/%012llu.json",(unsigned long long)event_seq);
        cJSON_AddStringToObject(o,"id",id);cJSON_AddStringToObject(o,"kind",e.kind);cJSON_AddNumberToObject(o,"revision",(double)e.revision);cJSON_AddNumberToObject(o,"target_index",(double)e.target_index);cJSON_AddStringToObject(o,"session",e.session);cJSON_AddStringToObject(o,"plan_hash",e.hash);cJSON_AddStringToObject(o,"nonce",e.nonce);cJSON_AddNumberToObject(o,"audio_id",(double)e.audio_id);cJSON_AddStringToObject(o,"file",e.file);cJSON_AddNumberToObject(o,"bitrate",e.bitrate);cJSON_AddStringToObject(o,"md5",e.md5);
        if(!write_json(file,o)){pthread_mutex_lock(&lock);invalidate();pthread_mutex_unlock(&lock);}cJSON_Delete(o);
    }
}
static void update_ui(void)
{
    cJSON *o=read_json("ipc/status.json");uint64_t stamp=0,rev=0;uint64_t now=mono();bool valid=o&&integer(o,"time_ms",&stamp)&&stamp<=now&&now-stamp<1000&&strcmp(str(o,"clock_id"),clock_token)==0&&integer(o,"revision",&rev);
    pthread_mutex_lock(&lock);
    if(!valid){if(pending){cJSON_Delete(pending);pending=NULL;}displayed_ms=0;invalidate();pthread_mutex_unlock(&lock);if(o)cJSON_Delete(o);return;}
    if(strcmp(str(o,"session"),permission.session)!=0&&permission.session[0]){invalidate();minimum_revision=0;permission.session[0]=0;}
    bool awaiting=strcmp(str(o,"status"),"awaiting_confirmation")==0&&rev>=minimum_revision;
    if(awaiting&&(!pending||strcmp(str(o,"nonce"),str(pending,"nonce")))){invalidate();if(pending)cJSON_Delete(pending);pending=NULL;}
    const char *summary=str(o,"summary");bool shown=false;
    pthread_mutex_unlock(&lock);
    if(strlen(summary)>0&&strlen(summary)<DJI_WIDGET_FLOATING_WINDOW_MSG_MAX_LEN)shown=DjiWidgetFloatingWindow_ShowMessage(summary)==OK;
    pthread_mutex_lock(&lock);
    if(shown&&awaiting){if(pending)cJSON_Delete(pending);pending=o;o=NULL;displayed_ms=now;}
    else if(!awaiting){if(shown&&strcmp(str(o,"status"),"active")==0){integer(o,"target_index",&displayed_target);displayed_ms=now;}if(pending)cJSON_Delete(pending);pending=NULL;if(strcmp(str(o,"status"),"active")!=0)invalidate();}
    pthread_mutex_unlock(&lock);if(o)cJSON_Delete(o);
}
static void read_command(void)
{
    cJSON *o=read_json("ipc/command.json");M300Command c={0};bool good=o&&integer(o,"sent_ms",&c.sent_ms)&&integer(o,"deadline_ms",&c.deadline_ms)&&integer(o,"revision",&c.revision)&&integer(o,"motion_id",&c.motion_id)&&integer(o,"target_index",&c.target_index);
    if(good){const cJSON *v=field(o,"velocity");c.x=num(v,"x");c.y=num(v,"y");c.z=num(v,"z");c.yaw=num(v,"yaw");c.enabled=truth(o,"enabled");copy(c.session,sizeof c.session,str(o,"session"));copy(c.bridge_boot,sizeof c.bridge_boot,str(o,"bridge_boot"));copy(c.clock_id,sizeof c.clock_id,str(o,"clock_id"));copy(c.plan_hash,sizeof c.plan_hash,str(o,"plan_hash"));copy(c.nonce,sizeof c.nonce,str(o,"nonce"));}
    pthread_mutex_lock(&lock);command=c;pthread_mutex_unlock(&lock);if(o)cJSON_Delete(o);
}
static void tick(void)
{
    uint64_t now=mono();pthread_mutex_lock(&lock);
    bool acquire=acquire_requested,release=release_requested;acquire_requested=release_requested=false;
    bool kinematics_fresh=vel_ms&&now-vel_ms<200&&quat_ms&&now-quat_ms<200&&angular_ms&&now-angular_ms<200&&alt_ms&&now-alt_ms<200;
    permission.fresh=kinematics_fresh&&home_ms&&now-home_ms<3000&&flight_ms&&now-flight_ms<500&&home_set_ms&&now-home_set_ms<3000&&home_set==DJI_FC_SUBSCRIPTION_HOME_POINT_SET_STATUS_SUCCESS;
    permission.in_air=flight==DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR;permission.velocity_healthy=velocity.health==0&&isfinite(velocity.data.x)&&isfinite(velocity.data.y)&&isfinite(velocity.data.z);permission.altitude_m=altitude-home_alt;
    bool enabled=permission.enabled,authority=permission.authority;
    pthread_mutex_unlock(&lock);
    if(acquire&&enabled&&!authority){T_DjiReturnCode rc=DjiFlightController_ObtainJoystickCtrlAuthority();pthread_mutex_lock(&lock);permission.authority=rc==OK;permission.prepared=rc==OK;pthread_mutex_unlock(&lock);}
    if(release&&authority&&enabled){T_DjiFlightControllerJoystickCommand z={0};DjiFlightController_ExecuteJoystickAction(z);DjiFlightController_ReleaseJoystickCtrlAuthority();pthread_mutex_lock(&lock);permission.authority=false;pthread_mutex_unlock(&lock);}
    pthread_mutex_lock(&lock);bool allowed=M300_CommandAllowed(&command,&permission,now);
    T_DjiFlightControllerJoystickCommand v={0};if(allowed){v.x=(float)command.x;v.y=(float)command.y;v.z=(float)command.z;v.yaw=(float)command.yaw;}
    bool send=permission.enabled&&permission.authority;
    cJSON *t=cJSON_CreateObject();base_json(t);cJSON_AddBoolToObject(t,"fresh",permission.fresh);cJSON_AddBoolToObject(t,"kinematics_fresh",kinematics_fresh);cJSON_AddBoolToObject(t,"in_air",permission.in_air);cJSON_AddBoolToObject(t,"velocity_healthy",permission.velocity_healthy);cJSON_AddBoolToObject(t,"has_authority",permission.authority);cJSON_AddBoolToObject(t,"prepared",permission.prepared);cJSON_AddNumberToObject(t,"altitude_m",permission.altitude_m);
    cJSON *vel=cJSON_CreateArray();cJSON_AddItemToArray(vel,cJSON_CreateNumber(velocity.data.x));cJSON_AddItemToArray(vel,cJSON_CreateNumber(velocity.data.y));cJSON_AddItemToArray(vel,cJSON_CreateNumber(velocity.data.z));cJSON_AddItemToObject(t,"velocity_neu",vel);
    double yaw=atan2(2.*(quat.q0*quat.q3+quat.q1*quat.q2),1.-2.*(quat.q2*quat.q2+quat.q3*quat.q3))*180./PI;
    cJSON_AddNumberToObject(t,"yaw_deg",yaw);cJSON_AddNumberToObject(t,"yaw_rate_deg_s",angular.z*180./PI);cJSON_AddNumberToObject(t,"liveview_dropped_chunks",atomic_load(&live_drops));cJSON_AddBoolToObject(t,"segment_allowed",allowed);cJSON_AddNumberToObject(t,"applied_motion_id",allowed?(double)command.motion_id:0);cJSON *applied=cJSON_CreateObject();cJSON_AddNumberToObject(applied,"x",v.x);cJSON_AddNumberToObject(applied,"y",v.y);cJSON_AddNumberToObject(applied,"z",v.z);cJSON_AddNumberToObject(applied,"yaw",v.yaw);cJSON_AddItemToObject(t,"applied_velocity",applied);pthread_mutex_unlock(&lock);
    T_DjiReturnCode send_result=OK;if(send)send_result=DjiFlightController_ExecuteJoystickAction(v);cJSON_AddBoolToObject(t,"sdk_command_called",send);cJSON_AddNumberToObject(t,"sdk_result",(double)send_result);if(send_result!=OK){pthread_mutex_lock(&lock);invalidate();pthread_mutex_unlock(&lock);}
    write_json("ipc/telemetry.json",t);cJSON_Delete(t);
}
static void *worker(void *p){(void)p;while(atomic_load(&running)){uint64_t start=mono();read_command();tick();flush_events();uint64_t spent=mono()-start;struct timespec wait={0,(long)(spent<50?50-spent:1)*1000000};nanosleep(&wait,NULL);}return NULL;}
static void *ui_worker(void *p){(void)p;while(atomic_load(&running)){update_ui();DjiLiveview_RequestIntraframeFrameData(camera_position,camera_source);struct timespec wait={0,500000000};nanosleep(&wait,NULL);}return NULL;}
#define SUB(topic,freq,callback) do{T_DjiReturnCode rc=DjiFcSubscription_SubscribeTopic(topic,freq,callback);if(rc!=OK)return rc;}while(0)
T_DjiReturnCode M300Runtime_Start(const char *package_root,const char *state_directory,const char *liveview_fifo,const T_DjiFlightControllerRidInfo *rid,bool enable_flight,E_DjiLiveViewCameraPosition position,E_DjiLiveViewCameraSource source)
{
    if(atomic_load(&running)||!rid||!package_root||!state_directory||!liveview_fifo)return FAIL;
    if(strlen(package_root)>=sizeof root||strlen(state_directory)>=sizeof state||strlen(liveview_fifo)>=sizeof fifo_path)return FAIL;
    copy(root,sizeof root,package_root);copy(state,sizeof state,state_directory);copy(fifo_path,sizeof fifo_path,liveview_fifo);
    FILE *f=fopen("/proc/sys/kernel/random/boot_id","r");char id[40]={0};if(!f)return FAIL;if(!fgets(id,sizeof id,f)){fclose(f);return FAIL;}fclose(f);id[strcspn(id,"\r\n")]=0;if(strlen(id)!=36)return FAIL;snprintf(clock_token,sizeof clock_token,"mono-v1:%s",id);snprintf(boot,sizeof boot,"%s-%llu",id,(unsigned long long)mono());
    copy(permission.clock_id,sizeof permission.clock_id,clock_token);copy(permission.bridge_boot,sizeof permission.bridge_boot,boot);permission.enabled=enable_flight;permission.paused=true;
    signal(SIGPIPE,SIG_IGN);
    T_DjiReturnCode rc=DjiFlightController_Init(*rid);if(rc!=OK)return rc;
    T_DjiFlightControllerJoystickMode mode={.horizontalControlMode=DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,.verticalControlMode=DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,.yawControlMode=DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,.horizontalCoordinate=DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,.stableControlMode=DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
    DjiFlightController_SetJoystickMode(mode);rc=DjiFlightController_RegJoystickCtrlAuthorityEventCallback(authority_event);if(rc!=OK)return rc;
    rc=DjiFcSubscription_Init();if(rc!=OK)return rc;
    SUB(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,on_velocity);SUB(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,on_quat);SUB(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,on_angular);SUB(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,on_alt);SUB(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,on_home);SUB(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_SET_STATUS,DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,on_home_set);SUB(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,on_flight);
    rc=DjiWidget_Init();if(rc!=OK)return rc;char ui[ROOTLEN];if(!join(ui,sizeof ui,root,"psdk/widget/cn_big_screen"))return FAIL;
    rc=DjiWidget_RegDefaultUiConfigByDirPath(ui);if(rc!=OK)return rc;rc=DjiWidget_RegHandlerList(handlers,sizeof handlers/sizeof handlers[0]);if(rc!=OK)return rc;rc=DjiWidget_RegSpeakerHandler(&speaker_handler);if(rc!=OK)return rc;
    rc=DjiLiveview_Init();if(rc!=OK)return rc;camera_position=position;camera_source=source;rc=DjiLiveview_StartH264Stream(position,source,liveview);if(rc!=OK)return rc;
    atomic_store(&running,true);if(pthread_create(&thread,NULL,worker,NULL)!=0){atomic_store(&running,false);return FAIL;}if(pthread_create(&ui_thread,NULL,ui_worker,NULL)!=0){atomic_store(&running,false);pthread_join(thread,NULL);return FAIL;}return OK;
}
void M300Runtime_Stop(void){if(!atomic_exchange(&running,false))return;pthread_join(thread,NULL);pthread_join(ui_thread,NULL);if(permission.enabled&&permission.authority){T_DjiFlightControllerJoystickCommand z={0};DjiFlightController_ExecuteJoystickAction(z);DjiFlightController_ReleaseJoystickCtrlAuthority();}DjiLiveview_StopH264Stream(camera_position,camera_source);if(fifo_fd>=0){close(fifo_fd);fifo_fd=-1;}pthread_mutex_lock(&lock);if(audio){fclose(audio);audio=NULL;}if(pending){cJSON_Delete(pending);pending=NULL;}invalidate();pthread_mutex_unlock(&lock);}
