"""Run actual application startup/mission guards with deterministic substitutes."""
from pathlib import Path
import os
import re
import resource
import shlex
import subprocess
import tempfile
import unittest

import test_flight_panel_widget as panel

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / 'psdk-overlay/samples/sample_c++/platform/linux/manifold2/application/application.cpp'
MAIN = APP.with_name('main.cpp')

CPP_PRELUDE = r'''
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <stdexcept>
#include <cstdio>
typedef int T_DjiReturnCode;
struct T_DjiUserInfo { int ignored; };
struct T_DjiAircraftInfoBaseInfo { int mountPosition; };
struct T_DjiFirmwareVersion { int majorVersion,minorVersion,modifyVersion,debugVersion; };
struct T_DjiTestApplyHighPowerHandler { int (*pinInit)(); int (*pinWrite)(); };
#define DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS 0
#define DJI_MOUNT_POSITION_EXTENSION_PORT 1
#define DJI_MOUNT_POSITION_EXTENSION_LITE_PORT 2
#define DJI_USE_SDK_CONFIG_BY_JSON 1
#define CONFIG_MODULE_SAMPLE_CAMERA_EMU_ON
#define CONFIG_MODULE_SAMPLE_CAMERA_MEDIA_ON
#define CONFIG_MODULE_SAMPLE_GIMBAL_EMU_ON
#define CONFIG_MODULE_SAMPLE_WIDGET_ON
#define CONFIG_MODULE_SAMPLE_WIDGET_SPEAKER_ON
#define CONFIG_MODULE_SAMPLE_POWER_MANAGEMENT_ON
#define CONFIG_MODULE_SAMPLE_DATA_TRANSMISSION_ON
static int forbidden, core_init, core_start, widget_start, success_logs, fail_stage;
#define USER_LOG_ERROR(...) ((void)0)
#define USER_LOG_INFO(...) ((void)++success_logs)
static void DjiUser_NormalExitHandler(int) {}
static int sleep(int) { return 0; }
static void DjiUserConfigManager_GetAppInfo(T_DjiUserInfo *) {}
static int DjiCore_Init(T_DjiUserInfo *) { ++core_init; return 0; }
static int DjiAircraftInfo_GetBaseInfo(T_DjiAircraftInfoBaseInfo *p) { p->mountPosition=1;return 0; }
static int DjiCore_SetAlias(const char *) { return 0; }
static int DjiCore_SetFirmwareVersion(T_DjiFirmwareVersion) { return 0; }
static int DjiCore_SetSerialNumber(const char *) { return 0; }
static int DjiTest_CameraEmuBaseStartService() { ++forbidden;return 0; }
static int DjiTest_CameraEmuMediaStartService() { ++forbidden;return 0; }
static int DjiTest_GimbalStartService() { ++forbidden;return 0; }
static int DjiTest_WidgetStartService() { ++forbidden;return 0; }
static int DjiTest_WidgetSpeakerStartService() { ++forbidden;return 0; }
static int DjiTest_HighPowerApplyPinInit() { ++forbidden;return 0; }
static int DjiTest_WriteHighPowerApplyPin() { ++forbidden;return 0; }
static int DjiTest_RegApplyHighPowerHandler(T_DjiTestApplyHighPowerHandler *) { ++forbidden;return 0; }
static int DjiTest_PowerManagementStartService() { ++forbidden;return 0; }
static int DjiTest_DataTransmissionStartService() { ++forbidden;return 0; }
static int BalloonChaseProbe_RunNoFlightProbe() { ++forbidden;return 0; }
static int BalloonWidget_StartService(const char *,const char *,const char *,const char *) { ++widget_start;return fail_stage==1?1:0; }
static int DjiCore_ApplicationStart() { ++core_start;return fail_stage==2?1:0; }
'''
CPP_EPILOGUE = r'''
int main(int argc,char **argv) {
    assert(argc==3);
    const bool display = !strcmp(argv[1],"display");
    setenv("BALLOON_PANEL_DISPLAY_ONLY",display?"1":"0",1);
    fail_stage=atoi(argv[2]); bool failed=false;
    try { actual_start(); } catch(const std::runtime_error &) { failed=true; }
    assert(core_init==1 && widget_start==1);
    assert(failed==(fail_stage!=0));
    assert(success_logs==(fail_stage==0?1:0));
    assert(core_start==(fail_stage==1?0:1));
    assert(forbidden==0);
    return 0;
}
'''

MISSION_EXTRA = r'''
#define BALLOON_MISSION_STATUS "status"
#define BALLOON_NL_STOP_FILE "nl-stop"
#define BALLOON_LATEST_TARGET_FILE "target"
#define BALLOON_DETECTION_REQUEST_FILE "request"
#define BALLOON_MISSION_TOKEN "mission-token"
#define BALLOON_EVIDENCE_TOKEN "evidence-token"
typedef struct { unsigned int epoch; bool record,kalman; const char *mode; char voiceSession[96],token[96]; } T_BalloonMissionTask;
static bool BalloonPanel_VoiceReady(void) { return true; }
static bool BalloonWidget_UsesLiveview(void) { return true; }
static bool BalloonClock_Id(char id[80]) { strcpy(id,"mono-v1:11111111-1111-4111-8111-111111111111");return true; }
static void RemoveFileIfExists(const char *p) { (void)p;assert(false); }
static bool WriteStateFile(const char *p,const char *s) { (void)p;(void)s;assert(false);return false; }
static bool BalloonWidget_WritePlain(const char *p,const char *s) { (void)p;(void)s;assert(false);return false; }
static void *BalloonWidget_MissionTask(void *p) { (void)p;assert(false);return NULL; }
static void BalloonWidget_ShowStep(const char *p,const char *s) { (void)p;(void)s;assert(false); }
'''
MISSION_MAIN = r'''
int main(void) {
    const char *modes[]={"any","voice","api_new"};
    s_displayOnly=true; s_recordEnabled=true;
    for(unsigned i=0;i<3;i++) {
        assert(BalloonWidget_BeginMission(modes[i])==DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE);
        assert(strstr(last_message,"仅显示模式") && strstr(last_message,"未启动"));
    }
    assert(!thread_calls && !child_calls && !cleared_gate && !s_isArmed && !s_missionBusy);
    assert(s_actionEpoch==0); return 0;
}
'''

class DisplayMode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='uav-display-stubs-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.folder = Path(cls.temp.name)
        app = APP.read_text()
        start = app.index('void Application::DjiUser_ApplicationStart()')
        end = app.index('\n}\n', start) + 3
        startup = app[start:end].replace('Application::DjiUser_ApplicationStart', 'actual_start')
        cpp = cls.folder/'application.cpp'
        cpp.write_text(CPP_PRELUDE + startup + CPP_EPILOGUE)
        cls.app_binary = cls.folder/'application'
        cls.compile(shlex.split(os.environ.get('CXX','c++')) + ['-std=gnu++17',str(cpp),'-o',str(cls.app_binary)])
        prelude = panel.PRELUDE
        prelude, removed = re.subn(r'static T_DjiReturnCode BalloonWidget_BeginMission\(const char \*mode\) \{.*?\n\}', '', prelude, flags=re.DOTALL)
        assert removed == 1
        widget = (panel.WIDGET/'psdk_balloon_widget.c').read_text()
        c = cls.folder/'mission.c'
        c.write_text(prelude + MISSION_EXTRA + panel.function(widget,'BalloonWidget_HelperBusy') +
                     panel.function(widget,'BalloonWidget_BeginMission') + MISSION_MAIN)
        cls.mission_binary = cls.folder/'mission'
        cls.compile(shlex.split(os.environ.get('CC','cc')) + ['-std=gnu11','-pthread',str(c),'-o',str(cls.mission_binary)])

    @staticmethod
    def compile(cmd):
        result = subprocess.run(cmd,capture_output=True,text=True)
        if result.returncode:
            raise AssertionError(result.stderr)

    def test_actual_application_excludes_every_optional_sample_and_probe(self):
        for mode in ('display','normal'):
            for failure in range(3):
                with self.subTest(mode=mode,failure=failure):
                    result = subprocess.run([str(self.app_binary),mode,str(failure)],cwd=self.folder,
                                            capture_output=True,text=True,timeout=5)
                    self.assertEqual(result.returncode,0,result.stderr)

    def test_actual_mission_guard_rejects_all_three_start_modes_before_side_effects(self):
        result = subprocess.run([str(self.mission_binary)],cwd=self.folder,capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)


MAIN_APPLICATION_STUB = r"""
#ifndef TEST_APPLICATION_HPP
#define TEST_APPLICATION_HPP
#include <cstring>
#include <stdexcept>
#include <cstdio>
class Application {
public:
    Application(int argc,char **argv) {
        if(argc>1 && !std::strcmp(argv[1],"core-offline"))
            throw std::runtime_error("Core init error (offline fixture)");
        if(argc>1 && !std::strcmp(argv[1],"unknown-error")) throw 7;
        std::fputs("fixture_application_ready\n",stdout);
    }
};
#endif
"""
MAIN_SLEEP_STUB = r"""
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
unsigned int sleep(unsigned int seconds) {
    if(seconds!=1) std::exit(78);
    std::fputs("fixture_idle_wait\n",stdout);
    std::fflush(stdout);
    std::exit(77);
}
"""

class ActualMainLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix='uav-main-stubs-')
        cls.addClassCleanup(cls.temp.cleanup)
        cls.folder = Path(cls.temp.name)
        # Compile the complete production main unchanged, replacing only its
        # Application dependency and the OS sleep call. No SDK is linked.
        (cls.folder/'application.hpp').write_text(MAIN_APPLICATION_STUB)
        main = cls.folder/'main.cpp'
        main.write_text(MAIN.read_text())
        sleep = cls.folder/'sleep.cpp'
        sleep.write_text(MAIN_SLEEP_STUB)
        cls.binary = cls.folder/'main-lifecycle'
        DisplayMode.compile(shlex.split(os.environ.get('CXX','c++')) +
                            ['-std=gnu++17',str(main),str(sleep),'-o',str(cls.binary)])

    def test_startup_exceptions_exit_failure_without_abort_or_idle(self):
        for failure in ('core-offline','unknown-error'):
            with self.subTest(failure=failure):
                result = subprocess.run([str(self.binary),failure],cwd=self.folder,
                                        capture_output=True,text=True,timeout=5,
                                        preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE,(0,0)))
                self.assertEqual(result.returncode,1,result.stderr)
                self.assertNotIn('fixture_idle_wait',result.stdout)
                self.assertNotIn('fixture_application_ready',result.stdout)

    def test_successful_startup_only_enters_idle_wait(self):
        result = subprocess.run([str(self.binary),'success'],cwd=self.folder,
                                capture_output=True,text=True,timeout=5)
        self.assertEqual(result.returncode,77,result.stderr)
        self.assertEqual(result.stdout,'fixture_application_ready\nfixture_idle_wait\n')

if __name__=='__main__':
    unittest.main()
