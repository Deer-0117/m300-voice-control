import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('host_staging',ROOT/'migration/prepare-host.py')
staging=importlib.util.module_from_spec(spec)
spec.loader.exec_module(staging)

FIXTURE='''// private source fixture, never a real SDK account
#ifdef CONFIG_MODULE_SAMPLE_CAMERA_EMU_ON
#include "camera.h"
#endif
void startup() {
    prepare_account();
    DjiCore_Init(&userInfo);
    returnCode = DjiCore_SetSerialNumber("existing-serial");
#ifdef CONFIG_MODULE_SAMPLE_CAMERA_EMU_ON
    start_camera_example();
#endif
#ifdef CONFIG_MODULE_SAMPLE_WIDGET_ON
    DjiTest_WidgetStartService();
#endif
#ifdef CONFIG_MODULE_SAMPLE_WIDGET_SPEAKER_ON
    DjiTest_WidgetSpeakerStartService();
#endif
    returnCode = BalloonWidget_StartService("original-path", "arm", "trigger", "stop");
    check_widget_error();
    returnCode = DjiCore_ApplicationStart();
    check_core_error();
    returnCode = BalloonChaseProbe_RunNoFlightProbe();
    check_probe_error();
    USER_LOG_INFO("Application start.");
}
void untouched_account_loader() { preserve_real_device_configuration(); }
'''

class HostStagingTests(unittest.TestCase):
    def test_only_expected_business_blocks_disabled(self):
        result=staging.patch_application(FIXTURE)
        self.assertEqual(result.count('#if 0 /* MIGRATION:'),2)
        self.assertIn('DjiCore_Init(&userInfo);',result)
        self.assertIn('returnCode = DjiCore_ApplicationStart();',result)
        self.assertIn('void untouched_account_loader() { preserve_real_device_configuration(); }',result)
        self.assertIn('#include "camera.h"',result)
        self.assertLess(result.index('#endif /* MIGRATION old business modules */'),result.index('returnCode = DjiCore_ApplicationStart();'))

    def test_missing_anchor_fails_closed(self):
        with self.assertRaises(ValueError):
            staging.patch_application(FIXTURE.replace('BalloonWidget_StartService','renamed_widget'))

    def test_duplicate_core_start_fails_closed(self):
        with self.assertRaises(ValueError):
            staging.patch_application(FIXTURE+'\nDjiCore_ApplicationStart();\n')

    def test_changed_legacy_widget_layout_fails_closed(self):
        with self.assertRaises(ValueError):
            staging.patch_application(FIXTURE.replace('DjiTest_WidgetSpeakerStartService();',''))

    def test_application_copy_excludes_scripts_logs_and_build_backups(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'source';source.mkdir()
            (source/'application.cpp').write_text(FIXTURE)
            (source/'application.hpp').write_text('Application(int argc, char **argv);')
            (source/'config.json').write_text('{"fixture":true}')
            (source/'discard.log').write_text('not source')
            (source/'worker.py').write_text('not host source')
            (source/'build-old').mkdir();(source/'build-old/leak.cpp').write_text('not source')
            target=root/'target'
            staging.copy_application(source,target)
            self.assertTrue((target/'config.json').is_file())
            self.assertEqual((target/'application.cpp').read_text(),FIXTURE)
            self.assertFalse((target/'discard.log').exists())
            self.assertFalse((target/'worker.py').exists())
            self.assertFalse((target/'build-old').exists())

    def test_host_build_info_is_before_sdk_construction(self):
        text=(ROOT/'migration/host/migration_main.cpp').read_text()
        self.assertLess(text.index('"--build-info"'),text.index('Application application('))
        self.assertIn('setenv("M300_ALLOW_FLIGHT", "0", 1)',text)
        cmake=(ROOT/'migration/host/CMakeLists.txt').read_text()
        self.assertIn('option(M300_GROUND_ONLY',cmake)
        self.assertIn('Flight build is not released yet',cmake)
