"""Exercise the real adapter with SDK substitutes; no DJI library/device I/O."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / 'psdk-overlay/samples/sample_c/module_sample/target_chase'

MAIN = r'''
int main(void) {
    /* A process that never enables panel management retains the old contract. */
    assert(s_panel_maintenance_mode == -1);
    unsetenv("BALLOON_NO_FLIGHT"); unsetenv("BALLOON_GIMBAL_TRACKING");
    assert(!DjiTargetChase_RequestedNoFlight());
    assert(DjiTargetChase_RequestedGimbalTracking());
    setenv("BALLOON_NO_FLIGHT", "1", 1);
    setenv("BALLOON_GIMBAL_TRACKING", "0", 1);
    assert(DjiTargetChase_RequestedNoFlight());
    assert(!DjiTargetChase_RequestedGimbalTracking());
    setenv("BALLOON_NO_FLIGHT", "false", 1);
    setenv("BALLOON_GIMBAL_TRACKING", "true", 1);
    assert(!DjiTargetChase_RequestedNoFlight());
    assert(DjiTargetChase_RequestedGimbalTracking());
    setenv("BALLOON_NO_FLIGHT", "1", 1);
    setenv("BALLOON_GIMBAL_TRACKING", "0", 1);

    /* Switching, including a repeated value, cannot invoke any SDK function. */
    assert(DjiTargetChase_SetPanelMaintenance(true) == 0);
    assert(DjiTargetChase_SetPanelMaintenance(true) == 0);
    assert(DjiTargetChase_RequestedNoFlight());
    assert(!DjiTargetChase_RequestedGimbalTracking());
    assert(sdk_calls == 0 && !s_initialized);
    assert(DjiTargetChase_Init(NULL) == DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT_IN_CURRENT_STATE);
    assert(sdk_calls == 0 && !s_initialized);

    assert(DjiTargetChase_SetPanelMaintenance(false) == 0);
    assert(!DjiTargetChase_RequestedNoFlight());
    assert(DjiTargetChase_RequestedGimbalTracking());
    assert(sdk_calls == 0 && !s_initialized);
    assert(!strcmp(getenv("BALLOON_NO_FLIGHT"), "1"));
    assert(!strcmp(getenv("BALLOON_GIMBAL_TRACKING"), "0"));

    /* Every rejected change leaves the pair of requested settings unchanged. */
    for (int old = 0; old <= 1; ++old) {
        assert(DjiTargetChase_SetPanelMaintenance(old != 0) == 0);
        for (int blocker = 0; blocker < 3; ++blocker) {
            s_initialized = blocker == 0;
            s_real_authority_obtained = blocker != 0;
            s_control_inhibited = blocker == 2; /* release pending */
            assert(DjiTargetChase_SetPanelMaintenance(old == 0) != 0);
            assert(DjiTargetChase_RequestedNoFlight() == (old != 0));
            assert(DjiTargetChase_RequestedGimbalTracking() == (old == 0));
            assert(sdk_calls == 0);
            s_initialized = s_real_authority_obtained = s_control_inhibited = false;
        }
    }

    /* The later explicit Init consumes the typed pair despite startup env=1/0. */
    config = TargetChase_DefaultConfig(640, 480, 73.7f, 45.7f);
    assert(DjiTargetChase_SetPanelMaintenance(false) == 0);
    assert(sdk_calls == 0);
    assert(DjiTargetChase_Init(&config) == 0);
    assert(s_initialized && !DjiTargetChase_IsNoFlightMode());
    assert(s_gimbal_tracking_enabled && resets == 0 && gimbal_modes == 0 && sdk_calls > 0);
    assert(s_gimbal_state == DJI_TARGET_CHASE_GIMBAL_READY);
    assert(!s_gimbal_search_enabled && !s_body_search_enabled && !s_gimbal_static_test_mode);
    unsigned before = sdk_calls;
    assert(DjiTargetChase_SetPanelMaintenance(true) != 0);
    assert(!DjiTargetChase_RequestedNoFlight() && DjiTargetChase_RequestedGimbalTracking());
    assert(sdk_calls == before);
    assert(DjiTargetChase_DeInit() == 0);
    before = sdk_calls;
    assert(DjiTargetChase_SetPanelMaintenance(true) == 0);
    assert(DjiTargetChase_Init(&config) != 0 && !s_initialized);
    assert(sdk_calls == before);
    puts("panel mode: legacy fallback, atomic pair, busy rejection, SDK-free toggles, Init guards PASS");
    return 0;
}
'''


class PanelModeControl(unittest.TestCase):
    def test_real_adapter_mode_contract(self):
        candidates = [Path(os.environ['TEST_PSDK_ROOT'])] if os.environ.get('TEST_PSDK_ROOT') else [
            ROOT / 'psdk-overlay',
        ]
        sdk = next((p for p in candidates if (p/'psdk_lib/include/dji_typedef.h').is_file()), None)
        if sdk is None:
            self.skipTest('public SDK headers unavailable; set TEST_PSDK_ROOT')
        # Reuse only the existing deterministic SDK fixture. Instrument every
        # SDK substitute so a mode setter or maintenance Init cannot silently
        # initialize a service, move a gimbal, or submit flight commands.
        fixture = (ROOT/'tests/authority_stop_test.c').read_text()
        fixture = fixture.replace(
            '#include "../psdk-overlay/samples/sample_c/module_sample/target_chase/dji_target_chase_task.c"',
            '#include "'+str(CONTROL/'dji_target_chase_task.c')+'"')
        fixture, count = re.subn(r'\bint main\(void\)', 'static int baseline_main(void)', fixture)
        self.assertEqual(count, 1)
        stub = re.compile(r'(^[A-Za-z_][^;{}\n]*\bDji(?:AircraftInfo|FlightController|FcSubscription|Platform|GimbalManager)\w*'
                          r'\([^;{}]*\)\s*\{)', re.MULTILINE)
        fixture, count = stub.subn(r'\1 ++sdk_calls;', fixture)
        self.assertGreaterEqual(count, 20)
        with tempfile.TemporaryDirectory(prefix='uav-panel-mode-stubs-') as temporary:
            folder = Path(temporary)
            source = folder/'mode.c'; binary = folder/'mode'
            source.write_text('static unsigned sdk_calls;\n'+fixture+'\n'+MAIN)
            module = sdk/'samples/sample_c/module_sample'
            subprocess.run(shlex.split(os.environ.get('CC', 'cc')) + [
                '-std=gnu99','-O2','-Wall','-Wextra','-Werror','-Wno-unused-function','-pthread',
                '-DCJSON_HIDE_SYMBOLS','-I'+str(sdk/'psdk_lib/include'),'-I'+str(module),
                str(source),str(CONTROL/'target_chase.c'),str(module/'utils/cJSON.c'),
                '-lm','-o',str(binary)], check=True, timeout=60, capture_output=True, text=True)
            result = subprocess.run([str(binary)], check=True, cwd=folder,
                                    timeout=10, capture_output=True, text=True)
            self.assertIn('Init guards PASS', result.stdout)


if __name__ == '__main__':
    unittest.main()
