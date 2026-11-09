"""Exercise selection transactions without invoking systemctl or aircraft SDK."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import widget_test_options as options
from balloon_clock import now_ms


class FakeServices:
    def __init__(self, root, clock):
        self.root, self.clock = root, clock
        self.states = {options.WORKER:dict(LoadState='loaded', ActiveState='active', MainPID='301'),
                       options.TRACKER:dict(LoadState='loaded', ActiveState='active', MainPID='302'),
                       options.PANEL:dict(LoadState='loaded', ActiveState='active', MainPID='123'),
                       options.LEGACY_SDK:dict(LoadState='loaded', ActiveState='inactive', MainPID='0')}
        self.calls = []
        self.fail_pixel = False
        self.cancel_on_start = None
        self.receipt()

    def state(self, unit):
        return self.states[unit].copy()

    def receipt(self):
        settings = json.loads((self.root / 'detector_config.json').read_text())
        value = dict(pid=int(self.states[options.WORKER]['MainPID']), clock_id=self.clock,
                     ready_ts_ms=now_ms(), active_detector=settings['active_detector'],
                     selector_config=settings, selector_sha256=options.settings_hash(settings),
                     neural_modules_loaded=[])
        (self.root / 'detector_runtime.json').write_text(json.dumps(value))

    def action(self, action, *units):
        self.calls.append((action, *units))
        for unit in units:
            if action == 'stop':
                self.states[unit].update(ActiveState='inactive', MainPID='0')
            else:
                mode = json.loads((self.root / 'detector_config.json').read_text())['active_detector']
                if unit == options.WORKER and mode == 'pixel' and self.fail_pixel:
                    raise OSError('injected worker startup failure')
                self.states[unit].update(ActiveState='active', MainPID=str(400+len(self.calls)))
                if unit == options.WORKER:
                    self.receipt()
                    if self.cancel_on_start:
                        self.cancel_on_start()
                        self.cancel_on_start = None


class OptionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.root, self.tmp, self.proc = (base / p for p in ('runtime', 'tmp', 'proc'))
        for directory in (self.root, self.tmp, self.proc):
            directory.mkdir()
        self.clock = 'mono-v1:ad828ee3-3076-44b6-83c5-a60b29cacde3'
        self.token, self.owner = 'test-current-epoch-1', 123
        owner = self.proc / str(self.owner)
        owner.mkdir()
        (owner / 'cmdline').write_bytes(b'/srv/bin/dji_sdk_demo_linux_cxx\0config.json\0')
        self.stat = '123 (sdk app) ' + ' '.join(['S'] + ['0'] * 18 + ['555'])
        (owner / 'stat').write_text(self.stat)
        (self.tmp / options.LEASE).write_text(f'{self.token}\n{self.owner}\n')
        self.settings = {'active_detector':'yolo', 'pixel_config':'pixel.json'}
        (self.root / 'detector_config.json').write_text(json.dumps(self.settings))
        (self.root / 'pixel.json').write_text('{}')
        self.services = FakeServices(self.root, self.clock)
        self.selection = options.Selection(self.token, self.owner, root=self.root, tmp=self.tmp,
                                           proc=self.proc, services=self.services, clock=self.clock)

    def runtime(self, **changes):
        path = self.root / 'detector_runtime.json'
        value = json.loads(path.read_text())
        value.update(changes)
        path.write_text(json.dumps(value))

    def test_ready_requires_current_boot_pid_and_exact_loaded_settings(self):
        self.selection.verify('yolo')
        for changed in ({'pid':999}, {'clock_id':'mono-v1:old'}, {'active_detector':'pixel'},
                        {'selector_sha256':'bad'}, {'ready_ts_ms':now_ms()+100000},
                        {'selector_config':{'active_detector':'yolo','pixel_config':'other.json'}}):
            self.services.receipt()
            self.runtime(**changed)
            with self.assertRaises(options.SelectionError):
                self.selection.verify('yolo')

    def test_check_allows_own_prepared_task_but_does_not_restart(self):
        (self.tmp / 'balloon_mission.token').write_text('own-task')
        self.selection.verify('yolo')
        self.assertEqual(self.services.calls, [])

    def test_start_check_refuses_incomplete_tracker_recovery(self):
        for state in ({'ActiveState':'inactive', 'MainPID':'0'},
                      {'ActiveState':'active', 'MainPID':'0'},
                      {'ActiveState':'activating', 'MainPID':'400'}):
            self.services.states[options.TRACKER].update(state)
            with self.assertRaisesRegex(options.SelectionError, '跟踪服务未就绪'):
                self.selection.verify('yolo')
        self.assertEqual(self.services.calls, [])

    def test_busy_markers_and_live_recording_refuse_switch_without_changes(self):
        for name in options.BUSY:
            path = self.tmp / name
            path.write_text('busy')
            with self.assertRaises(options.SelectionError):
                self.selection.switch('pixel')
            path.unlink()
        (self.tmp / 'balloon_prediction_record_ack.json').write_text(
            json.dumps({'clock_id':self.clock, 'status':'recording'}))
        with self.assertRaises(options.SelectionError):
            self.selection.switch('pixel')
        self.assertEqual(self.services.calls, [])

    def test_other_panel_process_refuses_even_when_old_service_would_be_inactive(self):
        path = self.proc / '777'
        path.mkdir()
        (path / 'cmdline').write_bytes(b'/old/dji_sdk_demo_linux_cxx\0')
        with self.assertRaisesRegex(options.SelectionError, 'Another SDK'):
            self.selection.switch('pixel')
        self.assertEqual(self.services.calls, [])

    def test_switch_requires_the_active_maintenance_panel_and_inactive_legacy_sdk(self):
        self.services.states[options.PANEL]['MainPID'] = '999'
        with self.assertRaisesRegex(options.SelectionError, 'active maintenance panel'):
            self.selection.switch('pixel')
        self.services.states[options.PANEL]['MainPID'] = str(self.owner)
        self.services.states[options.LEGACY_SDK]['ActiveState'] = 'deactivating'
        with self.assertRaisesRegex(options.SelectionError, 'legacy SDK service'):
            self.selection.switch('pixel')
        self.assertEqual(self.services.calls, [])

    def test_invalid_or_replaced_lease_and_reused_owner_pid_refuse(self):
        self.selection.lease()
        (self.tmp / options.LEASE).write_text(f'other-token\n{self.owner}\n')
        with self.assertRaises(options.Cancelled):
            self.selection.switch('pixel')
        (self.tmp / options.LEASE).write_text(f'{self.token}\n{self.owner}\n')
        (self.proc / str(self.owner) / 'stat').write_text(self.stat[:-3]+'556')
        with self.assertRaises(options.Cancelled):
            self.selection.verify('yolo')
        self.assertEqual(self.services.calls, [])

    def test_select_stops_old_workers_clears_targets_then_verifies_new_pid(self):
        (self.root / 'latest_target.tsv').write_text('old target')
        (self.tmp / 'balloon_visual_status.json').write_text('old visual receipt')
        self.selection.switch('pixel')
        self.selection.verify('pixel')
        self.assertEqual((self.tmp / options.ACK).read_text(), self.token + '\n')
        self.assertFalse((self.root / 'latest_target.tsv').exists())
        self.assertFalse((self.tmp / 'balloon_visual_status.json').exists())
        self.assertEqual(self.services.calls[0], ('stop', options.TRACKER, options.WORKER))
        self.assertEqual(self.services.state(options.TRACKER)['ActiveState'], 'active')
        backups = list((self.root / 'detector_switch_backups').iterdir())
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / 'latest_target.tsv').read_text(), 'old target')

    def test_switch_does_not_start_an_inactive_worker(self):
        self.services.states[options.WORKER].update(ActiveState='inactive', MainPID='0')
        with self.assertRaisesRegex(options.SelectionError, 'Detector service is stopped'):
            self.selection.switch('pixel')
        self.assertEqual(self.services.calls, [])

    def test_same_selection_is_noop_after_readiness_check(self):
        self.selection.switch('yolo')
        self.assertEqual(self.services.calls, [])

    def test_failed_new_worker_restores_configuration_and_prior_services(self):
        self.services.fail_pixel = True
        with self.assertRaises(OSError):
            self.selection.switch('pixel')
        self.assertEqual(json.loads(self.selection.config.read_text()), self.settings)
        self.assertEqual(self.services.state(options.WORKER)['ActiveState'], 'active')
        self.assertEqual(self.services.state(options.TRACKER)['ActiveState'], 'active')
        self.assertEqual(json.loads((self.root / 'detector_runtime.json').read_text())['active_detector'], 'yolo')

    def test_stop_after_new_worker_started_rolls_back_and_never_leaves_success_ack(self):
        self.services.cancel_on_start = lambda:(self.tmp / options.LEASE).unlink()
        with self.assertRaises(options.Cancelled):
            self.selection.switch('pixel')
        self.assertEqual(json.loads(self.selection.config.read_text()), self.settings)
        self.assertFalse((self.tmp / options.ACK).exists())
        self.assertEqual(json.loads((self.root / 'detector_runtime.json').read_text())['active_detector'], 'yolo')

    def test_stop_while_publishing_success_rolls_back_and_removes_ack(self):
        atomic_text = options.atomic_text

        def cancel_during_ack(path, text):
            atomic_text(path, text)
            if path == self.tmp / options.ACK:
                (self.tmp / options.LEASE).unlink()

        with patch.object(options, 'atomic_text', side_effect=cancel_during_ack):
            with self.assertRaises(options.Cancelled):
                self.selection.switch('pixel')
        self.assertEqual(json.loads(self.selection.config.read_text()), self.settings)
        self.assertFalse((self.tmp / options.ACK).exists())
        self.assertEqual(self.services.state(options.TRACKER)['ActiveState'], 'active')
        self.assertEqual(json.loads((self.root / 'detector_runtime.json').read_text())['active_detector'], 'yolo')

    def test_failure_restores_loaded_selection_when_config_was_edited(self):
        (self.root / 'detector_config.json').write_text(json.dumps(dict(self.settings, active_detector='pixel')))
        self.services.fail_pixel = True
        with self.assertRaises(OSError):
            self.selection.switch('pixel')
        self.assertEqual(json.loads(self.selection.config.read_text()), self.settings)

    def test_rollback_failure_is_reported_and_does_not_claim_success(self):
        original = self.services.action
        def broken(action, *units):
            if action == 'start' and options.WORKER in units:
                raise OSError('all worker starts fail')
            original(action, *units)
        self.services.action = broken
        with self.assertRaisesRegex(options.SelectionError, 'rollback incomplete'):
            self.selection.switch('pixel')
        report = next((self.root / 'detector_switch_backups').glob('*/rollback.json'))
        self.assertFalse(json.loads(report.read_text())['restore_commands_completed'])
        self.assertFalse((self.tmp / options.ACK).exists())

    def test_initially_inactive_tracker_stays_inactive(self):
        self.services.states[options.TRACKER].update(ActiveState='inactive',MainPID='0')
        self.selection.switch('pixel')
        self.assertEqual(self.services.state(options.TRACKER)['ActiveState'], 'inactive')

    def test_missing_pixel_dependency_configuration_refuses_before_service_changes(self):
        (self.root / 'pixel.json').unlink()
        with self.assertRaises(FileNotFoundError):
            self.selection.switch('pixel')
        self.assertEqual(self.services.calls, [])

    def test_unknown_selection_rejected(self):
        with self.assertRaises(options.SelectionError):
            self.selection.switch('unknown')
        self.assertEqual(self.services.calls, [])


if __name__ == '__main__':
    unittest.main()
