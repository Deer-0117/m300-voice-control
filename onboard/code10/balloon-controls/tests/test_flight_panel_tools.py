"""Checks use temporary source trees, fake /proc and no DJI/network interfaces."""
from contextlib import ExitStack
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT.parent / 'recognition/runtime'
sys.path.insert(0, str(RUNTIME))
import flight_panel_tools as panel
import capture_camera_timing as capture

BOOT = 'mono-v1:01234567-89ab-cdef-0123-456789abcdef'
OTHER_BOOT = 'mono-v1:01234567-89ab-cdef-0123-456789abcdee'


def valid_calibration():
    return dict(schema='host_receipt_delays_v1', verified=True, clock_id=BOOT,
                calibration_id='independent-001', video_delay_ms=100.5, pose_delay_ms=20,
                uncertainty_ms=25, valid_from_ms=1000, valid_until_ms=5000)


class CalibrationTests(unittest.TestCase):
    def test_bounds_are_inclusive_and_rounded_times_not_invented(self):
        data = valid_calibration()
        for stamp in (1000, 5000):
            self.assertEqual(panel.validate_calibration(data, BOOT, stamp), 'aligned')
        for stamp in (999, 5001):
            self.assertEqual(panel.validate_calibration(data, BOOT, stamp), 'timing_outside_window')
        for uncertainty in (25.00001, -0.1):
            data['uncertainty_ms'] = uncertainty
            self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_uncertain')

    def test_cross_boot_and_relative_phase_candidates_are_rejected(self):
        data = valid_calibration()
        self.assertEqual(panel.validate_calibration(data, OTHER_BOOT, 2000), 'timing_clock_mismatch')
        data['verified'] = False
        data['relative_lag_ms'] = 360
        self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_unverified')
        data['verified'] = 1
        self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_unverified')

    def test_delay_numeric_types_and_required_physical_delays(self):
        for name in ('video_delay_ms', 'pose_delay_ms'):
            for bad in (True, '100', -0.1, 1500.001, float('nan'), float('inf'), 10 ** 400):
                with self.subTest(name=name, bad_type=type(bad).__name__):
                    data = valid_calibration()
                    data[name] = bad
                    self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_invalid_delay')
            data = valid_calibration()
            del data[name]
            self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_invalid_delay')
        data = valid_calibration()
        data.update(video_delay_ms=0, pose_delay_ms=1500)
        self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'aligned')

    def test_window_must_be_finite_integral_and_json_safe(self):
        for key in ('valid_from_ms', 'valid_until_ms'):
            for bad in (0, -1, True, 1200.5, 9007199254740992, float('inf')):
                with self.subTest(key=key, bad=bad):
                    data = valid_calibration()
                    data[key] = bad
                    self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_invalid_window')
        data = valid_calibration()
        data['valid_until_ms'] = data['valid_from_ms']
        self.assertEqual(panel.validate_calibration(data, BOOT, 2000), 'timing_invalid_window')

    def test_clock_and_id_match_c_loader(self):
        for field, bad, reason in (
            ('clock_id', 'mono-v1:old', 'timing_invalid_clock'),
            ('clock_id', BOOT + '\x00anything', 'timing_invalid_clock'),
            ('calibration_id', '', 'timing_invalid_calibration_id'),
            ('calibration_id', 'a' * 80, 'timing_invalid_calibration_id'),
            ('calibration_id', 'not valid', 'timing_invalid_calibration_id'),
            ('calibration_id', '校准', 'timing_invalid_calibration_id'),
        ):
            data = valid_calibration()
            data[field] = bad
            self.assertEqual(panel.validate_calibration(data, BOOT, 2000), reason)

    def test_file_rejects_duplicates_oversize_and_nan(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'timing.json'
            texts = [json.dumps(valid_calibration())[:-1] + ',"verified":false}',
                     '{"x":NaN}', ' ' * 4096, '[1,2,3]']
            for text in texts:
                path.write_text(text)
                self.assertFalse(panel.calibration_status(path, BOOT, 2000)['ok'])
            path.write_text(json.dumps(valid_calibration()))
            self.assertTrue(panel.calibration_status(path, BOOT, 2000)['ok'])

    def test_missing_env_does_not_guess_calibration_file(self):
        with patch.dict(panel.os.environ, {}, clear=True):
            self.assertEqual(panel.calibration_status(), {'ok': False, 'reason': 'missing_calibration'})


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        base = Path(self.temp.name)
        self.sdk = base / 'sdk'
        self.runtime = base / 'runtime'
        self.proc = base / 'proc'
        self.proc.mkdir()
        self.runtime.mkdir()
        widget = self.sdk / panel.WIDGET_RELATIVE
        widget.mkdir(parents=True)
        source = widget / 'psdk_balloon_widget.c'
        source.write_text('/* source checksum fixture */')
        self.schema = [['main', i, 'button'] for i in (50, 51, 55, 56, 57, 58)]
        self.schema += [['config', i, 'switch' if i in (54, 69) else 'button']
                        for i in (69, 60, 61, 52, 53, 54, 65, 66, 67, 68)]
        self.aliases = {60: 50, 61: 51, 65: 55, 66: 56, 67: 57, 68: 58}
        self.names = {
            'cn': dict(zip(range(50, 59), ('开始测试', 'API联网', '相机/姿态采样', '部署/标定核验',
                                         'Kalman预测模式', '执行API语音任务', '新增API语音任务',
                                         '清除候选语音任务', '停止测试'))),
            'en': dict(zip(range(50, 59), ('Start test', 'Connect API', 'Camera / pose sampling',
                                         'Deployment / calibration check', 'Kalman prediction mode',
                                         'Execute API voice task', 'New API voice task',
                                         'Clear voice candidate', 'Stop test'))),
        }
        self.names['cn'][69] = '仅显示/维护'
        self.names['en'][69] = 'Display only / maintenance'
        self.rows = [{'device_path': str(source), 'sha256': panel.digest(source)}]
        self.config_paths = {}
        for language in ('cn', 'en'):
            config = {p + '_interface': {'widget_list': []} for p in ('main', 'config')}
            for page, index, kind in self.schema:
                entry = dict(widget_index=index, widget_type=kind,
                             widget_name=self.names[language][self.aliases.get(index, index)])
                if page == 'main':
                    entry['icon_file_set'] = {'icon_file_name_selected': 'button.png',
                                              'icon_file_name_unselected': 'button.png'}
                config[page + '_interface']['widget_list'].append(entry)
            path = widget / f'widget_file/{language}_big_screen/widget_config.json'
            path.parent.mkdir(parents=True)
            path.write_text(json.dumps(config))
            (path.parent / 'button.png').write_bytes(b'fixture')
            self.rows.append({'device_path': str(path), 'sha256': panel.digest(path)})
            self.config_paths[language] = path
        self.binary = self.sdk / 'bin/dji_sdk_demo_linux_cxx'
        self.binary.parent.mkdir()
        self.binary.write_bytes(b'compiled-fixture')
        self.manifest = dict(release_id='panel-20261102', files=self.rows,
                             binary=dict(device_path=str(self.binary), sha256=panel.digest(self.binary)),
                             widget_schema=self.schema)
        self.path = self.runtime / 'manifest.json'
        self.save_manifest()

    def save_manifest(self):
        self.path.write_text(json.dumps(self.manifest))

    def save_config(self, language, config):
        path = self.config_paths[language]
        path.write_text(json.dumps(config))
        for row in self.rows:
            if row['device_path'] == str(path):
                row['sha256'] = panel.digest(path)
        self.save_manifest()

    def audit(self):
        return panel.deployment_status(self.path, sdk_root=self.sdk, runtime_root=self.runtime, proc=self.proc)

    def process(self, pid, target=None):
        proc = self.proc / str(pid)
        proc.mkdir()
        (proc / 'comm').write_text('dji_sdk_demo_li\n')
        (proc / 'exe').symlink_to(self.binary if target is None else target)

    def test_stopped_is_reported_separately_from_deployment_integrity(self):
        result = self.audit()
        self.assertTrue(result['ok'])
        self.assertEqual(result['running']['status'], 'not_running')

    def test_running_binary_identity_is_verified(self):
        self.process(123)
        self.assertEqual(self.audit()['running']['status'], 'matched')
        other = self.sdk / 'other-binary'
        other.write_bytes(b'old-binary')
        (self.proc / '123/exe').unlink()
        (self.proc / '123/exe').symlink_to(other)
        result = self.audit()
        self.assertFalse(result['ok'])
        self.assertIn('running_binary_mismatch', result['issues'])

    def test_multiple_sdk_processes_do_not_claim_unique_active_version(self):
        self.process(123)
        self.process(124)
        self.assertFalse(self.audit()['ok'])

    def test_source_and_binary_mismatch_fail_independently(self):
        Path(self.rows[0]['device_path']).write_text('changed')
        self.binary.write_bytes(b'changed')
        result = self.audit()
        self.assertIn('source_hash_mismatch', result['issues'])
        self.assertIn('installed_binary_mismatch', result['issues'])

    def test_layout_and_icon_errors_are_detected_even_with_fresh_hash(self):
        path = Path(self.rows[1]['device_path'])
        config = json.loads(path.read_text())
        config['main_interface']['widget_list'][0]['widget_index'] = 18
        path.write_text(json.dumps(config))
        self.rows[1]['sha256'] = panel.digest(path)
        self.save_manifest()
        self.assertIn('widget_schema_mismatch', self.audit()['issues'])
        (path.parent / 'button.png').unlink()
        self.assertIn('widget_layout_or_icon_invalid', self.audit()['issues'])

    def test_schema_requires_exact_pages_unique_ids_and_action_types(self):
        for change in ('unknown_alias', 'duplicate_id', 'alias_type', 'kalman_type', 'maintenance_type', 'order', 'missing_maintenance', 'old_nine'):
            with self.subTest(change=change):
                schema = [row[:] for row in self.schema]
                if change == 'unknown_alias':
                    schema[7][1] = 62
                elif change == 'duplicate_id':
                    schema[7][1] = 50
                elif change == 'alias_type':
                    schema[7][2] = 'switch'
                elif change == 'kalman_type':
                    schema[11][2] = 'button'
                elif change == 'maintenance_type':
                    schema[6][2] = 'button'
                elif change == 'missing_maintenance':
                    del schema[6]
                elif change == 'order':
                    schema[6], schema[7] = schema[7], schema[6]
                else:
                    schema = schema[:6] + schema[9:12]
                self.manifest['widget_schema'] = schema
                self.save_manifest()
                with self.assertRaises(ValueError):
                    self.audit()

    def test_alias_names_must_match_each_language_and_canonical_action(self):
        self.assertTrue(self.audit()['ok'])
        for language, path in self.config_paths.items():
            original = path.read_text()
            for alias, action in self.aliases.items():
                with self.subTest(language=language, alias=alias):
                    config = json.loads(original)
                    entries = config['main_interface']['widget_list'] + config['config_interface']['widget_list']
                    by_id = {entry['widget_index']: entry for entry in entries}
                    self.assertEqual(by_id[alias]['widget_name'], self.names[language][action])
                    self.assertEqual(by_id[alias]['widget_name'], by_id[action]['widget_name'])
                    by_id[alias]['widget_name'] = self.names[language][51 if action != 51 else 50]
                    self.save_config(language, config)
                    self.assertIn('widget_name_mismatch', self.audit()['issues'])
            self.save_config(language, json.loads(original))

    def test_maintenance_switch_requires_first_setting_type_and_label(self):
        self.assertTrue(self.audit()['ok'])
        for language, path in self.config_paths.items():
            original = path.read_text()
            for change, issue in (('type', 'widget_schema_mismatch'),
                                  ('position', 'widget_schema_mismatch'),
                                  ('label', 'widget_name_mismatch'),
                                  ('missing', 'widget_schema_mismatch')):
                with self.subTest(language=language, change=change):
                    config = json.loads(original)
                    settings = config['config_interface']['widget_list']
                    self.assertEqual(settings[0]['widget_index'], 69)
                    self.assertEqual(settings[0]['widget_type'], 'switch')
                    self.assertEqual(settings[0]['widget_name'], self.names[language][69])
                    self.assertNotIn('icon_file_set', settings[0])
                    if change == 'type':
                        settings[0]['widget_type'] = 'button'
                    elif change == 'position':
                        settings[0], settings[1] = settings[1], settings[0]
                    elif change == 'label':
                        settings[0]['widget_name'] = 'Maintenance disabled'
                    else:
                        del settings[0]
                    self.save_config(language, config)
                    self.assertIn(issue, self.audit()['issues'])
            self.save_config(language, json.loads(original))

    def test_layout_rejects_alias_type_or_reused_id_with_fresh_hashes(self):
        for language, path in self.config_paths.items():
            original = path.read_text()
            for alias, action in self.aliases.items():
                for field, bad in (('widget_index', action), ('widget_type', 'switch')):
                    with self.subTest(language=language, alias=alias, field=field):
                        config = json.loads(original)
                        entry = next(row for row in config['config_interface']['widget_list']
                                     if row['widget_index'] == alias)
                        entry[field] = bad
                        self.save_config(language, config)
                        self.assertIn('widget_schema_mismatch', self.audit()['issues'])
            self.save_config(language, json.loads(original))

    def test_settings_icons_are_optional_but_supplied_paths_are_checked(self):
        self.assertTrue(self.audit()['ok'])
        for language, path in self.config_paths.items():
            original = path.read_text()
            for icon, good in (('button.png', True), ('../button.png', False), ('missing.png', False)):
                with self.subTest(language=language, icon=icon):
                    config = json.loads(original)
                    config['config_interface']['widget_list'][0]['icon_file_set'] = {'icon_file_name': icon}
                    self.save_config(language, config)
                    result = self.audit()
                    self.assertEqual(result['ok'], good)
                    if not good:
                        self.assertIn('widget_layout_or_icon_invalid', result['issues'])
            self.save_config(language, json.loads(original))

    def test_main_icons_remain_required(self):
        for language, path in self.config_paths.items():
            original = path.read_text()
            config = json.loads(original)
            del config['main_interface']['widget_list'][0]['icon_file_set']
            self.save_config(language, config)
            self.assertIn('widget_layout_or_icon_invalid', self.audit()['issues'])
            self.save_config(language, json.loads(original))

    def test_manifest_cannot_omit_c_or_languages_or_repeat_rows(self):
        for broken in (self.rows[1:], self.rows + [self.rows[0]]):
            self.manifest['files'] = broken
            self.save_manifest()
            with self.assertRaises(ValueError):
                self.audit()

    def test_manifest_cannot_read_secret_or_external_paths(self):
        for unsafe in (self.runtime / '.env', self.runtime.parent / 'secret.json'):
            self.manifest['files'] = self.rows + [dict(device_path=str(unsafe), sha256='0' * 64)]
            self.save_manifest()
            with self.assertRaises(ValueError):
                self.audit()


class SamplingTests(unittest.TestCase):
    def test_ground_confirmation_and_duration_precede_capture(self):
        called = []
        for duration, ground in ((30, False), (14, True), (61, True)):
            with self.assertRaises(ValueError):
                panel.sample(duration, ground, called.append)
        self.assertEqual(called, [])
        with patch.object(panel, 'publish'):
            result = panel.sample(30, True, called.append)
        self.assertEqual(called, [30])
        self.assertFalse(result['calibration_created'])

    def test_cancellation_runs_real_capture_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            request, record = base / 'timing.request', base / 'record.request'
            with ExitStack() as stack:
                for target, name, value in ((capture, 'REQUEST', request), (capture, 'RECORD', record),
                                             (capture, 'COMMIT', base / 'commit'), (capture, 'ACK', base / 'ack'),
                                             (capture, 'POSE', base / 'pose')):
                    stack.enter_context(patch.object(target, name, value))
                stack.enter_context(patch.object(capture, 'clock_id', return_value=BOOT))
                stack.enter_context(patch.object(capture, 'now_ms', return_value=1000))
                stack.enter_context(patch.object(capture.time, 'sleep', side_effect=panel.PanelCancelled))
                stack.enter_context(patch.object(panel, 'publish'))
                with self.assertRaises(panel.PanelCancelled):
                    panel.sample(30, True, capture.capture)
            self.assertFalse(request.exists())
            self.assertFalse(record.exists())

    def test_capture_never_overwrites_existing_mission_record_request(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            request, record = base / 'timing.request', base / 'record.request'
            record.write_text('existing-mission\n')
            with ExitStack() as stack:
                for name, value in (('REQUEST', request), ('RECORD', record), ('COMMIT', base / 'commit')):
                    stack.enter_context(patch.object(capture, name, value))
                stack.enter_context(patch.object(capture, 'clock_id', return_value=BOOT))
                stack.enter_context(patch.object(capture, 'now_ms', return_value=1000))
                stack.enter_context(patch.object(panel, 'publish'))
                with self.assertRaises(FileExistsError):
                    panel.sample(30, True, capture.capture)
            self.assertFalse(request.exists())
            self.assertEqual(record.read_text(), 'existing-mission\n')

    def test_utf8_message_has_atomic_complete_characters_and_240_byte_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'message.txt'
            panel.publish('采样完成' * 100, path)
            data = path.read_bytes()
            self.assertLessEqual(len(data), 240)
            self.assertTrue(data.endswith(b'\n'))
            data.decode('utf-8')


if __name__ == '__main__':
    unittest.main()
