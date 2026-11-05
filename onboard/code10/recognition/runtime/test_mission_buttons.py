"""Production handshake + actual sequence executor; no model or flight SDK."""
import importlib
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from balloon_liveview_decoder import PredictionRecorder

import balloon_mission as mission
with patch('os.execv'):
    worker = importlib.import_module('balloon_yolo_worker')


class MissionTests(unittest.TestCase):
    def test_small_lock_keeps_fast_pass_until_expired(self):
        seq = self.sequence
        seq.locked_center_norm = (0.5, 0.5)
        seq.locked_area_ratio = 0.001
        seq.locked_last_seen_ms = 1000
        with patch.dict('os.environ', {'BALLOON_FAR_TILE_ENABLED':'1', 'BALLOON_FAR_TILE_LOCK_GRACE_MS':'700'}):
            with patch.object(worker, 'now_ms', return_value=1200):
                self.assertFalse(seq.should_use_far_tiles([]))
            with patch.object(worker, 'now_ms', return_value=1800):
                self.assertTrue(seq.should_use_far_tiles([]))

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        root = Path(self.temp.name)
        paths = {name: root / getattr(mission, name).name for name in
                 ('TOKEN', 'REQUEST', 'ACK', 'READY', 'COMMIT', 'HEALTH', 'MESSAGE',
                  'STATUS', 'RECORD_REQUEST', 'RECORD_ACK', 'STOP', 'PLAN', 'STATE', 'EVENTS')}
        patcher = patch.multiple(mission, **paths)
        patcher.start(); self.addCleanup(patcher.stop)
        patcher = patch.multiple(worker, TASK_PLAN_FILE=paths['PLAN'], TASK_STATE_FILE=paths['STATE'],
                                 STOP_REQUEST_FILE=paths['STOP'], CONTROL_EVENT_LOG=root/'control.jsonl',
                                 SEARCH_REQUEST_FILE=root/'search', NEAR_RATE_HINT_FILE=root/'hint')
        patcher.start(); self.addCleanup(patcher.stop)
        self.sequence = worker.CapabilityTaskSequence()
        self.gate = mission.MissionGate()
        self.sequence.mission_gate = self.gate
        self.token = 'this-start'
        mission.TOKEN.write_text(self.token)

    def prepare(self, mode='pink', record=False):
        # Exercise the same idle polling used by the persistent worker. No
        # detection request/camera/trigger is necessary to obtain the receipt.
        done = threading.Event()
        def poll():
            while not done.is_set():
                self.gate.poll(self.sequence)
                done.wait(.01)
        thread = threading.Thread(target=poll)
        thread.start()
        try:
            mission.prepare(mode, self.token, record)
        finally:
            done.set(); thread.join()

    def test_pink_start_waits_for_real_executor_exact_plan(self):
        self.prepare()
        self.assertEqual(self.sequence.actions, mission.strike_actions(('pink', 'yellow')))
        self.assertEqual(self.sequence.current_index, 0)
        request, ack = mission.read_json(mission.REQUEST), mission.read_json(mission.ACK)
        for field in ('token', 'plan_id', 'digest', 'clock_id'):
            self.assertEqual(request[field], ack[field])
        self.assertEqual(mission.READY.read_text().strip(), self.token)
        self.assertFalse(mission.COMMIT.exists())

    def test_deleted_plan_cannot_reach_any_selector(self):
        self.prepare()
        mission.PLAN.unlink()
        with patch.object(self.sequence, '_select_locked_candidate', side_effect=AssertionError('fallback called')):
            self.assertIsNone(self.sequence.select([{'color': 'yellow'}]))
        self.assertFalse(mission.HEALTH.exists())
        self.assertIn('mission_plan_invalid', mission.STOP.read_text())

    def test_visual_ack_changes_immediately_with_action_and_cancellation(self):
        self.prepare()
        first = self.gate.visual_snapshot(self.sequence)
        self.assertEqual(first['requested_color'], 'pink')
        self.assertGreaterEqual(first['started_ts_ms'], mission.read_json(mission.REQUEST)['started_ts_ms'])
        # No 100 ms heartbeat delay is allowed when the action changes.
        self.sequence.current_index = 1
        with patch.object(mission, 'now_ms', return_value=first['started_ts_ms'] + 1):
            second = self.gate.visual_snapshot(self.sequence)
            ack = mission.read_json(mission.ACK)
            self.assertEqual(ack['current_index'], 1)
            self.assertEqual(ack['requested_color'], 'yellow')
            self.assertEqual(second['started_ts_ms'], first['started_ts_ms'] + 1)
        mission.TOKEN.unlink()
        self.assertIsNone(self.gate.visual_snapshot(self.sequence))
        self.assertFalse(mission.ACK.exists())

    def test_changed_same_id_plan_rejected(self):
        self.prepare()
        plan = mission.read_json(mission.PLAN)
        plan['actions'] = mission.strike_actions(('any',))
        mission.atomic_json(mission.PLAN, plan)
        self.assertFalse(self.gate.poll(self.sequence))
        self.assertFalse(mission.ACK.exists())

    def test_corrupt_plan_rejected_and_cannot_reuse_cached_actions(self):
        self.prepare()
        mission.PLAN.write_text('{broken')
        self.assertIsNone(self.sequence.select([]))
        self.assertFalse(mission.HEALTH.exists())

    def test_any_mode_is_an_explicit_plan(self):
        self.prepare('any')
        self.assertTrue(self.sequence.plan_id)
        self.assertEqual(self.sequence.actions, mission.strike_actions(('any',)))

    def test_old_boot_and_cancelled_token_rejected(self):
        self.prepare()
        request = mission.read_json(mission.REQUEST)
        request['clock_id'] = 'other-boot'
        mission.atomic_json(mission.REQUEST, request)
        self.assertFalse(self.gate.poll(self.sequence))
        mission.TOKEN.unlink()
        with self.assertRaisesRegex(ValueError, '取消'):
            mission.wait_ack(request, .01)

    def test_old_wrong_and_stale_receipts_timeout(self):
        request = dict(token=self.token, plan_id='p', digest='d', clock_id=mission.clock_id())
        for changed in ({'token': 'old'}, {'plan_id': 'old'}, {'digest': 'wrong'},
                        {'clock_id': 'old'}, {'ts_ms': mission.now_ms()-5000},
                        {'ts_ms': mission.now_ms()+5000}):
            ack = dict(request, ts_ms=mission.now_ms(), status='loaded')
            ack.update(changed)
            mission.atomic_json(mission.ACK, ack)
            with self.assertRaisesRegex(ValueError, '未确认'):
                mission.wait_ack(request, .01)

    def test_manual_next_keeps_outcome_unknown(self):
        self.prepare()
        old_id = self.sequence.plan_id
        self.sequence._mark_hit_unconfirmed('test_loss')
        self.assertFalse(self.gate.poll(self.sequence))
        self.prepare('next')
        self.assertNotEqual(old_id, self.sequence.plan_id)
        self.assertEqual(self.sequence.actions, mission.strike_actions(('yellow',)))
        self.assertEqual(self.sequence.completed_indices, set())
        events = [json.loads(line) for line in mission.EVENTS.read_text(encoding='utf-8').splitlines()]
        skipped = [row for row in events if row['event'] == 'operator_skipped_unconfirmed_target']
        self.assertEqual(skipped[0]['physical_outcome'], 'unknown')
        self.sequence._mark_hit_unconfirmed('test_loss')
        with self.assertRaisesRegex(ValueError, '最后'):
            mission.prepare('next', self.token, False)

    def test_next_during_active_plan_rejected(self):
        self.prepare()
        with self.assertRaisesRegex(ValueError, '没有等待'):
            mission.prepare('next', self.token, False)

    @patch('balloon_mission.now_ms', return_value=10000)
    def test_recording_requires_matching_current_receipt(self, _clock):
        self.prepare(record=True)
        for changes in ({'token': 'old'}, {'clock_id': 'old'}, {'ts_ms': 1}):
            receipt = dict(token=self.token, clock_id=mission.clock_id(), status='recording', ts_ms=mission.now_ms())
            receipt.update(changes)
            mission.atomic_json(mission.RECORD_ACK, receipt)
            with self.assertRaisesRegex(ValueError, '录像未确认'):
                mission.wait_record(self.token, .01)
        mission.atomic_json(mission.RECORD_ACK, dict(token=self.token, clock_id=mission.clock_id(), status='failed', error='disk'))
        with self.assertRaisesRegex(RuntimeError, '录像启动失败'):
            mission.wait_record(self.token, .01)
        mission.atomic_json(mission.RECORD_ACK, dict(token=self.token, clock_id=mission.clock_id(), status='recording', ts_ms=mission.now_ms()))
        mission.wait_record(self.token, .1)
        mission.COMMIT.write_text(self.token)
        self.assertTrue(self.gate.poll(self.sequence))
        mission.RECORD_ACK.unlink()
        self.assertFalse(self.gate.poll(self.sequence))

    def test_voice_must_commit_new_local_plan(self):
        self.prepare()
        old_plan = mission.read_json(mission.PLAN)
        old_request = mission.REQUEST.read_bytes()
        confirmation = dict(plan=old_plan, api_decision=dict(
            proposal_id='a' * 32, source='agent_interpret', session='recording-session'))
        for mode in ('voice', 'api_new'):
            with self.subTest(mode=mode):
                # Match C's gate reset before a new preparation. Returning an
                # API receipt without writing a new plan must never reach ACK.
                mission.READY.unlink(missing_ok=True)
                with patch('uav_voice_command.confirm', return_value=confirmation) as confirm, \
                     patch.object(mission, 'wait_ack') as wait_ack:
                    with self.assertRaisesRegex(ValueError, '未生成新的'):
                        mission.prepare(mode, self.token, False, 'recording-session')
                confirm.assert_called_once_with(session='recording-session', mission_token=self.token,
                                                allow_deferred=mode == 'api_new')
                wait_ack.assert_not_called()
                self.assertEqual(mission.read_json(mission.PLAN), old_plan)
                self.assertEqual(mission.REQUEST.read_bytes(), old_request)
                self.assertFalse(mission.READY.exists())
                self.assertFalse(mission.COMMIT.exists())

    def test_ui_languages_match_and_old_start_ids_are_absent(self):
        default = Path(__file__).resolve().parents[2]/'balloon-controls/psdk-overlay/samples/sample_c/module_sample/balloon_widget/widget_file'
        if not default.is_dir():
            default = Path('/home/uav/yanshou_uav/uavs_psdk/psdk_dji_3.9.2_cpp_json/samples/sample_c/module_sample/balloon_widget/widget_file')
        base = Path(os.environ.get('BALLOON_WIDGET_CONFIG_ROOT', str(default)))
        schemas = []
        aliases = {60: 50, 61: 51, 65: 55, 66: 56, 67: 57, 68: 58}
        for language in ('cn', 'en'):
            config = json.loads((base/f'{language}_big_screen/widget_config.json').read_text(encoding='utf-8'))
            main = config['main_interface']['widget_list']
            settings = config['config_interface']['widget_list']
            entries = main + settings
            schemas.append([(entry['widget_index'], entry['widget_type']) for entry in entries])
            self.assertEqual(len(entries), 16)
            self.assertEqual(len({entry['widget_index'] for entry in entries}), len(entries))
            self.assertEqual({entry['widget_index'] for entry in entries}, set(range(50, 59)) | set(aliases) | {69})
            self.assertEqual([entry['widget_index'] for entry in main], [50, 51, 55, 56, 57, 58])
            self.assertEqual([entry['widget_index'] for entry in settings], [69, 60, 61, 52, 53, 54, 65, 66, 67, 68])
            self.assertTrue(all(entry['widget_type'] == 'button' for entry in main))
            self.assertEqual([(entry['widget_index'], entry['widget_type']) for entry in settings],
                             [(69, 'switch'), (60, 'button'), (61, 'button'), (52, 'button'), (53, 'button'),
                              (54, 'switch'), (65, 'button'), (66, 'button'), (67, 'button'), (68, 'button')])
            self.assertNotIn('icon_file_set', settings[0])
            self.assertTrue(config['main_interface']['floating_window']['is_enable'])
            self.assertTrue(config['main_interface']['speaker']['is_enable_voice'])
            self.assertFalse(config['main_interface']['speaker']['is_enable_tts'])
            # New IDs prevent cached 18-29 controls from triggering new actions.
            names = ({50: '语音输入任务', 51: 'API联网', 52: '相机/姿态采样', 53: '部署/标定核验',
                      54: 'Kalman预测模式', 55: '执行API语音任务', 56: '新增API语音任务',
                      57: '清除候选语音任务', 58: '停止测试', 69: '仅显示/维护'}
                     if language == 'cn' else
                     {50: 'Voice task input', 51: 'Connect API', 52: 'Camera / pose sampling',
                      53: 'Deployment / calibration check', 54: 'Kalman prediction mode',
                      55: 'Execute API voice task', 56: 'New API voice task',
                      57: 'Clear voice candidate', 58: 'Stop test', 69: 'Display only / maintenance'})
            for entry in entries:
                self.assertFalse(entry['customize_rc_buttons_config']['is_enable'])
                action = aliases.get(entry['widget_index'], entry['widget_index'])
                self.assertEqual(entry['widget_name'], names[action])
            by_id = {entry['widget_index']: entry for entry in entries}
            for alias, action in aliases.items():
                self.assertEqual(by_id[alias]['widget_name'], by_id[action]['widget_name'])
                self.assertEqual(by_id[alias]['widget_type'], by_id[action]['widget_type'])
            self.assertEqual({aliases.get(entry['widget_index'], entry['widget_index']) for entry in entries},
                             set(range(50, 59)) | {69})
        self.assertEqual(schemas[0], schemas[1])

    def test_recorder_ack_matches_actual_session_and_flushes_on_stop(self):
        root = Path(self.temp.name)
        request = root/'balloon_prediction_record.request'
        request.write_text('first')
        recorder = PredictionRecorder(request, root/'recordings', 10, root/'record-log.jsonl')
        try:
            for index in range(4):
                recorder.submit(index, 1000+index*100, np.zeros((48, 64, 3), dtype=np.uint8))
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                if recorder.ack.exists() and mission.read_json(recorder.ack).get('token') == 'first':
                    break
                time.sleep(.01)
            ack = mission.read_json(recorder.ack)
            self.assertEqual((ack['token'], ack['status']), ('first', 'recording'))
            request.write_text('second')
            recorder.submit(5, 1500, np.zeros((48, 64, 3), dtype=np.uint8))
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                if mission.read_json(recorder.ack).get('token') == 'second':
                    break
                time.sleep(.01)
            self.assertEqual(mission.read_json(recorder.ack)['token'], 'second')
            request.unlink()
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                if mission.read_json(recorder.ack)['status'] == 'stopped':
                    break
                time.sleep(.01)
            self.assertEqual(mission.read_json(recorder.ack)['status'], 'stopped')
        finally:
            recorder.stop = True; recorder.wake.set(); recorder.thread.join(2)
        sessions = sorted((root/'recordings').glob('run_*'))
        self.assertEqual(len(sessions), 2)
        self.assertEqual([len((session/'frames.csv').read_text().splitlines())-1 for session in sessions], [4, 1])
        for session, token in zip(sessions, ('first', 'second')):
            metadata = mission.read_json(session/'session.json')
            self.assertEqual(metadata['mission_token'], token)
            self.assertEqual(metadata['status'], 'stopped')
            self.assertTrue((session/'liveview.mkv').is_file())

    def test_recorder_open_failure_reports_error(self):
        root = Path(self.temp.name)
        request = root/'balloon_prediction_record.request'
        request.write_text('failed-start')
        with patch('balloon_liveview_decoder.TimestampVideoWriter', side_effect=OSError('disk failure')):
            recorder = PredictionRecorder(request, root/'recordings', 10, root/'record-log.jsonl')
            try:
                recorder.submit(1, 1000, np.zeros((48, 64, 3), dtype=np.uint8))
                deadline = time.monotonic()+3
                while time.monotonic() < deadline and not recorder.ack.exists():
                    time.sleep(.01)
                receipt = mission.read_json(recorder.ack)
                self.assertEqual((receipt['token'], receipt['status']), ('failed-start', 'failed'))
                self.assertFalse(request.exists())
            finally:
                recorder.stop = True; recorder.wake.set(); recorder.thread.join(2)


    def test_pink_yellow_keeps_same_gate_for_second_target(self):
        self.prepare()
        self.sequence.hit_armed = True
        self.sequence.missing_since_ms = 1000
        with patch.object(worker, 'now_ms', return_value=5000):
            self.sequence.select([])
        self.assertEqual(self.sequence.current_index, 1)
        self.assertFalse(mission.STOP.exists())
        self.assertTrue(self.gate.poll(self.sequence))
        self.assertEqual(mission.read_json(mission.REQUEST)['token'], self.token)

    def test_atomic_ack_retries_transient_sharing_error(self):
        mission.ACK.write_text('old')
        original = Path.replace
        attempts = []
        def replace(path, target):
            attempts.append(path)
            if len(attempts) == 1:
                raise PermissionError('reader holds file')
            return original(path, target)
        with patch.object(mission, 'REPLACE_RETRIES', 3), patch.object(Path, 'replace', replace):
            mission.atomic_text(mission.ACK, 'new')
        self.assertEqual(mission.ACK.read_text(), 'new')
        self.assertEqual(len(attempts), 2)
        self.assertEqual(list(mission.ACK.parent.glob('*.tmp')), [])

    def test_atomic_ack_permanent_error_keeps_previous_receipt(self):
        mission.ACK.write_text('old')
        with patch.object(mission, 'REPLACE_RETRIES', 2), patch.object(
                Path, 'replace', side_effect=PermissionError('denied')) as replace:
            with self.assertRaises(PermissionError):
                mission.atomic_text(mission.ACK, 'new')
        self.assertEqual(replace.call_count, 2)
        self.assertEqual(mission.ACK.read_text(), 'old')
        self.assertEqual(list(mission.ACK.parent.glob('*.tmp')), [])


if __name__ == '__main__':
    unittest.main()
