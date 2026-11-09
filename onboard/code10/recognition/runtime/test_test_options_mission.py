"""Test selections through the existing mission, vision and voice gates."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import balloon_mission as mission
import balloon_visual_receipt as vision
import uav_voice_command as voice


class SelectedMissionTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        names = ('TOKEN', 'REQUEST', 'ACK', 'READY', 'COMMIT', 'HEALTH', 'MESSAGE',
                 'STATUS', 'RECORD_ACK', 'STOP', 'PLAN', 'STATE', 'EVENTS')
        files = patch.multiple(mission, **{name: self.root / getattr(mission, name).name for name in names})
        files.start()
        self.addCleanup(files.stop)
        mission.TOKEN.write_text('selected-test')
        self.sequence = SimpleNamespace(current_index=0, status='active')

        def load():
            plan = mission.read_json(mission.PLAN)
            self.sequence.plan_id, self.sequence.actions = plan['plan_id'], plan['actions']

        self.sequence._load_plan = load

    def prepare(self, detector='yolo', prediction=3, mode='pink'):
        gate = mission.MissionGate(active_detector=detector)
        self.sequence.mission_gate = gate
        wait_ack = mission.wait_ack

        def poll(request):
            self.assertTrue(gate.poll(self.sequence))
            wait_ack(request, timeout=.1)

        with patch.object(mission, 'wait_ack', side_effect=poll):
            mission.prepare(mode, 'selected-test', False, detector=detector, prediction=prediction)
        return gate

    def test_selected_options_and_visual_phase_are_acknowledged_and_archived(self):
        gate = self.prepare()
        request, ack = mission.read_json(mission.REQUEST), mission.read_json(mission.ACK)
        self.assertEqual((ack['detector'], ack['prediction']), ('yolo', 3))
        self.assertEqual(self.sequence.actions, mission.strike_actions(('pink', 'yellow')))
        archive = self.root / 'mission_records' / 'selected-test' / 'request.json'
        self.assertEqual(mission.read_json(archive), request)
        self.assertEqual(mission.READY.read_text().strip(), 'selected-test')
        self.assertIn('YOLO', mission.STATUS.read_text(encoding='utf-8'))
        self.assertEqual(gate.visual_snapshot(self.sequence)['requested_color'], 'pink')
        self.sequence.current_index = 1
        self.assertEqual(gate.visual_snapshot(self.sequence)['requested_color'], 'yellow')
        self.assertEqual(mission.read_json(mission.ACK)['current_index'], 1)

    def test_old_or_other_worker_cannot_acknowledge_selected_detector(self):
        self.prepare()
        for active_detector in (None, 'pixel'):
            gate = mission.MissionGate(active_detector)
            self.assertFalse(gate.poll(self.sequence))
            self.assertFalse(mission.ACK.exists())
            self.assertFalse(mission.HEALTH.exists())

    def test_old_mismatched_or_boolean_option_receipts_do_not_release_start(self):
        self.prepare(prediction=1)
        request = mission.read_json(mission.REQUEST)
        receipt = mission.read_json(mission.ACK)
        legacy = {key: value for key, value in receipt.items() if key not in mission.OPTION_KEYS}
        for wrong in (legacy, dict(receipt, detector='pixel'), dict(receipt, prediction=0),
                      dict(receipt, prediction=True)):
            mission.atomic_json(mission.ACK, wrong)
            with self.assertRaisesRegex(ValueError, '未确认'):
                mission.wait_ack(request, timeout=.001)

    def test_frame_must_match_selected_backend_and_action_context(self):
        gate = self.prepare()
        context = gate.visual_snapshot(self.sequence)
        stamp = mission.now_ms()
        evidence = dict(capture_ts_ms=stamp, detection_finished_ts_ms=stamp, seq=1)
        self.assertTrue(vision.frame_is_current(self.sequence, context, evidence, 640, 480, 'yolo'))
        self.assertFalse(vision.frame_is_current(self.sequence, context, evidence, 640, 480, 'pixel'))
        self.sequence.current_index = 1
        self.assertFalse(vision.frame_is_current(self.sequence, context, evidence, 640, 480, 'yolo'))

    def test_pixel_rejects_pink_plan_but_can_acknowledge_any(self):
        with self.assertRaisesRegex(ValueError, '只支持黄'):
            mission.prepare('pink', 'selected-test', False, detector='pixel', prediction=1)
        self.assertFalse(mission.PLAN.exists())
        self.assertFalse(mission.READY.exists())
        self.prepare('pixel', 1, 'any')
        self.assertIn('颜色像素', mission.STATUS.read_text(encoding='utf-8'))

    def test_incompatible_voice_candidate_does_not_publish_a_plan_or_get_consumed(self):
        names = ('CANDIDATE_FILE', 'SESSION_STATE', 'LOCK_FILE', 'SESSION_FILE', 'READY_FILE', 'MESSAGE_FILE')
        paths = {name: self.root / getattr(voice, name).name for name in names}
        paths.update(MISSION_TOKEN_FILE=mission.TOKEN, MISSION_COMMIT_FILE=mission.COMMIT)
        with patch.multiple(voice, **paths):
            voice.begin_session('voice-session')
            api_result = dict(kind='api', proposal_id='a' * 32, actions=mission.strike_actions(('pink',)))
            with patch.object(voice, '_api_proposal', return_value=api_result):
                candidate = voice.create_candidate('撞击粉色气球', 'voice-session')
            with patch.object(voice, 'store_plan') as store_plan:
                with self.assertRaisesRegex(ValueError, '只支持黄'):
                    mission.prepare('voice', 'selected-test', False, 'voice-session',
                                    detector='pixel', prediction=0, api_motion=True)
                store_plan.assert_not_called()
            self.assertEqual(mission.read_json(voice.CANDIDATE_FILE), candidate)
            self.assertEqual(voice.READY_FILE.read_text().strip(), 'voice-session')
            self.assertTrue(voice.status('voice-session')['ready'])
        self.assertFalse(mission.PLAN.exists())
        self.assertFalse(mission.REQUEST.exists())
        self.assertFalse(mission.READY.exists())

    def test_next_refuses_pending_state_from_another_boot(self):
        self.prepare()
        plan = mission.read_json(mission.PLAN)
        mission.atomic_json(mission.STATE, dict(plan_id=plan['plan_id'], status='awaiting_confirmation',
                                               current_index=0, runtime_clock_id='previous-boot'))
        with self.assertRaisesRegex(ValueError, '没有等待'):
            mission.prepare('next', 'selected-test', False, detector='yolo', prediction=1)
        self.assertEqual(mission.read_json(mission.PLAN), plan)

    def test_voice_requires_api_aware_launcher_and_cannot_become_local(self):
        for mode, api_motion in (('voice', False), ('api_new', False), ('any', True)):
            with self.subTest(mode=mode), patch.object(voice, 'confirm') as confirm:
                with self.assertRaisesRegex(ValueError, '运动来源不匹配'):
                    mission.prepare(mode, 'selected-test', False, api_motion=api_motion)
                confirm.assert_not_called()
        self.assertFalse(mission.REQUEST.exists())
        self.assertFalse(mission.READY.exists())

    def test_api_motion_source_must_be_echoed_and_matches_only_voice(self):
        plan = dict(plan_id='api-plan', source='widget_voice', motion_source='api',
                    api_decision={'proposal_id': 'proposal'}, actions=mission.strike_actions(('yellow',)))
        mission.atomic_json(mission.PLAN, plan)
        request = dict(mode='voice', token='selected-test', plan_id=plan['plan_id'],
                       clock_id=mission.clock_id(), digest=mission.digest(plan),
                       started_ts_ms=mission.now_ms(), motion_source='api', record=False)
        mission.atomic_json(mission.REQUEST, request)
        gate = mission.MissionGate()
        self.assertTrue(gate.poll(self.sequence))
        receipt = mission.read_json(mission.ACK)
        self.assertEqual(receipt.pop('motion_source'), 'api')
        mission.atomic_json(mission.ACK, receipt)
        with self.assertRaisesRegex(ValueError, '未确认'):
            mission.wait_ack(request, timeout=.01)
        for invalid in (dict(request, mode='any'), dict(request, motion_source='local'),
                        {k: v for k, v in request.items() if k != 'motion_source'}):
            mission.atomic_json(mission.REQUEST, invalid)
            self.assertFalse(gate.poll(self.sequence))
            self.assertFalse(mission.HEALTH.exists())

    def test_next_rejects_voice_and_legacy_plans_with_ambiguous_provenance(self):
        self.prepare()
        original = mission.read_json(mission.PLAN)
        mission.atomic_json(mission.STATE, dict(plan_id=original['plan_id'], status='awaiting_confirmation',
                                               current_index=0, runtime_clock_id=mission.clock_id()))
        for source in ('widget_voice', 'widget_api_new', 'api_voice_confirmed', 'widget_next'):
            plan = dict(original, source=source)
            if source == 'widget_next':
                del plan['motion_source']
            mission.atomic_json(mission.PLAN, plan)
            with self.assertRaisesRegex(ValueError, '只能跳过本地测试'):
                mission.prepare('next', 'selected-test', False)
            self.assertEqual(mission.read_json(mission.PLAN), plan)

    def test_invalid_selection_and_task_cannot_become_any(self):
        for detector, prediction in (('missing', 1), ('yolo', 2), ('pixel', None),
                                     (None, 1), ('yolo', True)):
            with self.assertRaisesRegex(ValueError, '选项无效'):
                mission.prepare('any', 'selected-test', False, detector=detector, prediction=prediction)
        with self.assertRaisesRegex(ValueError, '任务类型无效'):
            mission.prepare('unknown', 'selected-test', False)
        self.assertFalse(mission.PLAN.exists())


if __name__ == '__main__':
    unittest.main()
