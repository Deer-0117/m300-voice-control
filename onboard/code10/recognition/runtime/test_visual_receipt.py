"""Mission-bound YOLO/HSV evidence, without a model, network or flight SDK."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import balloon_visual_receipt as visual


class ReceiptTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'visual.json'
        patcher = patch.multiple(visual, STATUS=self.path, now_ms=lambda: 2000)
        patcher.start(); self.addCleanup(patcher.stop)
        self.context = dict(token='test', clock_id='boot', plan_id='plan',
                            current_index=0, requested_color='purple', started_ts_ms=1000)
        self.seq = Mock()
        self.seq.mission_gate.visual_snapshot.return_value = self.context
        self.payload = dict(capture_ts_ms=1800, detection_finished_ts_ms=1900, seq=8)
        self.target = dict(class_name='balloon', color_name='blue', confidence=.85,
                           bbox_xyxy=[100, 110, 200, 230])

    def publish(self, **changes):
        args = dict(sequence=self.seq, context=self.context, payload=self.payload,
                    best=self.target, width=640, height=480, source='yolo')
        args.update(changes)
        return visual.publish(**args)

    def test_local_yolo_color_group_is_explicit(self):
        self.assertTrue(self.publish())
        receipt = json.loads(self.path.read_text())
        self.assertTrue(receipt['target_found'])
        self.assertTrue(receipt['matches_requested'])
        self.assertEqual(receipt['requested_color'], 'purple')
        self.assertEqual(receipt['selected_color'], 'blue')
        self.assertEqual(receipt['x1'], 100)

    def test_pixel_local_source_and_absence_receipt(self):
        self.assertTrue(self.publish(source='pixel', best=None))
        receipt = json.loads(self.path.read_text())
        self.assertFalse(receipt['target_found'])
        self.assertIsNone(receipt['selected_color'])
        self.assertNotIn('x1', receipt)

    def test_wrong_color_or_non_balloon_does_not_authorize(self):
        for changes in ({'color_name': 'yellow'}, {'class_name': 'person'},
                        {'color_name': 'unknown'}, {'confidence': 0},
                        {'bbox_xyxy': [-1, 10, 20, 30]}, {'confidence': float('nan')}):
            with self.subTest(changes=changes):
                self.assertTrue(self.publish(best=dict(self.target, **changes)))
                self.assertFalse(json.loads(self.path.read_text())['target_found'])

    def test_pre_task_stale_future_missing_and_tracker_frames_rejected(self):
        for changes in ({'capture_ts_ms': 999}, {'capture_ts_ms': 500},
                        {'capture_ts_ms': 2001}, {'capture_ts_ms': None},
                        {'detection_finished_ts_ms': 2001}, {'seq': 0x80000001},
                        {'seq': True}):
            with self.subTest(changes=changes):
                self.assertFalse(self.publish(payload=dict(self.payload, **changes)))
        self.assertFalse(self.path.exists())

    def test_cancel_or_phase_change_during_inference_rejects_result(self):
        for changed in (None, dict(self.context, token='new'),
                        dict(self.context, current_index=1),
                        dict(self.context, started_ts_ms=1900),
                        dict(self.context, plan_id='replaced')):
            self.seq.mission_gate.visual_snapshot.return_value = changed
            self.assertFalse(self.publish())
        self.assertFalse(self.path.exists())

    def test_second_context_check_rejects_late_cancellation(self):
        self.seq.mission_gate.visual_snapshot.side_effect = [self.context, None]
        self.assertFalse(self.publish())
        self.assertFalse(self.path.exists())

    def test_api_sources_and_invalid_frame_size_never_authorize(self):
        for source in ('api', 'mil', ''):
            self.assertFalse(self.publish(source=source))
        self.assertFalse(self.publish(width=None))
        self.assertFalse(self.publish(height=float('inf')))

    def test_no_mission_means_no_authorization(self):
        self.assertIsNone(visual.snapshot(object()))
        self.assertFalse(self.publish(context=None))



class WorkerProcessReceiptTests(unittest.TestCase):
    """Exercise process_request and the real stateful task selector together.

    The detector and image decoder are deterministic substitutes. MissionGate,
    CapabilityTaskSequence, process_request and receipt publication are real.
    """
    def setUp(self):
        import test_mission_buttons as fixture
        import detect_latest_photo as photo
        self.worker = fixture.worker
        self.mission = fixture.mission
        self.fixture = fixture.MissionTests('test_visual_ack_changes_immediately_with_action_and_cancellation')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.fixture.prepare()
        self.sequence = self.fixture.sequence
        self.context = self.fixture.gate.visual_snapshot(self.sequence)
        self.root = Path(self.fixture.temp.name)
        self.receipt_path = self.root/'visual.json'
        self.logs = []
        patches = [
            patch.multiple(self.worker, TASK_SEQUENCE=self.sequence,
                           ensure_source_ready=lambda _: True,
                           append_latency=lambda *args, **kwargs: None,
                           append_jsonl=lambda path, payload: self.logs.append(dict(payload)),
                           write_latest_target=lambda *args, **kwargs: None,
                           detector_metadata=lambda *args, **kwargs: {},
                           classify_balloon_colors=lambda path, detections: None),
            patch.multiple(photo, prepare_detection_image=lambda path: (path, {}),
                           image_size=lambda path: (640, 480)),
            patch.object(visual, 'STATUS', self.receipt_path),
        ]
        for patcher in patches:
            patcher.start()
            self.addCleanup(patcher.stop)

    def run_frame(self, *, capture=None, source='pixel', during_inference=None, color='pink'):
        capture = self.mission.now_ms() if capture is None else capture

        class Detector:
            provides_colors = source == 'pixel'
            supports_far_tiles = False
            backend_name = source

            def predict_records(self, path):
                if during_inference:
                    during_inference()
                return [dict(class_name='balloon', color_name=color, color_confidence=1.0,
                             confidence=.95, bbox_xyxy=[200, 100, 400, 300])]

        request = self.worker.DetectionRequest(
            1, capture, self.mission.now_ms(), self.root/'synthetic.png')
        self.worker.process_request(request, Detector(), {'active_detector': source},
                                    self.root/'detections.jsonl', 'balloon')
        self.assertTrue(self.logs, 'process_request must finish and log the inspected frame')
        self.assertNotIn('error', self.logs[-1])

    def assert_unmodified_selector(self):
        self.assertFalse(self.sequence.hit_armed)
        self.assertIsNone(self.sequence.locked_center_norm)
        self.assertEqual(self.sequence.near_arm_count, 0)
        self.assertEqual(self.sequence.observed_color_cache, {})
        self.assertFalse(self.receipt_path.exists())

    def test_pre_action_capture_cannot_arm_hit_or_create_lock(self):
        with patch.object(self.sequence, 'select', wraps=self.sequence.select) as select:
            self.run_frame(capture=self.context['started_ts_ms'] - 1)
            select.assert_not_called()
        self.assert_unmodified_selector()

    def test_phase_change_during_inference_never_feeds_old_frame_to_new_action(self):
        def next_action():
            self.sequence.current_index = 1

        with patch.object(self.sequence, 'select', wraps=self.sequence.select) as select:
            self.run_frame(during_inference=next_action, color='yellow')
            select.assert_not_called()
        self.assertEqual(self.sequence.current_index, 1)
        self.assert_unmodified_selector()
        ack = self.mission.read_json(self.mission.ACK)
        self.assertEqual(ack['requested_color'], 'yellow')
        self.assertEqual(ack['current_index'], 1)

    def test_phase_change_at_selector_entry_is_rechecked_before_state_updates(self):
        real_select = self.sequence.select

        def change_then_select(detections, **kwargs):
            self.sequence.current_index = 1
            return real_select(detections, **kwargs)

        with patch.object(self.sequence, 'select', side_effect=change_then_select) as select:
            self.run_frame(color='yellow')
            select.assert_called_once()
        self.assertEqual(self.sequence.current_index, 1)
        self.assert_unmodified_selector()

    def test_cancellation_during_inference_never_feeds_selector(self):
        with patch.object(self.sequence, 'select', wraps=self.sequence.select) as select:
            self.run_frame(during_inference=self.mission.TOKEN.unlink)
            select.assert_not_called()
        self.assert_unmodified_selector()
        self.assertFalse(self.mission.ACK.exists())

    def test_current_pixel_frame_produces_bound_receipt(self):
        self.run_frame(source='pixel')
        receipt = json.loads(self.receipt_path.read_text())
        self.assertTrue(receipt['target_found'])
        self.assertEqual(receipt['source'], 'pixel')
        self.assertEqual(receipt['requested_color'], 'pink')
        self.assertEqual(receipt['selected_color'], 'pink')
        self.assertEqual(receipt['plan_id'], self.context['plan_id'])
        self.assertEqual(receipt['token'], self.context['token'])
        self.assertEqual(receipt['current_index'], 0)
        self.assertGreaterEqual(receipt['capture_ts_ms'], receipt['started_ts_ms'])

    def test_current_yolo_frame_produces_bound_receipt(self):
        self.run_frame(source='yolo')
        receipt = json.loads(self.receipt_path.read_text())
        self.assertTrue(receipt['target_found'])
        self.assertEqual(receipt['source'], 'yolo')
        self.assertEqual(receipt['requested_color'], 'pink')
        self.assertEqual(receipt['selected_color'], 'pink')
        self.assertEqual(receipt['plan_id'], self.context['plan_id'])


if __name__ == '__main__':
    unittest.main()
