"""No model or aircraft is started: verify sequence decisions from detection loss."""
import importlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

with patch('os.execv'):
    worker = importlib.import_module('balloon_yolo_worker')


class FlightOutcomeTest(unittest.TestCase):
    def exercise_loss(self, edge):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            actions = [{'capability': 'strike_balloon', 'parameters': {'color': color}}
                       for color in ('any', 'yellow')]
            (root/'plan.json').write_text(json.dumps({'plan_id': 'trial', 'actions': actions}))
            with patch.multiple(worker, TASK_PLAN_FILE=root/'plan.json',
                                TASK_STATE_FILE=root/'state.json',
                                STOP_REQUEST_FILE=root/'stop.request',
                                CONTROL_EVENT_LOG=root/'control.jsonl'), \
                    patch.object(worker, 'now_ms', return_value=5000):
                sequence = worker.CapabilityTaskSequence()
                sequence._load_plan()
                sequence.hit_armed = True
                sequence.missing_since_ms = 1000
                sequence.last_target_seen_at_edge = edge
                # Feed an empty detection result through the production selection path.
                self.assertIsNone(sequence.select([]))
                self.assertEqual(sequence.status, 'awaiting_confirmation')
                self.assertEqual(sequence.current_index, 0)
                self.assertEqual(sequence.completed_indices, set())
                self.assertTrue((root/'stop.request').is_file())
                events = [json.loads(line)['event'] for line in (root/'control.jsonl').read_text().splitlines()]
                self.assertIn('capability_hit_unconfirmed', events)
                self.assertNotIn('capability_task_advanced', events)
                # Repeated frames and worker restart must not resume/advance silently.
                self.assertIsNone(sequence.select([]))
                self.assertEqual(sequence.completed_indices, set())
                restored = worker.CapabilityTaskSequence()
                restored._load_plan()
                self.assertEqual(restored.status, 'awaiting_confirmation')
                self.assertEqual(restored.completed_indices, set())

    def test_edge_disappearance_is_not_a_confirmed_hit(self):
        self.exercise_loss(True)

    def test_central_disappearance_is_not_a_confirmed_hit(self):
        self.exercise_loss(False)

    def exercise_automatic_pink_yellow(self, edge):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            actions = [{'capability': 'strike_balloon', 'parameters': {'color': c}}
                       for c in ('pink', 'yellow')]
            (root/'plan.json').write_text(json.dumps({'plan_id': 'automatic', 'actions': actions}))
            with patch.multiple(worker, TASK_PLAN_FILE=root/'plan.json', TASK_STATE_FILE=root/'state.json',
                                STOP_REQUEST_FILE=root/'stop', CONTROL_EVENT_LOG=root/'events',
                                SEARCH_REQUEST_FILE=root/'search'), patch.object(worker, 'now_ms', return_value=5000):
                sequence = worker.CapabilityTaskSequence()
                sequence._load_plan()
                sequence.hit_armed = True
                sequence.missing_since_ms = 1000
                sequence.last_target_seen_at_edge = edge
                sequence.select([])
                self.assertEqual((sequence.status, sequence.current_index), ('active', 1))
                self.assertEqual(sequence.completed_indices, {0})
                self.assertFalse((root/'stop').exists())
                # Empty frames without a new close approach cannot finish yellow.
                sequence.select([])
                self.assertEqual(sequence.current_index, 1)
                restored = worker.CapabilityTaskSequence()
                restored._load_plan()
                self.assertEqual((restored.status, restored.current_index), ('active', 1))
                restored.hit_armed = True
                restored.missing_since_ms = 1000
                restored.last_target_seen_at_edge = edge
                restored.select([])
                self.assertEqual(restored.status, 'completed')
                self.assertEqual(restored.completed_indices, {0, 1})
                self.assertIn('capability_plan_complete', (root/'stop').read_text())
                rows = [json.loads(s) for s in (root/'events').read_text().splitlines()]
                outcomes = [r for r in rows if r['event'] == 'capability_step_completed_by_disappearance']
                self.assertEqual(len(outcomes), 2)
                self.assertTrue(all(r['physical_outcome'] == 'unknown' for r in outcomes))

    def test_api_pink_yellow_does_not_advance_or_search(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            actions = [{'capability': 'strike_balloon', 'parameters': {'color': color}}
                       for color in ('pink', 'yellow')]
            (root/'plan.json').write_text(json.dumps({
                'plan_id': 'api-motion', 'motion_source': 'api', 'actions': actions,
            }))
            with patch.multiple(worker, TASK_PLAN_FILE=root/'plan.json',
                                TASK_STATE_FILE=root/'state.json',
                                STOP_REQUEST_FILE=root/'stop',
                                CONTROL_EVENT_LOG=root/'events',
                                SEARCH_REQUEST_FILE=root/'search'), \
                    patch.object(worker, 'now_ms', return_value=5000), \
                    patch.dict('os.environ', {'BALLOON_NL_ALLOW_TARGET_DEFER': '1',
                                               'BALLOON_NL_TARGET_SEARCH_TIMEOUT_MS': '1000'}):
                sequence = worker.CapabilityTaskSequence()
                sequence._load_plan()
                sequence.search_missing_since_ms = 1000
                self.assertIsNone(sequence.select([]))
                self.assertFalse((root/'search').exists())
                self.assertEqual(sequence.current_index, 0)
                sequence.hit_armed = True
                sequence.missing_since_ms = 1000
                self.assertIsNone(sequence.select([]))
                self.assertEqual((sequence.status, sequence.current_index), ('awaiting_confirmation', 0))
                self.assertEqual(sequence.completed_indices, set())
                self.assertIn('capability_hit_unconfirmed', (root/'stop').read_text())

    def test_pink_yellow_edge_loss_automatically_advances(self):
        self.exercise_automatic_pink_yellow(True)

    def test_pink_yellow_central_loss_automatically_advances(self):
        self.exercise_automatic_pink_yellow(False)


if __name__ == '__main__':
    unittest.main()
