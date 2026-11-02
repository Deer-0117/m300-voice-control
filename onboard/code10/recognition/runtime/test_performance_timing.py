"""Timing must never change a tracker result or invent missing measurements."""
import importlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from performance_timing import PerformanceRecorder, BOUNDS_NS
import balloon_tracker_worker as tracker

with patch('os.execv'):
    worker = importlib.import_module('balloon_yolo_worker')


class PerformanceTimingTests(unittest.TestCase):
    def recorder(self, writer=None):
        self.now = 1_000_000_000
        self.output = []
        return PerformanceRecorder(Path('not-written'), 'test', enabled=True,
            clock=lambda: self.now, writer=writer or (lambda rows: self.output.extend(rows)))

    def test_disabled_profiler_has_no_clock_or_sink_calls(self):
        fail = Mock(side_effect=AssertionError('disabled measurement touched clock or sink'))
        rec = PerformanceRecorder(Path('unused'), 'test', enabled=False, clock=fail, writer=fail)
        self.assertEqual(rec.begin(), 0)
        rec.record('stage', 0, 0)
        rec.close()
        fail.assert_not_called()

    def test_all_attempts_counted_and_p95_is_histogram_upper_bound(self):
        rec = self.recorder()
        for duration in [100, 1000, 1001, 1_048_576_001]:
            rec.record('call', self.now, self.now+duration, success=duration<2000)
        self.now += 2_000_000_000
        rec.close()
        row, = self.output
        self.assertEqual(row['count'], 4)
        self.assertEqual(row['success_count'], 3)
        self.assertEqual(row['sum_ns'], 100+1000+1001+1_048_576_001)
        self.assertEqual(row['histogram_counts'][:2], [2, 1])
        self.assertEqual(row['histogram_counts'][-1], 1)
        self.assertIsNone(row['p95_upper_bound_ns'])
        self.assertEqual(row['aircraft_state'], 'unknown')
        self.assertEqual(row['mission_token'], '')

    def test_invalid_and_backward_stamps_are_not_zero_duration_samples(self):
        rec = self.recorder()
        for a, b in [(True, self.now), (self.now, float('nan')),
                     (self.now, self.now-1), (self.now-1, self.now)]:
            rec.record('call', a, b)
        rec.close()
        row, = self.output
        self.assertEqual(row['count'], 0)
        self.assertEqual(row['invalid_count'], 4)
        self.assertIsNone(row['min_ns'])
        self.assertEqual(sum(row['histogram_counts']), 0)

    def test_samples_are_aggregated_until_five_seconds(self):
        rec = self.recorder()
        rec.record('call', self.now, self.now+99)
        self.assertEqual(self.output, [])
        self.now += 5_000_000_000
        rec.record('call', self.now-10, self.now)
        self.assertEqual(len(self.output), 1)
        self.assertEqual(self.output[0]['count'], 2)
        rec.close()
        self.assertEqual(len(self.output), 1)

    def test_sink_failure_is_contained_and_reported_next_window(self):
        rec = self.recorder(writer=Mock(side_effect=OSError('disk unavailable')))
        rec.record('call', self.now, self.now+1000)
        self.now += 1000
        rec.close()
        self.assertEqual(rec.write_failures, 1)
        self.assertEqual(rec.write_failed_summaries, 1)
        self.assertEqual(rec.dropped_stage_samples, 0)
        rec.writer = self.output.extend
        rec.record('call', self.now, self.now+2000)
        self.now += 2000
        rec.close()
        row, = self.output
        self.assertEqual(row['window_id'], 1)
        self.assertEqual(row['write_failures'], 1)
        self.assertEqual(row['count'], 1)

    def test_stage_memory_is_bounded(self):
        rec = self.recorder()
        for i in range(100):
            rec.record(str(i), self.now, self.now+1)
        self.assertEqual(len(rec.stages), 16)
        self.assertEqual(rec.dropped_stage_samples, 84)
        self.assertEqual(rec.write_failed_summaries, 0)

    def test_diagnostic_sink_error_does_not_replace_tracker_exception(self):
        rec = self.recorder(writer=Mock(side_effect=RuntimeError('sink failed')))
        target = Mock()
        def update(image):
            self.now += 5_000_000_001
            raise ValueError('original tracker error')
        target.update.side_effect = update
        with self.assertRaisesRegex(ValueError, 'original tracker error'):
            tracker.timed_tracker_update(target, 'image', rec)
        self.assertEqual(rec.write_failures, 1)

    def test_tracker_returns_original_result_and_measures_even_unpublished_updates(self):
        rec = self.recorder()
        box = object()
        target = Mock()
        def update(image):
            self.now += 80_000_000
            return True, box
        target.update.side_effect = update
        ok, result, elapsed = tracker.timed_tracker_update(target, 'image', rec)
        self.assertTrue(ok)
        self.assertIs(result, box)
        self.assertGreaterEqual(elapsed, 0)
        target.update.assert_called_once_with('image')
        rec.close()
        self.assertEqual(self.output[0]['sum_ns'], 80_000_000)

    def test_tracker_exception_propagates_and_counts_failed_call(self):
        rec = self.recorder()
        target = Mock()
        def update(image):
            self.now += 30_000_000
            raise ValueError('tracker failed')
        target.update.side_effect = update
        with self.assertRaisesRegex(ValueError, 'tracker failed'):
            tracker.timed_tracker_update(target, 'image', rec)
        rec.close()
        self.assertEqual(self.output[0]['count'], 1)
        self.assertEqual(self.output[0]['success_count'], 0)
        self.assertEqual(self.output[0]['sum_ns'], 30_000_000)

    def test_predictor_logs_reject_bad_duration_without_faking_zero(self):
        with tempfile.TemporaryDirectory() as td, patch.object(worker, 'LATENCY_EVENT_LOG', Path(td)/'latency.jsonl'):
            for value in [-1, float('nan'), float('inf'), True, 1.25]:
                worker.append_latency('test', 7, value, detection_started_ts_ms=10)
            rows = [json.loads(line) for line in worker.LATENCY_EVENT_LOG.read_text().splitlines()]
        self.assertTrue(all(r['duration_ms'] is None and not r['timing_valid'] for r in rows[:4]))
        self.assertEqual(rows[-1]['duration_ms'], 1.25)
        self.assertEqual(rows[-1]['worker_session_id'], worker.WORKER_SESSION_ID)
        self.assertEqual(rows[-1]['detection_started_ts_ms'], 10)

    def test_emitted_json_is_finite_and_has_distinct_process_identity(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td)/'performance.jsonl'
            rec = PerformanceRecorder(path, 'test', enabled=True)
            start = rec.begin()
            rec.record('call', start, start+1000)
            rec.close()
            row = json.loads(path.read_text())
            other = PerformanceRecorder(path, 'test', enabled=True)
            self.assertNotEqual(row['profile_id'], other.profile_id)
            self.assertEqual(row['histogram_upper_bounds_ns'], BOUNDS_NS)
            self.assertEqual(row['p95_upper_bound_ns'], 1000)


if __name__ == '__main__':
    unittest.main()
