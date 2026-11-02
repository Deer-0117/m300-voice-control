"""Regression tests for wall-clock jumps and cross-process record validity."""
import importlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import balloon_clock as clock
import balloon_liveview_decoder as decoder
import balloon_tracker_worker as tracker

# Import only request/record logic; no model is constructed by this module.
with patch('os.execv'):
    worker = importlib.import_module('balloon_yolo_worker')


class ClockTests(unittest.TestCase):
    def test_wall_jump_does_not_change_monotonic_age(self):
        with patch('time.monotonic_ns', return_value=1_000_000_000), \
                patch('time.time_ns', return_value=0):
            first = clock.now_ms()
        with patch('time.monotonic_ns', return_value=1_200_000_000), \
                patch('time.time_ns', return_value=1_800_000_000_000_000_000):
            self.assertEqual(clock.now_ms() - first, 200)
        with patch('time.monotonic_ns', return_value=1_300_000_000), \
                patch('time.time_ns', return_value=0):
            self.assertEqual(clock.now_ms() - first, 300)

    def test_frame_sidecar_survives_file_mtime_change(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            metadata = root / 'latest.tsv'
            path, _, _, _, timestamp = decoder.publish_frame(
                root, metadata, 100, np.zeros((48, 64, 3), dtype=np.uint8), 90)
            os.utime(path, (1, 1))
            self.assertEqual(clock.frame_timestamp(path), timestamp)
            self.assertEqual(tracker.parse_liveview_metadata(metadata).ready_ts_ms, timestamp)
            path.with_suffix('.jpg.tsv').unlink()
            self.assertIsNone(clock.frame_timestamp(path))

    def test_boot_identity_and_legacy_records_are_rejected(self):
        current = clock.clock_id()
        self.assertIsNone(clock.split_record('1\t2\told-boot', 2))
        self.assertIsNone(clock.split_record('1\t2', 2))
        self.assertEqual(clock.split_record(f'1\t2\t{current}', 2), ['1', '2'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'target.tsv'
            tracker.atomic_write_target(path, 12, 640, 480, (10, 20, 30, 40),
                                        clock.now_ms(), 0.9, Path('frame_100.jpg'))
            self.assertEqual(tracker.parse_target(path).seq, 12)
            path.write_text(path.read_text().replace(current, 'mono-v1:previous-boot'))
            self.assertIsNone(tracker.parse_target(path))

    def test_detection_request_clock_and_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'request.tsv'
            now = clock.now_ms()
            for stamp, requested, token, valid in [
                (now - 100, now - 10, clock.clock_id(), True),
                (now + 5000, now + 5000, clock.clock_id(), False),
                (now - 100, now - 10, 'mono-v1:previous-boot', False),
                (now - 5000, now - 5000, clock.clock_id(), False),
            ]:
                path.write_text(f'1\t{stamp}\t{requested}\tframe_100.jpg\t{token}\n')
                self.assertEqual(worker.parse_request(path) is not None, valid)

    def test_near_hint_rejects_legacy_clock(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'hint'
            path.write_text(f'{clock.now_ms()}\t{clock.clock_id()}\n')
            self.assertTrue(decoder.near_rate_hint_active(path, 500))
            path.write_text(f'{clock.now_ms()}\n')
            self.assertFalse(decoder.near_rate_hint_active(path, 500))


if __name__ == '__main__':
    unittest.main()
