"""Synthetic camera turn: a fixed target should favor compensated prediction."""

import csv
import math
import os
import tempfile
import time
import unittest
from argparse import Namespace
from pathlib import Path

import cv2
import numpy as np

from prediction_compare import compare
from balloon_liveview_decoder import PredictionRecorder


class PredictionComparisonTest(unittest.TestCase):
    def test_native_pixel_ignores_camera_pose_and_retains_long_horizon(self):
        library = os.environ.get('BALLOON_NATIVE_PREDICTOR')
        if not library:
            self.skipTest('native library not configured')
        from prediction_native import NativePredictor
        from prediction_compare import AlphaBeta
        a = NativePredictor(Path(library), 640, 480, 73.7, 45.7, False)
        b = NativePredictor(Path(library), 640, 480, 73.7, 45.7, False)
        filt = AlphaBeta(.65, .12, 1280)
        try:
            for i in range(5):
                t=1000+i*100;u=300+i*5
                a.observe(t,(0,0),(u-10,230,u+10,250))
                b.observe(t,(i*12,i*3),(u-10,230,u+10,250))
                filt.update(u,t)
            pa, pb = a.predict_pixel(1810), b.predict_pixel(1810)
            self.assertEqual(pa,pb)
            self.assertAlmostEqual(pa[2],.5)
            fx=640/(2*math.tan(math.radians(73.7)/2))
            self.assertAlmostEqual(pa[0],math.degrees(math.atan((filt.predict(.5)-320)/fx)),places=4)
            self.assertIsNone(a.predict_pixel(1300))
        finally:
            a.close();b.close()

    def test_recording_start_and_stop(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            request = root / "record.request"
            request.write_text("record")
            recorder = PredictionRecorder(request, root / "runs", 10.0, root / "events.jsonl")
            for index in range(5):
                recorder.submit(index, 1000 + 100 * index,
                                np.zeros((48, 64, 3), dtype=np.uint8))
            # NTFS may retain the recorder's short-lived read handle.
            deadline = time.monotonic() + 1
            while True:
                try:
                    request.unlink()
                    break
                except PermissionError:
                    if os.name != "nt" or time.monotonic() >= deadline:
                        raise
                    time.sleep(0.01)
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                runs = list((root / "runs").glob("run_*"))
                events = root / "events.jsonl"
                if runs and events.exists() and "prediction_record_stopped" in events.read_text():
                    break
                time.sleep(0.05)
            recorder.stop = True
            recorder.wake.set()
            recorder.thread.join(timeout=1)
            self.assertEqual(len(runs), 1)
            self.assertEqual(len((runs[0] / "frames.csv").read_text().splitlines()), 6)
            self.assertGreater((runs[0] / "liveview.mkv").stat().st_size, 0)

    def test_stationary_target_during_camera_turn(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            session = root / "session"
            session.mkdir()
            labels = root / "labels.csv"
            poses = root / "poses.csv"
            writer = cv2.VideoWriter(str(session / "liveview.avi"),
                                     cv2.VideoWriter_fourcc(*"MJPG"), 10, (640, 480))
            self.assertTrue(writer.isOpened())
            with (session / "frames.csv").open("w", newline="") as frame_stream, \
                    labels.open("w", newline="") as label_stream, \
                    poses.open("w", newline="") as pose_stream:
                frames, boxes, pose = csv.writer(frame_stream), csv.writer(label_stream), csv.writer(pose_stream)
                frames.writerow(["video_frame", "source_frame_id", "frame_ts_ms"])
                boxes.writerow(["video_frame", "x1", "y1", "x2", "y2"])
                pose.writerow(["ts_ms", "yaw_deg", "pitch_deg"])
                fx = 640 / (2 * math.tan(math.radians(73.7) / 2))
                for index in range(40):
                    timestamp = 100000 + index * 100
                    yaw = index * 0.8
                    u = 320 + fx * math.tan(math.radians(-yaw))
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    writer.write(frame)
                    frames.writerow([index, index, timestamp])
                    boxes.writerow([index, u - 15, 225, u + 15, 255])
                    pose.writerow([timestamp, yaw, 0])
            writer.release()
            args = Namespace(session=session, labels=labels, poses=poses,
                             output=root / "out", observation_ms=200,
                             horizon_ms=100, max_age_ms=500,
                             future_tolerance_ms=60, max_pose_age_ms=120,
                             turn_threshold_deg=0.5, dropout_every_ms=0,
                             dropout_duration_ms=0, hfov_deg=73.7, vfov_deg=45.7,
                             alpha=0.65, beta=0.12)
            result = compare(args)
            self.assertGreater(result["paired_samples"], 20)
            self.assertLess(result["compensated"]["mean_center_error_px"],
                            result["pixel"]["mean_center_error_px"])
            library = os.environ.get("BALLOON_NATIVE_PREDICTOR")
            if library:
                args.native_predictor = Path(library)
                native_result = compare(args)
                self.assertTrue(native_result['native_flight_predictor'])
                self.assertEqual(native_result['adaptive']['samples'], result['paired_samples'])
                self.assertLess(native_result['adaptive']['p95_center_error_deg'], 0.01)
                self.assertLess(native_result['compensated']['p95_center_error_deg'], 0.01)
                # The same delayed/dropout input must reach all three methods.
                args.observation_delay_ms = 100
                args.dropout_every_ms, args.dropout_duration_ms = 700, 200
                delayed = compare(args)
                counts = [delayed[method]['samples'] for method in ('pixel', 'compensated', 'adaptive')]
                self.assertGreater(counts[0], 10)
                self.assertEqual(len(set(counts)), 1)
                self.assertLess(delayed['adaptive']['p95_center_error_deg'], 0.01)
                self.assertIn('mean_iou_delta', delayed['paired_comparison_to_pixel']['adaptive'])
                self.assertEqual(sum(delayed['by_observed_box_size'][group]['pixel']['samples']
                                     for group in ('small_box', 'large_box')), counts[0])
            # Mixing a newly monotonic pose file with an old recording must fail.
            content = poses.read_text().replace('ts_ms,yaw_deg,pitch_deg',
                                                'ts_ms,yaw_deg,pitch_deg,clock_id')
            lines = content.splitlines()
            poses.write_text(lines[0] + '\n' + '\n'.join(line + ',mono-v1:other-boot'
                                                         for line in lines[1:]) + '\n')
            with self.assertRaisesRegex(ValueError, 'legacy frame times'):
                compare(args)


if __name__ == "__main__":
    unittest.main()
