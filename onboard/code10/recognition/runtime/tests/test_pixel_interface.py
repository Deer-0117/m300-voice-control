"""Interface checks only; numeric buffers are not recognition fixtures."""

from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from pixel_callable_module import ColorThresholdDetector, load_color_config


class PixelInterfaceTests(unittest.TestCase):
    def setUp(self):
        self.detector = ColorThresholdDetector()

    def test_rejects_invalid_frame_before_opencv(self):
        invalid = [None, [], np.zeros((2, 2, 3), dtype=np.float32),
                   np.zeros((2, 2), dtype=np.uint8),
                   np.zeros((2, 2, 4), dtype=np.uint8),
                   np.zeros((0, 2, 3), dtype=np.uint8),
                   np.zeros((2, 0, 3), dtype=np.uint8)]
        with patch("pixel_callable_module.detector.cv2.cvtColor") as convert:
            for frame in invalid:
                with self.subTest(shape=getattr(frame, "shape", None)):
                    with self.assertRaises((TypeError, ValueError)):
                        self.detector.predict_frame(frame)
            convert.assert_not_called()

    def test_frame_keeps_source_and_uses_existing_record_processing(self):
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        expected = []
        with patch.object(self.detector, "_records_from_mask", return_value=expected) as records:
            self.assertIs(self.detector.predict_frame(frame, "相机/帧001"), expected)
        self.assertEqual(records.call_args.args[0], "相机/帧001")
        self.assertEqual(self.detector.last_timings_ms["read_decode"], 0.0)

    def test_default_source_remains_empty_string(self):
        with patch.object(self.detector, "_records_from_mask", return_value=[]) as records:
            self.detector.predict_frame(np.zeros((2, 2, 3), dtype=np.uint8))
        self.assertEqual(records.call_args.args[0], "")

    def test_file_decode_delegates_to_frame_without_exif_rotation(self):
        source = Path("中文目录/输入.jpg")
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        expected = []
        with patch.object(Path, "is_dir", return_value=False), \
             patch.object(Path, "read_bytes", return_value=b"decode boundary"), \
             patch("pixel_callable_module.detector.cv2.imdecode", return_value=frame) as decode, \
             patch.object(self.detector, "predict_frame", return_value=expected) as predict:
            self.assertIs(self.detector.predict_records(source), expected)
        decode.assert_called_once()
        self.assertEqual(decode.call_args.args[1], cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
        self.assertIs(predict.call_args.args[0], frame)
        self.assertEqual(predict.call_args.args[1], str(source))

    def test_default_config_is_packaged_config_json(self):
        self.assertEqual(load_color_config(), self.detector.config)


if __name__ == "__main__":
    unittest.main()
