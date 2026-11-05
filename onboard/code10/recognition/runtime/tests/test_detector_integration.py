import contextlib
import importlib.abc
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import detector_selector as selector
import balloon_yolo_worker as worker
import detect_latest_photo as photo
from balloon_clock import clock_id, now_ms, split_record
from hardcase_protocol import WorkerProducer


class DetectorIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.selection = self.root / 'selector.json'
        self.settings = {'active_detector': 'pixel', 'pixel_config': str(ROOT / 'pixel_callable_module/config.json')}
        self.selection.write_text(json.dumps(self.settings))

    def detector(self):
        return selector.load_detector(self.selection)

    def sample(self):
        hsv = np.zeros((720, 1280, 3), np.uint8)
        cv2.circle(hsv, (640, 360), 55, (40, 40, 200), -1)
        source = self.root / 'input.png'
        self.assertTrue(cv2.imwrite(str(source), cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)))
        return source

    def test_pixel_never_invokes_yolo_factory(self):
        factory = Mock(side_effect=AssertionError('YOLO must stay unloaded'))
        detector, config, weights = selector.load_detector(self.selection, factory)
        factory.assert_not_called()
        self.assertIsNone(weights)
        self.assertIsNone(selector.detector_metadata(detector, config)['weights'])
        self.assertEqual(config['color_detector']['max_area_ratio'], 1.0)

    def test_yolo_branch_only_calls_yolo_factory(self):
        self.settings['active_detector'] = 'yolo'
        self.selection.write_text(json.dumps(self.settings))
        factory = Mock(return_value=(object(), {'weights': Path('x.pt')}, Path('x.pt')))
        with patch('pixel_callable_module.ColorThresholdDetector', side_effect=AssertionError('pixel loaded')):
            _, config, _ = selector.load_detector(self.selection, factory)
        factory.assert_called_once_with()
        self.assertEqual(config['active_detector'], 'yolo')

    def test_cli_pixel_never_reexecs_and_yolo_uses_its_environment(self):
        with patch.object(selector, 'read_selection', return_value={'active_detector':'pixel'}), \
             patch('os.execv') as execute:
            selector.ensure_script_interpreter()
            execute.assert_not_called()
        with patch.object(selector, 'read_selection', return_value={'active_detector':'yolo'}), \
             patch.object(sys, 'prefix', '/usr'), patch.object(Path, 'is_file', return_value=True), \
             patch('os.execv') as execute:
            selector.ensure_script_interpreter()
            self.assertEqual(execute.call_args.args[0], str(ROOT / 'yolo_env/bin/python'))

    def test_missing_invalid_and_unknown_config_fail_without_fallback(self):
        factory = Mock()
        for text in ['{', '{}', '{"active_detector":"typo","pixel_config":"x"}',
                     '{"active_detector":"pixel","pixel_config":"missing.json"}',
                     '{"active_detector":"pixel","pixel_config":"x","typo":1}']:
            self.selection.write_text(text)
            with self.assertRaises((ValueError, FileNotFoundError)):
                selector.load_detector(self.selection, factory)
        factory.assert_not_called()

    def test_pixel_worker_process_request_preserves_color_and_wire_format(self):
        detector, config, _ = self.detector()
        source = self.sample()
        seq = Mock()
        # This fixture tests standalone detector wire compatibility; managed
        # mission authorization is exercised with a real gate in test_visual_receipt.
        seq.mission_gate = None
        seq.select.side_effect = lambda candidates: candidates[0] if candidates else None
        seq.should_use_far_tiles.side_effect = AssertionError('pixel must not request far tiles')
        with contextlib.ExitStack() as stack:
            for key, name in [('CONTROL_EVENT_LOG','control.jsonl'),('LATENCY_EVENT_LOG','latency.jsonl'),
                              ('LATEST_TARGET_FILE','target.tsv'),('LATEST_YOLO_TARGET_FILE','correction.tsv')]:
                stack.enter_context(patch.object(worker, key, self.root / name))
            stack.enter_context(patch.object(worker, 'TASK_SEQUENCE', seq))
            color = stack.enter_context(patch.object(worker, 'classify_balloon_colors', side_effect=AssertionError('color overwritten')))
            tiles = stack.enter_context(patch.object(worker, 'predict_far_tiles', side_effect=AssertionError('tiles invoked')))
            stack.enter_context(patch.object(photo, 'build_target_chase_plan', return_value=None))
            stamp = now_ms()
            request = worker.DetectionRequest(17, stamp, stamp, source)
            log = self.root / 'result.jsonl'
            worker.process_request(request, detector, config, log, 'balloon')
            payload = json.loads(log.read_text().splitlines()[-1])
            self.assertTrue(payload['ok'], payload.get('error'))
            self.assertNotIn('error', payload)
            self.assertEqual(payload['active_detector'], 'pixel')
            self.assertEqual(payload['best_balloon']['color_name'], 'yellow')
            self.assertEqual(payload['far_tile_detection_count'], 0)
            self.assertIsNone(payload['weights'])
            fields = split_record((self.root / 'target.tsv').read_text(), 11)
            self.assertIsNotNone(fields)
            self.assertEqual(fields[0], '17')
            self.assertEqual(fields[10], str(source))
            self.assertEqual((self.root / 'target.tsv').read_bytes(), (self.root / 'correction.tsv').read_bytes())
            self.assertIn('pixel_predict', (self.root / 'latency.jsonl').read_text())
            color.assert_not_called(); tiles.assert_not_called()

    def test_single_photo_entry_uses_selected_detector(self):
        source = self.sample()
        with patch.object(photo, 'load_detector', side_effect=self.detector), \
             patch.object(photo, 'build_target_chase_plan', return_value=None), \
             patch.object(photo, 'LATEST_TARGET_FILE', self.root / 'single.tsv'), \
             patch.object(photo, 'CONTROL_EVENT_LOG', self.root / 'single-control.jsonl'), \
             patch.object(sys, 'argv', ['photo', '--source', str(source), '--log', str(self.root / 'single.jsonl')]), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(photo.main(), 0)
        self.assertEqual(json.loads((self.root / 'single.jsonl').read_text())['active_detector'], 'pixel')

    def test_hardcase_metadata_has_pixel_class_map_and_no_model(self):
        detector, config, _ = self.detector()
        producer = WorkerProducer(True, self.root / 'missing-ipc', config, detector)
        self.addCleanup(producer.close)
        self.assertEqual(producer.class_map, {'0':'balloon'})
        self.assertEqual(producer.model_hashes, {})
        self.assertIn('no model weights', producer.model_reasons['color_threshold'])

    def test_fresh_pixel_process_blocks_all_neural_imports(self):
        code = '''
import importlib.abc,sys
class Block(importlib.abc.MetaPathFinder):
 def find_spec(self, fullname, path=None, target=None):
  if fullname.split('.')[0] in {'torch','ultralytics','tensorrt','yolo_infer','balloon_tensorrt'}:
   raise AssertionError('Unexpected neural import: '+fullname)
sys.meta_path.insert(0,Block())
import balloon_yolo_worker as w
from detector_selector import load_detector
from pathlib import Path
d,c,_=load_detector(Path(sys.argv[1]),w.load_yolo_detector)
assert d.predict_records(Path(sys.argv[2]))
assert not {'torch','ultralytics','tensorrt','yolo_infer','balloon_tensorrt'}.intersection(sys.modules)
'''
        result = subprocess.run([sys.executable, '-B', '-c', code, str(self.selection), str(self.sample())],
                                cwd=ROOT, capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
