"""Evidence download remains useful on a drone with no camera or video modules."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


collector = load('telemetry_export_collector', ROOT/'scripts/collect_flight_results.py')
finisher = load('telemetry_export_finisher', ROOT.parent/'recognition/runtime/finish_flight_download.py')


def archive_fixture(root, payload=b'{"seq":1}\n', corrupt=False):
    source = root/'evidence'
    (source/'logs').mkdir(parents=True)
    (source/'logs/background_telemetry.jsonl').write_bytes(payload)
    manifest = {'files': {'logs/background_telemetry.jsonl': {
        'bytes': len(payload),
        'sha256': '0'*64 if corrupt else hashlib.sha256(payload).hexdigest()}}}
    (source/'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
    archive = root/'flight.tar.gz'
    with tarfile.open(archive, 'w:gz') as tf:
        tf.add(source, arcname='evidence')
    return archive


class BackgroundExportTests(unittest.TestCase):
    def test_snapshot_retains_current_and_rotated_logs_without_camera(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source, dest = root/'input', root/'output'
            source.mkdir(); dest.mkdir()
            for name in ('background_telemetry.jsonl', 'background_telemetry.1.jsonl'):
                (source/name).write_bytes((name+'\n').encode())
            notices = []
            inventory = collector.snapshot_background_telemetry(source, dest, notices)
            self.assertEqual(len(inventory), 2)
            self.assertEqual(notices, [])
            for path in source.iterdir():
                self.assertEqual(path.read_bytes(), (dest/path.name).read_bytes())

    def test_snapshot_does_not_follow_growth_while_background_writer_runs(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source, dest = root/'input', root/'output'
            source.mkdir(); dest.mkdir()
            log = source/'background_telemetry.jsonl'
            log.write_bytes(b'first\n')
            original_open = Path.open

            def open_with_growth(path, *args, **kwargs):
                if path.parent == dest and args and args[0] == 'wb':
                    with original_open(log, 'ab') as live:
                        live.write(b'after snapshot\n')
                return original_open(path, *args, **kwargs)

            with patch.object(Path, 'open', open_with_growth):
                inventory = collector.snapshot_background_telemetry(source, dest, [])
            self.assertEqual((dest/log.name).read_bytes(), b'first\n')
            self.assertEqual(inventory[0]['snapshot_bytes'], 6)
            self.assertIn(b'after snapshot', log.read_bytes())

    def test_no_recordings_needs_no_camera_or_replay_dependencies(self):
        analyzer = types.ModuleType('analyze_background_telemetry')
        received = []

        def analyze(paths):
            received.extend(paths)
            return {'samples': 1}

        analyzer.analyze = analyze
        analyzer.format_text = lambda report: 'Telemetry samples: 1\n'
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            archive = archive_fixture(root)
            with patch.dict(sys.modules, {'replay_flight': None, 'cv2': None,
                                          'analyze_background_telemetry': analyzer}):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(finisher.finish(archive), 0)
            self.assertEqual(len(received), 1)
            processed = json.loads((root/'flight_verified/download_processing.json').read_text())
            self.assertEqual(processed['replays'], [])
            self.assertEqual(processed['errors'], [])
            self.assertTrue(Path(processed['telemetry_report']).exists())
            self.assertEqual((root/'flight_verified/background_telemetry_report.txt').read_text(),
                             'Telemetry samples: 1\n')

    def test_collect_telemetry_only_omits_video_and_refuses_active_task(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td) / 'runtime'
            session = root / 'prediction_recordings/run_sample'
            session.mkdir(parents=True)
            (session / 'liveview.mkv').write_bytes(b'video')
            name = 'background_telemetry.jsonl'
            (root / name).write_bytes(b'raw telemetry\n')
            boot_id = '11111111-2222-4333-8444-555555555555'
            with patch.object(collector, 'recording_busy', return_value=True):
                with self.assertRaisesRegex(ValueError, 'Stop the active task'):
                    collector.collect(root, Path(td), boot_id, telemetry_only=True)
            with patch.object(collector, 'recording_busy', return_value=False):
                receipt = collector.collect(root, Path(td), boot_id, telemetry_only=True)
            archive = Path(receipt['archive'])
            self.assertEqual(receipt['sha256'], hashlib.sha256(archive.read_bytes()).hexdigest())
            with tarfile.open(archive, 'r:gz') as source:
                self.assertFalse(any('prediction_recordings' in p for p in source.getnames()))
                self.assertEqual(source.extractfile(archive.name.removesuffix('.tar.gz')
                    + '/logs/' + name).read(), b'raw telemetry\n')

    def test_integrity_failure_rejected_before_telemetry_analysis(self):
        with tempfile.TemporaryDirectory() as td:
            archive = archive_fixture(Path(td), corrupt=True)
            with self.assertRaisesRegex(ValueError, 'file integrity failure'):
                finisher.finish(archive)

    def test_export_without_telemetry_does_not_invent_report(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            source = root/'evidence'
            source.mkdir()
            (source/'manifest.json').write_text('{"files":{}}')
            archive = root/'empty.tar.gz'
            with tarfile.open(archive, 'w:gz') as tf:
                tf.add(source, arcname='evidence')
            with patch.dict(sys.modules, {'replay_flight': None, 'cv2': None}):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(finisher.finish(archive), 0)
            processed = json.loads((root/'empty_verified/download_processing.json').read_text())
            self.assertIsNone(processed['telemetry_report'])


if __name__ == '__main__':
    unittest.main()
