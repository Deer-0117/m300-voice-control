"""Exercise the real stage and audit interfaces with a private build fixture."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/prepare_code10_trial.py'
spec = importlib.util.spec_from_file_location('prepare_code10_trial', SCRIPT)
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)

from flight_panel_tools import deployment_status


class Code10TrialTest(unittest.TestCase):
    def test_staged_manifest_accepts_both_panels_and_rejects_swapped_choices(self):
        with tempfile.TemporaryDirectory() as temp:
            parent = Path(temp)
            binary = parent / 'built'
            binary.write_bytes(b'\x7fELF\x02\x01' + bytes(12) + b'\xb7\x00')
            output = trial.stage_sources(parent / 'trial', binary)
            runtime = output / 'runtime'
            overlay = output / 'psdk-overlay'
            proc = parent / 'proc'
            proc.mkdir()
            manifest_path = runtime / 'flight_panel_manifest.json'
            manifest = json.loads(manifest_path.read_text())
            self.assertFalse((runtime / 'detector_config.json').exists())
            self.assertTrue((runtime / 'widget_test_options.py').is_file())
            self.assertTrue((output / 'tools/check_background_telemetry.py').is_file())

            for row in [*manifest['files'], manifest['binary']]:
                device = Path(row['device_path'])
                if trial.SDK in device.parents:
                    staged = overlay / device.relative_to(trial.SDK)
                else:
                    staged = runtime / device.relative_to(trial.RUNTIME)
                row['device_path'] = str(staged)
            installed = Path(manifest['binary']['device_path'])
            installed.parent.mkdir(parents=True)
            installed.write_bytes(binary.read_bytes())
            manifest_path.write_text(json.dumps(manifest))

            def audit():
                return deployment_status(manifest_path, sdk_root=overlay, runtime_root=runtime, proc=proc)

            self.assertTrue(audit()['ok'])
            self.assertEqual(audit()['running']['status'], 'not_running')
            layout_path = overlay / trial.WIDGET_RELATIVE / 'widget_file/cn_big_screen/widget_config.json'
            layout = json.loads(layout_path.read_text())
            next(row for row in layout['main_interface']['widget_list']
                 if row['widget_index'] == 83)['list_item'].reverse()
            layout_path.write_text(json.dumps(layout))
            self.assertIn('widget_layout_or_icon_invalid', audit()['issues'])


if __name__ == '__main__':
    unittest.main()
