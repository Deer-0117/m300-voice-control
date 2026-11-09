#!/usr/bin/env python3
"""Stage Code10 project sources and an optional deployment audit manifest."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile

CONTROLS = Path(__file__).resolve().parents[1]
RUNTIME_SOURCE = CONTROLS.parent / 'recognition/runtime'
sys.path.insert(0, str(RUNTIME_SOURCE))

from flight_panel_tools import RUNTIME, SDK, WIDGET_RELATIVE, WIDGET_SCHEMA, digest

V7 = CONTROLS / 'deployment/flight_panel_manifest.json'
RELEASE = 'flight-panel-conti-code10-20261109'


def stage_sources(output, binary=None):
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise ValueError(f'Output already exists: {output}')
    old_manifest = json.loads(V7.read_text(encoding='utf-8'))
    paths = {Path(row['device_path']) for row in old_manifest['files']}
    paths.remove(RUNTIME / 'detector_config.json')  # active detector is device state
    paths.update(SDK / WIDGET_RELATIVE / name for name in (
        'balloon_background_telemetry.h', 'balloon_background_telemetry_impl.h',
    ))
    paths.update(RUNTIME / name for name in (
        'widget_test_options.py', 'analyze_background_telemetry.py', 'finish_flight_download.py',
    ))
    binary_sha = None
    if binary is not None:
        with binary.open('rb') as stream:
            header = stream.read(20)
        if header[:6] != b'\x7fELF\x02\x01' or header[18:20] != b'\xb7\x00':
            raise ValueError('Expected the rebuilt Linux ARM64 SDK binary')
        binary_sha = digest(binary)

    with tempfile.TemporaryDirectory(prefix='.code10-stage-', dir=output.parent) as temp:
        root = Path(temp) / 'sources'
        rows = []

        def copy(source, destination):
            if not source.is_file() or source.is_symlink():
                raise ValueError(f'Expected a regular source file: {source}')
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)

        for device in sorted(paths):
            if SDK in device.parents:
                relative = device.relative_to(SDK)
                source = CONTROLS / 'psdk-overlay' / relative
                target = root / 'psdk-overlay' / relative
            elif RUNTIME in device.parents:
                relative = device.relative_to(RUNTIME)
                source = RUNTIME_SOURCE / relative
                target = root / 'runtime' / relative
            else:
                raise ValueError(f'Unexpected device path: {device}')
            copy(source, target)
            rows.append({'device_path': str(device), 'sha256': digest(target)})
        for lang in ('cn', 'en'):
            folder = Path('psdk-overlay') / WIDGET_RELATIVE / f'widget_file/{lang}_big_screen'
            for icon in sorted((CONTROLS / folder).glob('*.png')):
                copy(icon, root / folder / icon.name)
        for name in ('check_background_telemetry.py', 'collect_flight_results.py'):
            copy(CONTROLS / 'scripts' / name, root / 'tools' / name)

        manifest = {'release_id': RELEASE, 'files': rows, 'widget_schema': WIDGET_SCHEMA}
        if binary_sha is None:
            target = root / 'source_manifest.json'
        else:
            manifest['binary'] = dict(old_manifest['binary'], sha256=binary_sha)
            target = root / 'runtime/flight_panel_manifest.json'
        target.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        root.rename(output)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path, help='new directory for the staged files')
    parser.add_argument('--binary', type=Path, help='ARM64 executable rebuilt from these exact sources')
    args = parser.parse_args()
    try:
        print(stage_sources(args.output, args.binary))
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'Code10 staging failed: {exc}\n')


if __name__ == '__main__':
    main()
