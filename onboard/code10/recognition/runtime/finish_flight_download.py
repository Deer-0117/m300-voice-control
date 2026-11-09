"""Verify evidence, summarize flight telemetry and replay videos when present."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import tarfile
import tempfile


def unpack(archive, destination):
    with tarfile.open(archive, 'r:gz') as source:
        for member in source:
            name = PurePosixPath(member.name)
            if (not name.parts or name.is_absolute() or '..' in name.parts
                    or chr(92) in member.name or ':' in member.name
                    or not (member.isfile() or member.isdir())):
                raise ValueError('unsafe archive member: ' + member.name)
            target = destination.joinpath(*name.parts)
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.extractfile(member) as data, target.open('xb') as out:
                    shutil.copyfileobj(data, out)


def verify(root):
    manifest_path = root / 'manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    files = manifest['files']
    actual = {p.relative_to(root).as_posix() for p in root.rglob('*')
              if p.is_file() and p != manifest_path}
    if files.keys() != actual:
        raise ValueError('archive file list does not match the manifest')
    for name, expected in files.items():
        path = root / name
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        if digest.hexdigest() != expected['sha256'] or path.stat().st_size != expected['bytes']:
            raise ValueError('file integrity failure: ' + name)
    return len(files)


def finish(archive, skip_replay=False):
    archive = Path(archive)
    destination = archive.parent / (archive.name.removesuffix('.tar.gz') + '_verified')
    if destination.exists():
        raise ValueError('output already exists; keep it and choose a fresh download directory')
    with tempfile.TemporaryDirectory(prefix='.flight-evidence-', dir=archive.parent) as temp:
        scratch = Path(temp)
        unpack(archive, scratch)
        roots = list(scratch.iterdir())
        if len(roots) != 1 or not (roots[0] / 'manifest.json').is_file():
            raise ValueError('expected one export manifest')
        verified_files = verify(roots[0])
        destination.mkdir()
        root = roots[0].rename(destination / roots[0].name)
    reports, errors = [], []
    sessions = sorted((root/'prediction_recordings').glob('run_*'))
    if sessions and not skip_replay:
        try:
            from replay_flight import replay
        except ImportError as exc:
            errors.append(dict(error='Replay dependency unavailable: '+str(exc)))
        else:
            for session in sessions:
                try:
                    output = destination / 'replays' / (session.name + '.mkv')
                    reports.append(replay(session, output=output))
                except Exception as exc:
                    errors.append(dict(session=session.name, error=str(exc)))
    telemetry_paths = sorted((root/'logs').glob('background_telemetry*.jsonl'))
    telemetry = None
    if telemetry_paths:
        try:
            from analyze_background_telemetry import analyze, format_text
            telemetry = analyze(telemetry_paths)
            (destination/'background_telemetry_report.json').write_text(
                json.dumps(telemetry, ensure_ascii=False, indent=2), encoding='utf-8')
            (destination/'background_telemetry_report.txt').write_text(
                format_text(telemetry), encoding='utf-8')
        except Exception as exc:
            errors.append(dict(error='Telemetry analysis failed: '+str(exc)))
    result = dict(root=str(root), verified_files=verified_files, replays=reports,
                  telemetry_report=str(destination/'background_telemetry_report.json') if telemetry is not None else None,
                  errors=errors)
    (destination/'download_processing.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))
    return 2 if errors else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('--skip-replay', action='store_true', help='verify and analyze telemetry without video replay')
    args = parser.parse_args(argv)
    try:
        return finish(args.archive, args.skip_replay)
    except (OSError, ValueError, KeyError, tarfile.TarError) as exc:
        parser.exit(2, f'Flight evidence verification failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
