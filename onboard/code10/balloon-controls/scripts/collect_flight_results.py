"""Collect Code10 telemetry and optional task recordings on Jetson."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import tarfile
import tempfile
import time
import uuid


RUNTIME = Path('/home/uav/offline_balloon_strike_test')
OUTPUT = Path('/home/uav')
BOOT_ID = Path('/proc/sys/kernel/random/boot_id')
RECORD_REQUEST = Path('/tmp/balloon_prediction_record.request')
MISSION_COMMIT = Path('/tmp/balloon_mission.commit')


def snapshot_background_telemetry(root, destination, warnings):
    """Pin rotated files and bound reads while the read-only recorder continues."""
    opened, inventory, seen = [], [], set()
    try:
        for index in range(3, -1, -1):
            name = f'background_telemetry.{index}.jsonl' if index else 'background_telemetry.jsonl'
            path = root / name
            stream = None
            try:
                if path.is_symlink():
                    raise ValueError('symbolic link is not a telemetry log')
                fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0))
                stream = os.fdopen(fd, 'rb')
                info = os.fstat(stream.fileno())
                if not stat.S_ISREG(info.st_mode):
                    raise ValueError('not a regular file')
                identity = (info.st_dev, info.st_ino)
                if identity in seen:
                    warnings.append('Telemetry rotated during export; duplicate inode skipped: ' + path.name)
                    stream.close()
                    continue
                seen.add(identity)
                opened.append((path.name, stream, info.st_size))
            except FileNotFoundError:
                continue
            except (OSError, ValueError) as exc:
                if stream is not None:
                    stream.close()
                warnings.append('Could not open telemetry snapshot ' + path.name + ': ' + str(exc))
        for name, stream, size in opened:
            remaining = size
            with (destination / name).open('wb') as out:
                while remaining:
                    chunk = stream.read(min(1024 * 1024, remaining))
                    if not chunk:
                        warnings.append('Telemetry shortened during export: ' + name)
                        break
                    out.write(chunk)
                    remaining -= len(chunk)
            inventory.append({'file': 'logs/' + name, 'snapshot_bytes': size - remaining})
    finally:
        for _, stream, _ in opened:
            stream.close()
    return inventory


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def recording_busy(boot_id):
    if RECORD_REQUEST.exists() or MISSION_COMMIT.exists():
        return True
    try:
        ack = json.loads(RECORD_REQUEST.with_name('balloon_prediction_record_ack.json').read_text())
    except (OSError, ValueError):
        return False
    return (isinstance(ack, dict) and ack.get('clock_id') == 'mono-v1:' + boot_id
            and ack.get('status') == 'recording')


def collect(root, output_dir, boot_id, telemetry_only=False):
    if not root.is_dir():
        raise ValueError(f'Code10 runtime directory does not exist: {root}')
    boot = uuid.UUID(boot_id)
    if recording_busy(str(boot)):
        raise ValueError('Stop the active task and recording before exporting evidence.')
    clock_id = 'mono-v1:' + str(boot)
    export_id = 'flight_results_' + boot.hex[:8] + '_' + uuid.uuid4().hex[:10]
    output = output_dir / (export_id + '.tar.gz')
    warnings = []
    sessions = []
    with tempfile.TemporaryDirectory(prefix=export_id + '_', dir=output_dir) as scratch:
        scratch = Path(scratch)
        evidence = scratch / export_id
        logs = evidence / 'logs'
        logs.mkdir(parents=True)
        telemetry = snapshot_background_telemetry(root, logs, warnings)
        if not telemetry_only:
            for session in sorted((root / 'prediction_recordings').glob('run_*')):
                if not session.is_dir() or session.is_symlink():
                    continue
                shutil.copytree(session, evidence / 'prediction_recordings' / session.name,
                    ignore=lambda folder, names: [n for n in names if (Path(folder) / n).is_symlink()])
                sessions.append(session.name)
        if recording_busy(str(boot)):
            raise ValueError('A task or recording started during export. Stop it and retry.')
        if not telemetry and not sessions:
            raise ValueError('No telemetry or selected recordings are available.')
        if not telemetry:
            warnings.append('No background telemetry files were available.')
        if not telemetry_only and not sessions:
            warnings.append('No task recording was available. The archive contains telemetry.')

        files = {p.relative_to(evidence).as_posix(): {'bytes': p.stat().st_size, 'sha256': sha256(p)}
                 for p in sorted(evidence.rglob('*')) if p.is_file()}
        manifest = {
            'export_id': export_id,
            'clock_id': clock_id,
            'collection_monotonic_ms': time.monotonic_ns() // 1_000_000,
            'selection': 'telemetry_only' if telemetry_only else 'telemetry_and_recordings',
            'sessions': sessions,
            'background_telemetry': telemetry,
            'warnings': warnings,
            'files': files,
        }
        (evidence / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        partial = scratch / 'archive.tar.gz'
        with tarfile.open(partial, 'w:gz', compresslevel=1) as archive:
            archive.add(evidence, arcname=export_id)
        size = partial.stat().st_size
        checksum = sha256(partial)
        partial.replace(output)
    return {'archive': str(output), 'sha256': checksum, 'bytes': size,
            'sessions': sessions, 'warnings': warnings}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--telemetry-only', action='store_true', help='omit task recordings')
    args = parser.parse_args(argv)
    try:
        result = collect(RUNTIME, OUTPUT, BOOT_ID.read_text().strip(), args.telemetry_only)
    except (OSError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
