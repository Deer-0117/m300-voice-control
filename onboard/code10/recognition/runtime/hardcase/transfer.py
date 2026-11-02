"""Verified stopped-batch transfer and explicit, resumable quota reclamation.

Receipts record an operator's assertion, not a cryptographic identity or proof
that processing succeeded. Downloaded raw runs must remain immutable; processing
belongs in a separate working copy.
"""
from contextlib import contextmanager, nullcontext
from datetime import datetime, timezone
import hashlib
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import stat

from .dataset import canonical, new_output, read_json, sha256
from .storage import IntegrityError, RunStore, StorageUnavailable, _fsync_directory, _reject_links


def _root(path):
    path = Path(os.path.abspath(path))
    _reject_links(path)
    return path


def _relative(value):
    if (not isinstance(value, str) or not value or '\\' in value or ':' in value
            or PureWindowsPath(value).drive or value.startswith('/')
            or any(part in ('', '.', '..') or part.endswith(('.', ' '))
                   for part in value.split('/'))):
        raise IntegrityError('unsafe relative inventory path')
    return value


def _run_path(root, run_id, area='runs'):
    _relative(run_id)
    if '/' in run_id:
        raise IntegrityError('run ID must be a single path component')
    return _root(root / area / run_id)


def _digest_file(path):
    _reject_links(path)
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_BINARY', 0))
    with os.fdopen(fd, 'rb') as reader:
        before = os.fstat(reader.fileno())
        if not stat.S_ISREG(before.st_mode):
            raise IntegrityError('inventory contains a nonregular file')
        digest = hashlib.sha256()
        size = 0
        while True:
            chunk = reader.read(1024 * 1024)
            if not chunk:
                break
            digest.update(chunk)
            size += len(chunk)
        after = os.fstat(reader.fileno())
        _reject_links(path)
        current = path.stat()
        if (size != before.st_size or
                (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) !=
                (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns) or
                (current.st_dev, current.st_ino, current.st_size, current.st_mtime_ns) !=
                (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)):
            raise IntegrityError('file changed while creating inventory')
        return {'size': size, 'sha256': digest.hexdigest()}


def _inventory(root):
    root = _root(root)
    if not root.is_dir():
        raise IntegrityError('batch directory is missing')
    files, directories = [], []
    pending = [root]
    while pending:
        directory = pending.pop()
        _reject_links(directory)
        for path in sorted(directory.iterdir()):
            _reject_links(path)
            relative = _relative(path.relative_to(root).as_posix())
            info = path.lstat()
            if relative == 'writer.lock' and not stat.S_ISREG(info.st_mode):
                raise IntegrityError('writer lock must be a regular file')
            if stat.S_ISDIR(info.st_mode):
                directories.append(relative)
                pending.append(path)
            elif stat.S_ISREG(info.st_mode):
                if relative != 'writer.lock':
                    files.append(dict(path=relative, **_digest_file(path)))
            else:
                raise IntegrityError('inventory contains an unsupported file type')
    return sorted(files, key=lambda item: item['path']), sorted(directories)


def _sealed(value, field):
    return dict(value, **{field: sha256(canonical(value))})


def _validate_digest(value, field):
    if not isinstance(value, dict) or field not in value:
        raise IntegrityError('missing ' + field)
    unsigned = {key: item for key, item in value.items() if key != field}
    if sha256(canonical(unsigned)) != value[field]:
        raise IntegrityError('invalid ' + field)


def _manifest(value):
    _validate_digest(value, 'manifest_sha256')
    if value.get('schema_version') != 1 or not isinstance(value.get('pool_id'), str):
        raise IntegrityError('invalid manifest schema or pool identity')
    _relative(value.get('run_id'))
    if '/' in value['run_id']:
        raise IntegrityError('invalid manifest run ID')
    source = value.get('source')
    if (not isinstance(source, dict) or set(source) != {'run_path', 'data_root'}
            or any(not isinstance(path, str) or not path for path in source.values())):
        raise IntegrityError('manifest lacks original capture location')
    path_type = PureWindowsPath if PureWindowsPath(source['data_root']).drive else PurePosixPath
    origin_root, origin_run = path_type(source['data_root']), path_type(source['run_path'])
    if (not origin_root.is_absolute() or not origin_run.is_absolute()
            or origin_run != origin_root / 'runs' / value['run_id']):
        raise IntegrityError('invalid original capture location')
    files, directories = value.get('files'), value.get('directories')
    if not isinstance(files, list) or not isinstance(directories, list):
        raise IntegrityError('invalid inventory')
    paths = []
    for item in files:
        if not isinstance(item, dict) or set(item) != {'path', 'size', 'sha256'}:
            raise IntegrityError('invalid inventory file')
        path = _relative(item['path'])
        if path == 'writer.lock' or type(item['size']) is not int or item['size'] < 0:
            raise IntegrityError('invalid inventory file')
        digest = item['sha256']
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise IntegrityError('invalid file digest')
        paths.append(path)
    for directory in directories:
        _relative(directory)
        if directory == 'writer.lock':
            raise IntegrityError('writer lock cannot be a directory')
    if (paths != sorted(set(paths)) or directories != sorted(set(directories))
            or 'index.sqlite3' not in paths or set(paths) & set(directories)):
        raise IntegrityError('invalid inventory ordering, duplicates or database')
    for path in paths + directories:
        parent = path.rpartition('/')[0]
        if parent and parent not in directories:
            raise IntegrityError('inventory directory is missing')
    return value


def _verify(root, manifest, *, partial=False):
    files, directories = _inventory(root)
    expected = {item['path']: item for item in manifest['files']}
    actual = {item['path']: item for item in files}
    if (any(expected.get(path) != item for path, item in actual.items())
            or not set(directories) <= set(manifest['directories'])
            or (not partial and (set(actual) != set(expected)
                                 or directories != manifest['directories']))):
        raise IntegrityError('batch inventory has missing, changed, or extra evidence')
    return files, directories


def _write_new(path, report):
    _reject_links(path)
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0), 0o600)
    with os.fdopen(fd, 'wb') as writer:
        writer.write(canonical(report) + b'\n')
        writer.flush()
        os.fsync(writer.fileno())
    _fsync_directory(path.parent)


@contextmanager
def _stopped_lock(root):
    """Lock an existing OS lock without changing downloaded raw bytes."""
    path = root / 'writer.lock'
    _reject_links(path)
    if not path.exists():
        # A raw copy may omit the platform-specific lock, which is not evidence.
        yield
        return
    fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_BINARY', 0))
    with os.fdopen(fd, 'rb', buffering=0) as handle:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise IntegrityError('writer lock must be a regular file')
        try:
            if os.name == 'nt':
                import msvcrt
                msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise StorageUnavailable('run already has an active writer') from exc
        try:
            yield
        finally:
            if os.name == 'nt':
                handle.seek(0)
                msvcrt.locking(fd, msvcrt.LK_UNLCK, 1)
            else:
                fcntl.flock(fd, fcntl.LOCK_UN)


def _validate_run(root, run_id, *, lock=False):
    with _stopped_lock(root) if lock else nullcontext():
        with RunStore.open(root, read_only=True) as store:
            if store.run_info.get('run_id') != run_id:
                raise IntegrityError('run identity differs from requested batch')
            if store.statistics()['inflight']:
                raise StorageUnavailable('unfinished admissions require recovery before transfer')
            return _inventory(root)


def _original_location(root, source, manifest):
    """Sealed capture authority cannot be rebound by moving/copying its ledger."""
    original = manifest['source']
    original_root, original_run = Path(original['data_root']), Path(original['run_path'])
    if (not original_root.is_absolute() or not original_run.is_absolute()
            or original_root != root or original_run != source):
        raise IntegrityError('sealed batch requires its original capture location; relocated roots are not eligible')


def prepare_transfer(data_root, run_id, output):
    """Seal a stopped run and export its frozen manifest to a new JSON file."""
    from .retention import RetentionPool
    root = _root(data_root)
    source = _run_path(root, run_id)
    destination = new_output(output, [root])
    with RetentionPool(root) as pool, _stopped_lock(source):
        record = pool.run_record(run_id)
        if record is None or record['state'] not in ('open', 'sealed'):
            raise IntegrityError('only an open or sealed stopped run can be transferred')
        files, directories = _validate_run(source, run_id)
        if record['state'] == 'sealed':
            manifest = _manifest(record['manifest'])
            _original_location(root, source, manifest)
            _verify(source, manifest)
        else:
            manifest = _sealed({'schema_version': 1, 'pool_id': pool.pool_id,
                                'run_id': run_id, 'files': files,
                                'directories': directories,
                                'source': {'data_root': str(root), 'run_path': str(source)}},
                               'manifest_sha256')
            pool.set_transfer_manifest(run_id, manifest)
        _write_new(destination, manifest)
        return manifest


def _download_location(root, manifest):
    """A receipt requires a raw copy outside the original capture directory."""
    source = manifest['source']
    original_root, original_run = Path(source['data_root']), Path(source['run_path'])
    # An origin path from another OS is opaque on this ground computer.
    if original_root.is_absolute():
        if root.resolve().is_relative_to(original_root.resolve()):
            raise IntegrityError('confirmation requires a downloaded raw copy outside the capture root')
        try:
            if root.samefile(original_run):
                raise IntegrityError('the original capture is not a downloaded copy')
        except FileNotFoundError:
            pass


def confirm_processed(run, manifest_path, output, *, processed_by, processing_note):
    """Verify downloaded raw evidence and record explicit processing completion."""
    for value in (processed_by, processing_note):
        if not isinstance(value, str) or not value.strip():
            raise ValueError('processing operator and completion note are required')
    root = _root(run)
    destination = new_output(output, [root])
    manifest = _manifest(read_json(_root(manifest_path)))
    _download_location(root, manifest)
    with _stopped_lock(root):
        _verify(root, manifest)
        _validate_run(root, manifest['run_id'])
        receipt = _sealed({'schema_version': 1, 'pool_id': manifest['pool_id'],
                           'run_id': manifest['run_id'],
                           'manifest_sha256': manifest['manifest_sha256'],
                           'inventory_sha256': sha256(canonical(manifest['files'])),
                           'processed_by': processed_by.strip(),
                           'processing_note': processing_note.strip(),
                           'processed_at': datetime.now(timezone.utc).isoformat(),
                           'processing_completed': True}, 'receipt_sha256')
        _write_new(destination, receipt)
    return receipt


def _receipt(value, manifest):
    _validate_digest(value, 'receipt_sha256')
    if (value.get('schema_version') != 1 or value.get('processing_completed') is not True
            or any(value.get(key) != manifest[key] for key in ('pool_id', 'run_id', 'manifest_sha256'))
            or value.get('inventory_sha256') != sha256(canonical(manifest['files']))
            or any(not isinstance(value.get(key), str) or not value[key].strip()
                   for key in ('processed_by', 'processing_note', 'processed_at'))):
        raise IntegrityError('receipt does not confirm this frozen batch')
    return value


def _delete_file(path, expected=None):
    _reject_links(path)
    if not stat.S_ISREG(path.lstat().st_mode):
        raise IntegrityError('refusing to delete a nonregular file')
    if expected is not None and _digest_file(path) != {k: expected[k] for k in ('size', 'sha256')}:
        raise IntegrityError('evidence changed before deletion')
    path.unlink()
    _fsync_directory(path.parent)


def _delete_batch(staged, manifest):
    files, directories = _verify(staged, manifest, partial=True)
    for item in files:
        _delete_file(staged / item['path'], item)
    lock = staged / 'writer.lock'
    _reject_links(lock)
    if lock.exists():
        _delete_file(lock)
    for relative in sorted(directories, key=lambda value: (value.count('/'), value), reverse=True):
        directory = staged / relative
        _reject_links(directory)
        directory.rmdir()
        _fsync_directory(directory.parent)
    _reject_links(staged)
    staged.rmdir()
    _fsync_directory(staged.parent)


def cleanup_run(data_root, run_id, receipt_path, *, execute=False):
    """Preview by default; execute deletes only verified inventoried batch paths."""
    from .retention import RetentionPool
    if type(execute) is not bool:
        raise ValueError('execute must be an explicit boolean')
    root = _root(data_root)
    source = _run_path(root, run_id)
    staged = _run_path(root, run_id, '.cleanup')
    receipt = read_json(_root(receipt_path))
    with RetentionPool(root, create=False) as pool:
        record = pool.run_record(run_id)
        if record is None or record['state'] not in ('sealed', 'cleaning', 'cleared'):
            raise IntegrityError('cleanup requires a frozen transfer manifest')
        manifest = _manifest(record['manifest'])
        _original_location(root, source, manifest)
        if manifest['pool_id'] != pool.pool_id or manifest['run_id'] != run_id:
            raise IntegrityError('manifest does not belong to this pool and run')
        _receipt(receipt, manifest)
        if record['state'] in ('cleaning', 'cleared') and record['receipt'] != receipt:
            raise IntegrityError('cleanup must resume with the same receipt')
        source_exists, staged_exists = source.exists(), staged.exists()
        if source_exists and staged_exists:
            raise IntegrityError('both source and cleanup staging batch exist')
        if record['state'] == 'cleared':
            if source_exists or staged_exists:
                raise IntegrityError('cleared batch unexpectedly reappeared')
            files = []
        elif source_exists:
            _validate_run(source, run_id, lock=True)
            files, _ = _verify(source, manifest)
        elif staged_exists and record['state'] == 'cleaning':
            files, _ = _verify(staged, manifest, partial=True)
        elif record['state'] == 'cleaning':
            files = []
        else:
            raise IntegrityError('sealed source batch is missing or unexpectedly staged')
        report = {'run_id': run_id, 'state': record['state'], 'execute': bool(execute),
                  'files': len(files), 'bytes': sum(item['size'] for item in files),
                  'released': 0}
        if execute and record['state'] != 'cleared':
            pool.begin_cleanup(run_id, receipt)
            if source_exists:
                _reject_links(staged.parent)
                staged.parent.mkdir(exist_ok=True)
                _fsync_directory(root)
                _reject_links(source)
                _reject_links(staged)
                if staged.exists():
                    raise IntegrityError('cleanup staging path already exists')
                os.rename(source, staged)
                _fsync_directory(source.parent)
                _fsync_directory(staged.parent)
                staged_exists = True
            if staged_exists:
                _delete_batch(staged, manifest)
            # A previous attempt may have removed the final directory but
            # failed before its parent was synced. Retry that durability step
            # even when there are no remaining files to delete.
            for parent in (source.parent, staged.parent):
                _reject_links(parent)
                if parent.exists():
                    _fsync_directory(parent)
            pool.finish_cleanup(run_id)
            report.update(state='cleared', released=record['occupied'])
        report['retention'] = pool.statistics()
        return report


def retention_status(data_root):
    """Report durable quota, including reservations and unfinished cleanup."""
    from .retention import RetentionPool
    with RetentionPool(data_root, create=False) as pool:
        return pool.statistics()


def recover_run(data_root, run_id):
    """Explicitly settle interrupted local admissions without releasing slots."""
    from .retention import RetentionPool
    root = _root(data_root)
    source = _run_path(root, run_id)
    with RetentionPool(root) as pool:
        pool.ensure_writable(run_id)
        with RunStore.open(source, read_only=True) as before:
            inflight = before.statistics()['inflight']
        with RunStore.open(source, recover=True, _retention_pool=pool) as store:
            report = {'run_id': run_id, 'recovered_unknown': inflight,
                      'verification': store.verify_all(),
                      'statistics': store.statistics()}
        report['retention'] = pool.statistics()
        return report
