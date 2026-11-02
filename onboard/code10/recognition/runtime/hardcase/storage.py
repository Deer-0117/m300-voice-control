"""Single-writer, crash-recoverable local evidence store.

SQLite is the authority. Images and sidecar revisions are immutable; unindexed
files are reported, never promoted or deleted by recovery. Filesystem fsync and
SQLite FULL cannot substitute for testing the target disk's power-loss behavior.
"""

import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import shutil
import sqlite3
import stat
import threading
import uuid

from .image import InvalidImage, MAX_BYTES, validate_jpeg


class IntegrityError(RuntimeError):
    pass


class IdentityConflict(IntegrityError):
    pass


class QuotaExceeded(RuntimeError):
    pass


class StorageUnavailable(RuntimeError):
    pass


def _json(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(',', ':'), allow_nan=False)


def _hash(data):
    return hashlib.sha256(data).hexdigest()


def _key(value):
    if (not isinstance(value, (tuple, list)) or len(value) != 4
            or any(not isinstance(v, str) or not v or len(v) > 128 for v in value[:3])
            or type(value[3]) is not int or not 0 <= value[3] <= 999999999999):
        raise IdentityConflict('invalid frame identity')
    return _json(list(value))


def _reject_links(path):
    """Check existing ancestors as well as the leaf (including Windows junctions)."""
    for part in reversed((path,) + tuple(path.parents)):
        try:
            info = part.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
            raise IntegrityError('symlink/reparse point is not allowed: ' + str(part))


def _fsync_directory(path):
    if os.name != 'nt':
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0) | getattr(os, 'O_NOFOLLOW', 0))
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def _merge_additive(old, new):
    if old is None:
        return new
    if new is None or old == new:
        return old
    if isinstance(old, dict) and isinstance(new, dict):
        result = dict(old)
        for name, value in new.items():
            result[name] = _merge_additive(result.get(name), value)
        return result
    raise IdentityConflict('conflicting immutable metadata')


_SCHEMA = """
CREATE TABLE runs (run_id TEXT PRIMARY KEY, record_json TEXT NOT NULL, policy_json TEXT NOT NULL);
CREATE TABLE frames (
 sample_id TEXT PRIMARY KEY, key_json TEXT NOT NULL UNIQUE, image_path TEXT NOT NULL,
 image_sha256 TEXT NOT NULL, image_bytes INTEGER NOT NULL, width INTEGER NOT NULL,
 height INTEGER NOT NULL, quota_bucket TEXT NOT NULL CHECK(quota_bucket IN ('reserved','shared')),
 latest_revision INTEGER NOT NULL);
CREATE TABLE detections (
 detection_key TEXT PRIMARY KEY, sample_id TEXT NOT NULL REFERENCES frames(sample_id),
 record_json TEXT NOT NULL);
CREATE TABLE sample_revisions (
 sample_id TEXT NOT NULL REFERENCES frames(sample_id), revision INTEGER NOT NULL,
 sidecar_path TEXT NOT NULL UNIQUE, sidecar_sha256 TEXT NOT NULL, record_json TEXT NOT NULL,
 PRIMARY KEY(sample_id,revision));
CREATE TABLE events (event_id TEXT PRIMARY KEY, record_json TEXT NOT NULL);
CREATE TABLE admission_attempts (
 sample_id TEXT PRIMARY KEY, key_json TEXT NOT NULL UNIQUE, quota_bucket TEXT NOT NULL,
 state TEXT NOT NULL CHECK(state IN ('accepted','committed','terminal_failed')),
 failure TEXT, reserved_bytes INTEGER NOT NULL, metadata_bytes INTEGER NOT NULL);
CREATE TABLE counters (name TEXT PRIMARY KEY, value INTEGER NOT NULL);
CREATE TABLE human_reviews (review_id TEXT PRIMARY KEY, sample_id TEXT NOT NULL REFERENCES frames(sample_id), record_json TEXT NOT NULL);
CREATE TABLE exports (export_id TEXT PRIMARY KEY, record_json TEXT NOT NULL);
"""


class RunStore:
    """Use a writable instance on the thread that created/opened it.

    ``fault(stage)`` may raise at documented durability boundaries. BaseException
    simulates abrupt process loss; ordinary failures settle accepted admissions
    as terminal failures where the filesystem remains writable.
    """

    @classmethod
    def create(cls, root, run_info, *, total=3000, uniform_reserved=600,
               min_free_bytes=10 * 1024 ** 3, min_free_ratio=.1,
               expected_mount=None, expected_device=None,
               max_metadata_bytes=1024 ** 3, fault=None, _retention_pool=None):
        root = Path(os.path.abspath(root))
        _reject_links(root)
        if not isinstance(run_info, dict):
            raise ValueError('run_info must be an object')
        _json(run_info)
        policy = dict(total=total, uniform_reserved=uniform_reserved,
                      min_free_bytes=min_free_bytes, min_free_ratio=min_free_ratio,
                      expected_mount=str(expected_mount) if expected_mount is not None else None,
                      expected_device=expected_device, max_metadata_bytes=max_metadata_bytes)
        cls._validate_policy(policy)
        from .retention import assert_run_write_allowed
        assert_run_write_allowed(root, pool=_retention_pool)
        root.mkdir(mode=0o700)  # Never reuse, truncate, or replace an existing run.
        instance = cls(root, False, fault)
        instance._retention_pool = _retention_pool
        try:
            instance._set_policy(policy)
            instance._check_storage(32768)
            instance._lock()
            for directory in ('images', 'revisions'):
                instance._check_storage()
                (root / directory).mkdir(mode=0o700)
            instance._check_storage()
            instance.db = sqlite3.connect(str(instance._path('index.sqlite3')))
            instance._configure()
            instance.db.executescript(_SCHEMA)
            instance.db.execute('INSERT INTO runs VALUES (?,?,?)',
                                (str(run_info.get('run_id', 'unknown')), _json(run_info), _json(instance.policy)))
            instance.db.execute("INSERT INTO counters VALUES ('metadata_bytes',0)")
            instance.db.commit()
            instance._remember_database()
            instance.run_info = json.loads(_json(run_info))
            _fsync_directory(root)
            _fsync_directory(root.parent)
            return instance
        except BaseException:
            instance.close()
            raise

    @classmethod
    def open(cls, root, *, recover=False, read_only=False, ground_copy=False, total=None,
             uniform_reserved=None, min_free_bytes=None, min_free_ratio=None,
             expected_mount=None, expected_device=None, max_metadata_bytes=None, fault=None,
             _retention_pool=None):
        root = Path(os.path.abspath(root))
        _reject_links(root)
        if not root.is_dir():
            raise StorageUnavailable('run directory does not exist')
        if (read_only or ground_copy) and recover:
            raise ValueError('recovery requires the acquisition store')
        if read_only and ground_copy:
            raise ValueError('choose either read_only or ground_copy')
        instance = cls(root, read_only, fault)
        instance._retention_pool = _retention_pool
        instance.ground_copy = ground_copy
        try:
            db_path = instance._path('index.sqlite3')
            if not db_path.is_file():
                raise IntegrityError('run database is missing')
            if not read_only:
                instance._lock()
            instance.db = sqlite3.connect(db_path.as_uri() + ('?mode=ro' if read_only else '?mode=rw'), uri=True)
            instance.db.execute('PRAGMA foreign_keys=ON')
            integrity = instance.db.execute('PRAGMA integrity_check').fetchall()
            if integrity != [('ok',)]:
                raise IntegrityError('SQLite integrity check failed')
            rows = instance.db.execute('SELECT record_json,policy_json FROM runs').fetchall()
            if len(rows) != 1:
                raise IntegrityError('expected exactly one run')
            instance.run_info, policy = map(json.loads, rows[0])
            overrides = dict(total=total, uniform_reserved=uniform_reserved,
                             min_free_bytes=min_free_bytes, min_free_ratio=min_free_ratio,
                             expected_mount=expected_mount, expected_device=expected_device,
                             max_metadata_bytes=max_metadata_bytes)
            for name, value in overrides.items():
                if value is not None:
                    if name in ('total', 'uniform_reserved') and value != policy[name]:
                        raise IntegrityError('cannot change quota allocation of an existing run')
                    policy[name] = str(value) if name == 'expected_mount' else value
            instance._set_policy(policy)
            instance._remember_database()
            instance._check_storage(check_space=False)
            instance._configure()
            report = instance.verify_all()
            if not report['ok']:
                raise IntegrityError('; '.join(report['errors'][:5]))
            if recover:
                instance.recover()
            elif not read_only and not ground_copy and instance.statistics()['inflight']:
                raise StorageUnavailable('unfinished admissions require explicit recover=True')
            if ground_copy:
                instance.db.set_authorizer(instance._ground_authorizer)
            return instance
        except sqlite3.DatabaseError as exc:
            instance.close()
            raise IntegrityError('invalid SQLite authority') from exc
        except BaseException:
            instance.close()
            raise

    def __init__(self, root, read_only, fault):
        self.root = root
        self.read_only = read_only
        self.ground_copy = False
        self.fault = fault
        self.db = None
        self._lock_file = None
        self._thread = threading.get_ident()
        self._blocked = False
        self._db_identity = None
        self._retention_pool = None
        info = root.stat()
        self._root_identity = (info.st_dev, info.st_ino)

    @staticmethod
    def _validate_policy(policy):
        for name in ('total', 'uniform_reserved', 'min_free_bytes', 'max_metadata_bytes'):
            if type(policy[name]) is not int or policy[name] < 0:
                raise ValueError('invalid storage policy: ' + name)
        if (policy['uniform_reserved'] > policy['total']
                or not isinstance(policy['min_free_ratio'], (float, int))
                or not 0 <= policy['min_free_ratio'] <= 1):
            raise ValueError('invalid quota or free-space ratio')

    def _set_policy(self, policy):
        self._validate_policy(policy)
        self.policy = dict(policy)
        for name in ('total', 'uniform_reserved', 'min_free_bytes', 'min_free_ratio', 'max_metadata_bytes'):
            setattr(self, name, policy[name])
        self.expected_device = policy['expected_device']
        if self.expected_device is None:
            self.expected_device = self.root.stat().st_dev
        if isinstance(self.expected_device, str) and self.expected_device.startswith('/dev/'):
            self.expected_device = os.stat(self.expected_device).st_rdev
        self.expected_device = int(self.expected_device)
        self.expected_mount = Path(os.path.abspath(policy['expected_mount'])) if policy['expected_mount'] else None
        self.policy['expected_device'] = self.expected_device

    def _path(self, relative):
        if (not isinstance(relative, str) or not relative or '\\' in relative
                or PureWindowsPath(relative).drive or PurePosixPath(relative).is_absolute()
                or any(part in ('..', '.') for part in relative.split('/'))):
            raise IntegrityError('invalid relative evidence path')
        path = self.root.joinpath(*relative.split('/'))
        try:
            path.relative_to(self.root)
        except ValueError as exc:
            raise IntegrityError('evidence path escapes root') from exc
        _reject_links(path)
        return path

    def _lock(self):
        path = self._path('writer.lock')
        fd = os.open(path, os.O_CREAT | os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_BINARY', 0), 0o600)
        handle = os.fdopen(fd, 'r+b', buffering=0)
        try:
            if os.name == 'nt':
                import msvcrt
                if os.fstat(fd).st_size == 0:
                    handle.write(b'\0')
                handle.seek(0)
                msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_file = handle
            if not self.read_only:
                # Maintenance seals the root ledger while holding this same
                # per-run lock. Check after acquisition so raw/legacy APIs
                # cannot reopen a frozen batch in the rename/delete gap.
                from .retention import assert_run_write_allowed
                assert_run_write_allowed(self.root, pool=self._retention_pool)
        except OSError as exc:
            handle.close()
            raise StorageUnavailable('run already has a writer') from exc

    def _configure(self):
        self.db.execute('PRAGMA foreign_keys=ON')
        if not self.read_only:
            self.db.execute('PRAGMA journal_mode=DELETE')
            self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('PRAGMA busy_timeout=1000')

    def _remember_database(self):
        info = self._path('index.sqlite3').stat()
        self._db_identity = (info.st_dev, info.st_ino)

    @staticmethod
    def _ground_authorizer(action, first, second, database, trigger):
        """A copied run may gain review/export records, never new flight evidence."""
        if action in (sqlite3.SQLITE_INSERT, sqlite3.SQLITE_UPDATE, sqlite3.SQLITE_DELETE):
            return sqlite3.SQLITE_OK if first in ('human_reviews', 'exports') else sqlite3.SQLITE_DENY
        if action in (sqlite3.SQLITE_SELECT, sqlite3.SQLITE_READ, sqlite3.SQLITE_FUNCTION,
                      sqlite3.SQLITE_TRANSACTION, sqlite3.SQLITE_SAVEPOINT, sqlite3.SQLITE_RECURSIVE):
            return sqlite3.SQLITE_OK
        if action == sqlite3.SQLITE_PRAGMA and first in ('integrity_check', 'foreign_key_check'):
            return sqlite3.SQLITE_OK
        return sqlite3.SQLITE_DENY

    def _check_storage(self, reservation=0, *, check_space=True):
        try:
            _reject_links(self.root)
            info = self.root.stat()
            if (info.st_dev, info.st_ino) != self._root_identity:
                raise StorageUnavailable('storage device/root identity changed')
            acquisition = not (self.read_only or self.ground_copy)
            if acquisition and info.st_dev != self.expected_device:
                raise StorageUnavailable('storage device/root identity changed')
            if self._db_identity is not None:
                db_info = self._path('index.sqlite3').stat()
                if (db_info.st_dev, db_info.st_ino) != self._db_identity:
                    raise StorageUnavailable('SQLite authority was replaced')
            if acquisition and self.expected_mount is not None:
                _reject_links(self.expected_mount)
                self.root.relative_to(self.expected_mount)
                if not os.path.ismount(self.expected_mount) or self.expected_mount.stat().st_dev != self.expected_device:
                    raise StorageUnavailable('expected mount is unavailable')
            if check_space:
                usage = shutil.disk_usage(self.root)
                floor = max(self.min_free_bytes, usage.total * self.min_free_ratio)
                if usage.free - reservation < floor:
                    raise StorageUnavailable('free space after reservation is below safety floor')
        except (OSError, ValueError, IntegrityError) as exc:
            raise StorageUnavailable('storage is unavailable or unsafe') from exc

    def _writable(self):
        if self.read_only or self.ground_copy or self.db is None or self._blocked:
            raise StorageUnavailable('store is closed, read-only, or blocked by corruption')
        if threading.get_ident() != self._thread:
            raise StorageUnavailable('store must be used by its dedicated writer thread')
        if self._retention_pool is not None:
            self._retention_pool.ensure_writable(self.run_info['run_id'])
        self._check_storage()

    def _stage(self, name, fault=None):
        callback = fault if fault is not None else self.fault
        if callback is not None:
            callback(name)

    def _read(self, relative, limit):
        path = self._path(relative)
        fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_BINARY', 0))
        with os.fdopen(fd, 'rb') as handle:
            info = os.fstat(handle.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
                raise IntegrityError('invalid evidence file size/type')
            data = handle.read(limit + 1)
            if len(data) != info.st_size or len(data) > limit:
                raise IntegrityError('evidence file changed during read')
            return data

    def _publish(self, relative, data, kind, fault):
        path = self._path(relative)
        if path.exists():
            if self._read(relative, len(data)) != data:
                raise IntegrityError('immutable file content conflict')
            return
        self._check_storage(len(data))
        temporary = str(PurePosixPath(relative).parent / ('.tmp.' + uuid.uuid4().hex))
        temp_path = self._path(temporary)
        self._stage(kind + '.before_write', fault)
        fd = os.open(temp_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_BINARY', 0), 0o600)
        try:
            written = os.write(fd, data)
            if written != len(data):
                raise OSError('short write while saving ' + kind)
            self._stage(kind + '.after_write', fault)
            self._check_storage()
            self._stage(kind + '.before_fsync', fault)
            os.fsync(fd)
            self._stage(kind + '.after_fsync', fault)
        finally:
            os.close(fd)
        if self._read(temporary, len(data)) != data:
            raise IntegrityError('saved bytes differ from submitted evidence')
        self._check_storage()
        self._stage(kind + '.before_rename', fault)
        # A hard link publishes atomically without replace/overwrite semantics.
        try:
            os.link(temp_path, path, follow_symlinks=False)
        except FileExistsError:
            if self._read(relative, len(data)) != data:
                raise IntegrityError('immutable destination conflict')
        temp_path.unlink()
        self._stage(kind + '.after_rename', fault)
        self._check_storage()
        self._stage(kind + '.before_dir_fsync', fault)
        _fsync_directory(path.parent)
        self._stage(kind + '.after_dir_fsync', fault)

    @staticmethod
    def _detection_key(record):
        if not isinstance(record, dict):
            raise IdentityConflict('detection must be an object')
        try:
            return _key([record['run_id'], record['boot_id'], record['worker_session_id'], record['inference_seq']])
        except KeyError as exc:
            raise IdentityConflict('detection identity is missing') from exc

    @staticmethod
    def _event_id(record):
        value = record.get('event_id', record.get('id')) if isinstance(record, dict) else None
        if not isinstance(value, str) or not value or len(value) > 256:
            raise IdentityConflict('event identity is missing')
        return value

    @staticmethod
    def _merge_event(previous, incoming):
        if previous is None:
            return incoming
        old_revision = previous.get('event_revision', 0)
        new_revision = incoming.get('event_revision', 0)
        if any(type(v) is not int or v < 0 for v in (old_revision, new_revision)):
            raise IdentityConflict('invalid event revision')
        for field in ('id', 'event_id', 'center', 'source_mono_ns', 'reason', 'deadline'):
            if field in previous and field in incoming and previous[field] != incoming[field]:
                raise IdentityConflict('event identity conflict')
        if old_revision > new_revision:
            return previous
        # Old unversioned records remain readable. New collector snapshots are
        # monotonic revisions: 0 pending, 1 membership, 2 enqueue audit.
        if old_revision == new_revision and ('event_revision' in previous or 'event_revision' in incoming):
            return _merge_additive(previous, incoming)
        return dict(previous, **incoming)

    def _merge_metadata(self, sample_id, key, old, incoming):
        if not isinstance(incoming, dict):
            raise ValueError('metadata must be an object')
        incoming = json.loads(_json(incoming))
        result = dict(old)
        detection_records = {self._detection_key(item): item for item in old.get('detections', [])}
        new_detections = {}
        for record in incoming.get('detections', []):
            identity = self._detection_key(record)
            if json.loads(identity)[:2] != list(key[:2]):
                raise IdentityConflict('detection belongs to another run/boot')
            row = self.db.execute('SELECT sample_id,record_json FROM detections WHERE detection_key=?', (identity,)).fetchone()
            if row is not None and (row[0] != sample_id or row[1] != _json(record)):
                raise IdentityConflict('detection identity conflict')
            if identity in detection_records and detection_records[identity] != record:
                raise IdentityConflict('detection identity conflict')
            detection_records[identity] = record
            if row is None:
                new_detections[identity] = record
        if len(detection_records) > 8 or self.db.execute('SELECT COUNT(*) FROM detections').fetchone()[0] + len(new_detections) > 24000:
            raise QuotaExceeded('detection reference limit exceeded')
        event_records = {self._event_id(item): item for item in old.get('events', [])}
        event_updates = {}
        for event in incoming.get('events', []):
            identity = self._event_id(event)
            row = self.db.execute('SELECT record_json FROM events WHERE event_id=?', (identity,)).fetchone()
            latest = json.loads(row[0]) if row is not None else event_records.get(identity)
            merged_event = self._merge_event(latest, event)
            event_records[identity] = merged_event
            event_updates[identity] = merged_event
        if len(event_records) > 128:
            raise QuotaExceeded('event reference limit exceeded')
        if 'detections' in incoming or 'detections' in old:
            result['detections'] = list(detection_records.values())
        if 'events' in incoming or 'events' in old:
            result['events'] = list(event_records.values())
        for name, value in incoming.items():
            if name not in ('detections', 'events'):
                result[name] = _merge_additive(result.get(name), value)
        new_events = {identity for identity in event_records
                      if self.db.execute('SELECT 1 FROM events WHERE event_id=?', (identity,)).fetchone() is None}
        if self.db.execute('SELECT COUNT(*) FROM events').fetchone()[0] + len(new_events) > 10000:
            raise QuotaExceeded('run event limit exceeded')
        # Inherited sample snapshots are historical evidence, never fresh
        # updates to the authoritative event record.
        return result, new_detections, event_updates

    def _quota_bucket(self, kind):
        if kind not in ('uniform', 'event'):
            raise ValueError('kind must be uniform or event')
        stats = self.statistics()
        if kind == 'uniform' and stats['quota_reserved_used'] < self.uniform_reserved:
            return 'reserved'
        if stats['quota_shared_used'] < self.total - self.uniform_reserved:
            return 'shared'
        raise QuotaExceeded('sample quota exhausted')

    def _metadata_used(self):
        return self.db.execute("SELECT value FROM counters WHERE name='metadata_bytes'").fetchone()[0]

    def put(self, key, image_bytes, reasons, metadata=None, kind='event', *, fault=None):
        self._writable()
        key_json = _key(key)
        if any(self.run_info.get(name) is not None and self.run_info[name] != key[i]
               for i, name in enumerate(('run_id', 'boot_id'))):
            raise IdentityConflict('frame belongs to a different run/boot')
        sample_id = _hash(key_json.encode('utf-8'))
        previous = self.get(key)
        attempt = self.db.execute('SELECT state FROM admission_attempts WHERE sample_id=?', (sample_id,)).fetchone()
        if attempt is not None and attempt[0] != 'committed':
            raise StorageUnavailable('identity already has an unfinished or terminal failed admission')
        try:
            info = validate_jpeg(image_bytes)
        except InvalidImage as exc:
            raise IntegrityError(str(exc)) from exc
        if previous is not None and previous['image_sha256'] != info['sha256']:
            raise IdentityConflict('same frame identity has different image bytes')
        if isinstance(reasons, str):
            raise ValueError('reasons must be a sequence of strings')
        reason_list = list(reasons)
        if any(not isinstance(reason, str) or not reason or len(reason) > 256 for reason in reason_list):
            raise ValueError('invalid reason')
        reason_set = sorted(set(reason_list) | set(previous['reasons'] if previous else []))
        if len(reason_set) > 128:
            raise QuotaExceeded('reason reference limit exceeded')
        merged, new_detections, events = self._merge_metadata(
            sample_id, key, previous['metadata'] if previous else {}, metadata if metadata is not None else {})
        if previous is not None and merged == previous['metadata'] and reason_set == previous['reasons']:
            return previous
        bucket = previous['quota_bucket'] if previous else self._quota_bucket(kind)
        revision = previous['revision'] + 1 if previous else 1
        record = dict(key=json.loads(key_json), sample_id=sample_id,
                      image_path='images/' + info['sha256'] + '.jpg', image_sha256=info['sha256'],
                      image_bytes=info['bytes'], width=info['width'], height=info['height'],
                      metadata=merged, reasons=reason_set, revision=revision, quota_bucket=bucket)
        record_json = _json(record)
        sidecar_data = record_json.encode('utf-8')
        # Charge serialized metadata copies conservatively, including journal/index overhead.
        metadata_cost = len(sidecar_data) * 3 + sum(len(_json(r).encode('utf-8')) for r in new_detections.values())
        admission_cost = len(key_json.encode('utf-8')) + 256 if previous is None else 0
        if self._metadata_used() + metadata_cost + admission_cost > self.max_metadata_bytes:
            raise QuotaExceeded('run metadata budget exceeded')
        reservation = len(image_bytes) + metadata_cost + 32768
        self._check_storage(reservation)
        if self._retention_pool is not None:
            self._retention_pool.reserve(self.run_info['run_id'], sample_id)
            self._stage('retention.after_reserve', fault)
        admitted = False
        try:
            if previous is None:
                self.db.execute('INSERT INTO admission_attempts VALUES (?,?,?,?,?,?,?)',
                                (sample_id, key_json, bucket, 'accepted', None, reservation, admission_cost + metadata_cost))
                self.db.execute("UPDATE counters SET value=value+? WHERE name='metadata_bytes'", (admission_cost + metadata_cost,))
                self._stage('admission.before_commit', fault)
                self._check_storage()
                self.db.commit()
                admitted = True
                self._stage('admission.after_commit', fault)
            else:
                # Retain this conservative charge even if a later write fails;
                # failed immutable revisions/orphans must not evade the budget.
                self.db.execute("UPDATE counters SET value=value+? WHERE name='metadata_bytes'", (metadata_cost,))
                self._check_storage()
                self.db.commit()
            self._publish(record['image_path'], image_bytes, 'image', fault)
            sidecar_hash = _hash(sidecar_data)
            sidecar_path = 'revisions/{}.{:06d}.{}.json'.format(sample_id, revision, sidecar_hash)
            self._publish(sidecar_path, sidecar_data, 'revision', fault)
            self._check_storage()
            self.db.execute('BEGIN IMMEDIATE')
            if previous is None:
                self.db.execute('INSERT INTO frames VALUES (?,?,?,?,?,?,?,?,?)',
                                (sample_id, key_json, record['image_path'], info['sha256'], info['bytes'],
                                 info['width'], info['height'], bucket, revision))
            else:
                self.db.execute('UPDATE frames SET latest_revision=? WHERE sample_id=?', (revision, sample_id))
            self.db.execute('INSERT INTO sample_revisions VALUES (?,?,?,?,?)',
                            (sample_id, revision, sidecar_path, sidecar_hash, record_json))
            for identity, detection in new_detections.items():
                self.db.execute('INSERT INTO detections VALUES (?,?,?)', (identity, sample_id, _json(detection)))
            for identity, event in events.items():
                self.db.execute('INSERT INTO events VALUES (?,?) ON CONFLICT(event_id) DO UPDATE SET record_json=excluded.record_json',
                                (identity, _json(event)))
            self.db.execute("UPDATE admission_attempts SET state='committed',reserved_bytes=0 WHERE sample_id=?", (sample_id,))
            self._stage('transaction.before_commit', fault)
            self._check_storage()
            self.db.commit()
            self._stage('transaction.after_commit', fault)
            self._stage('acknowledge', fault)
            if self._retention_pool is not None:
                self._retention_pool._committed(self.run_info['run_id'], sample_id)
            return dict(record, sidecar_path=sidecar_path, sidecar_sha256=sidecar_hash)
        except BaseException as exc:
            self.db.rollback()
            if isinstance(exc, Exception) and admitted:
                # Never change a committed admission if the acknowledgement failed.
                try:
                    self._check_storage()
                    self.db.execute("UPDATE admission_attempts SET state='terminal_failed',failure=?,reserved_bytes=0 WHERE sample_id=? AND state='accepted'",
                                    (type(exc).__name__, sample_id))
                    self.db.commit()
                except (OSError, sqlite3.Error, StorageUnavailable):
                    self.db.rollback()  # Recovery will settle this durable accepted entry.
            raise

    def _record(self, row):
        sample_id, key_json, image_path, image_hash, size, width, height, bucket, revision = row
        reference = self.db.execute('SELECT sidecar_path,sidecar_sha256,record_json FROM sample_revisions WHERE sample_id=? AND revision=?',
                                    (sample_id, revision)).fetchone()
        if reference is None:
            raise IntegrityError('missing committed revision')
        sidecar_path, sidecar_hash, record_json = reference
        try:
            record = json.loads(record_json)
            if (_key(record['key']) != key_json or record['sample_id'] != sample_id
                    or _hash(key_json.encode('utf-8')) != sample_id
                    or (record['image_path'], record['image_sha256'], record['image_bytes'], record['width'],
                        record['height'], record['quota_bucket'], record['revision'])
                    != (image_path, image_hash, size, width, height, bucket, revision)):
                raise IntegrityError('frame and revision identity/reference mismatch')
            if image_path != 'images/' + image_hash + '.jpg':
                raise IntegrityError('invalid image reference')
            expected_sidecar = 'revisions/{}.{:06d}.{}.json'.format(sample_id, revision, sidecar_hash)
            if sidecar_path != expected_sidecar:
                raise IntegrityError('revision path belongs to another sample')
            self._path(image_path)
            payload = self._read(sidecar_path, self.max_metadata_bytes)
            if payload != record_json.encode('utf-8') or _hash(payload) != sidecar_hash:
                raise IntegrityError('metadata revision content/hash mismatch')
            return dict(record, sidecar_path=sidecar_path, sidecar_sha256=sidecar_hash)
        except (KeyError, TypeError, ValueError, OSError) as exc:
            raise IntegrityError('invalid committed record') from exc

    def get(self, key):
        row = self.db.execute('SELECT * FROM frames WHERE key_json=?', (_key(key),)).fetchone()
        return self._record(row) if row is not None else None

    def list_samples(self):
        return [self._record(row) for row in self.db.execute('SELECT * FROM frames ORDER BY key_json').fetchall()]

    def statistics(self):
        counts = dict(self.db.execute('SELECT state,COUNT(*) FROM admission_attempts GROUP BY state').fetchall())
        occupancy = dict(self.db.execute("SELECT quota_bucket,COUNT(*) FROM admission_attempts WHERE state IN ('accepted','committed') GROUP BY quota_bucket").fetchall())
        return {'accepted': sum(counts.values()), 'committed': counts.get('committed', 0),
                'terminal_failed': counts.get('terminal_failed', 0), 'inflight': counts.get('accepted', 0),
                'recovery_unknown': self.db.execute("SELECT COUNT(*) FROM admission_attempts WHERE failure='recovery_unknown'").fetchone()[0],
                'quota_reserved_used': occupancy.get('reserved', 0), 'quota_shared_used': occupancy.get('shared', 0),
                'metadata_bytes': self._metadata_used()}

    def verify_all(self):
        report = {'ok': True, 'errors': [], 'samples': 0, 'revisions': 0, 'orphans': [], 'temporary': []}
        referenced = {'index.sqlite3', 'writer.lock', 'index.sqlite3-journal'}
        try:
            self._check_storage(check_space=False)
            if self.db.execute('PRAGMA integrity_check').fetchall() != [('ok',)]:
                raise IntegrityError('SQLite integrity check failed')
            if self.db.execute('PRAGMA foreign_key_check').fetchall():
                raise IntegrityError('SQLite foreign key check failed')
            frames = self.db.execute('SELECT * FROM frames').fetchall()
            for row in frames:
                record = self._record(row)
                data = self._read(record['image_path'], MAX_BYTES)
                actual = validate_jpeg(data, record['width'], record['height'])
                if actual['sha256'] != record['image_sha256'] or actual['bytes'] != record['image_bytes']:
                    raise IntegrityError('image content/hash mismatch')
                referenced.add(record['image_path'])
                report['samples'] += 1
                historical = self.db.execute('SELECT revision FROM sample_revisions WHERE sample_id=? ORDER BY revision', (row[0],)).fetchall()
                if [x[0] for x in historical] != list(range(1, row[8] + 1)):
                    raise IntegrityError('revision history contains gaps or unreferenced future revisions')
                for (number,) in historical:
                    old = self._record(row[:8] + (number,))
                    referenced.add(old['sidecar_path'])
                    report['revisions'] += 1
                attempt = self.db.execute('SELECT key_json,quota_bucket,state FROM admission_attempts WHERE sample_id=?', (row[0],)).fetchone()
                if attempt != (row[1], row[7], 'committed'):
                    raise IntegrityError('frame has no matching committed admission')
                for item in record['metadata'].get('detections', []):
                    detection_row = self.db.execute('SELECT sample_id,record_json FROM detections WHERE detection_key=?', (self._detection_key(item),)).fetchone()
                    if detection_row != (row[0], _json(item)):
                        raise IntegrityError('detection reference mismatch')
            stats = self.statistics()
            if stats['committed'] != len(frames):
                raise IntegrityError('committed admission count differs from frame count')
            if stats['quota_reserved_used'] > self.uniform_reserved or stats['quota_shared_used'] > self.total - self.uniform_reserved:
                raise IntegrityError('quota ledger exceeds limits')
            for directory, dirs, files in os.walk(self.root, followlinks=False):
                for name in dirs + files:
                    path = Path(directory) / name
                    _reject_links(path)
                for name in files:
                    relative = (Path(directory) / name).relative_to(self.root).as_posix()
                    if relative not in referenced:
                        report['temporary' if '.tmp.' in name else 'orphans'].append(relative)
        except (IntegrityError, InvalidImage, OSError, sqlite3.Error, ValueError, KeyError, TypeError, StorageUnavailable) as exc:
            report['ok'] = False
            report['errors'].append(str(exc))
            self._blocked = True
        report['orphans'].sort()
        report['temporary'].sort()
        return report

    def recover(self):
        self._writable()
        report = self.verify_all()
        if not report['ok']:
            raise IntegrityError('; '.join(report['errors']))
        self._check_storage()
        with self.db:
            cursor = self.db.execute("UPDATE admission_attempts SET state='terminal_failed',failure='recovery_unknown',reserved_bytes=0 WHERE state='accepted'")
            report['recovered_unknown'] = cursor.rowcount
        report['statistics'] = self.statistics()
        return report

    def record_event(self, record):
        self._writable()
        identity = self._event_id(record)
        previous = self.db.execute('SELECT record_json FROM events WHERE event_id=?', (identity,)).fetchone()
        record = self._merge_event(json.loads(previous[0]) if previous else None, record)
        serialized = _json(record)
        if previous is not None and previous[0] == serialized:
            return
        if previous is None and self.db.execute('SELECT COUNT(*) FROM events').fetchone()[0] >= 10000:
            raise QuotaExceeded('run event limit exceeded')
        cost = len(serialized.encode('utf-8')) + 256
        if self._metadata_used() + cost > self.max_metadata_bytes:
            raise QuotaExceeded('run metadata budget exceeded')
        self._check_storage(cost + 32768)
        with self.db:
            self.db.execute('INSERT INTO events VALUES (?,?) ON CONFLICT(event_id) DO UPDATE SET record_json=excluded.record_json',
                            (identity, serialized))
            self.db.execute("UPDATE counters SET value=value+? WHERE name='metadata_bytes'", (cost,))

    def close(self):
        if self.db is not None:
            self.db.close()
            self.db = None
        if self._lock_file is not None:
            handle, self._lock_file = self._lock_file, None
            try:
                if os.name == 'nt':
                    import msvcrt
                    handle.seek(0)
                    msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
            finally:
                handle.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
