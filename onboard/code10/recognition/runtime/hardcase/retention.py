"""Durable, conservative retained-sample quota shared by every acquisition run.

Claims are never released on acquisition failures. Only a completed, explicitly
confirmed batch cleanup releases them; this module performs no data deletion.
"""

import json
import os
from pathlib import Path
import re
import sqlite3
import stat
import threading
import uuid

from .storage import (IntegrityError, QuotaExceeded, RunStore, StorageUnavailable,
                      _fsync_directory, _json, _reject_links)


_SCHEMA = """
CREATE TABLE pool (singleton INTEGER PRIMARY KEY CHECK(singleton=1),
                  pool_id TEXT NOT NULL, total INTEGER NOT NULL CHECK(total>=0));
CREATE TABLE runs (run_id TEXT PRIMARY KEY,
                   state TEXT NOT NULL CHECK(state IN ('open','sealed','cleaning','cleared')),
                   manifest TEXT, receipt TEXT);
CREATE TABLE claims (run_id TEXT NOT NULL REFERENCES runs(run_id),
                     sample_id TEXT NOT NULL,
                     kind TEXT NOT NULL CHECK(kind IN ('reserved','retained','orphan')),
                     PRIMARY KEY(run_id,sample_id));
"""


def _run_id(value):
    if (not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}', value)
            or value.endswith('.')
            or value.split('.')[0].upper() in {'CON', 'PRN', 'AUX', 'NUL',
                                             *('COM' + str(n) for n in range(1, 10)),
                                             *('LPT' + str(n) for n in range(1, 10))}):
        raise IntegrityError('unsafe run identifier')
    return value


def assert_run_write_allowed(run_root, *, pool=None):
    """Require the live owning pool for every writable managed source.

    RunStore calls this after taking its own writer lock. Maintenance holds that
    lock until sealing commits, closing the gap before a Windows cleanup rename.
    Independent ground copies have no parent retention authority and remain usable.
    """
    requested_root = Path(os.path.abspath(run_root))
    _reject_links(requested_root)
    # Windows accepts alternate casing and short-name aliases. Resolve only after
    # rejecting links so the lookup uses the actual directory spelling recorded
    # by the ledger, while Linux run identifiers remain case-sensitive. Nonexistent
    # creation paths retain their requested leaf but canonicalize existing parents.
    run_root = requested_root.resolve(strict=False)
    _reject_links(requested_root)
    _reject_links(run_root)
    area = os.path.normcase(run_root.parent.name)
    if area == os.path.normcase('.cleanup'):
        raise StorageUnavailable('cleanup staging cannot be opened for writing')
    if area != os.path.normcase('runs'):
        return
    root = run_root.parent.parent
    ledger, marker = root / 'retention.sqlite3', root / 'retention.json'
    _reject_links(ledger)
    _reject_links(marker)
    _reject_links(root / 'retention.sqlite3-journal')
    if not ledger.exists() and not marker.exists():
        return
    if not ledger.is_file() or not marker.is_file():
        raise IntegrityError('incomplete parent retention authority')
    if not isinstance(pool, RetentionPool):
        raise StorageUnavailable('managed source writes require its live retention pool')
    pool._check()
    if os.path.normcase(str(pool.root.resolve(strict=True))) != os.path.normcase(str(root)):
        raise StorageUnavailable('managed source writes require its live retention pool')
    record = pool.run_record(_run_id(run_root.name))
    # A fresh run is registered after its local authority is successfully created.
    if record is not None and record['state'] != 'open':
        raise StorageUnavailable('sealed or cleaning source cannot be opened for writing')


class RetentionPool:
    """One lifetime OS lock covers acquisition, maintenance and ledger mutation.

    ``create=False`` requires an existing pool, and never inventories or creates
    data. It still permits explicit durable maintenance transitions.
    """

    def __init__(self, data_root, total=None, create=True):
        if total is not None and (type(total) is not int or total < 0):
            raise ValueError('total must be a nonnegative integer')
        self.root = Path(os.path.abspath(data_root))
        self.db = None
        self._lock_file = None
        self._thread = threading.get_ident()
        self._db_identity = None
        self._root_identity = None
        try:
            _reject_links(self.root)
            if create:
                self.root.mkdir(parents=True, exist_ok=True)
            elif not self.root.is_dir():
                raise StorageUnavailable('retention pool does not exist')
            self._root_identity = self._identity(self.root)
            db_path = self.root / 'retention.sqlite3'
            marker = self.root / 'retention.json'
            for path in (db_path, marker, self.root / 'retention.sqlite3-journal',
                         self.root / 'runs', self.root / '.cleanup'):
                _reject_links(path)
            self._lock(create)
            exists = db_path.exists()
            if not exists and (not create or marker.exists()):
                if marker.exists():
                    raise IntegrityError('retention ledger is missing; refusing to reset claims')
                raise StorageUnavailable('retention ledger does not exist')
            if exists:
                if not marker.is_file():
                    raise IntegrityError('retention identity marker is missing')
                self.db = sqlite3.connect(db_path.as_uri() + '?mode=rw', uri=True)
                self._configure()
                if self.db.execute('PRAGMA integrity_check').fetchall() != [('ok',)]:
                    raise IntegrityError('retention ledger integrity check failed')
                if self.db.execute('PRAGMA foreign_key_check').fetchall():
                    raise IntegrityError('retention ledger foreign keys are invalid')
                rows = self.db.execute('SELECT pool_id,total FROM pool').fetchall()
                if len(rows) != 1:
                    raise IntegrityError('invalid retention policy')
                self.pool_id, self.total = rows[0]
                if type(self.total) is not int or self.total < 0:
                    raise IntegrityError('invalid retention total')
                if json.loads(marker.read_text(encoding='utf-8')) != self._marker():
                    raise IntegrityError('retention identity or policy changed')
                if total is not None and total != self.total:
                    raise IntegrityError('cannot change cumulative retention quota')
            else:
                self.pool_id, self.total = str(uuid.uuid4()), 1000 if total is None else total
                # Publishing the independent identity first makes interrupted first
                # initialization fail closed, rather than silently start over.
                with marker.open('x', encoding='utf-8', newline='\n') as handle:
                    handle.write(_json(self._marker()))
                    handle.flush()
                    os.fsync(handle.fileno())
                _fsync_directory(self.root)
                self.db = sqlite3.connect(str(db_path))
                self._configure()
                self.db.executescript(_SCHEMA)
                with self.db:
                    self.db.execute('INSERT INTO pool VALUES (1,?,?)', (self.pool_id, self.total))
                _fsync_directory(self.root)
            self._db_identity = self._identity(db_path)
            if create:
                self._reconcile()
        except sqlite3.DatabaseError as exc:
            self.close()
            raise IntegrityError('invalid retention SQLite authority') from exc
        except (json.JSONDecodeError, UnicodeError) as exc:
            self.close()
            raise IntegrityError('invalid retention identity marker') from exc
        except BaseException:
            self.close()
            raise

    def _marker(self):
        return {'schema_version': 1, 'pool_id': self.pool_id, 'total': self.total}

    @staticmethod
    def _identity(path):
        info = path.stat()
        return info.st_dev, info.st_ino

    def _configure(self):
        self.db.execute('PRAGMA foreign_keys=ON')
        self.db.execute('PRAGMA journal_mode=DELETE')
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('PRAGMA busy_timeout=1000')

    def _lock(self, create):
        path = self.root / 'retention.lock'
        _reject_links(path)
        flags = os.O_RDWR | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_BINARY', 0)
        if create:
            flags |= os.O_CREAT
        try:
            fd = os.open(path, flags, 0o600)
        except FileNotFoundError as exc:
            raise StorageUnavailable('retention lock is missing') from exc
        handle = os.fdopen(fd, 'r+b', buffering=0)
        try:
            if os.name == 'nt':
                import msvcrt
                if os.fstat(fd).st_size == 0:
                    if not create:
                        raise StorageUnavailable('retention lock is invalid')
                    handle.write(b'\0')
                handle.seek(0)
                msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._lock_file = handle
            self._lock_identity = self._identity(path)
        except OSError as exc:
            handle.close()
            raise StorageUnavailable('retention pool is busy with an active writer or maintenance') from exc
        except BaseException:
            handle.close()
            raise

    def _check(self):
        if self.db is None or self._lock_file is None or threading.get_ident() != self._thread:
            raise StorageUnavailable('retention pool is closed or used by another thread')
        try:
            for name in ('retention.sqlite3', 'retention.sqlite3-journal', 'retention.lock',
                         'retention.json', 'runs', '.cleanup'):
                _reject_links(self.root / name)
            if (self._identity(self.root) != self._root_identity
                    or self._identity(self.root / 'retention.sqlite3') != self._db_identity
                    or self._identity(self.root / 'retention.lock') != self._lock_identity):
                raise StorageUnavailable('retention authority or storage root was replaced')
        except OSError as exc:
            raise StorageUnavailable('retention storage is unavailable') from exc

    def _reconcile(self):
        self._check()
        if self.db.execute("SELECT 1 FROM runs WHERE state='cleaning'").fetchone():
            raise StorageUnavailable('unfinished batch cleanup requires explicit maintenance')
        staging = self.root / '.cleanup'
        if staging.exists() and (not staging.is_dir() or any(staging.iterdir())):
            raise IntegrityError('untracked cleanup data requires explicit maintenance')
        runs = self.root / 'runs'
        if not runs.exists():
            runs.mkdir(mode=0o700)
            _fsync_directory(self.root)
        seen = set()
        for path in sorted(runs.iterdir()):
            _reject_links(path)
            run_id = _run_id(path.name)
            if not path.is_dir():
                raise IntegrityError('unexpected file in runs directory')
            seen.add(run_id)
            record = self.run_record(run_id)
            if record is not None and record['state'] == 'cleared':
                raise IntegrityError('cleared run has reappeared')
            # Read-only opening verifies immutable evidence without recovery writes.
            # The additional legacy writer lock excludes writers predating this pool.
            with RunStore.open(path, read_only=True) as store:
                store._lock()
                if store.run_info.get('run_id') != run_id:
                    raise IntegrityError('run directory and stored identity disagree')
                claims = [(sample_id, 'retained' if state == 'committed' else 'reserved')
                          for sample_id, state in store.db.execute(
                              'SELECT sample_id,state FROM admission_attempts').fetchall()]
                referenced = {row[0] for row in store.db.execute('SELECT image_path FROM frames')}
                for directory, dirs, files in os.walk(path, followlinks=False):
                    for name in dirs + files:
                        _reject_links(Path(directory) / name)
                    for name in files:
                        file = Path(directory) / name
                        if not stat.S_ISREG(file.stat().st_mode):
                            raise IntegrityError('nonregular image evidence')
                        relative = file.relative_to(path).as_posix()
                        if (relative not in referenced and
                                (relative.startswith('images/') or file.suffix.lower() in ('.jpg', '.jpeg'))):
                            claims.append(('orphan:' + relative, 'orphan'))
                with self.db:
                    self.db.execute("INSERT OR IGNORE INTO runs VALUES (?,'open',NULL,NULL)", (run_id,))
                    for sample_id, kind in claims:
                        self.db.execute('INSERT INTO claims VALUES (?,?,?) ON CONFLICT(run_id,sample_id) '
                                        "DO UPDATE SET kind=CASE WHEN excluded.kind='retained' THEN 'retained' ELSE claims.kind END",
                                        (run_id, sample_id, kind))
        for run_id, state in self.db.execute('SELECT run_id,state FROM runs').fetchall():
            _run_id(run_id)
            if state != 'cleared' and run_id not in seen:
                raise IntegrityError('retained run is missing; claims cannot be released implicitly')

    def register_run(self, run_id):
        self._check()
        run_id = _run_id(run_id)
        path = self.root / 'runs' / run_id
        _reject_links(path)
        if not path.is_dir():
            raise StorageUnavailable('run directory does not exist')
        record = self.run_record(run_id)
        if record is not None:
            self.ensure_writable(run_id)
            return record
        with self.db:
            self.db.execute("INSERT INTO runs VALUES (?,'open',NULL,NULL)", (run_id,))
        return self.run_record(run_id)

    def run_record(self, run_id):
        self._check()
        row = self.db.execute('SELECT state,manifest,receipt FROM runs WHERE run_id=?',
                              (_run_id(run_id),)).fetchone()
        if row is None:
            return None
        occupied = self.db.execute('SELECT COUNT(*) FROM claims WHERE run_id=?', (run_id,)).fetchone()[0]
        return {'run_id': run_id, 'state': row[0],
                'manifest': json.loads(row[1]) if row[1] is not None else None,
                'receipt': json.loads(row[2]) if row[2] is not None else None,
                'occupied': occupied}

    def ensure_writable(self, run_id):
        record = self.run_record(run_id)
        if record is None or record['state'] != 'open':
            raise StorageUnavailable('run is not open for acquisition')
        _reject_links(self.root / 'runs' / run_id)

    def reserve(self, run_id, sample_id):
        self.ensure_writable(run_id)
        if not isinstance(sample_id, str) or not re.fullmatch('[0-9a-f]{64}', sample_id):
            raise IntegrityError('invalid retained sample identity')
        with self.db:
            self.db.execute('BEGIN IMMEDIATE')
            if self.db.execute('SELECT 1 FROM claims WHERE run_id=? AND sample_id=?',
                               (run_id, sample_id)).fetchone():
                return False
            if self.db.execute('SELECT COUNT(*) FROM claims').fetchone()[0] >= self.total:
                raise QuotaExceeded('cumulative retained-image quota exhausted')
            self.db.execute("INSERT INTO claims VALUES (?,?,'reserved')", (run_id, sample_id))
        return True

    def _committed(self, run_id, sample_id):
        self.ensure_writable(run_id)
        with self.db:
            self.db.execute("UPDATE claims SET kind='retained' WHERE run_id=? AND sample_id=?",
                            (run_id, sample_id))

    def statistics(self):
        self._check()
        counts = dict(self.db.execute('SELECT kind,COUNT(*) FROM claims GROUP BY kind').fetchall())
        used, reserved = sum(counts.values()), counts.get('reserved', 0)
        available = max(0, self.total - used)
        runs = [{'run_id': run_id, 'state': state, 'occupied': occupied}
                for run_id, state, occupied in self.db.execute(
                    'SELECT runs.run_id,runs.state,COUNT(claims.sample_id) FROM runs '
                    'LEFT JOIN claims ON claims.run_id=runs.run_id '
                    'GROUP BY runs.run_id ORDER BY runs.run_id').fetchall()]
        return {'pool_id': self.pool_id, 'total': self.total, 'used': used,
                'reserved': reserved, 'available': available,
                'total_images': self.total, 'used_images': used,
                'reserved_images': reserved, 'available_images': available,
                'orphan_images': counts.get('orphan', 0), 'runs': runs}

    def set_transfer_manifest(self, run_id, manifest):
        record = self.run_record(run_id)
        if not isinstance(manifest, dict) or manifest.get('pool_id') != self.pool_id or manifest.get('run_id') != run_id:
            raise IntegrityError('transfer manifest belongs to another pool or run')
        encoded = _json(manifest)
        if record is None:
            raise StorageUnavailable('unknown run')
        if record['state'] == 'sealed' and record['manifest'] == manifest:
            return
        if record['state'] != 'open':
            raise IntegrityError('run already sealed with another manifest or being cleaned')
        with self.db:
            self.db.execute("UPDATE runs SET state='sealed',manifest=? WHERE run_id=?", (encoded, run_id))

    def begin_cleanup(self, run_id, receipt):
        record = self.run_record(run_id)
        if not isinstance(receipt, dict) or receipt.get('pool_id') != self.pool_id or receipt.get('run_id') != run_id:
            raise IntegrityError('cleanup receipt belongs to another pool or run')
        encoded = _json(receipt)
        if record is None:
            raise StorageUnavailable('unknown run')
        if record['state'] in ('cleaning', 'cleared') and record['receipt'] == receipt:
            return
        if record['state'] != 'sealed':
            raise IntegrityError('cleanup requires a sealed run and the original receipt')
        with self.db:
            self.db.execute("UPDATE runs SET state='cleaning',receipt=? WHERE run_id=?", (encoded, run_id))

    def finish_cleanup(self, run_id):
        record = self.run_record(run_id)
        if record is None or record['state'] not in ('cleaning', 'cleared'):
            raise StorageUnavailable('cleanup has not been confirmed')
        for path in (self.root / 'runs' / run_id, self.root / '.cleanup' / run_id):
            _reject_links(path)
            if path.exists():
                raise StorageUnavailable('cleanup data still exists; cannot release slots')
        if record['state'] == 'cleared':
            return
        with self.db:
            self.db.execute('DELETE FROM claims WHERE run_id=?', (run_id,))
            self.db.execute("UPDATE runs SET state='cleared' WHERE run_id=?", (run_id,))

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


class ManagedRunStore:
    """Existing RunStore behavior with durable root reservations before writing."""

    def __init__(self, pool, store):
        self.pool, self.store = pool, store
        self.run_id = _run_id(store.run_info['run_id'])

    def __getattr__(self, name):
        return getattr(self.store, name)

    def put(self, key, image_bytes, reasons, metadata=None, kind='event', *, fault=None):
        self.pool.ensure_writable(self.run_id)
        return self.store.put(key, image_bytes, reasons, metadata, kind, fault=fault)

    def record_event(self, record):
        self.pool.ensure_writable(self.run_id)
        return self.store.record_event(record)

    def statistics(self):
        return dict(self.store.statistics(), retention=self.pool.statistics())

    def close(self):
        try:
            self.store.close()
        finally:
            self.pool.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()


def create_managed_run(data_root, run_info, *, total=1000, **store_options):
    if not isinstance(run_info, dict):
        raise ValueError('run_info must be an object')
    run_id = _run_id(run_info.get('run_id'))
    pool = RetentionPool(data_root, total=total)
    store = None
    try:
        if pool.statistics()['available'] == 0:
            raise QuotaExceeded('cumulative retained-image quota exhausted')
        if pool.run_record(run_id) is not None:
            raise IntegrityError('run identifier was already used')
        store_options.setdefault('uniform_reserved', min(200, total))
        store = RunStore.create(pool.root / 'runs' / run_id, run_info,
                                total=total, _retention_pool=pool, **store_options)
        pool.register_run(run_id)
        return ManagedRunStore(pool, store)
    except BaseException:
        if store is not None:
            store.close()
        pool.close()
        raise
