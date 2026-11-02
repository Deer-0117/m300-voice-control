"""Linux collector service and bounded, asynchronous single-writer adapter."""
from collections import Counter, deque
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import stat
import sys
import threading
import time
import uuid

from .collector import Collector, Limits


def json_bytes(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(',', ':'), allow_nan=False).encode('utf-8')


def strict_json(raw):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError('duplicate JSON key')
            result[key] = value
        return result
    def constant(value):
        raise ValueError('nonfinite JSON')
    return json.loads(raw, object_pairs_hook=pairs, parse_constant=constant)


class BoundedWriter:
    """All owned queue bytes include the active task until its completion.

    The factory creates the SQLite connection on the writer thread. Producer
    calls never wait for a lock, disk, or writer, and return False on pressure.
    Counters distinguish queue acceptance from durable storage admission.
    """
    def __init__(self, factory, max_jobs=32, max_bytes=32*1024*1024,
                 max_metadata=8*1024*1024):
        self.factory = factory
        self.max_jobs, self.max_bytes, self.max_metadata = max_jobs, max_bytes, max_metadata
        self.jobs = deque()
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.ready = threading.Event()
        self.startup_error = None
        self.stopping = False
        self.owned_jobs = self.owned_bytes = self.owned_metadata = 0
        self.counters = Counter({name: 0 for name in
                                 ('queue_rejected', 'queue_accepted', 'write_completed',
                                  'write_failed', 'stats_failed')})
        self.storage_stats = {}
        self.last_error = None
        self.thread = threading.Thread(target=self._run, name='hardcase-writer', daemon=True)
        self.thread.start()

    def _enqueue(self, action, key, image, reasons, metadata, kind):
        try:
            raw = json_bytes(metadata)
            if len(raw) > 1024*1024:
                return False
            metadata = json.loads(raw)  # freeze the mutable caller's metadata
            image_size = len(image) if image is not None else 0
            reason_list = sorted(set(reasons))
        except (TypeError, ValueError):
            return False
        if not self.lock.acquire(False):
            return False
        try:
            if (self.stopping or self.startup_error or self.owned_jobs >= self.max_jobs or
                    self.owned_bytes+image_size > self.max_bytes or
                    self.owned_metadata+len(raw) > self.max_metadata):
                self.counters['queue_rejected'] += 1
                return False
            self.jobs.append((action, key, image, reason_list, metadata, kind, image_size, len(raw)))
            self.owned_jobs += 1
            self.owned_bytes += image_size
            self.owned_metadata += len(raw)
            self.counters['queue_accepted'] += 1
            self.wake.set()
            return True
        finally:
            self.lock.release()

    def submit(self, key, image, reasons, metadata, kind):
        return self._enqueue('put', key, image, reasons, metadata, kind)

    def annotate(self, key, reasons, metadata):
        return self._enqueue('annotate', key, None, reasons, metadata, 'event')

    def event(self, event):
        return self._enqueue('event', None, None, [], event, 'event')

    def _run(self):
        store = None
        try:
            store = self.factory()
            self.storage_stats = store.statistics()
        except Exception as exc:
            self.startup_error = str(exc)[:512]
        finally:
            self.ready.set()
        if store is None or self.startup_error:
            if store is not None:
                store.close()
            return
        try:
            while True:
                with self.lock:
                    job = self.jobs.popleft() if self.jobs else None
                    stop = self.stopping and job is None
                    if job is None:
                        self.wake.clear()
                if stop:
                    break
                if job is None:
                    self.wake.wait(.1)
                    continue
                action, key, image, reasons, metadata, kind, nbytes, nmeta = job
                try:
                    if action == 'event':
                        store.record_event(metadata)
                    else:
                        if action == 'annotate':
                            old = store.get(key)
                            if old is None:
                                raise ValueError('annotation target was never committed')
                            # get validates the contained path; put verifies the
                            # bytes against the immutable image identity.
                            image = (store.root / old['image_path']).read_bytes()
                        store.put(key, image, reasons, metadata, kind=kind)
                    self.counters['write_completed'] += 1
                except Exception as exc:
                    self.counters['write_failed'] += 1
                    self.last_error = (type(exc).__name__+': '+str(exc))[:512]
                finally:
                    with self.lock:
                        self.owned_jobs -= 1
                        self.owned_bytes -= nbytes
                        self.owned_metadata -= nmeta
                try:
                    self.storage_stats = store.statistics()
                except Exception:
                    self.counters['stats_failed'] += 1
        finally:
            store.close()

    def snapshot(self):
        with self.lock:
            return dict(self.counters, queued_jobs=self.owned_jobs, queued_bytes=self.owned_bytes,
                        queued_metadata_bytes=self.owned_metadata, storage=dict(self.storage_stats),
                        last_error=self.last_error)

    def close(self, timeout=5):
        with self.lock:
            self.stopping = True
            self.wake.set()
        self.thread.join(timeout)
        return not self.thread.is_alive()


def atomic_json(path, value):
    path = Path(path)
    if path.is_symlink() or path.parent.is_symlink():
        raise ValueError('symlink not allowed for control/status')
    tmp = path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(json_bytes(value))
        os.replace(tmp, path)
    finally:
        if tmp.exists():
            tmp.unlink()


class ControlPublisher:
    def __init__(self, ipc_dir, run_id, boot_id, config_sha256):
        self.path = Path(ipc_dir)/'control.json'
        self.run_id, self.boot_id, self.config_sha256 = run_id, boot_id, config_sha256
        self.generation = 0
        self.enabled = None
        if self.path.exists():
            if self.path.is_symlink() or self.path.stat().st_size > 4096:
                raise ValueError('unsafe prior control file')
            prior = strict_json(self.path.read_bytes())
            generation = prior.get('generation')
            if type(generation) is not int or generation < 0:
                raise ValueError('invalid prior control generation')
            self.generation = generation+1

    def publish(self, enabled, now_ns):
        if self.enabled is not None and self.enabled != enabled:
            self.generation += 1
        self.enabled = enabled
        control = dict(protocol_version=1, run_id=self.run_id, boot_id=self.boot_id,
                       config_sha256=self.config_sha256, generation=self.generation,
                       enabled=enabled, published_mono_ns=now_ns)
        atomic_json(self.path, control)
        return control


def load_config(path):
    raw = Path(path).read_bytes()
    if len(raw) > 65536:
        raise ValueError('configuration too large')
    supplied = strict_json(raw)
    defaults = dict(schema_version=1, enabled=True, data_root=None, mount_point=None,
                    expected_filesystem_uuid=None, ipc_dir='/dev/shm/balloon-hardcase',
                    allowed_uid=None, allowed_gid=None, min_free_bytes=10*1024**3,
                    min_free_ratio=.1, total_images=1000, uniform_reserved=200,
                    queue_jobs=32, queue_bytes=32*1024**2,
                    limits=asdict(Limits()), release={})
    if not isinstance(supplied, dict) or set(supplied)-set(defaults):
        raise ValueError('unknown configuration fields')
    limits = dict(defaults['limits'])
    if not isinstance(supplied.get('limits', {}), dict) or set(supplied.get('limits', {}))-set(limits):
        raise ValueError('unknown limit fields')
    limits.update(supplied.get('limits', {}))
    defaults.update(supplied)
    defaults['limits'] = asdict(Limits(**limits))
    if defaults['schema_version'] != 1 or type(defaults['enabled']) is not bool:
        raise ValueError('unsupported config version/enabled')
    for name in ('queue_jobs', 'queue_bytes', 'total_images'):
        if type(defaults[name]) is not int or defaults[name] <= 0:
            raise ValueError('invalid '+name)
    for name in ('min_free_bytes', 'uniform_reserved'):
        if type(defaults[name]) is not int or defaults[name] < 0:
            raise ValueError('invalid '+name)
    if not 0 <= defaults['uniform_reserved'] <= defaults['total_images']:
        raise ValueError('invalid reserved quota')
    ratio = defaults['min_free_ratio']
    if type(ratio) not in (int, float) or not math.isfinite(ratio) or not 0 <= ratio <= 1:
        raise ValueError('invalid free space ratio')
    if not isinstance(defaults['release'], dict):
        raise ValueError('release provenance must be an object')
    if defaults['enabled']:
        for name in ('data_root', 'mount_point', 'expected_filesystem_uuid', 'ipc_dir'):
            if not isinstance(defaults[name], str) or not defaults[name]:
                raise ValueError('enabled capture requires '+name)
        for name in ('allowed_uid', 'allowed_gid'):
            if type(defaults[name]) is not int or defaults[name] < 0:
                raise ValueError('enabled capture requires '+name)
        if '/' in defaults['expected_filesystem_uuid'] or '\\' in defaults['expected_filesystem_uuid']:
            raise ValueError('filesystem UUID must be a device identifier')
    return defaults


def check_mount(cfg):
    mount = Path(cfg['mount_point'])
    root = Path(cfg['data_root'])
    if not mount.is_absolute() or not root.is_absolute() or root == mount:
        raise ValueError('data_root must be a child of the dedicated mount')
    for path in (mount, root):
        for part in (path, *path.parents):
            if part.is_symlink():
                raise ValueError('storage path contains a symlink')
    if not root.is_relative_to(mount) or not os.path.ismount(mount):
        raise ValueError('expected dedicated mount is absent')
    device = os.stat(mount).st_dev
    uuid_device = os.stat(Path('/dev/disk/by-uuid')/cfg['expected_filesystem_uuid']).st_rdev
    if device != uuid_device or device == os.stat('/').st_dev:
        raise ValueError('mount UUID mismatch or system-disk fallback')
    return device


def create_capture_store(cfg, run_info, device):
    """Open the writer's run under the persistent data-root retention budget."""
    from .retention import create_managed_run
    return create_managed_run(cfg['data_root'], run_info,
                              total=cfg['total_images'], uniform_reserved=cfg['uniform_reserved'],
                              min_free_bytes=cfg['min_free_bytes'], min_free_ratio=cfg['min_free_ratio'],
                              expected_mount=cfg['mount_point'], expected_device=device)


def wait_for_writer_ready(writer, stopped):
    """Full-pool verification may take time; remain interruptible during startup."""
    while not stopped.is_set():
        if writer.ready.wait(.1):
            if writer.startup_error:
                raise RuntimeError('store startup failed: ' + str(writer.startup_error))
            return True
    return False


def run_service(cfg):
    if not cfg['enabled']:
        return 0
    if not sys.platform.startswith('linux'):
        raise RuntimeError('live FD capture requires Linux; use the offline replay on this host')
    import fcntl
    from .protocol import boot_id
    from .transport import receive_message

    device = check_mount(cfg)
    ipc = Path(cfg['ipc_dir'])
    if not ipc.is_absolute() or not ipc.is_relative_to(Path('/dev/shm')) or ipc == Path('/dev/shm'):
        raise ValueError('IPC must be a private directory under /dev/shm')
    if any(p.is_symlink() for p in (ipc, *ipc.parents)):
        raise ValueError('IPC path cannot contain symlinks')
    ipc.mkdir(mode=0o700, parents=False, exist_ok=True)
    info = ipc.stat()
    if info.st_uid != os.geteuid() or stat.S_IMODE(info.st_mode) != 0o700:
        raise ValueError('IPC directory must be owned by service UID with mode0700')
    lock_path = ipc/'collector.lock'
    lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    sockets = []
    paths = []
    writer = publisher = collector = None
    stopped = threading.Event()
    previous_signals = {}
    try:
        fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for signum in (signal.SIGTERM, signal.SIGINT):
            previous_signals[signum] = signal.signal(signum, lambda *_: stopped.set())
        run, boot = uuid.uuid4().hex, boot_id()
        digest = hashlib.sha256(json_bytes(cfg)).hexdigest()
        run_info = dict(run_id=run, boot_id=boot, config_sha256=digest, config=cfg,
                        started_utc_ms=time.time_ns()//1000000, release=cfg['release'],
                        validation='ground-and-performance-validation-required')
        writer = BoundedWriter(lambda: create_capture_store(cfg, run_info, device),
                    max_jobs=cfg['queue_jobs'], max_bytes=cfg['queue_bytes'],
                    max_metadata=cfg['limits']['metadata_bytes']//2)
        if not wait_for_writer_ready(writer, stopped):
            return 0
        publisher = ControlPublisher(ipc, run, boot, digest)
        runtime_limits = dict(cfg['limits'])
        runtime_limits['metadata_bytes'] //= 2
        collector = Collector(run, boot, publisher.generation, writer, Limits(**runtime_limits))
        for filename in ('frames.sock', 'results.sock'):
            path = ipc/filename
            if path.exists():
                if path.is_symlink() or not stat.S_ISSOCK(path.lstat().st_mode):
                    raise ValueError('unsafe existing IPC path')
                path.unlink()  # lock proves no other service owns this endpoint
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            sockets.append(sock)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
            sock.setblocking(False)
            sock.bind(str(path))
            path.chmod(0o600)
            paths.append(path)
        heartbeat = status_time = 0
        while not stopped.is_set():
            now = time.monotonic_ns()
            if now >= heartbeat:
                try:
                    check_mount(cfg)
                except (OSError, ValueError):
                    if collector.enabled:
                        publisher.publish(False, now)
                        collector.reconfigure(publisher.generation, False, now)
                    collector.counts['mount_unavailable'] += 1
                publisher.publish(collector.enabled, now)
                heartbeat = now+250000000
            ready, _, _ = select.select(sockets, [], [], .05)
            for sock in ready:
                for _ in range(32):
                    try:
                        item = receive_message(sock, cfg['allowed_uid'], cfg['allowed_gid'])
                        if item is None:
                            break
                        collector.process(*item, time.monotonic_ns())
                    except (OSError, ValueError, TypeError, KeyError):
                        collector.counts['ipc_rejected'] += 1
            collector.tick(time.monotonic_ns())
            if now >= status_time:
                atomic_json(ipc/'status.json', dict(run_id=run, collector=collector.stats(),
                                                   writer=writer.snapshot()))
                status_time = now+SECOND
        return 0
    finally:
        if publisher:
            try:
                publisher.publish(False, time.monotonic_ns())
            except (OSError, ValueError):
                pass
        if collector and publisher and publisher.generation > collector.generation:
            collector.reconfigure(publisher.generation, False, time.monotonic_ns())
        if writer:
            drained = writer.close(5)
            try:
                atomic_json(ipc/'status.json', dict(drained=drained, writer=writer.snapshot(),
                    collector=collector.stats() if collector else {},
                    queued_after_timeout='unknown-until-recovery' if not drained else 0))
            except (OSError, ValueError):
                pass
        for sock in sockets:
            sock.close()
        for path in paths:
            path.unlink(missing_ok=True)
        for signum, handler in previous_signals.items():
            signal.signal(signum, handler)
        os.close(lock_fd)


SECOND = 1_000_000_000
