#!/usr/bin/env python3
"""Serialized detector selection for the owning PSDK widget; never starts flight.

The maintenance panel reserves its action mutex/epoch and writes token and owner
PID before forking. Stop revokes the lease. The child restores the previous
configuration before the owner accepts another Start.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time

from balloon_clock import clock_id, now_ms
from detector_selector import read_selection

ROOT = Path(__file__).resolve().parent
TMP = Path('/tmp')
PROC = Path('/proc')
WORKER = 'balloon-yolo-worker.service'
TRACKER = 'balloon-tracker-worker.service'
PANEL = 'psdk-flight-panel-display.service'
LEGACY_SDK = 'psdk-balloon.service'
LEASE = 'balloon_test_options.request'
ACK = 'balloon_test_options.ack'
MESSAGE = 'balloon_test_options.message'
BUSY = ('balloon_mission.token', 'balloon_mission.commit', 'balloon_mission.json',
        'balloon_mission.ready',
        'balloon_strike.armed', 'balloon_strike.trigger',
        'balloon_timing_capture.request', 'balloon_prediction_record.request')
TARGETS = ('latest_target.tsv', 'latest_yolo_target.tsv',
           'latest_detection_request.tsv', 'detector_runtime.json')


class SelectionError(RuntimeError):
    pass


class Cancelled(SelectionError):
    pass


def atomic_text(path, text):
    path = Path(path)
    if path.is_symlink():
        raise SelectionError('Unexpected symbolic state file')
    fd, name = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(text)
        os.replace(name, path)
    finally:
        Path(name).unlink(missing_ok=True)


def settings_hash(settings):
    return hashlib.sha256(json.dumps(settings, sort_keys=True).encode()).hexdigest()


class Services:
    def state(self, unit):
        result = subprocess.run(['systemctl', 'show', unit, '-p', 'LoadState',
                                 '-p', 'ActiveState', '-p', 'MainPID'],
                                capture_output=True, text=True, timeout=5, check=True)
        values = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        if values.get('LoadState') != 'loaded':
            raise SelectionError('Required service is not installed: ' + unit)
        return values

    def action(self, action, *units):
        subprocess.run(['systemctl', action, *units], capture_output=True,
                       text=True, timeout=8, check=True)


class Selection:
    def __init__(self, token, owner_pid, *, root=ROOT, tmp=TMP, proc=PROC,
                 services=None, clock=None):
        if not re.fullmatch(r'[A-Za-z0-9:._-]{1,160}', token) or owner_pid <= 1:
            raise SelectionError('Invalid widget lease')
        self.token, self.owner_pid = token, owner_pid
        self.root, self.tmp, self.proc = Path(root), Path(tmp), Path(proc)
        self.services = services or Services()
        self.clock = clock or clock_id()
        self.owner_start = None
        self.rolling_back = False

    @property
    def config(self):
        return self.root / 'detector_config.json'

    def lease(self):
        if self.rolling_back:
            return
        path = self.tmp / LEASE
        if path.is_symlink():
            raise Cancelled('Widget selection lease is invalid')
        try:
            raw = path.read_text(encoding='ascii')
            if raw != f'{self.token}\n{self.owner_pid}\n':
                raise Cancelled('Widget selection was cancelled or replaced')
            owner = self.proc / str(self.owner_pid)
            argv = (owner / 'cmdline').read_bytes().split(b'\0')
            name = Path(argv[0].decode(errors='replace')).name
            if not re.search(r'dji_sdk|psdk|balloon', name, re.I):
                raise SelectionError('Lease owner is not the PSDK application')
            # starttime field 22, robust against spaces in process comm.
            start = (owner / 'stat').read_text().rsplit(')', 1)[1].split()[19]
            if self.owner_start is None:
                self.owner_start = start
            elif start != self.owner_start:
                raise Cancelled('Widget owner process changed')
        except (OSError, UnicodeError, IndexError) as exc:
            raise Cancelled('Widget owner or selection lease disappeared') from exc

    def idle(self):
        self.lease()
        panel = self.services.state(PANEL)
        legacy = self.services.state(LEGACY_SDK)
        if panel.get('ActiveState') != 'active' or int(panel.get('MainPID', '0')) != self.owner_pid:
            raise SelectionError('Detector selection must be started by the active maintenance panel service')
        if legacy.get('ActiveState') != 'inactive' or int(legacy.get('MainPID', '-1')) != 0:
            raise SelectionError('Stop the legacy SDK service before switching detectors')
        for name in BUSY:
            path = self.tmp / name
            if path.exists() or path.is_symlink():
                raise SelectionError('Finish the active task/recording before switching: ' + name)
        ack = self.tmp / 'balloon_prediction_record_ack.json'
        if ack.is_symlink():
            raise SelectionError('Recorder status has an unexpected symbolic path')
        if ack.exists():
            try:
                record = json.loads(ack.read_text())
                if not isinstance(record, dict):
                    raise ValueError('Recorder status is not an object')
            except (OSError, ValueError) as exc:
                raise SelectionError('Recorder status is unreadable') from exc
            if record.get('clock_id') == self.clock and record.get('status') == 'recording':
                raise SelectionError('Recorder has not stopped yet')
        # A unit state alone does not rule out another SDK process.
        for item in self.proc.iterdir():
            if not item.name.isdecimal() or int(item.name) in (os.getpid(), self.owner_pid):
                continue
            try:
                name = Path((item / 'cmdline').read_bytes().split(b'\0')[0].decode(errors='replace')).name
            except FileNotFoundError:
                continue
            except PermissionError as exc:
                raise SelectionError('Cannot verify other SDK processes') from exc
            if re.search(r'dji_sdk|psdk|balloon_telemetry', name, re.I):
                raise SelectionError('Another SDK/panel process is running')

    def ready(self, selected):
        self.lease()
        settings = read_selection(self.config)
        current = self.services.state(WORKER)
        try:
            receipt = self.root / 'detector_runtime.json'
            if receipt.is_symlink():
                raise SelectionError('Detector readiness has an unexpected symbolic path')
            runtime = json.loads(receipt.read_text())
            good = (current['ActiveState'] == 'active' and int(current['MainPID']) > 1
                    and type(runtime['pid']) is int and int(current['MainPID']) == runtime['pid']
                    and runtime['clock_id'] == self.clock
                    and settings['active_detector'] == selected == runtime['active_detector']
                    and runtime['selector_config'] == settings
                    and runtime['selector_sha256'] == settings_hash(settings)
                    and type(runtime['ready_ts_ms']) is int and 0 < runtime['ready_ts_ms'] <= now_ms())
            if good and selected == 'pixel' and runtime['neural_modules_loaded']:
                raise SelectionError('Pixel detector unexpectedly loaded neural modules')
            return runtime if good else None
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def wait_ready(self, selected, timeout=45):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.lease()
            result = self.ready(selected)
            if result is not None:
                return result
            time.sleep(.2)
        raise SelectionError('Detector did not become ready')

    def verify(self, selected):
        if self.ready(selected) is None:
            raise SelectionError('Selected detector and current worker do not match; select it again')
        tracker = self.services.state(TRACKER)
        if tracker.get('ActiveState') != 'active' or int(tracker.get('MainPID', '0')) <= 1:
            raise SelectionError('跟踪服务未就绪；请先恢复跟踪服务再开始测试')

    def acknowledge(self):
        atomic_text(self.tmp / MESSAGE, '识别方案已就绪\n')
        self.lease()
        ack = self.tmp / ACK
        try:
            atomic_text(ack, self.token + '\n')
            self.lease()
        except BaseException:
            ack.unlink(missing_ok=True)
            raise

    def switch(self, selected):
        if selected not in ('yolo', 'pixel'):
            raise SelectionError('Unknown detector')
        self.idle()
        if self.ready(selected) is not None:
            self.acknowledge()
            return
        before = {unit: self.services.state(unit) for unit in (WORKER, TRACKER)}
        if any(x['ActiveState'] not in ('active', 'inactive') for x in before.values()):
            raise SelectionError('Detector/tracker is still starting or stopping')
        if before[WORKER]['ActiveState'] != 'active' or int(before[WORKER]['MainPID']) <= 1:
            raise SelectionError('Detector service is stopped; restore it before switching')
        previous = read_selection(self.config)
        desired = dict(previous, active_detector=selected)
        # A manually edited file is not proof of the configuration loaded by the
        # running worker. Prefer its identity-bound receipt for rollback.
        try:
            runtime = json.loads((self.root / 'detector_runtime.json').read_text())
            loaded = runtime['selector_config']
            if (runtime['clock_id'] == self.clock
                    and int(before[WORKER]['MainPID']) == runtime['pid']
                    and type(loaded) is dict
                    and set(loaded) == {'active_detector', 'pixel_config'}
                    and loaded['active_detector'] in ('yolo', 'pixel')
                    and isinstance(loaded['pixel_config'], str) and loaded['pixel_config']
                    and settings_hash(loaded) == runtime['selector_sha256']):
                previous = loaded
        except (OSError, ValueError, KeyError, TypeError):
            pass
        if selected == 'pixel':
            from pixel_callable_module import load_color_config
            load_color_config(self.config.parent / desired['pixel_config'])
        archive = self.root / 'detector_switch_backups'
        archive.mkdir(exist_ok=True)
        backup = Path(tempfile.mkdtemp(prefix='widget_', dir=archive))
        shutil.copy2(self.config, backup / self.config.name)
        atomic_text(backup / 'selection.json', json.dumps({'token':self.token, 'clock_id':self.clock,
                    'previous':previous, 'selected':selected, 'services':before}))
        touched = False
        try:
            self.idle()
            touched = True  # a failed stop may already have stopped one unit
            self.services.action('stop', TRACKER, WORKER)
            if any(int(self.services.state(unit)['MainPID']) for unit in before):
                raise SelectionError('Old detector or tracker did not exit')
            self.idle()
            stale_paths = [self.root / name for name in TARGETS]
            stale_paths.append(self.tmp / 'balloon_visual_status.json')
            for path in stale_paths:
                if path.is_symlink():
                    raise SelectionError('Unexpected symbolic target file')
                if path.exists():
                    shutil.copy2(path, backup / path.name)
                    path.unlink()
            atomic_text(self.config, json.dumps(desired, indent=2) + '\n')
            self.lease()
            self.services.action('start', WORKER)
            self.wait_ready(selected)
            if before[TRACKER]['ActiveState'] == 'active':
                self.lease()
                self.services.action('start', TRACKER)
                if self.services.state(TRACKER)['ActiveState'] != 'active':
                    raise SelectionError('Tracker failed to restart')
            self.idle()
            # Cancellation before publishing success must still roll back.
            self.acknowledge()
        except BaseException as exc:
            if touched:
                self.rolling_back = True
                errors = []
                for action in (lambda: (self.tmp / ACK).unlink(missing_ok=True),
                               lambda: self.services.action('stop', WORKER, TRACKER),
                               lambda: atomic_text(self.config, json.dumps(previous, indent=2) + '\n')):
                    try:
                        action()
                    except Exception as error:
                        errors.append(type(error).__name__)
                (self.root / 'detector_runtime.json').unlink(missing_ok=True)
                for unit in (WORKER, TRACKER):
                    if before[unit]['ActiveState'] == 'active':
                        try:
                            self.services.action('start', unit)
                        except Exception as error:
                            errors.append(type(error).__name__)
                atomic_text(backup / 'rollback.json', json.dumps({'restore_commands_completed':not errors,
                            'runtime_readiness_verified':False,
                            'errors':errors, 'reason':str(exc)}))
                if errors:
                    raise SelectionError('Detector switch failed; rollback incomplete, inspect ' + str(backup)) from exc
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--select-detector', choices=('yolo', 'pixel'))
    modes.add_argument('--check-detector', choices=('yolo', 'pixel'))
    parser.add_argument('--token', required=True)
    parser.add_argument('--owner-pid', type=int, required=True)
    args = parser.parse_args()
    selection = Selection(args.token, args.owner_pid)
    def cancelled(*_):
        # Do not allow a second Stop signal to interrupt restoration.
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        raise Cancelled('Selection cancelled; previous configuration restored when possible')
    signal.signal(signal.SIGTERM, cancelled)
    signal.signal(signal.SIGINT, cancelled)
    try:
        import fcntl
        lock = (ROOT / 'detector_switch.lock').open('a')
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        selection.lease()
        (TMP / ACK).unlink(missing_ok=True)
        if args.select_detector:
            selection.switch(args.select_detector)
        else:
            selection.verify(args.check_detector)
            selection.acknowledge()
        return 0
    except (SelectionError, OSError, ValueError, subprocess.SubprocessError) as exc:
        atomic_text(TMP / MESSAGE, '识别方案未就绪：' + str(exc) + '\n')
        print(str(exc), flush=True)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
