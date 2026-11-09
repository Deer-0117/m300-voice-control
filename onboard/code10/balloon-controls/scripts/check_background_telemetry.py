"""Read-only deployment check; stdlib only, also runnable over SSH stdin."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import time

ROOT = Path('/home/uav/offline_balloon_strike_test')
SERVICE = 'psdk-flight-panel-display'
TOPICS = ('velocity', 'quaternion', 'angular_rate', 'flight_status', 'control_device', 'display_mode')
COUNTERS = ('queue_dropped_total', 'write_errors_total')
TAIL_BYTES = 512 * 1024


def _integer(value, maximum=2**64-1):
    return type(value) is int and 0 <= value <= maximum


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _diagnostic(value):
    """Keep corrupt metadata printable without emitting NaN or nested input."""
    if value is None or type(value) in (str, bool, int) or _finite(value):
        return value
    return 'invalid'


def _advanced(before, after):
    ms = (after[0] - before[0]) % 2**32
    us = (after[1] - before[1]) % 2**32
    return 0 < ms < 2**31 or (ms == 0 and 0 < us < 2**31)


def _topic_reason(row, name):
    item = row.get(name)
    if not isinstance(item, dict):
        return 'missing'
    if type(item.get('code')) is not int or item['code'] != 0:
        return 'sdk_error'
    if item.get('valid') is not True or item.get('fresh') is not True:
        return 'invalid_or_stale'
    begin, end = item.get('host_read_begin_ms'), item.get('host_read_end_ms')
    progress, age = item.get('last_progress_ms'), item.get('sample_age_ms')
    if not all(_integer(x) for x in (begin, end, progress, age)):
        return 'invalid_timestamps'
    if not row['sample_begin_ms'] <= begin <= end <= row['sample_end_ms']:
        return 'invalid_read_interval'
    limit = 200 if name in TOPICS[:3] else 500
    if not 0 < progress <= end or age > limit or row['sample_end_ms'] - progress > limit:
        return 'stale_sdk_sample'
    stamp = item.get('sdk_millisecond_raw'), item.get('sdk_microsecond_raw')
    if not all(_integer(x, 2**32-1) for x in stamp) or stamp == (0, 0):
        return 'invalid_sdk_timestamp'
    if name == 'velocity':
        if item.get('health') != 1 or type(item.get('health')) is not int:
            return 'velocity_unhealthy'
        if not all(_finite(item.get(k)) for k in ('n_mps', 'e_mps', 'u_mps')):
            return 'invalid_velocity'
    elif name == 'quaternion':
        values = [item.get(k) for k in ('w', 'x', 'y', 'z')]
        if not all(_finite(x) for x in values) or not .9 <= math.hypot(*values) <= 1.1:
            return 'invalid_quaternion'
    elif name == 'angular_rate':
        if item.get('frame') != 'NED' or not all(_finite(item.get(k)) for k in ('x_radps', 'y_radps', 'z_radps')):
            return 'invalid_angular_rate'
    elif name == 'flight_status':
        if type(item.get('value')) is not int or item['value'] not in (0, 1, 2):
            return 'unknown_flight_status'
    elif name == 'control_device':
        if type(item.get('device_status')) is not int or item['device_status'] not in (0, 1, 4, 5):
            return 'unknown_control_device'
        if type(item.get('vrc_status')) is not int or item['vrc_status'] not in (0, 1):
            return 'unknown_control_flags'
    elif not _integer(item.get('value'), 254):
        return 'unknown_display_mode'
    return None


def inspect(rows, boot, pid, now):
    """Inspect one snapshot. run() additionally requires progress between polls."""
    clock = 'mono-v1:' + boot
    prefix = clock + ':' + str(pid) + ':'
    selected, bad, conflicts = {}, 0, set()
    for row in rows:
        if not isinstance(row, dict) or row.get('schema') != 'background_telemetry_v1' or row.get('event') != 'telemetry_sample':
            continue
        session = row.get('session_id')
        if row.get('clock_id') != clock or not isinstance(session, str) or not session.startswith(prefix):
            continue
        if not session[len(prefix):].isdigit():
            continue
        if not all(_integer(row.get(k)) for k in ('seq', 'ts_ms', 'sample_begin_ms', 'sample_end_ms')):
            bad += 1
            continue
        if (not row['sample_begin_ms'] <= row['sample_end_ms'] == row['ts_ms'] <= now
                or now - row['ts_ms'] > 1000):
            continue
        key = session, row['seq']
        if key in selected and selected[key] != row:
            conflicts.add(key)
        selected[key] = row
    usable = [r for key, r in selected.items() if key not in conflicts]
    latest = max(usable, key=lambda r: r['ts_ms'], default=None)
    current = sorted((r for r in usable if latest is not None and r['session_id'] == latest['session_id']),
                     key=lambda r: r['seq'])
    monotonic = len(current) >= 2 and all(b['ts_ms'] > a['ts_ms'] for a, b in zip(current, current[1:]))
    result = dict(status='TELEMETRY_NOT_READY', clock_id=clock, pid=str(pid),
                  session_id=latest['session_id'] if latest else None,
                  latest_seq=latest['seq'] if latest else None,
                  log_age_ms=now-latest['ts_ms'] if latest else None,
                  recent_samples=len(current), sequence_progress=monotonic,
                  invalid_rows=bad, conflicting_samples=len(conflicts), topics={}, counters={}, missing=[])
    for name in TOPICS:
        last = latest.get(name, {}) if latest else {}
        if not isinstance(last, dict):
            last = {}
        reason = _topic_reason(latest, name) if latest else 'no_current_process_sample'
        stamps = []
        for row in current:
            if _topic_reason(row, name) is None and row[name].get('new_sample') is True:
                item = row[name]
                stamp = item['sdk_millisecond_raw'], item['sdk_microsecond_raw']
                if not stamps or stamp != stamps[-1]:
                    stamps.append(stamp)
        progress = len(stamps) >= 2 and all(_advanced(a, b) for a, b in zip(stamps, stamps[1:]))
        ready = reason is None and progress
        result['topics'][name] = dict(ready=ready, reason=reason or ('ready' if progress else 'waiting_for_sdk_progress'),
                                     code=_diagnostic(last.get('code')), valid=_diagnostic(last.get('valid')),
                                     fresh=_diagnostic(last.get('fresh')),
                                     new_sample=_diagnostic(last.get('new_sample')), sdk_progress=progress,
                                     observed_age_ms=now-last['last_progress_ms'] if _integer(last.get('last_progress_ms')) else None,
                                     frame=_diagnostic(last.get('frame')), health=_diagnostic(last.get('health')),
                                     value=_diagnostic(last.get('value')), device_status=_diagnostic(last.get('device_status')))
        if not ready:
            result['missing'].append(name)
    counters_ok = bool(current)
    for key in COUNTERS:
        values = [r.get(key) for r in current]
        valid = bool(values) and all(_integer(v) for v in values)
        changed = valid and any(a != b for a, b in zip(values, values[1:]))
        result['counters'][key] = {'latest': values[-1] if valid else None, 'changed': changed, 'valid': valid}
        counters_ok &= valid and not changed
    result['counters_stable'] = bool(counters_ok)
    if monotonic and counters_ok and not result['missing'] and not conflicts:
        result['status'] = 'TELEMETRY_OK'
    elif current:
        result['status'] = 'TELEMETRY_PARTIAL'
    return result


def read_tail(root=ROOT):
    rows, errors = [], []
    for name in ('background_telemetry.1.jsonl', 'background_telemetry.jsonl'):
        path = root/name
        try:
            if path.is_symlink():
                raise ValueError('symlinked log')
            fd = os.open(path, os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0))
            with os.fdopen(fd, 'rb') as stream:
                info = os.fstat(stream.fileno())
                if not stat.S_ISREG(info.st_mode):
                    raise ValueError('not a regular log')
                start = max(0, info.st_size - TAIL_BYTES)
                stream.seek(start)
                lines = stream.read(info.st_size-start).splitlines(keepends=True)
                if start:
                    lines = lines[1:]
                for line in lines:
                    if line.endswith(b'\n'):
                        try:
                            rows.append(json.loads(line))
                        except (ValueError, UnicodeError):
                            errors.append(name + ': invalid JSON line')
        except FileNotFoundError:
            continue
        except (OSError, ValueError) as exc:
            errors.append(name + ': ' + str(exc))
    return rows, errors


def _service_name(value):
    # A single unit, never a systemctl option, path or wildcard selector.
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@:-]*', value):
        raise ValueError('service must be a single systemd service name')
    return value


def service_state(service=SERVICE):
    completed = subprocess.run(['systemctl', 'show', _service_name(service), '-p', 'ActiveState', '-p', 'MainPID'],
                               capture_output=True, text=True, timeout=3, check=True)
    values = dict(line.split('=', 1) for line in completed.stdout.splitlines() if '=' in line)
    pid = values.get('MainPID', '0')
    return values.get('ActiveState') == 'active' and pid.isdigit() and int(pid) > 0, pid


def run(timeout=20, root=ROOT, service=SERVICE):
    if not 0 <= timeout <= 30:
        raise ValueError('timeout must be between 0 and 30 seconds')
    service = _service_name(service)
    root = Path(root)
    deadline = time.monotonic() + timeout
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    previous, observed_errors = None, {}
    while True:
        try:
            active, pid = service_state(service)
        except (OSError, subprocess.SubprocessError) as exc:
            active, pid = False, '0'
            service_error = str(exc)
        else:
            service_error = None
        rows, errors = read_tail(root)
        result = inspect(rows, boot, pid, time.monotonic_ns() // 1000000)
        result.update(service=service, log_dir=str(root),
                      service_active=active, service_error=service_error, read_warnings=errors,
                      observation_progress=False)
        key = result['session_id']
        values = tuple(result['counters'][k]['latest'] for k in COUNTERS)
        if key is not None and all(_integer(value) for value in values):
            if key not in observed_errors:
                observed_errors[key] = values
            elif values != observed_errors[key]:
                result['counters_stable'] = False
                result['counter_change_during_check'] = True
        if (previous and key is not None and key == previous['session_id']
                and _integer(result['latest_seq']) and _integer(previous['latest_seq'])
                and result['latest_seq'] > previous['latest_seq']):
            result['observation_progress'] = True
        if (result['status'] == 'TELEMETRY_OK' and active and result['observation_progress']
                and result['counters_stable']):
            return result
        if result['status'] == 'TELEMETRY_OK':
            result['status'] = 'TELEMETRY_PARTIAL'
        if not active:
            result['status'] = 'TELEMETRY_NOT_READY'
        if time.monotonic() >= deadline:
            return result
        previous = result
        time.sleep(min(.25, max(0, deadline-time.monotonic())))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--service', default=SERVICE,
                        help='systemd service whose current MainPID must own the samples (default: %(default)s)')
    parser.add_argument('--log-dir', type=Path, default=ROOT,
                        help='directory containing background_telemetry*.jsonl (default: %(default)s)')
    args = parser.parse_args(argv)
    try:
        result = run(args.timeout, root=args.log_dir, service=args.service)
    except (OSError, ValueError) as exc:
        result = dict(status='TELEMETRY_NOT_READY', error=str(exc))
    print(json.dumps(result, ensure_ascii=True, allow_nan=False))
    print(result['status'])
    return 0 if result['status'] == 'TELEMETRY_OK' else 2


if __name__ == '__main__':
    raise SystemExit(main())
