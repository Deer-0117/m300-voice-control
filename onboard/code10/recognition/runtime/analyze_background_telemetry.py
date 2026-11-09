#!/usr/bin/env python3
"""Summarize observed aircraft telemetry peaks, without video or flight commands."""
from __future__ import annotations

import argparse
from collections import Counter
import glob
import hashlib
import json
import math
from pathlib import Path

SCHEMA = 'background_telemetry_v1'
MANUAL_MODES = {0, 1, 6}  # SDK MANUAL_CTRL, ATTITUDE, P_GPS; requires RC ownership too.
AUTO_MODES = {9, 11, 12, 15, 17, 33, 40}
GROUPS = ('rc_manual_airborne', 'automatic_airborne', 'unknown_airborne',
          'ground', 'unknown_flight_state')
LABELS = {
    'rc_manual_airborne': '空中、RC控制且处于手动/姿态/GPS模式',
    'automatic_airborne': '空中、自动模式或其他控制设备',
    'unknown_airborne': '空中、控制来源无法确认',
    'ground': '地面', 'unknown_flight_state': '飞行状态无法确认',
}
METRICS = {
    'horizontal_speed_mps': ('最高水平地速', 'm/s'),
    'speed_3d_mps': ('最高三维速度', 'm/s'),
    'ascent_speed_mps': ('最高上升速度', 'm/s'),
    'descent_speed_mps': ('最高下降速度', 'm/s'),
    'heading_rate_abs_degps': ('最大航向变化速度估计（姿态差分）', '度/秒'),
    'ned_vertical_angular_rate_abs_degps': ('最大NED竖直轴角速度（非航向导数）', '度/秒'),
}
NOTES = [
    '统计值是有效采样记录中的峰值，不是无人机性能极限；漏采、轮转及遥测延迟可能遗漏真实峰值。',
    '覆盖范围仅指输入文件保留的记录；即使内部序号连续，也不能证明已覆盖整次飞行或最后一次峰值。',
    '只用飞控遥测；不使用视频、相机姿态或控制指令中的yaw_rate_degps。',
    '手飞组要求空中状态、RC实际控制设备、非虚拟RC及手动/姿态/GPS显示模式均有新鲜数据。',
    '状态和控制来源按采样时观测归类，低频状态更新及未知传输延迟仍可能影响切换边界；未知不能算手飞。',
    'sample_age_ms只表示主机观察SDK数据进展后的年龄，不是校准后的物理测量年龄。',
    '航向变化速度由相邻新四元数的航向差除以主机读取时间差估计，不是校准SDK测量时刻上的瞬时导数。',
    '航向差分只使用同一开机、进程、控制分类内的10至200毫秒连续样本，俯仰超过80度时不估计。',
    'NED竖直轴角速度保留为另一指标；飞机倾斜时不能直接当成机头航向变化速度或飞行轨迹转弯速度。',
    '丢队列和写盘失败计数是各会话已观察到的累计最大值，不逐行相加；未落盘的最后计数无法恢复。',
]


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def integer(value, maximum=2**64 - 1):
    return type(value) is int and 0 <= value <= maximum


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key')
        result[key] = value
    return result


def _topic(row, name, max_age_ms):
    topic = row.get(name)
    if not isinstance(topic, dict):
        return None
    if (topic.get('valid') is not True or topic.get('fresh') is not True
            or type(topic.get('code')) is not int or topic['code'] != 0):
        return None
    begin, end = topic.get('host_read_begin_ms'), topic.get('host_read_end_ms')
    progress, age = topic.get('last_progress_ms'), topic.get('sample_age_ms')
    if not all(integer(x) for x in (begin, end, progress, age)):
        return None
    if not row['sample_begin_ms'] <= begin <= end <= row['sample_end_ms']:
        return None
    if not 0 < progress <= end or max(age, row['sample_end_ms'] - progress) > max_age_ms:
        return None
    stamp = (topic.get('sdk_millisecond_raw'), topic.get('sdk_microsecond_raw'))
    if not all(integer(x, 2**32 - 1) for x in stamp) or stamp == (0, 0):
        return None
    return topic


def _classify(row):
    status = _topic(row, 'flight_status', 500)
    if status is None or type(status.get('value')) is not int or status['value'] not in (0, 1, 2):
        return 'unknown_flight_state'
    if status['value'] != 2:
        return 'ground'
    display = _topic(row, 'display_mode', 500)
    if display is None or type(display.get('value')) is not int:
        return 'unknown_airborne'
    if display['value'] in AUTO_MODES:
        return 'automatic_airborne'
    control = _topic(row, 'control_device', 500)
    if control is None:
        return 'unknown_airborne'
    device, virtual = control.get('device_status'), control.get('vrc_status')
    if type(device) is not int or type(virtual) not in (int, bool) or virtual not in (0, 1):
        return 'unknown_airborne'
    if device in (1, 4, 5) or (device == 0 and virtual == 1):
        return 'automatic_airborne'
    if device == 0 and virtual == 0 and display['value'] in MANUAL_MODES:
        return 'rc_manual_airborne'
    return 'unknown_airborne'


def _motion(row, name, fields, require_new=True):
    topic = _topic(row, name, 200)
    if topic is None or (require_new and topic.get('new_sample') is not True):
        return None
    values = tuple(topic.get(key) for key in fields)
    if not all(finite(value) for value in values):
        return None
    if name == 'velocity' and (type(topic.get('health')) is not int or topic['health'] != 1):
        return None
    if name == 'angular_rate' and topic.get('frame') != 'NED':
        return None
    stamp = (topic['sdk_millisecond_raw'], topic['sdk_microsecond_raw'])
    return values, stamp, topic['host_read_end_ms']


def _normalize(row, source, line):
    if (not isinstance(row.get('clock_id'), str) or not row['clock_id'].startswith('mono-v1:')
            or not isinstance(row.get('session_id'), str) or not row['session_id']):
        raise ValueError('missing identity')
    for key in ('seq', 'ts_ms', 'sample_begin_ms', 'sample_end_ms'):
        if not integer(row.get(key)):
            raise ValueError('invalid time/sequence')
    if not row['sample_begin_ms'] <= row['sample_end_ms'] or row['ts_ms'] != row['sample_end_ms']:
        raise ValueError('invalid sample interval')
    quaternion = _motion(row, 'quaternion', ('w', 'x', 'y', 'z'), require_new=False)
    cached_quaternion = (quaternion is not None and row['quaternion'].get('new_sample') is False
                         and _attitude(quaternion[0]) is not None)
    return dict(clock_id=row['clock_id'], session_id=row['session_id'], seq=row['seq'], ts_ms=row['ts_ms'],
                group=_classify(row), source=source, line=line,
                velocity=_motion(row, 'velocity', ('n_mps', 'e_mps', 'u_mps')),
                quaternion=_motion(row, 'quaternion', ('w', 'x', 'y', 'z')),
                cached_quaternion=cached_quaternion,
                angular_rate=_motion(row, 'angular_rate', ('x_radps', 'y_radps', 'z_radps')),
                queue_dropped_total=row.get('queue_dropped_total'), write_errors_total=row.get('write_errors_total'))


def _bucket():
    return {'rows': 0, 'metrics': {name: {'count': 0, 'maximum': None, 'unit': unit}
                                for name, (_, unit) in METRICS.items()}}


def _peak(bucket, metric, value, row, sample_ts, sdk_stamp, signed=None, previous_seq=None):
    if not finite(value):
        return False
    data = bucket['metrics'][metric]
    data['count'] += 1
    if data['maximum'] is None or value > data['maximum']['value']:
        point = {key: row[key] for key in ('clock_id', 'session_id', 'seq', 'ts_ms', 'source', 'line')}
        point.update(value=value, host_sample_ts_ms=sample_ts,
                     sdk_millisecond_raw=sdk_stamp[0], sdk_microsecond_raw=sdk_stamp[1])
        if signed is not None:
            point['signed_value'] = signed
        if previous_seq is not None:
            point['previous_seq'] = previous_seq
        data['maximum'] = point
    return True


def _attitude(values):
    norm = math.hypot(*values)
    if not .9 <= norm <= 1.1:
        return None
    w, x, y, z = (value / norm for value in values)
    pitch = math.degrees(math.asin(max(-1, min(1, 2 * (w * y - z * x)))))
    if abs(pitch) > 80:
        return None
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return yaw, pitch


def expand_inputs(paths):
    """Expand quoted globs/directories ourselves, including on Windows shells."""
    if isinstance(paths, (str, Path)):
        paths = [paths]
    result = []
    for value in paths:
        path = Path(value)
        if path.is_dir():
            matches = sorted(path.glob('background_telemetry*.jsonl'))
        elif path.is_file():
            matches = [path]
        else:
            matches = [Path(match) for match in sorted(glob.glob(str(path))) if Path(match).is_file()]
        if not matches:
            raise FileNotFoundError(f'No telemetry input matches: {value}')
        result.extend(match.resolve() for match in matches)
    return list(dict.fromkeys(result))


def analyze(paths, clock_id=None):
    """Read immutable input logs; returns grouped peaks without writing files."""
    quality = Counter()
    sources, records, conflicts = [], {}, set()
    available_clocks = set()
    for path in expand_inputs(paths):
        digest = hashlib.sha256()
        file_quality = Counter()
        with path.open('rb') as stream:
            for line_number, line in enumerate(stream, 1):
                digest.update(line)
                file_quality['lines'] += 1
                try:
                    row = json.loads(line, object_pairs_hook=_strict_object)
                    if not isinstance(row, dict):
                        raise ValueError('not an object')
                except (ValueError, UnicodeError):
                    file_quality['invalid_json'] += 1
                    continue
                if row.get('schema') != SCHEMA or row.get('event') != 'telemetry_sample':
                    file_quality['other_events'] += 1
                    continue
                if isinstance(row.get('clock_id'), str):
                    available_clocks.add(row['clock_id'])
                if clock_id is not None and row.get('clock_id') != clock_id:
                    file_quality['other_clocks'] += 1
                    continue
                try:
                    item = _normalize(row, str(path), line_number)
                except ValueError:
                    file_quality['invalid_samples'] += 1
                    continue
                identity = (item['clock_id'], item['session_id'], item['seq'])
                fingerprint = hashlib.sha256(json.dumps(row, sort_keys=True, separators=(',', ':')).encode()).digest()
                if identity in records:
                    if records[identity][0] != fingerprint:
                        conflicts.add(identity)
                        file_quality['conflicting_samples'] += 1
                    else:
                        file_quality['duplicate_samples'] += 1
                else:
                    records[identity] = (fingerprint, item)
                    file_quality['unique_samples'] += 1
        quality.update(file_quality)
        sources.append(dict(path=str(path), sha256=digest.hexdigest(), quality=dict(file_quality)))
    grouped = {}
    for identity, (_, item) in records.items():
        if identity in conflicts:
            continue
        grouped.setdefault(identity[:2], []).append(item)
    sessions = []
    for (boot, session_id), rows in sorted(grouped.items()):
        rows.sort(key=lambda row: row['seq'])
        groups = {group: _bucket() for group in GROUPS}
        counters = {'queue_dropped_total_observed_max': None, 'write_errors_total_observed_max': None}
        diagnostics = Counter()
        seen = {name: set() for name in ('velocity', 'quaternion', 'angular_rate')}
        latest = {name: None for name in seen}
        previous = None
        last_row = None
        first_ms, last_ms = None, None
        for row in rows:
            if last_row is not None:
                gap = row['ts_ms'] - last_row['ts_ms']
                if gap <= 0:
                    diagnostics['nonmonotonic_samples'] += 1
                    previous = None
                    continue
                diagnostics['largest_sample_gap_ms'] = max(diagnostics['largest_sample_gap_ms'], gap)
                diagnostics['missing_sequence_samples'] += max(0, row['seq'] - last_row['seq'] - 1)
                if gap > 200:
                    diagnostics['gaps_over_200ms'] += 1
                    previous = None
                if row['group'] != last_row['group']:
                    diagnostics['control_group_transitions'] += 1
                    previous = None
            last_row = row
            first_ms = row['ts_ms'] if first_ms is None else first_ms
            last_ms = row['ts_ms']
            for key in ('queue_dropped_total', 'write_errors_total'):
                value = row[key]
                target = key + '_observed_max'
                if integer(value):
                    counters[target] = max(counters[target] or 0, value)
            bucket = groups[row['group']]
            bucket['rows'] += 1
            for name in ('velocity', 'angular_rate', 'quaternion'):
                motion = row[name]
                if motion is None:
                    diagnostics[name + '_not_new_valid_fresh'] += 1
                    if name == 'quaternion' and not row['cached_quaternion']:
                        previous = None
                    continue
                values, stamp, host_ts = motion
                if stamp in seen[name]:
                    diagnostics[name + '_duplicate_sdk_sample'] += 1
                    if name == 'quaternion':
                        previous = None
                    continue
                seen[name].add(stamp)
                # SDK millisecond counter can wrap at uint32; microsecond is a
                # separate raw SDK field. Do not pretend either is host time.
                old = latest[name]
                if old is not None:
                    delta_ms = (stamp[0] - old[0][0]) % (2**32)
                    advanced = (0 < delta_ms < 2**31 or
                                (delta_ms == 0 and 0 < (stamp[1] - old[0][1]) % (2**32) < 2**31))
                    if not advanced or host_ts <= old[1]:
                        diagnostics[name + '_nonmonotonic_topic'] += 1
                        if name == 'quaternion':
                            previous = None
                        continue
                latest[name] = stamp, host_ts
                if name == 'velocity':
                    n, e, u = values
                    for metric, value in (('horizontal_speed_mps', math.hypot(n, e)),
                                          ('speed_3d_mps', math.hypot(n, e, u)),
                                          ('ascent_speed_mps', max(0, u)), ('descent_speed_mps', max(0, -u))):
                        if not _peak(bucket, metric, value, row, host_ts, stamp):
                            diagnostics['nonfinite_derived_values'] += 1
                elif name == 'angular_rate':
                    signed = math.degrees(values[2])
                    if not _peak(bucket, 'ned_vertical_angular_rate_abs_degps', abs(signed), row, host_ts, stamp, signed):
                        diagnostics['nonfinite_derived_values'] += 1
                else:
                    attitude = _attitude(values)
                    if attitude is None:
                        diagnostics['quaternion_invalid_or_near_vertical'] += 1
                        previous = None
                        continue
                    yaw = attitude[0]
                    if previous is not None and previous[2] == row['group']:
                        dt = host_ts - previous[0]
                        if 10 <= dt <= 200:
                            delta = (yaw - previous[1] + 180) % 360 - 180
                            if abs(delta) < 180:
                                signed = delta * 1000 / dt
                                _peak(bucket, 'heading_rate_abs_degps', abs(signed), row, host_ts, stamp,
                                      signed, previous[3])
                            else:
                                diagnostics['ambiguous_half_turn'] += 1
                        else:
                            diagnostics['heading_time_interval_rejected'] += 1
                    previous = host_ts, yaw, row['group'], row['seq']
        sessions.append(dict(clock_id=boot, session_id=session_id, first_ts_ms=first_ms, last_ts_ms=last_ms,
                             first_seq=rows[0]['seq'], last_seq=rows[-1]['seq'],
                             prefix_missing_possible=rows[0]['seq'] > 1,
                             complete_flight_verified=False,
                             groups=groups, diagnostics=dict(diagnostics), **counters))
    return dict(schema='background_telemetry_report_v1', selected_clock_id=clock_id,
                available_clocks=sorted(available_clocks), sources=sources, input_quality=dict(quality),
                excluded_conflicting_identities=len(conflicts), sessions=sessions, notes=NOTES)


def format_text(report):
    lines = ['后台遥测：已记录样本中的峰值', '数据来自飞机遥测，不依赖摄像头。', '']
    if not report['sessions']:
        lines += ['没有匹配且有效的遥测样本，不能将最大值填写为0。', '']
    for session in report['sessions']:
        lines += [f"开机：{session['clock_id']}", f"记录会话：{session['session_id']}",
                  f"记录覆盖：{session['first_ts_ms']}～{session['last_ts_ms']} ms（不是自动识别的起降时刻）",
                  f"保留序号：{session['first_seq']}～{session['last_seq']}；未证明包含完整航次。",
                  f"已观察累计丢队列：{session['queue_dropped_total_observed_max']}；写盘错误：{session['write_errors_total_observed_max']}"]
        if session['prefix_missing_possible']:
            lines.append('会话开头不在输入中，之前的速度或转向峰值可能已被日志轮转覆盖。')
        for group in GROUPS:
            bucket = session['groups'][group]
            if not bucket['rows']:
                if group == 'rc_manual_airborne':
                    lines.append('手飞：没有可确认的空中RC手动样本。')
                continue
            lines += ['', f"{LABELS[group]}：{bucket['rows']}个记录点"]
            for metric, (label, unit) in METRICS.items():
                data = bucket['metrics'][metric]
                peak = data['maximum']
                if peak is None:
                    lines.append(f'{label}：无有效样本')
                else:
                    lines.append(f"{label}：{peak['value']:.3f} {unit}；有效样本 {data['count']}；"
                                 f"主机时间 {peak['host_sample_ts_ms']} ms，序号 {peak['seq']}")
        lines += ['', '质量检查：' + json.dumps(session['diagnostics'], ensure_ascii=False), '']
    lines += ['读取质量：' + json.dumps(report['input_quality'], ensure_ascii=False), '', '说明：']
    lines += [f'{index}. {note}' for index, note in enumerate(report['notes'], 1)]
    return '\n'.join(lines) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', type=Path, nargs='+')
    parser.add_argument('--clock-id')
    parser.add_argument('--json-out', type=Path)
    parser.add_argument('--text-out', type=Path)
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args(argv)
    try:
        inputs = expand_inputs(args.inputs)
        outputs = [p.resolve() for p in (args.json_out, args.text_out) if p is not None]
        if len(set(outputs)) != len(outputs) or any(
                out == src or (out.exists() and src.exists() and out.samefile(src))
                for out in outputs for src in inputs):
            raise ValueError('Output must not overwrite an input or another output')
        if not args.overwrite and any(p.exists() for p in outputs):
            raise ValueError('Output exists; choose a new path or pass --overwrite')
        report = analyze(inputs, args.clock_id)
        text = format_text(report)
        encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        for path, content in ((args.json_out, encoded), (args.text_out, text)):
            if path is not None:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding='utf-8')
        print(text)
        return 0
    except (OSError, ValueError) as exc:
        parser.exit(1, str(exc) + '\n')


if __name__ == '__main__':
    main()
