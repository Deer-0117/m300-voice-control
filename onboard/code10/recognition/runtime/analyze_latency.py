#!/usr/bin/env python3
"""Audit software latency by boot identity; time ranges are not proof of flight."""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

DEFAULT_LOG = Path(__file__).resolve().parent / 'latency_events.jsonl'
NOTES = {
    'request_queue_wait': '请求提交到检测开始；包含在画面到检测开始中。',
    'capture_to_detection_start': '解码画面时间戳到检测开始，不是曝光时间。',
    'yolo_predict': '全图检测调用，含接口内部处理，不是纯GPU内核耗时。',
    'yolo_far_tiles': '远距离分块补检合计；仅实际补检时记录。',
    'detection_total': '检测开始到完成，含全图、可选补检及其他处理。',
    'capture_to_detection_finish': '解码画面时间戳到检测完成，含多个前述阶段。',
    'mil_published_update': '成功发布结果时的MIL更新耗时，仅为已发布子集。',
    'mil_lost_update': 'MIL返回但目标丢失的更新耗时，与成功发布子集分开。',
}
WARNINGS = [
    '阶段存在包含关系，不能相加；软件调用耗时的倒数不是系统FPS。',
    'capture沿用旧字段名，表示解码端画面时间，尚未标定为真实曝光时间。',
    '开机周期或手工时间范围可能混有地面与飞行操作，不自动标记为纯空中试验。',
    '预测提前量不是算法计算耗时；SDK调用返回不是飞机物理动作完成。',
    '旧MIL日志只覆盖已发布及明确丢失子集，不覆盖所有异常或未发布更新。',
    '缺完成日志仅表示记录覆盖不完整，不能推断为推理失败率。',
    '原始P95使用排序线性插值；聚合计时P95为直方图桶上界，二者不能混称。',
    '计时器丢弃/写入异常计数是进程累计值，各profile仅取已观察最大值，不跨阶段或窗口相加；缺字段表示未知。写入失败批次可能已有部分落盘，不等于精确丢失条数。',
]
PROFILER_COUNTERS = ('dropped_windows_total', 'dropped_stage_samples', 'write_failed_summaries', 'write_failures')


def number(v):
    try:
        return type(v) in (int, float) and math.isfinite(v) and v >= 0
    except OverflowError:
        return False


def stamp(v):
    return type(v) is int and 0 <= v <= 2**64-1


def clock(r):
    v = r.get('clock_id')
    return v if isinstance(v, str) and v.startswith('mono-v1:') and 8 < len(v) <= 100 else None


def percentile(values, fraction):
    if not values or not 0 <= fraction <= 1:
        raise ValueError('percentile needs samples and fraction in [0,1]')
    values = sorted(values)
    pos = (len(values)-1)*fraction
    lo, hi, w = math.floor(pos), math.ceil(pos), pos-math.floor(pos)
    return values[lo]*(1-w)+values[hi]*w


def distribution(values):
    return dict(count=len(values), min_ms=min(values), mean_ms=sum(v/len(values) for v in values),
                p50_ms=percentile(values, .5), p95_ms=percentile(values, .95), max_ms=max(values),
                percentile_method='linear_interpolation')


def strict_object(pairs):
    d = {}
    for key, value in pairs:
        if key in d:
            raise ValueError('duplicate JSON key')
        d[key] = value
    return d


def read_jsonl(path):
    path = Path(path).resolve(strict=True)
    rows, q, sha = [], Counter(), hashlib.sha256()
    with path.open('rb') as stream:
        for line, raw in enumerate(stream, 1):
            sha.update(raw)
            q['lines'] += 1
            if not raw.strip():
                q['blank_lines'] += 1
                continue
            try:
                if len(raw) > 4*1024*1024:
                    raise ValueError('line too large')
                d = json.loads(raw.decode('utf-8-sig'), object_pairs_hook=strict_object,
                               parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
                if not isinstance(d, dict):
                    raise ValueError('not an object')
                d['_line'] = line
                rows.append(d)
            except (ValueError, UnicodeError, RecursionError):
                q['invalid_json_lines'] += 1
    return rows, dict(path=str(path), sha256=sha.hexdigest(), quality=dict(q))


def valid_latency(r):
    return (number(r.get('duration_ms')) and stamp(r.get('ts_ms')) and
            isinstance(r.get('stage'), str) and 0 < len(r['stage']) <= 128 and
            ('seq' not in r or stamp(r['seq'])))


def read_events(path):
    """Legacy reader API; aggregation must still separate clock identities."""
    rows, src = read_jsonl(path)
    good = [r for r in rows if valid_latency(r)]
    return good, src['quality'].get('invalid_json_lines', 0)+len(rows)-len(good)


def select_rows(rows, src, selected_clock, start, end, validator=valid_latency):
    q, selected = Counter(src['quality']), []
    for r in rows:
        if not validator(r):
            q['invalid_record'] += 1
        elif clock(r) is None:
            q['unclocked'] += 1
        elif clock(r) != selected_clock:
            q['other_clock'] += 1
        elif (start is not None and r['ts_ms'] < start) or (end is not None and r['ts_ms'] > end):
            q['outside_window'] += 1
        else:
            q['selected'] += 1
            selected.append(r)
    src['quality'] = dict(q)
    return selected


def grouped_stages(rows):
    groups = defaultdict(list)
    for r in rows:
        groups[r['stage']].append(float(r['duration_ms']))
    return {key: dict(distribution(vals), meaning=NOTES.get(key, '需核对原始阶段定义。'))
            for key, vals in sorted(groups.items())}


def detection_strata(events, rows, selected_clock):
    legacy, modern, q = defaultdict(list), defaultdict(list), Counter()
    for d in rows:
        if clock(d) != selected_clock:
            q['other_or_unknown_clock'] += 1
            continue
        valid = (stamp(d.get('seq')) and stamp(d.get('detection_started_ts_ms')) and
                 stamp(d.get('detection_finished_ts_ms')) and stamp(d.get('capture_ts_ms')) and
                 d['capture_ts_ms'] <= d['detection_started_ts_ms'] <= d['detection_finished_ts_ms'] and
                 d.get('ok') is True and number(d.get('far_tile_latency_ms')))
        if not valid:
            q['invalid_detection_record'] += 1
            if stamp(d.get('seq')):
                legacy[d['seq']].append(None)
                if isinstance(d.get('worker_session_id'), str) and stamp(d.get('detection_started_ts_ms')):
                    modern[(d['worker_session_id'], d['seq'], d['detection_started_ts_ms'])].append(None)
            continue
        legacy[d['seq']].append(d)
        worker = d.get('worker_session_id')
        if isinstance(worker, str) and worker:
            modern[(worker, d['seq'], d['detection_started_ts_ms'])].append(d)
    q['ambiguous_legacy_sequences'] = sum(len(v) > 1 for v in legacy.values())
    groups = defaultdict(list)
    for e in events:
        if not stamp(e.get('seq')):
            q['unmatched_latency_event'] += 1
            continue
        worker, begin = e.get('worker_session_id'), e.get('detection_started_ts_ms')
        if (worker is not None or begin is not None) and not (isinstance(worker, str) and worker and stamp(begin)):
            q['ambiguous_or_missing_match'] += 1
            continue
        candidates = modern.get((worker, e['seq'], begin), []) if worker is not None or begin is not None else legacy.get(e['seq'], [])
        if len(candidates) != 1 or candidates[0] is None:
            q['ambiguous_or_missing_match'] += 1
            continue
        d = candidates[0]
        start, finish, capture = d['detection_started_ts_ms'], d['detection_finished_ts_ms'], d['capture_ts_ms']
        request = d.get('request_ts_ms')
        expected = {'request_queue_wait': start-request if stamp(request) and request <= start else None,
            'capture_to_detection_start': start-capture, 'yolo_predict': d.get('latency_ms'),
            'yolo_far_tiles': d.get('far_tile_latency_ms'), 'detection_total': finish-start,
            'capture_to_detection_finish': finish-capture}.get(e['stage'])
        total = e['stage'] in ('detection_total', 'capture_to_detection_finish')
        times_match = e['ts_ms'] >= finish if total else start <= e['ts_ms'] <= finish
        if not number(expected) or abs(e['duration_ms']-expected) > .002 or not times_match:
            q['endpoint_or_duration_mismatch'] += 1
            continue
        groups['with_far_tiles' if d['far_tile_latency_ms'] > 0 else 'without_far_tiles'].append(e)
        q['matched_latency_events'] += 1
    return dict(quality=dict(q), groups={key: grouped_stages(vals) for key, vals in sorted(groups.items())},
                rule='same boot; unique seq or exact worker/seq/start; matching timestamps and durations; no guessed restart')


def aggregate_performance(rows, src, selected_clock, start_ms, end_ms):
    q, eligible, ids, groups, diagnostics = Counter(src['quality']), [], Counter(), {}, {}
    for r in rows:
        try:
            if clock(r) is None:
                q['unclocked'] += 1
                continue
            if clock(r) != selected_clock:
                q['other_clock'] += 1
                continue
            edges, hist = r['histogram_upper_bounds_ns'], r['histogram_counts']
            n, success, low, high, total = (r[k] for k in ('count', 'success_count', 'min_ns', 'max_ns', 'sum_ns'))
            start, end = r['window_start_ns'], r['window_end_ns']
            if not (type(r['schema_version']) is int and r['schema_version'] == 1 and r['event'] == 'performance_summary'):
                raise ValueError("invalid performance aggregate")
            if not (isinstance(edges, list) and len(edges) >= 2 and edges[-1] is None):
                raise ValueError("invalid performance aggregate")
            if not (all(stamp(x) and x > 0 for x in edges[:-1]) and all(a < b for a, b in zip(edges[:-2], edges[1:-1]))):
                raise ValueError("invalid performance aggregate")
            if not (isinstance(hist, list) and len(hist) == len(edges) and all(stamp(x) for x in hist)):
                raise ValueError("invalid performance aggregate")
            if not (stamp(n) and stamp(success) and success <= n and sum(hist) == n):
                raise ValueError("invalid performance aggregate")
            if not (all(stamp(v) for v in (start, end, total, r['invalid_count'], r['window_id']))):
                raise ValueError("invalid performance aggregate")
            if not (start <= end):
                raise ValueError("invalid performance aggregate")
            if n:
                if not (stamp(low) and stamp(high) and low <= high and n*low <= total <= n*high):
                    raise ValueError("invalid performance aggregate")
                minimum_bucket = next(i for i, bound in enumerate(edges) if bound is None or low <= bound)
                maximum_bucket = next(i for i, bound in enumerate(edges) if bound is None or high <= bound)
                if not (hist[minimum_bucket] and hist[maximum_bucket]):
                    raise ValueError("invalid performance aggregate")
                if not (not any(hist[:minimum_bucket]) and not any(hist[maximum_bucket+1:])):
                    raise ValueError("invalid performance aggregate")
            else:
                if not (all(v is None or type(v) is int and v == 0 for v in (low, high)) and total == 0):
                    raise ValueError("invalid performance aggregate")
            if not (r['aircraft_state'] in ('air', 'ground', 'unknown')):
                raise ValueError("invalid performance aggregate")
            if not (r.get('prediction_mode') is None or type(r['prediction_mode']) is int and r['prediction_mode'] in (0, 1, 2, 3)):
                raise ValueError("invalid performance aggregate")
            if not (r.get('no_flight') is None or type(r['no_flight']) is bool or type(r['no_flight']) is int and r['no_flight'] in (0, 1)):
                raise ValueError("invalid performance aggregate")
            if not (isinstance(r['profile_id'], str) and r['profile_id'] and isinstance(r['stage'], str) and r['stage']):
                raise ValueError("invalid performance aggregate")
            if not (isinstance(r.get('mission_token', ''), str) and isinstance(r.get('source', ''), str)):
                raise ValueError("invalid performance aggregate")
            if not all(name not in r or stamp(r[name]) for name in (*PROFILER_COUNTERS, 'previous_flush_ns')):
                raise ValueError('invalid profiler diagnostic counter')
            if (start_ms is not None and start < start_ms*1000000) or (end_ms is not None and end > end_ms*1000000):
                q['window_not_fully_contained'] += 1
                continue
            key = tuple(r.get(k) for k in ('profile_id', 'mission_token', 'source', 'stage', 'prediction_mode', 'no_flight', 'aircraft_state'))
            identity = key+(r['window_id'],)
            ids[identity] += 1
            eligible.append((key+(tuple(edges),), identity, r))
            diagnostic_key = (r['profile_id'], r.get('source'))
            if diagnostic_key not in diagnostics:
                diagnostics[diagnostic_key] = dict(profile_id=r['profile_id'], source=r.get('source'),
                    cumulative_maxima={name: None for name in PROFILER_COUNTERS}, previous_flush_max_ns=None)
            diag = diagnostics[diagnostic_key]
            for name in PROFILER_COUNTERS:
                if name in r:
                    old = diag['cumulative_maxima'][name]
                    diag['cumulative_maxima'][name] = r[name] if old is None else max(old, r[name])
            if 'previous_flush_ns' in r:
                old = diag['previous_flush_max_ns']
                diag['previous_flush_max_ns'] = r['previous_flush_ns'] if old is None else max(old, r['previous_flush_ns'])
        except (AssertionError, KeyError, TypeError, ValueError):
            q['invalid_aggregate'] += 1
    # Distinct IDs do not make overlapping windows independent samples.
    overlap = set()
    by_group = defaultdict(list)
    for _, identity, r in eligible:
        if ids[identity] == 1:
            by_group[identity[:-1]].append((identity, r))
    for windows in by_group.values():
        ordered = sorted(windows, key=lambda item: item[1]['window_start_ns'])
        active = []
        for identity, r in ordered:
            active = [(other, end) for other, end in active if end > r['window_start_ns']]
            if active:
                overlap.add(identity)
                overlap.update(other for other, _ in active)
            active.append((identity, r['window_end_ns']))
    for key, identity, r in eligible:
        if ids[identity] != 1:
            q['duplicate_window_rows_excluded'] += 1
            continue
        if identity in overlap:
            q['overlapping_window_rows_excluded'] += 1
            continue
        if key not in groups:
            groups[key] = {k: r.get(k) for k in ('profile_id', 'mission_token', 'source', 'stage', 'prediction_mode', 'no_flight', 'aircraft_state')}
            groups[key].update(windows=0, count=0, success_count=0, sum_ns=0, min_ns=None, max_ns=None,
                               invalid_count=0, histogram_counts=[0]*len(key[-1]), histogram_upper_bounds_ns=list(key[-1]))
        d = groups[key]
        d['windows'] += 1
        for name in ('count', 'success_count', 'sum_ns', 'invalid_count'):
            d[name] += r[name]
        if r['count']:
            d['min_ns'] = r['min_ns'] if d['min_ns'] is None else min(d['min_ns'], r['min_ns'])
            d['max_ns'] = r['max_ns'] if d['max_ns'] is None else max(d['max_ns'], r['max_ns'])
        d['histogram_counts'] = [a+b for a, b in zip(d['histogram_counts'], r['histogram_counts'])]
        q['selected_windows'] += 1
    result = list(groups.values())
    for r in result:
        r['mean_ms'] = r['sum_ns']/r['count']/1e6 if r['count'] else None
        r['p95_upper_bound_ms'] = None
        accum = 0
        if r['count']:
            for edge, count in zip(r['histogram_upper_bounds_ns'], r['histogram_counts']):
                accum += count
                if accum >= (r['count']*95+99)//100:
                    r['p95_upper_bound_ms'] = edge/1e6 if edge is not None else None
                    break
        r['percentile_method'] = 'histogram_upper_bound; null means no samples or overflow'
    src['quality'] = dict(q)
    src['profiler_diagnostics'] = dict(profiles=list(diagnostics.values()),
        scope='max observed on valid fully contained windows, including duplicate/overlapping rows excluded from duration aggregation',
        interpretation='cumulative process counters, not interval totals; null unknown; write failure batches may be partly persisted')
    return result


def build_report(log, *, clock_id=None, start_ms=None, end_ms=None, tracker_log=None, detection_log=None, performance_log=None):
    if any(v is not None and not stamp(v) for v in (start_ms, end_ms)) or start_ms is not None and end_ms is not None and start_ms > end_ms:
        raise ValueError('invalid inclusive time window')
    rows, src = read_jsonl(log)
    clocks = sorted({clock(r) for r in rows if valid_latency(r) and clock(r) is not None})
    if clock_id is None:
        if len(clocks) != 1:
            raise ValueError('select --clock-id; valid identified clocks: '+', '.join(clocks))
        clock_id = clocks[0]
    if clock_id not in clocks:
        raise ValueError('selected clock has no valid latency records')
    selected = select_rows(rows, src, clock_id, start_ms, end_ms)
    if not selected:
        raise ValueError('no valid latency samples in selected clock/window')
    report = dict(schema_version=1, scope=dict(clock_id=clock_id, start_ms=start_ms, end_ms=end_ms,
        boundaries='inclusive event timestamp; aggregate windows require full containment',
        flight_attribution='not inferred; may include ground and flight operations'), available_clocks=clocks,
        sources={'latency': src}, warnings=list(WARNINGS), stages=grouped_stages(selected))
    if tracker_log:
        rows, src = read_jsonl(tracker_log)
        relevant = [dict(r, stage='mil_published_update' if r['event'] == 'tracker_target_written' else 'mil_lost_update', duration_ms=r.get('update_ms'))
                    for r in rows if r.get('event') in ('tracker_target_written', 'tracker_lost')]
        src['quality']['other_events'] = len(rows)-len(relevant)
        report['tracker_subsets'] = grouped_stages(select_rows(relevant, src, clock_id, start_ms, end_ms))
        report['sources']['tracker'] = src
    if detection_log:
        rows, src = read_jsonl(detection_log)
        report['detection_strata'] = detection_strata(selected, rows, clock_id)
        report['sources']['detection'] = src
    if performance_log:
        paths = performance_log if isinstance(performance_log, (list, tuple)) else [performance_log]
        if len({Path(v).resolve() for v in paths}) != len(paths):
            raise ValueError('duplicate performance source path')
        report['aggregate_performance'] = []
        for index, path in enumerate(paths):
            rows, src = read_jsonl(path)
            groups = aggregate_performance(rows, src, clock_id, start_ms, end_ms)
            for group in groups:
                group['source_file'] = src['path']
            report['aggregate_performance'].extend(groups)
            report['sources']['performance_'+str(index+1)] = src
    return report


def escape(v):
    return str(v).replace('|', '\\|').replace('\n', ' ').replace('\r', ' ')


def markdown(report):
    s = report['scope']
    lines = ['# 机载软件耗时报告', '', f"开机标识：`{s['clock_id']}`。",
             f"事件时间范围：{s['start_ms']} ～ {s['end_ms']} ms（空值表示不限制）。",
             '时间范围仅用于选择记录，不证明全程处于空中。', '']
    def table(title, groups):
        lines.extend([f'## {title}', '', '| 阶段 | N | 平均 ms | P50 ms | P95 ms | 最大 ms |', '|---|---:|---:|---:|---:|---:|'])
        for name, d in groups.items():
            lines.append('| '+escape(name)+' | '+str(d['count'])+' | '+' | '.join(f"{d[k]:.3f}" for k in ('mean_ms', 'p50_ms', 'p95_ms', 'max_ms'))+' |')
        lines.append('')
    table('原始逐次计时', report['stages'])
    if 'tracker_subsets' in report:
        table('MIL已记录子集', report['tracker_subsets'])
    if 'detection_strata' in report:
        for name, groups in report['detection_strata']['groups'].items():
            table('检测分组：'+name, groups)
        lines.extend(['匹配审计：`'+json.dumps(report['detection_strata']['quality'], sort_keys=True)+'`', ''])
    if 'aggregate_performance' in report:
        lines.extend(['## 聚合算法计时', '', '| 阶段 / 来源 / profile / 任务 / 模式 / 禁飞 / 状态 | N | 成功 | 平均 ms | P95桶上界 ms |', '|---|---:|---:|---:|---:|'])
        for d in report['aggregate_performance']:
            label = ' / '.join(str(d[k]) for k in ('stage', 'source', 'profile_id', 'mission_token', 'prediction_mode', 'no_flight', 'aircraft_state'))
            mean = '不可用' if d['mean_ms'] is None else f"{d['mean_ms']:.6f}"
            p95 = '无样本或超出最大桶' if d['p95_upper_bound_ms'] is None else f"≤{d['p95_upper_bound_ms']:.6f}"
            lines.append(f"| {escape(label)} | {d['count']} | {d['success_count']} | {mean} | {p95} |")
        lines.append('')
    lines.extend(['## 口径与缺口', '', *('- '+w for w in report['warnings']), '', '## 阶段定义', ''])
    lines.extend('- `'+escape(k)+'`：'+v['meaning'] for k, v in report['stages'].items())
    lines.extend(['', '## 原始文件与质量检查', ''])
    for name, src in report['sources'].items():
        lines.extend([f"- {name}：`{escape(src['path'])}`", f"  SHA-256：`{src['sha256']}`；计数：`{json.dumps(src['quality'], sort_keys=True)}`"])
        if 'profiler_diagnostics' in src:
            lines.extend(['  计时器累计诊断（null为未知，各进程取最大值，不能逐行相加）：`'+
                          escape(json.dumps(src['profiler_diagnostics'], ensure_ascii=False, sort_keys=True))+'`'])
    return '\n'.join(lines)+'\n'


def print_summary(events):
    clocks = {clock(r) for r in events}
    if len(clocks) != 1 or None in clocks:
        raise ValueError('summary requires one identified clock')
    for stage, row in grouped_stages(events).items():
        print(stage, json.dumps(row, ensure_ascii=False, allow_nan=False))


def print_sequence(events, seq):
    print(f'\nseq={seq}: recorded rows only; seq reuse does not prove a single request')
    for r in sorted((r for r in events if r.get('seq') == seq), key=lambda d: (d['ts_ms'], d['_line'])):
        print(r['ts_ms'], r.get('clock_id'), r.get('worker_session_id'), r['stage'], r['duration_ms'], 'ms')


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('log', nargs='?', type=Path, default=DEFAULT_LOG)
    p.add_argument('--seq', type=int)
    p.add_argument('--clock-id')
    p.add_argument('--start-ms', type=int)
    p.add_argument('--end-ms', type=int)
    for option in ('tracker-log', 'detection-log', 'json-out', 'markdown-out'):
        p.add_argument('--'+option, type=Path)
    p.add_argument('--performance-log', type=Path, action='append')
    p.add_argument('--overwrite', action='store_true')
    a = p.parse_args(argv)
    try:
        inputs = {v.resolve() for v in (a.log, a.tracker_log, a.detection_log, *(a.performance_log or [])) if v}
        outputs = [v.resolve() for v in (a.json_out, a.markdown_out) if v]
        aliases_input = any(v in inputs or any(v.exists() and i.exists() and v.samefile(i) for i in inputs) for v in outputs)
        aliases_output = any(x.exists() and y.exists() and x.samefile(y) for i, x in enumerate(outputs) for y in outputs[i+1:])
        if len(set(outputs)) != len(outputs) or aliases_input or aliases_output:
            raise ValueError('outputs must be distinct and cannot replace source logs')
        if a.seq is not None and not stamp(a.seq):
            raise ValueError('seq must be a nonnegative integer')
        if not a.overwrite and any(v.exists() for v in outputs):
            raise ValueError('output exists; choose new paths or explicit --overwrite')
        r = build_report(a.log, clock_id=a.clock_id, start_ms=a.start_ms, end_ms=a.end_ms,
                         tracker_log=a.tracker_log, detection_log=a.detection_log, performance_log=a.performance_log)
        text = markdown(r)
        encoded = json.dumps(r, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
        for path, data in ((a.json_out, encoded), (a.markdown_out, text)):
            if path:
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open('w' if a.overwrite else 'x', encoding='utf-8', newline='\n') as stream:
                    stream.write(data)
        print(text)
        if a.seq is not None:
            events, _ = read_events(a.log)
            events = [e for e in events if clock(e) == r['scope']['clock_id'] and
                      (a.start_ms is None or e['ts_ms'] >= a.start_ms) and (a.end_ms is None or e['ts_ms'] <= a.end_ms)]
            print_sequence(events, a.seq)
        return 0
    except (OSError, ValueError) as exc:
        p.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
