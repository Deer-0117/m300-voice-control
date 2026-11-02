"""Opt-in, bounded-memory call timing. Summaries are not physical response times.

One recorder belongs to one thread. Recording uses integer monotonic timestamps;
best-effort disk writes happen at most once per five-second window, plus close.
Sink time is deliberately outside the measured algorithm and reported separately.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import time
import uuid

from balloon_clock import clock_id

BOUNDS_NS = [1000 << i for i in range(21)] + [None]


class PerformanceRecorder:
    def __init__(self, path: Path, source: str, *, enabled=None, clock=None, writer=None):
        self.enabled = (os.environ.get('BALLOON_PROFILE_TIMING') == '1'
                        if enabled is None else bool(enabled))
        self.path = Path(path)
        self.source = source
        self.clock = clock or time.monotonic_ns
        self.writer = writer or self._write
        self.stages = {}
        self.window_id = 0
        self.write_failures = 0
        self.dropped_stage_samples = 0
        self.write_failed_summaries = 0
        self.previous_flush_ns = 0
        self.profile_id = f'{os.getpid()}-{uuid.uuid4().hex}'
        self.clock_id = clock_id() if self.enabled else ''
        self.window_start = self.clock() if self.enabled else 0

    def begin(self):
        return self.clock() if self.enabled else 0

    def record(self, stage: str, start_ns: int, end_ns: int, success=True):
        if not self.enabled:
            return
        # This is a fixed-stage profiler, not an unbounded per-frame dictionary.
        if stage not in self.stages and len(self.stages) >= 16:
            self.dropped_stage_samples += 1
            return
        item = self.stages.setdefault(stage, dict(count=0, success_count=0, sum_ns=0,
            min_ns=None, max_ns=None, invalid_count=0, histogram_counts=[0]*len(BOUNDS_NS)))
        if (type(start_ns) is not int or type(end_ns) is not int or
                start_ns < self.window_start or end_ns < start_ns):
            item['invalid_count'] += 1
            return
        elapsed = end_ns-start_ns
        item['count'] += 1
        item['success_count'] += int(bool(success))
        item['sum_ns'] += elapsed
        item['min_ns'] = elapsed if item['min_ns'] is None else min(item['min_ns'], elapsed)
        item['max_ns'] = elapsed if item['max_ns'] is None else max(item['max_ns'], elapsed)
        bucket = next(i for i, bound in enumerate(BOUNDS_NS) if bound is None or elapsed <= bound)
        item['histogram_counts'][bucket] += 1
        if end_ns-self.window_start >= 5_000_000_000:
            self.flush(end_ns)

    def _write(self, rows):
        with self.path.open('a', encoding='utf-8') as stream:
            for row in rows:
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False,
                                        separators=(',', ':'))+'\n')

    def flush(self, end_ns=None):
        if not self.enabled or not self.stages:
            return
        end_ns = self.clock() if end_ns is None else end_ns
        if type(end_ns) is not int or end_ns < self.window_start:
            return
        rows = []
        for stage, item in self.stages.items():
            cumulative = 0
            p95_bound = None
            threshold = (95*item['count']+99)//100
            if threshold:
                for count, bound in zip(item['histogram_counts'], BOUNDS_NS):
                    cumulative += count
                    if cumulative >= threshold:
                        p95_bound = bound
                        break
            rows.append(dict(schema_version=1, event='performance_summary',
                clock_id=self.clock_id, profile_id=self.profile_id, source=self.source,
                mission_token='', mission_attribution='unknown', window_id=self.window_id,
                window_start_ns=self.window_start, window_end_ns=end_ns,
                prediction_mode=None, no_flight=None, aircraft_state='unknown',
                stage=stage, **item, histogram_upper_bounds_ns=BOUNDS_NS,
                p95_upper_bound_ns=p95_bound, write_failures=self.write_failures,
                dropped_stage_samples=self.dropped_stage_samples,
                write_failed_summaries=self.write_failed_summaries,
                previous_flush_ns=self.previous_flush_ns))
        write_start = self.clock()
        try:
            self.writer(rows)
        except Exception:
            # Optional diagnostics must not turn a successful tracker call into
            # a failed one, or replace its original exception with a sink error.
            self.write_failures += 1
            # A partially completed write is possible: this counts summaries
            # involved in a failed write, not an exact number of lost records.
            self.write_failed_summaries += len(rows)
        finally:
            self.previous_flush_ns = max(0, self.clock()-write_start)
            self.stages.clear()
            self.window_id += 1
            self.window_start = end_ns

    def close(self):
        self.flush()
