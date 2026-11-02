"""Deterministic bounded capture decisions. No persistent I/O on this path.

The sink is a nonblocking queue interface; only its writer owns the SQLite store.
Times are CLOCK_MONOTONIC nanoseconds from the same boot, never wall time.
"""
from collections import Counter, OrderedDict
from dataclasses import dataclass
import hashlib
import json
import math
import uuid

SECOND = 1_000_000_000


def _size(value):
    return len(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class Limits:
    cache_frames: int = 96
    cache_bytes: int = 64 * 1024 * 1024
    pending_results: int = 128
    pending_events: int = 128
    identities: int = 6000
    sessions: int = 8
    metadata_bytes: int = 16 * 1024 * 1024
    max_events: int = 10000
    max_detections: int = 24000

    def __post_init__(self):
        if any(type(v) is not int or v <= 0 for v in vars(self).values()):
            raise ValueError('all resource limits must be positive integers')


class Collector:
    def __init__(self, run_id, boot_id, generation, sink, limits=None):
        self.run_id, self.boot_id, self.generation = run_id, boot_id, generation
        self.sink, self.limits = sink, limits or Limits()
        self.enabled = True
        self.cache = OrderedDict()
        self.seen = OrderedDict()
        self.seen_bytes = 0
        self.saved = OrderedDict()
        self.sessions = OrderedDict()
        # Admission and forgotten-frame boundaries have the same per-run
        # lifetime as exact identities, including across reconfiguration.
        self.identity_scopes = {}
        self.workers = OrderedDict()
        self.pending = OrderedDict()
        self.events = OrderedDict()
        self.counts = Counter()

    def _remember(self, table, key, value, limit):
        if table is self.seen:
            if key in table:
                self.seen_bytes -= _size([key, table[key]])
            self.seen_bytes += _size([key, value])
        table[key] = value
        table.move_to_end(key)
        while len(table) > limit:
            if table is self.seen:
                self._forget_identity()
            else:
                table.popitem(last=False)
        if table is self.seen:
            # Compact identity history outlives JPEGs, but shares the metadata
            # budget. Incremental accounting avoids serializing all identities
            # for every video frame.
            metadata = self._memory()[1]
            while table and metadata > self.limits.metadata_bytes:
                metadata -= self._forget_identity()

    def _forget_identity(self):
        key, value = self.seen.popitem(last=False)
        released = _size([key, value])
        self.seen_bytes -= released
        if key[0] == 'frame' and value[1] is not None:
            scope = key[1][:3]
            if scope in self.identity_scopes:
                # Decoder frame IDs increase within a session. Once exact
                # history is lost, reject unknown older frames/results instead
                # of treating an observed frame as a fresh candidate.
                previous = self.identity_scopes[scope]
                boundary = max(previous, key[1][-1])
                self.identity_scopes[scope] = boundary
                released -= _size([scope, boundary]) - _size([scope, previous])
        return released

    def _memory(self):
        return (sum(len(v['image']) for v in self.cache.values()),
                sum(_size(v['metadata']) for v in self.cache.values()) +
                sum(_size(v['message']) for v in self.pending.values()) +
                sum(_size(v) for v in self.events.values()) + self.seen_bytes +
                sum(_size([k, v]) for k, v in self.identity_scopes.items()))

    def stats(self):
        image_bytes, metadata_bytes = self._memory()
        return dict(self.counts, cache_frames=len(self.cache), cache_bytes=image_bytes,
                    metadata_bytes=metadata_bytes, pending_results=len(self.pending),
                    pending_events=len(self.events), identity_count=len(self.seen),
                    active_sessions=len(self.sessions), active_workers=len(self.workers))

    def _room(self, image_bytes=0, metadata_bytes=0, new_frame=False):
        if image_bytes > self.limits.cache_bytes or metadata_bytes > self.limits.metadata_bytes:
            return False
        while self.cache:
            images, metadata = self._memory()
            if (images + image_bytes <= self.limits.cache_bytes and
                    metadata + metadata_bytes <= self.limits.metadata_bytes and
                    len(self.cache) + int(new_frame) <= self.limits.cache_frames):
                return True
            key, _ = self.cache.popitem(last=False)
            # Pending observations refer to the cache, never retain a hidden image.
            for dkey, value in list(self.pending.items()):
                if value['key'] == key:
                    del self.pending[dkey]
                    self.counts['image_missing'] += 1
            self.counts['cache_evicted'] += 1
        # Identity history is reclaimable too; otherwise a full history can
        # permanently starve admission even after every JPEG has been evicted.
        metadata = self._memory()[1]
        while self.seen and metadata + metadata_bytes > self.limits.metadata_bytes:
            metadata -= self._forget_identity()
        return (image_bytes <= self.limits.cache_bytes and
                metadata + metadata_bytes <= self.limits.metadata_bytes)

    def _patch(self, key, reasons, metadata):
        if key not in self.saved:
            return
        if not self.sink.annotate(key, reasons, metadata):
            self.counts['queue_rejected'] += 1

    def _save(self, key, reasons, kind, extra=None):
        entry = self.cache.get(key)
        if entry is None:
            self.counts['image_missing'] += 1
            return False
        metadata = dict(entry['metadata'])
        if extra:
            metadata.update(extra)
        # Queue acceptance does not prove a durable write. Resubmit the original
        # bytes; the store can retry pre-admission failures. Durably terminal
        # admissions remain terminal and are counted by the writer.
        accepted = self.sink.submit(key, entry['image'], reasons, metadata, kind)
        if accepted:
            self._remember(self.saved, key, True, self.limits.identities)
            self.counts['queue_enqueued'] += 1
        else:
            self.counts['queue_rejected'] += 1
        return accepted

    def process(self, message, image, now_ns):
        self.counts['received'] += 1
        if (not self.enabled or message.get('run_id') != self.run_id or
                message.get('boot_id') != self.boot_id or
                message.get('generation') != self.generation):
            self.counts['stale_run'] += 1
            return
        if (type(now_ns) is not int or now_ns < 0 or
                message.get('kind') not in ('FrameReady', 'DetectionResult')):
            self.counts['invalid_message'] += 1
            return
        # Deadlines precede ingestion. A delayed event loop must not extend the
        # association/context window simply because a new message woke it up.
        self.tick(now_ns)
        try:
            from .image import validate_jpeg
            payload = message['payload']
            is_frame = message['kind'] == 'FrameReady'
            width = payload['width'] if is_frame else payload['image_width']
            height = payload['height'] if is_frame else payload['image_height']
            info = validate_jpeg(image, width, height)
            key = (message['run_id'], message['boot_id'], message['decoder_session_id'], message['frame_id'])
            if is_frame:
                source = payload['source_mono_ns']
                if type(source) is not int or not 0 <= source <= now_ns:
                    raise ValueError('invalid source clock')
            if _size(message) > 1024*1024:
                raise ValueError('metadata too large')
        except (KeyError, ValueError, TypeError, OverflowError):
            self.counts['invalid_message'] += 1
            return
        checksum = info['sha256']
        scope = key[:3]
        if scope not in self.identity_scopes:
            if len(self.identity_scopes) >= self.limits.sessions:
                self.counts['session_limit'] += 1
                return
            if not self._room(metadata_bytes=_size([scope, -1])):
                self.counts['resource_rejected'] += 1
                return
            self.identity_scopes[scope] = -1
        old = self.cache.get(key)
        known = self.seen.get(('frame', key))
        known_source = known[1] if known else (old['source'] if old else None)
        known_arrival = known[2] if known else (old['frame_arrival'] if old else None)
        if ((old and old['sha'] != checksum) or (known and known[0] != checksum)
                or (is_frame and known_source is not None and known_source != source)):
            self.counts['identity_conflict'] += 1
            return
        if known_source is None and key[-1] <= self.identity_scopes[scope]:
            if is_frame:
                self.counts['expired_identity'] += 1
                return
            # Keep normal result validation/deduplication and metadata patches,
            # but retain no JPEG or pending fallback for forgotten identities.
            self._result(key, {'source': None, 'expired_identity': True,
                              'metadata': {'detections': []}}, message, now_ns)
            return
        if old is None:
            metadata = {'source_mono_ns': known_source, 'detections': [], 'events': []}
            if not self._room(len(image), _size(metadata) + _size(message), True):
                self.counts['resource_rejected'] += 1
                return
            old = {'image': image, 'sha': checksum, 'source': known_source, 'arrival': now_ns,
                   'frame_arrival': known_arrival,
                   'metadata': metadata}
            self.cache[key] = old
        # Cache eviction must not turn a known old frame into an unknown-time
        # result-only candidate. Keep its original clock with the identity.
        identity = (checksum, source if is_frame else known_source,
                    (known_arrival if known_arrival is not None else now_ns)
                    if is_frame else known_arrival)
        if is_frame:
            self._frame(key, old, message, now_ns)
        else:
            self._result(key, old, message, now_ns)
        self._remember(self.seen, ('frame', key), identity, self.limits.identities)
        self.tick(now_ns)

    def _frame(self, key, entry, message, now):
        source = message['payload']['source_mono_ns']
        if entry['source'] is not None:
            self.counts['duplicate_frame' if entry['source'] == source else 'identity_conflict'] += 1
            return
        # The frame establishes source time even if its full audit metadata
        # cannot fit. Retire unknown-time fallbacks before admission and count
        # only the metadata that will remain after association.
        waiting = []
        for dkey, pending in list(self.pending.items()):
            if pending['key'] == key:
                waiting.append(pending['message'])
                del self.pending[dkey]
        growth = (_size(dict(entry['metadata'], frame=message, source_mono_ns=source))
                  - _size(entry['metadata']))
        if self._memory()[1] + growth > self.limits.metadata_bytes:
            self.counts['metadata_rejected'] += 1
            # process() retains the compact source identity. Discard the
            # unknown-time cache entry so no later result can reuse it.
            self.cache.pop(key, None)
            return
        entry['source'] = source
        entry['frame_arrival'] = now
        entry['metadata'].update(frame=message, source_mono_ns=source)
        was_saved = key in self.saved
        scope = key[:3]
        state = self.sessions.get(scope)
        if state is None:
            if len(self.sessions) >= self.limits.sessions:
                # Refuse a new session rather than forget a cooldown and restart it.
                self.counts['session_limit'] += 1
                return
            state = {'origin': source, 'latest': source, 'bucket': -1, 'cooldowns': {}}
            self.sessions[scope] = state
        if source < state['latest']:
            self.counts['out_of_order_frame'] += 1
        else:
            bucket = (source - state['origin']) // SECOND
            if bucket != state['bucket']:
                self.counts['uniform_buckets'] += 1
                self._save(key, ['uniform'], 'uniform')
            state.update(latest=source, bucket=bucket)
        if was_saved:
            self._patch(key, [], {'frame': message, 'source_mono_ns': source})
        for pending_message in waiting:
            self._associated(key, entry, pending_message, now)

    def _result(self, key, entry, message, now):
        payload = message['payload']
        if (type(payload.get('ok')) is not bool or
                (payload['ok'] and payload.get('error') is not None) or
                (not payload['ok'] and not payload.get('error'))):
            self.counts['invalid_result'] += 1
            return
        try:
            worker = message['worker_session_id']
            q = message['inference_seq']
            if type(q) is not int or q < 0:
                raise ValueError('invalid sequence')
            dkey = (self.run_id, self.boot_id, worker, q)
            digest = _digest(message)
        except (KeyError, ValueError, TypeError):
            self.counts['invalid_result'] += 1
            return
        prior = self.seen.get(('detection', dkey))
        if prior:
            self.counts['duplicate_result' if prior == digest else 'identity_conflict'] += 1
            return
        if self.counts['detection_records'] >= self.limits.max_detections:
            self.counts['detection_limit'] += 1
            return
        if self._memory()[1] + 2*_size(message) > self.limits.metadata_bytes:
            self.counts['metadata_rejected'] += 1
            return
        if len(entry['metadata']['detections']) >= 8:
            self.counts['sample_detection_limit'] += 1
            return
        self._remember(self.seen, ('detection', dkey), digest, self.limits.identities)
        entry['metadata']['detections'].append(message)
        self.counts['detection_records'] += 1
        self._patch(key, [], {'detections': [message]})
        if entry.get('expired_identity'):
            self.counts['expired_identity'] += 1
        elif entry['source'] is not None:
            self._associated(key, entry, message, now)
        elif len(self.pending) >= self.limits.pending_results:
            self.counts['pending_result_limit'] += 1
        else:
            self.pending[dkey] = {'key': key, 'message': message, 'deadline': now+250000000}

    def _rules(self, payload):
        if not payload['ok']:
            return []
        width, height = payload['image_width'], payload['image_height']
        reasons = set()
        for box in payload.get('detections', []):
            try:
                if box.get('valid', True) is False or box.get('invalid') or box.get('origin') == 'tracker':
                    continue
                confidence = box['confidence']
                x1, y1, x2, y2 = box['bbox_xyxy']
                if (not all(type(x) in (int, float) and math.isfinite(x)
                            for x in (confidence, x1, y1, x2, y2)) or
                        not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height)):
                    continue
                if .25 <= confidence <= .45:
                    reasons.add('low_confidence')
                if (x2-x1)*(y2-y1)/(width*height) <= .003:
                    reasons.add('small_target')
            except (KeyError, TypeError, ValueError, ZeroDivisionError):
                continue
        return sorted(reasons)

    def _associated(self, key, entry, message, now):
        source = entry['source']
        if now > source + 2*SECOND:
            self.counts['late_result'] += 1
            return
        worker = (message['worker_session_id'], key[2])
        state = self.workers.get(worker)
        if state is None:
            if len(self.workers) >= self.limits.sessions:
                self.counts['worker_limit'] += 1
                return
            state = {'q': -1, 'source': -1, 'positive': None, 'empty': 0, 'result_cooldowns': {}}
            self.workers[worker] = state
        q = message['inference_seq']
        if q <= state['q'] or source < state['source']:
            self.counts['out_of_order_result'] += 1
            return
        if q != state['q']+1:
            state['empty'] = 0
        state.update(q=q, source=source)
        if state['positive'] is not None and source > state['positive'] + 2*SECOND:
            state.update(positive=None, empty=0)
        p = message['payload']
        reasons = self._rules(p)
        if not p['ok']:
            state['empty'] = 0
        elif any(box.get('valid', True) and box.get('origin') != 'tracker'
                 for box in p.get('detections', [])):
            state.update(positive=source, empty=0)
        elif p.get('detections'):
            # Malformed/tracker boxes are neither a detector positive nor a
            # successful empty detector result.
            state['empty'] = 0
        elif state['positive'] is not None:
            state['empty'] += 1
            if state['empty'] >= 3:
                reasons.append('disappearance')
                state.update(positive=None, empty=0)
        for reason in reasons:
            self._event(key, source, reason, now)

    def _event(self, key, source, reason, now):
        state = self.sessions.get(key[:3])
        if state is None:
            return
        last = state['cooldowns'].get(reason)
        if last is not None and source-last < 10*SECOND:
            self.counts['cooldown'] += 1
            return
        if (len(self.events) >= self.limits.pending_events or
                self.counts['events_created'] >= self.limits.max_events):
            self.counts['event_limit'] += 1
            return
        eid = uuid.uuid4().hex
        event = {'id': eid, 'center': list(key), 'source_mono_ns': source,
                 'reason': reason, 'deadline': source+3*SECOND, 'event_revision': 0}
        if self._memory()[1] + _size(event) > self.limits.metadata_bytes:
            self.counts['metadata_rejected'] += 1
            return
        self.events[eid] = event
        state['cooldowns'][reason] = source
        self.counts['events_created'] += 1
        self._save(key, [reason], 'event', {'events': [event]})

    def _result_only(self, pending):
        key, message = pending['key'], pending['message']
        if key not in self.cache:
            self.counts['image_missing'] += 1
            return
        worker = (message['worker_session_id'], key[2])
        state = self.workers.get(worker)
        if state is None:
            if len(self.workers) >= self.limits.sessions:
                self.counts['worker_limit'] += 1
                return
            state = {'q': -1, 'source': -1, 'positive': None, 'empty': 0, 'result_cooldowns': {}}
            self.workers[worker] = state
        ns = message['payload'].get('inference_end_ns')
        if type(ns) is not int or ns < 0:
            self.counts['invalid_result'] += 1
            return
        q = message['inference_seq']
        if q <= state['q']:
            self.counts['out_of_order_result'] += 1
            return
        state.update(q=q, empty=0)
        reasons = []
        for reason in self._rules(message['payload']):
            last = state['result_cooldowns'].get(reason)
            if last is None or ns-last >= 10*SECOND:
                reasons.append(reason)
                state['result_cooldowns'][reason] = ns
            else:
                self.counts['cooldown'] += 1
        if reasons:
            self._save(key, reasons+['result_only'], 'event')
        self.counts['result_only'] += 1

    def _settle(self, event, status='settled'):
        center = tuple(event['center'])
        source = event['source_mono_ns']
        candidates = [(v['source'], k[-1], k) for k, v in self.cache.items()
                      if k[:3] == center[:3] and v['source'] is not None and
                      v['frame_arrival'] is not None and v['frame_arrival'] <= event['deadline']]
        candidates.sort()
        before = [k for ns, _, k in candidates if source-SECOND <= ns < source][:2]
        after = [k for ns, _, k in candidates if source < ns <= source+SECOND][:2]
        keys = [center]+before+after
        final = dict(event, event_revision=1, status=status, missing_before=2-len(before),
                     missing_after=2-len(after), frame_keys=[list(k) for k in keys])
        saved = []
        for key in keys:
            if key not in self.cache:
                self._patch(key, [event['reason']], {'events': [final]})
            elif self._save(key, [event['reason']], 'event', {'events': [final]}):
                saved.append(list(key))
        final['enqueued_frame_keys'] = saved
        final['event_revision'] = 2
        if not self.sink.event(final):
            self.counts['queue_rejected'] += 1

    def tick(self, now_ns):
        for dkey, pending in list(self.pending.items()):
            if now_ns >= pending['deadline']:
                del self.pending[dkey]
                self._result_only(pending)
        for eid, event in list(self.events.items()):
            if now_ns >= event['deadline']:
                del self.events[eid]
                self._settle(event)
        for key, entry in list(self.cache.items()):
            stamp = entry['source'] if entry['source'] is not None else entry['arrival']
            if now_ns > stamp+4*SECOND:
                del self.cache[key]
                self.counts['cache_expired'] += 1
        self._room()

    def reconfigure(self, generation, enabled, now_ns):
        if type(generation) is not int or generation <= self.generation:
            raise ValueError('generation must increase')
        for event in self.events.values():
            self._settle(event, 'generation_change')
        self.counts['generation_pending_dropped'] += len(self.pending)
        self.events.clear()
        self.pending.clear()
        self.cache.clear()
        self.sessions.clear()
        self.workers.clear()
        self.generation, self.enabled = generation, bool(enabled)
