"""Portable deterministic replay; does not exercise Linux IPC or inference.

Input JSON: {schema_version: 1, events: [{at_ns, message, image}]}.
Messages use inline v1 payloads. JPEG paths are relative to the manifest.
The synthetic fixture has no human approvals and is not training evidence.
"""
import argparse
import io
import json
from pathlib import Path
import sys
import uuid

from .hardcase.collector import Collector, SECOND
from .hardcase.image import validate_jpeg
from .hardcase.protocol import loads, validate_header, validate_payload
from .hardcase.service import json_bytes
from .hardcase.storage import RunStore


class ReplaySink:
    """Synchronous offline sink; live service uses the bounded writer instead."""
    def __init__(self, store):
        self.store = store

    def submit(self, key, image, reasons, metadata, kind):
        self.store.put(key, image, reasons, metadata, kind=kind)
        return True

    def annotate(self, key, reasons, metadata):
        record = self.store.get(key)
        if record is None:
            return False
        self.store.put(key, (self.store.root / record['image_path']).read_bytes(),
                       reasons, metadata, kind='event')
        return True

    def event(self, event):
        self.store.record_event(event)
        return True


def synthetic_events():
    from PIL import Image, ImageDraw
    run, session, worker = (uuid.uuid4().hex for _ in range(3))
    boot = '00000000-0000-0000-0000-000000000000'
    for i in range(20):
        ns = SECOND + i * 200_000_000
        stream = io.BytesIO()
        image = Image.new('RGB', (128, 96), (20 + i*5, 40, 80))
        ImageDraw.Draw(image).ellipse((40, 20, 65, 55), fill=(230, 80, 110))
        image.save(stream, format='JPEG')
        blob = stream.getvalue()
        path = f'synthetic/frame_{i+1:012d}_{session}.jpg'
        frame = dict(protocol_version=1, kind='FrameReady', run_id=run, boot_id=boot,
            generation=0, decoder_session_id=session, frame_id=i+1,
            producer_mono_ns=ns, metadata_mode='inline', payload=dict(
                width=128, height=96, source_mono_ns=ns, wall_utc_ms=0,
                frame_ts_ms=ns//1_000_000, encode_ms=0, codec='jpeg', original_path=path))
        yield ns, frame, blob
        box = dict(class_id=0, class_name='balloon', confidence=.3 if i==5 else .9,
            bbox_xyxy=[40, 20, 43, 23] if i==4 else [40, 20, 65, 55],
            origin='full', stage='synthetic_post_nms')
        boxes = [] if i in (6, 7, 8) else [box]
        result = dict(frame, kind='DetectionResult', producer_mono_ns=ns+10_000_000,
            worker_session_id=worker, inference_seq=i, request_seq=i, payload=dict(
                ok=True, error=None, image_width=128, image_height=96, source_path=path,
                pin_used=True, request_start_ns=ns, inference_start_ns=ns+1,
                inference_end_ns=ns+10_000_000, backend='synthetic', model_sha256=None,
                config_sha256=None, online_conf=.25, online_iou=.7, class_map={'0':'balloon'},
                detections=boxes, full_detections=boxes, tile_detections=[],
                selected_target=box if boxes else None, detection_stage='synthetic_post_nms',
                timings={}, capture_ts_ms=ns//1_000_000, request_ts_ms=ns//1_000_000,
                raw_result={'synthetic':True}, unavailable_reasons={
                    'model_sha256':'synthetic fixture, no model',
                    'config_sha256':'synthetic fixture, no inference configuration'}))
        yield ns+10_000_000, result, blob


def read_manifest(path):
    path = Path(path).resolve(strict=True)
    if path.stat().st_size > 16*1024*1024:
        raise ValueError('manifest exceeds 16 MiB; split the replay')
    data = loads(path.read_bytes(), 16*1024*1024)
    if (not isinstance(data, dict) or set(data) != {'schema_version', 'events'} or
            data['schema_version'] != 1 or not isinstance(data['events'], list) or
            not 0 < len(data['events']) <= 100000):
        raise ValueError('invalid replay manifest')
    events = []
    for entry in data['events']:
        if not isinstance(entry, dict) or set(entry) != {'at_ns', 'image', 'message'}:
            raise ValueError('invalid replay event')
        relative = Path(entry['image'])
        target = path.parent / relative
        if (relative.is_absolute() or '..' in relative.parts or
                any(p.is_symlink() for p in (target, *target.parents)) or
                not target.resolve(strict=True).is_relative_to(path.parent)):
            raise ValueError('image must be a contained relative path')
        if not 0 < target.stat().st_size <= 8*1024*1024:
            raise ValueError('JPEG size limit')
        events.append((entry['at_ns'], entry['message'], target))
    return events


def replay(events, output, synthetic=False):
    # Validate all envelopes before creating output. Keep file-backed JPEGs lazy.
    events = list(events)
    if not events:
        raise ValueError('empty replay')
    identity = None
    previous = -1
    for now, message, image in events:
        if type(now) is not int or now < previous:
            raise ValueError('arrival clock must be monotonically ordered')
        previous = now
        validate_header(message)
        if message['metadata_mode'] != 'inline':
            raise ValueError('offline replay requires inline payloads')
        validate_payload(message, message['payload'])
        current = tuple(message[k] for k in ('run_id', 'boot_id', 'generation'))
        if identity is not None and current != identity:
            raise ValueError('one run/boot/generation per replay')
        if message['producer_mono_ns'] > now:
            raise ValueError('arrival precedes production')
        identity = current
    output = Path(output).absolute()
    if output.exists() or any(p.is_symlink() for p in (output, *output.parents)):
        raise ValueError('output must be a new directory without symlinks')
    output.mkdir(parents=True, mode=0o700)
    run, boot, generation = identity
    info = dict(run_id=run, boot_id=boot, generation=generation, synthetic=synthetic,
                purpose='offline-software-replay', validation='NOT_HARDWARE_VALIDATION')
    with RunStore.create(output/'run', info, min_free_bytes=0, min_free_ratio=0) as store:
        collector = Collector(run, boot, generation, ReplaySink(store))
        for now, message, image in events:
            blob = image if isinstance(image, bytes) else image.read_bytes()
            validate_jpeg(blob)
            collector.process(message, blob, now)
        collector.tick(previous+4*SECOND)
        report = dict(schema_version=1, synthetic=synthetic, input_messages=len(events),
            run='run', collector=collector.stats(), storage=store.statistics(),
            integrity=store.verify_all(),
            hardware_transport='NOT_RUN', inference='NOT_RUN', human_reviews_created=0)
        if not report['integrity']['ok']:
            raise ValueError('replay store integrity failed')
    (output/'replay_report.json').write_bytes(json_bytes(report))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--synthetic', action='store_true')
    source.add_argument('--manifest', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        events = synthetic_events() if args.synthetic else read_manifest(args.manifest)
        report = replay(events, args.output, synthetic=args.synthetic)
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    except (ValueError, OSError, TypeError, KeyError, RuntimeError) as exc:
        print('replay: '+str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
