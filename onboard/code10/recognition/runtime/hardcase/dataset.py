"""Offline candidate review and immutable, human-only dataset export.

No detector output is ground truth. Callers must pin review IDs and a class map.
All groups are computed over the complete supplied runs before selecting samples.
"""
from __future__ import annotations

from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import random
import stat

from PIL import Image

TOOL_VERSION = 'hardcase-dataset/1'
REVIEW_FIELDS = {'review_id', 'sample_key', 'image_sha256', 'annotator_id',
                 'reviewed_utc', 'tool_version', 'class_map_version', 'status',
                 'confirmed_empty', 'objects', 'comment', 'supersedes_review_id'}


def canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode('utf-8')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_json(path):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON key: ' + key)
            result[key] = value
        return result
    def invalid_constant(value):
        raise ValueError('non-finite JSON value: ' + value)
    return json.loads(Path(path).read_text(encoding='utf-8'), object_pairs_hook=pairs,
                      parse_constant=invalid_constant)


def _no_symlinks(path):
    path = Path(path).absolute()
    for part in [path, *path.parents]:
        try:
            info = part.lstat()
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
            raise ValueError('symlink/junction forbidden: ' + str(part))
    return path


def safe_source(root, relative):
    if not isinstance(relative, str) or not relative or '\\' in relative:
        raise ValueError('source must be a portable relative path')
    posix = PurePosixPath(relative)
    if posix.is_absolute() or PureWindowsPath(relative).drive or '..' in posix.parts:
        raise ValueError('source path escapes run')
    root = _no_symlinks(root).resolve()
    path = _no_symlinks(root / relative)
    if not path.resolve().is_relative_to(root) or not path.is_file():
        raise ValueError('source missing or outside run: ' + relative)
    return path


def new_output(path, sources=()):
    output = _no_symlinks(path)
    resolved = output.resolve()
    if output.exists():
        raise ValueError('output must be a new directory')
    for source in sources:
        source = _no_symlinks(source).resolve()
        if resolved.is_relative_to(source) or source.is_relative_to(resolved):
            raise ValueError('output overlaps source')
    if not output.parent.is_dir():
        raise ValueError('output parent must already exist')
    return output


def write_report(output, report, sources=(), dry_run=False, name='report.json'):
    destination = new_output(output, sources)
    if not dry_run:
        destination.mkdir()
        (destination / name).write_bytes(canonical(report) + b'\n')
    return report


def _class_map(class_map):
    if not isinstance(class_map, dict) or not isinstance(class_map.get('version'), str) or not class_map['version']:
        raise ValueError('class map needs a frozen version')
    names = class_map.get('names')
    if not isinstance(names, dict) or not names:
        raise ValueError('class map names must be a nonempty id/name mapping')
    if set(names) != {str(i) for i in range(len(names))}:
        raise ValueError('YOLO class IDs must be contiguous from zero')
    if any(not isinstance(v, str) or not v.strip() for v in names.values()):
        raise ValueError('invalid class name')
    return names


def _verify_sample(store, sample):
    for key in ('image_path', 'sidecar_path'):
        path = safe_source(store.root, sample[key])
        digest = sample['image_sha256' if key == 'image_path' else 'sidecar_sha256']
        if sha256(path.read_bytes()) != digest:
            raise ValueError('source digest mismatch: ' + sample['sample_id'])
    image = safe_source(store.root, sample['image_path'])
    if image.stat().st_size != sample['image_bytes']:
        raise ValueError('image size mismatch')
    with Image.open(image) as decoded:
        if decoded.format != 'JPEG' or decoded.size != (sample['width'], sample['height']):
            raise ValueError('image dimensions/format mismatch')
        decoded.load()


def validate_run(store):
    report = store.verify_all()
    if not report.get('ok'):
        raise ValueError('run integrity failed: ' + str(report.get('errors')))
    samples = store.list_samples()
    seen = set()
    for sample in samples:
        _verify_sample(store, sample)
        if sample['sample_id'] in seen:
            raise ValueError('duplicate sample identity')
        seen.add(sample['sample_id'])
    _reviews(store)
    return {'tool_version': TOOL_VERSION, 'ok': True, 'samples': len(samples),
            'integrity': report, 'labels_inferred': 0}


def _collect(stores):
    all_samples, owners = [], {}
    for store in stores:
        validate_run(store)
        for sample in store.list_samples():
            sid = sample['sample_id']
            if sid in owners:
                raise ValueError('duplicate sample across runs')
            owners[sid] = store
            all_samples.append(sample)
    return sorted(all_samples, key=lambda s: s['sample_id']), owners


def export_review_manifest(stores, output, dry_run=False):
    destination = new_output(output, [s.root for s in stores])
    samples, _ = _collect(stores)
    result = {'schema_version': 1, 'tool_version': TOOL_VERSION,
              'run_provenance': {str(s.run_info['run_id']): s.run_info for s in stores}, 'samples': [
        {'sample_id': s['sample_id'], 'sample_key': s['key'], 'image_path': s['image_path'],
         'image_sha256': s['image_sha256'], 'width': s['width'], 'height': s['height'],
         'run_id': s['key'][0], 'metadata_revision': s['revision'],
         'metadata_sha256': s['sidecar_sha256'], 'reasons': s['reasons'],
         'detections': s['metadata'].get('detections', []),
         'events': s['metadata'].get('events', []), 'integrity': 'verified',
         'status': 'pending', 'confirmed_empty': False, 'objects': []} for s in samples]}
    if not dry_run:
        destination.mkdir()
        (destination / 'review_manifest.json').write_bytes(canonical(result) + b'\n')
    return result


def _validate_review_envelope(wrapper, require_binding=False):
    if not isinstance(wrapper, dict) or not isinstance(wrapper.get('review'), dict):
        raise ValueError('invalid review envelope')
    if set(wrapper) == {'review', 'sha256'}:
        if sha256(canonical(wrapper['review'])) != wrapper['sha256']:
            raise ValueError('review hash mismatch')
        if require_binding:
            raise ValueError('legacy unbound review requires an explicit superseding human review')
    elif set(wrapper) == {'review', 'class_map', 'class_map_sha256', 'sha256'}:
        _class_map(wrapper['class_map'])
        if sha256(canonical(wrapper['class_map'])) != wrapper['class_map_sha256']:
            raise ValueError('review class map digest mismatch')
        payload = {key: value for key, value in wrapper.items() if key != 'sha256'}
        if sha256(canonical(payload)) != wrapper['sha256']:
            raise ValueError('review envelope hash mismatch')
        if wrapper['review'].get('class_map_version') != wrapper['class_map']['version']:
            raise ValueError('review class map binding version mismatch')
    else:
        raise ValueError('invalid review envelope')
    return wrapper['review']


def _reviews(store, *, envelopes=False):
    records = {}
    wrappers = {}
    for review_id, sample_id, text in store.db.execute('SELECT review_id,sample_id,record_json FROM human_reviews'):
        wrapper = json.loads(text)
        review = _validate_review_envelope(wrapper)
        if review.get('review_id') != review_id:
            raise ValueError('review identity mismatch')
        sample = store.get(review.get('sample_key', []))
        if sample is None or sample['sample_id'] != sample_id or sample['image_sha256'] != review.get('image_sha256'):
            raise ValueError('review/sample binding mismatch')
        if 'class_map' in wrapper:
            _validate_review(review, sample, wrapper['class_map'])
        records[review_id] = review
        wrappers[review_id] = wrapper
    # Validate the whole version graph, including superseded records.
    predecessors, roots = set(), set()
    for review in records.values():
        prior = review.get('supersedes_review_id')
        key = tuple(review['sample_key'])
        if prior is None:
            if key in roots:
                raise ValueError('multiple review roots for one sample')
            roots.add(key)
        else:
            if prior not in records or prior in predecessors or records[prior]['sample_key'] != review['sample_key']:
                raise ValueError('broken or branched review chain')
            predecessors.add(prior)
        visited = set()
        current = review
        while current.get('supersedes_review_id') is not None:
            ident = current['review_id']
            if ident in visited:
                raise ValueError('cyclic review chain')
            visited.add(ident)
            current = records.get(current['supersedes_review_id'])
            if current is None:
                raise ValueError('missing review predecessor')
    return wrappers if envelopes else records


def _validate_review(review, sample, class_map):
    names = _class_map(class_map)
    if not isinstance(review, dict) or set(review) != REVIEW_FIELDS:
        raise ValueError('review must contain exactly the documented fields')
    for name in ('review_id', 'annotator_id', 'reviewed_utc', 'tool_version', 'class_map_version'):
        if not isinstance(review[name], str) or not review[name].strip():
            raise ValueError('missing human review field: ' + name)
    try:
        when = datetime.fromisoformat(review['reviewed_utc'].replace('Z', '+00:00'))
        if when.utcoffset() is None or when.utcoffset().total_seconds() != 0:
            raise ValueError('reviewed_utc must be UTC')
    except (TypeError, ValueError) as exc:
        raise ValueError('invalid reviewed_utc') from exc
    if review['class_map_version'] != class_map['version']:
        raise ValueError('class map version mismatch')
    if review['sample_key'] != sample['key'] or review['image_sha256'] != sample['image_sha256']:
        raise ValueError('review does not match original image')
    if review['status'] not in ('approved', 'rejected') or type(review['confirmed_empty']) is not bool:
        raise ValueError('explicit human approval/rejection required')
    if not isinstance(review['objects'], list) or not isinstance(review['comment'], str):
        raise ValueError('objects/comment have invalid type')
    prior = review['supersedes_review_id']
    if prior is not None and (not isinstance(prior, str) or not prior):
        raise ValueError('invalid review predecessor')
    if review['objects'] and review['confirmed_empty']:
        raise ValueError('objects conflict with confirmed_empty')
    if review['status'] == 'approved' and not review['objects'] and not review['confirmed_empty']:
        raise ValueError('empty label needs explicit human confirmation')
    seen_objects = set()
    for obj in review['objects']:
        if not isinstance(obj, dict) or set(obj) != {'class_id', 'bbox_xyxy'}:
            raise ValueError('object requires class_id and original bbox_xyxy')
        if type(obj['class_id']) is not int or str(obj['class_id']) not in names:
            raise ValueError('invalid class ID')
        box = obj['bbox_xyxy']
        if not isinstance(box, list) or len(box) != 4 or any(type(x) not in (int, float) or not math.isfinite(x) for x in box):
            raise ValueError('box requires four finite original coordinates')
        x1, y1, x2, y2 = box
        if not (0 <= x1 < x2 <= sample['width'] and 0 <= y1 < y2 <= sample['height']):
            raise ValueError('box outside original image or zero area; clipping is forbidden')
        identity = (obj['class_id'], *box)
        if identity in seen_objects:
            raise ValueError('duplicate annotation object')
        seen_objects.add(identity)
    canonical(review)


def import_human_reviews(store, reviews, class_map, dry_run=False):
    validate_run(store)
    _class_map(class_map)
    if not isinstance(reviews, list) or not reviews:
        raise ValueError('nonempty review batch required')
    envelopes = _reviews(store, envelopes=True)
    for envelope in envelopes.values():
        prior_map = envelope.get('class_map')
        if prior_map is not None and prior_map['version'] == class_map['version'] and canonical(prior_map) != canonical(class_map):
            raise ValueError('class map contents changed without a new version')
    history = {rid: envelope['review'] for rid, envelope in envelopes.items()}
    staged, seen = [], set()
    for review in reviews:
        if not isinstance(review, dict):
            raise ValueError('review must be an object')
        sample = store.get(review.get('sample_key', []))
        if sample is None:
            raise ValueError('unknown review sample')
        _validate_review(review, sample, class_map)
        rid = review['review_id']
        if rid in seen or rid in history:
            raise ValueError('review IDs are immutable and cannot be reimported')
        seen.add(rid)
        prior = review['supersedes_review_id']
        same = [r for r in history.values() if r['sample_key'] == review['sample_key']]
        used = {r['supersedes_review_id'] for r in same}
        if same:
            if prior not in history or history[prior]['sample_key'] != review['sample_key'] or prior in used:
                raise ValueError('new revision must supersede the current leaf')
        elif prior is not None:
            raise ValueError('first review must not supersede unknown revision')
        history[rid] = review
        payload = {'review': review, 'class_map': class_map,
                   'class_map_sha256': sha256(canonical(class_map))}
        staged.append((rid, sample['sample_id'], canonical(dict(payload, sha256=sha256(canonical(payload)))).decode('utf-8')))
    if not dry_run:
        with store.db:
            store.db.executemany('INSERT INTO human_reviews(review_id,sample_id,record_json) VALUES(?,?,?)', staged)
    return {'tool_version': TOOL_VERSION, 'would_import': len(staged),
            'imported': 0 if dry_run else len(staged), 'review_ids': [x[0] for x in staged]}


def group_samples(samples, manual_groups=None, dhashes=None, dhash_threshold=0):
    """Connected components: run, event, scene/target, image, manual/near-dup."""
    ids = [s['sample_id'] for s in samples]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate sample')
    parent = {sid: sid for sid in ids}
    def find(sid):
        while parent[sid] != sid:
            parent[sid] = parent[parent[sid]]
            sid = parent[sid]
        return sid
    def union(a, b):
        a, b = find(a), find(b)
        parent[max(a, b)] = min(a, b)
    tokens = {}
    for sample in samples:
        sid, meta = sample['sample_id'], sample['metadata']
        links = [('run', sample['key'][0]), ('image', sample['image_sha256'])]
        links.extend(('event', e.get('id', e.get('event_id'))) for e in meta.get('events', []))
        for field in ('scene_id', 'target_segment_id', 'near_duplicate_group'):
            if meta.get(field) is not None:
                links.append((field, str(meta[field])))
        if manual_groups and sid in manual_groups:
            links.append(('manual', str(manual_groups[sid])))
        for token in links:
            if token[1] is None:
                continue
            if token in tokens:
                union(sid, tokens[token])
            else:
                tokens[token] = sid
    if type(dhash_threshold) is not int or not 0 <= dhash_threshold <= 64:
        raise ValueError('dHash threshold must be between 0 and 64')
    if dhashes:
        if set(dhashes) != set(ids):
            raise ValueError('dHash values must cover entire pool')
        ordered = sorted(ids)
        for index, first in enumerate(ordered):
            for second in ordered[index + 1:]:
                if (dhashes[first] ^ dhashes[second]).bit_count() <= dhash_threshold:
                    union(first, second)
    return {sid: find(sid) for sid in sorted(ids)}


def split_groups(groups, assignments=None, frozen_test_ids=(), seed=0, existing=None):
    if not groups:
        raise ValueError('cannot split an empty pool')
    allocations = {}
    for mapping in (existing or {}, assignments or {}, {sid: 'test' for sid in frozen_test_ids}):
        for sid, split in mapping.items():
            if sid not in groups or split not in ('train', 'val', 'test'):
                raise ValueError('invalid split sample or split name')
            group = groups[sid]
            if group in allocations and allocations[group] != split:
                raise ValueError('connected group conflicts across splits')
            allocations[group] = split
    remaining = sorted(set(groups.values()) - allocations.keys())
    random.Random(seed).shuffle(remaining)
    if not assignments and not existing and len(set(groups.values())) < 3:
        raise ValueError('insufficient independent groups; supply explicit single-split fixture assignment')
    # Ensure all three partitions when enough independent groups are available.
    for split in ('test', 'val', 'train'):
        if split not in allocations.values() and remaining:
            allocations[remaining.pop()] = split
    for group in remaining:
        allocations[group] = 'train'
    if 'train' not in allocations.values():
        raise ValueError('zero training groups')
    return {sid: allocations[group] for sid, group in groups.items()}


def _dhash(path):
    with Image.open(path) as image:
        pixels = list(image.convert('L').resize((9, 8)).getdata())
    value = 0
    for row in range(8):
        for col in range(8):
            value = (value << 1) | int(pixels[row * 9 + col] > pixels[row * 9 + col + 1])
    return value


def _grouping_config(manual_groups, dhash_threshold, existing_split):
    if existing_split is not None:
        if not isinstance(existing_split, dict) or not {'groups', 'assignments'} <= set(existing_split):
            raise ValueError('existing split must be a full manifest with groups and assignments')
        frozen_manual = existing_split.get('manual_groups', {})
        frozen_threshold = existing_split.get('dhash_threshold')
        if manual_groups is not None and canonical(manual_groups) != canonical(frozen_manual):
            raise ValueError('manual grouping configuration conflicts with frozen split')
        if dhash_threshold is not None and canonical(dhash_threshold) != canonical(frozen_threshold):
            raise ValueError('dHash configuration conflicts with frozen split')
        manual_groups, dhash_threshold = frozen_manual, frozen_threshold
    if manual_groups is None:
        manual_groups = {}
    if not isinstance(manual_groups, dict):
        raise ValueError('manual groups must map sample IDs to groups')
    if dhash_threshold is not None and (type(dhash_threshold) is not int or not 0 <= dhash_threshold <= 64):
        raise ValueError('dHash threshold must be between 0 and 64')
    return manual_groups, dhash_threshold


def _pool_dhashes(samples, owners, dhash_threshold):
    if dhash_threshold is None:
        return None
    return {s['sample_id']: _dhash(safe_source(owners[s['sample_id']].root, s['image_path'])) for s in samples}


def _group_pool(samples, manual_groups, dhashes, dhash_threshold, existing_split):
    manual_groups, dhash_threshold = _grouping_config(manual_groups, dhash_threshold, existing_split)
    if set(manual_groups) - {s['sample_id'] for s in samples}:
        raise ValueError('manual grouping contains samples outside the complete pool')
    if dhashes is not None and dhash_threshold is None:
        if existing_split is not None:
            raise ValueError('dHash grouping conflicts with frozen split configuration')
        dhash_threshold = 0
    if dhash_threshold is not None and dhashes is None:
        raise ValueError('dHash grouping requires hashes for the entire pool')
    computed = group_samples(samples, manual_groups, dhashes, dhash_threshold or 0)
    pool_hash = sha256(canonical([{key: s.get(key) for key in
        ('sample_id', 'key', 'image_sha256', 'revision', 'sidecar_sha256')}
        for s in sorted(samples, key=lambda s: s['sample_id'])]))
    groups, assignments = computed, {}
    if existing_split is not None:
        groups, assignments = existing_split['groups'], existing_split['assignments']
        if not isinstance(groups, dict) or not isinstance(assignments, dict) or set(groups) != set(computed) or set(assignments) != set(computed):
            raise ValueError('frozen split must cover exactly the complete sample pool')
        if 'pool_sha256' in existing_split and existing_split['pool_sha256'] != pool_hash:
            raise ValueError('frozen split sample pool digest mismatch')
        intrinsic_groups, group_splits = {}, {}
        for sid, group in groups.items():
            if not isinstance(group, str) or not group or assignments[sid] not in ('train', 'val', 'test'):
                raise ValueError('invalid frozen group or split assignment')
            if intrinsic_groups.setdefault(computed[sid], group) != group:
                raise ValueError('frozen manifest splits a connected group')
            if group_splits.setdefault(group, assignments[sid]) != assignments[sid]:
                raise ValueError('frozen group conflicts across splits')
        for sid in existing_split.get('frozen_test_ids', []):
            if assignments.get(sid) != 'test':
                raise ValueError('frozen test ID conflicts with split assignments')
    provenance = {'pool_sha256': pool_hash,
                  'existing_split_sha256': sha256(canonical(existing_split)) if existing_split is not None else None,
                  'dhashes_sha256': sha256(canonical(dhashes)) if dhashes is not None else None}
    return groups, assignments, manual_groups, dhash_threshold, provenance


def _frozen_ids(frozen_test_ids, groups, existing_split):
    frozen = set(frozen_test_ids)
    if existing_split is not None:
        frozen.update(existing_split.get('frozen_test_ids', []))
    if frozen - set(groups):
        raise ValueError('unknown frozen test sample')
    if existing_split is not None and any(existing_split['assignments'][sid] != 'test' for sid in frozen):
        raise ValueError('frozen test IDs conflict with existing split assignments')
    return sorted(frozen)


def _yolo(review, sample):
    lines = []
    for obj in review['objects']:
        x1, y1, x2, y2 = obj['bbox_xyxy']
        w, h = sample['width'], sample['height']
        values = [(x1+x2)/(2*w), (y1+y2)/(2*h), (x2-x1)/w, (y2-y1)/h]
        if values[2] <= 0 or values[3] <= 0:
            raise ValueError('annotation dimensions underflow YOLO numeric precision')
        lines.append(str(obj['class_id']) + ' ' + ' '.join(format(value, '.17g') for value in values) + '\n')
    return ''.join(lines).encode('utf-8')


def export_yolo_dataset(stores, review_ids, class_map, output, *, assignments=None,
                        frozen_test_ids=(), seed=0, manual_groups=None,
                        dhash_threshold=None, existing_split=None, dry_run=False):
    destination = new_output(output, [s.root for s in stores])
    _class_map(class_map)
    samples, owners = _collect(stores)
    manual_groups, dhash_threshold = _grouping_config(manual_groups, dhash_threshold, existing_split)
    dhashes = _pool_dhashes(samples, owners, dhash_threshold)
    groups, existing_assignments, manual_groups, dhash_threshold, grouping_provenance = _group_pool(
        samples, manual_groups, dhashes, dhash_threshold, existing_split)
    frozen_test_ids = _frozen_ids(frozen_test_ids, groups, existing_split)
    splits = split_groups(groups, assignments, frozen_test_ids, seed, existing_assignments)
    reviews = {}
    for store in stores:
        for rid, envelope in _reviews(store, envelopes=True).items():
            if rid in reviews:
                raise ValueError('ambiguous review ID across runs')
            reviews[rid] = (store, envelope)
    if not review_ids or len(review_ids) != len(set(review_ids)):
        raise ValueError('explicit unique review IDs are required')
    selected, selected_reviews, selected_ids, hashes = [], [], set(), {}
    blobs, image_sources = {}, {}
    selected_envelopes = {}
    for rid in sorted(review_ids):
        if rid not in reviews:
            raise ValueError('unknown pinned review: ' + str(rid))
        store, envelope = reviews[rid]
        review = _validate_review_envelope(envelope, require_binding=True)
        if canonical(envelope['class_map']) != canonical(class_map):
            raise ValueError('selected review class map binding differs from export class map')
        sample = store.get(review['sample_key'])
        _validate_review(review, sample, class_map)
        if review['status'] != 'approved':
            raise ValueError('only explicitly approved reviews may be exported')
        sid = sample['sample_id']
        if sid in selected_ids:
            raise ValueError('duplicate sample selection')
        selected_ids.add(sid)
        label = _yolo(review, sample)
        if sample['image_sha256'] in hashes:
            raise ValueError('duplicate image bytes in selection; select one reviewed observation')
        hashes[sample['image_sha256']] = sid
        split = splits[sid]
        image_path, label_path = f'images/{split}/{sid}.jpg', f'labels/{split}/{sid}.txt'
        image_sources[image_path] = (safe_source(store.root, sample['image_path']), sample['image_sha256'])
        blobs[label_path] = label
        selected.append({'sample_id': sid, 'sample_key': sample['key'], 'run_id': sample['key'][0],
                         'metadata_revision': sample['revision'], 'metadata_sha256': sample['sidecar_sha256'],
                         'review_id': rid, 'review_sha256': sha256(canonical(review)),
                         'review_envelope_sha256': envelope['sha256'],
                         'image_path': image_path, 'image_sha256': sample['image_sha256'],
                         'label_path': label_path, 'label_sha256': sha256(label),
                         'group': groups[sid], 'split': split, 'reasons': sample['reasons'],
                         'grouping_metadata': {k: sample['metadata'].get(k) for k in
                             ('events', 'scene_id', 'target_segment_id', 'near_duplicate_group')}})
        selected_reviews.append(review)
        selected_envelopes[rid] = envelope
    if not any(s['split'] == 'train' for s in selected):
        raise ValueError('selected reviews contain zero training images')
    split_manifest = {'schema_version': 1, 'seed': seed, 'groups': groups, 'assignments': splits,
                      'frozen_test_ids': sorted(frozen_test_ids), 'manual_groups': manual_groups or {},
                      'pool_sha256': grouping_provenance['pool_sha256'], 'grouping_provenance': grouping_provenance,
                      'method': 'connected-components: run/event/scene/target/exact-hash/manual/dhash',
                      'dhash_threshold': dhash_threshold, 'near_duplicate_review':
                      'dhash-enabled' if dhash_threshold is not None else 'manual/run grouping only; visual near-duplicate review required'}
    blobs['split_manifest.json'] = canonical(split_manifest) + b'\n'
    blobs['review_manifest.json'] = canonical({'reviews': selected_reviews, 'class_map': class_map,
                                             'review_envelopes': selected_envelopes}) + b'\n'
    yaml = 'path: .\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n'
    for cid, name in sorted(class_map['names'].items(), key=lambda x: int(x[0])):
        yaml += f'  {cid}: {json.dumps(name, ensure_ascii=False)}\n'
    blobs['data.yaml'] = yaml.encode('utf-8')
    manifest = {'schema_version': 1, 'tool_version': TOOL_VERSION, 'frozen': True,
                'class_map': class_map, 'seed': seed, 'samples': selected,
                'run_provenance': {str(s.run_info['run_id']): s.run_info for s in stores},
                'pool_sha256': sha256(canonical([{'key': s['key'], 'sha256': s['image_sha256'],
                           'revision': s['revision']} for s in samples])),
                'file_hashes': dict(sorted({**{path: sha256(data) for path, data in blobs.items()},
                                           **{path: source[1] for path, source in image_sources.items()}}.items()))}
    blobs['dataset_manifest.json'] = canonical(manifest) + b'\n'
    blobs['SHA256SUMS.json'] = canonical({**manifest['file_hashes'],
                      'dataset_manifest.json': sha256(blobs['dataset_manifest.json'])}) + b'\n'
    if not dry_run:
        destination.mkdir()
        for split in ('train', 'val', 'test'):
            for kind in ('images', 'labels'):
                (destination / kind / split).mkdir(parents=True)
        # Copy one image at a time: a full run can contain many GiB of originals.
        # Manifests are written last; an interrupted output is never a frozen dataset.
        for path, (source, digest) in image_sources.items():
            copied = hashlib.sha256()
            with source.open('rb') as reader, (destination / path).open('xb') as writer:
                while True:
                    chunk = reader.read(1024 * 1024)
                    if not chunk:
                        break
                    copied.update(chunk)
                    writer.write(chunk)
                writer.flush()
                os.fsync(writer.fileno())
            if copied.hexdigest() != digest:
                raise ValueError('source changed during export; output is incomplete')
        for path, data in blobs.items():
            with (destination / path).open('xb') as file:
                file.write(data)
                file.flush()
                os.fsync(file.fileno())
    return manifest


def compare_selections(samples, *, budget, seeds, d0_hashes=(), frozen_test_ids=(),
                       manual_groups=None, dhashes=None, dhash_threshold=None, existing_split=None):
    if type(budget) is not int or budget <= 0 or len(set(seeds)) < 3:
        raise ValueError('positive budget and at least three distinct fixed seeds required')
    groups, assignments, manual_groups, dhash_threshold, grouping_provenance = _group_pool(
        samples, manual_groups, dhashes, dhash_threshold, existing_split)
    frozen_test_ids = _frozen_ids(frozen_test_ids, groups, existing_split)
    excluded_groups = {groups[sid] for sid in frozen_test_ids}
    excluded_groups.update(groups[sid] for sid, split in assignments.items() if split == 'test')
    excluded_groups.update(groups[s['sample_id']] for s in samples if s['image_sha256'] in set(d0_hashes))
    unique = {}
    for sample in sorted(samples, key=lambda s: s['sample_id']):
        if groups[sample['sample_id']] not in excluded_groups:
            if sample['image_sha256'] not in unique:
                unique[sample['image_sha256']] = dict(sample, reasons=list(sample['reasons']))
            else:
                prior = unique[sample['image_sha256']]
                prior['reasons'] = sorted(set(prior['reasons']) | set(sample['reasons']))
    candidates = sorted(unique.values(), key=lambda s: s['sample_id'])
    pools = {'A_uniform_random': [s['sample_id'] for s in candidates],
             'B_low_confidence': [s['sample_id'] for s in candidates if 'low_confidence' in s['reasons']],
             'C_multirule': [s['sample_id'] for s in candidates if set(s['reasons']) &
                            {'low_confidence', 'small_target', 'consecutive_empty', 'disappearance'}]}
    effective = min([budget] + [len(pool) for pool in pools.values()])
    if effective == 0:
        raise ValueError('comparison unavailable: at least one policy has no eligible candidates')
    selections = {}
    for seed in sorted(set(seeds)):
        selections[str(seed)] = {}
        for name, pool in pools.items():
            choices = list(pool)
            random.Random(seed).shuffle(choices)
            selections[str(seed)][name] = choices[:effective]
    return {'tool_version': TOOL_VERSION, 'requested_budget': budget, 'effective_budget': effective,
            'pool_sha256': sha256(canonical(candidates)), 'policy_available': {k: len(v) for k, v in pools.items()},
            'd0_hashes': sorted(set(d0_hashes)), 'frozen_test_ids': sorted(frozen_test_ids),
            'excluded_groups': sorted(excluded_groups), 'groups': groups,
            'manual_groups': manual_groups, 'dhash_threshold': dhash_threshold,
            'split_assignments': assignments, 'grouping_provenance': grouping_provenance,
            'selections': selections, 'annotation_standard': 'same pinned class map and human approval policy required',
            'review_hours': None, 'label_object_count': None,
            'note': 'Selection comparison only; no measured accuracy improvement.'}
