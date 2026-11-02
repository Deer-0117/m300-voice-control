"""Bounded v1 wire validation; deliberately independent of GPU/image libraries."""
from __future__ import annotations

import json
import math
from pathlib import Path
import re
from functools import lru_cache

MAX_WIRE_BYTES = 61440
MAX_METADATA_BYTES = 1048576
MAX_IMAGE_BYTES = 8 * 1024 * 1024
MAX_BOXES = 4096
MAX_DEPTH = 16
HEX_ID = re.compile(r'[0-9a-f]{32}\Z')
HASH = re.compile(r'[0-9a-f]{64}\Z')
BOOT = re.compile(r'[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}\Z')
SESSION_FRAME = re.compile(r'frame_(\d{12})_([0-9a-f]{32})\.(?:jpg|bmp)\Z')
LEGACY_FRAME = re.compile(r'frame_(\d+)\.(?:jpg|bmp)\Z')


def _require(ok, reason):
    if not ok:
        raise ValueError(reason)


def _integer(value, maximum=(1 << 63) - 1):
    return type(value) is int and 0 <= value <= maximum


def _number(value, low=0, high=float('inf')):
    try:
        return type(value) in (float, int) and math.isfinite(value) and low <= value <= high
    except OverflowError:
        return False


def _tree(value, depth=0):
    _require(depth <= MAX_DEPTH, 'JSON nesting exceeds 16')
    if isinstance(value, dict):
        _require(all(isinstance(k, str) for k in value), 'non-string JSON key')
        for item in value.values(): _tree(item, depth + 1)
    elif isinstance(value, list):
        for item in value: _tree(item, depth + 1)
    elif isinstance(value, float):
        _require(math.isfinite(value), 'nonfinite JSON number')
    else:
        _require(value is None or type(value) in (str, int, bool), 'not JSON data')


def _pairs(items):
    value = {}
    for key, item in items:
        _require(key not in value, 'duplicate JSON key')
        value[key] = item
    return value


def loads(raw, max_bytes=MAX_WIRE_BYTES):
    """Decode strict JSON. Schema validation is explicit, including control files."""
    try:
        if isinstance(raw, str): raw = raw.encode('utf-8')
        _require(isinstance(raw, bytes) and 0 < len(raw) <= max_bytes, 'JSON byte limit')
        value = json.loads(raw.decode('utf-8'), object_pairs_hook=_pairs,
                           parse_constant=lambda s: (_ for _ in ()).throw(ValueError('nonfinite JSON number')))
        _tree(value)
        return value
    except (UnicodeError, RecursionError, OverflowError) as exc:
        raise ValueError('invalid JSON') from exc


def dumps(value, max_bytes=MAX_WIRE_BYTES):
    _tree(value)
    raw = json.dumps(value, allow_nan=False, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    _require(len(raw) <= max_bytes, 'JSON byte limit')
    return raw


def _fields(value, required, optional=()):
    _require(isinstance(value, dict), 'object required')
    _require(set(required) <= value.keys(), 'missing required field')
    _require(value.keys() <= set(required) | set(optional), 'unknown field')


def _text(value, limit=511):
    return isinstance(value, str) and len(value) <= limit and '\x00' not in value


def validate_header(message):
    required = 'protocol_version kind run_id boot_id generation decoder_session_id frame_id producer_mono_ns metadata_mode'.split()
    _fields(message, required, 'worker_session_id request_seq inference_seq metadata_bytes payload'.split())
    _tree(message)
    _require(type(message['protocol_version']) is int and message['protocol_version'] == 1, 'protocol version')
    _require(message['kind'] in ('FrameReady', 'DetectionResult'), 'message kind')
    for key in ('run_id','decoder_session_id'):
        _require(isinstance(message[key], str) and HEX_ID.fullmatch(message[key]), key)
    _require(isinstance(message['boot_id'],str) and BOOT.fullmatch(message['boot_id']), 'boot_id')
    for key in ('generation','producer_mono_ns'):
        _require(_integer(message[key]), key)
    _require(_integer(message['frame_id'], 999999999999), 'frame_id')
    if message['kind'] == 'FrameReady':
        _require(not set(('worker_session_id','request_seq','inference_seq')) & message.keys(), 'frame has detection identity')
        _require(message['metadata_mode'] == 'inline', 'frame metadata must be inline')
    else:
        _require(isinstance(message.get('worker_session_id'), str) and HEX_ID.fullmatch(message['worker_session_id']), 'worker_session_id')
        for key in ('request_seq','inference_seq'):
            _require(_integer(message.get(key)), key)
    if message['metadata_mode'] == 'inline':
        _require(isinstance(message.get('payload'), dict) and 'metadata_bytes' not in message, 'inline metadata')
    elif message['metadata_mode'] == 'fd':
        _require('payload' not in message and _integer(message.get('metadata_bytes'),MAX_METADATA_BYTES)
                 and message['metadata_bytes'] > 0, 'fd metadata')
    else:
        raise ValueError('metadata_mode')
    return message


def _box(box, width, height):
    _fields(box, 'class_id class_name confidence bbox_xyxy origin stage'.split(),
            'valid invalid_reason tile_roi tile_to_full source_index'.split())
    valid = box.get('valid',True)
    _require(type(valid) is bool, 'box valid')
    _require(_integer(box['class_id']) or (not valid and box['class_id'] is None), 'box class id')
    _require(_text(box['class_name'],128) or (not valid and box['class_name'] is None), 'box class name')
    _require(_number(box['confidence'],0,1) or (not valid and box['confidence'] is None), 'box confidence')
    _require(box['origin'] in ('full','tile','tracker') and _text(box['stage'],64), 'box origin/stage')
    coords = box['bbox_xyxy']
    _require((isinstance(coords,list) and len(coords)==4 and all(_number(v,-1e9,1e9) for v in coords))
             or (not valid and coords is None), 'box coordinates')
    if valid:
        x1,y1,x2,y2=coords
        _require(width is not None and height is not None and 0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height,'box out of bounds')
        _require('invalid_reason' not in box, 'valid box has error')
    else:
        _require(_text(box.get('invalid_reason')) and bool(box['invalid_reason']), 'invalid box reason')
    if 'source_index' in box: _require(_integer(box['source_index'],MAX_BOXES), 'source index')
    for key in ('tile_roi','tile_to_full'):
        if key in box:
            values=box[key]
            _require(isinstance(values,list) and len(values)==(4 if key=='tile_roi' else 6)
                     and all(_number(v,-1e9,1e9) for v in values),key)


def validate_payload(header, payload):
    _tree(payload)
    if header['kind'] == 'FrameReady':
        _fields(payload,'width height source_mono_ns wall_utc_ms frame_ts_ms encode_ms codec original_path'.split())
        _require(_integer(payload['width'],3840) and payload['width']>0, 'width')
        _require(_integer(payload['height'],2160) and payload['height']>0,'height')
        for key in ('source_mono_ns','wall_utc_ms','frame_ts_ms'): _require(_integer(payload[key]),key)
        _require(payload['source_mono_ns'] <= header['producer_mono_ns'],'source time after producer')
        _require(_number(payload['encode_ms']) and payload['codec']=='jpeg' and _text(payload['original_path']),'frame payload')
        return payload
    required='ok error image_width image_height source_path pin_used request_start_ns inference_start_ns inference_end_ns backend model_sha256 config_sha256 online_conf online_iou class_map detections full_detections tile_detections selected_target detection_stage timings capture_ts_ms request_ts_ms raw_result'.split()
    _fields(payload,required, ('unavailable_reasons','model_info','passes'))
    _require(type(payload['ok']) is bool and payload['pin_used'] is True,'result state/pin')
    if payload['ok']:
        _require(payload['error'] is None,'success has error')
    else:
        _fields(payload['error'],('type','message'))
        _require(_text(payload['error']['type'],128) and _text(payload['error']['message'],4096),'result error')
    for key,maximum in (('image_width',3840),('image_height',2160)):
        _require((payload[key] is None and not payload['ok']) or (_integer(payload[key],maximum) and payload[key]>0),key)
    for key in ('request_start_ns','inference_end_ns'): _require(_integer(payload[key]),key)
    start=payload['inference_start_ns']
    _require((start is None and not payload['ok']) or _integer(start),'inference_start_ns')
    _require(payload['request_start_ns'] <= (start if start is not None else payload['inference_end_ns']) <= payload['inference_end_ns'] <= header['producer_mono_ns'],'result time order')
    for key in ('capture_ts_ms','request_ts_ms'):
        _require(payload[key] is None or _integer(payload[key]),key)
    _require(_text(payload['source_path']) and _text(payload['backend'],128) and _text(payload['detection_stage'],64),'result text')
    reasons=payload.get('unavailable_reasons',{})
    _require(isinstance(reasons,dict) and all(_text(k,128) and _text(v,1024) for k,v in reasons.items()),'unavailable reasons')
    for key in ('model_sha256','config_sha256'):
        _require((payload[key] is None and bool(reasons.get(key))) or (isinstance(payload[key],str) and HASH.fullmatch(payload[key])), key)
    for key in ('online_conf','online_iou'):
        _require((payload[key] is None and bool(reasons.get(key))) or _number(payload[key],0,1),key)
    _require(isinstance(payload['class_map'],dict) and len(payload['class_map'])<=4096
             and all(k.isdecimal() and _text(v,128) for k,v in payload['class_map'].items()),'class map')
    _require(isinstance(payload['timings'],dict) and all(_text(k,128) and (v is None or _number(v)) for k,v in payload['timings'].items()),'timings')
    _require(isinstance(payload['raw_result'],dict),'raw result')
    count=0
    for key in ('detections','full_detections','tile_detections'):
        _require(isinstance(payload[key],list),key)
        count+=len(payload[key]); _require(count<=MAX_BOXES,'box count')
        for box in payload[key]: _box(box,payload['image_width'],payload['image_height'])
    if payload['selected_target'] is not None: _box(payload['selected_target'],payload['image_width'],payload['image_height'])
    return payload


def frame_key(message):
    return tuple(message[k] for k in ('run_id','boot_id','decoder_session_id','frame_id'))


def detection_key(message):
    return tuple(message[k] for k in ('run_id','boot_id','worker_session_id','inference_seq'))


def parse_frame_filename(path):
    name=Path(path).name
    match=SESSION_FRAME.fullmatch(name)
    if match: return match.group(2),int(match.group(1))
    match=LEGACY_FRAME.fullmatch(name)
    return ('unknown',int(match.group(1))) if match else None


@lru_cache(maxsize=1)
def boot_id():
    """Linux boot identity; no invented fallback identity on other platforms."""
    value=Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip()
    _require(BOOT.fullmatch(value),'invalid OS boot identity')
    return value
