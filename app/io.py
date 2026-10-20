from __future__ import annotations
import json, os, time, uuid
from pathlib import Path

def now_ms(): return time.monotonic_ns() // 1_000_000

def atomic_json(file: Path, value):
    file.parent.mkdir(parents=True, exist_ok=True)
    temporary = file.with_name(file.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            os.chmod(temporary, 0o600)
            json.dump(value, stream, ensure_ascii=False, allow_nan=False)
        temporary.replace(file)
    finally: temporary.unlink(missing_ok=True)

def read_json(file):
    return json.loads(Path(file).read_text(encoding='utf-8'))

def event_log(state, event, **fields):
    with (state/'logs/events.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps({'time_ms': now_ms(), 'event': event, **fields}, ensure_ascii=False, allow_nan=False)+'\n')
