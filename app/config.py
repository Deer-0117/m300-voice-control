from __future__ import annotations
import os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def path(value):
    p = Path(value).expanduser()
    return (p if p.is_absolute() else ROOT / p).resolve()

def load_env():
    values = {}
    file = ROOT / '.env'
    if file.exists():
        for line in file.read_text().splitlines():
            if not line.strip() or line.lstrip().startswith('#'): continue
            key, sep, value = line.partition('=')
            if not sep: raise ValueError('invalid .env line')
            values[key.strip()] = value.strip().strip('\"\'')
    values.update({k: v for k, v in os.environ.items() if k.startswith(('OPENAI_', 'UAV_'))})
    return values

class Config:
    def __init__(self, values=None):
        self.values = load_env() if values is None else values
        v = self.values
        self.state = path(v.get('UAV_STATE_DIR', 'state'))
        self.key = v.get('OPENAI_API_KEY', '').strip()
        self.base_url = v.get('OPENAI_BASE_URL', 'https://api.deepseek.com').rstrip('/')
        self.model = v.get('OPENAI_CHAT_MODEL_ID', 'deepseek-flash')
        self.timeout = float(v.get('UAV_API_TIMEOUT_S', '30'))
        self.flight_enabled = v.get('UAV_ENABLE_FLIGHT', '0') == '1'
        self.whisper = path(v.get('UAV_WHISPER_CLI', 'third_party/whisper.cpp/build/bin/whisper-cli'))
        self.whisper_model = path(v.get('UAV_WHISPER_MODEL', 'third_party/whisper.cpp/models/ggml-base.bin'))
        self.metadata = path(v.get('UAV_LIVEVIEW_METADATA', 'inputs/liveview_latest.tsv'))
        self.frame_dir = path(v.get('UAV_LIVEVIEW_FRAME_DIR', 'inputs/liveview_frames'))
        self.max_visual_calls = int(v.get('UAV_MAX_VISUAL_DECISIONS','100'))
        if not 1<=self.max_visual_calls<=1000:raise ValueError('invalid visual budget')
        self.frame_max_age_ms = 500
        self.telemetry_max_age_ms = 250
        self.max_horizontal = .5
        self.max_vertical = .2
        self.max_yaw = 5.
        self.max_duration = .5
        self.min_altitude = 2.
        self.max_altitude = 10.
        self.camera_note = v.get('UAV_CAMERA_DESCRIPTION', 'Forward-facing fixed focal length camera; confirm mounting and level gimbal before flight.')
        if not self.base_url.startswith('https://'): raise ValueError('API endpoint must use HTTPS')

    def initialize(self):
        for d in ('ipc/events', 'ipc/audio', 'logs', 'api-transcript', 'asr'):
            (self.state / d).mkdir(parents=True, exist_ok=True)
        os.chmod(self.state, 0o700)
